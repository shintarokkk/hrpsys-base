#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "LimbTorqueController.h"
#include "../ImpedanceController/JointPathEx.h"
#include <hrpModel/JointPath.h>
#include <hrpUtil/Eigen3d.h>

#include "../RobotHardware/RobotHardwareService_impl.h"

typedef coil::Guard<coil::Mutex> Guard;

#ifndef deg2rad
#define deg2rad(x) ((x) * M_PI / 180.0)
#endif
#ifndef rad2deg
#define rad2deg(rad) (rad * 180 / M_PI)
#endif

// Module specification
// <rtc-template block="module_spec">
static const char* limbtorquecontroller_spec[] =
    {
        "implementation_id", "LimbTorqueController",
        "type_name",         "LimbTorqueController",
        "description",       "limb torque controller component",
        "version",           HRPSYS_PACKAGE_VERSION,
        "vendor",            "AIST",
        "category",          "example",
        "activity_type",     "DataFlowComponent",
        "max_instance",      "10",
        "language",          "C++",
        "lang_type",         "compile",
        // Configuration variables
        "conf.default.debugLevel", "0",
        "conf.default.pgain", "1000",
        "conf.default.dgain", "1000",
        ""
    };
// </rtc-template>

LimbTorqueController::LimbTorqueController(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      m_qCurrentIn("qCurrent", m_qCurrent), //センサ値:重力補償用
      m_dqCurrentIn("dqCurrent", m_dqCurrent),
      m_qRefIn("qRef", m_qRef),
      m_tqRefIn("tqRef", m_tqRef),
      m_basePosIn("basePosIn", m_basePos), //eus等からのref値
      m_baseRpyIn("baseRpyIn", m_baseRpy), //eus等からのref値
      m_rpyIn("rpy", m_rpy), //IMU情報
      m_qOut("q", m_q), //qRefそのまま出すのでよい
      m_tqOut("tq", m_tq), //torque out
      m_LimbTorqueControllerServicePort("LimbTorqueControllerService"),
      m_robot(hrp::BodyPtr()),
      m_robotRef(hrp::BodyPtr()),
      m_debugLevel(0),
      dummy(0)
{
    m_service0.limbtorque(this);
}

LimbTorqueController::~LimbTorqueController()
{
}

RTC::ReturnCode_t LimbTorqueController::onInitialize()
{
    std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
    bindParameter("debugLevel", m_debugLevel, "0");

    addInPort("qCurrent", m_qCurrentIn);
    addInPort("dqCurrent", m_dqCurrentIn);
    addInPort("qRef", m_qRefIn);
    addInPort("tqRef", m_tqRefIn);
    addInPort("basePosIn", m_basePosIn);
    addInPort("baseRpyIn", m_baseRpyIn);
    addInPort("rpy", m_rpyIn);

    // Set OutPort buffer
    addOutPort("q", m_qOut);
    addOutPort("tq", m_tqOut); //torque out

    // Set service provider to Ports
    m_LimbTorqueControllerServicePort.registerProvider("service0", "LimbTorqueControllerService", m_service0);

    // Set CORBA Service Ports
    addPort(m_LimbTorqueControllerServicePort);

    RTC::Properties& prop = getProperties();
    coil::stringTo(m_dt, prop["dt"].c_str());

    m_robot = hrp::BodyPtr(new hrp::Body());
    m_robotRef = hrp::BodyPtr(new hrp::Body());

    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
    std::cout << "Model file name = " << prop["model"].c_str() << std::endl;
    if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(),
                                 CosNaming::NamingContext::_duplicate(naming.getRootContext())
                                 )){
      std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
      return RTC::RTC_ERROR;
    }
    loadBodyFromModelLoader(m_robotRef, prop["model"].c_str(),
                            CosNaming::NamingContext::_duplicate(naming.getRootContext()));

    // Setting for wrench data ports (real + virtual)
    std::vector<std::string> fsensor_names;
    //   find names for real force sensors
    unsigned int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
    for ( unsigned int i=0; i<npforce; ++i ){
        fsensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
    }
    // load virtual force sensors
    readVirtualForceSensorParamFromProperties(m_vfs, m_robot, prop["virtual_force_sensor"], std::string(m_profile.instance_name));
    unsigned int nvforce = m_vfs.size();
    for (unsigned int i=0; i<nvforce; i++){
        for ( std::map<std::string, hrp::VirtualForceSensorParam>::iterator it = m_vfs.begin(); it != m_vfs.end(); ++it ) {
            if (it->second.id == (int)i) fsensor_names.push_back(it->first);
        }
    }
    //   add ports for all force sensors
    unsigned int nforce  = npforce + nvforce;
    m_force.resize(nforce);
    m_forceIn.resize(nforce);
    // m_ref_force.resize(nforce);
    // m_ref_forceIn.resize(nforce);
    std::cerr << "[" << m_profile.instance_name << "] force sensor ports" << std::endl;
    for (unsigned int i=0; i<nforce; i++){
        // actual inport
        m_forceIn[i] = new RTC::InPort<RTC::TimedDoubleSeq>(fsensor_names[i].c_str(), m_force[i]);
        m_force[i].data.length(6);
        registerInPort(fsensor_names[i].c_str(), *m_forceIn[i]);
        // ref inport
        //m_ref_force[i].data.length(6);
        // for (unsigned int j=0; j<6; j++) m_ref_force[i].data[j] = 0.0;
        // m_ref_forceIn[i] = new RTC::InPort<RTC::TimedDoubleSeq>(std::string("ref_"+fsensor_names[i]+"In").c_str(), m_ref_force[i]);
        // registerInPort(std::string("ref_"+fsensor_names[i]+"In").c_str(), *m_ref_forceIn[i]);
        std::cerr << "[" << m_profile.instance_name << "]   name = " << fsensor_names[i] << std::endl;
    }

    for (unsigned int i=0; i<m_forceIn.size(); i++){
      abs_forces.insert(std::pair<std::string, hrp::Vector3>(m_forceIn[i]->name(), hrp::Vector3::Zero()));
      abs_moments.insert(std::pair<std::string, hrp::Vector3>(m_forceIn[i]->name(), hrp::Vector3::Zero()));
    }
    unsigned int dof = m_robot->numJoints();
    for ( unsigned int i = 0 ; i < dof; i++ ){
      if ( (int)i != m_robot->joint(i)->jointId ) {
        std::cerr << "[" << m_profile.instance_name << "] jointId is not equal to the index number" << std::endl;
        return RTC::RTC_ERROR;
      }
    }
    torque_output_type = CALC_TORQUE;

    // setting from conf file
    coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
    coil::vstring ltc_pgain_str = coil::split(prop["ltc_pgain"], ",");
    coil::vstring ltc_dgain_str = coil::split(prop["ltc_dgain"], ",");
    std::map<std::string, std::string> base_name_map;
    if (end_effectors_str.size() > 0) {
        size_t prop_num = 10; //the number of parameters of each end effector in conf file: "ee-name, ee-link-name, base-link-name, pos.x, pos.y, pos.z, rot-axis,x, rot-axis.y, rot-axis.z, rot-angle"
        size_t num = end_effectors_str.size()/prop_num;
        default_pgain.resize(num);
        default_dgain.resize(num);
        for (size_t i = 0; i < num; i++) {
            std::string ee_name, ee_target, ee_base;
            double conf_pgain, conf_dgain;
            coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
            coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
            coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());
            coil::stringTo(conf_pgain, ltc_pgain_str[i].c_str());
            coil::stringTo(conf_dgain, ltc_dgain_str[i].c_str());
            ee_trans eet;
            for (size_t j = 0; j < 3; j++) {
                coil::stringTo(eet.localPos(j), end_effectors_str[i*prop_num+3+j].c_str());
            }
            double tmpv[4];  ///tmpvとは?
            for (int j = 0; j < 4; j++ ) {
                coil::stringTo(tmpv[j], end_effectors_str[i*prop_num+6+j].c_str());
            }
            eet.localR = Eigen::AngleAxis<double>(tmpv[3], hrp::Vector3(tmpv[0], tmpv[1], tmpv[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
            eet.target_name = ee_target;
            eet.pgain = conf_pgain;
            eet.dgain = conf_dgain;
            default_pgain[i] = conf_pgain;
            default_dgain[i] = conf_dgain;
            ee_map.insert(std::pair<std::string, ee_trans>(ee_name , eet));
            base_name_map.insert(std::pair<std::string, std::string>(ee_name, ee_base));
            std::cerr << "[" << m_profile.instance_name << "] End Effector [" << ee_name << "]" << ee_target << " " << ee_base << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   target = " << ee_target << ", base = " << ee_base << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   localPos = " << eet.localPos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   localR = " << eet.localR.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
        }
    }

    // initialize limb torque params
    std::cerr << "[" << m_profile.instance_name << "] Add limb torque params" << std::endl;
    for ( unsigned int i=0; i<m_forceIn.size(); ++i ){
        std::string sensor_name = m_forceIn[i]->name();
        hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_name);
        std::string sensor_link_name;
        if ( sensor ) {
            // real force sensor
            sensor_link_name = sensor->link->name;
        } else if ( m_vfs.find(sensor_name) !=  m_vfs.end()) {
            // virtual force sensor
            sensor_link_name = m_vfs[sensor_name].link->name;
        } else {
            std::cerr << "[" << m_profile.instance_name << "]   unknown force param" << std::endl;
            continue;
        }
        // 1. Check whether adequate ee_map exists for the sensor.
        std::string ee_name;
        double eet_pgain, eet_dgain;
        bool is_ee_exists = false;
        for ( std::map<std::string, ee_trans>::iterator it = ee_map.begin(); it != ee_map.end(); ++it ) {
            hrp::Link* alink = m_robot->link(it->second.target_name);
            std::string tmp_base_name = base_name_map[it->first];
            while (alink != NULL && alink->name != tmp_base_name && !is_ee_exists) {
                if ( alink->name == sensor_link_name ) {
                    is_ee_exists = true;
                    ee_name = it->first;
                    eet_pgain = it->second.pgain;
                    eet_dgain = it->second.dgain;
                }
                alink = alink->parent;
            }
        }
        if (!is_ee_exists) {
            std::cerr << "[" << m_profile.instance_name << "]   No such ee setting for " << sensor_name << " and " << sensor_link_name << "!!. Limb torque param for " << sensor_name << " cannot be added!!" << std::endl;
            continue;
        }
        // 2. Check whether already limb torque param exists, which has the same target link as the sensor.
        if (m_lt_param.find(ee_name) != m_lt_param.end()) {
            std::cerr << "[" << m_profile.instance_name << "]   Already limb torque param (target_name=" << sensor_link_name << ", ee_name=" << ee_name << ") exists!!. Limb torque param for " << sensor_name << " cannot be added!!" << std::endl;
            continue;
        }
        // 3. Check whether joint path is adequate.
        hrp::Link* target_link = m_robot->link(ee_map[ee_name].target_name);
        LTParam p;
        p.manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(base_name_map[ee_name]), target_link, m_dt, false, std::string(m_profile.instance_name)));
        if ( ! p.manip ) {
            std::cerr << "[" << m_profile.instance_name << "]   Invalid joint path from " << base_name_map[ee_name] << " to " << target_link->name << "!! Limb torque param for " << sensor_name << " cannot be added!!" << std::endl;
            continue;
        }
        // p.basic_jacobians.resize(p.manip->numJoints());
        // p.inertia_matrices.resize(p.manip->numJoints());
        // p.gen_inertia_matrix = Eigen::MatrixXd::Zero(p.manip->numJoints(), p.manip->numJoints());
        //Set size of matrices used in inertial compensation
        // for (int i=0; i<p.manip->numJoints(); ++i){
        //     p.basic_jacobians[i] = Eigen::MatrixXd::Zero(6, p.manip->numJoints());
        //     p.inertia_matrices[i] = Eigen::MatrixXd::Zero(6, 6);
        //     p.inertia_matrices[i] <<
        //         p.manip->joint(i)->m, 0.0, 0.0, 0.0, 0.0, 0.0,
        //         0.0, p.manip->joint(i)->m, 0.0, 0.0, 0.0, 0.0,
        //         0.0, 0.0, p.manip->joint(i)->m, 0.0, 0.0, 0.0,
        //         0.0, 0.0, 0.0, p.manip->joint(i)->I.row(0),
        //         0.0, 0.0, 0.0, p.manip->joint(i)->I.row(1),
        //         0.0, 0.0, 0.0, p.manip->joint(i)->I.row(2);
        // }
        // 4. Set limb torque param
        p.sensor_name = sensor_name;
        p.pgain = eet_pgain;
        p.dgain = eet_dgain;
        p.gravitational_acceleration = hrp::Vector3(0.0, 0.0, 9.80665); //TODO: set from outside
        m_lt_param[ee_name] = p;
        std::cerr << "[" << m_profile.instance_name << "]   sensor = " << sensor_name << ", sensor-link = " << sensor_link_name << ", ee_name = " << ee_name << ", ee-link = " << target_link->name << std::endl;
    }

    std::vector<std::pair<hrp::Link*, hrp::Link*> > interlocking_joints;
    readInterlockingJointsParamFromProperties(interlocking_joints, m_robot, prop["interlocking_joints"], std::string(m_profile.instance_name));
    if (interlocking_joints.size() > 0) {
        for ( std::map<std::string, LTParam>::iterator it = m_lt_param.begin(); it != m_lt_param.end(); ++it ) {
            std::cerr << "[" << m_profile.instance_name << "] Interlocking Joints for [" << it->first << "]" << std::endl;
            it->second.manip->setInterlockingJointPairIndices(interlocking_joints, std::string(m_profile.instance_name));
        }
    }

    // allocate memory for outPorts
    m_q.data.length(dof);
    m_tq.data.length(dof);
    qold.resize(dof);
    dqold.resize(dof);
    qoldRef.resize(dof);
    dqoldRef.resize(dof);
    for (int i=0; i<dof; ++i){
        qoldRef[i] = 0.0;
        dqoldRef[i] = 0.0;
    }
    loop = 0;

    return RTC::RTC_OK;
}

RTC::ReturnCode_t LimbTorqueController::onFinalize()
{
    return RTC::RTC_OK;
}

RTC::ReturnCode_t LimbTorqueController::onActivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;

    return RTC::RTC_OK;
}

RTC::ReturnCode_t LimbTorqueController::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;

  //必要ならパラメータ等リセットd  return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t LimbTorqueController::onExecute(RTC::UniqueId ec_id)
{
    if(DEBUGP){
       std::cout << "[" << m_profile.instance_name << "]" << "(" << ec_id << "):" << __func__ << std::endl;
   }
   loop ++;

   //Read Import
   if (m_qCurrentIn.isNew()) {
       m_qCurrentIn.read();
   }
   if (m_dqCurrentIn.isNew()) {
       m_dqCurrentIn.read();
   }
   if (m_qRefIn.isNew()) {
       m_qRefIn.read();
   }
   if (m_tqRefIn.isNew()) {
       m_tqRefIn.read();
   }
   if (m_basePosIn.isNew()) {
       m_basePosIn.read();
   }
   if (m_baseRpyIn.isNew()) {
       m_baseRpyIn.read();
   }
   if (m_rpyIn.isNew()) {
       m_rpyIn.read();
   }

   if ( m_qRef.data.length() ==  m_robot->numJoints() &&
        m_qCurrent.data.length() ==  m_robot->numJoints() &&
        m_dqCurrent.data.length() == m_robot->numJoints()) {
       getTargetParameters(); //上位から来るref値を変数にセット
       getActualParameters(); //センサ値を変数にセット

       //ImpactHandler();

       if (torque_output_type == CALC_TORQUE) {
           //参照トルク計算
           //以下の関数内部で各joint->uにトルクを加算していく
           //calcGravityCompensation(); //重力補償
           //calcInertiaCompensation();
           calcLimbInverseDynamics();
           calcJointDumpingTorque(); //関節角度ダンピング
           calcMinMaxAvoidanceTorque();
           //データの記述?
           //Write Outport
       }
       // else if (torque_output_type == REF_TORQUE) {
       //     addDumpingToRefTorque();
       // }

       //std::cout << "m_tq=[";
       for ( size_t i = 0; i<m_robot->numJoints(); ++i){
           m_q.data[i] = m_robotRef->joint(i)->q; //same as input qRef at this moment
           m_tq.data[i] = m_robot->joint(i)->u;
           // printf("%3.2lf",m_tq.data[i]);
           // if( i==m_robot->numJoints()-1){
           //     printf("]\n");
           // }else{
           //     printf(", ");
           // }
       }

       m_qOut.write();
       m_tqOut.write();
   }

   return RTC::RTC_OK;
}

void LimbTorqueController::getTargetParameters()
{
    for ( size_t i = 0; i < m_robotRef->numJoints(); ++i){
        m_robotRef->joint(i)->q = m_qRef.data[i];
        m_robotRef->joint(i)->dq = loop>1 ? ( m_qRef.data[i] - qoldRef[i] ) / m_dt : 0;
        m_robotRef->joint(i)->ddq = (m_robotRef->joint(i)->dq - dqoldRef[i]) / m_dt;
        //if 外から指令値in set 指令値 as dqRef & ddqRef
        qoldRef[i] = m_qRef.data[i];
        dqoldRef[i] = m_robotRef->joint(i)->dq;
        if (torque_output_type == REF_TORQUE){
            //m_robot->joint(i)->u = m_tqRef.data[i];
            m_robot->joint(i)->u = 0; //temporary
        }
    }
}

void LimbTorqueController::getActualParameters()
{
    //Current robot state
    for ( unsigned int i = 0; i<m_robot->numJoints(); ++i ){
        m_robot->joint(i)->q = m_qCurrent.data[i];
        //m_robot->joint(i)->dq = loop>1 ? ( m_qCurrent.data[i] - qold[i] ) / m_dt : 0;
        m_robot->joint(i)->dq = m_dqCurrent.data[i];
        m_robot->joint(i)->ddq = m_robotRef->joint(i)->ddq; //TODO
        if (torque_output_type != REF_TORQUE){
            m_robot->joint(i)->u = 0;
        }
        qold[i] = m_qCurrent.data[i];
    }
    m_robot->rootLink()->p = hrp::Vector3::Zero();
    m_robot->calcForwardKinematics();

#if 0
    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    hrp::Matrix33 senR = sen->link->R * sen->localR;
    hrp::Matrix33 act_Rs(hrp::rotFromRpy(m_rpy.data.r, m_rpy.data.p, m_rpy.data.y));
#else
    hrp::Matrix33 senR = hrp::Matrix33::Identity();
    hrp::Matrix33 act_Rs = hrp::Matrix33::Identity();
#endif

    m_robot->rootLink()->R = act_Rs * (senR.transpose() * m_robot->rootLink()->R);
    m_robot->calcForwardKinematics();
}

void LimbTorqueController::calcLimbInverseDynamics()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        if (param.is_active) {
            if (DEBUGP) {
                std::cerr << "ここにデバッグメッセージを流す" << std::endl;
            }
            hrp::Link* base_link = param.manip->baseLink();
            Eigen::MatrixXd base_rot = base_link->R;
            //TODO: set out f, tau
            hrp::Vector3 out_f, out_tau;
            out_f = Eigen::Vector3d::Zero(3);
            out_tau = Eigen::Vector3d::Zero(3);
            //TODO: acceleration of baseLink due to fullbody movement
            base_link->dvo = base_rot * param.gravitational_acceleration;
            base_link->dw.setZero();
            m_robot->calcInverseDynamics(base_link, out_f, out_tau);
        }
        ++it;
    }
}

void LimbTorqueController::calcGravityCompensation()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        if (param.is_active) {
            if (DEBUGP) {
                std::cerr << "ここにデバッグメッセージを流す" << std::endl;
            }
            hrp::JointPathExPtr manip = param.manip;
            hrp::Vector3 tmp_f, tmp_tau;
            //TODO: add sibling
            hrp::Link* tmp_updating_joint = manip->endLink();
            while (tmp_updating_joint->parent) {
                tmp_updating_joint->wc = tmp_updating_joint->p + tmp_updating_joint->R*tmp_updating_joint->c; //position of CoM of the link
                if (tmp_updating_joint->child) {
                    tmp_updating_joint->subm = tmp_updating_joint->child->subm + tmp_updating_joint->m; //total mass of subtree
                    tmp_updating_joint->submwc = tmp_updating_joint->m*tmp_updating_joint->wc + tmp_updating_joint->child->submwc; //total momentum of subtree around world origin
                }else{
                    tmp_updating_joint->subm = tmp_updating_joint->m;
                    tmp_updating_joint->submwc = tmp_updating_joint->m*tmp_updating_joint->wc;
                }
                hrp::Vector3 world_joint_axis = tmp_updating_joint->R*tmp_updating_joint->a;
                hrp::Vector3 local_moment_arm = tmp_updating_joint->submwc/tmp_updating_joint->subm - tmp_updating_joint->p;
                tmp_updating_joint->u += tmp_updating_joint->subm*(local_moment_arm.cross(param.gravitational_acceleration)).dot(world_joint_axis);
                tmp_updating_joint = tmp_updating_joint->parent;
            }
        }
        ++it;
    } //end while
}

// void LimbTorqueController::calcInertiaCompensation()
// {
//     std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
//     while(it != m_lt_param.end()){
//         LTParam& param = it->second;
//         if (param.is_active) {
//             if (DEBUGP) {
//                 std::cerr << "ここにデバッグメッセージを流す" << std::endl;
//             }
//             hrp::JointPathExPtr manip = param.manip;
//             hrp::Link* tmp_updating_joint = manip->baseLink();
//             param.gen_inertia_matrix = Eigen::MatrixXd::Zero(manip->numJoints(), manip->numJoints()); //reset
//             //calculate basic jacobian
//             Eigen::Vector3d omega, arm, omegaxarm;
//             Eigen::VectorXd joint_acc(manip->numJoints()), joint_tq(manip->numJoints());
//             for (int i=0; i<manip->numJoints(); ++i){
//                 param.basic_jacobians[i] = Eigen::MatrixXd::Zero(6, manip->numJoints()); //reset
//                 joint_acc(i) = m_robotRef->joint(manip->joint(i)->jointId)->ddq;
//                 for (int j=0; j<=i; ++j){
//                     omega = manip->joint(j)->R*manip->joint(j)->a;
//                     arm = manip->joint(i)->wc - manip->joint(j)->p; //wc is updated in calcGravityCompensation
//                     omegaxarm = omega.cross(arm);
//                     param.basic_jacobians[i].col(j) << omegaxarm, omega;
//                 }
//                 param.gen_inertia_matrix += param.basic_jacobians[i].transpose() * param.inertia_matrices[i] * param.basic_jacobians[i];
//             }
//             joint_tq = param.gen_inertia_matrix * joint_acc;
//             for (int i=0; i<manip->numJoints(); ++i){
//                 manip->joint(i)->u += joint_tq(i);
//             }
//         }
//         ++it;
//     }
// }

void LimbTorqueController::calcJointDumpingTorque()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        if (param.is_active) {
            if (DEBUGP) {
                std::cerr << "ここにデバッグメッセージを流す" << std::endl;
            }
            hrp::JointPathExPtr manip = param.manip;
            std::string ee_name = it->first;
            for ( size_t i = 0; i < manip->numJoints(); ++i ) {
                hrp::Link* act_joint = manip->joint(i);
                hrp::Link* ref_joint = m_robotRef->joint(act_joint->jointId);
                double q_error = act_joint->q - ref_joint->q;
                double dq_error = act_joint->dq - ref_joint->dq;
                act_joint->u += -m_lt_param[ee_name].pgain*q_error - m_lt_param[ee_name].dgain*dq_error;
                //std::cout << "calcJointDumpingGain: act_joint(" << manip->joint(i)->name << ")=" << act_joint->dq << std::endl;
            }}
        ++it;
    }
}

void LimbTorqueController::calcMinMaxAvoidanceTorque()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        if (param.is_active) {
            if (DEBUGP) {
                std::cerr << "ここにデバッグメッセージを流す" << std::endl;
            }
            hrp::JointPathExPtr manip = param.manip;
            double margin = deg2rad(4.0);
            for ( size_t i = 0; i < manip->numJoints(); ++i ) {
                double now_angle = manip->joint(i)->q;
                double max_angle = manip->joint(i)->ulimit;
                double min_angle = manip->joint(i)->llimit;
                if ( (now_angle+margin) >= max_angle) {
                    double max_torque = manip->joint(i)->climit * manip->joint(i)->gearRatio * manip->joint(i)->torqueConst;
                    manip->joint(i)->u += - max_torque/margin*(now_angle - (max_angle-margin));
                    if (loop%10 == 0){
                        std::cout << "!!MinMax Angle Warning!!" << "[" << m_profile.instance_name << "] " << manip->joint(i)->name << " is near limit: " << "max=" << rad2deg(max_angle) << ", now=" << rad2deg(now_angle) << ", applying min/max avoidance torque." << std::endl;}
                }else if ( (now_angle-margin) <= min_angle) {
                    double max_torque = manip->joint(i)->climit * manip->joint(i)->gearRatio * manip->joint(i)->torqueConst;
                    manip->joint(i)->u += max_torque/margin*(min_angle+margin - now_angle);
                    if (loop%10 == 0){
                        std::cout << "!!MinMax Angle Warning!!" << "[" << m_profile.instance_name << "] " << manip->joint(i)->name << " is near limit: " << "min=" << rad2deg(min_angle) << ", now=" << rad2deg(now_angle) << ", applying min/max avoidance torque." << std::endl;}
                }
            }
        }
        ++it;
    }
}

//for REF_TORQUE mode
void LimbTorqueController::addDumpingToRefTorque()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        if (param.is_active) {
            if (DEBUGP) {
                std::cerr << "ここにデバッグメッセージを流す" << std::endl;
            }
            hrp::JointPathExPtr manip = param.manip;
            std::string ee_name = it->first;
            for ( size_t i = 0; i < manip->numJoints(); ++i ) {
                hrp::Link* act_joint = manip->joint(i);
                act_joint->u += - m_lt_param[ee_name].dgain * act_joint->dq;
            }}
        ++it;
    }
}

bool LimbTorqueController::startLimbTorqueController(const std::string& i_name_)
{
    {
        Guard guard(m_mutex);
        if ( m_lt_param.find(i_name_) == m_lt_param.end() ) {
            std::cerr << "[" << m_profile.instance_name << "] Could not find limbtorque controller param [" << i_name_ << "]" << std::endl;
            return false;
        }
        if ( m_lt_param[i_name_].is_active ) {
            std::cerr << "[" << m_profile.instance_name << "] Limbtorque control [" << i_name_ << "] is already started" << std::endl;
            return false;
        }
        std::cerr << "[" << m_profile.instance_name << "] Start limbtorque control [" << i_name_ << "]" << std::endl;
        m_lt_param[i_name_].is_active = true;
    }
    return true;
}

bool LimbTorqueController::stopLimbTorqueController(const std::string& i_name_)
{
    {
        Guard guard(m_mutex);
        if ( m_lt_param.find(i_name_) == m_lt_param.end() ) {
            std::cerr << "[" << m_profile.instance_name << "] Could not find limbtorque controller param [" << i_name_ << "]" << std::endl;
            return false;
        }
        if ( !m_lt_param[i_name_].is_active ) {
            std::cerr << "[" << m_profile.instance_name << "] Limbtorque control [" << i_name_ << "] is already stopped" << std::endl;
            return false;
        }
        std::cerr << "[" << m_profile.instance_name << "] Stop limbtorque control [" << i_name_ << "]" << std::endl;
        //TODO: stop limb torque control
        m_lt_param[i_name_].is_active = false; //need transition?
    }
    return true;
}

bool LimbTorqueController::setLimbTorqueControllerParam(const std::string& i_name_, OpenHRP::LimbTorqueControllerService::limbtorqueParam i_param_)
{
    {
        Guard guard(m_mutex);
        std::string name = std::string(i_name_);
        if ( m_lt_param.find(name) == m_lt_param.end() ) {
            std::cerr << "[" << m_profile.instance_name << "] Could not find limb torque controller param [" << name << "]" << std::endl;
            return false;
        }

        std::cerr << "[" << m_profile.instance_name << "] Update limb toruqe parameters" << std::endl;

        m_lt_param[name].pgain = i_param_.Pgain;
        m_lt_param[name].dgain = i_param_.Dgain;

        //TODO: force_gain, moment_gain
        m_lt_param[name].force_gain = hrp::Vector3(i_param_.force_gain[0], i_param_.force_gain[1], i_param_.force_gain[2]).asDiagonal();
        m_lt_param[name].moment_gain = hrp::Vector3(i_param_.moment_gain[0], i_param_.moment_gain[1], i_param_.moment_gain[2]).asDiagonal();

        //TODO: コントローラがACTIVEの時モード変更できないように制限
        switch (i_param_.torque_output_type){
        case OpenHRP::LimbTorqueControllerService::CALC_TORQUE:
            torque_output_type = CALC_TORQUE;
            break;
        case OpenHRP::LimbTorqueControllerService::REF_TORQUE:
            torque_output_type = REF_TORQUE;
            break;
        default: break;
        }

        std::cerr << "[" << m_profile.instance_name << "] set parameters" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]             name : " << name << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]  Pgain, Dgain : " << m_lt_param[name].pgain << " " << m_lt_param[name].dgain << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]       force_gain : " << m_lt_param[name].force_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]      moment_gain : " << m_lt_param[name].moment_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
    }
    return true;
}

void LimbTorqueController::copyLimbTorqueControllerParam(LimbTorqueControllerService::limbtorqueParam& i_param_, const LTParam& param)
{
    i_param_.Pgain = param.pgain;
    i_param_.Dgain = param.dgain;
    for (size_t i = 0; i < 3; i++) i_param_.force_gain[i] = param.force_gain(i,i);
    for (size_t i = 0; i < 3; i++) i_param_.moment_gain[i] = param.moment_gain(i,i);

    if (param.is_active) i_param_.controller_mode = OpenHRP::LimbTorqueControllerService::MODE_ACTIVE;
    else i_param_.controller_mode = OpenHRP::LimbTorqueControllerService::MODE_IDLE;
}


bool LimbTorqueController::getLimbTorqueControllerParam(const std::string& i_name_, LimbTorqueControllerService::limbtorqueParam& i_param_)
{
    if ( m_lt_param.find(i_name_) == m_lt_param.end() ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find limb torque controller param [" << i_name_ << "]" << std::endl;
        copyLimbTorqueControllerParam(i_param_, LTParam());
        return false;
    }
    copyLimbTorqueControllerParam(i_param_, m_lt_param[i_name_]);
    return true;
}

void LimbTorqueController::ImpactHandler()
{
    //forceInよりm_lt_paramのiteratorについてチェックしたほうがよい
    //forceはgeteActualParameterで読んでおく?
    for (unsigned int i=0; i<m_forceIn.size(); i++){
        if ( m_forceIn[i]->isNew() ) {
            m_forceIn[i]->read();
            if (m_force[i].data[2] <= -100.0){
                std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
                for (int j=0; j<i; ++j) {++it;}
                std::string ee_name = it->first;
                m_lt_param[ee_name].pgain = 0.1*default_pgain[i];
                m_lt_param[ee_name].dgain = 0.1*default_dgain[i];
            }
        }
    }
    //ゲイン低下からのリリースもしなくてはならない
}

extern "C"
{
    void LimbTorqueControllerInit(RTC::Manager* manager)
    {
        RTC::Properties profile(limbtorquecontroller_spec);
        manager->registerFactory(profile,
                                 RTC::Create<LimbTorqueController>,
                                 RTC::Delete<LimbTorqueController>);
    }
};
