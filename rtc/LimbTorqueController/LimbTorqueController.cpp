#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "LimbTorqueController.h"
#include "../ImpedanceController/JointPathEx.h"
#include <hrpModel/JointPath.h>
#include <hrpUtil/Eigen3d.h>

typedef coil::Guard<coil::Mutex> Guard;

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
        ""
    };
// </rtc-template>

LimbTorqueController::LimbTorqueController(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      m_qCurrentIn("qCurrent", m_qCurrent), //センサ値:重力補償用
      m_qRefIn("qRef", m_qRef),
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
    addInPort("qRef", m_qRefIn);
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

    // setting from conf file
    coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
    std::map<std::string, std::string> base_name_map;
    if (end_effectors_str.size() > 0) {
        size_t prop_num = 10; //the number of parameters of each end effector in conf file: "ee-name, ee-link-name, base-link-name, pos.x, pos.y, pos.z, rot-axis,x, rot-axis.y, rot-axis.z, rot-angle"
        size_t num = end_effectors_str.size()/prop_num;
        for (size_t i = 0; i < num; i++) {
            std::string ee_name, ee_target, ee_base;
            coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
            coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
            coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());
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
        bool is_ee_exists = false;
        for ( std::map<std::string, ee_trans>::iterator it = ee_map.begin(); it != ee_map.end(); ++it ) {
            hrp::Link* alink = m_robot->link(it->second.target_name);
            std::string tmp_base_name = base_name_map[it->first];
            while (alink != NULL && alink->name != tmp_base_name && !is_ee_exists) {
                if ( alink->name == sensor_link_name ) {
                    is_ee_exists = true;
                    ee_name = it->first;
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
        // 4. Set limb torque param
        p.sensor_name = sensor_name;
        p.pgain = 1.0; //TODO: tune, enable to read from conf
        p.dgain = 1.0; //TODO: tune, enable to read from conf
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
    qoldRef.resize(dof);
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

  //必要ならパラメータ等リセット
  return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t LimbTorqueController::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << "LimbTorqueController::onExecute(" << ec_id << ")" << std::endl;
   loop ++;

   //Read Import
   if (m_qCurrentIn.isNew()) {
       m_qCurrentIn.read();
   }
   if (m_qRefIn.isNew()) {
       m_qRefIn.read();
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

   getTargetParameters(); //上位から来るref値を変数にセット
   getActualParameters(); //センサ値を変数にセット

   //参照トルク計算
   //以下の関数内部で各joint->uにトルクを加算していく
   calcGravityCompensation(); //重力補償: uを設定
   calcJointDumpingTorque(); //関節角度ダンピング
   //calcEndEffectorForceTorque(); //あとで実装
   //calcMinMaxAvoidanceTorque();

   //データの記述?

   //Write Outport
   for ( size_t i = 0; m_robot->numJoints(); ++i){
       m_q.data[i] = m_robotRef->joint(i)->q; //same as input qRef at this moment
       m_tq.data[i] = m_robot->joint(i)->u;
   }

   m_qOut.write();
   m_tqOut.write();

   return RTC::RTC_OK;
}

void LimbTorqueController::getTargetParameters()
{
    for ( size_t i = 0; i < m_robotRef->numJoints(); ++i){
        m_robotRef->joint(i)->q = m_qRef.data[i];
        m_robotRef->joint(i)->dq = loop>0 ? ( m_qRef.data[i] - qoldRef[i] ) / m_dt : 0; //OK?
        qoldRef[i] = m_qRef.data[i];
    }
}

void LimbTorqueController::getActualParameters()
{
    //Current robot state
    for ( unsigned int i = 0; m_robot->numJoints(); ++i ){
        m_robot->joint(i)->q = m_qCurrent.data[i];
        m_robot->joint(i)->dq = loop>0 ? ( m_qCurrent.data[i] - qold[i] ) / m_dt : 0; //OK?
        qold[i] = m_qCurrent.data[i];
    }
    m_robot->rootLink()->p = hrp::Vector3::Zero();
    m_robot->calcForwardKinematics(); //何してるか理解してない(duplicated from ST)
    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    hrp::Matrix33 senR = sen->link->R * sen->localR;
    hrp::Matrix33 act_Rs(hrp::rotFromRpy(m_rpy.data.r, m_rpy.data.p, m_rpy.data.y));
    m_robot->rootLink()->R = act_Rs * (senR.transpose() * m_robot->rootLink()->R);
    m_robot->calcForwardKinematics(); //なぜ二回呼んでるか理解してない(duplicated from ST)
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
            GCCallback(manip->endLink(),tmp_f, tmp_tau);
        }
    } //end while
}

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
                double dq_error = act_joint->dq - ref_joint->dq; //inport追加の必要あり
                act_joint->u += m_lt_param[ee_name].pgain*q_error + m_lt_param[ee_name].dgain*dq_error;
            }}
    }
}

//duplicated from Body.cpp: calcInverseDynamics in openhrp3
void LimbTorqueController::GCCallback(hrp::Link* ptr, hrp::Vector3& out_f, hrp::Vector3& out_tau)
{
    hrp::Link* parent = ptr->parent;
    if(parent){
        hrp::Vector3 dsv, dsw, sv, sw;

        if(ptr->jointType != hrp::Link::FIXED_JOINT){
            sw.noalias() = parent->R * ptr->a;
            sv = ptr->p.cross(sw);
        }else{
            sw.setZero();
            sv.setZero();
        }
        dsv = parent->w.cross(sv) + parent->vo.cross(sw);
        dsw = parent->w.cross(sw);

        ptr->dw  = parent->dw  + dsw * ptr->dq;// + sw * ptr->ddq;
        ptr->dvo = parent->dvo + dsv * ptr->dq;// + sv * ptr->ddq;

        ptr->sw = sw;
        ptr->sv = sv;
    }

    hrp::Vector3  c,P,L;
    hrp::Matrix33 I,c_hat;

    c = ptr->R * ptr->c + ptr->p;
    I.noalias() = ptr->R * ptr->I * ptr->R.transpose();
    c_hat = hrp::hat(c);
    I.noalias() += ptr->m * c_hat * c_hat.transpose();
    P.noalias() = ptr->m * (ptr->vo + ptr->w.cross(c));
    L = ptr->m * c.cross(ptr->vo) + I * ptr->w;

    out_f   = ptr->m * (ptr->dvo + ptr->dw.cross(c)) + ptr->w.cross(P);
    out_tau = ptr->m * c.cross(ptr->dvo) + I * ptr->dw + ptr->vo.cross(P) + ptr->w.cross(L);

    if(ptr->child){
        hrp::Vector3 f_c;
        hrp::Vector3 tau_c;
        GCCallback(ptr->child, f_c, tau_c);
        out_f   += f_c;
        out_tau += tau_c;
    }

    ptr->u = ptr->sv.dot(out_f) + ptr->sw.dot(out_tau);  //must be "+=" if not called first

    if(ptr->sibling){
        hrp::Vector3 f_s;
        hrp::Vector3 tau_s;
        GCCallback(ptr->sibling, f_s, tau_s);
        out_f   += f_s;
        out_tau += tau_s;
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
