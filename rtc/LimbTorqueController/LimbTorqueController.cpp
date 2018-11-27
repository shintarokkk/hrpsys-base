#include <boost/algorithm/string.hpp> //only for iequals
#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "LimbTorqueController.h"
#include "../ImpedanceController/JointPathEx.h"
#include <hrpModel/JointPath.h>
#include <hrpUtil/Eigen3d.h>
#include <hrpUtil/MatrixSolvers.h>

#include "../RobotHardware/RobotHardwareService_impl.h"
#include <sys/time.h>

typedef coil::Guard<coil::Mutex> Guard;

#ifndef deg2rad
#define deg2rad(x) ((x) * M_PI / 180.0)
#endif
#ifndef rad2deg
#define rad2deg(rad) (rad * 180 / M_PI)
#endif
#define RTC_PERIOD 0.002 //[sec]

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
        "conf.default.pgain", "10",
        "conf.default.dgain", "3",
        ""
    };
// </rtc-template>

template<typename T>
std::ostream& operator<<(std::ostream& os, const RMSfilter<T>& fil)
{
    os << "RMSfilter: N = " << fil.N() << " dt = " << fil.dt() << std::endl << " data: [";
    for (int i=0; i<2*fil.N(); i++){
        os << "stored data #." << i << "[" << fil.data_que()[i] << "]" << std::endl;
    }
    os << "]";
    return os;
}

LimbTorqueController::LimbTorqueController(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      m_qCurrentIn("qCurrent", m_qCurrent), //センサ値:重力補償用
      m_dqCurrentIn("dqCurrent", m_dqCurrent),
      m_tqCurrentIn("tqCurrent", m_tqCurrent), //actual torque
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
    addInPort("tqCurrent", m_tqCurrentIn);
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
    std::cerr << "[" << m_profile.instance_name << "] force sensor ports" << std::endl;
    for (unsigned int i=0; i<nforce; i++){
        // actual inport
        m_forceIn[i] = new RTC::InPort<RTC::TimedDoubleSeq>(fsensor_names[i].c_str(), m_force[i]);
        m_force[i].data.length(6);
        registerInPort(fsensor_names[i].c_str(), *m_forceIn[i]);
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
    coil::vstring ltc_cgain_str = coil::split(prop["ltc_cgain"], ",");
    coil::vstring ltc_rgain_str = coil::split(prop["ltc_rgain"], ",");
    coil::vstring ltc_climit_str = coil::split(prop["ltc_climit"], ","); //トルク制御用の電流リミット(元のモデル等より少し小さめにすることを想定)
    coil::vstring ltc_collision_threshold_raw = coil::split(prop["ltc_collision_threshold"], "|"); //エンドエフェクタ毎に"|"で区切ってもらう
    coil::vstring ltc_eepgain_str = coil::split(prop["ltc_ee_pgain"], ",");
    coil::vstring ltc_eedgain_str = coil::split(prop["ltc_ee_dgain"], ",");
    std::map<std::string, std::string> base_name_map;
    if (end_effectors_str.size() > 0) {
        size_t prop_num = 10; //the number of parameters of each end effector in conf file: "ee-name, ee-link-name, base-link-name, pos.x, pos.y, pos.z, rot-axis,x, rot-axis.y, rot-axis.z, rot-angle"
        size_t num = end_effectors_str.size()/prop_num;
        default_pgain.resize(num);
        default_dgain.resize(num);
        default_cgain.resize(num);
        default_rgain.resize(num);
        for (size_t i = 0; i < num; i++) {
            std::string ee_name, ee_target, ee_base;
            double conf_pgain, conf_dgain, conf_cgain, conf_rgain;
            hrp::dmatrix one_gen_imat, one_old_gen_imat, one_gen_mom_observer_gain, one_collision_resistance_gain;
            hrp::dvector one_gen_mom, one_old_gen_mom, one_gen_mom_res;
            hrp::dvector one_accum_tau, one_accum_beta, one_accum_res, one_initial_gen_mom;
            hrp::dvector one_collision_threshold;
            hrp::dvector one_resist_of_one_step_before;

            coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
            coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
            coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());
            coil::stringTo(conf_pgain, ltc_pgain_str[i].c_str());
            coil::stringTo(conf_dgain, ltc_dgain_str[i].c_str());
            coil::stringTo(conf_cgain, ltc_cgain_str[i].c_str());
            coil::stringTo(conf_rgain, ltc_rgain_str[i].c_str());
            coil::vstring collision_threshold_str_for_one_ee = coil::split(ltc_collision_threshold_raw[i], ",");
            one_collision_threshold.resize(collision_threshold_str_for_one_ee.size());
            for (unsigned int j=0; j<collision_threshold_str_for_one_ee.size(); ++j){
                coil::stringTo(one_collision_threshold[j], collision_threshold_str_for_one_ee[j].c_str());
            }
            ee_trans eet;
            for (size_t j = 0; j < 3; j++) {
                coil::stringTo(eet.localPos(j), end_effectors_str[i*prop_num+3+j].c_str());
            }
            double tmpv[4];
            for (int j = 0; j < 4; j++ ) {
                coil::stringTo(tmpv[j], end_effectors_str[i*prop_num+6+j].c_str());
            }
            eet.localR = Eigen::AngleAxis<double>(tmpv[3], hrp::Vector3(tmpv[0], tmpv[1], tmpv[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
            eet.target_name = ee_target;
            default_pgain[i] = conf_pgain;
            default_dgain[i] = conf_dgain;
            default_cgain[i] = conf_cgain;
            default_rgain[i] = conf_rgain;
            ee_map.insert(std::pair<std::string, ee_trans>(ee_name , eet));
            base_name_map.insert(std::pair<std::string, std::string>(ee_name, ee_base));
            gen_imat.insert(std::pair<std::string, hrp::dmatrix>(ee_name, one_gen_imat));
            old_gen_imat.insert(std::pair<std::string, hrp::dmatrix>(ee_name, one_old_gen_imat));
            gen_mom_observer_gain.insert(std::pair<std::string, hrp::dmatrix>(ee_name, one_gen_mom_observer_gain));
            collision_resistance_gain.insert(std::pair<std::string, hrp::dmatrix>(ee_name, one_collision_resistance_gain));
            gen_mom.insert(std::pair<std::string, hrp::dvector>(ee_name, one_gen_mom));
            old_gen_mom.insert(std::pair<std::string, hrp::dvector>(ee_name, one_old_gen_mom));
            gen_mom_res.insert(std::pair<std::string, hrp::dvector>(ee_name, one_gen_mom_res));
            accum_tau.insert(std::pair<std::string, hrp::dvector>(ee_name, one_accum_tau));
            accum_beta.insert(std::pair<std::string, hrp::dvector>(ee_name, one_accum_beta));
            accum_res.insert(std::pair<std::string, hrp::dvector>(ee_name, one_accum_res));
            collision_uncheck_count.insert(std::pair<std::string, int>(ee_name, 0));
            collision_link.insert(std::pair<std::string, std::string>(ee_name, "no collision"));
            default_collision_threshold.insert(std::pair<std::string, hrp::dvector>(ee_name, one_collision_threshold));
            initial_gen_mom.insert(std::pair<std::string, hrp::dvector>(ee_name, one_initial_gen_mom));
            resist_of_one_step_before.insert(std::pair<std::string, hrp::dvector>(ee_name, one_resist_of_one_step_before));
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
        if (m_lt_col_param.find(ee_name) != m_lt_col_param.end()) {
            std::cerr << "[" << m_profile.instance_name << "]   Already limb torque param (target_name=" << sensor_link_name << ", ee_name=" << ee_name << ") exists!!. Limb torque param for " << sensor_name << " cannot be added!!" << std::endl;
            continue;
        }
        // 3. Check whether joint path is adequate.
        hrp::Link* target_link = m_robot->link(ee_map[ee_name].target_name);
        hrp::Link* ref_target_link = m_robotRef->link(ee_map[ee_name].target_name);
        LTParam p, p_ref;
        p.manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(base_name_map[ee_name]), target_link, m_dt, false, std::string(m_profile.instance_name)));
        p_ref.manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robotRef, m_robotRef->link(base_name_map[ee_name]), ref_target_link, m_dt, false, std::string(m_profile.instance_name)));
        if ( ! p.manip ) {
            std::cerr << "[" << m_profile.instance_name << "]   Invalid joint path from " << base_name_map[ee_name] << " to " << target_link->name << "!! Limb torque param for " << sensor_name << " cannot be added!!" << std::endl;
            continue;
        }

        // 4. Set limb torque param
        p.sensor_name = sensor_name;
        p_ref.sensor_name = sensor_name;
        p.ee_name = ee_name;
        p.target_name = ee_map[ee_name].target_name;
        //現状すべてのlimbに同じ(最初の)ゲインを設定している
        p.pgain = default_pgain[0];  p_ref.pgain = default_pgain[0];
        p.dgain = default_dgain[0];  p_ref.dgain = default_dgain[0];
        p.gravitational_acceleration = hrp::Vector3(0.0, 0.0, 9.80665); //TODO: set from outside
        p_ref.gravitational_acceleration = hrp::Vector3(0.0, 0.0, 9.80665);
        hrp::Vector3 ee_pgainp_vec, ee_pgainr_vec, ee_dgainp_vec, ee_dgainr_vec;
        for (int i=0; i<3; i++){
            coil::stringTo(ee_pgainp_vec(i), ltc_eepgain_str[i].c_str());
            coil::stringTo(ee_pgainr_vec(i), ltc_eepgain_str[i+3].c_str());
            coil::stringTo(ee_dgainp_vec(i), ltc_eedgain_str[i].c_str());
            coil::stringTo(ee_dgainr_vec(i), ltc_eedgain_str[i+3].c_str());
        }
        p.ee_pgain_p = ee_pgainp_vec.asDiagonal();
        p.ee_pgain_r = ee_pgainr_vec.asDiagonal();
        p.ee_dgain_p = ee_dgainp_vec.asDiagonal();
        p.ee_dgain_r = ee_dgainr_vec.asDiagonal();
        m_lt_param[ee_name] = p;
        m_ref_lt_param[ee_name] = p_ref;

        //set collision param
        CollisionParam col_p;
        col_p.cgain = default_cgain[0];
        col_p.resist_gain = default_rgain[0];
        col_p.collision_threshold = default_collision_threshold[ee_name];
        col_p.check_mode = 0;
        col_p.handle_mode = 0;
        col_p.max_collision_uncheck_count = 1000;
        m_lt_col_param[ee_name] = col_p;
        hrp::dvector temp_dvec6 = hrp::dvector(6,1);
        temp_dvec6 << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        resist_direction[ee_name] = temp_dvec6;
        //for ee compensation
        hrp::dmatrix temp_jacobian(6, p.manip->numJoints()), temp_inv_j(p.manip->numJoints(), 6);
        act_ee_jacobian[ee_name] = temp_jacobian;
        ref_ee_jacobian[ee_name] = temp_jacobian;
        inv_ee_jacobian[ee_name] = temp_inv_j;
        inv_ee_jacobian_t[ee_name] = temp_jacobian;
        ee_pos_comp_force[ee_name] = hrp::Vector3::Zero();
        ee_ori_comp_moment[ee_name] = hrp::Vector3::Zero();
        ee_vel_comp_force[ee_name] = hrp::Vector3::Zero();
        ee_w_comp_moment[ee_name] = hrp::Vector3::Zero();
        ee_pos_ori_comp_wrench[ee_name] = hrp::dvector::Zero(6);
        ee_vel_w_comp_wrench[ee_name] = hrp::dvector::Zero(6);
        ee_compensation_torque[ee_name] = hrp::dvector::Zero(p.manip->numJoints());
        null_space_torque[ee_name] = hrp::dvector::Zero(p.manip->numJoints());
        ee_pos_error[ee_name] = hrp::Vector3::Zero();
        ee_ori_error[ee_name] = hrp::Vector3::Zero();
        current_act_ee_pos[ee_name] = hrp::Vector3::Zero();
        current_ref_ee_pos[ee_name] = hrp::Vector3::Zero();
        prev_ref_ee_pos[ee_name] = hrp::Vector3::Zero();
        act_ee_vel[ee_name] = hrp::Vector3::Zero();
        ref_ee_vel[ee_name] = hrp::Vector3::Zero();
        act_ee_w[ee_name] = hrp::Vector3::Zero();
        ref_ee_w[ee_name] = hrp::Vector3::Zero();
        dq_for_each_arm[ee_name] = hrp::dvector::Zero(p.manip->numJoints());
        current_act_ee_rot[ee_name] = hrp::Matrix33::Identity();
        current_ref_ee_rot[ee_name] = hrp::Matrix33::Identity();
        prev_ref_ee_rot[ee_name] = hrp::Matrix33::Identity();
        ee_vel_error[ee_name] = hrp::dvector::Zero(6);
        ee_w_error[ee_name] = hrp::dvector::Zero(6);;
        RMSfilter<hrp::Vector3> temp_vel_filter(5, RTC_PERIOD);
        ee_vel_filter[ee_name] = temp_vel_filter;
        RMSfilter<hrp::Matrix33> temp_w_filter(5, RTC_PERIOD);
        ee_w_filter[ee_name] = temp_w_filter;
        oscontrol_initialized[ee_name] = false;
        //for velocity estimation
        velest_now[ee_name] = hrp::dvector::Zero(p.manip->numJoints());
        velest_prev[ee_name] = hrp::dvector::Zero(p.manip->numJoints());
        velest_prevprev[ee_name] = hrp::dvector::Zero(p.manip->numJoints());
        imp_now[ee_name] = hrp::dvector::Zero(p.manip->numJoints());
        imp_prev[ee_name] = hrp::dvector::Zero(p.manip->numJoints());
        imp_prevprev[ee_name] = hrp::dvector::Zero(p.manip->numJoints());
        velest_initialized[ee_name] = false;
        overwrite_refangle[ee_name] = false;
        stop_overwriting_q_transition_count[ee_name] = 0;

        std::cerr << "[" << m_profile.instance_name << "]   sensor = " << sensor_name << ", sensor-link = " << sensor_link_name << ", ee_name = " << ee_name << ", ee_link = " << target_link->name << std::endl;
    }

    std::vector<std::pair<hrp::Link*, hrp::Link*> > interlocking_joints;
    readInterlockingJointsParamFromProperties(interlocking_joints, m_robot, prop["interlocking_joints"], std::string(m_profile.instance_name));
    if (interlocking_joints.size() > 0) {
        for ( std::map<std::string, LTParam>::iterator it = m_lt_param.begin(); it != m_lt_param.end(); ++it ) {
            std::cerr << "[" << m_profile.instance_name << "] Interlocking Joints for [" << it->first << "]" << std::endl;
            it->second.manip->setInterlockingJointPairIndices(interlocking_joints, std::string(m_profile.instance_name));
        }
    }
    //initiate variables for CollisionDetector
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        std::string ee_name = it->first;
        LTParam& param = it->second;
        gen_mom[ee_name] = hrp::dvector::Zero(param.manip->numJoints());
        old_gen_mom[ee_name] = hrp::dvector::Zero(param.manip->numJoints());
        gen_mom_res[ee_name] = hrp::dvector::Zero(param.manip->numJoints());
        accum_tau[ee_name] = hrp::dvector::Zero(param.manip->numJoints());
        accum_beta[ee_name] = hrp::dvector::Zero(param.manip->numJoints());
        accum_res[ee_name] = hrp::dvector::Zero(param.manip->numJoints());
        initial_gen_mom[ee_name] = hrp::dvector::Zero(param.manip->numJoints());
        resist_of_one_step_before[ee_name] = hrp::dvector::Zero(param.manip->numJoints());
        resist_dir_torque[ee_name] = hrp::dvector::Zero(param.manip->numJoints());
        relax_dir_torque[ee_name] = hrp::dvector::Zero(param.manip->numJoints());

        ++it;
    }
    std::map<std::string, CollisionParam>::iterator c_it = m_lt_col_param.begin();
    while(c_it != m_lt_col_param.end()){
        std::string ee_name = c_it->first;
        CollisionParam& param = c_it->second;
        hrp::dvector temp_gains = hrp::dvector::Constant(m_lt_param[ee_name].manip->numJoints(), param.cgain);
        hrp::dvector temp_rgains = hrp::dvector::Constant(m_lt_param[ee_name].manip->numJoints(), param.resist_gain);
        gen_mom_observer_gain[ee_name] = temp_gains.asDiagonal();
        collision_resistance_gain[ee_name] = temp_rgains.asDiagonal();
        collision_detector_initialized[ee_name] = false;
        gen_imat_initialized[ee_name] = false;
        ++c_it;
    }

    link_inertia_matrix.resize(dof);
    for ( unsigned int i = 0 ; i < dof; i++ ){
        link_inertia_matrix[i] = hrp::dmatrix::Zero(6,6);
        hrp::Vector3 RotorInertia = hrp::dvector::Constant(3, m_robot->joint(i)->Jm2);
        hrp::Matrix33 RotorInertiaMat = RotorInertia.asDiagonal();
        hrp::Matrix33 TotalInertia = m_robot->joint(i)->I + RotorInertiaMat;
        link_inertia_matrix[i] <<
            m_robot->joint(i)->m, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, m_robot->joint(i)->m, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, m_robot->joint(i)->m, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, TotalInertia.row(0),
            0.0, 0.0, 0.0, TotalInertia.row(1),
            0.0, 0.0, 0.0, TotalInertia.row(2);
    }

    // allocate memory for outPorts
    m_q.data.length(dof);
    m_tq.data.length(dof);
    qold.resize(dof);
    dqold.resize(dof);
    qoldRef.resize(dof);
    dqoldRef.resize(dof);
    temp_ref_vel.resize(dof);
    temp_ref_acc.resize(dof);
    temp_vel.resize(dof);
    temp_acc.resize(dof);
    temp_ref_u.resize(dof);
    temp_u.resize(dof);
    temp_invdyn_result.resize(dof);
    overwritten_qRef_prev.resize(dof); // use this because ref_vel in iob is calculated as (ref_q - prev_ref_q) / dt
    estimated_reference_velocity.resize(dof);
    transition_velest.resize(dof);
    log_est_q.resize(dof);
    log_act_q.resize(dof);
    log_ref_q.resize(dof);
    max_stop_overwriting_q_transition_count = 1000;
    for (int i=0; i<dof; ++i){
        coil::stringTo(m_robot->joint(i)->climit, ltc_climit_str[i].c_str()); //limiting current value to one in conf file
        qoldRef[i] = 0.0;
        dqoldRef[i] = 0.0;
    }
    actual_torque_vector.resize(dof);
    loop = 0;

    spit_log = false;

    //set IIR filter constants (for velocity estimation)
    iir_cutoff_frequency = 10.0; //Hz
    iir_alpha = std::cos(M_PI*iir_cutoff_frequency*RTC_PERIOD/2.0) / std::sin(M_PI*iir_cutoff_frequency*RTC_PERIOD/2.0);
    iir_a0 = 1.0 / ( 1.0 + std::sqrt(2.0)*iir_alpha + iir_alpha*iir_alpha );
    iir_a1 = 2.0 * iir_a0;
    iir_a2 = iir_a0;
    iir_b1 = ( 2.0 - 2.0*iir_alpha*iir_alpha ) * iir_a0;
    iir_b2 = ( 1.0 - std::sqrt(2.0)*iir_alpha + iir_alpha*iir_alpha ) * iir_a0;

#if 1  //for legged robot
    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    if (sen == NULL) {
        std::cerr << "[" << m_profile.instance_name << "] WARNING! This robot model has no GyroSensor named 'gyrometer'! " << std::endl;
    }
#endif
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
  Guard guard(m_mutex);
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;

  //必要ならパラメータ等リセット
  return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t LimbTorqueController::onExecute(RTC::UniqueId ec_id)
{
    if(DEBUGP){
       std::cout << "[" << m_profile.instance_name << "]" << "(" << ec_id << "):" << __func__ << std::endl;
   }
   loop ++;

   //Read Import
    for (unsigned int i=0; i<m_forceIn.size(); i++){
      if ( m_forceIn[i]->isNew() ) {
          m_forceIn[i]->read();
      }
        // TODO: add ref force
        // if ( m_ref_forceIn[i]->isNew() ) {
        //     m_ref_forceIn[i]->read();
        // }
   }
   if (m_qCurrentIn.isNew()) {
       m_qCurrentIn.read();
   }
   if (m_dqCurrentIn.isNew()) {
       m_dqCurrentIn.read();
   }
   if (m_tqCurrentIn.isNew()) {
       m_tqCurrentIn.read();
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

   Guard guard(m_mutex);
   if ( m_qRef.data.length() ==  m_robot->numJoints() &&
        m_qCurrent.data.length() ==  m_robot->numJoints() &&
        m_dqCurrent.data.length() == m_robot->numJoints() &&
        m_tqCurrent.data.length() == m_robot->numJoints()) {
       getTargetParameters(); //上位から来るref値を変数にセット
       calcForceMoment();
       getActualParameters(); //センサ値を変数にセット

       if (torque_output_type == CALC_TORQUE) {

           calcLimbInverseDynamics();
           //calcJointDumpingTorque(); //関節角度ダンピング
           calcEECompensation();
           calcNullJointDumping();
           //calcMinMaxAvoidanceTorque(); //Null Spaceで考慮しているので使わない
           //estimateRefVel(); // to be tested more

       }
       else if (torque_output_type == REF_TORQUE) {
           addDumpingToRefTorque();
       }

       CollisionDetector();
       CollisionHandler();

       for ( size_t i = 0; i<m_robot->numJoints(); ++i){
           m_q.data[i] = m_robotRef->joint(i)->q;
           hrp::Link* current_joint = m_robot->joint(i);
           //checking torque limits
           double max_torque = current_joint->climit * current_joint->gearRatio * current_joint->torqueConst;
           if (current_joint->u > max_torque){
               if(loop%100 == 0){
                   std::cout << "[ltc]joint(" << i << ") reached max torque limit!!"
                             << "original ref=" << current_joint->u << ",max=" << max_torque
                             << std::endl;
               }
               current_joint->u = max_torque;
           }else if (current_joint->u < -max_torque){
               if(loop%100 == 0){
                   std::cout << "[ltc]joint(" << i << ") reached min torque limit!!"
                             << " original ref=" << current_joint->u << ",min=" << -max_torque
                             << std::endl;
               }
               current_joint->u = -max_torque;
           }
           m_tq.data[i] = m_robot->joint(i)->u;
       }

       m_qOut.write();
       m_tqOut.write();
   }

   if(spit_log){
       DebugOutput();
   }

   if (loop%1000 == 0){
       std::cout << "[ltc] torque ref is: [";
       for ( size_t i = 0; i<m_robot->numJoints(); ++i){
           std::cout << m_tq.data[i] << ", ";
       }
       std::cout << "]" << std::endl;
   }

   return RTC::RTC_OK;
}

void LimbTorqueController::getTargetParameters()
{
    for ( size_t i = 0; i < m_robotRef->numJoints(); ++i){
        m_robotRef->joint(i)->q = m_qRef.data[i];
        log_ref_q(i) = m_qRef.data[i];
        m_robotRef->joint(i)->dq = loop>1 ? ( m_qRef.data[i] - qoldRef[i] ) / m_dt : 0;
        estimated_reference_velocity(i) = m_robotRef->joint(i)->dq;
        m_robotRef->joint(i)->ddq = (m_robotRef->joint(i)->dq - dqoldRef[i]) / m_dt;
        //if 外から指令値in set 指令値 as dqRef & ddqRef
        qoldRef[i] = m_qRef.data[i];
        dqoldRef[i] = m_robotRef->joint(i)->dq;
        if (torque_output_type == REF_TORQUE){
            //m_robot->joint(i)->u = m_tqRef.data[i];
            m_robot->joint(i)->u = 0; //temporary
        }
    }
#if 1 //for legged robot
    target_root_p = hrp::Vector3(m_basePos.data.x, m_basePos.data.y, m_basePos.data.z);
    target_root_R = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
#else
    target_root_p = hrp::Vector3::Zero();
    target_root_R = hrp::Matrix33::Identity();
#endif
    m_robotRef->rootLink()->p = target_root_p;
    m_robotRef->rootLink()->R = target_root_R;
    m_robotRef->calcForwardKinematics();
    std::map<std::string, LTParam>::iterator it = m_ref_lt_param.begin();
    while(it != m_ref_lt_param.end()){
        std::string ee_name = it->first;
        LTParam& param = it->second;
        if(param.is_active){
            hrp::JointPathExPtr manip = param.manip;
            manip->calcJacobian(ref_ee_jacobian[ee_name]);
        }
        it++;
    }
}

// copied from ImpedanceController
void LimbTorqueController::calcForceMoment ()
{
      for (unsigned int i=0; i<m_forceIn.size(); i++){
        if ( m_force[i].data.length()==6 ) {
          std::string sensor_name = m_forceIn[i]->name();
          hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_name);
          hrp::Vector3 data_p(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
          hrp::Vector3 data_r(m_force[i].data[3], m_force[i].data[4], m_force[i].data[5]);
          // hrp::Vector3 ref_data_p(m_ref_force[i].data[0], m_ref_force[i].data[1], m_ref_force[i].data[2]);
          // hrp::Vector3 ref_data_r(m_ref_force[i].data[3], m_ref_force[i].data[4], m_ref_force[i].data[5]);
          if ( DEBUGP ) {
            std::cerr << "[" << m_profile.instance_name << "] force and moment [" << sensor_name << "]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   sensor force  = " << data_p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   sensor moment = " << data_r.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
            // std::cerr << "[" << m_profile.instance_name << "]   reference force  = " << ref_data_p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
            // std::cerr << "[" << m_profile.instance_name << "]   reference moment = " << ref_data_r.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
          }
          hrp::Matrix33 sensorR;
          hrp::Vector3 sensorPos, eePos;
          if ( sensor ) {
            // real force sensore
            sensorR = sensor->link->R * sensor->localR;
            sensorPos = sensor->link->p + sensorR * sensor->localPos;
          } else if ( m_vfs.find(sensor_name) !=  m_vfs.end()) {
            // virtual force sensor
            if ( DEBUGP ) {
              std::cerr << "[" << m_profile.instance_name << "]   sensorR = " << m_vfs[sensor_name].localR.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
            }
            sensorR = m_vfs[sensor_name].link->R * m_vfs[sensor_name].localR;
            sensorPos = m_vfs[sensor_name].link->p + m_vfs[sensor_name].link->R * m_vfs[sensor_name].localPos;
          } else {
            std::cerr << "[" << m_profile.instance_name << "]   unknown force param" << std::endl;
          }
          abs_forces[sensor_name] = sensorR * data_p;
          for ( std::map<std::string, LTParam>::iterator it = m_lt_param.begin(); it != m_lt_param.end(); it++ ) {
              if ( it->second.sensor_name == sensor_name ) eePos = ee_map[sensor_name].localPos;
          }
          abs_moments[sensor_name] = sensorR * data_r + (sensorPos - eePos).cross(abs_forces[sensor_name]);
          // End effector local frame
          // hrp::Matrix33 eeR (parentlink->R * ee_map[parentlink->name].localR);
          // abs_ref_forces[sensor_name] = eeR * ref_data_p;
          // abs_ref_moments[sensor_name] = eeR * ref_data_r;
          // World frame
          // abs_ref_forces[sensor_name] = ref_data_p;
          // abs_ref_moments[sensor_name] = ref_data_r;
          if ( DEBUGP ) {
            std::cerr << "[" << m_profile.instance_name << "]   abs force  = " << abs_forces[sensor_name].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   abs moment = " << abs_moments[sensor_name].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
            // std::cerr << "[" << m_profile.instance_name << "]   abs ref force  = " << abs_ref_forces[sensor_name].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
            // std::cerr << "[" << m_profile.instance_name << "]   abs ref moment = " << abs_ref_moments[sensor_name].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
          }
        }
      }
}

void LimbTorqueController::getActualParameters()
{
    //Current robot state
    for ( unsigned int i = 0; i<m_robot->numJoints(); ++i ){
        m_robot->joint(i)->q = m_qCurrent.data[i];
        log_act_q(i) = m_qCurrent.data[i];
        //m_robot->joint(i)->dq = loop>1 ? ( m_qCurrent.data[i] - qold[i] ) / m_dt : 0;
        m_robot->joint(i)->dq = m_dqCurrent.data[i];
        m_robot->joint(i)->ddq = m_robotRef->joint(i)->ddq; //TODO
        actual_torque_vector(i) = m_tqCurrent.data[i];
        if (torque_output_type != REF_TORQUE){
            m_robot->joint(i)->u = 0;
        }
        qold[i] = m_qCurrent.data[i];
    }
    m_robot->rootLink()->p = target_root_p;
    m_robot->rootLink()->R = target_root_R;
    m_robot->calcForwardKinematics(); //is this part necessary?
#if 1 //for legged robot
    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    hrp::Matrix33 senR = sen->link->R * sen->localR;
    hrp::Matrix33 act_Rs(hrp::rotFromRpy(m_rpy.data.r, m_rpy.data.p, m_rpy.data.y));
#else
    hrp::Matrix33 senR = hrp::Matrix33::Identity();
    hrp::Matrix33 act_Rs = hrp::Matrix33::Identity();
#endif
    m_robot->rootLink()->R = act_Rs * (senR.transpose() * m_robot->rootLink()->R);
    m_robot->calcForwardKinematics();
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        std::string ee_name = it->first;
        LTParam& param = it->second;
        if(param.is_active){
            hrp::JointPathExPtr manip = param.manip;
            manip->calcJacobian(act_ee_jacobian[ee_name]);
        }
        it++;
    }
}

void LimbTorqueController::calcLimbInverseDynamics()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    std::map<std::string, LTParam>::iterator ref_it = m_ref_lt_param.begin();
    for(int i=0; i<m_robot->numJoints(); ++i){
        temp_ref_vel[i] = m_robotRef->joint(i)->dq;
        temp_ref_acc[i] = m_robotRef->joint(i)->ddq;
        temp_ref_u[i] = m_robotRef->joint(i)->u;
        temp_vel[i] = m_robot->joint(i)->dq;
        temp_acc[i] = m_robot->joint(i)->ddq;
        temp_u[i] = m_robot->joint(i)->u;
        temp_invdyn_result[i] = 0;
    }
    while(ref_it != m_ref_lt_param.end()){
        LTParam& ref_param = ref_it->second;
        LTParam& param = it->second;
        if (ref_param.is_active) {
            if (DEBUGP) {
                std::cerr << "ここにデバッグメッセージを流す" << std::endl;
            }
            hrp::Link* ref_base_link = ref_param.manip->baseLink();
            hrp::Link* base_link = param.manip->baseLink();
            Eigen::MatrixXd ref_base_rot = ref_base_link->R;
            Eigen::MatrixXd base_rot = base_link->R;
            hrp::Vector3 out_f, out_tau;
            out_f = Eigen::Vector3d::Zero(3);
            out_tau = Eigen::Vector3d::Zero(3);
            //TODO: acceleration of baseLink due to fullbody movement
            if (ref_base_link->parent){
                ref_base_link->parent->dvo = ref_base_rot * ref_param.gravitational_acceleration;
                base_link->parent->dvo = base_rot * param.gravitational_acceleration;
            } else{
                ref_base_link->dvo = ref_base_rot * ref_param.gravitational_acceleration;
                base_link->dvo = base_rot * param.gravitational_acceleration;
            }
            m_robotRef->calcInverseDynamics(ref_base_link, out_f, out_tau);
            for(int i=0; i<ref_param.manip->numJoints(); i++){
                int jid = ref_param.manip->joint(i)->jointId;
                temp_invdyn_result[jid] += m_robotRef->joint(jid)->u;
                m_robotRef->joint(jid)->u = 0.0;
                m_robotRef->joint(jid)->dq = 0.0;
                m_robotRef->joint(jid)->ddq = 0.0;
                m_robot->joint(jid)->u = 0.0;
                m_robot->joint(jid)->dq = 0.0;
                m_robot->joint(jid)->ddq = 0.0;
            }
            out_f = Eigen::Vector3d::Zero(3);
            out_tau = Eigen::Vector3d::Zero(3);
            m_robotRef->calcInverseDynamics(ref_base_link, out_f, out_tau); //gravity compensation for ref
            out_f = Eigen::Vector3d::Zero(3);
            out_tau = Eigen::Vector3d::Zero(3);
            m_robot->calcInverseDynamics(base_link, out_f, out_tau); //gravity compensation for act
            for(int i=0; i<ref_param.manip->numJoints(); i++){
                int jid = ref_param.manip->joint(i)->jointId;
                temp_invdyn_result[jid] += m_robot->joint(jid)->u - m_robotRef->joint(jid)->u;
            }
        }
        ++it;
        ++ref_it;
    }
    for(int i=0; i<m_robot->numJoints(); i++){
        m_robot->joint(i)->u = temp_u[i] + temp_invdyn_result[i];
        m_robotRef->joint(i)->dq = temp_ref_vel[i];
        m_robotRef->joint(i)->ddq = temp_ref_acc[i];
        m_robotRef->joint(i)->u = 0;
        m_robot->joint(i)->dq = temp_vel[i];
        m_robot->joint(i)->ddq = temp_acc[i];
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
        std::string ee_name = it->first;
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
                if ( (now_angle+margin) >= max_angle ) {
                    double max_torque = manip->joint(i)->climit * manip->joint(i)->gearRatio * manip->joint(i)->torqueConst;
                    double avoid_torque = - max_torque/margin*(now_angle - (max_angle-margin));
                    manip->joint(i)->u = std::min(manip->joint(i)->u, avoid_torque);
                    if (loop%500 == 0){
                        std::cout << "!!MinMax Angle Warning!!" << "[" << m_profile.instance_name << "] " << manip->joint(i)->name << " is near limit: " << "max=" << rad2deg(max_angle) << ", now=" << rad2deg(now_angle) << ", applying min/max avoidance torque." << std::endl;}
                }else if ( (now_angle-margin) <= min_angle) {
                    double max_torque = manip->joint(i)->climit * manip->joint(i)->gearRatio * manip->joint(i)->torqueConst;
                    double avoid_torque = max_torque/margin*(min_angle+margin - now_angle);
                    manip->joint(i)->u = std::max(manip->joint(i)->u, avoid_torque);
                    if (loop%200 == 0){
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
                // act_joint->u += - m_lt_param[ee_name].dgain * act_joint->dq;
                act_joint->u = 0;
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
        m_ref_lt_param[i_name_].is_active = true;
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
        //TODO: transition?
        m_lt_param[i_name_].is_active = false;
        m_ref_lt_param[i_name_].is_active = false;
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

        hrp::dvector ee_pgain_vec(6), ee_dgain_vec(6);
        ee_pgain_vec << i_param_.ee_pgain[0], i_param_.ee_pgain[1], i_param_.ee_pgain[2], i_param_.ee_pgain[3], i_param_.ee_pgain[4], i_param_.ee_pgain[5];
        ee_dgain_vec << i_param_.ee_dgain[0], i_param_.ee_dgain[1], i_param_.ee_dgain[2], i_param_.ee_dgain[3], i_param_.ee_dgain[4], i_param_.ee_dgain[5];
        m_lt_param[name].ee_pgain_p = ee_pgain_vec.head(3).asDiagonal();
        m_lt_param[name].ee_pgain_r = ee_pgain_vec.tail(3).asDiagonal();
        m_lt_param[name].ee_dgain_p = ee_dgain_vec.head(3).asDiagonal();
        m_lt_param[name].ee_dgain_r = ee_dgain_vec.tail(3).asDiagonal();
        hrp::dmatrix concatenated_pgain = ee_pgain_vec.asDiagonal();  //just for print
        hrp::dmatrix concatenated_dgain = ee_dgain_vec.asDiagonal();  //just for print

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
        std::cerr << "[" << m_profile.instance_name << "]       ee_pgain : " << concatenated_pgain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]      ee_dgain : " << concatenated_dgain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
    }
    return true;
}

void LimbTorqueController::copyLimbTorqueControllerParam(LimbTorqueControllerService::limbtorqueParam& i_param_, const LTParam& param)
{
    i_param_.Pgain = param.pgain;
    i_param_.Dgain = param.dgain;
    for (int i = 0; i < 3; i++){
        i_param_.ee_pgain[i] = param.ee_pgain_p(i,i);
        i_param_.ee_pgain[i+3] = param.ee_pgain_r(i,i);
        i_param_.ee_dgain[i] = param.ee_dgain_p(i,i);
        i_param_.ee_dgain[i+3] = param.ee_dgain_r(i,i);
    }

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

bool LimbTorqueController::setCollisionParam(const std::string& i_name_, OpenHRP::LimbTorqueControllerService::collisionParam i_param_)
{
    Guard guard(m_mutex);
    std::string name = std::string(i_name_);
    if ( m_lt_col_param.find(name) == m_lt_col_param.end() ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find collision param [" << name << "]" << std::endl;
        return false;
    }

    std::cerr << "[" << m_profile.instance_name << "] Update collision parameters" << std::endl;

    for (unsigned int i=0; i<m_lt_col_param[name].collision_threshold.size(); ++i){
        m_lt_col_param[name].collision_threshold[i] = i_param_.Thresh[i];
    }
    m_lt_col_param[name].cgain = i_param_.Cgain;
    m_lt_col_param[name].resist_gain = i_param_.Rgain;
    //m_lt_col_param[name].collisionhandle_p = i_param_.Handle;
    m_lt_col_param[name].max_collision_uncheck_count = i_param_.MaxCount;
    //m_lt_col_param[name].test_bool1 = i_param_.TestB1;
    //m_lt_col_param[name].test_int1 = i_param_.TestI1;
    m_lt_col_param[name].check_mode = i_param_.CheckMode;
    m_lt_col_param[name].handle_mode = i_param_.HandleMode;
    collision_uncheck_count[name] = 0; //initialize to 0(check collision)

    hrp::dvector temp_gains = hrp::dvector::Constant(m_lt_param[name].manip->numJoints(), m_lt_col_param[name].cgain);
    hrp::dvector temp_rgains = hrp::dvector::Constant(m_lt_param[name].manip->numJoints(), m_lt_col_param[name].resist_gain);
    gen_mom_observer_gain[name] = temp_gains.asDiagonal();
    collision_resistance_gain[name] = temp_rgains.asDiagonal();

    std::cerr << "[" << m_profile.instance_name << "] set parameters" << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]             name : " << name << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]  Collision Checking Threshold : " << m_lt_col_param[name].collision_threshold.transpose() << std::endl;
    std::cerr << "[" << m_profile.instance_name << "] Cgain : " << m_lt_col_param[name].cgain << std::endl;
    std::cerr << "[" << m_profile.instance_name << "] Rgain : " << m_lt_col_param[name].resist_gain << std::endl;
    std::cerr << "[" << m_profile.instance_name << "] Collision Check Mode : " << m_lt_col_param[name].check_mode << std::endl;
    std::cerr << "[" << m_profile.instance_name << "] Collision Handle Mode : " << m_lt_col_param[name].handle_mode << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]      max collision uncheck count(uncheck time = count*hrpsys_periodic_rate) : " << m_lt_col_param[name].max_collision_uncheck_count << std::endl;
    return true;
}

void LimbTorqueController::copyCollisionParam(LimbTorqueControllerService::collisionParam& i_param_, const CollisionParam& param)
{
    for (size_t i=0; i<param.collision_threshold.size(); ++i){
        i_param_.Thresh[i] = param.collision_threshold[i];
    }
    i_param_.Cgain = param.cgain;
    i_param_.Rgain = param.resist_gain;
    i_param_.MaxCount = param.max_collision_uncheck_count;
    i_param_.CheckMode = param.check_mode;
    i_param_.HandleMode = param.handle_mode;
}

bool LimbTorqueController::getCollisionParam(const std::string& i_name_, LimbTorqueControllerService::collisionParam_out i_param_)
{
    if ( m_lt_col_param.find(i_name_) == m_lt_col_param.end() ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find collision param [" << i_name_ << "]" << std::endl;
        i_param_ = new OpenHRP::LimbTorqueControllerService::collisionParam();
        copyCollisionParam(*i_param_, CollisionParam());
        return false;
    }
    i_param_ = new OpenHRP::LimbTorqueControllerService::collisionParam();
    i_param_->Thresh.length(m_lt_col_param[i_name_].collision_threshold.size());
    copyCollisionParam(*i_param_, m_lt_col_param[i_name_]);
    return true;
}

bool LimbTorqueController::getCollisionTorque(const std::string& i_name_, OpenHRP::LimbTorqueControllerService::DblSequence_out c_vec_)
{
    if ( gen_mom_res.find(i_name_) == gen_mom_res.end() ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find limb torque controller param [" << i_name_ << "]" << std::endl;
        return false;
    }
    c_vec_ = new OpenHRP::LimbTorqueControllerService::DblSequence;
    c_vec_->length(gen_mom_res[i_name_].size());
    for (size_t i=0; i<gen_mom_res[i_name_].size(); ++i){
        c_vec_[i] = gen_mom_res[i_name_](i);
    }
    return true;
}

bool LimbTorqueController::getCollisionStatus(const std::string& i_name_, OpenHRP::LimbTorqueControllerService::collisionStatus_out i_param_)
{
    if ( gen_mom_res.find(i_name_) == gen_mom_res.end() ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find limb torque controller param [" << i_name_ << "]" << std::endl;
        return false;
    }
    i_param_ = new OpenHRP::LimbTorqueControllerService::collisionStatus();
    i_param_->CollisionTorque.length(m_lt_param[i_name_].manip->numJoints());
    for (int i=0; i<m_lt_param[i_name_].manip->numJoints(); i++){
        switch(m_lt_col_param[i_name_].check_mode){
        case 0:
            i_param_->CollisionTorque[i] = 0;
        case 1:
            i_param_->CollisionTorque[i] = gen_mom_res[i_name_](i);
        case 3:
            i_param_->CollisionTorque[i] = resist_dir_torque[i_name_](i);
        }
    }
    if (collision_uncheck_count[i_name_] > 0){
        i_param_->IsCollision = true;
        i_param_->CollisionLink = collision_link[i_name_].c_str();
    }else{
        i_param_->IsCollision = false;
        i_param_->CollisionLink = "No Collision";
    }
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

void LimbTorqueController::CollisionDetector()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        std::string ee_name = it->first;
        CollisionParam& col_param = m_lt_col_param[ee_name];
        switch (col_param.check_mode){
        case 0:
            break;
        case 1:
            CollisionDetector1(it);
            break;
        case 2:
            CollisionDetector2(it);
            break;
        case 3:
            CollisionDetector3(it);
            break;
        }
        if(collision_uncheck_count[ee_name] > 0){
            --collision_uncheck_count[ee_name];
        }
        ++it;
    }
}

//method E(Estimation of tau_ext via Momentum Observer) in S.Haddadin et al. "Robot Collisions: A Survey on Detection, Isolation, and Identification," TRO2017
void LimbTorqueController::CollisionDetector1(std::map<std::string, LTParam>::iterator it)
{
    int dof = m_robot->numJoints();
    hrp::dvector dq_backup(dof), ddq_backup(dof), tq_backup(dof);
    for (unsigned int i=0; i<dof; ++i){
        dq_backup[i] = m_robot->joint(i)->dq;
        ddq_backup[i] = m_robot->joint(i)->ddq;
        tq_backup[i] = m_robot->joint(i)->u;
    }
    std::string ee_name = it->first;
    LTParam& param = it->second;
    if (param.is_active) {
        if (DEBUGP) {
            std::cerr << "ここにデバッグメッセージを流す" << std::endl;
        }
        calcGeneralizedInertiaMatrix(it);
        hrp::JointPathExPtr manip = param.manip;
        int limbdof = manip->numJoints();
        hrp::Link* base_link = manip->baseLink();
        hrp::dmatrix base_rot = base_link->R;
        hrp::Vector3 out_f, out_tau;
        out_f = hrp::Vector3::Zero(3);
        out_tau = hrp::Vector3::Zero(3);
        base_link->dvo = base_rot * param.gravitational_acceleration;
        base_link->dw.setZero();  //assuming base_link is fixed! should be modified for humanoid
        for (unsigned int i=0; i<limbdof; ++i){
            manip->joint(i)->ddq = 0.0;
        }
        hrp::dvector c_and_g(limbdof), q_dot(limbdof), beta(limbdof), mot_tq(limbdof);
        hrp::dmatrix  inertia_dot;
        m_robot->calcInverseDynamics(base_link, out_f, out_tau);
        for (unsigned int i=0; i<limbdof; ++i){
            c_and_g(i) = manip->joint(i)->u;
            q_dot(i) = dq_backup[manip->joint(i)->jointId];
        }
        hrp::dvector time_vector = hrp::dvector::Constant(limbdof, 1.0/RTC_PERIOD);
        hrp::dmatrix time_matrix = time_vector.asDiagonal();
        inertia_dot = time_matrix * (gen_imat[ee_name] - old_gen_imat[ee_name]);
        beta = c_and_g - inertia_dot*q_dot;
        for (unsigned int i=0; i<limbdof; ++i){
            manip->joint(i)->ddq = dq_backup[manip->joint(i)->jointId];
            manip->joint(i)->dq = 0.0;
        }
        base_link->dvo.setZero();
        base_link->dw.setZero();  //assuming base_link is fixed! should be modified for humanoid
        out_f = hrp::Vector3::Zero(3);
        out_tau = hrp::Vector3::Zero(3);
        m_robot->calcInverseDynamics(base_link, out_f, out_tau);
        for (unsigned int i=0; i<limbdof; ++i){
            gen_mom[ee_name](i) = manip->joint(i)->u;
            // use command torque //seems good for simulation(choreonoid), which does not simulate joint friction
            //mot_tq(i) = tq_backup[manip->joint(i)->jointId];
            // use sensor torque //probably correct for torque-controlled real robot?
            mot_tq(i) = actual_torque_vector[manip->joint(i)->jointId];
        }
        if (!collision_detector_initialized[ee_name]){
            //old_gen_mom[ee_name] = gen_mom[ee_name];
            initial_gen_mom[ee_name] = gen_mom[ee_name];
            collision_detector_initialized[ee_name] = true;
        }

        accum_tau[ee_name] += mot_tq*RTC_PERIOD;
        // if(collision_uncheck_count[ee_name] > 0){
        //     accum_tau[ee_name] += resist_of_one_step_before[ee_name]*RTC_PERIOD;
        // }

        accum_beta[ee_name] += beta*RTC_PERIOD;
        accum_res[ee_name] += gen_mom_res[ee_name]*RTC_PERIOD;
        //calc residual (need filter?)
        gen_mom_res[ee_name]
            = gen_mom_observer_gain[ee_name]
            * (gen_mom[ee_name] - initial_gen_mom[ee_name]
               - (accum_tau[ee_name]-accum_beta[ee_name]+accum_res[ee_name])
               );

        for (int i=limbdof-1; i>=0; --i) {
            if( (std::abs(gen_mom_res[ee_name][i]) > m_lt_col_param[ee_name].collision_threshold[i]) && (collision_uncheck_count[ee_name] == 0) ){
                collision_uncheck_count[ee_name] = m_lt_col_param[ee_name].max_collision_uncheck_count;
                collision_link[ee_name] = manip->joint(i)->name;
                std::cout << std::endl;
                std::cout << "[LimbTorqueController] Collision Detected at " << ee_name << " joint " << i << std::endl;
                std::cout << "thresh = " << m_lt_col_param[ee_name].collision_threshold.transpose() << std::endl;
                std::cout << "now    = " << gen_mom_res[ee_name].transpose() << std::endl;
                std::cout << std::endl;
                break;
            }
        }
        // if (loop%100 == 0){
        //     // if (loop%1000==0){
        //     //     std::cout << "CollisionDetector: residual is ... "  << std::endl << gen_mom_res[ee_name] << std::endl;
        //     // }
        //     struct timeval nowtime;
        //     gettimeofday(&nowtime, NULL);
        //     debugmom << nowtime.tv_sec << "." << nowtime.tv_usec;
        //     debugtau << nowtime.tv_sec << "." << nowtime.tv_usec;
        //     debugbet << nowtime.tv_sec << "." << nowtime.tv_usec;
        //     debugares << nowtime.tv_sec << "." << nowtime.tv_usec;
        //     debugres << nowtime.tv_sec << "." << nowtime.tv_usec;
        //     for (unsigned int i=0; i<limbdof; ++i){
        //         debugmom << " " << gen_mom[ee_name](i);
        //         debugtau << " " << accum_tau[ee_name](i);
        //         debugbet << " " << accum_res[ee_name](i);
        //         debugares << " " << accum_res[ee_name](i);
        //         debugres << " " << gen_mom_res[ee_name](i);
        //     }
        //     debugmom << std::endl;
        //     debugtau << std::endl;
        //     debugbet << std::endl;
        //     debugares << std::endl;
        //     debugres << std::endl;
        // }
    }
    for (unsigned int i=0; i<dof; ++i){
        m_robot->joint(i)->dq = dq_backup[i];
        m_robot->joint(i)->ddq = ddq_backup[i];
        m_robot->joint(i)->u = tq_backup[i];
    }
}

void LimbTorqueController::calcGeneralizedInertiaMatrix(std::map<std::string, LTParam>::iterator it)
{
    std::string ee_name = it->first;
    LTParam& param = it->second;
    hrp::JointPathExPtr manip = param.manip;
    hrp::Vector3 omega, arm, omegaxarm;
    int limbdof = manip->numJoints();
    if(gen_imat_initialized[ee_name]){
        old_gen_imat[ee_name] = gen_imat[ee_name];
    }
    gen_imat[ee_name] = hrp::dmatrix::Zero(limbdof, limbdof);
    //calculate basic jacobian
    std::vector<hrp::dmatrix> basic_jacobians;
    basic_jacobians.resize(limbdof);
    for (int i=0; i<limbdof; ++i){
        //basic_jacobians[i] = Eigen::MatrixXd::Zero(6, limbdof);
        manip->joint(i)->wc = manip->joint(i)->p + manip->joint(i)->R*manip->joint(i)->c;
        basic_jacobians[i] =  hrp::dmatrix::Zero(6, limbdof);
        for (int j=0; j<=i; ++j){
            omega = manip->joint(j)->R * manip->joint(j)->a; //unit vector of joint axis direction
            arm = manip->joint(i)->wc - manip->joint(j)->p;
            omegaxarm = omega.cross(arm);
            basic_jacobians[i].col(j) << omegaxarm, omega;
        }
        gen_imat[ee_name] += basic_jacobians[i].transpose() * link_inertia_matrix[manip->joint(i)->jointId] * basic_jacobians[i];
    }
    if (!gen_imat_initialized[ee_name]){
        old_gen_imat[ee_name] = gen_imat[ee_name];
        gen_imat_initialized[ee_name] = true;
    }
}

void LimbTorqueController::CollisionHandler()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        std::string ee_name = it->first;
        CollisionParam& col_param = m_lt_col_param[ee_name];
        switch (col_param.handle_mode){
        case 0:
            break;
            //TODO: case 1
        case 2: //resist collision
            LTParam& ltc_param = it->second;
            hrp::JointPathExPtr manip = ltc_param.manip;
            if (ltc_param.is_active) {
                if (DEBUGP) {
                    std::cerr << "ここにデバッグメッセージを流す" << std::endl;
                }
                if(collision_uncheck_count[ee_name] > 0){
                    hrp::dvector collision_resistance_torque = - collision_resistance_gain[ee_name] * gen_mom_res[ee_name];
                    for (int i=0; i<manip->numJoints(); ++i){
                        manip->joint(i)->u += collision_resistance_torque(i);
                    }
                    resist_of_one_step_before[ee_name] = collision_resistance_torque;
                }
                if(collision_uncheck_count[ee_name] == 0){
                    resist_of_one_step_before[ee_name] = hrp::dvector::Zero(manip->numJoints());
                }
            }
            break;
        }
        it++;
    }
}

void LimbTorqueController::DebugOutput()
{
    if (loop%3 == 0){
        std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
        while (it != m_lt_param.end()){
            LTParam& param = it->second;
            std::string ee_name = it->first;
            CollisionParam& col_param = m_lt_col_param[ee_name];
            if (param.is_active) {
                hrp::JointPathExPtr manip = param.manip;
                int limbdof = manip->numJoints();
                struct timeval nowtime;
                gettimeofday(&nowtime, NULL);
                long int micro_time = nowtime.tv_sec*1e+6 + nowtime.tv_usec;
                if(log_type == 1){
                    *(debug_mom[ee_name]) << micro_time;
                    *(debug_actau[ee_name]) << micro_time;
                    *(debug_acbet[ee_name]) << micro_time;
                    *(debug_acres[ee_name]) << micro_time;
                    *(debug_res[ee_name]) << micro_time;
                    *(debug_reftq[ee_name]) << micro_time;
                    *(debug_f[ee_name]) << micro_time;
                    if(col_param.check_mode == 3){
                        *(debug_resdir[ee_name]) << micro_time;
                    }
                }
                else if(log_type == 2){
                    *(debug_ee_pocw[ee_name]) << micro_time;
                    *(debug_ee_vwcw[ee_name]) << micro_time;
                    *(debug_eect[ee_name]) << micro_time;
                    *(debug_nst[ee_name]) << micro_time;
                    *(debug_ee_poserror[ee_name]) << micro_time;
                    *(debug_ee_orierror[ee_name]) << micro_time;
                    *(debug_ee_velerror[ee_name]) << micro_time;
                    *(debug_ee_werror[ee_name]) << micro_time;
                    *(debug_reftq[ee_name]) << micro_time;
                    *(debug_acteevel[ee_name]) << micro_time;
                    *(debug_refeevel[ee_name]) << micro_time;
                    *(debug_acteew[ee_name]) << micro_time;
                    *(debug_refeew[ee_name]) << micro_time;
                    *(debug_dq[ee_name]) << micro_time;
                    *(debug_velest[ee_name]) << micro_time;
                    *(debug_velact[ee_name]) << micro_time;
                    *(debug_qest[ee_name]) << micro_time;
                    *(debug_qact[ee_name]) << micro_time;
                    *(debug_qref[ee_name]) << micro_time;
                }
                if(log_type == 1){
                    //calc external force
                    hrp::dvector external_force = hrp::dvector::Zero(6);
                    hrp::dmatrix inv_j(manip->numJoints(), 6);
                    hrp::calcPseudoInverse(act_ee_jacobian[ee_name].transpose(), inv_j);
                    external_force = inv_j * gen_mom_res[ee_name];
                    for (unsigned int i=0; i<limbdof; ++i){
                        *(debug_mom[ee_name]) << " " << (gen_mom[ee_name](i) - initial_gen_mom[ee_name](i));
                        *(debug_actau[ee_name]) << " " << accum_tau[ee_name](i);
                        *(debug_acbet[ee_name]) << " " << accum_beta[ee_name](i);
                        *(debug_acres[ee_name]) << " " << accum_res[ee_name](i);
                        *(debug_res[ee_name]) << " " << gen_mom_res[ee_name](i);
                        *(debug_reftq[ee_name]) << " " << manip->joint(i)->u;
                        if(col_param.check_mode == 3){
                            *(debug_resdir[ee_name]) << " " << resist_dir_torque[ee_name](i);
                        }
                    }
                    for (unsigned int i=0; i<6; ++i){
                        *(debug_f[ee_name]) << " " << external_force(i);
                    }
                    *(debug_mom[ee_name]) << std::endl;
                    *(debug_actau[ee_name]) << std::endl;
                    *(debug_acbet[ee_name]) << std::endl;
                    *(debug_acres[ee_name]) << std::endl;
                    *(debug_res[ee_name]) << std::endl;
                    *(debug_reftq[ee_name]) << std::endl;
                    *(debug_f[ee_name]) << std::endl;
                    if(col_param.check_mode == 3){
                        *(debug_resdir[ee_name]) << std::endl;
                    }
                }
                else if(log_type == 2){
                    for (int i=0; i<3; i++){
                        *(debug_ee_poserror[ee_name]) << " " << ee_pos_error[ee_name](i);
                        *(debug_ee_orierror[ee_name]) << " " << ee_ori_error[ee_name](i);
                        *(debug_ee_velerror[ee_name]) << " " << ee_vel_error[ee_name](i);
                        *(debug_ee_werror[ee_name]) << " " << ee_w_error[ee_name](i);
                        *(debug_acteevel[ee_name]) << " " << act_ee_vel[ee_name](i);
                        *(debug_refeevel[ee_name]) << " " << ref_ee_vel[ee_name](i);
                        *(debug_acteew[ee_name]) << " " << act_ee_w[ee_name](i);
                        *(debug_refeew[ee_name]) << " " << ref_ee_w[ee_name](i);
                    }
                    for (int i=0; i<6; i++){
                        *(debug_ee_pocw[ee_name]) << " " << ee_pos_ori_comp_wrench[ee_name](i);
                        *(debug_ee_vwcw[ee_name]) << " " << ee_vel_w_comp_wrench[ee_name](i);
                    }
                    for (int i=0; i<limbdof; i++){
                        *(debug_eect[ee_name]) << " " << ee_compensation_torque[ee_name](i);
                        *(debug_nst[ee_name]) << " " << null_space_torque[ee_name](i);
                        *(debug_reftq[ee_name]) << " " << manip->joint(i)->u;
                        *(debug_dq[ee_name]) << " " << dq_for_each_arm[ee_name](i);
                        *(debug_velest[ee_name]) << " " << estimated_reference_velocity(manip->joint(i)->jointId);
                        *(debug_velact[ee_name]) << " " << m_robot->joint(manip->joint(i)->jointId)->dq;
                        *(debug_qest[ee_name]) << " " << log_est_q(manip->joint(i)->jointId);
                        *(debug_qact[ee_name]) << " " << log_act_q(manip->joint(i)->jointId);
                        *(debug_qref[ee_name]) << " " << log_ref_q(manip->joint(i)->jointId);
                    }
                    *(debug_ee_pocw[ee_name]) << std::endl;
                    *(debug_ee_vwcw[ee_name]) << std::endl;
                    *(debug_eect[ee_name]) << std::endl;
                    *(debug_nst[ee_name]) << std::endl;
                    *(debug_ee_poserror[ee_name]) << std::endl;
                    *(debug_ee_orierror[ee_name]) << std::endl;
                    *(debug_ee_velerror[ee_name]) << std::endl;
                    *(debug_ee_werror[ee_name]) << std::endl;
                    *(debug_reftq[ee_name]) << std::endl;
                    *(debug_acteevel[ee_name]) << std::endl;
                    *(debug_refeevel[ee_name]) << std::endl;
                    *(debug_acteew[ee_name]) << std::endl;
                    *(debug_refeew[ee_name]) << std::endl;
                    *(debug_dq[ee_name]) << std::endl;
                    *(debug_velest[ee_name]) << std::endl;
                    *(debug_velact[ee_name]) << std::endl;
                    *(debug_qest[ee_name]) << std::endl;
                    *(debug_qact[ee_name]) << std::endl;
                    *(debug_qref[ee_name]) << std::endl;
                }
            }
            it++;
        }
    }
}

//simple, rough collision detection method based on measured torque
void LimbTorqueController::CollisionDetector2(std::map<std::string, LTParam>::iterator it)
{
    std::string ee_name = it->first;
    LTParam& param = it->second;
    if (param.is_active) {
        if (DEBUGP) {
            std::cerr << "ここにデバッグメッセージを流す" << std::endl;
        }
        hrp::JointPathExPtr manip = param.manip;
        int limbdof = manip->numJoints();
        hrp::dvector mot_tq(limbdof);
        for (unsigned int i=0; i<limbdof; ++i){
            mot_tq(i) = manip->joint(i)->u;
        }
        gen_mom_res[ee_name] = mot_tq - actual_torque_vector;
        for (int i=limbdof-1; i>=0; --i) {
            if( (std::abs(gen_mom_res[ee_name][i]) > m_lt_col_param[ee_name].collision_threshold[i]) && (collision_uncheck_count[ee_name] == 0) ){
                collision_uncheck_count[ee_name] = m_lt_col_param[ee_name].max_collision_uncheck_count;
                collision_link[ee_name] = manip->joint(i)->name;
                std::cout << "[LimbTorqueController] Collision Detected at " << ee_name << " joint " << i << std::endl;
                std::cout << "thresh = " << m_lt_col_param[ee_name].collision_threshold.transpose() << std::endl;
                std::cout << "now    = " << gen_mom_res[ee_name].transpose() << std::endl;
                break;
            }
        }
    }
}

//i_name_: base file path of log
//i_logname_: what log to take; "collision" or "operational"
bool LimbTorqueController::startLog(const std::string& i_name_, const std::string& i_logname_)
{
    Guard guard(m_mutex);
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        if (param.is_active) {
            std::string ee_name = it->first;
            struct stat st;
            std::string basepath = i_name_ + std::string("/LimbTorqueControllerDebug/");
            if(stat(basepath.c_str(), &st) != 0){
                mkdir(basepath.c_str(), 0775);
            }
            std::string logpath = basepath + ee_name + std::string("_");
            if (boost::iequals(i_logname_, "collision")){
                debug_mom[ee_name] = new std::ofstream((logpath + std::string("gen_mom.dat")).c_str());
                debug_actau[ee_name] = new std::ofstream((logpath + std::string("ac_tau.dat")).c_str());
                debug_acbet[ee_name] = new std::ofstream((logpath + std::string("ac_beta.dat")).c_str());
                debug_acres[ee_name] = new std::ofstream((logpath + std::string("ac_res.dat")).c_str());
                debug_res[ee_name] = new std::ofstream((logpath +std::string("res.dat")).c_str());
                debug_reftq[ee_name] = new std::ofstream((logpath + std::string("ref_tq.dat")).c_str());
                debug_f[ee_name] = new std::ofstream((logpath + std::string("ext_f.dat")).c_str());
                debug_resdir[ee_name] = new std::ofstream((logpath + std::string("resdir.dat")).c_str());
                log_type = 1;
                spit_log = true;
                std::cout << "[ltc] startLog succeed: open log stream for collision detection!!" << std::endl;
            }
            else if (boost::iequals(i_logname_, "operational")){
                debug_ee_pocw[ee_name] = new std::ofstream((logpath + std::string("ee_pos_ori_wrench.dat")).c_str());
                debug_ee_vwcw[ee_name] = new std::ofstream((logpath + std::string("ee_vel_w_wrench.dat")).c_str());
                debug_eect[ee_name] = new std::ofstream((logpath + std::string("ee_comp_torque.dat")).c_str());
                debug_nst[ee_name] = new std::ofstream((logpath + std::string("null_space_torque.dat")).c_str());
                debug_ee_poserror[ee_name] = new std::ofstream((logpath + std::string("ee_pos_error.dat")).c_str());
                debug_ee_orierror[ee_name] = new std::ofstream((logpath + std::string("ee_ori_error.dat")).c_str());
                debug_ee_velerror[ee_name] = new std::ofstream((logpath + std::string("ee_vel_error.dat")).c_str());
                debug_ee_werror[ee_name] = new std::ofstream((logpath + std::string("ee_w_error.dat")).c_str());
                debug_reftq[ee_name] = new std::ofstream((logpath + std::string("ref_tq.dat")).c_str());
                debug_acteevel[ee_name] = new std::ofstream((logpath  + std::string("act_ee_vel.dat")).c_str());
                debug_refeevel[ee_name] = new std::ofstream((logpath  + std::string("ref_ee_vel.dat")).c_str());
                debug_acteew[ee_name] = new std::ofstream((logpath  + std::string("act_ee_w.dat")).c_str());
                debug_refeew[ee_name] = new std::ofstream((logpath  + std::string("ref_ee_w.dat")).c_str());
                debug_dq[ee_name] = new std::ofstream((logpath  + std::string("dq.dat")).c_str());
                debug_velest[ee_name] = new std::ofstream((logpath  + std::string("velest.dat")).c_str());
                debug_velact[ee_name] = new std::ofstream((logpath  + std::string("velact.dat")).c_str());
                debug_qest[ee_name] = new std::ofstream((logpath  + std::string("qest.dat")).c_str());
                debug_qact[ee_name] = new std::ofstream((logpath  + std::string("qact.dat")).c_str());
                debug_qref[ee_name] = new std::ofstream((logpath  + std::string("qref.dat")).c_str());
                log_type = 2;
                spit_log = true;
                std::cout << "[ltc] startLog succeed: open log stream for operational space control!!" << std::endl;
            }
            else{
                std::cerr << "[ltc] startLog error: allowed logname are: 'collision' or 'operational'. You commanded " << i_logname_ << "." << std::endl;
                return false;
            }
        }
        it++;
    }
    return true;
}

bool LimbTorqueController::stopLog()
{
    Guard guard(m_mutex);
    spit_log = false;
    log_type = 0;
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        if (param.is_active) { //TODO: delete ofstream when LTC goes inactive during taking log?
            std::string ee_name = it->first;
            if(log_type == 1){
                delete debug_mom[ee_name];
                delete debug_actau[ee_name];
                delete debug_acbet[ee_name];
                delete debug_acres[ee_name];
                delete debug_res[ee_name];
                delete debug_reftq[ee_name];
                delete debug_f[ee_name];
                delete debug_resdir[ee_name];
            }
            else if(log_type == 2){
                delete debug_ee_pocw[ee_name];
                delete debug_ee_vwcw[ee_name];
                delete debug_eect[ee_name];
                delete debug_nst[ee_name];
                delete debug_ee_poserror[ee_name];
                delete debug_ee_orierror[ee_name];
                delete debug_ee_velerror[ee_name];
                delete debug_ee_werror[ee_name];
                delete debug_reftq[ee_name];
                delete debug_acteevel[ee_name];
                delete debug_refeevel[ee_name];
                delete debug_acteew[ee_name];
                delete debug_refeew[ee_name];
                delete debug_dq[ee_name];
                delete debug_velest[ee_name];
                delete debug_velact[ee_name];
                delete debug_qest[ee_name];
                delete debug_qact[ee_name];
                delete debug_qref[ee_name];
            }
        }
        it++;
    }
    std::cout << "[ltc] successfully stop log!!" << std::endl;
    return true;
}

//modification of generalized momentum method, including direction filtering
void LimbTorqueController::CollisionDetector3(std::map<std::string, LTParam>::iterator it)
{
    int dof = m_robot->numJoints();
    hrp::dvector dq_backup(dof), ddq_backup(dof), tq_backup(dof);
    for (unsigned int i=0; i<dof; ++i){
        dq_backup[i] = m_robot->joint(i)->dq;
        ddq_backup[i] = m_robot->joint(i)->ddq;
        tq_backup[i] = m_robot->joint(i)->u;
    }
    std::string ee_name = it->first;
    LTParam& param = it->second;
    if (param.is_active) {
        if (DEBUGP) {
            std::cerr << "ここにデバッグメッセージを流す" << std::endl;
        }
        calcGeneralizedInertiaMatrix(it);
        hrp::JointPathExPtr manip = param.manip;
        int limbdof = manip->numJoints();
        hrp::Link* base_link = manip->baseLink();
        hrp::dmatrix base_rot = base_link->R;
        hrp::Vector3 out_f, out_tau;
        out_f = hrp::Vector3::Zero(3);
        out_tau = hrp::Vector3::Zero(3);
        base_link->dvo = base_rot * param.gravitational_acceleration;
        base_link->dw.setZero();  //assuming base_link is fixed! should be modified for humanoid
        for (unsigned int i=0; i<limbdof; ++i){
            manip->joint(i)->ddq = 0.0;
        }
        hrp::dvector c_and_g(limbdof), q_dot(limbdof), beta(limbdof), mot_tq(limbdof);
        hrp::dmatrix  inertia_dot;
        m_robot->calcInverseDynamics(base_link, out_f, out_tau);
        for (unsigned int i=0; i<limbdof; ++i){
            c_and_g(i) = manip->joint(i)->u;
            q_dot(i) = dq_backup[manip->joint(i)->jointId];
        }
        hrp::dvector time_vector = hrp::dvector::Constant(limbdof, 1.0/RTC_PERIOD);
        hrp::dmatrix time_matrix = time_vector.asDiagonal();
        inertia_dot = time_matrix * (gen_imat[ee_name] - old_gen_imat[ee_name]);
        beta = c_and_g - inertia_dot*q_dot;
        for (unsigned int i=0; i<limbdof; ++i){
            manip->joint(i)->ddq = dq_backup[manip->joint(i)->jointId];
            manip->joint(i)->dq = 0.0;
        }
        base_link->dvo.setZero();
        base_link->dw.setZero();  //assuming base_link is fixed! should be modified for humanoid
        out_f = hrp::Vector3::Zero(3);
        out_tau = hrp::Vector3::Zero(3);
        m_robot->calcInverseDynamics(base_link, out_f, out_tau);
        for (unsigned int i=0; i<limbdof; ++i){
            gen_mom[ee_name](i) = manip->joint(i)->u;
            // use command torque //seems good for simulation(choreonoid), which does not simulate joint friction
            mot_tq(i) = tq_backup[manip->joint(i)->jointId];
            // use sensor torque //probably correct for torque-controlled real robot?
            mot_tq(i) = actual_torque_vector[manip->joint(i)->jointId];
        }
        if (!collision_detector_initialized[ee_name]){
            //old_gen_mom[ee_name] = gen_mom[ee_name];
            initial_gen_mom[ee_name] = gen_mom[ee_name];
            collision_detector_initialized[ee_name] = true;
        }

        accum_tau[ee_name] += mot_tq*RTC_PERIOD;
        // if(collision_uncheck_count[ee_name] > 0){
        //     accum_tau[ee_name] += resist_of_one_step_before[ee_name]*RTC_PERIOD;
        // }

        accum_beta[ee_name] += beta*RTC_PERIOD;
        accum_res[ee_name] += gen_mom_res[ee_name]*RTC_PERIOD;
        //calc residual (need filter?)
        gen_mom_res[ee_name]
            = gen_mom_observer_gain[ee_name]
            * (gen_mom[ee_name] - initial_gen_mom[ee_name]
               - (accum_tau[ee_name]-accum_beta[ee_name]+accum_res[ee_name])
               );

        //reduce residual to just one direction
        hrp::calcPseudoInverse(act_ee_jacobian[ee_name].transpose(), inv_ee_jacobian_t[ee_name]);
        hrp::dvector external_force(6,1);
        hrp::Link* temp_ee_target = m_robot->link(param.target_name);
        hrp::Matrix33 temp_ee_rot = temp_ee_target->R * ee_map[ee_name].localR;
        hrp::dmatrix temp_ee_rot_double(6,6); //for transforming both translation and rotation at a time
        temp_ee_rot_double
            << temp_ee_rot, hrp::Matrix33::Zero(),
            hrp::Matrix33::Zero(), hrp::Matrix33::Zero();
        //std::cout << temp_ee_rot_double << std::endl;
        hrp::dvector world_resist_direction = temp_ee_rot_double * resist_direction[ee_name];
        //std::cout << "resist direction for " << ee_name << " is " << world_resist_direction.transpose() << std::endl;
        external_force = temp_ee_rot_double.transpose() * inv_ee_jacobian_t[ee_name]*gen_mom_res[ee_name]; //in end-effector local frame
        resist_dir_torque[ee_name] = act_ee_jacobian[ee_name].transpose() * (world_resist_direction.dot(inv_ee_jacobian_t[ee_name]*gen_mom_res[ee_name]) * world_resist_direction);
        // if(loop%300==0){
        //     std::cout << std::endl;
        //     std::cout << "[CollisionDetector] external force for " << ee_name << " is: " << external_force.transpose() << std::endl;
        //     std::cout << std::endl;
        // }

        for (int i=limbdof-1; i>=0; --i) {
            if( (std::abs(resist_dir_torque[ee_name][i]) > m_lt_col_param[ee_name].collision_threshold[i]) && (collision_uncheck_count[ee_name] == 0) ){
                collision_uncheck_count[ee_name] = m_lt_col_param[ee_name].max_collision_uncheck_count;
                collision_link[ee_name] = manip->joint(i)->name;
                std::cout << std::endl;
                std::cout << "[LimbTorqueController] Collision Detected at " << ee_name << " joint " << i << std::endl;
                std::cout << "thresh = " << m_lt_col_param[ee_name].collision_threshold.transpose() << std::endl;
                std::cout << "now    = " << resist_dir_torque[ee_name].transpose() << std::endl;
                std::cout << "corresponding external force = " << external_force.transpose() << std::endl;
                std::cout << std::endl;
                break;
            }
        }
    }
    for (unsigned int i=0; i<dof; ++i){
        m_robot->joint(i)->dq = dq_backup[i];
        m_robot->joint(i)->ddq = ddq_backup[i];
        m_robot->joint(i)->u = tq_backup[i];
    }
}

void LimbTorqueController::calcEECompensation()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        std::string ee_name = it->first;
        LTParam& ref_param = m_ref_lt_param[ee_name];
        if (param.is_active) {
            if (DEBUGP) {
                std::cerr << "ここにデバッグメッセージを流す" << std::endl;
            }
            hrp::JointPathExPtr manip = param.manip;
            hrp::JointPathExPtr ref_manip = ref_param.manip;
            int limb_dof = manip->numJoints();
            hrp::Link* target = m_robot->link(ee_map[it->first].target_name);
            hrp::Link* act_el = m_robot->link(ee_map[ee_name].target_name);
            hrp::Link* ref_el = m_robotRef->link(ee_map[ee_name].target_name);
            //hrp::Matrix33 eeR = act_el->R * ee_map[ee_name].localR;
            hrp::Matrix33 eeR = ref_el->R * ee_map[ee_name].localR;
            // calculate position, orientation error and compensation force for it
            //ee_pos_error[ee_name] = (ref_el->p + ref_el->R*ee_map[ee_name].localPos) - (act_el->p + act_el->R*ee_map[ee_name].localPos);
            current_act_ee_pos[ee_name] = act_el->p + act_el->R*ee_map[ee_name].localPos;
            current_ref_ee_pos[ee_name] = ref_el->p + ref_el->R*ee_map[ee_name].localPos;
            ee_pos_error[ee_name] = current_ref_ee_pos[ee_name] - current_act_ee_pos[ee_name];
            //ee_pos_comp_force[ee_name] = param.ee_pgain_p * ee_pos_error[ee_name];
            ee_pos_comp_force[ee_name] = eeR * param.ee_pgain_p * eeR.transpose() * ee_pos_error[ee_name];

            // orientation feedback method 1 (seems to work well)
            Eigen::Quaternion<double> act_ee_quat(act_el->R*ee_map[ee_name].localR), ref_ee_quat(ref_el->R*ee_map[ee_name].localR);
            // check and correct jump of quaternion
            if((ref_ee_quat.conjugate()*act_ee_quat).w() < 0){
                Eigen::Quaternion<double> negated_ref(-ref_ee_quat.w(), -ref_ee_quat.x(), -ref_ee_quat.y(), -ref_ee_quat.z());
                ref_ee_quat = negated_ref;
            }
            // J.S. Yuan, "Closed-loop manipulator control using quaternion feedback," in IEEE Journal on Robotics and Automation, vol.4, no.4, pp.434-440, Aug. 1988.
            ee_ori_error[ee_name] = ref_ee_quat.w()*act_ee_quat.vec() - act_ee_quat.w()*ref_ee_quat.vec() + ref_ee_quat.vec().cross(act_ee_quat.vec());
            ee_ori_comp_moment[ee_name] = - eeR * param.ee_pgain_r * eeR.transpose() * ee_ori_error[ee_name];
            // orientation feedback method 2 (somehow control go wrong in some cases)
            // F. Caccavale et al., "Six-DOF Impedance Control of Dual-Arm Cooperative Manipulators," in IEEE/ASME Transactions on Mechatronics, vol.13, no.5, pp.576-586, Oct. 2008.
            // Eigen::Quaternion<double> ee_ori_error_quat = ref_ee_quat.conjugate() * act_ee_quat;
            // ee_ori_comp_moment[ee_name] = - 2.0 * (ee_ori_error_quat.w() * hrp::Matrix33::Identity() + hrp::hat(ee_ori_error_quat.vec())) * (eeR * param.ee_pgain_r * eeR.transpose()) * ee_ori_error_quat.vec();

            if(loop%1000==0){
                std::cout << "EE Compensation Debug for " << ee_name << " start"  << std::endl;
                std::cout << "pos error is: " << std::endl << ee_pos_error[ee_name].transpose() << std::endl;
                std::cout << "orientation error is: " << std::endl << ee_ori_error[ee_name].transpose() << std::endl;
            }

            ee_pos_ori_comp_wrench[ee_name] << ee_pos_comp_force[ee_name], ee_ori_comp_moment[ee_name];
            // calculate translational velocity, angular velocity error
            current_act_ee_rot[ee_name] = act_el->R * ee_map[ee_name].localR;
            current_ref_ee_rot[ee_name] = ref_el->R * ee_map[ee_name].localR;
            // set current paramters to previous parameters (run only at first)
            if(!oscontrol_initialized[ee_name]){
                ee_vel_filter[ee_name].fill(current_act_ee_pos[ee_name]);
                ee_w_filter[ee_name].fill(current_act_ee_rot[ee_name]);
                prev_ref_ee_pos[ee_name] = current_ref_ee_pos[ee_name];
                prev_ref_ee_rot[ee_name] = current_ref_ee_rot[ee_name];
                oscontrol_initialized[ee_name] = true;
            }
            // set current position, rotation to the head of filters
            ee_vel_filter[ee_name].push(current_act_ee_pos[ee_name]);
            ee_w_filter[ee_name].push(current_act_ee_rot[ee_name]);
            // calculate ee velocity
            act_ee_vel[ee_name] = ee_vel_filter[ee_name].get_velocity();
            ref_ee_vel[ee_name] = (current_ref_ee_pos[ee_name] - prev_ref_ee_pos[ee_name]) / RTC_PERIOD;
            ee_vel_error[ee_name] = ref_ee_vel[ee_name] - act_ee_vel[ee_name];
            // callulate ee angular velocity
            hrp::Matrix33 act_ee_w_omega_mat = ee_w_filter[ee_name].get_velocity() * current_act_ee_rot[ee_name].transpose();
            hrp::Matrix33 ref_ee_w_omega_mat = ((current_ref_ee_rot[ee_name] - prev_ref_ee_rot[ee_name])/RTC_PERIOD) * current_ref_ee_rot[ee_name].transpose();
            act_ee_w[ee_name] <<
                ((act_ee_w_omega_mat(2,1) - act_ee_w_omega_mat(1,2)) / 2),
                ((act_ee_w_omega_mat(0,2) - act_ee_w_omega_mat(2,0)) / 2),
                ((act_ee_w_omega_mat(1,0) - act_ee_w_omega_mat(0,1)) / 2);
            ref_ee_w[ee_name] <<
                ((ref_ee_w_omega_mat(2,1) - ref_ee_w_omega_mat(1,2)) / 2),
                ((ref_ee_w_omega_mat(0,2) - ref_ee_w_omega_mat(2,0)) / 2),
                ((ref_ee_w_omega_mat(1,0) - ref_ee_w_omega_mat(0,1)) / 2);
            ee_w_error[ee_name] = ref_ee_w[ee_name] - act_ee_w[ee_name];
            // multiply gain to error
            // ee_vel_comp_force[ee_name] = param.ee_dgain_p * ee_vel_error[ee_name];
            // ee_w_comp_moment[ee_name] = param.ee_dgain_r * ee_w_error[ee_name];
            ee_vel_comp_force[ee_name] = eeR * param.ee_dgain_p * eeR.transpose() * ee_vel_error[ee_name];
            ee_w_comp_moment[ee_name] = eeR * param.ee_dgain_r * eeR.transpose() * ee_w_error[ee_name];
            ee_vel_w_comp_wrench[ee_name] << ee_vel_comp_force[ee_name], ee_w_comp_moment[ee_name];
            // map wrench to joint torque
            ee_compensation_torque[ee_name] = act_ee_jacobian[ee_name].transpose() * (ee_pos_ori_comp_wrench[ee_name] + ee_vel_w_comp_wrench[ee_name]);
            // save current position, rotation as previous
            prev_ref_ee_pos[ee_name] = current_ref_ee_pos[ee_name];
            prev_ref_ee_rot[ee_name] = current_ref_ee_rot[ee_name];
            // debug output
            if(loop%1000==0){
                std::cout << "EE pgain force = " << ee_pos_ori_comp_wrench[ee_name].transpose() << std::endl;
                std::cout << "EE vel error = " << ee_vel_error[ee_name].transpose() << std::endl;
                std::cout << "EE w error = " << ee_w_error[ee_name].transpose() << std::endl;
                std::cout << "EE dgain force = " << ee_vel_w_comp_wrench[ee_name].transpose() << std::endl;
                std::cout << "EE Compensation Debug for " << ee_name << " end"  << std::endl;
            }
            // copy torque value to m_robot
            for (int i=0; i<limb_dof; i++){
                m_robot->joint(manip->joint(i)->jointId)->u += ee_compensation_torque[ee_name](i);
            }
        }
        ++it;
    }
}

void LimbTorqueController::calcNullJointDumping()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        std::string ee_name = it->first;
        LTParam& ref_param = m_ref_lt_param[ee_name];
        if (param.is_active) {
            if (DEBUGP) {
                std::cerr << "ここにデバッグメッセージを流す" << std::endl;
            }
            hrp::JointPathExPtr manip = param.manip;
            hrp::JointPathExPtr ref_manip = ref_param.manip;
            int limb_dof = manip->numJoints();
            hrp::dvector q_error(limb_dof), dq_error(limb_dof);
            for (int i=0; i<limb_dof; i++) {
                q_error(i) = ref_manip->joint(i)->q - manip->joint(i)->q;
                dq_error(i) = ref_manip->joint(i)->dq - manip->joint(i)->dq;
                dq_for_each_arm[ee_name][i] = manip->joint(i)->dq; //for debug log
            }
            hrp::dmatrix Jnull(limb_dof, limb_dof);
            manip->calcJacobianInverseNullspace(act_ee_jacobian[ee_name], inv_ee_jacobian[ee_name], Jnull); // including min/max avoidance
            null_space_torque[ee_name] = Jnull * (param.pgain * q_error + param.dgain * dq_error);
            if(loop%1000==0){
                std::cout << "pos error = " << q_error.transpose() << std::endl;
                std::cout << "null_tau  = " << null_space_torque[ee_name].transpose() << std::endl;
            }
            for (int i=0; i<limb_dof; i++){
                m_robot->joint(manip->joint(i)->jointId)->u += null_space_torque[ee_name](i);
            }
        }
        it++;
    }
}

// Estimate Reference Velocity
void LimbTorqueController::estimateRefVel()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        std::string ee_name = it->first;
        LTParam& ref_param = m_ref_lt_param[ee_name];
        if (param.is_active) {
            if (DEBUGP) {
                std::cerr << "ここにデバッグメッセージを流す" << std::endl;
            }
            hrp::JointPathExPtr manip = param.manip;
            hrp::JointPathExPtr ref_manip = ref_param.manip;
            int limbdof = manip->numJoints();
            hrp::dvector impedance_vel(limbdof); //reference velocity generated by impedance(ee compensation)
            Eigen::Matrix<double, 6, 1> ee_ref_vel;
            hrp::Vector3 ee_vel_p, ee_vel_r;
            // TODO: these calculations assume isotropic PD gains!
            ee_vel_p = ( param.ee_dgain_p(0,0) * ee_vel_error[ee_name]
                         + param.ee_pgain_p(0,0) * ee_pos_error[ee_name] * RTC_PERIOD)
                / ( param.ee_dgain_p(0,0) + param.ee_pgain_p(0,0) * RTC_PERIOD );
            ee_vel_r = ( param.ee_dgain_r(0,0) * ee_w_error[ee_name]
                         + param.ee_pgain_r(0,0) * ee_ori_error[ee_name] * RTC_PERIOD)
                / ( param.ee_dgain_r(0,0) + param.ee_pgain_r(0,0) * RTC_PERIOD );
            ee_ref_vel << ee_vel_p, ee_vel_r;
            imp_now[ee_name] = - inv_ee_jacobian[ee_name] * ee_ref_vel; // is sign correct?
            if(!velest_initialized[ee_name]){
                for(int i=0; i<limbdof; i++){
                    int jid = manip->joint(i)->jointId;
                    velest_prev[ee_name](i) = m_robotRef->joint(jid)->dq;
                    velest_prevprev[ee_name](i) = m_robotRef->joint(jid)->dq;
                    imp_prev[ee_name](i) = imp_now[ee_name](i);
                    imp_prevprev[ee_name](i) = imp_now[ee_name](i);
                    velest_initialized[ee_name] = true;
                }
            }
            for(int i=0; i<limbdof; i++){
                int jid = manip->joint(i)->jointId;
                velest_now[ee_name](i) =
                    - iir_b1 * velest_prev[ee_name](i)
                    - iir_b2 * velest_prevprev[ee_name](i)
                    + iir_a0 * imp_now[ee_name](i)
                    + iir_a1 * imp_prev[ee_name](i)
                    + iir_a2 * imp_prevprev[ee_name](i);
                estimated_reference_velocity(jid) += velest_now[ee_name](i); // estimated_vel = (ref_vel calculated from difference of reference angle vector) + (ref_vel estimated from impedance force)
                imp_prevprev[ee_name](i) = imp_prev[ee_name](i);
                imp_prev[ee_name](i) = imp_now[ee_name](i);
                velest_prevprev[ee_name](i) = velest_prev[ee_name](i);
                velest_prev[ee_name](i) = velest_now[ee_name](i);
            }
            //TODO: what if start and stop switches frequently?
            //TODO: what if start or stop overwriting during moving joints?
            if(overwrite_refangle[ee_name]){
                if(stop_overwriting_q_transition_count[ee_name] == 0){
                    for(int i=0; i<limbdof; i++){
                        int jid = manip->joint(i)->jointId;
                        m_robotRef->joint(jid)->q = overwritten_qRef_prev(jid) + estimated_reference_velocity(jid) * RTC_PERIOD; // in iob, ref_vel is calculated as ('reference joint angle' - 'previous reference joint angle') / dt
                    }
                }else{ // transition to stop overwriting
                    for(int i=0; i<limbdof; i++){
                        int jid = manip->joint(i)->jointId;
                        transition_velest(jid) = (m_robotRef->joint(jid)->q - overwritten_qRef_prev(jid)) / (stop_overwriting_q_transition_count[ee_name] * RTC_PERIOD); // in one step
                        m_robotRef->joint(jid)->q = overwritten_qRef_prev(jid) + transition_velest(jid) * RTC_PERIOD;
                    }
                    stop_overwriting_q_transition_count[ee_name]--;
                    if(stop_overwriting_q_transition_count[ee_name] == 0){
                        overwrite_refangle[ee_name] = false;
                    }
                }
            }
        }
        it++;
    }
    // set previous overwritten joint angles
    for(int i=0; i<m_robotRef->numJoints(); i++){
        overwritten_qRef_prev(i) = m_robotRef->joint(i)->q;
        log_est_q(i) = m_robotRef->joint(i)->q;
    }
}

// start overwriting qRef with qRef+estimated_ref_vel * dt
// do not call during position control
bool LimbTorqueController::startRefVelEstimation(const std::string& i_name_)
{
    Guard guard(m_mutex);
    std::string name = std::string(i_name_);
    if ( m_lt_param.find(name) == m_lt_param.end() ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find end effector named [" << name << "]" << std::endl;
        return false;
    }
    overwrite_refangle[name] = true;
    std::cout << "[" << m_profile.instance_name << "] start overwriting ref angle of " << name << " with estimated reference velocity!" << std::endl;
    return true;
}

// stop overwriting qRef, using transition. do not use normally?
bool LimbTorqueController::stopRefVelEstimation(const std::string& i_name_)
{
    Guard guard(m_mutex);
    std::string name = std::string(i_name_);
    if ( m_lt_param.find(name) == m_lt_param.end() ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find end effector named [" << name << "]" << std::endl;
        return false;
    }
    stop_overwriting_q_transition_count[name] = max_stop_overwriting_q_transition_count;
    std::cout << "[" << m_profile.instance_name << "] stop overwriting ref angle of " << name << " with estimated reference velocity!" << std::endl;
    return true;
}
