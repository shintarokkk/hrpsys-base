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
      m_emergencySignalOut("emergencySignal", m_emergencySignal),
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
    addOutPort("emergencySignal", m_emergencySignalOut);

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
    m_ref_force.resize(nforce);
    m_ref_forceOut.resize(nforce);
    std::cerr << "[" << m_profile.instance_name << "] force sensor ports" << std::endl;
    for (unsigned int i=0; i<nforce; i++){
        // actual inport
        m_forceIn[i] = new RTC::InPort<RTC::TimedDoubleSeq>(fsensor_names[i].c_str(), m_force[i]);
        m_force[i].data.length(6);
        registerInPort(fsensor_names[i].c_str(), *m_forceIn[i]);
        std::cerr << "[" << m_profile.instance_name << "]   name = " << fsensor_names[i] << std::endl;
        // reference outport
        m_ref_force[i].data.length(6);
        for (unsigned int j=0; j<6; j++){
            m_ref_force[i].data[j] = 0.0;
        }
        m_ref_forceOut[i] = new RTC::OutPort<RTC::TimedDoubleSeq>(std::string("ref_"+fsensor_names[i]+"Out").c_str(), m_ref_force[i]);
        registerOutPort(std::string("ref_"+fsensor_names[i]+"Out").c_str(), *m_ref_forceOut[i]);
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
            eet.ee_index = i;
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
        p.is_active = false;
        p.sensor_name = sensor_name;
        p_ref.sensor_name = sensor_name;
        p.ee_name = ee_name;
        p.target_name = ee_map[ee_name].target_name;
        //現状すべてのlimbに同じ(最初の)ゲインを設定している
        p.pgain = default_pgain[0];  p_ref.pgain = default_pgain[0];
        p.dgain = default_dgain[0];  p_ref.dgain = default_dgain[0];
        p.gravitational_acceleration = hrp::Vector3(0.0, 0.0, 9.80665); //TODO: set from outside
        p_ref.gravitational_acceleration = hrp::Vector3(0.0, 0.0, 9.80665);
        p.amode = IDLE_NORMAL;
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
        p.task_succeed = false;
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
        //for ee compensation
        act_ee_jacobian[ee_name].resize(6, p.manip->numJoints());
        inv_act_ee_jacobian[ee_name].resize(p.manip->numJoints(), 6);
        null_act_ee_jacobian[ee_name].resize(p.manip->numJoints(), p.manip->numJoints());
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
        act_eepos[ee_name] = hrp::Vector3::Zero();
        ref_eepos[ee_name] = hrp::Vector3::Zero();
        //prev_ref_eepos[ee_name] = hrp::Vector3::Zero();
        act_ee_vel[ee_name] = hrp::Vector3::Zero();
        ref_ee_vel[ee_name] = hrp::Vector3::Zero();
        act_ee_w[ee_name] = hrp::Vector3::Zero();
        ref_ee_w[ee_name] = hrp::Vector3::Zero();
        act_eeR[ee_name] = hrp::Matrix33::Identity();
        ref_eeR[ee_name] = hrp::Matrix33::Identity();
        prev_ref_eeR[ee_name] = hrp::Matrix33::Identity();
        ee_vel_error[ee_name] = hrp::dvector::Zero(6);
        ee_w_error[ee_name] = hrp::dvector::Zero(6);
        RMSfilter<hrp::Vector3> temp_vel_filter(5, RTC_PERIOD);
        ee_vel_filter[ee_name] = temp_vel_filter;
        ref_ee_vel_filter[ee_name] = temp_vel_filter;
        RMSfilter<hrp::Matrix33> temp_w_filter(5, RTC_PERIOD);
        ee_w_filter[ee_name] = temp_w_filter;
        ref_ee_w_filter[ee_name] = temp_w_filter;
        geteestates_initialized[ee_name] = false;
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
        eeest_initialized[ee_name] = false;
        //for error check & mode selection
        TaskDescription td;
        TaskState ts;
        td.type = MOVE_POS;
        td.dual = false;
        td.velocity_check_dir = hrp::Vector3(1.0, 0.0, 0.0);
        td.F_init.resize(6);
        td.F_init = hrp::dvector::Zero(6);
        td.rel_pos_target = hrp::Vector3::Zero();
        td.vel_check_thresh = 0.3;
        td.vel_check_limit = 1.0;
        td.cont_vel_error_limit = 1.0;
        td.pos_target_thresh = 0.05; //m
        td.pos_error_limit = 0.15; //m
        ts.world_pos_target = hrp::Vector3::Zero();
        ts.world_pos_targ_dir = hrp::Vector3(1.0, 0.0, 0.0);
        ts.world_ori_targ_dir = hrp::Vector3(1.0, 0.0, 0.0);
        td.emergency_recover_time = 2.0;
        td.add_static_force = false;
        td.static_rfu_gain = 0.01;
        ts.F_now.resize(6);
        ts.F_now = hrp::dvector::Zero(6);
        ts.pos_over_limit = false;
        ts.pos_reach_target = false;
        ts.ori_over_limit = false;
        ts.vel_over_thresh = false;
        ts.vel_over_limit = false;
        ts.torque_over_limit = false;
        ts.F_eeR = hrp::Matrix33::Identity();
        ts.f2c_transition_count = 0;
        ts.max_f2c_t_count = 500;
        ts.em_transition_count = 0;
        ts.max_em_t_count = -1;
        ts.F_em_init.resize(6);
        ts.initial_pos = hrp::Vector3::Zero();
        ts.initial_ori = hrp::dquaternion(1.0, 0.0, 0.0, 0.0);
        ts.emergency_q.resize(p.manip->numJoints());
        ts.task_succeed_flag = false;
        ts.remove_static_force_count = 0;
        max_rsfc = std::floor(1.5 / RTC_PERIOD); //max remove static force time
        limb_task_target[ee_name] = td;
        limb_task_state[ee_name] = ts;
        release_emergency_called[ee_name] = false;
        start_emergency_called = false;
        fix_mode_normal[ee_name] = true; // do not enable modeselector by default
        // temporary: setting global for debugging
        check_dir_vel_err[ee_name] = 0.0;
        other_dir_vel_err[ee_name] = 0.0;
        dist_to_target[ee_name] = 0.0;
        pos_error_norm[ee_name] = 0.0;
        world_ref_wrench[ee_name].resize(6);

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
    invdyn_accvel_tq.resize(dof);
    invdyn_grav_tq.resize(dof);
    eecomp_tq.resize(dof);
    nullspace_tq.resize(dof);
    // error checkers
    reference_torque.resize(dof);
    idle_vlimit = 0.3; //1.0m/s = 3.6 km/h
    idle_olimit  = std::cos(deg2rad(45.0/2.0)); //45 degrees
    compliant_end_vel_thresh = 0.05; //0.05 m/s
    overwritten_qRef_prev.resize(dof); // use this because ref_vel in iob is calculated as (ref_q - prev_ref_q) / dt
    reference_climit.resize(dof);
    estimated_reference_velocity.resize(dof);
    transition_velest.resize(dof);
    log_est_q.resize(dof);
    log_act_q.resize(dof);
    log_ref_q.resize(dof);

    max_stop_overwriting_q_transition_count = 1000;
    for (int i=0; i<dof; ++i){
        coil::stringTo(reference_climit(i), ltc_climit_str[i].c_str()); //limiting current value to one in conf file
        qoldRef[i] = 0.0;
        dqoldRef[i] = 0.0;
    }
    actual_torque_vector.resize(dof);
    loop = 0;

    spit_log = false;
    is_emergency = false;
    reset_emergency_flag = false;

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
        // set parameters
        getTargetParameters(); //上位から来るref値を変数にセット
        calcForceMoment();
        getActualParameters(); //センサ値を変数にセット
        getEEStates(); //エンドエフェクタの位置、速度、力をget
        estimateEEVelForce();
        // error check
        VelocityErrorChecker();
        PositionErrorChecker();
        ActualTorqueChecker();
        ModeSelector();
        ReferenceForceUpdater();

        calcRefTorque();
        //estimateRefdq(); // to be tested more
        //CollisionDetector();
        ReferenceTorqueChecker();

        // set reference joint angles and torques
        for ( size_t i = 0; i<m_robot->numJoints(); ++i){
            m_q.data[i] = m_robotRef->joint(i)->q;
            hrp::Link* current_joint = m_robot->joint(i);
            // checking torque limits
            double max_torque = reference_climit(i) * current_joint->gearRatio * current_joint->torqueConst;
            if (reference_torque(i) > max_torque){
                if(loop%100 == 0){
                    std::cout << "[ltc]joint(" << i << ") reached max torque limit!!"
                              << "original ref=" << reference_torque(i) << ",max=" << max_torque
                              << std::endl;
                }
                reference_torque(i) = max_torque;
            }else if (reference_torque(i) < -max_torque){
                if(loop%100 == 0){
                    std::cout << "[ltc]joint(" << i << ") reached min torque limit!!"
                              << " original ref=" << reference_torque(i) << ",min=" << -max_torque
                              << std::endl;
                }
                reference_torque(i) = -max_torque;
            }
            m_tq.data[i] = reference_torque(i);
        }

        m_qOut.write();
        m_tqOut.write();
    }

    // write ref_force
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        std::string ee_name = it->first;
        int arm_idx = ee_map[ee_name].ee_index;
        if(param.is_active){
            for(int i=0; i<6; i++){
                m_ref_force[arm_idx].data[i] = - world_ref_wrench[ee_name][i]; //TODO: is sign correct???
                // m_ref_force[arm_idx].data[i] = 0.0;
            }
        }
        // need to write tm? (does not seem to be used...)
        m_ref_force[arm_idx].tm = m_qRef.tm;
        m_ref_forceOut[arm_idx]->write();
        it++;
    }

    // write emregency signal
    if (reset_emergency_flag) {
      m_emergencySignal.data = 0;
      m_emergencySignalOut.write();
      reset_emergency_flag = false;
      is_emergency = false;
    } else if (is_emergency) {
      m_emergencySignal.data = 1;
      m_emergencySignalOut.write();
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
        m_robotRef->joint(i)->u = 0.0;
        invdyn_accvel_tq(i) = 0.0;
        invdyn_grav_tq(i) = 0.0;
        eecomp_tq(i) = 0.0;
        nullspace_tq(i) = 0.0;
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
        m_robot->joint(i)->u = 0;
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
}

void LimbTorqueController::getEEStates()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        if (param.is_active) {
            std::string ee_name = it->first;
            LTParam& param = it->second;
            LTParam& ref_param = m_ref_lt_param[ee_name];
            hrp::JointPathExPtr manip = param.manip;
            hrp::JointPathExPtr ref_manip = ref_param.manip;
            // calculate jacobian, inverse, and null
            manip->calcJacobian(act_ee_jacobian[ee_name]);
            manip->calcJacobianInverseNullspace(act_ee_jacobian[ee_name], inv_act_ee_jacobian[ee_name], null_act_ee_jacobian[ee_name]); // including min/max avoidance
            hrp::Link* act_el = m_robot->link(ee_map[ee_name].target_name);
            hrp::Link* ref_el = m_robotRef->link(ee_map[ee_name].target_name);
            // end-effector position
            act_eepos[ee_name] = act_el->p + act_el->R*ee_map[ee_name].localPos;
            ref_eepos[ee_name] = ref_el->p + ref_el->R*ee_map[ee_name].localPos;
            ee_pos_error[ee_name] = ref_eepos[ee_name] - act_eepos[ee_name];
            // end-effector orientation
            act_eeR[ee_name] = act_el->R * ee_map[ee_name].localR;
            ref_eeR[ee_name] = ref_el->R * ee_map[ee_name].localR;
            act_eequat[ee_name] = act_eeR[ee_name]; // converting to quaternion
            ref_eequat[ee_name] = ref_eeR[ee_name]; // converting to quaternion
            hrp::dquaternion quat_diff;
            safe_quaternion_comparison(ref_eequat[ee_name], act_eequat[ee_name], quat_diff);
            ee_ori_error[ee_name] = quat_diff.vec();
            // set current paramters to previous parameters (run only at first)
            if(!geteestates_initialized[ee_name]){
                ee_vel_filter[ee_name].fill(act_eepos[ee_name]);
                ee_w_filter[ee_name].fill(act_eeR[ee_name]);
                ref_ee_vel_filter[ee_name].fill(ref_eepos[ee_name]);
                ref_ee_w_filter[ee_name].fill(ref_eeR[ee_name]);
                //prev_ref_eepos[ee_name] = ref_eepos[ee_name];
                //prev_ref_eeR[ee_name] = ref_eeR[ee_name];
                geteestates_initialized[ee_name] = true;
            }
            // set current position, rotation to the head of filters
            ee_vel_filter[ee_name].push(act_eepos[ee_name]);
            ref_ee_vel_filter[ee_name].push(ref_eepos[ee_name]);
            ee_w_filter[ee_name].push(act_eeR[ee_name]);
            ref_ee_w_filter[ee_name].push(ref_eeR[ee_name]);
            // calculate ee velocity
            act_ee_vel[ee_name] = ee_vel_filter[ee_name].get_velocity();
            //ref_ee_vel[ee_name] = (ref_eepos[ee_name] - prev_ref_eepos[ee_name]) / RTC_PERIOD;
            ref_ee_vel[ee_name] = ref_ee_vel_filter[ee_name].get_velocity();
            ee_vel_error[ee_name] = ref_ee_vel[ee_name] - act_ee_vel[ee_name];
            // callulate ee angular velocity
            hrp::Matrix33 act_ee_w_omega_mat = ee_w_filter[ee_name].get_velocity() * act_eeR[ee_name].transpose();
            hrp::Matrix33 ref_ee_w_omega_mat = ref_ee_w_filter[ee_name].get_velocity() * ref_eeR[ee_name].transpose();
            //hrp::Matrix33 ref_ee_w_omega_mat = ((ref_eeR[ee_name] - prev_ref_eeR[ee_name])/RTC_PERIOD) * ref_eeR[ee_name].transpose();
            act_ee_w[ee_name] <<
                ((act_ee_w_omega_mat(2,1) - act_ee_w_omega_mat(1,2)) / 2),
                ((act_ee_w_omega_mat(0,2) - act_ee_w_omega_mat(2,0)) / 2),
                ((act_ee_w_omega_mat(1,0) - act_ee_w_omega_mat(0,1)) / 2);
            ref_ee_w[ee_name] <<
                ((ref_ee_w_omega_mat(2,1) - ref_ee_w_omega_mat(1,2)) / 2),
                ((ref_ee_w_omega_mat(0,2) - ref_ee_w_omega_mat(2,0)) / 2),
                ((ref_ee_w_omega_mat(1,0) - ref_ee_w_omega_mat(0,1)) / 2);
            ee_w_error[ee_name] = ref_ee_w[ee_name] - act_ee_w[ee_name];

            // save current position, rotation as previous
            //prev_ref_eepos[ee_name] = ref_eepos[ee_name];
            //prev_ref_eeR[ee_name] = ref_eeR[ee_name];
        } // end if param is active
        it++;
    } //end while
}

void LimbTorqueController::calcRefTorque()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        if (param.is_active) {
            switch(param.amode){
            case(EMERGENCY):
                calcEmergencyLimbInverseDynamics(it);
                calcEmergencyEECompensation(it);
                calcEmergencyNullJointDumping(it);
                // std::cout << std::endl << "MODE: EMREGENCY" << std::endl;
                // std::cout << "EE: " << it->first << std::endl;
                // std::cout << "invdyn_accvel_tq = " << invdyn_accvel_tq.transpose() << std::endl;
                // std::cout << "invdyn_grav_tq = " << invdyn_grav_tq.transpose() << std::endl;
                // std::cout << "eecomp_tq = " << eecomp_tq.transpose() << std::endl;
                // std::cout << "nullspace_tq = " << nullspace_tq.transpose() << std::endl;
                break;
            case(MANIP_CONTACT):
                calcLimbInverseDynamics(it);
                calcContactEECompensation(it);
                calcNullJointDumping(it);
                // std::cout << std::endl << "MODE: CONTACT" << std::endl;
                // std::cout << "EE: " << it->first << std::endl;
                // std::cout << "invdyn_accvel_tq = " << invdyn_accvel_tq.transpose() << std::endl;
                // std::cout << "invdyn_grav_tq = " << invdyn_grav_tq.transpose() << std::endl;
                // std::cout << "eecomp_tq = " << eecomp_tq.transpose() << std::endl;
                // std::cout << "nullspace_tq = " << nullspace_tq.transpose() << std::endl;
                // if(strcmp(it->first.c_str(), "rarm")==0){
                //     std::cout << "invdyn_accvel_tq = " << invdyn_accvel_tq.transpose() << std::endl;
                //     std::cout << "invdyn_grav_tq = " << invdyn_grav_tq.transpose() << std::endl;
                //     std::cout << "eecomp_tq = " << eecomp_tq.transpose() << std::endl;
                //     std::cout << "nullspace_tq = " << nullspace_tq.transpose() << std::endl;
                // }
                break;
            case(IDLE_COMPLIANT):
                //TODO
                calcLimbInverseDynamics(it);
                calcEECompensation(it);
                calcNullJointDumping(it);
                break;
            case(IDLE_HARD):
                //TODO
                calcLimbInverseDynamics(it);
                calcEECompensation(it);
                calcNullJointDumping(it);
                break;
            case(MANIP_FREE):
                calcLimbInverseDynamics(it);
                calcManipEECompensation(it);
                calcNullJointDumping(it);
                break;
            default: //IDLE_NORMAL
                calcLimbInverseDynamics(it);
                calcEECompensation(it);
                calcNullJointDumping(it);
                break;
            }
        } // end if active
        it++;
    } // end while
    reference_torque = invdyn_accvel_tq + invdyn_grav_tq + eecomp_tq + nullspace_tq;
}

void LimbTorqueController::calcLimbInverseDynamics(std::map<std::string, LTParam>::iterator it)
{
    std::string ee_name = it->first;
    LTParam& param = it->second;
    int robotdof = m_robot->numJoints();
    hrp::dvector ref_dq_backup(robotdof), ref_ddq_backup(robotdof), act_dq_backup(robotdof), act_ddq_backup(robotdof), act_tq_backup(robotdof); // backup
    for(int i=0; i<robotdof; ++i){
        ref_dq_backup(i) = m_robotRef->joint(i)->dq;
        ref_ddq_backup(i) = m_robotRef->joint(i)->ddq;
        act_dq_backup(i) = m_robot->joint(i)->dq;
        act_ddq_backup(i) = m_robot->joint(i)->ddq;
        act_tq_backup(i) = m_robot->joint(i)->u; //not necessary right now
    }
    LTParam& ref_param = m_ref_lt_param[ee_name];
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
        invdyn_accvel_tq(jid) = m_robotRef->joint(jid)->u;
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
        invdyn_accvel_tq(jid) -= m_robotRef->joint(jid)->u;
        invdyn_grav_tq(jid) = m_robot->joint(jid)->u;
    }
    for(int i=0; i<robotdof; i++){
        m_robotRef->joint(i)->u = 0;
        m_robot->joint(i)->u = act_tq_backup(i);
        m_robotRef->joint(i)->dq = ref_dq_backup(i);
        m_robotRef->joint(i)->ddq = ref_ddq_backup(i);
        m_robot->joint(i)->dq = act_dq_backup(i);
        m_robot->joint(i)->ddq = act_ddq_backup(i);
    }
}

void LimbTorqueController::calcEmergencyLimbInverseDynamics(std::map<std::string, LTParam>::iterator it)
{
    std::string ee_name = it->first;
    LTParam& param = it->second;
    int robotdof = m_robot->numJoints();
    hrp::dvector act_dq_backup(robotdof), act_ddq_backup(robotdof), act_tq_backup(robotdof); // backup
    for(int i=0; i<robotdof; ++i){
        act_dq_backup(i) = m_robot->joint(i)->dq;
        act_ddq_backup(i) = m_robot->joint(i)->ddq;
        act_tq_backup(i) = m_robot->joint(i)->u; //not necessary right now
    }
    hrp::Link* base_link = param.manip->baseLink();
    Eigen::MatrixXd base_rot = base_link->R;
    for(int i=0; i<param.manip->numJoints(); i++){
        int jid = param.manip->joint(i)->jointId;
        invdyn_accvel_tq(jid) = 0.0;
        m_robot->joint(jid)->u = 0.0;
        m_robot->joint(jid)->dq = 0.0;
        m_robot->joint(jid)->ddq = 0.0;
    }
    hrp::Vector3 out_f, out_tau;
    out_f = Eigen::Vector3d::Zero(3);
    out_tau = Eigen::Vector3d::Zero(3);
    m_robot->calcInverseDynamics(base_link, out_f, out_tau); //gravity compensation for act
    for(int i=0; i<param.manip->numJoints(); i++){
        int jid = param.manip->joint(i)->jointId;
        invdyn_grav_tq(jid) = m_robot->joint(jid)->u;
    }
    for(int i=0; i<robotdof; i++){
        m_robot->joint(i)->u = act_tq_backup(i);
        m_robot->joint(i)->dq = act_dq_backup(i);
        m_robot->joint(i)->ddq = act_ddq_backup(i);
    }
}

//method E(Estimation of tau_ext via Momentum Observer) in S.Haddadin et al. "Robot Collisions: A Survey on Detection, Isolation, and Identification," TRO2017
void LimbTorqueController::CollisionDetector()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
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
                //mot_tq(i) = reference_torque(manip->joint(i)->jointId);
                // use sensor torque //probably correct for torque-controlled real robot?
                mot_tq(i) = actual_torque_vector[manip->joint(i)->jointId];
            }
            if (!collision_detector_initialized[ee_name]){
                //old_gen_mom[ee_name] = gen_mom[ee_name];
                initial_gen_mom[ee_name] = gen_mom[ee_name];
                collision_detector_initialized[ee_name] = true;
            }

            accum_tau[ee_name] += mot_tq*RTC_PERIOD;

            accum_beta[ee_name] += beta*RTC_PERIOD;
            accum_res[ee_name] += gen_mom_res[ee_name]*RTC_PERIOD;
            //calc residual
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
            if (collision_uncheck_count[ee_name] == 0){
                collision_uncheck_count[ee_name]--;
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
        it++;
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

// calculation "eeR * (some_gain) * eeR.transpose()" can be converted to just "(some_gain)" if the gain is always isotropic
void LimbTorqueController::calcEECompensation(std::map<std::string, LTParam>::iterator it)
{
    std::string ee_name = it->first;
    LTParam& param = it->second;
    LTParam& ref_param = m_ref_lt_param[ee_name];
    hrp::JointPathExPtr manip = param.manip;
    hrp::JointPathExPtr ref_manip = ref_param.manip;
    int limb_dof = manip->numJoints();
    // calculate position, orientation feedback
    ee_pos_comp_force[ee_name] = ref_eeR[ee_name] * param.ee_pgain_p * ref_eeR[ee_name].transpose() * ee_pos_error[ee_name]; // position feedback
    // orientation feedback
    // J.S. Yuan, "Closed-loop manipulator control using quaternion feedback," in IEEE Journal on Robotics and Automation, vol.4, no.4, pp.434-440, Aug. 1988.
    //ee_ori_comp_moment[ee_name] = - ref_eeR[ee_name] * param.ee_pgain_r * ref_eeR[ee_name].transpose() * ee_ori_error[ee_name];
    ee_ori_comp_moment[ee_name] = ref_eeR[ee_name] * param.ee_pgain_r * ref_eeR[ee_name].transpose() * ee_ori_error[ee_name]; //test

    if(loop%1000==0){
        std::cout << "EE Compensation Debug for " << ee_name << " start"  << std::endl;
        std::cout << "pos error is: " << std::endl << ee_pos_error[ee_name].transpose() << std::endl;
        std::cout << "orientation error is: " << std::endl << ee_ori_error[ee_name].transpose() << std::endl;
    }
    ee_pos_ori_comp_wrench[ee_name] << ee_pos_comp_force[ee_name], ee_ori_comp_moment[ee_name];

    // calculate velocity, angular velocity feedback
    ee_vel_comp_force[ee_name] = ref_eeR[ee_name] * param.ee_dgain_p * ref_eeR[ee_name].transpose() * ee_vel_error[ee_name];
    ee_w_comp_moment[ee_name] = ref_eeR[ee_name] * param.ee_dgain_r * ref_eeR[ee_name].transpose() * ee_w_error[ee_name];
    ee_vel_w_comp_wrench[ee_name] << ee_vel_comp_force[ee_name], ee_w_comp_moment[ee_name];

    // map wrench to joint torque
    ee_compensation_torque[ee_name] = act_ee_jacobian[ee_name].transpose() * (ee_pos_ori_comp_wrench[ee_name] + ee_vel_w_comp_wrench[ee_name]);

    // debug output
    if(loop%1000==0){
        std::cout << "EE pgain force = " << ee_pos_ori_comp_wrench[ee_name].transpose() << std::endl;
        std::cout << "EE vel error = " << ee_vel_error[ee_name].transpose() << std::endl;
        std::cout << "EE w error = " << ee_w_error[ee_name].transpose() << std::endl;
        std::cout << "EE dgain force = " << ee_vel_w_comp_wrench[ee_name].transpose() << std::endl;
        std::cout << "EE Compensation Debug for " << ee_name << " end"  << std::endl;
    }
    // store reference torque values
    for (int i=0; i<limb_dof; i++){
        eecomp_tq(manip->joint(i)->jointId) = ee_compensation_torque[ee_name](i);
    }
}

void LimbTorqueController::calcEmergencyEECompensation(std::map<std::string, LTParam>::iterator it)
{
    std::string ee_name = it->first;
    LTParam& param = it->second;
    LTParam& ref_param = m_ref_lt_param[ee_name];
    TaskState& ts = limb_task_state[ee_name];
    hrp::JointPathExPtr manip = param.manip;
    hrp::JointPathExPtr ref_manip = ref_param.manip;
    int limb_dof = manip->numJoints();
    // re-calculate position, orientation with emergency states
    // position & orientation: reference is stopped at the transition to emergency mode
    hrp::Vector3 em_ee_pos_error = ts.initial_pos - act_eepos[ee_name];
    hrp::dquaternion em_ee_ori_error_quat;
    safe_quaternion_comparison(ts.initial_ori, act_eequat[ee_name], em_ee_ori_error_quat);
    // calculate position, orientation feedback
    ee_pos_comp_force[ee_name] = act_eeR[ee_name] * param.ee_pgain_p * act_eeR[ee_name].transpose() * em_ee_pos_error; // position feedback
    // orientation feedback
    ee_ori_comp_moment[ee_name] = act_eeR[ee_name] * param.ee_pgain_r * act_eeR[ee_name].transpose() * em_ee_ori_error_quat.vec();
    ee_pos_ori_comp_wrench[ee_name] << ee_pos_comp_force[ee_name], ee_ori_comp_moment[ee_name];

    // calculate velocity, angular velocity feedback
    // velocity & angular velocity: reference is zero
    ee_vel_error[ee_name] = - act_ee_vel[ee_name];
    ee_w_error[ee_name] = - act_ee_w[ee_name];
    ee_vel_comp_force[ee_name] = act_eeR[ee_name] * param.ee_dgain_p * act_eeR[ee_name].transpose() * ee_vel_error[ee_name];
    ee_w_comp_moment[ee_name] = act_eeR[ee_name] * param.ee_dgain_r * act_eeR[ee_name].transpose() * ee_w_error[ee_name];
    ee_vel_w_comp_wrench[ee_name] << ee_vel_comp_force[ee_name], ee_w_comp_moment[ee_name];

    if(false){
        std::cout << "initial_pos = " << ts.initial_pos.transpose() << std::endl;
        std::cout << "act_pos = " << act_eepos[ee_name].transpose() << std::endl;
        std::cout << "em_ee_pos_error = " << em_ee_pos_error.transpose() << std::endl;
        std::cout << "pos_ori_comp_wrench = " << ee_pos_ori_comp_wrench[ee_name].transpose() << std::endl;
        std::cout << "vel_w_comp_wrench = " << ee_vel_w_comp_wrench[ee_name].transpose() << std::endl;
        std::cout << "F_now = " << ts.F_now.transpose() << std::endl;
    }
    world_ref_wrench[ee_name] << ts.F_now;
    // std::cout << std::endl << "Emergency EEComp Torque:" << ee_name << std::endl;
    // std::cout << "pos_ori_comp_wrench = " << ee_pos_ori_comp_wrench[ee_name].transpose() << std::endl;
    // std::cout << "vel_w_comp_wrench = " << ee_vel_w_comp_wrench[ee_name].transpose() << std::endl;
    // std::cout << "emergency_wrench = " << emergency_wrench.transpose() << std::endl;
    // map wrench to joint torque
    ee_compensation_torque[ee_name] = act_ee_jacobian[ee_name].transpose() * (ee_pos_ori_comp_wrench[ee_name] + ee_vel_w_comp_wrench[ee_name] + world_ref_wrench[ee_name]);

    // debug output
    if(loop%1000==0){
        std::cout << "EE pgain force = " << ee_pos_ori_comp_wrench[ee_name].transpose() << std::endl;
        std::cout << "EE vel error = " << ee_vel_error[ee_name].transpose() << std::endl;
        std::cout << "EE w error = " << ee_w_error[ee_name].transpose() << std::endl;
        std::cout << "EE dgain force = " << ee_vel_w_comp_wrench[ee_name].transpose() << std::endl;
        std::cout << "EE Compensation Debug for " << ee_name << " end"  << std::endl;
    }
    // store reference torque values
    for (int i=0; i<limb_dof; i++){
        eecomp_tq(manip->joint(i)->jointId) = ee_compensation_torque[ee_name](i);
    }
}

void LimbTorqueController::calcManipEECompensation(std::map<std::string, LTParam>::iterator it)
{
    std::string ee_name = it->first;
    LTParam& param = it->second;
    LTParam& ref_param = m_ref_lt_param[ee_name];
    TaskState& ts = limb_task_state[ee_name];
    hrp::JointPathExPtr manip = param.manip;
    hrp::JointPathExPtr ref_manip = ref_param.manip;
    int limb_dof = manip->numJoints();
    // calculate position, orientation feedback
    ee_pos_comp_force[ee_name] = ref_eeR[ee_name] * param.ee_pgain_p * ref_eeR[ee_name].transpose() * ee_pos_error[ee_name]; // position feedback
    // orientation feedback
    // J.S. Yuan, "Closed-loop manipulator control using quaternion feedback," in IEEE Journal on Robotics and Automation, vol.4, no.4, pp.434-440, Aug. 1988.
    //ee_ori_comp_moment[ee_name] = - ref_eeR[ee_name] * param.ee_pgain_r * ref_eeR[ee_name].transpose() * ee_ori_error[ee_name];
    ee_ori_comp_moment[ee_name] = ref_eeR[ee_name] * param.ee_pgain_r * ref_eeR[ee_name].transpose() * ee_ori_error[ee_name]; //test

    if(loop%1000==0){
        std::cout << "EE Compensation Debug for " << ee_name << " start"  << std::endl;
        std::cout << "pos error is: " << std::endl << ee_pos_error[ee_name].transpose() << std::endl;
        std::cout << "orientation error is: " << std::endl << ee_ori_error[ee_name].transpose() << std::endl;
    }
    ee_pos_ori_comp_wrench[ee_name] << ee_pos_comp_force[ee_name], ee_ori_comp_moment[ee_name];

    // calculate velocity, angular velocity feedback
    ee_vel_comp_force[ee_name] = ref_eeR[ee_name] * param.ee_dgain_p * ref_eeR[ee_name].transpose() * ee_vel_error[ee_name];
    ee_w_comp_moment[ee_name] = ref_eeR[ee_name] * param.ee_dgain_r * ref_eeR[ee_name].transpose() * ee_w_error[ee_name];
    ee_vel_w_comp_wrench[ee_name] << ee_vel_comp_force[ee_name], ee_w_comp_moment[ee_name];

    // map wrench to joint torque
    if(limb_task_target[ee_name].add_static_force || ts.remove_static_force_count > 0){ //add static reactive force or transition from add_static_force=true->false
        world_ref_wrench[ee_name] << ts.F_now.head(3), hrp::Vector3::Zero();
        ee_compensation_torque[ee_name] = act_ee_jacobian[ee_name].transpose() * (ee_pos_ori_comp_wrench[ee_name] + ee_vel_w_comp_wrench[ee_name] + world_ref_wrench[ee_name]);
    }else{
        ee_compensation_torque[ee_name] = act_ee_jacobian[ee_name].transpose() * (ee_pos_ori_comp_wrench[ee_name] + ee_vel_w_comp_wrench[ee_name]);
    }

    // debug output
    if(loop%1000==0){
        std::cout << "EE pgain force = " << ee_pos_ori_comp_wrench[ee_name].transpose() << std::endl;
        std::cout << "EE vel error = " << ee_vel_error[ee_name].transpose() << std::endl;
        std::cout << "EE w error = " << ee_w_error[ee_name].transpose() << std::endl;
        std::cout << "EE dgain force = " << ee_vel_w_comp_wrench[ee_name].transpose() << std::endl;
        std::cout << "EE Compensation Debug for " << ee_name << " end"  << std::endl;
    }
    // store reference torque values
    for (int i=0; i<limb_dof; i++){
        eecomp_tq(manip->joint(i)->jointId) = ee_compensation_torque[ee_name](i);
    }
}

void LimbTorqueController::calcContactEECompensation(std::map<std::string, LTParam>::iterator it)
{
    std::string ee_name = it->first;
    LTParam& param = it->second;
    LTParam& ref_param = m_ref_lt_param[ee_name];
    hrp::JointPathExPtr manip = param.manip;
    hrp::JointPathExPtr ref_manip = ref_param.manip;
    TaskState& ts = limb_task_state[ee_name];
    int limb_dof = manip->numJoints();
    // calculate position, orientation feedback
    ee_pos_comp_force[ee_name] = ref_eeR[ee_name] * param.ee_pgain_p * ref_eeR[ee_name].transpose() * ee_pos_error[ee_name]; // position feedback
    // orientation feedback
    // J.S. Yuan, "Closed-loop manipulator control using quaternion feedback," in IEEE Journal on Robotics and Automation, vol.4, no.4, pp.434-440, Aug. 1988.
    //ee_ori_comp_moment[ee_name] = - ref_eeR[ee_name] * param.ee_pgain_r * ref_eeR[ee_name].transpose() * ee_ori_error[ee_name];
    ee_ori_comp_moment[ee_name] = ref_eeR[ee_name] * param.ee_pgain_r * ref_eeR[ee_name].transpose() * ee_ori_error[ee_name]; //test

    if(loop%1000==0){
        std::cout << "EE Compensation Debug for " << ee_name << " start"  << std::endl;
        std::cout << "pos error is: " << std::endl << ee_pos_error[ee_name].transpose() << std::endl;
        std::cout << "orientation error is: " << std::endl << ee_ori_error[ee_name].transpose() << std::endl;
    }
    ee_pos_ori_comp_wrench[ee_name] << ee_pos_comp_force[ee_name], ee_ori_comp_moment[ee_name];

    // calculate velocity, angular velocity feedback
    ee_vel_comp_force[ee_name] = ref_eeR[ee_name] * param.ee_dgain_p * ref_eeR[ee_name].transpose() * ee_vel_error[ee_name];
    ee_w_comp_moment[ee_name] = ref_eeR[ee_name] * param.ee_dgain_r * ref_eeR[ee_name].transpose() * ee_w_error[ee_name];
    ee_vel_w_comp_wrench[ee_name] << ee_vel_comp_force[ee_name], ee_w_comp_moment[ee_name];
    //contact_wrench << cont_w_f, cont_w_m;
    world_ref_wrench[ee_name] << ts.F_now;
    // std::cout << std::endl << "Contact EEComp Torque:" << ee_name << std::endl;
    // std::cout << "pos_ori_comp_wrench = " << ee_pos_ori_comp_wrench[ee_name].transpose() << std::endl;
    // std::cout << "vel_w_comp_wrench = " << ee_vel_w_comp_wrench[ee_name].transpose() << std::endl;
    // std::cout << "emergency_wrench = " << contact_wrench.transpose() << std::endl;
    // map wrench to joint torque
    ee_compensation_torque[ee_name] = act_ee_jacobian[ee_name].transpose() * (ee_pos_ori_comp_wrench[ee_name] + ee_vel_w_comp_wrench[ee_name] + world_ref_wrench[ee_name]);

    // debug output
    if(loop%1000==0){
        std::cout << "EE pgain force = " << ee_pos_ori_comp_wrench[ee_name].transpose() << std::endl;
        std::cout << "EE vel error = " << ee_vel_error[ee_name].transpose() << std::endl;
        std::cout << "EE w error = " << ee_w_error[ee_name].transpose() << std::endl;
        std::cout << "EE dgain force = " << ee_vel_w_comp_wrench[ee_name].transpose() << std::endl;
        std::cout << "EE Compensation Debug for " << ee_name << " end"  << std::endl;
    }
    // store reference torque values
    for (int i=0; i<limb_dof; i++){
        eecomp_tq(manip->joint(i)->jointId) = ee_compensation_torque[ee_name](i);
    }
}

void LimbTorqueController::calcNullJointDumping(std::map<std::string, LTParam>::iterator it)
{
    std::string ee_name = it->first;
    LTParam& param = it->second;
    LTParam& ref_param = m_ref_lt_param[ee_name];
    hrp::JointPathExPtr manip = param.manip;
    hrp::JointPathExPtr ref_manip = ref_param.manip;
    int limb_dof = manip->numJoints();
    hrp::dvector q_error(limb_dof), dq_error(limb_dof);
    for (int i=0; i<limb_dof; i++) {
        q_error(i) = ref_manip->joint(i)->q - manip->joint(i)->q;
        dq_error(i) = ref_manip->joint(i)->dq - manip->joint(i)->dq;
    }
    null_space_torque[ee_name] = null_act_ee_jacobian[ee_name] * (param.pgain * q_error + param.dgain * dq_error);
    if(loop%1000==0){
        std::cout << "q error = " << q_error.transpose() << std::endl;
        std::cout << "null_tau  = " << null_space_torque[ee_name].transpose() << std::endl;
    }
    for (int i=0; i<limb_dof; i++){
        nullspace_tq(manip->joint(i)->jointId) = null_space_torque[ee_name](i);
    }
}

void LimbTorqueController::calcEmergencyNullJointDumping(std::map<std::string, LTParam>::iterator it)
{
    std::string ee_name = it->first;
    LTParam& param = it->second;
    LTParam& ref_param = m_ref_lt_param[ee_name];
    hrp::JointPathExPtr manip = param.manip;
    hrp::JointPathExPtr ref_manip = ref_param.manip;
    int limb_dof = manip->numJoints();
    hrp::dvector q_error(limb_dof), dq_error(limb_dof);
    for (int i=0; i<limb_dof; i++) {
        q_error(i) = limb_task_state[ee_name].emergency_q(i) - manip->joint(i)->q;
        dq_error(i) = - manip->joint(i)->dq;
    }
    null_space_torque[ee_name] = null_act_ee_jacobian[ee_name] * (param.pgain * q_error + param.dgain * dq_error);
    if(loop%1000==0){
        std::cout << "q error = " << q_error.transpose() << std::endl;
        std::cout << "null_tau  = " << null_space_torque[ee_name].transpose() << std::endl;
    }
    for (int i=0; i<limb_dof; i++){
        nullspace_tq(manip->joint(i)->jointId) = null_space_torque[ee_name](i);
    }
}

// Estimate Reference Velocity
void LimbTorqueController::estimateRefdq()
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
            imp_now[ee_name] = - inv_act_ee_jacobian[ee_name] * ee_ref_vel; // is sign correct?
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

// eeest_initialized: EE速度フィルタが始まってからチェックする
// TODO; モード依存のチェック(directional)
void LimbTorqueController::VelocityErrorChecker()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        std::string ee_name = it->first;
        if (param.is_active && eeest_initialized[ee_name]) {
            TaskDescription& td = limb_task_target[ee_name];
            TaskState& ts = limb_task_state[ee_name];
            switch(param.amode){
                // Free motion
            case(MANIP_FREE):{
                if(td.type == FIX || td.type == MOTION_ONLY){
                    // ignoring check direction... this implemantation should be changed for other tasks than just collision detection only
                    // hrp::Vector3 ref_act_eevel_diff = ref_eeR[ee_name].transpose() * ref_ee_vel[ee_name] - act_eeR[ee_name].transpose() * filtered_ee_vel[ee_name]; // in ee local coordinates
                    hrp::Vector3 ref_act_eevel_diff = ref_ee_vel[ee_name] - filtered_ee_vel[ee_name]; // calculate in world because no direction is specified
                    other_dir_vel_err[ee_name] = ref_act_eevel_diff.norm();
                    if (other_dir_vel_err[ee_name] > td.vel_check_limit){
                        ts.vel_over_limit = true;
                    }
                }else{
                    // hrp::Vector3 ref_act_eevel_diff = ref_eeR[ee_name].transpose() * ref_ee_vel[ee_name] - act_eeR[ee_name].transpose() * filtered_ee_vel[ee_name]; // in ee local coordinates
                    hrp::Vector3 ref_act_eevel_diff = ref_eeR[ee_name].transpose() * ref_ee_vel[ee_name] - ref_eeR[ee_name].transpose() * filtered_ee_vel[ee_name]; // in ee local coordinates: use ref_eeR because act_eeR is too noisy
                    // double check_dir_vel_err = td.velocity_check_dir.dot(ref_act_eevel_diff);
                    // double other_dir_vel_err = (ref_act_eevel_diff - ref_act_eevel_diff.dot(td.velocity_check_dir) * td.velocity_check_dir).norm();
                    // temporary: setting global for debugging
                    check_dir_vel_err[ee_name] = td.velocity_check_dir.dot(ref_act_eevel_diff);
                    other_dir_vel_err[ee_name] = (ref_act_eevel_diff - ref_act_eevel_diff.dot(td.velocity_check_dir) * td.velocity_check_dir).norm();
                    if (check_dir_vel_err[ee_name] > td.vel_check_thresh){
                        ts.vel_over_thresh = true;
                    }else if (other_dir_vel_err[ee_name] > td.vel_check_limit){
                        ts.vel_over_limit = true;
                    }
                }
                break;
            } // end case manip_normal
            case(MANIP_CONTACT):{
                if(filtered_ee_vel[ee_name].norm() > td.cont_vel_error_limit){
                    ts.vel_over_limit = true;
                }
                break;
            } // end case manip_contact
            case(IDLE_NORMAL):{ // mode IDLE_NORMAL
                hrp::Vector3 ref_act_eevel_diff = ref_ee_vel[ee_name] - filtered_ee_vel[ee_name];
                if (ref_act_eevel_diff.norm() > 0.1){ //threshold should be tuned
                    ts.vel_over_limit = true;
                }
                break;
            }
            case(IDLE_COMPLIANT):{
                hrp::Vector3 ref_act_eevel_diff = ref_ee_vel[ee_name] - filtered_ee_vel[ee_name];
                if (ref_act_eevel_diff.norm() < compliant_end_vel_thresh){ //threshold should be tuned
                    ts.vel_over_thresh = true;
                }
                break;
            } // end case idle_compliant
            case(IDLE_HARD):{
                hrp::Vector3 ref_act_eevel_diff = ref_ee_vel[ee_name] - filtered_ee_vel[ee_name];
                if (ref_act_eevel_diff.norm() < compliant_end_vel_thresh){ //threshold should be tuned
                    ts.vel_over_thresh = true;
                }
                break;
            } // end case idle_hard
            default:{ // EMERGENCY
                ts.vel_over_limit = false; //do not check
                break;
            }
            } // end switch amode
        }
        it++;
    }
}

// TODO; モード依存のチェック(directional)
void LimbTorqueController::PositionErrorChecker()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        std::string ee_name = it->first;
        if (param.is_active && eeest_initialized[ee_name]) {
            TaskDescription& td = limb_task_target[ee_name];
            TaskState& ts = limb_task_state[ee_name];
            hrp::JointPathExPtr manip = param.manip;
            int limbdof = manip->numJoints();
            if(!poserrchecker_initialized[ee_name]){
                minmaxavoid_torque[ee_name].resize(limbdof);
                poserrchecker_initialized[ee_name] = true;
            }
            switch(param.amode){
            case(MANIP_CONTACT):{
                switch(td.type){
                case(MOVE_POS):{
                    hrp::Vector3 vec_to_target = ts.world_pos_target - act_eepos[ee_name];
                    dist_to_target[ee_name] = vec_to_target.dot(ts.world_pos_targ_dir);
                    pos_error_norm[ee_name] = (vec_to_target - dist_to_target[ee_name] * ts.world_pos_targ_dir).norm();
                    if (dist_to_target[ee_name] <= td.pos_target_thresh && !ts.task_succeed_flag){
                        ts.pos_reach_target = true;
                    } else if(pos_error_norm[ee_name] > td.pos_error_limit){
                        ts.pos_over_limit = true;
                    }
                    break;
                } //end case MOVE_POS
                case(MOTION_ONLY):{
                    // hrp::Vector3 vec_to_target = ts.world_pos_target - act_eepos[ee_name];
                    // dist_to_target[ee_name] = vec_to_target.dot(ts.world_pos_targ_dir);
                    // pos_error_norm[ee_name] = (vec_to_target - dist_to_target[ee_name] * ts.world_pos_targ_dir).norm();
                    pos_error_norm[ee_name] = ee_pos_error[ee_name].norm();
                    if(pos_error_norm[ee_name] > td.pos_error_limit){
                        ts.pos_over_limit = true;
                    }
                    break;
                }
                case(FIX):
                    break; //TODO: implement this
                }
                break;
            } // end case MANIP_CONTACT
            case(EMERGENCY):{
                break; //do not check
            }
            case(MANIP_FREE):{
                // end-effector orientation error check
                hrp::dquaternion quat_error;
                safe_quaternion_comparison(ref_eequat[ee_name], act_eequat[ee_name], quat_error);
                if (quat_error.w() < idle_olimit){  //be careful of magnitude relation: comparing cos(x/2) (quatdiff.w() will be bigger near x=0)
                    std::cout << "Error: " << quat_error.w() << std::endl;
                    std::cout << "Limit: " << idle_olimit << std::endl;
                    ts.ori_over_limit = true;
                }
                break;
            }
            default:{ //IDLE_NORMAL, IDLE_COMPLIANT, IDLE_HARD
                // joint angle min-max check
                minmaxavoid_torque[ee_name] = hrp::dvector::Zero(limbdof);
                double mm_margin = deg2rad(3.0); // should be tuned
                for ( int i = 0; i < limbdof; i++ ) {
                    double now_angle = manip->joint(i)->q;
                    double max_angle = manip->joint(i)->ulimit;
                    double min_angle = manip->joint(i)->llimit;
                    if ( (now_angle+mm_margin) >= max_angle ) {
                        double max_torque = manip->joint(i)->climit * manip->joint(i)->gearRatio * manip->joint(i)->torqueConst;
                        double avoid_torque = - max_torque/mm_margin*(now_angle - (max_angle-mm_margin));
                        minmaxavoid_torque[ee_name](i) = avoid_torque;
                    }else if ( (now_angle-mm_margin) <= min_angle) {
                        double max_torque = manip->joint(i)->climit * manip->joint(i)->gearRatio * manip->joint(i)->torqueConst;
                        double avoid_torque = max_torque/mm_margin*(min_angle+mm_margin - now_angle);
                        minmaxavoid_torque[ee_name](i) = avoid_torque;
                    }
                }
                // end-effector orientation error check
                hrp::dquaternion quat_error;
                safe_quaternion_comparison(ref_eequat[ee_name], act_eequat[ee_name], quat_error);
                if (quat_error.w() < idle_olimit){  //be careful of magnitude relation: comparing cos(x/2) (quatdiff.w() will be bigger near x=0)
                    std::cout << "Error: " << quat_error.w() << std::endl;
                    std::cout << "Limit: " << idle_olimit << std::endl;
                    ts.ori_over_limit = true;
                }
                break;
            } //end default case
            } //end switch
        }
        it++;
    }
}

void LimbTorqueController::ReferenceTorqueChecker()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        std::string ee_name = it->first;
        if (param.is_active) {
            hrp::JointPathExPtr manip = param.manip;
            int limbdof = manip->numJoints();
            for (int i = 0; i<limbdof; i++){
                int jid = m_robot->joint(i)->jointId;
                hrp::Link* current_joint = m_robot->joint(jid);
                double max_torque = reference_climit(jid) * current_joint->gearRatio * current_joint->torqueConst;
                if (reference_torque(jid) > max_torque){
                    reference_torque(i) = max_torque;
                    std::cout << "[ltc]joint(" << i << ") reached max torque limit!!"
                              << "original ref=" << reference_torque(i) << ",max=" << max_torque
                              << std::endl;
                }else if (reference_torque(jid) < -max_torque){
                    reference_torque(i) = -max_torque;
                    std::cout << "[ltc]joint(" << i << ") reached min torque limit!!"
                              << "original ref=" << reference_torque(i) << ",min=" << -max_torque
                              << std::endl;
                }
            }
        }
        it++;
    }
}

void LimbTorqueController::ActualTorqueChecker()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        std::string ee_name = it->first;
        if (param.is_active && (param.amode==MANIP_CONTACT
                                || (param.amode!=EMERGENCY && limb_task_target[ee_name].add_static_force))) {
            hrp::JointPathExPtr manip = param.manip;
            int limbdof = manip->numJoints();
            for (int i = 0; i<limbdof; i++){
                int jid = manip->joint(i)->jointId;
                hrp::Link* current_joint = m_robot->joint(jid);
                //double max_torque = current_joint->climit * current_joint->gearRatio * current_joint->torqueConst;
                double max_torque = 0.7 * current_joint->climit * current_joint->gearRatio * current_joint->torqueConst; //limiting actual by reduced limit for safety
                if (actual_torque_vector(jid) > max_torque){
                    std::cout << "Actual torque over limit: max = " << max_torque << ", current = " << actual_torque_vector(jid) << std::endl;
                    limb_task_state[ee_name].torque_over_limit = true;
                }else if (actual_torque_vector(jid) < -max_torque){
                    std::cout << "Actual torque over limit: min = " << -max_torque << ", current = " << actual_torque_vector(jid) << std::endl;
                    limb_task_state[ee_name].torque_over_limit = true;
                }
            } //end for
        }
        it++;
    }
}

// Update reference force in MANIP_CONTACT mode
void LimbTorqueController::ReferenceForceUpdater()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        std::string ee_name = it->first;
        if (param.is_active) {
            TaskState& ts = limb_task_state[ee_name];
            TaskDescription& td = limb_task_target[ee_name];
            hrp::JointPathExPtr manip = param.manip;
            // transition to emergency
            if(param.amode == EMERGENCY){
                if(ts.em_transition_count > 0){ //emergency transition
                    ts.F_now.head(3) = ts.F_em_init.head(3) * (static_cast<double>(ts.em_transition_count) / static_cast<double>(ts.max_em_t_count)); //force
                    ts.F_now.tail(3) = ts.F_em_init.tail(3) * (static_cast<double>(ts.em_transition_count) / static_cast<double>(ts.max_em_t_count)); //moment
                    // std::cout << "[emememeememem]" << std::endl;
                    // std::cout << "em_transition_count = " << ts.em_transition_count << std::endl;
                    // std::cout << "F_now = " << ts.F_now.transpose() << std::endl;
                    ts.em_transition_count--;
                }
                if(ts.em_transition_count <= 0){ //emergency stable: waiting for release
                    ts.F_now = hrp::dvector::Zero(6);
                }
            } // end if emergency
            if(param.amode == MANIP_CONTACT){
                if(ts.task_succeed_flag){ //if in MANIP_CONTACT and already succeeded in the task: same as emergency
                    ts.F_now.head(3) = ts.F_now.head(3) + td.static_rfu_gain * (- filtered_f_s[ee_name] - ts.F_now.head(3));  // same as add_static_force
                    if(ts.em_transition_count > 0){ //emergency transition
                        //ts.F_now = (static_cast<double>(ts.em_transition_count) / static_cast<double>(ts.max_em_t_count)) * ts.F_em_init;
                        ts.em_transition_count--;
                    }
                    // if(ts.em_transition_count <= 0){ //emergency stable: waiting for release
                    //     ts.F_now = hrp::dvector::Zero(6);
                    // }
                } //end if task_succeed_flag
                // transition from MANIP_FREE to MANIP_CONTACT
                else if(ts.f2c_transition_count > 0){
                    if(td.type != FIX){
                        ts.F_now.head(3) = ts.F_em_init.head(3) + static_cast<double>((ts.max_f2c_t_count - ts.f2c_transition_count)) * (ts.F_eeR * td.F_init.head(3) - ts.F_em_init.head(3)) / static_cast<double>(ts.max_f2c_t_count);
                    }else{ //if type=FIX // TODO:ここは使うときに修正する
                        ts.F_now = ts.F_now.head(3) + td.static_rfu_gain * (- filtered_f_s[ee_name] - ts.F_now.head(3));
                    }
                    ts.f2c_transition_count--;
                }
                else{ // if transition is over and task has not succeed
                    switch(td.type){
                    case(MOVE_POS):{
                        ts.F_now.head(3) = ts.F_now.head(3) + td.static_rfu_gain * (- filtered_f_s[ee_name] - ts.F_now.head(3));  // same as add_static_force situation
                        break;
                    } //end case MOVE_POS
                    case(FIX): // TODO:ここは使うときに修正する
                        ts.F_now.head(3) = ts.F_now.head(3) + td.static_rfu_gain * (- filtered_f_s[ee_name] - ts.F_now.head(3));  //caution: this is in world frame!
                        break; //do nothing
                    case(MOTION_ONLY): //although this case won't happen
                        ts.F_now.head(3) = ts.F_now.head(3) + td.static_rfu_gain * (- filtered_f_s[ee_name] - ts.F_now.head(3));  //caution: this is in world frame!
                        break;
                    } //end switch task type
                }
            } //end if MANIP_CONTACT
            else if(param.amode == MANIP_FREE && td.add_static_force){
                ts.F_now.head(3) = ts.F_now.head(3) + td.static_rfu_gain * (- filtered_f_s[ee_name] - ts.F_now.head(3));  //caution: this is in world frame!
                //ts.F_now.head(3) = ts.F_now.head(3) + td.static_rfu_gain * (- abs_forces[param.sensor_name] - ts.F_now.head(3));  //cution: just for evaluation of filter
            }else if(ts.remove_static_force_count > 0){
                double redu_rate = (static_cast<double>(ts.remove_static_force_count) / static_cast<double>(max_rsfc));
                ts.F_now.head(3) =  redu_rate * ts.F_em_init.head(3);
                ts.remove_static_force_count--;
                if(ts.remove_static_force_count == 0){
                    td.add_static_force = false;
                }
            }
        } //end if param is active
        it++;
    }
}

//TODO: 片手だけEMERGENCYの時とかあまりしっかり作れていない気がする
void LimbTorqueController::ModeSelector()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    bool change_to_contact = false; //for two arms
    bool change_to_emergency = false; //for two arms
    bool change_to_idle = false; //for two arms
    bool dual_task_succeed = false; //for two arms
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        std::string ee_name = it->first;
        if (param.is_active) {
            if(fix_mode_normal[ee_name]){
                param.amode = IDLE_NORMAL;
                if(loop%300==0){
                    std::cout << "ee = " << ee_name << std::endl;
                    std::cout << "      [ltc] MODE: IDLE_NORMAL... fixed by setting call startModeChange if you want to release" << std::endl;
                }
            } else{
                TaskDescription& td = limb_task_target[ee_name];
                TaskState&  ts = limb_task_state[ee_name];
                switch(param.amode){
                case(IDLE_NORMAL):
                    if(loop%300==0){
                        std::cout << "      [ltc] MODE: IDLE_NORMAL" << std::endl;
                    }
                    if(ts.pos_over_limit || ts.ori_over_limit){
                        if(ts.pos_over_limit){
                            for (int i=0; i<100; i++){
                                std::cout << "change to IDLE_HARD because of position!!!!!" << std::endl;
                            }
                        }
                        if(ts.ori_over_limit){
                            for (int i=0; i<100; i++){
                                std::cout << "change to IDLE_HARD because of orientation!!!!!" << std::endl;
                            }
                        }
                        param.amode = IDLE_HARD;
                        reset_taskstate_bool(ts);
                    }else if(ts.vel_over_limit){
                        for (int i=0; i<100; i++){
                            std::cout << "change to IDLE_COMPLIANT because of velocity!!!!!" << std::endl;
                        }
                        param.amode = IDLE_COMPLIANT;
                        reset_taskstate_bool(ts);
                    }
                    break;
                case(IDLE_COMPLIANT):
                    if(loop%300==0){
                        std::cout << "      [ltc] MODE: IDLE_COMPLIANT" << std::endl;
                    }
                    if(ts.pos_over_limit || ts.ori_over_limit){
                        param.amode = IDLE_HARD;
                        reset_taskstate_bool(ts);
                    }else if(ts.vel_over_thresh){
                        param.amode = IDLE_NORMAL;
                        reset_taskstate_bool(ts);
                    }
                    //TODO: make recovery from compliant mode...position and velocity check?
                    break;
                case(IDLE_HARD):
                    if(loop%300==0){
                        std::cout << "      [ltc] MODE: IDLE_HARD" << std::endl;
                    }
                    if(ts.vel_over_thresh){
                        param.amode = IDLE_NORMAL;
                        reset_taskstate_bool(ts);
                    }
                    //TODO: make recovery from HARD mode...position check?
                    break;
                case(MANIP_FREE):
                    if(loop%300==0){
                        std::cout << "      [ltc] MODE: MANIP_FREE" << std::endl;
                    }
                    if(ts.vel_over_limit || ts.pos_over_limit || ts.ori_over_limit || ts.torque_over_limit){
                        for(int i=0; i<100; i++){
                            std::cout << ee_name << ": ";
                            std::cout << "Transition from MANIP_FREE to EMERGEBCY because of ";
                            if(ts.vel_over_limit){
                                std::cout << "VELOCITY limit !!!!" << std::endl;
                            }
                            if(ts.pos_over_limit){
                                std::cout << "POSITION limit !!!!" << std::endl;
                            }
                            if(ts.ori_over_limit){
                                std::cout << "ORIENTATION limit !!!!" << std::endl;
                            }
                            if(ts.torque_over_limit){
                                std::cout << "ACT_TORQUE limit !!!!" << std::endl;
                            }
                        }
                        if(td.dual){
                            change_to_emergency = true;
                            is_emergency = true;
                        }else{
                            param.amode = EMERGENCY;
                            is_emergency = true;
                            reset_taskstate_bool(ts);
                            ts.initial_pos = act_eepos[ee_name];
                            ts.initial_ori = act_eequat[ee_name];
                            ts.max_em_t_count = std::floor(td.emergency_recover_time / RTC_PERIOD);
                            ts.em_transition_count = ts.max_em_t_count;
                            ts.F_em_init = ts.F_now;
                            ts.remove_static_force_count = 0; // 強制的にemergencyモードに入る
                            td.add_static_force = false; // recoverしたとき変なことにならないように
                            for(int i=0; i<param.manip->numJoints(); i++){
                                ts.emergency_q(i) = param.manip->joint(i)->q;
                            }
                        }
                    }else if(ts.vel_over_thresh){
                        for(int i=0; i<100; i++){
                            std::cout << ee_name << ": ";
                            std::cout << "Transition to CONTACT!!!!!!!!" << std::endl;
                        }
                        if(td.dual){
                            change_to_contact = true;
                        }else{
                            param.amode = MANIP_CONTACT;
                            reset_taskstate_bool(ts);
                            ts.f2c_transition_count = ts.max_f2c_t_count;
                            ts.F_em_init = ts.F_now;  //for transition from manip_free&add_static_force to manip_contact
                            if(ts.remove_static_force_count > 0){
                                ts.remove_static_force_count = 0;
                                td.add_static_force = false;
                            }
                        }
                    }
                    break;
                case(MANIP_CONTACT):
                    if(loop%300==0){
                        std::cout << "      [ltc] MODE: MANIP_CONTACT" << std::endl;
                    }
                    if(ts.torque_over_limit || ts.pos_over_limit || ts.ori_over_limit || ts.vel_over_limit){
                        if(ts.torque_over_limit){
                            for(int i=0; i<100; i++){
                                std::cout << "CONTACT to EMERGENCY because of torque limit!!!" << std::endl;
                            }
                        }
                        if(ts.pos_over_limit){
                            for(int i=0; i<100; i++){
                                std::cout << "CONTACT to EMERGENCY because of position limit!!!" << std::endl;
                            }
                        }
                        if(ts.ori_over_limit){
                            for(int i=0; i<100; i++){
                                std::cout << "CONTACT to EMERGENCY because of orientation limit!!!" << std::endl;
                            }
                        }
                        if(ts.vel_over_limit){
                            for(int i=0; i<100; i++){
                                std::cout << "CONTACT to EMERGENCY because of velocity limit!!!" << std::endl;
                            }
                        }
                        if(td.dual){
                            change_to_emergency = true;
                            is_emergency = true;
                        }else{
                            param.amode = EMERGENCY; //is_emergency is already true
                            is_emergency = true;
                            reset_taskstate_bool(ts);
                            ts.initial_pos = act_eepos[ee_name];
                            ts.initial_ori = act_eequat[ee_name];
                            ts.max_em_t_count = std::floor(td.emergency_recover_time / RTC_PERIOD);
                            ts.em_transition_count = ts.max_em_t_count;
                            ts.remove_static_force_count = 0;
                            td.add_static_force = false;
                            ts.F_em_init = ts.F_now;
                            for(int i=0; i<param.manip->numJoints(); i++){
                                ts.emergency_q(i) = param.manip->joint(i)->q;
                            }
                        }
                    }else if(ts.pos_reach_target){
                        if(td.type == MOVE_POS){
                            for(int i=0; i<100; i++){
                                std::cout << ee_name << ": ";
                                std::cout << "Reach Task Goal!!!!!!!!" << std::endl;
                            }
                            if(td.dual){
                                dual_task_succeed = true;
                            }else{
                                reset_taskstate_bool(ts);
                                {  // 自然にadd_static_forceのように移行するのがうまくいかなければこちらを復活
                                    // ts.max_em_t_count = std::floor(td.emergency_recover_time / RTC_PERIOD);
                                    // ts.em_transition_count = ts.max_em_t_count;
                                }
                                if(ts.remove_static_force_count > 0){
                                    ts.remove_static_force_count = 0;
                                    td.add_static_force = false;
                                }
                                ts.F_em_init = ts.F_now;
                                ts.task_succeed_flag = true;
                            }
                        }
                    }
                    if( ts.task_succeed_flag && ts.em_transition_count <= 0 ){
                        param.task_succeed = true;
                    }
                    break;
                case(EMERGENCY):
                    if(loop%300==0){
                        std::cout << "      [ltc] MODE: EMERGENCY" << std::endl;
                    }
                    //TODO: make recovery from emregency mode...velocity check?
                    if(ts.em_transition_count <= 0){
                        //eusからの指令待機
                        if (release_emergency_called[ee_name]){
                            reset_emergency_flag = true;
                            if(td.dual){
                                change_to_idle = true;
                            }else{
                                param.amode = IDLE_NORMAL;
                                ts.initial_pos = act_eepos[ee_name];
                                ts.initial_ori = act_eequat[ee_name];
                                ts.F_now = hrp::dvector::Zero(6);
                                reset_taskstate_bool(ts);
                                release_emergency_called[ee_name] = false;
                            }
                        }
                    }
                    break;
                }
            } //end else
        } //end if param.active
        it++;
    }
    it = m_lt_param.begin();
    // for dual arm task
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        std::string ee_name = it->first;
        if (param.is_active && !fix_mode_normal[ee_name]) {
            TaskDescription& td = limb_task_target[ee_name];
            TaskState& ts = limb_task_state[ee_name];
            if(start_emergency_called){
                change_to_emergency = true;
                is_emergency = true; //stops joint angle ref
                start_emergency_called = false;
            }else if(change_to_contact){
                param.amode = MANIP_CONTACT;
                reset_taskstate_bool(ts);
                ts.f2c_transition_count = ts.max_f2c_t_count;
                ts.F_em_init = ts.F_now;  //for transition from manip_free&add_static_force to manip_contact
                if(ts.remove_static_force_count > 0){
                    ts.remove_static_force_count = 0;
                    td.add_static_force = false;
                }
            }else if(change_to_emergency){
                ts.remove_static_force_count = 0;
                td.add_static_force = false;
                reset_taskstate_bool(ts);
                ts.initial_pos = act_eepos[ee_name];
                ts.initial_ori = act_eequat[ee_name];
                ts.max_em_t_count = std::floor(td.emergency_recover_time / RTC_PERIOD);
                ts.em_transition_count = ts.max_em_t_count;
                ts.F_em_init = ts.F_now;
                for(int i=0; i<param.manip->numJoints(); i++){
                    ts.emergency_q(i) = param.manip->joint(i)->q;
                }
                param.amode = EMERGENCY;
            }else if(change_to_idle){
                param.amode = IDLE_NORMAL;
                ts.initial_pos = act_eepos[ee_name];
                ts.initial_ori = act_eequat[ee_name];
                ts.F_now = hrp::dvector::Zero(6);
                release_emergency_called[ee_name] = false;
                reset_taskstate_bool(ts);
                param.task_succeed = false; //reset
            }else if(dual_task_succeed){ //when MANIP_CONTACT task succeed
                reset_taskstate_bool(ts);
                {  // 自然にadd_static_forceのように移行するのがうまくいかなければこちらを復活
                    // ts.max_em_t_count = std::floor(td.emergency_recover_time / RTC_PERIOD);
                    // ts.em_transition_count = ts.max_em_t_count;
                }
                if(ts.remove_static_force_count > 0){
                    ts.remove_static_force_count = 0;
                    td.add_static_force = false;
                }
                ts.F_em_init = ts.F_now;
                ts.task_succeed_flag = true;
            }
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
                    *(debug_dqest[ee_name]) << micro_time;
                    *(debug_dqact[ee_name]) << micro_time;
                    *(debug_qest[ee_name]) << micro_time;
                    *(debug_qact[ee_name]) << micro_time;
                    *(debug_qref[ee_name]) << micro_time;
                    *(debug_acteescrew[ee_name]) << micro_time;
                    *(debug_acteewrench[ee_name]) << micro_time;
                    //MOVE_POS: MANIP_FREE
                    *(debug_cdve[ee_name]) << micro_time; //check_dir_vel_err
                    *(debug_odve[ee_name]) << micro_time; //other_dir_vel_err
                    //MOVE_POS: MANIP_CONCTACT
                    *(debug_dtt[ee_name]) << micro_time; //dist_to_target
                    *(debug_pen[ee_name]) << micro_time; //pos_error_norm
                    *(debug_wrw[ee_name]) << micro_time;
                    // new eeest
                    *(debug_filtereevel[ee_name]) << micro_time;
                    *(debug_filtereef_d[ee_name]) << micro_time;
                    *(debug_filtereef_s[ee_name]) << micro_time;
                    *(debug_act_torque[ee_name]) << micro_time;
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
                        *(debug_reftq[ee_name]) << " " << reference_torque(i);
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
                        *(debug_wrw[ee_name]) << " " << world_ref_wrench[ee_name](i);
                    }
                    for (int i=0; i<3; i++){
                        *(debug_acteescrew[ee_name]) << " " << act_ee_vel[ee_name](i);
                        *(debug_acteewrench[ee_name]) << " " << abs_forces[param.sensor_name](i);
                        *(debug_filtereevel[ee_name]) << " " << filtered_ee_vel[ee_name](i);
                        *(debug_filtereef_d[ee_name]) << " " << filtered_f_d[ee_name](i);
                        *(debug_filtereef_s[ee_name]) << " " << filtered_f_s[ee_name](i);
                    }
                    for (int i=0; i<3; i++){
                        *(debug_acteescrew[ee_name]) << " " << act_ee_w[ee_name](i);
                        *(debug_acteewrench[ee_name]) << " " << abs_moments[param.sensor_name](i);
                    }
                    for (int i=0; i<limbdof; i++){
                        int jid = manip->joint(i)->jointId;
                        *(debug_eect[ee_name]) << " " << ee_compensation_torque[ee_name](i);
                        *(debug_nst[ee_name]) << " " << null_space_torque[ee_name](i);
                        *(debug_reftq[ee_name]) << " " << reference_torque(jid);
                        *(debug_dqest[ee_name]) << " " << estimated_reference_velocity(jid);
                        *(debug_dqact[ee_name]) << " " << m_robot->joint(jid)->dq;
                        *(debug_qest[ee_name]) << " " << log_est_q(jid);
                        *(debug_qact[ee_name]) << " " << log_act_q(jid);
                        *(debug_qref[ee_name]) << " " << log_ref_q(jid);
                        *(debug_act_torque[ee_name]) << " " << actual_torque_vector(jid);
                    }
                    *(debug_cdve[ee_name]) << " " << check_dir_vel_err[ee_name];
                    *(debug_odve[ee_name]) << " " << other_dir_vel_err[ee_name];
                    *(debug_dtt[ee_name]) << " " << dist_to_target[ee_name];
                    *(debug_pen[ee_name]) << " " << pos_error_norm[ee_name];
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
                    *(debug_dqest[ee_name]) << std::endl;
                    *(debug_dqact[ee_name]) << std::endl;
                    *(debug_qest[ee_name]) << std::endl;
                    *(debug_qact[ee_name]) << std::endl;
                    *(debug_qref[ee_name]) << std::endl;
                    *(debug_acteescrew[ee_name]) << std::endl;
                    *(debug_acteewrench[ee_name]) << std::endl;
                    *(debug_cdve[ee_name]) << std::endl;
                    *(debug_odve[ee_name]) << std::endl;
                    *(debug_dtt[ee_name]) << std::endl;
                    *(debug_pen[ee_name]) << std::endl;
                    *(debug_wrw[ee_name]) << std::endl;
                    *(debug_filtereevel[ee_name]) << std::endl;
                    *(debug_filtereef_d[ee_name]) << std::endl;
                    *(debug_filtereef_s[ee_name]) << std::endl;
                    *(debug_act_torque[ee_name]) << std::endl;
                }
            }
            it++;
        }
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

    // Controller mode
    if (param.is_active) i_param_.controller_mode = OpenHRP::LimbTorqueControllerService::MODE_ACTIVE;
    else i_param_.controller_mode = OpenHRP::LimbTorqueControllerService::MODE_IDLE;

    // Arm mode
    switch(param.amode){
    case 0:
        i_param_.amode = OpenHRP::LimbTorqueControllerService::IDLE_NORMAL;
    case 1:
        i_param_.amode = OpenHRP::LimbTorqueControllerService::IDLE_HARD;
    case 2:
        i_param_.amode = OpenHRP::LimbTorqueControllerService::IDLE_COMPLIANT;
    case 3:
        i_param_.amode = OpenHRP::LimbTorqueControllerService::MANIP_FREE;
    case 4:
        i_param_.amode = OpenHRP::LimbTorqueControllerService::MANIP_CONTACT;
    case 5:
        i_param_.amode = OpenHRP::LimbTorqueControllerService::EMERGENCY;
    }
    i_param_.task_succeed = param.task_succeed;
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

// start overwriting qRef with qRef+estimated_ref_vel * dt
// do not call during position control
bool LimbTorqueController::startRefdqEstimation(const std::string& i_name_)
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
bool LimbTorqueController::stopRefdqEstimation(const std::string& i_name_)
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

bool LimbTorqueController::releaseEmergency(const std::string& i_name_, bool cancel)
{
    Guard guard(m_mutex);
    std::string name = std::string(i_name_);
    if ( m_lt_param.find(name) == m_lt_param.end() ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find end effector named [" << name << "]" << std::endl;
        return false;
    }
    if(cancel){
        release_emergency_called[name] = false;
    }else{
        release_emergency_called[name] = true;
    }
    std::cout << "[ltc] release emergency mode!!" << std::endl;
    return true;
}

bool LimbTorqueController::checkEmergencyFlag(const std::string& i_name_, bool& i_flag_)
{
    std::string name = std::string(i_name_);
    if ( m_lt_param.find(name) == m_lt_param.end() ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find end effector named [" << name << "]" << std::endl;
        return false;
    }
    i_flag_ = new bool;
    i_flag_ = release_emergency_called[name];
    return true;
}

//i_name_: base file path of log
//i_logname_: what log to take; "collision" or "operational"
bool LimbTorqueController::startLog(const std::string& i_name_, const std::string& i_logname_, const std::string& i_dirname_)
{
    Guard guard(m_mutex);
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        if (param.is_active) {
            std::string ee_name = it->first;
            struct stat st;
            std::string basepath = i_name_ + std::string("/LimbTorqueControllerDebug_") + i_dirname_ + std::string("/");
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
                debug_dqest[ee_name] = new std::ofstream((logpath  + std::string("est_dq.dat")).c_str());
                debug_dqact[ee_name] = new std::ofstream((logpath  + std::string("act_dq.dat")).c_str());
                debug_qest[ee_name] = new std::ofstream((logpath  + std::string("qest.dat")).c_str());
                debug_qact[ee_name] = new std::ofstream((logpath  + std::string("qact.dat")).c_str());
                debug_qref[ee_name] = new std::ofstream((logpath  + std::string("qref.dat")).c_str());
                debug_acteescrew[ee_name] = new std::ofstream((logpath  + std::string("act_ee_screw.dat")).c_str());
                debug_acteewrench[ee_name] = new std::ofstream((logpath  + std::string("act_ee_wrench.dat")).c_str());
                debug_cdve[ee_name] = new std::ofstream((logpath + std::string("cdve.dat")).c_str());
                debug_odve[ee_name] = new std::ofstream((logpath + std::string("odve.dat")).c_str());
                debug_dtt[ee_name] = new std::ofstream((logpath + std::string("dtt.dat")).c_str());
                debug_pen[ee_name] = new std::ofstream((logpath + std::string("pen.dat")).c_str());

                debug_wrw[ee_name] = new std::ofstream((logpath + std::string("world_refwrench.dat")).c_str());
                debug_filtereevel[ee_name] = new std::ofstream((logpath + std::string("filter_eevel.dat")).c_str());
                debug_filtereef_d[ee_name] = new std::ofstream((logpath + std::string("filter_eef_d.dat")).c_str());
                debug_filtereef_s[ee_name] = new std::ofstream((logpath + std::string("filter_eef_s.dat")).c_str());
                debug_act_torque[ee_name] = new std::ofstream((logpath + std::string("act_torque.dat")).c_str());
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
                delete debug_dqest[ee_name];
                delete debug_dqact[ee_name];
                delete debug_qest[ee_name];
                delete debug_qact[ee_name];
                delete debug_qref[ee_name];
                delete debug_acteescrew[ee_name];
                delete debug_acteewrench[ee_name];
                delete debug_cdve[ee_name];
                delete debug_odve[ee_name];
                delete debug_dtt[ee_name];
                delete debug_pen[ee_name];
                delete debug_wrw[ee_name];
                delete debug_filtereevel[ee_name];
                delete debug_filtereef_d[ee_name];
                delete debug_filtereef_s[ee_name];
                delete debug_act_torque[ee_name];
            }
        }
        it++;
    }
    std::cout << "[ltc] successfully stop log!!" << std::endl;
    return true;
}

bool LimbTorqueController::giveTaskDescription(const std::string& i_name_, OpenHRP::LimbTorqueControllerService::taskDescription task_description)
{
    Guard guard(m_mutex);
    std::string name = std::string(i_name_);
    if ( m_lt_param.find(name) == m_lt_param.end() ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find limb torque controller param [" << name << "]" << std::endl;
        return false;
    }
    TaskDescription& td = limb_task_target[name];
    TaskState& ts = limb_task_state[name];
    bool static_force_is_added = false;
    if( ts.f2c_transition_count > 0 || ts.em_transition_count > 0 || ts.remove_static_force_count > 0){
        std::cerr << "[" << m_profile.instance_name << "] You must not call giveTaskdescription while transition!! fail:  [" << name << "]" << std::endl;
        return false;
    }
    if( !ts.task_succeed_flag && m_lt_param[name].amode == MANIP_CONTACT ){
        std::cerr << "[" << m_profile.instance_name << "] You must not call giveTaskdescription while contact task!! fail:  [" << name << "]" << std::endl;
        return false;
    }
    switch(task_description.type){
    case(MOVE_POS):
        td.type = MOVE_POS;
        break;
    case(FIX):
        td.type = FIX;
        break;
    case(MOTION_ONLY):
        td.type = MOTION_ONLY;
        break;
    }
    td.dual = task_description.dual;
    // set task descriptions
    td.velocity_check_dir.normalize();
    for(int i=0; i<6; i++){
        td.F_init(i) = task_description.F_init[i];
    }
    for(int i=0; i<3; i++){
        td.velocity_check_dir(i) = task_description.velocity_check_dir[i];
        td.rel_pos_target(i) = task_description.rel_pos_target[i];
    }
    td.vel_check_thresh = task_description.vel_check_thresh;
    td.vel_check_limit = task_description.vel_check_limit;
    td.cont_vel_error_limit = task_description.cont_vel_error_limit;
    td.pos_target_thresh = task_description.pos_target_thresh;
    td.pos_error_limit = task_description.pos_error_limit;
    td.emergency_recover_time = task_description.emergency_recover_time;
    if(td.add_static_force){
        static_force_is_added = true;
    }
    td.add_static_force = task_description.add_static_force;
    td.static_rfu_gain = task_description.static_rfu_gain;
    // if add_static_force, start force transition
    reset_taskstate_bool(ts);
    if(static_force_is_added && !task_description.add_static_force){ //static_forceを消す遷移に入る
        for(int i=0; i<100; i++){
            std::cout << name  << ": stf transition to 0!!!!!!" << std::endl;
        }
        ts.remove_static_force_count = max_rsfc; //1.5秒かけてremove //ReferenceForceUpdaterを参照
        ts.F_em_init.head(3) = ts.F_now.head(3);
    } //その他::そのまま継続
    // initialize task state
    // set task goals
    switch(task_description.type){
    case(MOVE_POS):{
        ts.initial_pos = ref_eepos[name];
        ts.world_pos_target = ref_eepos[name] + ref_eeR[name] * td.rel_pos_target;
        ts.world_pos_targ_dir = (ref_eeR[name] * td.rel_pos_target).normalized();
        ts.F_eeR = ref_eeR[name];
        break;
    }
    case(FIX):
        break;
    case(MOTION_ONLY):
        break;
    }
    m_lt_param[name].amode = MANIP_FREE; //automatically set to manip_normal (is this ok?)
    m_lt_param[name].task_succeed = false; //reset
    std::cout << "[" << m_profile.instance_name << "] successfully set task description for " << name << std::endl;
    return true;
}

void LimbTorqueController::copyTaskDescription(OpenHRP::LimbTorqueControllerService::taskDescription& i_taskd_, const TaskDescription& param)
{
    switch(param.type){
    case(MOVE_POS):
        i_taskd_.type = OpenHRP::LimbTorqueControllerService::MOVE_POS;
        break;
    case(FIX):
        i_taskd_.type = OpenHRP::LimbTorqueControllerService::FIX;
        break;
    case(MOTION_ONLY):
        i_taskd_.type = OpenHRP::LimbTorqueControllerService::MOTION_ONLY;
        break;
    }
    i_taskd_.dual = param.dual;
    for(int i=0; i<6; i++){
        i_taskd_.F_init[i] = param.F_init(i);
    }
    i_taskd_.vel_check_thresh = param.vel_check_thresh;
    i_taskd_.vel_check_limit = param.vel_check_limit;
    i_taskd_.cont_vel_error_limit = param.cont_vel_error_limit;
    i_taskd_.pos_target_thresh = param.pos_target_thresh;
    i_taskd_.pos_error_limit = param.pos_error_limit;
    for(int i=0; i<3; i++){
        i_taskd_.velocity_check_dir[i] = param.velocity_check_dir(i);
        i_taskd_.rel_pos_target[i] = param.rel_pos_target(i);
    }
    i_taskd_.emergency_recover_time = param.emergency_recover_time;
    i_taskd_.add_static_force = param.add_static_force;
    i_taskd_.static_rfu_gain = param.static_rfu_gain;
}

bool LimbTorqueController::getTaskDescription(const std::string& i_name_, OpenHRP::LimbTorqueControllerService::taskDescription_out i_taskd_)
{
    std::string name = std::string(i_name_);
    if ( m_lt_param.find(i_name_) == m_lt_param.end() ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find task description [" << i_name_ << "]" << std::endl;
        i_taskd_ = new OpenHRP::LimbTorqueControllerService::taskDescription();
        copyTaskDescription(*i_taskd_, TaskDescription());
        return false;
    }
    i_taskd_ = new OpenHRP::LimbTorqueControllerService::taskDescription();
    i_taskd_->velocity_check_dir.length(3);
    i_taskd_->F_init.length(6);
    i_taskd_->rel_pos_target.length(3);
    copyTaskDescription(*i_taskd_, limb_task_target[name]);
    return true;
}

void LimbTorqueController::copyTaskState(OpenHRP::LimbTorqueControllerService::taskState& i_tasks_, const TaskState& param)
{
    i_tasks_.pos_over_limit = param.pos_over_limit;
    i_tasks_.pos_reach_target = param.pos_reach_target;
    i_tasks_.ori_over_limit = param.ori_over_limit;
    i_tasks_.vel_over_thresh = param.vel_over_thresh;
    i_tasks_.vel_over_limit = param.vel_over_limit;
    i_tasks_.torque_over_limit = param.torque_over_limit;
    i_tasks_.f2c_transition_count = param.f2c_transition_count;
    i_tasks_.max_f2c_t_count = param.max_f2c_t_count;
    i_tasks_.em_transition_count = param.em_transition_count;
    i_tasks_.max_em_t_count = param.max_em_t_count;
    i_tasks_.initial_ori[0] = param.initial_ori.w();
    for(int i=0; i<3; i++){
        i_tasks_.world_pos_target[i] = param.world_pos_target(i);
        i_tasks_.world_pos_targ_dir[i] = param.world_pos_targ_dir(i);
        i_tasks_.world_ori_targ_dir[i] = param.world_ori_targ_dir(i);
        i_tasks_.initial_pos[i] = param.initial_pos(i);
        i_tasks_.initial_ori[i+1] = param.initial_ori.vec()(i);
        for(int j=0; j<3; j++){
            i_tasks_.F_eeR[i*3+j] = param.F_eeR(i, j);
        }
    }
    for(int i=0; i<6; i++){
        i_tasks_.F_now[i] = param.F_now(i);
        i_tasks_.F_em_init[i] = param.F_em_init(i);
    }
    for(int i=0; i<param.emergency_q.size(); i++){
        i_tasks_.emergency_q[i] = param.emergency_q(i);
    }
    i_tasks_.task_succeed_flag = param.task_succeed_flag;
}

bool LimbTorqueController::getTaskState(const std::string &i_name_, OpenHRP::LimbTorqueControllerService::taskState_out i_tasks_)
{
    std::string name = std::string(i_name_);
    if ( limb_task_state.find(i_name_) == limb_task_state.end() ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find task state [" << i_name_ << "]" << std::endl;
        i_tasks_ = new OpenHRP::LimbTorqueControllerService::taskState();
        copyTaskState(*i_tasks_, TaskState());
        return false;
    }
    i_tasks_ = new OpenHRP::LimbTorqueControllerService::taskState();
    i_tasks_->world_pos_target.length(3);
    i_tasks_->world_pos_targ_dir.length(3);
    i_tasks_->world_ori_targ_dir.length(3);
    i_tasks_->initial_pos.length(3);
    i_tasks_->F_eeR.length(9);
    i_tasks_->F_em_init.length(6);
    i_tasks_->F_now.length(6);
    i_tasks_->initial_ori.length(4);
    i_tasks_->emergency_q.length(m_lt_param[name].manip->numJoints());
    copyTaskState(*i_tasks_, limb_task_state[name]);
    return true;
}

bool LimbTorqueController::startModeChange(const std::string &i_name_)
{
    Guard guard(m_mutex);
    std::string name = std::string(i_name_);
    if ( limb_task_state.find(i_name_) == limb_task_state.end() ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find end effector [" << i_name_ << "]" << std::endl;
        return false;
    }
    fix_mode_normal[name] = false;
    reset_taskstate_bool(limb_task_state[name]);
    std::cout << "[" << m_profile.instance_name << "] Start watching error and changing mode for " << i_name_ << "!" << std::endl;
    return true;
}

bool LimbTorqueController::stopModeChange(const std::string &i_name_)
{
    Guard guard(m_mutex);
    std::string name = std::string(i_name_);
    if ( limb_task_state.find(i_name_) == limb_task_state.end() ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find end effector [" << i_name_ << "]" << std::endl;
        return false;
    }
    fix_mode_normal[name] = true;
    reset_taskstate_bool(limb_task_state[name]);
    std::cout << "[" << m_profile.instance_name << "] Stop watching error and changing mode for " << i_name_ << "!" << std::endl;
    return true;
}

//強制的にEmergencyモードに入る(modechange始まるまでは呼べないようにしてある)
bool LimbTorqueController::startEmergency()
{
    Guard guard(m_mutex);
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        std::string ee_name = it->first;
        LTParam& param = it->second;
        if( param.is_active && fix_mode_normal[ee_name] ){
            std::cerr << "[" << m_profile.instance_name << "] Mode change has not been started at end effector [" << ee_name << "]" << std::endl;
            return false;
        }
        it++;
    }
    start_emergency_called = true;
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

// Do not use this for now: it is not compatible with reference torque regulator for now
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

//under development
// assuming isotropic ee impedance gain
void LimbTorqueController::estimateEEVelForce()
{
    std::map<std::string, LTParam>::iterator it = m_lt_param.begin();
    while(it != m_lt_param.end()){
        LTParam& param = it->second;
        std::string ee_name = it->first;
        if (param.is_active) {
            if (DEBUGP) {
                std::cerr << "ここにデバッグメッセージを流す" << std::endl;
            }
            if(!eeest_initialized[ee_name]){
                estimateEEVelForce_init(it);
                eeest_initialized[ee_name] = true;
            }
            const hrp::Matrix33 Iden33 = hrp::Matrix33::Identity();
            hrp::Vector3 prior_state_est;
            hrp::Matrix33 state_update_matrix, prior_error_covar;
            Matrix32 kalman_gain;
            Vector2 observation;
            double eepgain_value = param.ee_pgain_p(0,0); // assuming isotropic ee impedance gain
            double eedgain_value = param.ee_dgain_p(0,0); // assuming isotropic ee impedance gain
            ee_state_coeff[ee_name] <<
                1.0, RTC_PERIOD/virtual_ee_mass_for_vel[ee_name], 0.0,
                -eepgain_value*RTC_PERIOD, 1.0 - eedgain_value*RTC_PERIOD/virtual_ee_mass[ee_name], 0.0,
                0.0, 0.0, 1.0;
            // x, y, z
            for (int i=0; i<3; i++){
                // prediction step
                prior_state_est = ee_state_coeff[ee_name] * ee_state_est[ee_name][i];
                prior_error_covar = ee_state_coeff[ee_name] * ee_error_covar[ee_name][i] * ee_state_coeff[ee_name].transpose() + ee_system_noise[ee_name];
                // filtering step
                observation << (act_ee_vel[ee_name](i) - ref_ee_vel[ee_name](i)), abs_forces[param.sensor_name](i);
                kalman_gain = prior_error_covar * ee_obs_coeff[ee_name].transpose() * ( ee_obs_coeff[ee_name] * prior_error_covar * ee_obs_coeff[ee_name].transpose() + ee_obs_noise[ee_name] ).inverse();
                ee_state_est[ee_name][i] = prior_state_est + kalman_gain * (observation - ee_obs_coeff[ee_name] * prior_state_est);
                ee_error_covar[ee_name][i] = (Iden33 - kalman_gain * ee_obs_coeff[ee_name]) * prior_error_covar;
            }
            // reorganize variants into screw and wrench
            for(int i=0; i<3; i++){
                filtered_ee_vel[ee_name](i) = ee_state_est[ee_name][i](0) + ref_ee_vel[ee_name](i);
                filtered_f_d[ee_name](i) = ee_state_est[ee_name][i](1);
                filtered_f_s[ee_name](i) = ee_state_est[ee_name][i](2);
            }
        }
        it++;
    }
}

void LimbTorqueController::estimateEEVelForce_init(const std::map<std::string, LTParam>::iterator it)
{
    std::string ee_name = it->first;
    if (DEBUGP) {
        std::cerr << "ここにデバッグメッセージを流す" << std::endl;
    }
    const hrp::Matrix33 Iden33 = hrp::Matrix33::Identity();
    const hrp::Vector3 zv3 = hrp::Vector3::Zero();
    // set iteratively updated parameters
    for (int i=0; i<3; i++){
        ee_state_est[ee_name].push_back(zv3);
        ee_error_covar[ee_name].push_back(Iden33); //change value?
    }
    // set fixed values
    virtual_ee_mass[ee_name] = 20.0; //kg
    virtual_ee_mass_for_vel[ee_name] = 1.0; //kg
    ee_obs_coeff[ee_name] <<
        1.0, 0.0, 0.0,
        0.0, 1.0, 1.0;
    ee_system_noise[ee_name] <<
        1e-4, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 5e-2;
    ee_obs_noise[ee_name] <<
        4e-2, 0.0,
        0.0, 1.0;
}
