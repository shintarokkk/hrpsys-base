// -*- C++ -*-
/*!
 * @file  LimbTorqueController.h
 * @brief LimbTorqueController component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef LIMBTORQUE_H
#define LIMBTORQUE_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>
#include "../ImpedanceController/JointPathEx.h"

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "LimbTorqueControllerService_impl.h"

class LimbTorqueController
 : public RTC::DataFlowComponentBase
{
public:

    LimbTorqueController(RTC::Manager* manager);
    virtual ~LimbTorqueController();

    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onFinalize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

    //start/stop controller, set/get parameter function
    bool startLimbTorqueController(const std::string& i_name_);
    bool stopLimbTorqueController(const std::string& i_name_);
    bool setLimbTorqueControllerParam(const std::string& i_name_, OpenHRP::LimbTorqueControllerService::limbtorqueParam i_param_);
    bool getLimbTorqueControllerParam(const std::string& i_name_, OpenHRP::LimbTorqueControllerService::limbtorqueParam& i_param_);
    bool setCollisionParam(const std::string& i_name_, OpenHRP::LimbTorqueControllerService::collisionParam i_param_);
    bool getCollisionParam(const std::string& i_name_, LimbTorqueControllerService::collisionParam_out i_param_);
    bool getCollisionTorque(const std::string& i_name_, OpenHRP::LimbTorqueControllerService::DblSequence_out c_vec_);
    bool startLog();
    bool stopLog();

protected:

    RTC::TimedDoubleSeq m_qCurrent;
    RTC::InPort<RTC::TimedDoubleSeq> m_qCurrentIn;
    RTC::TimedDoubleSeq m_dqCurrent;
    RTC::InPort<RTC::TimedDoubleSeq> m_dqCurrentIn;
    RTC::TimedDoubleSeq m_tqCurrent;
    RTC::InPort<RTC::TimedDoubleSeq> m_tqCurrentIn;
    RTC::TimedDoubleSeq m_qRef;
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;
    RTC::TimedDoubleSeq m_tqRef;
    RTC::InPort<RTC::TimedDoubleSeq> m_tqRefIn;
    RTC::TimedPoint3D m_basePos;
    RTC::InPort<RTC::TimedPoint3D> m_basePosIn;
    RTC::TimedOrientation3D m_baseRpy;
    RTC::InPort<RTC::TimedOrientation3D> m_baseRpyIn;
    std::vector<RTC::TimedDoubleSeq> m_force;
    std::vector<RTC::InPort<RTC::TimedDoubleSeq> *> m_forceIn;
    RTC::TimedOrientation3D m_rpy;
    RTC::InPort<RTC::TimedOrientation3D> m_rpyIn;

    RTC::TimedDoubleSeq m_q;
    RTC::OutPort<RTC::TimedDoubleSeq> m_qOut;
    RTC::TimedDoubleSeq m_tq;
    RTC::OutPort<RTC::TimedDoubleSeq> m_tqOut;

    RTC::CorbaPort m_LimbTorqueControllerServicePort;

    LimbTorqueControllerService_impl m_service0;

private:

    struct LTParam {
        std::string target_name; //Name of end link
        std::string ee_name; // Name of ee (e.g., rleg, lleg, ...)
        std::string sensor_name; // Name of force sensor in the limb
        double pgain, dgain;
        hrp::Vector3 gravitational_acceleration; //重力加速度hrpsys全体のがあればそっちを使う
        hrp::JointPathExPtr manip;  //manipulator?
        // std::vector<Eigen::MatrixXd> basic_jacobians, inertia_matrices;
        // Eigen::MatrixXd gen_inertia_matrix;
        bool is_active;
        //TODO
        hrp::Matrix33 force_gain, moment_gain;
    };
    struct ee_trans {
        std::string target_name;
        hrp::Vector3 localPos;
        hrp::Matrix33 localR;
    };
    enum {CALC_TORQUE, REF_TORQUE} torque_output_type;
    struct CollisionParam{
        hrp::dvector collision_threshold;
        double cgain, resist_gain;  //collision observer gain, collision resistance gain
        bool collisionhandle_p;
        int max_collision_uncheck_count;
        bool test_bool1;
        int test_int1;
    };

    void copyLimbTorqueControllerParam (OpenHRP::LimbTorqueControllerService::limbtorqueParam& i_param_, const LTParam& param);
    void copyCollisionParam(LimbTorqueControllerService::collisionParam& i_param_, const CollisionParam& param);
    void getTargetParameters();
    void getActualParameters();
    void calcLimbInverseDynamics();
    void calcGravityCompensation();
    //void calcInertiaCompensation();
    void calcJointDumpingTorque();
    void calcMinMaxAvoidanceTorque();
    void addDumpingToRefTorque();
    void SimpleCollisionDetector();

    std::map<std::string, LTParam> m_lt_param, m_ref_lt_param;
    std::map<std::string, CollisionParam> m_lt_col_param;
    std::map<std::string, ee_trans> ee_map;
    std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
    std::map<std::string, hrp::Vector3> abs_forces, abs_moments, abs_ref_forces, abs_ref_moments;
    double m_dt;
    hrp::BodyPtr m_robot;
    hrp::BodyPtr m_robotRef;
    coil::Mutex m_mutex;
    hrp::dvector qold, qoldRef, dqold, dqoldRef;
    unsigned int m_debugLevel;
    int dummy;
    unsigned int loop;
    std::vector<double> default_pgain, default_dgain;

    void CollisionDetector();
    void CollisionHandler();
    void calcGeneralizedInertiaMatrix(std::map<std::string, LTParam>::iterator it);
    std::map<std::string, hrp::dmatrix> gen_imat, old_gen_imat; //generalized inertia matrix
    std::map<std::string, hrp::dmatrix> limb_inertia_matrix;
    std::map<std::string, hrp::dvector> gen_mom, old_gen_mom, gen_mom_res; //generalized momentum, and its residual
    std::map<std::string, hrp::dvector> accum_tau, accum_beta, accum_res, initial_gen_mom;
    std::map<std::string, bool> collision_p;
    std::map<std::string, hrp::dvector> resist_of_one_step_before;
    std::vector<hrp::dmatrix> link_inertia_matrix; //6D inertia matrix
    std::ofstream debug_mom, debug_actau, debug_acbet, debug_acres;
    std::ofstream debug_res, debug_reftq, debug_f;
    void DebugOutput();
    std::vector<double> default_cgain, default_rgain;
    bool collision_detector_initialized, gen_imat_initialized;
    int collision_uncheck_count;

    //collision-param
    std::map<std::string, hrp::dmatrix> gen_mom_observer_gain, collision_resistance_gain;
    std::map<std::string, hrp::dvector> default_collision_threshold;
    int max_collision_uncheck_count;
    hrp::dvector actual_torque_vector;
    bool spit_log;

    // double exec_time1, exec_time2;
    // double get_dtime();
};

extern "C"
{
    void LimbTorqueControllerInit(RTC::Manager* manager);
};

#endif // LIMBTORQUE_H
