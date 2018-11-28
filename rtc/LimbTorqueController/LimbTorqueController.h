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

#include <deque>
#include <sys/stat.h>
#include <sys/types.h>
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

typedef Eigen::Matrix<double, 2, 2> Matrix22;
typedef Eigen::Matrix<double, 2, 1> Vector2;

// minimum-root-mean-square-error filter for first order differenciation of Vector3
// .      3x_k - 4x_(k-N) + x_(k-2N)
// x_k = -----------------------------
//                2N*dt
// N: half the window size
template <typename T>
class RMSfilter
{
public:
    RMSfilter() {}

    RMSfilter(unsigned int n, double t)
    {
        N_ = n;
        dt_ = t;
        data_que_.resize(2*N_);
    }

    void push(const T& obj)
    {
        data_que_.pop_front();
        data_que_.push_back(obj);
    }
    void fill(const T& obj)
    {
        for (int i=0; i<2*N_; i++){
            push(obj);
        }
    }
    T get_velocity()
    {
        T velocity = (3.0*data_que_[2*N_-1] - 4.0*data_que_[N_-1] + data_que_[0]) / (2.0*N_*dt_);
        return velocity;
    }
    int N() const { return N_; }
    double dt() const { return dt_; }
    std::deque<T> data_que() const { return data_que_; }

private:
    int N_; // window_size
    double dt_;
    std::deque<T> data_que_;
};

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
    bool getCollisionStatus(const std::string& i_name_, LimbTorqueControllerService::collisionStatus_out i_param_);
    bool startLog(const std::string& i_name_, const std::string& i_logname_);
    bool stopLog();
    bool startRefdqEstimation(const std::string& i_name_);
    bool stopRefdqEstimation(const std::string& i_name_);

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
        hrp::JointPathExPtr manip;
        bool is_active;
        hrp::Matrix33 ee_pgain_p, ee_pgain_r;
        hrp::Matrix33 ee_dgain_p, ee_dgain_r;
    };
    struct ee_trans {
        std::string target_name;
        hrp::Vector3 localPos;
        hrp::Matrix33 localR;
    };
    struct CollisionParam{
        hrp::dvector collision_threshold;
        double cgain, resist_gain;  //collision observer gain, collision resistance gain
        int max_collision_uncheck_count;
        int check_mode; //{0,1,2,3} = {no check, general momentum method, simple method, directional}
        int handle_mode; //{0,1,2} = {no check, shock absorption, shock resistance}
    };

    void copyLimbTorqueControllerParam (OpenHRP::LimbTorqueControllerService::limbtorqueParam& i_param_, const LTParam& param);
    void copyCollisionParam(LimbTorqueControllerService::collisionParam& i_param_, const CollisionParam& param);
    void getTargetParameters();
    void calcForceMoment();
    void getActualParameters();
    void calcLimbInverseDynamics();
    void calcMinMaxAvoidanceTorque();
    void calcEECompensation();
    void calcNullJointDumping();
    void estimateRefdq();

    std::map<std::string, LTParam> m_lt_param, m_ref_lt_param;
    std::map<std::string, CollisionParam> m_lt_col_param;
    std::map<std::string, ee_trans> ee_map;
    std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
    std::map<std::string, hrp::Vector3> abs_forces, abs_moments, abs_ref_forces, abs_ref_moments;
    std::map<std::string, hrp::dmatrix> act_ee_jacobian, inv_ee_jacobian;
    //for ee compensation
    std::map<std::string, hrp::Vector3> ee_pos_comp_force, ee_ori_comp_moment, ee_vel_comp_force, ee_w_comp_moment;
    std::map<std::string, hrp::dvector> ee_pos_ori_comp_wrench, ee_vel_w_comp_wrench;
    std::map<std::string, hrp::dvector> ee_compensation_torque;
    std::map<std::string, hrp::Vector3> ee_pos_error, ee_ori_error;
    std::map<std::string, hrp::Matrix33> current_act_ee_rot, current_ref_ee_rot, prev_ref_ee_rot;
    std::map<std::string, hrp::Vector3> current_act_ee_pos, current_ref_ee_pos, prev_ref_ee_pos;
    std::map<std::string, hrp::Vector3> act_ee_vel, ref_ee_vel, act_ee_w, ref_ee_w;
    std::map<std::string, hrp::Vector3> ee_vel_error, ee_w_error;
    std::map<std::string, RMSfilter<hrp::Vector3> > ee_vel_filter;
    std::map<std::string, RMSfilter<hrp::Matrix33> > ee_w_filter;
    std::map<std::string, bool> oscontrol_initialized;
    //for null space torque
    std::map<std::string, hrp::dvector> null_space_torque;
    //basic models, necessities
    double m_dt;
    hrp::BodyPtr m_robot;
    hrp::BodyPtr m_robotRef;
    coil::Mutex m_mutex;
    hrp::dvector qold, qoldRef, dqold, dqoldRef;
    hrp::Matrix33 target_root_R;
    hrp::Vector3 target_root_p;
    unsigned int m_debugLevel;
    int dummy;
    unsigned int loop;
    std::vector<double> default_pgain, default_dgain;
    std::vector<double> temp_ref_vel, temp_ref_acc, temp_vel, temp_acc, temp_ref_u, temp_u; //for calcLimbInverseDynamics
    std::vector<double> temp_invdyn_result;

    void CollisionDetector();
    void calcGeneralizedInertiaMatrix(std::map<std::string, LTParam>::iterator it);
    std::map<std::string, hrp::dmatrix> gen_imat, old_gen_imat; //generalized inertia matrix
    std::map<std::string, hrp::dvector> gen_mom, old_gen_mom, gen_mom_res; //generalized momentum, and its residual
    std::map<std::string, hrp::dvector> accum_tau, accum_beta, accum_res, initial_gen_mom;
    std::map<std::string, int> collision_uncheck_count;
    std::map<std::string, std::string> collision_link;
    std::map<std::string, bool> collision_detector_initialized, gen_imat_initialized;
    std::vector<hrp::dmatrix> link_inertia_matrix; //6D inertia matrix
    std::vector<double> default_cgain, default_rgain;

    // collision-param
    std::map<std::string, hrp::dmatrix> gen_mom_observer_gain, collision_resistance_gain;
    std::map<std::string, hrp::dvector> default_collision_threshold;
    int max_collision_uncheck_count;
    hrp::dvector actual_torque_vector;

    // for debug log
    std::map<std::string, std::ofstream*> debug_mom, debug_actau, debug_acbet, debug_acres, debug_res, debug_reftq, debug_f; //CollisionDetector
    std::map<std::string, std::ofstream*> debug_ee_pocw, debug_ee_vwcw, debug_eect, debug_nst, debug_acteevel, debug_refeevel, debug_acteew, debug_refeew; //calcEECompensation
    std::map<std::string, std::ofstream*> debug_ee_poserror, debug_ee_orierror, debug_ee_velerror, debug_ee_werror; //calcEECompensation
    std::map<std::string, std::ofstream*> debug_dqest, debug_dqact, debug_qest, debug_qact, debug_qref; //estimateRefdq
    std::map<std::string, std::ofstream*> debug_acteescrew, debug_esteescrew, debug_acteewrench, debug_esteewrench; //ee vel&force estimation
    void DebugOutput();
    bool spit_log;
    int log_type; //1:collision, 2:operational

    // reference velocity estimation
    hrp::dvector overwritten_qRef_prev, estimated_reference_velocity, transition_velest, log_est_q, log_act_q, log_ref_q;
    // IIR filter parameters (for velocity estimation)
    double iir_cutoff_frequency, iir_alpha, iir_a0, iir_a1, iir_a2, iir_b1, iir_b2;
    std::map<std::string, hrp::dvector> velest_now, velest_prev, velest_prevprev, imp_now, imp_prev, imp_prevprev; //estimated joint velocities and velocity generated by impedance(ee compensation)
    std::map<std::string, bool> velest_initialized, overwrite_refangle;
    std::map<std::string, int> stop_overwriting_q_transition_count;
    int max_stop_overwriting_q_transition_count;

    // end-effector screw and wrench estimation
    std::map<std::string, bool> eeest_initialized;
    std::map<std::string, std::vector<Vector2> > trans_est, rot_est;
    std::map<std::string, std::vector<Matrix22> > trans_covariance, rot_covariance;
    std::map<std::string, hrp::Matrix33> impedance_mass_mat, impedance_inertia_mat;
    std::map<std::string, Matrix22>  translational_system_noise_matrix, rotational_system_noise_matrix, translational_observation_noise_matrix, rotational_observation_noise_matrix;
    std::map<std::string, hrp::dvector> filtered_screw, filtered_wrench;
    void estimateEEVelForce();
    void estimateEEVelForce_init(const std::map<std::string, LTParam>::iterator it);

    // disturbance observer
    std::map<std::string, hrp::Vector3> velocity_discrepancy, force_increase, prev_filtered_force;
    std::map<std::string, bool> dist_obs_initialized;
    std::map<std::string, std::ofstream*> debug_vel_discrepancy, debug_force_inc;
    void DisturbanceObserver();
};

extern "C"
{
    void LimbTorqueControllerInit(RTC::Manager* manager);
};

#endif // LIMBTORQUE_H
