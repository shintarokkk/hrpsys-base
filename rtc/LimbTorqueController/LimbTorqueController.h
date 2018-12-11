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
    bool releaseEmergency(const std::string& i_name_, bool cancel); //現在に近い関節角を送ってから実行する
    bool giveTaskDescription(const std::string& i_name_, OpenHRP::LimbTorqueControllerService::taskDescription task_description);
    bool getTaskDescription(const std::string& i_name_, OpenHRP::LimbTorqueControllerService::taskDescription_out i_taskd_);
    bool getTaskState(const std::string &i_name_, OpenHRP::LimbTorqueControllerService::taskState_out i_tasks_);
    bool startModeChange(const std::string &i_name_);
    bool stopModeChange(const std::string &i_name_);

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

    enum arm_mode {IDLE_NORMAL, IDLE_HARD, IDLE_COMPLIANT, MANIP_FREE, MANIP_CONTACT, EMERGENCY};

    struct LTParam {
        std::string target_name; //Name of end link
        std::string ee_name; // Name of ee (e.g., rleg, lleg, ...)
        std::string sensor_name; // Name of force sensor in the limb
        double pgain, dgain;
        hrp::Vector3 gravitational_acceleration; //重力加速度hrpsys全体のがあればそっちを使う
        hrp::JointPathExPtr manip;
        bool is_active;
        arm_mode amode;
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

    // MOVE: シャベルを押し込んだり物を手だけ押したり
    // POS, ROT, POSROT: 目標とるすものが位置か、回転か、両方か
    // FIX: 手でものを持った体相対の位置は固定しながら歩いたり、人間の作業のためにものを支えたり
    enum task_target_type {MOVE_POS, MOVE_ROT, MOVE_POSROT, FIX};
    // wordings: limit->desired to be avoided | threshold->desired to reach
    struct TaskDescription{
        task_target_type type;
        bool dual;
        hrp::Vector3 velocity_check_dir; //velocity error checking direction to transition to MANIP_CONTACT TODO: angular velocity もチェック?
        hrp::dvector F_init; //initial reference force
        double vel_force_gain; //F_now = F_max - vel_force_gain*act_ee_vel
        double w_force_gain; //F_now = F_max - w_force_gain*act_ee_w
        hrp::Vector3 rel_pos_target; //position target relative to initial state //in ee local at initial state
        hrp::dquaternion rel_ori_target; //orientation target relative to initial state //in ee local at initial state
        // targetのpos_reach_thresh mm、ori_error_thresh degに手先が近づいたら目標達成とする
        // 手先現在地とtargetを結ぶ直線に垂直な平面上に射影した手先位置がpos_error_threshだけ遠ざかったらemergency
        // 手先現在地とtarget間の回転をrpyで表した時、目標のrpyベクトルに垂直な平面上に射影した手先角度がori_error_threshだけ遠ざかったらemergency
        double vel_check_thresh; //velocity error threshold from MANIP_FREE to MANIP_CONTACT
        double vel_check_limit; //velocity error threshold from MANIP_FREE to EMERGENCY
        double cont_vel_error_limit; //velocity threshold from MANIP_CONTACT to EMERGENCY
        double cont_w_error_limit; //angular velocity threshold from MANIP_CONTACT to EMERGENCY
        double pos_target_thresh; //position distance threshold from MANIP_CONTACT to IDLE_NORMAL
        double pos_error_limit; //position error threshold from MANIP_CONTACT to EMERGENCY
        double ori_target_thresh; //orientation distance threshold from MANIP_CONTACT to IDLE_NORMAL //=cos(x/2) x is threshold in radian
        double ori_error_limit; //orientation error limit from MANIP_CONTACT to EMERGENCY //=cos(x/2) where x is limit in radian
    };

    // MANIP_FREE, MANIP_CONTACTモード中に更新していくエラー等の状態->やっぱり全てのモードで使う
    struct TaskState{
        hrp::Vector3 world_pos_target; //ee position target in MANIP_CONTACT
        hrp::dquaternion world_ori_target; //ee orientation target in MANIP_CONTACT
        hrp::Vector3 world_pos_targ_dir; //normalized direction of target from initial state: in MANIP_CONTACT
        hrp::Vector3 world_ori_targ_dir; //normalized direction of target from initial state: in MANIP_CONTACT: use only for angular velocity
        hrp::dvector F_now; //current reference force
        bool pos_over_limit; //whether position error exceeded limit or not
        bool pos_reach_target; //whether position reached near target or not
        bool ori_over_limit; //whether orientation error exceeded limit or not
        bool ori_reach_target; //whether orientation reached near target or not
        bool vel_over_thresh; // wheter ee velocity exceeds threshold or not (from MANIP_FREE to MANIP_CONTACT) (from IDLE_COMPLIANT to IDLE_NORMAL)
        bool vel_over_limit; // whether ee velocity exceeds limit or not (in MANIP_CONTACT and MANIP_FREE)
        bool w_over_limit; // whether ee angular velocity exceeds limit or not (in MANIP_CONTACT and MANIP_FREE)
        bool torque_over_limit; // whether joint torque exceeds limit or not (in MANIP_CONTACT)
        hrp::Matrix33 F_eeR; //end-effector rotation for reference force in EmergencyEECompensation
        int f2c_transition_count; //transition from MANIP_FREE to MANIP_CONTACT (F_now = F_init @ count==0)
        int max_f2c_t_count; //maximum value of f2c_transition_count
        int em_transition_count; //transition to EMERGENCY: releaving reference force
        int max_em_t_count; //maximum value of em_transition_count
        hrp::dvector F_em_init; //F_now at the moment of emergency transition
        double init_point_vel, init_point_w; //ee velocity to target direction at the time F_now reached F_init
        hrp::Vector3 initial_pos; //act ee pos(in world coordinate) at the moment of mode transition
        hrp::dquaternion initial_ori; //act ee orientation(in world coordinate) at the moment of mode transition
        hrp::dvector emergency_q; //joint angles at the moment of transition to emrgency mode
    };
    std::map<std::string, TaskDescription> limb_task_target;
    std::map<std::string, TaskState> limb_task_state;

    void copyLimbTorqueControllerParam (OpenHRP::LimbTorqueControllerService::limbtorqueParam& i_param_, const LTParam& param);
    void copyCollisionParam(LimbTorqueControllerService::collisionParam& i_param_, const CollisionParam& param);
    void copyTaskDescription(OpenHRP::LimbTorqueControllerService::taskDescription& i_param_, const TaskDescription& param);
    void copyTaskState(OpenHRP::LimbTorqueControllerService::taskState& i_tasks_, const TaskState& param);
    void getTargetParameters();
    void calcForceMoment();
    void getActualParameters();
    void getEEStates();
    void calcRefTorque();
    void calcLimbInverseDynamics(std::map<std::string, LTParam>::iterator it);
    void calcEmergencyLimbInverseDynamics(std::map<std::string, LTParam>::iterator it);
    void calcEECompensation(std::map<std::string, LTParam>::iterator it);
    void calcEmergencyEECompensation(std::map<std::string, LTParam>::iterator it);
    void calcContactEECompensation(std::map<std::string, LTParam>::iterator it);
    void calcNullJointDumping(std::map<std::string, LTParam>::iterator it);
    void calcEmergencyNullJointDumping(std::map<std::string, LTParam>::iterator it);
    void estimateRefdq();
    void calcMinMaxAvoidanceTorque();// Do not use this for now: it is not compatible with reference torque regulator for now

    std::map<std::string, LTParam> m_lt_param, m_ref_lt_param;
    std::map<std::string, CollisionParam> m_lt_col_param;
    std::map<std::string, ee_trans> ee_map;
    std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
    std::map<std::string, hrp::Vector3> abs_forces, abs_moments, abs_ref_forces, abs_ref_moments;

    // end-effector states
    std::map<std::string, hrp::Vector3> act_eepos, ref_eepos, prev_ref_eepos;
    std::map<std::string, hrp::dquaternion> act_eequat, ref_eequat;
    std::map<std::string, hrp::Matrix33> act_eeR, ref_eeR, prev_ref_eeR;
    std::map<std::string, hrp::Vector3> act_ee_vel, ref_ee_vel, act_ee_w, ref_ee_w;
    std::map<std::string, hrp::Vector3> ee_pos_error, ee_ori_error, ee_vel_error, ee_w_error;
    std::map<std::string, RMSfilter<hrp::Vector3> > ee_vel_filter, ref_ee_vel_filter;
    std::map<std::string, RMSfilter<hrp::Matrix33> > ee_w_filter, ref_ee_w_filter;
    std::map<std::string, hrp::dmatrix> act_ee_jacobian, inv_act_ee_jacobian, null_act_ee_jacobian;
    std::map<std::string, bool> geteestates_initialized;
    // for ee compensation
    std::map<std::string, hrp::Vector3> ee_pos_comp_force, ee_ori_comp_moment, ee_vel_comp_force, ee_w_comp_moment;
    std::map<std::string, hrp::dvector> ee_pos_ori_comp_wrench, ee_vel_w_comp_wrench;
    std::map<std::string, hrp::dvector> ee_compensation_torque;
    // for null space torque
    std::map<std::string, hrp::dvector> null_space_torque;
    // for emergency ee compensation
    std::map<std::string, hrp::Vector3> emergency_ref_eepos;
    std::map<std::string, hrp::dquaternion> emergency_ref_eequat;
    std::map<std::string, hrp::Matrix33> emergency_ref_eeR;
    // basic models, necessities
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
    std::map<std::string, hrp::Vector3> force_increase, prev_filtered_force;
    std::map<std::string, bool> observe_disturbance, dist_obs_initialized;
    std::map<std::string, std::ofstream*> debug_vel_discrepancy, debug_force_inc;
    std::map<std::string, bool> disturbance_detected;

    // reference torque regulator
    hrp::dvector invdyn_accvel_tq, invdyn_grav_tq, eecomp_tq, nullspace_tq;
    hrp::dvector reference_torque;
    std::map<std::string, bool> reftqregulator_initialized;
    std::map<std::string, int> disturbance_uncheck_count;
    int max_disturbance_uncheck_count;
    std::map<std::string, hrp::Vector3> reaction_eepos, reaction_eevel, reaction_eew;
    std::map<std::string, hrp::dquaternion> reaction_eeori;

    // Error Checkers
    std::map<std::string, bool> poserrchecker_initialized;
    std::map<std::string, hrp::dvector> minmaxavoid_torque; //零以外でminmax_detected=trueの役割
    std::map<std::string, bool> release_emergency_called;
    std::map<std::string, bool> fix_mode_normal; //idle_normalにモード固定(servooff時など使う): 他のエラーチェック等より優先する
    hrp::dvector reference_climit; //climit used for reference torque
    // error limits
    double idle_vlimit, idle_olimit, compliant_end_vel_thresh;
    void VelocityErrorChecker();
    void PositionErrorChecker();
    void ActualTorqueChecker();
    void ReferenceTorqueChecker();
    void ReferenceForceUpdater();
    void ModeSelector();

    // returns quaternion that means rotation from act to ref
    inline void safe_quaternion_comparison(const hrp::dquaternion& _q_ref, const hrp::dquaternion& _q_act, hrp::dquaternion& _q_diff)
    {
        // check and correct jump of quaternion (to prevent windup in orientation feedback)
        if ( (_q_ref.conjugate()*_q_act).w() < 0 ){
            hrp::dquaternion negated_q_ref(-_q_ref.w(), -_q_ref.x(), -_q_ref.y(), -_q_ref.z());
            _q_diff = negated_q_ref * _q_act.conjugate();
        }else{
            _q_diff = _q_ref * _q_act.conjugate();
        }
    }

    // SLERP with sign check (cannot be applied if initial and final are different by more than 180 degrees)
    inline void safe_slerp(const hrp::dquaternion& _q_initial, const hrp::dquaternion& _q_final, const double& ratio, hrp::dquaternion& _q_inbetween)
    {
        // check and correct jump of quaternion (to prevent windup in orientation feedback)
        if ( (_q_initial.conjugate()*_q_final).w() < 0 ){
            hrp::dquaternion negated_q_initial(-_q_initial.w(), -_q_initial.x(), -_q_initial.y(), -_q_initial.z());
            _q_inbetween = negated_q_initial.slerp(ratio, _q_final);
        }else{
            _q_inbetween = _q_initial.slerp(ratio, _q_final);
        }
    }

    // decomposition of quaternion to rotations around the axis of direction_q(twist) and its orthogonal vector(swing)
    // reference from https://stackoverflow.com/questions/3684269/component-of-a-quaternion-rotation-around-an-axis
    // TODO: 符号チェック要る?(ref,actの順番を間違えないようにし続ければいらないかも)
    // 180度近い回転をすると特異点になる(そのような目標回転を与えなければOK?)
    // need singularity check? (rotation near 180 degrees)
    inline void swing_twist_decomposition(const hrp::dquaternion& _original_q, const hrp::dquaternion& _direction_q, hrp::dquaternion& _swing, hrp::dquaternion& _twist)
    {
        hrp::Vector3 twist_axis = _direction_q.vec().normalized();
        _twist.w() = _original_q.w();
        _twist.vec() = twist_axis.dot(_original_q.vec()) * twist_axis;
        _twist.normalize();
        _swing = _original_q * _twist.conjugate();
    }
    inline void reset_taskstate_bool(TaskState& _ts)
    {
        _ts.pos_over_limit = false;
        _ts.pos_reach_target = false;
        _ts.ori_over_limit = false;
        _ts.ori_reach_target = false;
        _ts.vel_over_thresh = false;
        _ts.vel_over_limit = false;
        _ts.w_over_limit = false;
        _ts.torque_over_limit = false;
        _ts.em_transition_count = 0;
        _ts.f2c_transition_count = 0;
    }

    // temporary setting these global for debugging
    //MOVE_POS
    std::map<std::string, double> check_dir_vel_err, other_dir_vel_err, dist_to_target, pos_error_norm;
    std::map<std::string, std::ofstream*> debug_cdve, debug_odve, debug_dtt, debug_pen;
    //MOVE_POSROT
    std::map<std::string, double> pdist_to_target, ppos_error_norm, target_dir_quatdiffw, unwanted_dir_quatdiffw;
    std::map<std::string, std::ofstream*> debug_pdtt, debug_ppen, debug_tdqw, debug_udqw;
};

extern "C"
{
    void LimbTorqueControllerInit(RTC::Manager* manager);
};

#endif // LIMBTORQUE_H
