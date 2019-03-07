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
typedef Eigen::Matrix<double, 3, 2> Matrix32;
typedef Eigen::Matrix<double, 2, 3> Matrix23;
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
    bool startLog(const std::string& i_name_, const std::string& i_dirname_);
    bool stopLog();
    bool releaseEmergency(const std::string& i_name_, bool cancel); //現在に近い関節角を送ってから実行する
    bool giveTaskDescription(const std::string& i_name_, OpenHRP::LimbTorqueControllerService::taskDescription task_description);
    bool getTaskDescription(const std::string& i_name_, OpenHRP::LimbTorqueControllerService::taskDescription_out i_taskd_);
    bool getTaskState(const std::string &i_name_, OpenHRP::LimbTorqueControllerService::taskState_out i_tasks_);
    bool startModeChange(const std::string &i_name_);
    bool stopModeChange(const std::string &i_name_);
    bool startEmergency();
    bool startEmergencyreleaseFz();
    bool releaseEmergencyholdFz(const std::string &i_name_);
    bool checkEmergencyFlag(const std::string& i_name_, bool& i_flag_);

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
    std::vector<RTC::TimedDoubleSeq> m_ref_force;
    std::vector<RTC::OutPort<RTC::TimedDoubleSeq> *> m_ref_forceOut;
    RTC::TimedOrientation3D m_rpy;
    RTC::InPort<RTC::TimedOrientation3D> m_rpyIn;
    RTC::TimedLong m_emergencySignal;
    RTC::OutPort<RTC::TimedLong> m_emergencySignalOut;

    RTC::TimedDoubleSeq m_q;
    RTC::OutPort<RTC::TimedDoubleSeq> m_qOut;
    RTC::TimedDoubleSeq m_tq;
    RTC::OutPort<RTC::TimedDoubleSeq> m_tqOut;

    RTC::CorbaPort m_LimbTorqueControllerServicePort;

    LimbTorqueControllerService_impl m_service0;

private:

    enum arm_mode {IDLE_NORMAL, MANIP, EMERGENCY};

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
        int ee_index;
    };

    // wordings: limit->desired to be avoided | threshold->desired to reach
    struct TaskDescription{
        bool dual;
        double vel_check_limit; //velocity error threshold from MANIP to EMERGENCY
        double emergency_recover_time; // set this to minus value for no emergency transition
        bool add_static_force; // whether to add static force during MANIP_FREE or not to
        double static_rfu_gain;
        bool emergency_hold_fz; //whether to hold z-direction reference force during emergency
    };

    // エラー等の状態
    struct TaskState{
        hrp::dvector F_now; //current reference force
        bool vel_over_limit; // whether ee velocity exceeds limit or not (in MANIP_CONTACT and MANIP_FREE)
        bool torque_over_limit; // whether joint torque exceeds limit or not (in MANIP_CONTACT)
        int em_transition_count; //transition to EMERGENCY: releaving reference force  //also used from add_static_force=false->add_static_force=true
        int max_em_t_count; //maximum value of em_transition_count
        hrp::dvector F_em_init; //F_now at the moment of emergency transition //also used at transition from add_static_force=false->add_static_force=true
        hrp::Vector3 initial_pos; //act ee pos(in world coordinate) at the moment of mode transition
        hrp::dquaternion initial_ori; //act ee orientation(in world coordinate) at the moment of mode transition
        hrp::dvector emergency_q; //joint angles at the moment of transition to emrgency mode
        int remove_static_force_count;
    };
    std::map<std::string, TaskDescription> limb_task_target;
    std::map<std::string, TaskState> limb_task_state;

    void copyLimbTorqueControllerParam (OpenHRP::LimbTorqueControllerService::limbtorqueParam& i_param_, const LTParam& param);
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
    void calcManipEECompensation(std::map<std::string, LTParam>::iterator it); //used in MANIP_FREE mode: can add resisting static force
    void calcContactEECompensation(std::map<std::string, LTParam>::iterator it);
    void calcNullJointDumping(std::map<std::string, LTParam>::iterator it);
    void calcEmergencyNullJointDumping(std::map<std::string, LTParam>::iterator it);

    std::map<std::string, LTParam> m_lt_param, m_ref_lt_param;
    std::map<std::string, ee_trans> ee_map;
    std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
    std::map<std::string, hrp::Vector3> abs_forces, abs_moments, abs_ref_forces, abs_ref_moments;

    // end-effector states
    std::map<std::string, hrp::Vector3> act_eepos, ref_eepos;
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

    hrp::dvector actual_torque_vector;

    // for debug log
    std::map<std::string, std::ofstream*> debug_reftq;
    std::map<std::string, std::ofstream*> debug_ee_pocw, debug_ee_vwcw, debug_eect, debug_nst, debug_acteevel, debug_refeevel, debug_acteew, debug_refeew; //calcEECompensation
    std::map<std::string, std::ofstream*> debug_ee_poserror, debug_ee_orierror, debug_ee_velerror, debug_ee_werror; //calcEECompensation
    std::map<std::string, std::ofstream*> debug_dqact, debug_qact, debug_qref;
    std::map<std::string, std::ofstream*> debug_acteescrew, debug_acteewrench; //ee vel&force estimation
    void DebugOutput();
    bool spit_log;

    hrp::dvector log_act_q, log_ref_q;

    // reference torque regulator
    hrp::dvector invdyn_accvel_tq, invdyn_grav_tq, eecomp_tq, nullspace_tq;
    hrp::dvector reference_torque;

    // Error Checkers
    std::map<std::string, bool> release_emergency_called;
    std::map<std::string, bool> fix_mode_normal; //idle_normalにモード固定(servooff時など使う): 他のエラーチェック等より優先する
    hrp::dvector reference_climit; //climit used for reference torque
    // error limits
    void VelocityErrorChecker();
    void ActualTorqueChecker();
    void ReferenceTorqueChecker();
    void ReferenceForceUpdater();
    void ModeSelector();

    // emergency flags
    bool reset_emergency_flag, is_emergency;
    bool start_emergency_called, start_emergency_release_fz_called, release_emergency_hold_fz_called;

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

    inline void reset_taskstate_bool(TaskState& _ts)
    {
        _ts.vel_over_limit = false;
        _ts.torque_over_limit = false;
        _ts.em_transition_count = 0;
        //remove_static_force_countはリセットしない(IDLE_NORMAL->IDLE_COMPLIANTなどでゼロにされると困る)
    }

    std::map<std::string, hrp::dvector> world_ref_wrench; //actualはabs_forces
    std::map<std::string, std::ofstream*> debug_wrw; //actualはdebug_acteewrench

    // for new ee estimation: only estimate translational part
    // initially set values //mapじゃなくてもよいかも(両手に同一の値設定しているので)
    std::map<std::string, bool> eeest_initialized;
    std::map<std::string, double> virtual_ee_mass, virtual_ee_mass_for_vel;
    std::map<std::string, Matrix23> ee_obs_coeff;
    std::map<std::string, hrp::Matrix33> ee_system_noise;
    std::map<std::string, Matrix22> ee_obs_noise;
    // iteratively updated paramters
    std::map<std::string, hrp::Matrix33> ee_state_coeff; //usually constant(change when ee gain is changed)
    std::map<std::string, std::vector<hrp::Vector3> > ee_state_est;
    std::map<std::string, std::vector<hrp::Matrix33> > ee_error_covar;
    // reordered estimate values
    std::map<std::string, hrp::Vector3> filtered_ee_vel, filtered_f_d, filtered_f_s;
    void calcStaticForceFilter();
    void calcStaticForceFilter_init(const std::map<std::string, LTParam>::iterator it);

    std::map<std::string, std::ofstream*> debug_filtereevel, debug_filtereef_d, debug_filtereef_s;
    std::map<std::string, std::ofstream*> debug_act_torque;
    int max_rsfc; //max_remove_static_force_count

    // for raw filter
    std::map<std::string, hrp::Vector3> raw_act_ee_vel, raw_ref_ee_vel, prev_act_eepos, prev_ref_eepos;
    std::map<std::string, std::ofstream*> debug_raw_acteevel, debug_raw_refeevel;
};

extern "C"
{
    void LimbTorqueControllerInit(RTC::Manager* manager);
};

#endif // LIMBTORQUE_H
