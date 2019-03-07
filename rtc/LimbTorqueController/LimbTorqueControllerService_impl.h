// -*-C++-*-
#ifndef LIMBTORQUESERVICESVC_IMPL_H
#define LIMBTORQUESERVICESVC_IMPL_H

#include "hrpsys/idl/LimbTorqueControllerService.hh"

using namespace OpenHRP;

class LimbTorqueController;

class LimbTorqueControllerService_impl
  : public virtual POA_OpenHRP::LimbTorqueControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:

    /**
       \brief constructor
    */
  LimbTorqueControllerService_impl();

    /**
       \brief destructor
    */
  virtual ~LimbTorqueControllerService_impl();

  CORBA::Boolean startLimbTorqueController(const char *i_name_);
  CORBA::Boolean stopLimbTorqueController(const char *i_name_);
  CORBA::Boolean setLimbTorqueControllerParam(const char *i_name_, const OpenHRP::LimbTorqueControllerService::limbtorqueParam &i_param_);
  CORBA::Boolean getLimbTorqueControllerParam(const char *i_name_, OpenHRP::LimbTorqueControllerService::limbtorqueParam_out i_param_);
    CORBA::Boolean startLog(const char *i_name_, const char *i_dirname_);
  CORBA::Boolean stopLog();
    CORBA::Boolean releaseEmergency(const char *i_name_, CORBA::Boolean cancel);
  CORBA::Boolean giveTaskDescription(const char *i_name_, const OpenHRP::LimbTorqueControllerService::taskDescription &i_taskd_);
  CORBA::Boolean getTaskDescription(const char *i_name_, OpenHRP::LimbTorqueControllerService::taskDescription_out i_taskd_);
  CORBA::Boolean getTaskState(const char *i_name_, OpenHRP::LimbTorqueControllerService::taskState_out i_tasks_);
    CORBA::Boolean startModeChange(const char *i_name_);
    CORBA::Boolean stopModeChange(const char *i_name_);
    CORBA::Boolean startEmergency();
    CORBA::Boolean startEmergencyreleaseFz();
    CORBA::Boolean releaseEmergencyholdFz(const char *i_name_);
    CORBA::Boolean checkEmergencyFlag(const char *i_name_, CORBA::Boolean_out i_flag_);

  //
  void limbtorque(LimbTorqueController *i_limbtorque);
private:
  LimbTorqueController *m_limbtorque;
};

#endif
