#include "LimbTorqueControllerService_impl.h"
#include "LimbTorqueController.h"
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>

LimbTorqueControllerService_impl::LimbTorqueControllerService_impl() : m_limbtorque(NULL)
{
}

LimbTorqueControllerService_impl::~LimbTorqueControllerService_impl()
{
}

CORBA::Boolean LimbTorqueControllerService_impl::startLimbTorqueController(const char *i_name_)
{
  return m_limbtorque->startLimbTorqueController(std::string(i_name_));
}

CORBA::Boolean LimbTorqueControllerService_impl::stopLimbTorqueController(const char *i_name_)
{
  return m_limbtorque->stopLimbTorqueController(std::string(i_name_));
}

CORBA::Boolean LimbTorqueControllerService_impl::setLimbTorqueControllerParam(const char *i_name_, const OpenHRP::LimbTorqueControllerService::limbtorqueParam &i_param_)
{
  return m_limbtorque->setLimbTorqueControllerParam(std::string(i_name_), i_param_);
}

CORBA::Boolean LimbTorqueControllerService_impl::getLimbTorqueControllerParam(const char *i_name_, OpenHRP::LimbTorqueControllerService::limbtorqueParam_out i_param_)
{
    i_param_ = new OpenHRP::LimbTorqueControllerService::limbtorqueParam();
    i_param_->ee_pgain.length(6);
    i_param_->ee_dgain.length(6);
    return m_limbtorque->getLimbTorqueControllerParam(std::string(i_name_), *i_param_);
}

CORBA::Boolean LimbTorqueControllerService_impl::setCollisionParam(const char *i_name_, const OpenHRP::LimbTorqueControllerService::collisionParam &i_param_)
{
  return m_limbtorque->setCollisionParam(std::string(i_name_), i_param_);
}

CORBA::Boolean LimbTorqueControllerService_impl::getCollisionParam(const char *i_name_, const OpenHRP::LimbTorqueControllerService::collisionParam_out i_param_)
{
  return m_limbtorque->getCollisionParam(std::string(i_name_), i_param_);
}

CORBA::Boolean LimbTorqueControllerService_impl::getCollisionTorque(const char *i_name_, OpenHRP::LimbTorqueControllerService::DblSequence_out c_vec_)
{
  return m_limbtorque->getCollisionTorque(std::string(i_name_), c_vec_);
}

CORBA::Boolean LimbTorqueControllerService_impl::getCollisionStatus(const char *i_name_, OpenHRP::LimbTorqueControllerService::collisionStatus_out i_param_)
{
  return m_limbtorque->getCollisionStatus(std::string(i_name_), i_param_);
}

CORBA::Boolean LimbTorqueControllerService_impl::startLog(const char *i_name_, const char *i_logname_, const char *i_dirname_)
{
    return m_limbtorque->startLog(std::string(i_name_), std::string(i_logname_), std::string(i_dirname_));
}

CORBA::Boolean LimbTorqueControllerService_impl::stopLog()
{
  return m_limbtorque->stopLog();
}

CORBA::Boolean LimbTorqueControllerService_impl::startRefdqEstimation(const char *i_name_)
{
  return m_limbtorque->startRefdqEstimation(std::string(i_name_));
}

CORBA::Boolean LimbTorqueControllerService_impl::stopRefdqEstimation(const char *i_name_)
{
  return m_limbtorque->stopRefdqEstimation(std::string(i_name_));
}

CORBA::Boolean LimbTorqueControllerService_impl::releaseEmergency(const char *i_name_, CORBA::Boolean cancel)
{
    return m_limbtorque->releaseEmergency(std::string(i_name_), cancel);
}

CORBA::Boolean LimbTorqueControllerService_impl::giveTaskDescription(const char *i_name_, const OpenHRP::LimbTorqueControllerService::taskDescription &i_taskd_)
{
    return m_limbtorque->giveTaskDescription(std::string(i_name_), i_taskd_);
}

CORBA::Boolean LimbTorqueControllerService_impl::getTaskDescription(const char *i_name_, OpenHRP::LimbTorqueControllerService::taskDescription_out i_taskd_)
{
    return m_limbtorque->getTaskDescription(std::string(i_name_), i_taskd_);
}

CORBA::Boolean LimbTorqueControllerService_impl::getTaskState(const char *i_name_, OpenHRP::LimbTorqueControllerService::taskState_out i_tasks_)
{
    return m_limbtorque->getTaskState(std::string(i_name_), i_tasks_);
}

CORBA::Boolean LimbTorqueControllerService_impl::startModeChange(const char *i_name_)
{
    return m_limbtorque->startModeChange(std::string(i_name_));
}

CORBA::Boolean LimbTorqueControllerService_impl::stopModeChange(const char *i_name_)
{
    return m_limbtorque->stopModeChange(std::string(i_name_));
}

CORBA::Boolean LimbTorqueControllerService_impl::startEmergency()
{
    return m_limbtorque->startEmergency();
}

CORBA::Boolean LimbTorqueControllerService_impl::startEmergencyreleaseFz()
{
    return m_limbtorque->startEmergencyreleaseFz();
}

CORBA::Boolean LimbTorqueControllerService_impl::releaseEmergencyholdFz(const char *i_name_)
{
    return m_limbtorque->releaseEmergencyholdFz(std::string(i_name_));
}

CORBA::Boolean LimbTorqueControllerService_impl::checkEmergencyFlag(const char *i_name_, CORBA::Boolean_out i_flag_)
{
    return m_limbtorque->checkEmergencyFlag(std::string(i_name_), i_flag_);
}

void LimbTorqueControllerService_impl::limbtorque(LimbTorqueController *i_limbtorque)
{
  m_limbtorque = i_limbtorque;
}
