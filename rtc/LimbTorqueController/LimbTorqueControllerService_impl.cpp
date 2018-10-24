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
    i_param_->force_gain.length(3);
    i_param_->moment_gain.length(3);
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

CORBA::Boolean LimbTorqueControllerService_impl::startLog(const char *i_name_)
{
  return m_limbtorque->startLog(std::string(i_name_));
}

CORBA::Boolean LimbTorqueControllerService_impl::stopLog()
{
  return m_limbtorque->stopLog();
}

void LimbTorqueControllerService_impl::limbtorque(LimbTorqueController *i_limbtorque)
{
  m_limbtorque = i_limbtorque;
}

