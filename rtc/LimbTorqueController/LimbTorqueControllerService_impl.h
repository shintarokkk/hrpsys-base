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

  //
  void limbtorque(LimbTorqueController *i_limbtorque);
private:
  LimbTorqueController *m_limbtorque;
};

#endif
