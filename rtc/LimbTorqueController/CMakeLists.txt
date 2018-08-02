set(comp_sources LimbTorqueController.cpp LimbTorqueControllerService_impl.cpp ../ImpedanceController/JointPathEx.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(LimbTorqueController SHARED ${comp_sources})
target_link_libraries(LimbTorqueController ${libs})
set_target_properties(LimbTorqueController PROPERTIES PREFIX "")

add_executable(LimbTorqueControllerComp LimbTorqueControllerComp.cpp ${comp_sources})
target_link_libraries(LimbTorqueControllerComp ${libs})

set(target LimbTorqueController LimbTorqueControllerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
