include_directories(argos3/plugins/robot/kilobot/control_interface argos3/plugins/robot/kilobot/simulator)

add_library(ALF_demoC_loop_function MODULE demoC_ALF.h demoC_ALF.cpp)

target_link_libraries(ALF_demoC_loop_function
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_media
  argos3plugin_simulator_kilobot
  argos3plugin_simulator_kilolib
)



if(ARGOS_QTOPENGL_FOUND)
add_library(ALF_qt_user_functions MODULE ALF_qt_user_functions.h
ALF_qt_user_functions.cpp)
endif(ARGOS_QTOPENGL_FOUND)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(ALF_demoC_loop_function argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)