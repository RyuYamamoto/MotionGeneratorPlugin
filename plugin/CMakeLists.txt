
if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_MOTION_GENERATOR "Building a sample plugin \"Sample1Plugin\"" OFF)

if(BUILD_MOTION_GENERATOR)
  set(target CnoidMotionGeneratorPlugin)
  add_cnoid_plugin(${target} SHARED MotionGeneratorPlugin.cpp Kinematics.cpp Link.cpp)
  target_link_libraries(${target} CnoidBodyPlugin)
  apply_common_setting_for_plugin(${target})

  if(QT5)
    qt5_use_modules(${target} Widgets)
  endif()
endif()

install_sample_source(MotionGeneratorPlugin.cpp)
