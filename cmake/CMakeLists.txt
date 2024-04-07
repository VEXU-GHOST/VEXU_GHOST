add_executable(helloworld src/helloworld.cpp)
install(TARGETS
  helloworld
  DESTINATION lib/${\\wsl.localhost\Ubuntu\home\annievu\VEXU_GHOST})

colcon build --packages-select annie-onboarding
source ~/.bashrc
ros2 run annie-onboarding helloworld