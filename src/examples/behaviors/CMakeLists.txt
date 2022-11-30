if(ARGOS_BUILD_FOR_SIMULATOR)
  include_directories(${CMAKE_SOURCE_DIR}/plugins/robots/kilobot/control_interface)

 
  #
  # Checking max_turning_ticks
  #
  add_executable(simple_rotation simple_rotation.c)
  target_link_libraries(simple_rotation argos3plugin_simulator_kilolib)


  #
  # ARK loop function: navigation_experiment
  #
  add_executable(move_to_position move_to_position.c)  
  target_link_libraries(move_to_position argos3plugin_simulator_kilolib)

  #
  # ARK loop function: crwlevy
  #
  add_executable(crwlevy crwlevy.c)  
  target_link_libraries(crwlevy argos3plugin_simulator_kilolib)

  #
  # ARK loop function: lmcrw (levy modulated correlated random walk)
  #
  add_executable(lmcrw lmcrw.c)  
  target_link_libraries(lmcrw argos3plugin_simulator_kilolib)

  #
  # ARK loop function: crwlevy_with_positioning
  #
  add_executable(crwlevy_with_positioning crwlevy_with_positioning.c)  
  target_link_libraries(crwlevy_with_positioning argos3plugin_simulator_kilolib)

  #
  # My test : pi/2 left rotation
  #
  add_executable(test_rotation test_rotation.c)
  target_link_libraries(test_rotation argos3plugin_simulator_kilolib)

  #
  # Debugging example
  #
  add_executable(test_debug test_debug.h test_debug.c)
  target_link_libraries(test_debug argos3plugin_simulator_kilolib)
  endif(ARGOS_BUILD_FOR_SIMULATOR)


