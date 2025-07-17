devel@HSA_Project_devcont:/workspaces/HSA_Project/test_skin_ws$ git checkout
Your branch is up to date with 'origin/main'.
devel@HSA_Project_devcont:/workspaces/HSA_Project/test_skin_ws$ catkin_make
Base path: /workspaces/HSA_Project/test_skin_ws
Source space: /workspaces/HSA_Project/test_skin_ws/src
Build space: /workspaces/HSA_Project/test_skin_ws/build
Devel space: /workspaces/HSA_Project/test_skin_ws/devel
Install space: /workspaces/HSA_Project/test_skin_ws/install
####
#### Running command: "make cmake_check_build_system" in "/workspaces/HSA_Project/test_skin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /workspaces/HSA_Project/test_skin_ws/devel
-- Using CMAKE_PREFIX_PATH: /opt/ros/noetic
-- This workspace overlays: /opt/ros/noetic
-- Found PythonInterp: /usr/bin/python3 (found suitable version "3.8.10", minimum required is "3") 
-- Using PYTHON_EXECUTABLE: /usr/bin/python3
-- Using Debian Python package layout
-- Using empy: /usr/lib/python3/dist-packages/em.py
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /workspaces/HSA_Project/test_skin_ws/build/test_results
-- Forcing gtest/gmock from source, though one was otherwise available.
-- Found gtest sources under '/usr/src/googletest': gtests will be built
-- Found gmock sources under '/usr/src/googletest': gmock will be built
-- Found PythonInterp: /usr/bin/python3 (found version "3.8.10") 
-- Using Python nosetests: /usr/bin/nosetests3
-- catkin 0.8.10
-- BUILD_SHARED_LIBS is on
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 1 packages in topological order:
-- ~~  - skin_force_publisher
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'skin_force_publisher'
-- ==> add_subdirectory(skin_force_publisher)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- Find cmake package 'Eigen3'... 
-- Find cmake package 'TThreads'... 
-- skin_force_publisher: 1 messages, 0 services
-- Configuring done
-- Generating done
-- Build files have been written to: /workspaces/HSA_Project/test_skin_ws/build
####
#### Running command: "make -j20 -l20" in "/workspaces/HSA_Project/test_skin_ws/build"
####
[  0%] Built target roscpp_generate_messages_cpp
[  0%] Built target geometry_msgs_generate_messages_eus
[  0%] Built target std_msgs_generate_messages_nodejs
[  0%] Built target tf2_msgs_generate_messa
...
Scanning dependencies of target force_publisher_node
[ 81%] Building CXX object skin_force_publisher/CMakeFiles/force_publisher_node.dir/src/force_publisher_node.cpp.o
[ 90%] Linking CXX executable /workspaces/HSA_Project/test_skin_ws/devel/lib/skin_force_publisher/force_publisher_node
[100%] Built target force_publisher_node
devel@HSA_Project_devcont:/workspaces/HSA_Project/test_skin_ws$ 