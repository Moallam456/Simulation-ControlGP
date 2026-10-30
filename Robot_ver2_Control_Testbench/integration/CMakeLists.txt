cmake_minimum_required(VERSION 3.16)
project(RobotVer2PortableTests C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
enable_testing()
get_filename_component(PACKAGE_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(UPSTREAM "${PACKAGE_ROOT}/vendor/GPRobotControl_Simulation-15aa22efc071554fe06f1e42e1007633dee15cc9")
add_library(rb2 STATIC "${PACKAGE_ROOT}/control/robot_ver2_kinematics.c")
target_include_directories(rb2 PUBLIC "${PACKAGE_ROOT}/control")
target_link_libraries(rb2 PUBLIC m)
set(RB2_PREPARED_CONTROL "" CACHE PATH "Separate checkout made by prepare_control.py")
if(RB2_PREPARED_CONTROL)
    add_subdirectory("${RB2_PREPARED_CONTROL}/ControlCore" prepared_control_core)
    add_executable(controller_compatibility "${PACKAGE_ROOT}/tests/controller_compatibility.c")
    target_link_libraries(controller_compatibility PRIVATE control_core)
    add_test(NAME robot_ver2_controller_compatibility COMMAND controller_compatibility)
    foreach(name single_segment_line_test single_segment_circular_test single_segment_circular_stream_test
                 trajectory_buffer_test s_curve_profile_test single_segment_line_stream_test
                 buffered_stream_test joint_trajectory_test)
        add_test(NAME regression_${name} COMMAND ${name})
    endforeach()
endif()
add_executable(protocol_test "${PACKAGE_ROOT}/tests/protocol_test.cpp")
add_test(NAME physics_protocol COMMAND protocol_test)
add_executable(kinematics_test "${PACKAGE_ROOT}/tests/kinematics_test.c")
target_link_libraries(kinematics_test PRIVATE rb2)
add_test(NAME robot_ver2_kinematics COMMAND kinematics_test)
foreach(name admittance_control sensor_config wrench_processing)
    if(EXISTS "${UPSTREAM}/HandGuiding/Tests/test_${name}.c" AND EXISTS "${UPSTREAM}/HandGuiding/${name}.c")
        add_executable(upstream_${name} "${UPSTREAM}/HandGuiding/Tests/test_${name}.c" "${UPSTREAM}/HandGuiding/${name}.c")
        target_link_libraries(upstream_${name} PRIVATE m)
        add_test(NAME upstream_${name} COMMAND upstream_${name})
    endif()
endforeach()
target_sources(upstream_admittance_control PRIVATE "${UPSTREAM}/HandGuiding/admittance_matlab_reference.c")
add_executable(upstream_avatar_node
    "${UPSTREAM}/ServoDrive/AvatarM/Sim/test_avatar_m_node_sim.c"
    "${UPSTREAM}/ServoDrive/AvatarM/Sim/avatar_m_node_sim.c"
    "${UPSTREAM}/ServoDrive/AvatarM/avatar_m_pdo.c"
    "${UPSTREAM}/ServoDrive/CiA402/cia402.c")
target_link_libraries(upstream_avatar_node PRIVATE m)
add_test(NAME upstream_avatar_node COMMAND upstream_avatar_node)
add_executable(upstream_position
    "${PACKAGE_ROOT}/tests/position_test.c"
    "${UPSTREAM}/ServoDrive/AvatarM/avatar_m_position.c")
target_include_directories(upstream_position PRIVATE "${UPSTREAM}")
target_link_libraries(upstream_position PRIVATE m)
add_test(NAME upstream_position_roundtrip COMMAND upstream_position)
option(RB2_BUILD_CAN_ADAPTER "Build actual SIL Kit motor bank (Linux)" OFF)
if(RB2_BUILD_CAN_ADAPTER)
    find_package(SilKit REQUIRED CONFIG PATHS "${SILKIT_ROOT}" NO_DEFAULT_PATH)
    find_package(Threads REQUIRED)
    add_executable(rb2_physics_motor_bank physics_motor_bank.cpp
        "${UPSTREAM}/Simulation/AvatarM_CAN/avatar_m_silkit_node.cpp"
        "${UPSTREAM}/CANComm/SILKit/silkit_can_backend.cpp"
        "${UPSTREAM}/ServoDrive/AvatarM/Sim/avatar_m_node_sim.c"
        "${UPSTREAM}/ServoDrive/AvatarM/avatar_m_pdo.c"
        "${UPSTREAM}/ServoDrive/AvatarM/avatar_m_position.c"
        "${UPSTREAM}/ServoDrive/CiA402/cia402.c")
    target_include_directories(rb2_physics_motor_bank PRIVATE "${UPSTREAM}" "${UPSTREAM}/Simulation/AvatarM_CAN")
    target_link_libraries(rb2_physics_motor_bank PRIVATE SilKit::SilKit Threads::Threads)
    add_executable(can_client_test "${PACKAGE_ROOT}/tests/can_client_test.cpp"
        "${UPSTREAM}/CANComm/SILKit/silkit_can_backend.cpp"
        "${UPSTREAM}/ServoDrive/AvatarM/avatar_m_pdo.c")
    target_include_directories(can_client_test PRIVATE "${UPSTREAM}")
    target_link_libraries(can_client_test PRIVATE SilKit::SilKit Threads::Threads)
endif()
