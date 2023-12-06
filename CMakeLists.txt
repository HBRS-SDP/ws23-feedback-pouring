cmake_minimum_required(VERSION 3.12)

project(SDP)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../install)

Set(FETCHCONTENT_QUIET FALSE)

# check if kinova api exists
if(NOT EXISTS ../kinova_api/lib/release/libKortexApiCpp.a)
    message(WARNING "kinova_api not found.")
    message("Downloading file...")
    File(DOWNLOAD 
        https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.6.0/linux_x86-64_x86_gcc.zip 
        ../kinova_api.zip 
        SHOW_PROGRESS STATUS DOWNLOAD_RESULT)
    list(GET DOWNLOAD_RESULT 0 DOWNLOAD_RESULT_CODE)
    if(NOT DOWNLOAD_RESULT_CODE EQUAL 0)
        message(FATAL_ERROR "Failed downloading! Error: ${DOWNLOAD_RESULT}.")
    endif()
    message("Creating directory...")
    file(MAKE_DIRECTORY ../kinova_api)
    message("Extracting file...")
    execute_process(COMMAND unzip ../kinova_api.zip -d ../kinova_api OUTPUT_QUIET)
    message("Removing file...")
    file(REMOVE ../kinova_api.zip)
    if(NOT EXISTS ../kinova_api/lib/release/libKortexApiCpp.a)
        message(FATAL_ERROR "Something went wrong with the download and extraction of kinova_api.")
    endif()
endif()

# add subdirectories
# add_subdirectory(../orocos_kinematics_dynamics/orocos_kdl)

# include_directories(../orocos_kinematics_dynamics/orocos_kdl/src/)

if(NOT EXISTS ../orocos_kinematics_dynamics)
    # Execute a shell command to clone the repository
    execute_process(
        COMMAND git clone https://github.com/orocos/orocos_kinematics_dynamics.git ../orocos_kinematics_dynamics
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        RESULT_VARIABLE GIT_RESULT
    )

    # Check the result of the clone operation
    if(GIT_RESULT EQUAL "0")
        message("Repository cloned successfully")
    else()
        message("Failed to clone repository")
    endif()
endif()




add_compile_definitions(_OS_UNIX)

find_package(Boost COMPONENTS system filesystem REQUIRED)

# Kinova Kortex related
set(KORTEX_DIR "/home/vicky/sdp/kinova_api/")
link_libraries(${KORTEX_DIR}lib/release/libKortexApiCpp.a)


set(KDL_DIR "/home/vicky/sdp/orocos_kinematics_dynamics/orocos_kdl/")
link_libraries(${KDL_DIR}/build/src/liborocos-kdl.so)

set(KDL_PARSER_DIR "/usr/")
link_libraries(kdl_parser)
link_libraries(urdf)

link_libraries(
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  pthread
)


include_directories(
  include
  ${KORTEX_DIR}/include
  ${KORTEX_DIR}/include/client
  ${KORTEX_DIR}/include/common
  ${KORTEX_DIR}/include/messages
  ${KORTEX_DIR}/include/client_stubs
  ${KORTEX_DIR}/include/google
  ${KDL_DIR}/src
  ${KDL_PARSER_DIR}/include/kdl_parser
  /usr/include/eigen3/
)

add_executable(main
feedback_pouring/src/arm_initialisation.cpp
)

target_link_libraries(main
    ${orocos_kdl_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)