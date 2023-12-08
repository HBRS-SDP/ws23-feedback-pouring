cmake_minimum_required(VERSION 3.12)

project(SDP)

set(CMAKE_CXX_STANDARD 17)

# include(FetchContent)

# set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../install)

# Set(FETCHCONTENT_QUIET FALSE)

# check if kinova api exists
# if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/lib/release/libKortexApiCpp.a)
#     message(WARNING "kinova_api not found.")
#     message("Downloading file...")
#     File(DOWNLOAD 
#         https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.6.0/linux_x86-64_x86_gcc.zip 
#         ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api.zip 
#         SHOW_PROGRESS STATUS DOWNLOAD_RESULT)
#     list(GET DOWNLOAD_RESULT 0 DOWNLOAD_RESULT_CODE)
#     if(NOT DOWNLOAD_RESULT_CODE EQUAL 0)
#         message(FATAL_ERROR "Failed downloading! Error: ${DOWNLOAD_RESULT}.")
#     endif()
#     message("Creating directory...")
#     file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api)
#     message("Extracting file...")
#     execute_process(COMMAND unzip ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api.zip -d ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api OUTPUT_QUIET)
#     message("Removing file...")
#     file(REMOVE ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api.zip)
#     if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/lib/release/libKortexApiCpp.a)
#         message(FATAL_ERROR "Something went wrong with the download and extraction of kinova_api.")
#     endif()
# endif()

# add subdirectories
# add_subdirectory(orocos_kinematics_dynamics/orocos_kdl)

# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/orocos_kinematics_dynamics/orocos_kdl/src/)

add_compile_definitions(_OS_UNIX)

find_package(Boost COMPONENTS system filesystem REQUIRED)

# Kinova Kortex related
set(KORTEX_DIR "/home/salvin/INSTALL/kinova_api/")
link_libraries(${KORTEX_DIR}lib/release/libKortexApiCpp.a)

include_directories(
  include
  ${KORTEX_DIR}/include
  ${KORTEX_DIR}/include/client
  ${KORTEX_DIR}/include/common
  ${KORTEX_DIR}/include/messages
  ${KORTEX_DIR}/include/client_stubs
  ${KORTEX_DIR}/include/google
)


# KDL related
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(orocos_kdl REQUIRED orocos_kdl)
# pkg_check_modules(kdl_parser REQUIRED kdl_parser)

# link_libraries(/home/salvin/INSTALL/lib/libkdl_parser.so REQUIRED)
# include_directories(
#   ${orocos_kdl_INCLUDE_DIRS}
#   ${kdl_parser_INCLUDE_DIRS}
# )

find_package(urdf)

set(KDL_DIR "/home/salvin/INSTALL/KDL/")
link_libraries(${KDL_DIR}lib/liborocos-kdl.so)

set(KDL_PARSER_DIR "/home/salvin/INSTALL/KDL_PARSER/")
link_libraries(${KDL_PARSER_DIR}lib/libkdl_parser.so)

link_libraries(
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  pthread
)

include_directories(
  ${KDL_DIR}/include
  ${KDL_PARSER_DIR}/include
)

add_executable(main
src/arm_initialisation.cpp
)

target_link_libraries(main
    ${orocos_kdl_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    urdf
)
