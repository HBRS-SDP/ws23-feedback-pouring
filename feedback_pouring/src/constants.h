// constants.h
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <string>

struct Pose
{
    double x;
    double y;
    double z;
    double theta_x;
    double theta_y;
    double theta_z;
};

const std::string URDF_FILE_NAME = "GEN3-7DOF-VISION_ARM_URDF_V12";
const std::string ROBOT_IP = "192.168.1.12";
const std::string USER_NAME = "admin";
const std::string PASSWORD = "admin";
const Pose TARGET_POSE = {0.548, -0.289, 0.133, 90.11, -0.002, 87.125};

#endif // CONSTANTS_H
