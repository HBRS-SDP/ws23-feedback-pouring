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
const int PORT_ID = 1000;
const Pose TARGET_POSE = {0.548, -0.289, 0.133, 90.11, -0.002, 87.125};
const float STARTING_SPEED = 1.8f;
const float STOPPING_SPEED = -40.0f;
const float PI = 3.14159265358979323846f;
const float DEGREES= 180.0;

#endif // CONSTANTS_H
