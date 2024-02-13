

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <InterconnectCyclicClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <DeviceManagerClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <Common.pb.h>
#include <Base.pb.h>
#include <BaseCyclic.pb.h>
#include <InterconnectCyclic.pb.h>
#include <fstream>

#include "chain.hpp"
#include "tree.hpp"
// #include "frames_io.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "chainexternalwrenchestimator.hpp"
#include <cmath>
#include "constants.h"
// #include <gnuplot-iostream.h>

using namespace KDL;

namespace k_api = Kinova::Api;

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_DURATION = std::chrono::seconds{20};

// // Data structure to represent a 3D pose
// struct Pose
// {
//     double x;
//     double y;
//     double z;
//     double theta_x;
//     double theta_y;
//     double theta_z;
// };

#define PORT PORT_ID
// #define PORT 10000
#define DEG_TO_RAD(x) (x) * PI / DEGREES
// #define DEG_TO_RAD(x) (x) * 3.14159265358979323846 / 180.0
#define RAD_TO_DEG(x) (x) * DEGREES / PI
// #define RAD_TO_DEG(x) (x) * 180.0 / 3.14159265358979323846

// Actuator speed (deg/s)
const float SPEED = STARTING_SPEED;
// const float SPEED = 1.8f;
const float SPEED_STOP = STOPPING_SPEED;
// const float SPEED_STOP = -40.0f;

// Gnuplot gp;

int initialize_robot_urdf(const std::string &urdf_path, KDL::Chain &robot_chain,
                          const std::string &base_link, const std::string &tool_link)
{
    KDL::Tree robot_tree;

    // load the robot URDF into the KDL tree
    if (!kdl_parser::treeFromFile(urdf_path, robot_tree))
    {
        return -1;
    }

    // create the KDL chain
    if (!robot_tree.getChain(base_link, tool_link, robot_chain))
    {
        // _logger->logError("Failed to get KDL chain");
        return -1;
    }

    //   _logger->logInfo("Successfully initialized robot urdf");
    return 0;
}

// Create an event listener that will set the promise action event to the exit value
// Will set to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent> &finish_promise)
{
    return [&finish_promise](k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch (action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

// Create an event listener that will set the sent reference to the exit value
// Will set to either END or ABORT
// Read the value of returnAction until it is set
std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_ref(k_api::Base::ActionEvent &returnAction)
{
    return [&returnAction](k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch (action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            returnAction = action_event;
            break;
        default:
            break;
        }
    };
}

bool gripper_control(k_api::Base::BaseClient *base)
{
    k_api::Base::GripperCommand gripper_command;
    gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);
    auto finger = gripper_command.mutable_gripper()->add_finger();
    finger->set_finger_identifier(1);

    for (float position = 0.0; position < 1.0; position += 0.1)
    {
        finger->set_value(position);
        base->SendGripperCommand(gripper_command);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return true;
}

bool move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list()) 
    {
        if (action.name() == "Home") 
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0) 
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
        return false;
    } 
    else 
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            return false;
        }
        const auto promise_event = finish_future.get();

        // std::cout << "Move to Home completed" << std::endl;
        // std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl; 

        return true;
    }
}


bool arm_initialisation(k_api::Base::BaseClient* base, const Pose& targetPose, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{

    auto action = k_api::Base::Action();
    action.set_name("Move to Cartesian Position");
    action.set_application_data("");

    auto constrained_pose = action.mutable_reach_pose();
    auto pose = constrained_pose->mutable_target_pose();
    pose->set_x(targetPose.x);
    pose->set_y(targetPose.y);
    pose->set_z(targetPose.z);
    pose->set_theta_x(targetPose.theta_x);
    pose->set_theta_y(targetPose.theta_y);
    pose->set_theta_z(targetPose.theta_z);

    // Connect to notification action topic
    // (Reference alternative)
    // See angular examples for Promise alternative
    k_api::Base::ActionEvent event = k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT;
    auto reference_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_ref(event),
        k_api::Common::NotificationOptions()
    );

    base->ExecuteAction(action);

    // Wait for reference value to be set
    // (Reference alternative)
    // See angular examples for Promise alternative
    // Set a timeout after 20s of wait
    const auto timeout = std::chrono::system_clock::now() + TIMEOUT_DURATION;
    while(event == k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT &&
        std::chrono::system_clock::now() < timeout)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    base->Unsubscribe(reference_notification_handle);

    if(event == k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        return false;
    }

    // std::cout << "Cartesian movement completed" << std::endl;
    // std::cout << "Reference value : " << k_api::Base::ActionEvent_Name(event) << std::endl;

    return true;

}


double find_gripper_mass(k_api::Base::BaseClient *base, k_api::BaseCyclic::BaseCyclicClient *base_cyclic, k_api::InterconnectCyclic::InterconnectCyclicClient *interconnect_cyclic)
{

    /**
    * Constructor for the estimator, it will allocate all the necessary memory
    * param chain The kinematic chain of the robot, an internal copy will be made.
    * param gravity The gravity-acceleration vector to use during the calculation.
    * param sample_frequency Frequency at which users updates it estimation loop (in Hz).
    * param estimation_gain Parameter used to control the estimator's convergence
    * param filter_constant Parameter defining how much the estimated signal should be filtered by the low-pass filter.
    *                        This input value should be between 0 and 1. Higher the number means more noise needs to be filtered-out.
    *                        The filter can be turned off by setting this value to 0.
    * param eps If a SVD-singular value is below this value, its inverse is set to zero. Default: 0.00001
    */

    k_api::Base::JointSpeeds joint_speeds;
    k_api::Base::JointSpeeds joint_speeds1;
    k_api::Base::JointTorques joint_torques;

    Vector gravity(0.0, 0.0, -9.39);
    double sample_frequency = 20.0; // Hz
    double estimation_gain = 1.0;
    double filter_constant = 0.5;
    double eps = 0.00001; 
    int n = 150; 
    double prev_force_x = 0.0;
    double prev_force_y = 0.0;
    double prev_force_z = 0.0;
    double gripper_mass = 0.0f;
    bool gripper_boolean = false;
    double prev_gripper = 0.0f;
    double current_gripper = 0.0f;
    double gripper = 0.0f;
    double force_req = 0.0f;

    // Create the KDL chain
    KDL::Chain chain;
    // initialize_robot_urdf("../../src/feedback_pouring/urdf/GEN3-7DOF-VISION_ARM_URDF_V12.urdf", chain, "base_link", "end_effector_link");
    initialize_robot_urdf("../../src/feedback_pouring/urdf/"+URDF_FILE_NAME+".urdf", chain, "base_link", "end_effector_link");

    ChainExternalWrenchEstimator extwrench_estimator(chain, gravity, sample_frequency, estimation_gain, filter_constant, eps, n);

    KDL::JntArray jnt_torque(chain.getNrOfJoints());
    KDL::JntArray jnt_pos(chain.getNrOfJoints());
    KDL::JntArray jnt_vel(chain.getNrOfJoints());
    KDL::JntArray jnt_pos_fixed(chain.getNrOfJoints());

    Wrench wrench;
    Wrench initial_wrench;

    KDL::Vector imu_acceleration;

    while (1)
    {
        auto feedback_init = base_cyclic->RefreshFeedback();
        for (int i = 0; i < chain.getNrOfJoints(); i++)
        {
            jnt_torque.data(i) = feedback_init.actuators(i).torque();
            jnt_pos.data(i) = DEG_TO_RAD(feedback_init.actuators(i).position());
            jnt_vel.data(i) = DEG_TO_RAD(feedback_init.actuators(i).velocity());
        }

        extwrench_estimator.JntToExtWrench(jnt_pos, jnt_vel, jnt_torque, initial_wrench);

        imu_acceleration = KDL::Vector(
            feedback_init.interconnect().imu_acceleration_x(),
            feedback_init.interconnect().imu_acceleration_y(),
            feedback_init.interconnect().imu_acceleration_z());

        // considering x and y direction only leaving the z axis
        auto acceleration = std::sqrt(imu_acceleration.x() * imu_acceleration.x() + imu_acceleration.y() * imu_acceleration.y());


        force_req = initial_wrench.force.Norm();
        std::cout << "Force: " << force_req << std::endl;

        // Mass of the gripper
        gripper_mass = sqrt(initial_wrench.force.x() * initial_wrench.force.x() + initial_wrench.force.y() * initial_wrench.force.y()) / acceleration;

        current_gripper = (floor(gripper_mass * 100.0)) / 100.0;
        std::cout << "Gripper Mass: " << gripper_mass << std::endl;

        if (!gripper_boolean)
        {
            if (prev_gripper != current_gripper)
            {
                prev_gripper = current_gripper;
            }
            else
            {
                gripper_boolean = true;
                gripper = current_gripper;
                break;
            }
        }
    }

    return force_req;
}

bool feedback_pouring(k_api::Base::BaseClient *base, k_api::BaseCyclic::BaseCyclicClient *base_cyclic, k_api::InterconnectCyclic::InterconnectCyclicClient *interconnect_cyclic)
{

    k_api::Base::JointSpeeds joint_speeds;
    k_api::Base::JointSpeeds joint_speeds1;
    k_api::Base::JointTorques joint_torques;

    Vector gravity(0.0, 0.0, -9.39);
    double sample_frequency = 20.0; // Hz
    double estimation_gain = 1.0;
    double filter_constant = 0.5;
    double eps = 0.00001;
    int n = 150;

    // Create the KDL chain
    KDL::Chain chain;
    // initialize_robot_urdf("../../src/feedback_pouring/urdf/GEN3-7DOF-VISION_ARM_URDF_V12.urdf", chain, "base_link", "end_effector_link");
    initialize_robot_urdf("../../src/feedback_pouring/urdf/"+URDF_FILE_NAME+".urdf", chain, "base_link", "end_effector_link");

    ChainExternalWrenchEstimator extwrench_estimator(chain, gravity, sample_frequency, estimation_gain, filter_constant, eps, n);

    KDL::JntArray jnt_torque(chain.getNrOfJoints());
    KDL::JntArray jnt_pos(chain.getNrOfJoints());
    KDL::JntArray jnt_pos_stop(chain.getNrOfJoints());
    KDL::JntArray jnt_vel(chain.getNrOfJoints());
    KDL::JntArray jnt_pos_fixed(chain.getNrOfJoints());

    Wrench wrench;
    Wrench initial_wrench;

    KDL::Vector imu_acceleration;

    std::vector<float> speeds1;
    std::vector<float> speeds;

    int actuator_count = base->GetActuatorCount().count();
    const int last_actuator_index = 6;

    extwrench_estimator.setInitialMomentum(jnt_pos, jnt_vel);

    for (size_t i = 0; i < actuator_count; ++i)
    {
        auto joint_speed = joint_speeds.add_joint_speeds();
        joint_speed->set_joint_identifier(i);
        joint_speed->set_value(0.0f);
        joint_speed->set_duration(1);
    }
    for (size_t i = 0; i < actuator_count; ++i)
    {
        auto joint_speed1 = joint_speeds1.add_joint_speeds();
        joint_speed1->set_joint_identifier(i);
        joint_speed1->set_value(0.0f);
        joint_speed1->set_duration(1);
    }

    float i = 0.05;
    float decremental_speed = 0.0f;
    float percentage = 20;
    bool loop = true;
    bool stop_loop = false;
    // double gripper_mass = 8.75;
    bool initial_wrench_set = true;
    bool target_mass_set = false;
    double target_mass = 0.0f;
    double error = 0.0f;
    float mass_extract_previous = 0.0f;
    double time_period = 0.0f;
    bool gripper_mass_set = false;

    // Initialize the previous force values
    double prev_force_x = 0.0;
    double prev_force_y = 0.0;
    double prev_force_z = 0.0;
    float initial_mass = 0.0f;
    float final_mass = 0.0f;

    std::vector<std::pair<double, double>> combined;
    std::vector<std::pair<double, double>> Fxy_pts;
    std::vector<std::pair<double, double>> Fyy_pts;

    while (loop)
    {
        auto feedback_init = base_cyclic->RefreshFeedback();

        imu_acceleration = KDL::Vector(
            feedback_init.interconnect().imu_acceleration_x(),
            feedback_init.interconnect().imu_acceleration_y(),
            feedback_init.interconnect().imu_acceleration_z()

        );

        // auto acceleration = std::sqrt(imu_acceleration.x()*imu_acceleration.x() + imu_acceleration.y()*imu_acceleration.y());
        double acceleration = imu_acceleration.Norm();
        for (int i = 0; i < chain.getNrOfJoints(); i++)
        {
            jnt_torque.data(i) = feedback_init.actuators(i).torque();
            jnt_pos.data(i) = DEG_TO_RAD(feedback_init.actuators(i).position());
            jnt_vel.data(i) = DEG_TO_RAD(feedback_init.actuators(i).velocity());
        }

        extwrench_estimator.JntToExtWrench(jnt_pos, jnt_vel, jnt_torque, wrench);

        // std::cout << "Force_x: " << wrench.force.x() << " Force_y: " << wrench.force.y() << " Force_z: " << wrench.force.z() << std::endl;

        // double force = std::sqrt(wrench.force.x()*wrench.force.x() + wrench.force.y()*wrench.force.y());
        // To match the force to Zero when there is no object given to gripper --> 83.75 (found this value with trial and error)
        double force = wrench.force.Norm() - 83.75;

        auto mass = force / acceleration;
        
        // To stabilise the weight before getting the actual weight 
        // (eg: In measuring scale, the weight won't be measured immediately, it shows the actual weight with some delay)
        float mass_extract_current = (floor(mass * 1000.0)) / 1000.0;

        if (!target_mass_set)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            if (mass_extract_previous != mass_extract_current)
            {
                mass_extract_previous = mass_extract_current;
            }
            else if (mass_extract_current > 0.09)
            {
                initial_mass = mass;
                target_mass = mass * (1 - percentage / 100);
                // std::cout << "Calculating mass while pouring: " << target_mass << std::endl;
                final_mass = target_mass;
                target_mass_set = true;
            }
        }
        std::cout << "***** Pouring in progress *****" << std::endl;

        // To match the user defined percentage with the measured weight (to get the accuracy -> mass > target mass + 15% of target mass)
        if (target_mass)
        {
            if (mass > (target_mass + 0.15 * target_mass))
            {
                decremental_speed = SPEED / i;
                // joint_speeds.mutable_joint_speeds(last_actuator_index)->set_value(decremental_speed);
                joint_speeds.mutable_joint_speeds(last_actuator_index)->set_value(SPEED);
                base->SendJointSpeedsCommand(joint_speeds);
            }
            else if (mass < (target_mass + 0.15 * target_mass))
            {
                while (1)
                {

                    auto feedback_stop = base_cyclic->RefreshFeedback();
                    for (int i = 0; i < chain.getNrOfJoints(); i++)
                    {
                        jnt_pos_stop.data(i) = DEG_TO_RAD(feedback_stop.actuators(i).position());
                    }
                    joint_speeds1.mutable_joint_speeds(last_actuator_index)->set_value(SPEED_STOP);
                    base->SendJointSpeedsCommand(joint_speeds1);
                    if (RAD_TO_DEG(jnt_pos_stop.data(6)) < 88)
                    {

                        break;
                    }
                }

                loop = false;
            }
        }


        i = i + 0.01;
    }

    // Stop the robot
    std::cout << "Initial mass ----> " << initial_mass << std::endl;
    std::cout << "Final mass ----> " << final_mass << std::endl;
    std::cout << "Stopping the robot" << std::endl;
    base->Stop();

    return true;
}

int main(int argc, char **argv)
{

    // Create API objects
    auto error_callback = [](k_api::KError err)
    { cout << "_________ callback error _________" << err.toString(); };
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(ROBOT_IP, PORT);
    // 192.168.1.12 for another arm mounted to table

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(USER_NAME);
    create_session_info.set_password(PASSWORD);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating session for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    std::cout << "Session created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);
    auto interconnect_cyclic = new k_api::InterconnectCyclic::InterconnectCyclicClient(router);
    double force_by_gripper = 0.0f;
    

    // Set the target pose
    // Pose targetPose = {0.573, -0.03, 0.149, 90.093, 0.003, 89.917};
    Pose targetPose = TARGET_POSE;

    // core
    // Example core
    bool success = true;
    success &= move_to_home_position(base);
    success &= arm_initialisation(base, targetPose, base_cyclic);
    
    // force_by_gripper =  find_gripper_mass(base, base_cyclic, interconnect_cyclic);

    success &= feedback_pouring(base, base_cyclic, interconnect_cyclic);

    // Close API session
    session_manager->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();

    // Destroy the API
    delete base;
    delete session_manager;
    delete router;
    delete transport;

    return success ? 0 : 1;
};