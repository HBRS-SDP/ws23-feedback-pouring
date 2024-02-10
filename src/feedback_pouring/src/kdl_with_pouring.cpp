

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <DeviceManagerClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <Common.pb.h>
#include <fstream>

#include "chain.hpp"
#include "tree.hpp"
// #include "frames_io.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "chainexternalwrenchestimator.hpp"

using namespace KDL;

namespace k_api = Kinova::Api;

#define PORT 10000

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);

// Actuator speed (deg/s)
const float SPEED = 5.0f;
const float SPEED_STOP = -30.0f;

int initialize_robot_urdf(const std::string &urdf_path, KDL::Chain &robot_chain,
                          const std::string &base_link, const std::string &tool_link)
{
    KDL::Tree robot_tree;

    // load the robot URDF into the KDL tree
    if (!kdl_parser::treeFromFile(urdf_path, robot_tree))
    {
        // printf("Failed to construct KDL tree");

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
create_action_event_listener_by_promise(std::promise<k_api::Base::ActionEvent> &finish_promise)
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

bool control_end_effector(k_api::Base::BaseClient *base, k_api::BaseCyclic::BaseCyclicClient *base_cyclic, std::vector<std::vector<float>> &torque_history, std::vector<KDL::Wrench> &wrench_history)
{
    std::cout << "Sending the angular velocities to the robot for 10 seconds..." << std::endl;

    k_api::Base::JointSpeeds joint_speeds;
    k_api::Base::JointTorques joint_torques;

    Vector gravity(0.0, 0.0, -9.81);
    double sample_frequency = 1000.0; // Hz
    double estimation_gain = 45.0;
    double filter_constant = 0.5;

    // Create the KDL chain
    KDL::Chain chain;
    initialize_robot_urdf("../../src/feedback_pouring/urdf/gen3_robotiq_2f_85.urdf", chain, "base_link", "end_effector_link");

    ChainExternalWrenchEstimator extwrench_estimator(chain, gravity, sample_frequency, estimation_gain, filter_constant);
    KDL::JntArray jnt_torque(chain.getNrOfJoints());
    KDL::JntArray jnt_pos(chain.getNrOfJoints());
    KDL::JntArray jnt_vel(chain.getNrOfJoints());
    Wrench f_tool_estimated;

    std::vector<float> torque_snapshot;

    std::vector<float> speeds;

    int actuator_count = base->GetActuatorCount().count();

    speeds = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, SPEED};
    for (size_t i = 0; i < speeds.size(); ++i)
    {
        auto joint_speed = joint_speeds.add_joint_speeds();
        joint_speed->set_joint_identifier(i);
        joint_speed->set_value(speeds.at(i));
        joint_speed->set_duration(1);
    }
    for (int time = 0; time <= 10; ++time)
    {
        base->SendJointSpeedsCommand(joint_speeds);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        auto feedback = base_cyclic->RefreshFeedback();
        for (int i = 0; i < actuator_count; i++)
        {

            // torque_snapshot.push_back(feedback.actuators(i).torque());
            jnt_torque.data(i) = feedback.actuators(i).torque();
            jnt_pos.data(i) = feedback.actuators(i).position();
            jnt_vel.data(i) = feedback.actuators(i).velocity();

            extwrench_estimator.JntToExtWrench(jnt_pos, jnt_vel, jnt_torque, f_tool_estimated);
        }

        wrench_history.push_back(f_tool_estimated);
    }

    // Stop the robot
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
    transport->connect("192.168.1.12", PORT);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
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

    bool success = true;
    std::vector<std::vector<float>> torque_history;
    std::vector<KDL::Wrench> wrench_history;

    success &= control_end_effector(base, base_cyclic, torque_history, wrench_history);

    // Print wrench history
    for (const auto &wrench : wrench_history)
    {
        std::cout << "Wrench (Force): "
                  << wrench.force.x() << ", "
                  << wrench.force.y() << ", "
                  << wrench.force.z()
                  << std::endl;

        std::cout << "Wrench (Torque): "
                  << wrench.torque.x() << ", "
                  << wrench.torque.y() << ", "
                  << wrench.torque.z()
                  << std::endl;
    }

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