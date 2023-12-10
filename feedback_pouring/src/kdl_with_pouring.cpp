

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

    // Creating KDL chain using URDF
    KDL::Chain chain;
    initialize_robot_urdf("../../src/feedback_pouring/urdf/gen3_robotiq_2f_85.urdf", chain, "base_link", "end_effector_link");

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

    return true;
};