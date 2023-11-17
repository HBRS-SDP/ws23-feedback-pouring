

#include <BaseClientRpc.h>

#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <DeviceManagerClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <Common.pb.h>



namespace k_api = Kinova::Api;

#define PORT 10000

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);

// Actuator speed (deg/s)
const float SPEED = 8.0f;
const float SPEED_STOP = 30.0f;

// Create an event listener that will set the promise action event to the exit value
// Will set to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)> 
    create_action_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
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


bool example_send_joint_speeds(k_api::Base::BaseClient* base)
{
    std::cout << "Sending the angular velocities to the robot for 10 seconds..." << std::endl;

    k_api::Base::JointSpeeds joint_speeds;
    k_api::Base::JointSpeeds joint_speeds1;

    std::vector<float> speeds;
    std::vector<float> speeds1;
    // The 7DOF robot will spin in the same direction for 10 seconds    
    int actuator_count = base->GetActuatorCount().count();
    if (actuator_count == 7)
    {
        speeds = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, SPEED};
        for (size_t i = 0 ; i < speeds.size(); ++i)
        {
            auto joint_speed = joint_speeds.add_joint_speeds();
            joint_speed->set_joint_identifier(i);
            joint_speed->set_value(speeds.at(i));
            joint_speed->set_duration(1);
        }
        base->SendJointSpeedsCommand(joint_speeds);

        // Wait 10 seconds
        std::this_thread::sleep_for(std::chrono::milliseconds(11000));

        speeds1= {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -SPEED_STOP};
        for (size_t i = 0 ; i < speeds1.size(); ++i)
        {
            auto joint_speed1 = joint_speeds1.add_joint_speeds();
            joint_speed1->set_joint_identifier(i);
            joint_speed1->set_value(speeds1.at(i));
            joint_speed1->set_duration(1);
        }
        base->SendJointSpeedsCommand(joint_speeds1);

        // Wait 10 seconds
        std::this_thread::sleep_for(std::chrono::milliseconds(2500));

    }
    else
    {
        std::cout << "This code is intended to work only with the 7DOF robot." << std::endl;
        return false;
    }
    
    // Stop the robot
    std::cout << "Stopping the robot" << std::endl;
    base->Stop();

    return true;
}

int main(int argc, char **argv)
{
    

    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
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
    
    // Example core
    bool success = true;
    // success &= example_move_to_start_position(base);
    success &= example_send_joint_speeds(base);

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

    return success ? 0: 1;
};