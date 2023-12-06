
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>

#include <kdl.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <eigen3/Eigen/Core>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <chainhdsolver_vereshchagin.hpp>
#include <chainexternalwrenchestimator.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl_parser.hpp>

#include <urdf/model.h>

#include <random>
#include <time.h>

#define PORT 10000

namespace k_api = Kinova::Api;

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_DURATION = std::chrono::seconds{20};

// Data structure to represent a 3D pose
struct Pose {
    double x;
    double y;
    double z;
    double theta_x;
    double theta_y;
    double theta_z;
};

//Extract kinova model from URDF file
KDL::Chain get_robot_model() 
{
    urdf::Model urdf_model;
    KDL::Tree tree;
    KDL::Chain full_chain;

    if (!urdf_model.initFile("/home/vicky/sdp/ws23-feedback-pouring/feedback_pouring/urdf/gen3_robotiq_2f_85.urdf"))
    {
        printf("ERROR: Failed to parse urdf robot model \n");
        assert(0);
    }

    //Extract KDL tree from the URDF file
    if (!kdl_parser::treeFromUrdfModel(urdf_model, tree))
    {
        printf("ERROR: Failed to construct kdl tree \n");
        assert(0);
    }

    //Extract KDL chain from KDL tree
    tree.getChain("base_link", "end_effector_link", full_chain);
    return full_chain; 
}


// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)> 
    create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
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

// Create an event listener that will set the sent reference to the exit value
// Will set to either END or ABORT
// Read the value of returnAction until it is set
std::function<void(k_api::Base::ActionNotification)>
    create_event_listener_by_ref(k_api::Base::ActionEvent& returnAction)
{
    return [&returnAction](k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
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

bool move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
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

        std::cout << "Move to Home completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl; 

        return true;
    }
}


bool move_to_cartesian_position(k_api::Base::BaseClient* base, const Pose& targetPose, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    std::cout << "Moving to Desired position" << std::endl;

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

    std::cout << "Executing action" << std::endl;
    base->ExecuteAction(action);

    std::cout << "Waiting for movement to finish ..." << std::endl;

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

    std::cout << "Cartesian movement completed" << std::endl;
    std::cout << "Reference value : " << k_api::Base::ActionEvent_Name(event) << std::endl;

    return true;

}

void external_wrench_estimator_test()
{
    /**
     * Closed-loop test for the external wrench estimator class:
     * Simple controlled behaviour of the robot subjected to an external force is simulated.
     * The external wrench estimator is called in each iteration of the control loop 
     * so to converge to final wrench value.
     * In the end, estimated wrench is compared to the ground-truth values of the simulated wrench. 
     */
    std::cout << "KDL External Wrench Estimator Test" << std::endl;

    /**
     * This EPS has a slightly different purpose than the EPSes of the other solver-tests. While other EPSes are taking care of the differences that
     * originate from e.g. floating-number imprecisions, different compilers (or same compiler but different flags) used between different machines (OS), etc. 
     * The EPS specified below is there to cover those imperfections as well but, it's also there to
     * take into account the noise in estimated signals (the differences between estimated and ground-truth wrenches), caused by other computations in this test
     * (ones coming from the implemented controller and the dynamics simulator) not just those coming from the estimator itself.
     */
    double eps_wrench = 0.5, eps_torque = 0.3;
    int ret;
    KDL::Chain kinova_model = get_robot_model();
    unsigned int nj = kinova_model.getNrOfJoints();
    unsigned int ns = kinova_model.getNrOfSegments();

    // Initialize state and control variables
    KDL::JntArray q(nj); // Current joint position
    KDL::JntArray qd(nj); // Current joint velocity
    KDL::JntArray qdd(nj); // Resultant joint accelerations
    KDL::JntArrayVel jnt_position_velocity(nj); // variable necessary for the FK vel solver
    KDL::JntArray jnt_array_zero(nj); // Zero joint input for RNE
    KDL::JntArray command_torque(nj); // Control torque to actuate the robot
    KDL::JntArray constraint_tau(nj); // It will result in zero in Vereshchagin for this test
    KDL::JntArray gravity_torque(nj); // Gravity torque computed by RNE
    KDL::JntArray ext_torque_reference(nj); // Ground-truth joint torques due to the external force applied on the end-effector
    KDL::JntArray ext_torque_estimated(nj); // Estimated joint torques
    KDL::Wrenches f_ext_base(ns); // External Wrenches acting on the end-effector, expressed in base-link coordinates
    KDL::Wrenches f_ext_zero(ns); // Zero Wrenches
    KDL::Wrench f_tool_estimated; // External Wrenches estimated by the momentum-observer
    KDL::Frame end_effector_pose;
    KDL::Frame desired_end_eff_pose;
    KDL::Jacobian jacobian_end_eff(nj);
    KDL::FrameVel end_eff_twist;
    KDL::FrameVel desired_end_eff_twist;
    Eigen::Matrix<double, 6, 1> end_eff_force; // variable necessary for the control
    Eigen::Matrix<double, 6, 1> end_eff_pos_error; // variable necessary for the control
    Eigen::Matrix<double, 6, 1> end_eff_vel_error; // variable necessary for the control

    // Arm root acceleration (robot's base mounted on an even surface)
    KDL::Vector linearAcc(0.0, 0.0, -9.81); KDL::Vector angularAcc(0.0, 0.0, 0.0);
    
    // Initialize kinematics solvers
    KDL::ChainFkSolverPos_recursive fksolverpos(kinova_model);
    KDL::ChainFkSolverVel_recursive fksolvervel(kinova_model);
    KDL::ChainJntToJacSolver jacobian_solver(kinova_model);

    // RNE ID solver for control purposes
    KDL::ChainIdSolver_RNE IdSolver(kinova_model, linearAcc);

    // Vereshchagin Hybrid Dynamics solver for simulation purposes
    int numberOfConstraints = 6;
    KDL::Jacobian alpha(numberOfConstraints); // Constraint Unit forces at the end-effector
    KDL::JntArray beta(numberOfConstraints); // Acceleration energy at the end-effector
    KDL::SetToZero(alpha); // Set to zero to deactivate all constraints
    KDL::SetToZero(beta); // Set to zero to deactivate all constraints
    KDL::Twist vereshchagin_root_Acc(-linearAcc, angularAcc); // Note: Vereshchagin solver takes root acc. with opposite sign comparead to the above FD and RNE solvers
    KDL::ChainHdSolver_Vereshchagin constraintSolver(kinova_model, vereshchagin_root_Acc, numberOfConstraints);

    // External Wrench Estimator
    double sample_frequency = 1000.0; // Hz
    double estimation_gain  = 45.0;
    double filter_constant  = 0.5;
    KDL::ChainExternalWrenchEstimator extwrench_estimator(kinova_model, linearAcc, sample_frequency, estimation_gain, filter_constant);

    // Prepare test cases
    std::vector<KDL::JntArray> jnt_pos;
    std::vector<KDL::Wrench> wrench_reference;

    // Initialize random generator
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis_force(-15.0, 15.0);
    std::uniform_real_distribution<> dis_moment(-0.9, 0.9);
    std::uniform_real_distribution<> dis_jnt_vel(-0.5, 0.5);

    // Set first test case
    q(0) = 1.0;
    q(1) = 0.0;
    q(2) = 0.0;
    q(3) = 4.71;
    q(4) = 0.0;
    q(5) = 1.57;
    q(6) = 5.48;
    jnt_pos.push_back(q);
    wrench_reference.push_back(KDL::Wrench(KDL::Vector(dis_force(gen), dis_force(gen), dis_force(gen)), KDL::Vector(0.0, 0.0, 0.0))); // Ground-truth external wrench acting on the end-effector expressed in local end-effector's frame

    // Set second test case
    q(0) = 2.96;
    q(1) = 1.02;
    q(2) = 6.15;
    q(3) = 1.61;
    q(4) = 0.22;
    q(5) = 0.17;
    q(6) = 0.01;
    jnt_pos.push_back(q);
    wrench_reference.push_back(KDL::Wrench(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(dis_moment(gen), dis_moment(gen), 0.0))); // expressed in local end-effector's frame

    // Set third test case
    q(0) = 1.12;
    q(1) = 0.66;
    q(2) = 6.15;
    q(3) = 4.09;
    q(4) = 1.64;
    q(5) = 0.12;
    q(6) = 0.01;
    jnt_pos.push_back(q);
    wrench_reference.push_back(KDL::Wrench(KDL::Vector(dis_force(gen), dis_force(gen), dis_force(gen)), KDL::Vector(dis_moment(gen), 0.0, dis_moment(gen)))); // expressed in local end-effector's frame

    // ##########################################################################################
    // Control and simulation
    // ##########################################################################################

    // Control gains for a simple PD controller
    double k_p = 1500.0; // Proportional
    double k_d = 300.0; // Derivative

    // Time required to complete the task
    double simulationTime = 0.4; // in seconds
    double timeDelta = 1.0 / sample_frequency; // unit of seconds

    // Iterate over test cases
    for (unsigned int i = 0; i < jnt_pos.size(); i++)
    {
        // Re-set control and simulation variables
        q = jnt_pos[i];
        qd(0) = dis_jnt_vel(gen);
        qd(1) = dis_jnt_vel(gen);
        qd(2) = dis_jnt_vel(gen);
        qd(3) = dis_jnt_vel(gen);
        qd(4) = dis_jnt_vel(gen);
        qd(5) = dis_jnt_vel(gen);
        qd(6) = dis_jnt_vel(gen);

        end_eff_force.setZero();
        end_eff_pos_error.setZero();
        end_eff_vel_error.setZero();
        f_ext_base = f_ext_zero;

        // Initialize the estimator
        extwrench_estimator.updateInternalDataStructures();
        extwrench_estimator.setInitialMomentum(q, qd); // sets the offset for future estimation (momentum calculation)

        // Set the desired Cartesian state
        fksolverpos.JntToCart(q, end_effector_pose);
        desired_end_eff_pose.p(0) = end_effector_pose.p(0) + 0.02;
        desired_end_eff_pose.p(1) = end_effector_pose.p(1) + 0.02;
        desired_end_eff_pose.p(2) = end_effector_pose.p(2) + 0.02;
        desired_end_eff_twist.p.v(0) = 0.0;
        desired_end_eff_twist.p.v(1) = 0.0;
        desired_end_eff_twist.p.v(2) = 0.0;

        for (double t = 0.0; t <= simulationTime; t = t + timeDelta)
        {
            ret = jacobian_solver.JntToJac(q, jacobian_end_eff);
            if (ret < 0)
            {
                std::cout << "Jacobian solver ERROR: " << ret << std::endl;
                break;
            }

            ret = fksolverpos.JntToCart(q, end_effector_pose);
            if (ret < 0)
            {
                std::cout << "FK pos solver ERROR: " << ret << std::endl;
                break;
            }

            jnt_position_velocity.q = q;
            jnt_position_velocity.qdot = qd;
            ret = fksolvervel.JntToCart(jnt_position_velocity, end_eff_twist);
            if (ret < 0)
            {
                std::cout << "FK vel solver ERROR: " << ret << std::endl;
                break;
            }

            end_eff_pos_error(0) = end_effector_pose.p(0) - desired_end_eff_pose.p(0);
            end_eff_pos_error(1) = end_effector_pose.p(1) - desired_end_eff_pose.p(1);
            end_eff_pos_error(2) = end_effector_pose.p(2) - desired_end_eff_pose.p(2);

            end_eff_vel_error(0) = end_eff_twist.p.v(0) - desired_end_eff_twist.p.v(0);
            end_eff_vel_error(1) = end_eff_twist.p.v(1) - desired_end_eff_twist.p.v(1);
            end_eff_vel_error(2) = end_eff_twist.p.v(2) - desired_end_eff_twist.p.v(2);

            end_eff_force = -end_eff_pos_error * k_p - end_eff_vel_error * k_d;

            // Compute gravity joint torques (hide external wrench from this dynamics calculation)
            ret = IdSolver.CartToJnt(q, jnt_array_zero, jnt_array_zero, f_ext_zero, gravity_torque);
            if (ret < 0)
            {
                std::cout << "KDL RNE solver ERROR: " << ret << std::endl;
                break;
            }

            // Compute joint control commands
            command_torque.data =  jacobian_end_eff.data.transpose() * end_eff_force;
            command_torque.data += gravity_torque.data;

            // Start simulating the external force
            if (t > 0.2) f_ext_base[ns - 1] = end_effector_pose.M * wrench_reference[i];

            // Compute resultant joint accelerations that simulate robot's behaviour, given the command torques (add external wrench in this dynamics calculation)
            ret = constraintSolver.CartToJnt(q, qd, qdd, alpha, beta, f_ext_base, command_torque, constraint_tau);
            if (ret < 0)
            {
                std::cout << "KDL Vereshchagin solver ERROR: " << ret << std::endl;
                break;
            }
            
            // State integration: integrate from model accelerations to next joint state (positions and velocities)
            qd.data = qd.data + qdd.data * timeDelta; // Euler Forward
            q.data = q.data + qd.data * timeDelta; // Symplectic Euler

            // Saturate integrated joint position for full circle crossing
            for (unsigned int j = 0; j < nj; j++)
            {
                q(j) = std::fmod(q(j), 360 * KDL::deg2rad);
                if (q(j) < 0.0) q(j) += 360 * KDL::deg2rad;
            }
            
            // Estimate external wrench
            extwrench_estimator.JntToExtWrench(q, qd, command_torque, f_tool_estimated);
        }

        // Inverse Force Kinematics
        Eigen::Matrix<double, 6, 1> wrench;
        wrench(0) = f_ext_base[ns - 1](0);
        wrench(1) = f_ext_base[ns - 1](1);
        wrench(2) = f_ext_base[ns - 1](2);
        wrench(3) = f_ext_base[ns - 1](3);
        wrench(4) = f_ext_base[ns - 1](4);
        wrench(5) = f_ext_base[ns - 1](5);
        ext_torque_reference.data = jacobian_end_eff.data.transpose() * wrench;

        // Get estimated joint torque 
        extwrench_estimator.getEstimatedJntTorque(ext_torque_estimated);

        // ##################################################################################
        // Final comparison
        // ##################################################################################
        // CPPUNIT_ASSERT(Equal(f_tool_estimated(0), wrench_reference[i](0), eps_wrench));
        // CPPUNIT_ASSERT(Equal(f_tool_estimated(1), wrench_reference[i](1), eps_wrench));
        // CPPUNIT_ASSERT(Equal(f_tool_estimated(2), wrench_reference[i](2), eps_wrench));
        // CPPUNIT_ASSERT(Equal(f_tool_estimated(3), wrench_reference[i](3), eps_wrench));
        // CPPUNIT_ASSERT(Equal(f_tool_estimated(4), wrench_reference[i](4), eps_wrench));
        // CPPUNIT_ASSERT(Equal(f_tool_estimated(5), wrench_reference[i](5), eps_wrench));

        // CPPUNIT_ASSERT(Equal(ext_torque_estimated(0), ext_torque_reference(0), eps_torque));
        // CPPUNIT_ASSERT(Equal(ext_torque_estimated(1), ext_torque_reference(1), eps_torque));
        // CPPUNIT_ASSERT(Equal(ext_torque_estimated(2), ext_torque_reference(2), eps_torque));
        // CPPUNIT_ASSERT(Equal(ext_torque_estimated(3), ext_torque_reference(3), eps_torque));
        // CPPUNIT_ASSERT(Equal(ext_torque_estimated(4), ext_torque_reference(4), eps_torque));
        // CPPUNIT_ASSERT(Equal(ext_torque_estimated(5), ext_torque_reference(5), eps_torque));
        // CPPUNIT_ASSERT(Equal(ext_torque_estimated(6), ext_torque_reference(6), eps_torque));
    }

    return;
}


int main(int argc, char **argv)
{

    KDL::Chain kinova_model = get_robot_model();

    std::cout << "Segments:  " << kinova_model.getNrOfSegments() << "Joints:  " << kinova_model.getNrOfJoints() << std::endl;

    // Create API objects
    // auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    // auto transport = new k_api::TransportClientTcp();
    // auto router = new k_api::RouterClient(transport, error_callback);
    // transport->connect("192.168.1.12", PORT);

    // // Set session data connection information
    // auto create_session_info = k_api::Session::CreateSessionInfo();
    // create_session_info.set_username("admin");
    // create_session_info.set_password("admin");
    // create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    // create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // // Session manager service wrapper
    // std::cout << "Creating session for communication" << std::endl;
    // auto session_manager = new k_api::SessionManager(router);
    // session_manager->CreateSession(create_session_info);
    // std::cout << "Session created" << std::endl;

    // // Create services
    // auto base = new k_api::Base::BaseClient(router);
    // auto base_feedback = new k_api::BaseCyclic::BaseCyclicClient(router);

    // // Set the target pose
    // Pose targetPose = {0.573, -0.03, 0.149, 90.093, 0.003, 89.917};

    // // core
    // bool success = true;
    // success &= move_to_home_position(base);
    // success &= move_to_cartesian_position(base, targetPose, base_feedback);

    // // You can also refer to the 110-Waypoints examples if you want to execute
    // // a trajectory defined by a series of waypoints in joint space or in Cartesian space
    
    // // Close API session
    // session_manager->CloseSession();

    // // Deactivate the router and cleanly disconnect from the transport object
    // router->SetActivationStatus(false);
    // transport->disconnect();

    // // Destroy the API
    // delete base;
    // delete session_manager;
    // delete router;
    // delete transport;

    // return success? 0: 1;
    return 1;
}
