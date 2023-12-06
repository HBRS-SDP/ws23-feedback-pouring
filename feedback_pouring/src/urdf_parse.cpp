#include "chain.hpp"
#include "tree.hpp"
// #include "frames_io.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chaindynparam.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"

// #include "/home/mas/workspace/SDP/mir/src/mas_industrial_robotics/mir_manipulation/ws23-feedback-pouring/orocos_kinematics_dynamics/orocos_kdl/tests/solvertest.cpp"

int initialize_robot_urdf(const std::string& urdf_path, KDL::Chain& robot_chain,
                                 const std::string& base_link, const std::string& tool_link)
{
  KDL::Tree robot_tree;

  // load the robot URDF into the KDL tree
  if (!kdl_parser::treeFromFile(urdf_path, robot_tree))
  {
    // _logger->logError("Failed to construct KDL tree");
    
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

int getLinkIdFromChain(KDL::Chain& chain, const std::string& link_name)
{
  for (int i = 0; i < chain.getNrOfSegments(); i++)
  {
    if (chain.getSegment(i).getName() == link_name)
    {
      return i;
    }
  }

  return -1;
}

int main(int argc, char **argv)
{    
    // success &= example_move_to_home_position(base);
    KDL::Chain chain;

    initialize_robot_urdf("/home/mas/workspace/SDP/mir/src/mas_industrial_robotics/mir_manipulation/ws23-feedback-pouring/feedback_pouring/urdf/gen3_robotiq_2f_85.urdf", chain, "base_link", "bracelet_link");
    // ExternalWrenchEstimatorTest()
    for (int i = 0; i < chain.getNrOfSegments(); i++)
  {
    std::cout << chain.getSegment(i).getName() << std::endl;
  }
    std::cout << chain.getNrOfSegments() << std::endl;
    std::cout << chain.getNrOfJoints() << std::endl;

    KDL::Vector gravity_vector(0.0, 0.0, -9.81);  // Assuming Earth gravity in the z-direction

    // Assuming you have already set up joint_positions, joint_velocities, and joint_accelerations
    KDL::JntArray joint_positions(chain.getNrOfJoints());
    KDL::JntArray joint_velocities(chain.getNrOfJoints());
    KDL::JntArray joint_accelerations(chain.getNrOfJoints());

    // Compute the Cartesian velocities and accelerations
    KDL::ChainDynParam dyn_param(chain, gravity_vector);
    KDL::JntArray joint_efforts(chain.getNrOfJoints());
    dyn_param.JntToGravity(joint_positions, joint_efforts);

    // Assuming you have also defined the kinematic chain and state
    KDL::ChainFkSolverPos_recursive fk_solver(chain);

    // Declare the jac_solver object here
    KDL::ChainJntToJacSolver jac_solver(chain);

    // Calculate Jacobian
    KDL::Jacobian jac;
    jac_solver.JntToJac(joint_positions, jac);

    // Convert joint efforts to Cartesian wrench
    Eigen::VectorXd joint_efforts_eigen = joint_efforts.data;
    Eigen::VectorXd wrench_eigen = jac.data * joint_efforts_eigen;

    // Create a KDL::Wrench and set its values manually
    KDL::Wrench wrench;
    wrench.force.x(jac.data(0, 0));
    wrench.force.y(jac.data(1, 0));
    wrench.force.z(jac.data(2, 0));
    wrench.torque.x(jac.data(3, 0));
    wrench.torque.y(jac.data(4, 0));
    wrench.torque.z(jac.data(5, 0));

    // Print wrench components
    std::cout << "Force: (" << wrench.force.x() << ", " << wrench.force.y() << ", " << wrench.force.z() << ")" << std::endl;
    std::cout << "Torque: (" << wrench.torque.x() << ", " << wrench.torque.y() << ", " << wrench.torque.z() << ")" << std::endl;

    return 0;
}