#include "chain.hpp"
#include "tree.hpp"
// #include "frames_io.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "chainexternalwrenchestimator.hpp"

using namespace KDL;

int initialize_robot_urdf(const std::string& urdf_path, KDL::Chain& robot_chain,
                                 const std::string& base_link, const std::string& tool_link)
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

KDL::Wrench end_effector_wrench( std::vector<KDL::Wrench>& wrench_history)
{
    
    Vector gravity(0.0, 0.0, -9.81);
    double sample_frequency = 1000.0; // Hz
    double estimation_gain = 45.0;
    double filter_constant = 0.5;

    // Create the KDL chain
    KDL::Chain chain;
    initialize_robot_urdf("../../src/feedback_pouring/urdf/gen3_robotiq_2f_85.urdf", chain, "base_link", "end_effector_link");

    for (int i = 0; i < chain.getNrOfSegments(); i++)
    {
      std::cout << chain.getSegment(i).getName() << std::endl;
    }

    // declare the solver to estimate the external wrench
    ChainExternalWrenchEstimator extwrench_estimator(chain, gravity, sample_frequency, estimation_gain, filter_constant);
    KDL::JntArray jnt_pos(chain.getNrOfJoints());
    KDL::JntArray jnt_vel(chain.getNrOfJoints());
    KDL::JntArray jnt_torque(chain.getNrOfJoints());
    Wrench f_tool_estimated;

    jnt_pos.data << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
    jnt_vel.data << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
    jnt_torque.data << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;

    extwrench_estimator.JntToExtWrench(jnt_pos, jnt_vel, jnt_torque, f_tool_estimated);


    return f_tool_estimated;
}

int main(int argc, char **argv)
{

  
  std::vector<KDL::Wrench> torque_history;

     
  KDL::Wrench f_tool_estimated = end_effector_wrench(torque_history);

  // Print the f_tool_estimated
  std::cout << "Estimated external wrench (force, torque): " << std::endl;
  std::cout << "Force: " << f_tool_estimated.force.x() << ", " << f_tool_estimated.force.y() << ", " << f_tool_estimated.force.z() << std::endl;
  std::cout << "Torque: " << f_tool_estimated.torque.x() << ", " << f_tool_estimated.torque.y() << ", " << f_tool_estimated.torque.z() << std::endl;



    return 0;
}