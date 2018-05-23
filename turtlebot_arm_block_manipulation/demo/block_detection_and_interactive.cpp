#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <turtlebot_arm_block_manipulation/BlockDetectionAction.h>
#include <turtlebot_arm_block_manipulation/InteractiveBlockManipulationAction.h>

#include <string>
#include <sstream>

const std::string pick_and_place_topic = "/pick_and_place";

namespace turtlebot_arm_block_manipulation
{

class BlockManipulationAction
{

private:
	ros::NodeHandle nh_;
	//Actions and services
	actionlib::SimpleActionClient<BlockDetectionAction> block_detection_action_;
	actionlib::SimpleActionClient<InteractiveBlockManipulationAction> interactive_manipulation_action_;

	BlockDetectionGoal block_detection_goal_;
	InteractiveBlockManipulationGoal interactive_manipulation_goal_;

	//Parameters
	std::string arm_link;
	double z_down;
	double block_size;
	bool once;

public:
	BlockManipulationAction() : nh_("~"),
		block_detection_action_("block_detection",true),
		interactive_manipulation_action_("interactive_manipulation",true)
	{
		//load parameters
		nh_.param<std::string>("arm_link",arm_link,"/arm_link");
		nh_.param<double>("table_height",z_down,0.01);
		nh_.param<double>("block_size",block_size,0.03);
		nh_.param<bool>("once",once,false);

		// Initialize goals
		block_detection_goal_.frame = arm_link;
		block_detection_goal_.table_height = z_down;
		block_detection_goal_.block_size = block_size;

		interactive_manipulation_goal_.block_size = block_size;
		interactive_manipulation_goal_.frame = arm_link;

		ROS_INFO("Finished initializing, waiting for servers...");

		block_detection_action_.waitForServer();
		ROS_INFO("Found block detection server.");

		interactive_manipulation_action_.waitForServer();
		ROS_INFO("Found interactive manipulation.");

		ROS_INFO("Found servers.");
	}

	void detectBlocks()
	{
		block_detection_action_.sendGoal(block_detection_goal_, boost::bind(&BlockManipulationAction::addBlocks,this,_1,_2));
	}

	void addBlocks(const actionlib::SimpleClientGoalState& state, const BlockDetectionResultConstPtr& result)
	{
		ROS_INFO("Got block detection callback. Adding blocks.");
		geometry_msgs::Pose block;

		if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Succeeded!");
		else
		{
			ROS_INFO("Did not Succeeded! %s",state.toString().c_str());
			ros::shutdown();
		}
		interactive_manipulation_action_.sendGoal(interactive_manipulation_goal_);
	}
	
};

};

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "block_manipulation");

  turtlebot_arm_block_manipulation::BlockManipulationAction manip;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  while(ros::ok())
  {
  	manip.detectBlocks();
  	std::cout << "Press Enter for restarting block detection" << std::endl;
  	std::cin.ignore();
  }
  spinner.stop();
}
