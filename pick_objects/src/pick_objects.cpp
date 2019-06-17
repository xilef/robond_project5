#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <string>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PickObjectNode
{
public:
	PickObjectNode() : m_mbClient("move_base", true)
	{
		while(!m_mbClient.waitForServer(ros::Duration(5.0)))
		{
			ROS_INFO("Waiting for the move_base action server to come up");
		}
		
		setupGoals();
	}
	
	int startMove()
	{
		move_base_msgs::MoveBaseGoal goal;

		for (unsigned int x = 0; x < m_mbGoal.size(); x++)
		{
			goal = m_mbGoal[x];
			goal.target_pose.header.stamp = ros::Time::now();

			ROS_INFO("Sending goal %d", x + 1);
			m_mbClient.sendGoal(goal);
	
			m_mbClient.waitForResult();
	
			if (m_mbClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Move to goal %d success", x + 1);
			}
			else
			{
				ROS_INFO("Move to goal %d fail", x + 1);
				return 1;
			}
	
			ros::Duration(5).sleep();
		}
		
		return 0;
	}
	
private:
	ros::NodeHandle							m_hNode;
	MoveBaseClient							m_mbClient;
	vector<move_base_msgs::MoveBaseGoal>	m_mbGoal;

	void setupGoals()
	{
		bool bXFound = true, bYFound = true, bWFound = true;
		unsigned int iIndex = 0;
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		
		std::string strNodeName = ros::this_node::getName();

		while(bXFound || bYFound || bWFound)
		{
			bXFound = false;
			bYFound = false;
			bWFound = false;
			iIndex++;

			float fXTarget = 0.0f;
			bXFound = m_hNode.getParam(strNodeName + "/xTarget" + to_string(iIndex), fXTarget);
			float fYTarget = 0.0f;
			bYFound = m_hNode.getParam(strNodeName + "/yTarget" + to_string(iIndex), fYTarget);
			float fWTarget = 0.0f;
			bWFound = m_hNode.getParam(strNodeName + "/wTarget" + to_string(iIndex), fWTarget);

			if (bXFound || bYFound || bWFound)
			{
				ROS_INFO("index %d found", iIndex);
				goal.target_pose.pose.position.x = fXTarget;
				goal.target_pose.pose.position.y = fYTarget;
				goal.target_pose.pose.orientation.w = fWTarget;

				m_mbGoal.push_back(goal);
			}
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pick_objects");
	
	PickObjectNode obj;
	
	obj.startMove();

	return 0;
}
