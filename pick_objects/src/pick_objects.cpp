#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>
#include "add_markers/MarkerData.h"
#include <vector>
#include <string>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PickObjectNode
{
public:
	PickObjectNode() : m_mbClient("move_base", true)
	{
		m_clientAdd = m_hNode.serviceClient<add_markers::MarkerData>("/add_markers/add_marker");
		m_clientHide = m_hNode.serviceClient<std_srvs::Empty>("/add_markers/hide_marker");

		while(!m_mbClient.waitForServer(ros::Duration(5.0)))
		{
			ROS_INFO("Waiting for the move_base action server to come up");
		}
		
		setupGoals();
	}
	
	/*int startMove()
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
	}*/

	int startMove()
	{
		// Move to the start goal first
		m_mbStartGoal.target_pose.header.stamp = ros::Time::now();
		ROS_INFO("Sending start goal");
		m_mbClient.sendGoal(m_mbStartGoal);

		m_mbClient.waitForResult();

		if (m_mbClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Start goal reached! Picking up something here.");
			std_srvs::Empty req;
			if (!m_clientHide.call(req))
			{
				ROS_ERROR("Failed to call service!");
			}
		}
		else
		{
			ROS_INFO("Move to goal start goal fail");
			return 1;
		}

		ros::Duration(5).sleep();

		m_mbEndGoal.target_pose.header.stamp = ros::Time::now();
		ROS_INFO("Pick up done. Sending end goal");
		m_mbClient.sendGoal(m_mbEndGoal);

		m_mbClient.waitForResult();

		if (m_mbClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("End goal reached! Dropping off here.");
			add_markers::MarkerData req;
			req.request.position.x = m_mbEndGoal.target_pose.pose.position.x;
			req.request.position.y = m_mbEndGoal.target_pose.pose.position.y;
			if (!m_clientAdd.call(req))
			{
				ROS_ERROR("Failed to call service!");
			}
		}
		else
		{
			ROS_INFO("Move to end goal fail");
			return 1;
		}

		return 0;
	}
	
private:

	/*void setupGoals()
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
	}*/
	
	void setupGoals()
	{
		std::string strNodeName = ros::this_node::getName();
		m_mbStartGoal.target_pose.header.frame_id = "map";
		m_mbEndGoal.target_pose.header.frame_id = "map";

		float fXTarget = 0.0f;
		float fYTarget = 0.0f;
		float fWTarget = 0.0f;

		m_hNode.getParam(strNodeName + "/startGoalX", fXTarget);
		m_hNode.getParam(strNodeName + "/startGoalY", fYTarget);
		m_hNode.getParam(strNodeName + "/startGoalW", fWTarget);

		m_mbStartGoal.target_pose.pose.position.x = fXTarget;
		m_mbStartGoal.target_pose.pose.position.y = fYTarget;
		m_mbStartGoal.target_pose.pose.orientation.w = fWTarget;
		
		// show the marker for the start at startup
		add_markers::MarkerData req;
		req.request.position.x = fXTarget;
		req.request.position.y = fYTarget;
		if (!m_clientAdd.call(req))
		{
			ROS_ERROR("Failed to call service!");
		}

		fXTarget = 0.0f;
		fYTarget = 0.0f;
		fWTarget = 0.0f;
		m_hNode.getParam(strNodeName + "/endGoalX", fXTarget);
		m_hNode.getParam(strNodeName + "/endGoalY", fYTarget);
		m_hNode.getParam(strNodeName + "/endGoalW", fWTarget);

		m_mbEndGoal.target_pose.pose.position.x = fXTarget;
		m_mbEndGoal.target_pose.pose.position.y = fYTarget;
		m_mbEndGoal.target_pose.pose.orientation.w = fWTarget;
	}

	ros::NodeHandle							m_hNode;
	MoveBaseClient							m_mbClient;
	//vector<move_base_msgs::MoveBaseGoal>	m_mbGoal;
	move_base_msgs::MoveBaseGoal			m_mbStartGoal;
	move_base_msgs::MoveBaseGoal			m_mbEndGoal;
	ros::ServiceClient						m_clientAdd;
	ros::ServiceClient						m_clientHide;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pick_objects");
	
	PickObjectNode obj;
	
	obj.startMove();

	return 0;
}
