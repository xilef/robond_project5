#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class AddMarkersService
{
public:
	AddMarkersService()
	{
		m_pubMarker = m_hNode.advertise<visualization_msgs::Marker>("visualization_marker", 1);

		// setup basic marker details
		// only need to change the pose
		m_marker.header.frame_id = "map";

		m_marker.ns = "goal";
		m_marker.id = 0;

    	m_marker.type = visualization_msgs::Marker::SPHERE;

		m_marker.pose.orientation.x = 0.0;
		m_marker.pose.orientation.y = 0.0;
		m_marker.pose.orientation.z = 0.0;
		m_marker.pose.orientation.w = 1.0;

		m_marker.scale.x = 0.5;
		m_marker.scale.y = 0.5;
		m_marker.scale.z = 0.5;

		m_marker.color.r = 0.988f;
		m_marker.color.g = 0.552f;
		m_marker.color.b = 0.113f;
		m_marker.color.a = 1.0;

		m_marker.lifetime = ros::Duration();

		while (m_pubMarker.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				break;
			}
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(1);
		}

		ROS_INFO("Ready to set marker!");
	}
	
	void addMarker(geometry_msgs::Point position)
	{
		m_marker.action = visualization_msgs::Marker::ADD;

		m_marker.pose.position = position;

		m_pubMarker.publish(m_marker);

		ROS_INFO_STREAM("Marker set - x: " + std::to_string(position.x) + " y: " + std::to_string(position.y) + " z: " + std::to_string(position.z) + ".");
	}
	
	void hideMarker()
	{
		m_marker.action = visualization_msgs::Marker::DELETE;

		m_pubMarker.publish(m_marker);

		ROS_INFO_STREAM("Marker removed.");
	}

private:
	ros::NodeHandle				m_hNode;
	ros::Publisher				m_pubMarker;
	visualization_msgs::Marker	m_marker;
};

int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_markers_orig");
	ros::NodeHandle hNode;
	std::string strNodeName = ros::this_node::getName();
	
	geometry_msgs::Point ptStart, ptEnd;

	float fXTarget = 0.0f;
	float fYTarget = 0.0f;

	hNode.getParam(strNodeName + "/startGoalX", fXTarget);
	hNode.getParam(strNodeName + "/startGoalY", fYTarget);

	ptStart.x = fXTarget;
	ptStart.y = fYTarget;

	fXTarget = 0.0f;
	fYTarget = 0.0f;
	hNode.getParam(strNodeName + "/endGoalX", fXTarget);
	hNode.getParam(strNodeName + "/endGoalY", fYTarget);

	ptEnd.x = fXTarget;
	ptEnd.y = fYTarget;

	AddMarkersService obj;

	obj.addMarker(ptStart);
	ros::Duration(5).sleep();
	obj.hideMarker();

	ros::Duration(5).sleep();

	obj.addMarker(ptEnd);
	ros::Duration(5).sleep();
	obj.hideMarker();

	return 0;
}
