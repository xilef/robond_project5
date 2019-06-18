#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>
#include "add_markers/MarkerData.h"

class AddMarkersService
{
public:
	AddMarkersService()
	{
		ROS_INFO_STREAM("Setup add_markers service class.");

		m_pubMarker = m_hNode.advertise<visualization_msgs::Marker>("visualization_marker", 1);

		m_servAdd = m_hNode.advertiseService("/add_markers/add_marker", &AddMarkersService::addMarkerHandler, this);
		m_servHide = m_hNode.advertiseService("/add_markers/hide_marker", &AddMarkersService::hideMarkerHandler, this);

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

		ROS_INFO("Ready to set marker!");
	}

private:
	bool addMarkerHandler(add_markers::MarkerData::Request& req, add_markers::MarkerData::Response& res)
	{
		m_marker.action = visualization_msgs::Marker::ADD;

		m_marker.pose.position = req.position;

		m_pubMarker.publish(m_marker);

		res.msg_feedback = "Marker set - x: " + std::to_string(req.position.x) + " y: " + std::to_string(req.position.y) + " z: " + std::to_string(req.position.z) + ".";

		ROS_INFO_STREAM(res.msg_feedback);

		return true;
	}

	bool hideMarkerHandler(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
	{
		m_marker.action = visualization_msgs::Marker::DELETE;

		m_pubMarker.publish(m_marker);

		ROS_INFO_STREAM("Marker removed.");

		return true;
	}

	ros::NodeHandle		m_hNode;
	ros::Publisher		m_pubMarker;
	ros::ServiceServer	m_servAdd;
	ros::ServiceServer	m_servHide;
	visualization_msgs::Marker m_marker;
};

int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_markers");

	AddMarkersService obj;

	ros::spin();

	return 0;
}
