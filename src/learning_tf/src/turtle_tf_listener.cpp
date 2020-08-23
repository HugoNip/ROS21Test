#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "my_tf_listener");

	ros::NodeHandle node;

	// server
	// request for spawn turtle2
	ros::service::waitForService("/spawn");
	ros::ServiceClient add_turtle = 
		node.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn srv;
	add_turtle.call(srv); // need position data

	// create a publisher to make turtle2 move
	ros::Publisher turtle_vel =
		node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
	
	// create tf listener
	tf::TransformListener listener;

	ros::Rate rate(10.0);
	while (node.ok()) {
		// get data about relationship of coordinate between turtle1 and turtle2
		tf::StampedTransform transform;

		try
		{
			listener.waitForTransform("turtle2", "turtle1", ros::Time(0),
				ros::Duration(3.0));
			listener.lookupTransform("turtle2", "turtle1", ros::Time(0),
				transform);
		}
		catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		// publish order base on this relationship
		geometry_msgs::Twist vel_msg;
		vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
										transform.getOrigin().x());
		vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
									  pow(transform.getOrigin().y(), 2));
		turtle_vel.publish(vel_msg);

		rate.sleep();		
	}
	
	return 0;
}