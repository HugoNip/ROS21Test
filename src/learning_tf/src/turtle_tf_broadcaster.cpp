/**
 * This tutorial teaches you how to **broadcast coordinate frames to tf**. 
 * 
 * In this case, we want to broadcast the changing coordinate frames 
 * of the turtles, as they move around.
 */

#include <ros/ros.h>
/**
 * The tf package provides an implementation of a TransformBroadcaster to 
 * help make the task of publishing transforms easier. To use 
 * the TransformBroadcaster, we need to include the tf/transform_broadcaster.h 
 * header file.
 */
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg)
{
	/**
	 * step1: create a TransformBroadcaster object 
	 * that we'll use later to send the transformations over the wire.
	 */
	static tf::TransformBroadcaster br;

	/**
	 * step2: create a Transform object
	 * copy the information from the 2D turtle pose into the 3D transform.
	 */
	tf::Transform transform;
	// translation
	transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));

	// rotation.
	tf::Quaternion q;
	q.setRPY(0, 0, msg->theta);	
	transform.setRotation(q);

	/**
	 * step3: broadcast tf data between world and turtle
	 * 
	 * This is where the real work is done. 
	 * 
	 * Sending a transform with a TransformBroadcaster 
	 * requires four arguments.
	 * 1) First, we pass in the transform itself.
	 *    Now we need to give the transform being published a timestamp, 
	 *    we'll just stamp it with the current time, ros::Time::now().
	 * 2) Then, we need to pass the name of the parent frame of the link 
	 *    we're creating, in this case "world"
	 * 3) Finally, we need to pass the name of the child frame of the link 
	 *    we're creating, in this case this is the name of the turtle itself.
	 */
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char **argv)
{
	/**
	 * terminal:
	 * rosrun learning_tf turtle1_tf_broadcaster __name:=turtle_tf_broadcaster /turtle1
	 * rosrun learning_tf turtle2_tf_broadcaster __name:=turtle_tf_broadcaster /turtle2
	 */
	ros::init(argc, argv, "my_tf_broadcaster");

	if (argc != 2)
	{
		ROS_ERROR("need turtle name as argument");
		return -1;
	}

	turtle_name = argv[1]; // turtle1/turtle2

	// subscriber
	// subscribe the pose message
	// topic: turtle1/pose
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

	ros::spin();	

	return 0;
}