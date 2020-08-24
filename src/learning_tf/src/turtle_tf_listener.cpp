/** 
 * Description: This tutorial teaches you how to 
 * use tf to get access to frame transformations.
 */

#include <ros/ros.h>
/**
 * The tf package provides an implementation of a TransformListener 
 * to help make the task of **receiving transforms** easier.
 * 
 * To use the TransformListener, we need to include the 
 * tf/transform_listener.h header file.
 */
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char **argv) {

	ros::init(argc, argv, "my_tf_listener");

	ros::NodeHandle node;

	// server
	// request for spawn turtle2
	ros::service::waitForService("/spawn");
	ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn srv;
	add_turtle.call(srv); // need position data

	// create a publisher to make turtle2 move
	ros::Publisher turtle_vel =	node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
	
	/**
	 * Here, we create a TransformListener object.
	 * Once the listener is created, it starts receiving tf transformations 
	 * over the wire, and buffers them for up to 10 seconds.
	 * 
	 * The TransformListener object should be scoped to persist otherwise 
	 * it's cache will be unable to fill and almost every query will fail. 
	 * 
	 * A common method is to make the TransformListener object 
	 * a member variable of a class.
	 */
	tf::TransformListener listener;

	ros::Rate rate(10.0);
	while (node.ok()) {
		// get data about relationship of coordinate between turtle1 and turtle2
		tf::StampedTransform transform;

		try
		{	
			/**
			 * tf provides a nice tool that will wait until a transform becomes available.
			 * 
			 * The waitForTransform() takes four arguments:
			 * 
			 * 1) Wait for the **transform** from this frame...
			 * 2) ... to this frame,
			 * 3) at this time, and
			 * 4) timeout: don't wait for longer than this maximum duration
			 * 
			 * Note: The use of ros::Time::now() is for this example. 
			 * Usually this would be the timestamp of the data wishing to be transformed.
			 * 
			 * So waitForTransform() will actually block until the transform between 
			 * the two turtles becomes available (this will usually take a few milliseconds), 
			 * OR --if the transform does not become available-- until the timeout has been reached.
			 */
			listener.waitForTransform("turtle2", "turtle1", ros::Time(0), ros::Duration(3.0));

			/**
			 * The real work is done, we query the listener for a specific
			 * transformation. Let's take a look at the four arguments:
			 * 
			 * We want the transform from frame /turtle1 to frame /turtle2.
			 * 
			 * The time at which we want to transform.
			 * 
			 * Providing ros::Time(0) will just get us the latest available transform.
			 * 
			 * The object (transform) in which we store the resulting transform.
			 * 
			 * All this is wrapped in a try-catch block to catch possible exceptions.
			 * 
			 * 
			 * how to get a transform at a specific time
			 * 
			 * You can also see we specified a time equal to 0. 
			 * For tf, time 0 means "the latest available" transform in the buffer.
			 */
			listener.lookupTransform("turtle2", "turtle1", ros::Time(0), transform);
		}
		catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		// ======================= have got transoform ===========================
		
		/**
		 * publish order base on this relationship
		 * Here, the transform is used to calculate new linear and angular velocities 
		 * for turtle2, based on its distance and angle from turtle1. 
		 * 
		 * The new velocity is published in the topic "turtle2/cmd_vel" 
		 * and the sim will use that to update turtle2's movement.
		 */
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