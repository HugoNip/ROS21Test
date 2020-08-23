#include <ros/ros.h>
#include <turtlesim/Pose.h>

/**
 * here's a condensed version of what's going on:
 * 1) Initialize the ROS system
 * 2) Subscribe to the chatter topic
 * 3) Spin, waiting for messages to arrive
 * 4) When a message arrives, the poseCallback() function is called
 */

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    /**
     * This is the callback function that will get called 
     * when a new message has arrived on the "pose_subscriber" topic. 
     * 
     * The message is passed in a boost shared_ptr, which means 
     * you can store it off if you want, without worrying about 
     * it getting deleted underneath you, and without copying the underlying data.
     */
    ROS_INFO("Turtle pose: x:%0.6f, y:%0.6f", msg->x, msg->y);
}

int main(int argc, char ** argv)
{
    // step 1: initialize ros node
    /**
     * 
     */
    ros::init(argc, argv, "pose_subscriber");

    // 
    ros::NodeHandle n;

    // subscribe topic, wait for topic message, call backfunction, process message
    // create a subscriber
    // /turtle1/pose: topic name
    // 10: queue length
    // poseCallback: if there is a message, call the callback function

    /**
     * The subscribe() call is how you **tell ROS** that 
     * you **want to receive messages** on a given topic ("/turtle1/pose").
     * 
     * This invokes a call to the ROS master node, which keeps 
     * a registry of who is publishing and who is subscribing.  
     * 
     * Messages are passed to a callback function, here called poseCallback.  
     * 
     * subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe. When all copies of the Subscriber
     * object go out of scope, this callback will automatically be unsubscribed from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the message
     * queue. If messages are arriving faster than they are being processed, this
     * is the number of messages that will be buffered up before beginning to throw
     * away the oldest ones.
     * 
     * 
     * 
     * Subscribe to the "pose_subscriber" topic with the master. 
     * ROS will call the poseCallback() function whenever a new message arrives. 
     * 
     * There are versions of the NodeHandle::subscribe() function 
     * which allow you to specify a class member function, or even 
     * anything callable by a Boost.Function object. 
     */
    ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 10, poseCallback);

    /**
     * ros::spin() enters a loop, calling message callbacks as fast as possible.
     * 
     * Don't worry though, if there's nothing for it to do it won't use much CPU. 
     * 
     * ros::spin() will exit once ros::ok() returns false, 
     * which means ros::shutdown() has been called, 
     * either by the default Ctrl-C handler, 
     * the master telling us to shutdown, or it being called manually.
     */
    ros::spin();

    return 0;
}