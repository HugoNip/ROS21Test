/**
 * ros/ros.h is a convenience include that includes all the headers necessary 
 * to use the most common public pieces of the ROS system.
 */
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 * 
 * Here's the condensed version of what's going on:
 * 
 * 1) Initialize the ROS system
 * 2) Advertise that we are going to be publishing "geometry_msgs::Twist" messages 
 * on the chatter topic to the master.
 * 3) Loop while publishing messages to chatter 10 times a second
 */
int main(int argc, char **argv) {

    /**
     * step 1: create a new node
     * 
     * "Node" is the ROS term for an executable 
     * that is connected to the ROS network. 
     * 
     * Here we'll create a publisher ("velocity_publisher") node 
     * which will continually broadcast a message.
     *
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it. 
     * 
     * The third argument to init() is the name of the node, the name is unique.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     * 
     * 
     * 
     * Initialize ROS. This allows ROS to do **name remapping** through the 
     * command line -- not important for now. 
     * 
     * This is also where we specify the **name of our node**. 
     * Node names must be **unique** in a running system.
     * 
     * The name used here must be a base name, ie. it cannot have a / in it.
     */
    ros::init(argc, argv, "velocity_publisher");

    /**
     * Create a handle to this process' node.
     * 
     * NodeHandle is the main access point to communications with the ROS system.
     * 
     * The first NodeHandle constructed will fully initialize this node, and 
     * the last NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /** 
     * step 2: register in ROS Master, including topic name and message type
     * 
     * create a publisher
     * geometry_msgs::Twist: message type
     * "turtle1/cmd_vel": topic name
     * 10: how many messages to buffer up before throwing some away
     *
     * The advertise() function is how you tell ROS that
     * you **want to publish** on a given topic name ("turtle1/cmd_vel").
     * 
     * This invokes a call to the ROS master node, which keeps
     * a registry of who is publishing and who is subscribing.
     * 
     * After this advertise() call is made, the master node will notify 
     * anyone who is trying to subscribe to this topic name ("turtle1/cmd_vel"), 
     * and they will in turn negotiate a peer-to-peer connection with this node.
     * 
     * advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  
     * Once all copies of the returned Publisher object are destroyed, 
     * the topic will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages. If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     * 
     * 
     * 
     * Tell the master that we are going to be publishing a message 
     * of type "geometry_msgs::Twist" on the topic chatter. 
     * 
     * This **lets the master tell any nodes** listening on chatter 
     * that we are going to publish data on that topic.
     * 
     * The second argument is the size of our publishing queue. 
     * In this case if we are publishing too quickly it will buffer up 
     * a maximum of 10 messages before beginning to throw away old ones.
     * 
     * NodeHandle::advertise() returns a ros::Publisher object, 
     * which serves two purposes: 
     * 1) it contains a **publish()** method that lets you publish messages 
     *    onto the topic it was created with, and 
     * 2) when it goes out of scope, it will automatically unadvertise.
     * 
     * The Twist is necessary because our topic '/turtle1/cmd_vel' 
     * uses the Twist message, you can check with the following command:
     * $ rostopic info /turtle1/cmd_vel
     * 
     * Type: geometry_msgs::Twist
     * Publisher: None
     * Subscribers: turtlesim
     */
    ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

    /**
     * A ros::Rate object allows you to specify a frequency that you would like to loop at. 
     * It will keep track of how long it has been since the last call to Rate::sleep(), 
     * and sleep for the correct amount of time.
     * 
     * In this case we tell it we want to run at 10Hz.
     */
    ros::Rate loop_rate(10);

    /**
     * A count of how many messages we have sent. 
     * This is used to create a unique string for each message.
     */
    int count = 0;

    /**
     * By default roscpp will install a SIGINT handler 
     * which provides Ctrl-C handling which will cause ros::ok() 
     * to return false if that happens.
     * 
     * ros::ok() will return false if:
     * 1) a SIGINT is received **(Ctrl-C)**
     * 2) we have been kicked off the network by **another node with the same name**
     * 3) **ros::shutdown()** has been called by another part of the application.
     * 4) all ros::NodeHandles have been **destroyed**
     * 
     * Once ros::ok() returns false, all ROS calls will fail.
     */    
    while (ros::ok()) {

        /**
         * step 3: create a new message object
         * 
         * This is a message object. You stuff it with data, and then publish it.
         * 
         * We broadcast a message on ROS using a message-adapted class, 
         * generally generated from a vel_msg file. 
         * More complicated datatypes are possible, 
         * but for now we're going to use the "geometry_msgs::Twist vel_msg" message, 
         * which has one member: "linear.x", "angular.z".
         */
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.5;
        vel_msg.angular.z = 0.2;

        /**
         * step 4: publish message
         * 
         * The publish() function is how you send messages.
         * The parameter is the message object.
         * 
         * The type of this object must agree with the type given as a template parameter 
         * to the advertise<>() call, as was done in the constructor above.
         * 
         * Now we actually broadcast the message to anyone who is connected.
         */
        turtle_vel_pub.publish(vel_msg);
        
        /**
         * ROS_INFO and friends are our replacement for printf/cout. 
         */
        ROS_INFO("Publish turtle velocity command[%0.2f m/s, %0.2f rad/s]",
                    vel_msg.linear.x, vel_msg.angular.z);

        /**
         * Calling ros::spinOnce() here is **not necessary** for this simple program, 
         * because we are **not** receiving any **callbacks**. 
         * 
         * However, if you were to add a subscription into this application, 
         * and did not have ros::spinOnce() here, 
         * your callbacks would never get called. 
         * So, add it for good measure.
         */
        ros::spinOnce();

        /**
         * Now we use the ros::Rate object to sleep for the time 
         * remaining to let us hit our 10Hz publish rate.
         */
        loop_rate.sleep();
    }

    return 0;
}