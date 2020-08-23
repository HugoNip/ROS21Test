/**
 * service name: /turtle_command
 * service type: std_srvs/Tirgger
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>

ros::Publisher turtle_vel_pub;
bool pubCommand = false;

/**
 * Here we'll create the service ("tutle_command_server") node
 * which will receive "req"and return "res".
 */

/**
 * This function provides the service, 
 * it takes in the request and response type defined in the srv file and returns a boolean.
 */
bool commandCallback(std_srvs::Trigger::Request &req,
                     std_srvs::Trigger::Response &res) {
    pubCommand = !pubCommand;

    ROS_INFO("Publish turtle velocity command [%s]", pubCommand==true?"Yes":"no");

    /**
     * response
     * 
     * set feed back data
     * rossrv show std_srvs/Trigger
     * service type: std_srvs/Tirgger 
     */
     res.success = true;
     res.message = "Change turtle command state!";

    // Finally the service returns true when it is complete.
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tutle_command_server");

    ros::NodeHandle n;

    /**
     * Here the service is created and advertised over ROS.
     * 
     * command: rosrun call /turtle_command "{}"
     */
    ros::ServiceServer command_service = n.advertiseService("/turtle_command", commandCallback);

    // create a publisher to make the turtle move
    turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    // 
    ROS_INFO("Ready to receive turtle command.");

    ros::Rate loop_rate(10);

    // get ther data of "pubCommand" from server
    while(ros::ok())
    {
        // check callback function queue once
        ros::spinOnce();

        // return response from callbackfunction
        if(pubCommand) // change to true
        {
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = 0.5;
            vel_msg.angular.z = 0.2;
            turtle_vel_pub.publish(vel_msg);
        }

        loop_rate.sleep();
    }

    return 0;


}