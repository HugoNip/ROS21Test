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
 * 
 * -------------------------------------------------------------------------
 * Callback Signature
 * 
 * The signature of the service callback is:
 * 
 * bool callback(MReq& request, MRes& response);
 * 
 * where MReq and MRes match the request/response types provided to advertiseService(). 
 * 
 * A return value of true means the service succeeded, and 
 * the response object has been filled with the necessary data. 
 * 
 * A return value of false means the call has failed and 
 * the response object will not be sent to the caller.
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
    // server node name
    ros::init(argc, argv, "tutle_command_server");

    ros::NodeHandle n;

    /**
     * Here the service is created and advertised over ROS.
     * 
     * service name: "/turtle_command"
     * 
     * command: rosrun call /turtle_command "{}"
     * 
     * 
     * In roscpp you provide a service by creating a ros::ServiceServer through the 
     * ros::NodeHandle::advertiseService() method. 
     * 
     * advertiseService() works very similar to how the subscribe() method works, 
     * in that you provide a service name and a callback to be invoked when the service is called.
     * 
     * 
     * 
     * 
     * 3.1 Options
     * There are a number of different versions of advertiseService(), 
     * for different types of callbacks, but the general signature is:
     * 
     * template<class MReq, class MRes>
     * ros::ServiceServer nh.advertiseService(const std::string& service, <callback>);
     * 
     * MReq [usually unnecessary]
     * This is a template argument specifying the request message type. 
     * For most versions you do not need to explicitly define this, 
     * as the compiler can deduce it from the callback function.
     * 
     * MRes [usually unnecessary]
     * This is a template argument specifying the response message type. 
     * For most versions you do not need to explicitly define this, 
     * as the compiler can deduce it from the callback function.
     * 
     * service
     * The name of the service to provide
     * 
     * <callback>
     * The callback to invoke when a request has arrived
     */
    ros::ServiceServer command_service = n.advertiseService("/turtle_command", commandCallback);


    // the service is get the response of "pubCommand": true/false.
    // ===============================================================
    // then using this value to publish/stop publishing the message...


    // create a publisher to make the turtle move
    // control velocity
    // command velocity
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