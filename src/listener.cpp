#include <cstdlib> // EXIT_SUCCESS
#include <string>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "util.hpp"


class Listener 
{
public:
//constructor(s)
    Listener()
    : count_listens(0) { }

public:
// method(s)
    void on_callback(const std_msgs::Float64MultiArray::ConstPtr & msg)
    {
        count_listens++;
            // ROS's version of DEBUG_MSG
    ROS_INFO(
      "L: MSG #%d: array[0] = %f"
        , count_listens
        , msg->data[0]);
    }

private:
//member
    int count_listens;
};

int main(int argc, char **argv)
{
    // name must be unique
    const std::string my_name = "listener_template";
    // always needed to do before using anything ros related (can be used to define cmd line options)
    ros::init(argc, argv, my_name);
    // when we start this program, then it's a ROS node. It can have multiple handles
    ros::NodeHandle n_handle;
    // initialize a listener object which has state
    Listener stateful_listener;
    // subscribe to globally defined channel, with the pointer to the state and the function
    ros::Subscriber sub = n_handle.subscribe(util::forwarding_channel
                                           , util::buffer
                                           , &Listener::on_callback
                                           , &stateful_listener);
    // this checks if any callbacks want to be triggered, equivalent to while(ros::ok()) { ros::spinOnce() }
    ros::spin();

    return EXIT_SUCCESS;
}