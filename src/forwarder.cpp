#include <cstdlib> // EXIT_SUCCESS
#include <string>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "util.hpp"

/*
 * Abstracted publisher interface because the official ROS documentation is proposing horrible
 * C++ code which does not encapsulate anything, uses 'new' and raw pointers. We're in 2017 for gods sake
 */
class Forwarder {

public:
// constructor(s)
    Forwarder(const std::string & publisher_name
           , const int buffer
           , const int update_rate_hz)
      : pub(n_handle.advertise<std_msgs::Float64MultiArray>(publisher_name, buffer))
      , loop_rate(update_rate_hz)
      , count_forward(0) { }

public:
// methods
    /*
     * This method will be called if our own subscriber gives us a new Float64MultiArray message
     * On this callback we will re-publish the array with a maximul loop rate defined in the constructor
     */
    void on_callback(const std_msgs::Float64MultiArray::Ptr & msg)
    {
        // logging message count
        ++count_forward;
        // ROS's version of DEBUG_MSG
        ROS_INFO(
          "F: MSG #%d: array[0] = %f"
            , count_forward
            , msg->data[0]);
        // publish the array and sleep for the predefined time
        const std_msgs::Float64MultiArray::Ptr changed_msg = update_message(msg);
        pub.publish(changed_msg);
        loop_rate.sleep();
    }

    /*
     * Transform incoming raw float64 array
     */ 
    const std_msgs::Float64MultiArray::Ptr update_message(const std_msgs::Float64MultiArray::Ptr & msg)
    {
        for (uint i = 0; i < msg -> layout.dim[0].size; ++i)
        {
            msg -> data[i] *= 2; 
        }

        return msg;
    }


private:
// member
    ros::NodeHandle n_handle;
    ros::Publisher pub;
    ros::Rate loop_rate;
    int count_forward;

};


int main(int argc, char ** argv)
{
    // name must be unique
    const std::string my_name = "forwarder_template";
    // always needed to do before using anything ros related (can be used to define cmd line options)
    ros::init(argc, argv, my_name);
    // when we start this program, then it's a ROS node. It can have multiple handles
    ros::NodeHandle n_handle;
    // create our forwarding object with state
    Forwarder forwarder(util::forwarding_channel, util::buffer, util::update_rate_hz);
    //subscriber-call should only happen once!
    ros::Subscriber sub = n_handle.subscribe(util::data_source_channel
                                           , util::buffer
                                           , &Forwarder::on_callback
                                           , &forwarder);
    // this checks if any callbacks want to be triggered, equivalent to while(ros::ok()) { ros::spinOnce() }
    ros::spin();

    return EXIT_SUCCESS;
}