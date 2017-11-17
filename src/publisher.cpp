#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "util.hpp"

#include <cstdlib>
#include <sstream>
#include <vector>
#include <algorithm>

class Publisher
{

public:
// constructors
    Publisher(const std::string & publisher_name
            , const int buffer
            , const int update_rate_hz)
      : pub(n_handle.advertise<std_msgs::Float64MultiArray>(publisher_name, buffer)) // init publisher to publish Float64MultiArray with size 'buffer'
      , loop_rate(update_rate_hz)
      , count_published(0) { }                                               // sleep at least this many ms after call of 'publish()' 

public:
// methods
    /*
     * If this method is called we will publish the 'array' to the network, with a maximum rate of 'loop_rate'
     */
    void publish(const std_msgs::Float64MultiArray & msg)
    { 
        ++count_published;
                // ROS's version of DEBUG_MSG
        ROS_INFO(
          "P: MSG #%d: array[0] = %f"
            , count_published
            , msg.data[0]);
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

private:
// member
    ros::NodeHandle n_handle;
    ros::Publisher  pub;
    ros::Rate loop_rate;
    int count_published;
};

int main(int argc, char **argv)
{
    // name must be unique
    const std::string my_name = "publisher_template";
    // always needed to do before using anything ros related (can be used to define cmd line options) 
    ros::init(argc, argv, my_name);
    // initialize custom publisher    
    Publisher publisher(util::data_source_channel, util::buffer, util::update_rate_hz);
    // counter to add to the array based on amount of publish() calls 
    double count = 0;
    std::vector<double> seq_array { 1.0, 2.0, 3.0, 4.0, 5.0 };

    while (ros::ok())
    {
        std::transform(seq_array.begin(), seq_array.end(), seq_array.begin()
                    , [&](double f) -> double { return f + count; });

        const std_msgs::Float64MultiArray ros_float_array= util::create_float_64_array(seq_array);
        publisher.publish(ros_float_array);
        ++count;
    } 

    return EXIT_SUCCESS;
}