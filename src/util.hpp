#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <vector>

#if DEBUG_MODE
#define DEBUG_MSG(msg) fprintf(stderr, \
                               "[INFO] (%64s:%4d:%16s) %s\n", \
                               __FILE__, \
                               __LINE__, \
                               __func__, \
                               msg)
#else
#define DEBUG_MSG(msg)
#endif // DEBUG_MODE (in [package-name]/CMakeLists.txt)


/*
 * Example method which is used by the tests
 */
int foo()
{   
    DEBUG_MSG("foo() was called from here!");
    return 1;
}


namespace util {

    const std::string data_source_channel = "data_source_channel";
    const std::string forwarding_channel  = "forwarding_channel";

    // Size of the message que. If messages are arriving faster than they are being processed, this
    // is the number if messages that will be buffered up before beginning to throw away the oldest ones.
    const int buffer          = 1000;

    // maximum publish frequency (can be slower, if publish is not triggered fast enough)
    const int update_rate_hz  = 1000;

    
    /*
     * convertion from vector<T> to ros-equivalent container, label can be see via `rosdump`
     * 
     * Invariant: T should be castable to double
     *
     * THIS IS ONLY A QUICK IMPLEMENTATION. One should implement a custom message. To quote the wiki
     * 
     *     "can be useful for quick prototyping, they are NOT intended for "long-term" usage"
     * 
     */
    template<typename T> 
    std_msgs::Float64MultiArray create_float_64_array(std::vector<T> & vec, std::string label)
    {
        std_msgs::Float64MultiArray msg;

        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].size   = vec.size();
        msg.layout.dim[0].stride = 1;
        msg.layout.dim[0].label  = label;

        std::transform(vec.begin(), vec.end(), std::back_inserter(msg.data)
                    , [&](T d) { return static_cast<double>(d); });
        return msg;
    }

    /*
     * convertion from vector<double> to ros-equivalent container
     */
    std_msgs::Float64MultiArray create_float_64_array(std::vector<double> & vec)
    {
        return create_float_64_array(vec, "util::create_float_64_array (default label)");
    }


}