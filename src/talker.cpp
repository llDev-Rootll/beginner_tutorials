/**
 * Copyright (c) 2021 Arunava Basu
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <sstream>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/String.h"
#include "talker.h"
#include "beginner_tutorials/string.h"

// Default string
extern std::string new_string = "Default string";

/**
 * @brief A function that provides a service of changing the output string of the publisher
 * 
 * @param req service request
 * @param res service response
 * @return returns true once it changes the string
 */
bool changeOutput(beginner_tutorials::string::Request  &req,
         beginner_tutorials::string::Response &res) {
  new_string = req.new_string;

  ROS_DEBUG_STREAM("String was changed to : " << new_string.c_str());
  res.res_s = req.new_string;
  return true;
}

/**
 * A simple ros publisher and subscriber.
 */

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("pub_sub", 1000);

  // service created to change the output string
  ros::ServiceServer service =
  n.advertiseService("change_output", changeOutput);
  int rate = 10;

  rate = atoi(argv[1]);
  // Checks if the rate is greater than 100
  if ( rate >= 100 ) {
    ROS_WARN_STREAM("Too high a frequency, resetting to 10");
    rate = 10;
  }
  // Checks if the rate is equal to 0
  if ( rate == 0 ) {
    ROS_FATAL("Frequency cannot be 0, resetting to 10");
    rate = 10;
  }
  // Checks if the rate is negative
  if ( rate < 0 ) {
    ROS_ERROR("Frequency cannot be negative, resetting to 10");
    rate = 10;
  }

  // Set the frequency rate
  ROS_DEBUG_STREAM("Frequency set to : " << rate);
  ros::Rate loop_rate(rate);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  tf::TransformBroadcaster br;

  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
    std::stringstream ss;
    ss << new_string;
    msg.data = ss.str();

    /**
     * @brief broadcast a static transformation with nonzero roational 
     * and translational components called /talk with parent /world
     * 
     */
    br.sendTransform(tf::StampedTransform(tf::Transform(
      tf::Quaternion(0, 0, 0, 1),
     tf::Vector3(0.1, 0.0, 0.2)), ros::Time::now(), "world", "talk"));
    // ROS_INFO_STREAM(msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
