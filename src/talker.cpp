 /**
 *  MIT License
 *
 *  Copyright (c) 2020 Sneha Nayak
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */
/**
 * Copyright 2020
 * Copyright owner: Sneha Nayak
 * [legal/copyright]
 */
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/changeStringName.h"
#include "tf/transform_broadcaster.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

struct str_msg {
  std::string message;
};
str_msg MessageObj;

/**
 * @brief service function to handle change of message string in chatter
 * @return bool
 */
bool change_string_func(beginner_tutorials::changeStringName::Request  &req,
beginner_tutorials::changeStringName::Response &res) {
  MessageObj.message = req.change;
  ROS_INFO("request: New string =%s", req.change.c_str());
  res.response = req.change;
  // ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

/**
 * @brief  Tf Broadcast
 * @param  none
 * @return none
 */
void poseCallback() {
    static tf::TransformBroadcaster bc;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(10.0, 20.0, 30.0));  // assign static value
    tf::Quaternion q;
    q.setRPY(1, 1, 0);
    transform.setRotation(q);
    bc.sendTransform(tf::StampedTransform(transform,
    ros::Time::now(), "world", "talk"));
    // broadcast /talk frame with parent /world
}
/**
 * @brief main function to publish custom messages to chatter
 * @return execution status
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");
  // Use Logger Level INFO
    ROS_INFO_STREAM("Started node talker.");

    ros::NodeHandle node("~");
    MessageObj.message = "Hi, this is the Talker Node.";
    std::string param;
    node.getParam("param", param);
    ROS_INFO_STREAM("Got param: " << param << ".\n");
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
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);
  ros::ServiceServer change_string =
  n.advertiseService("change_string", change_string_func);
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    poseCallback();

  // Changing msg.data type depending on the parameter set through command line
  // or can be changes using ros service call to change_string
    if (param == "error" || MessageObj.message =="error") {
    ss << "ERROR";
    ROS_ERROR_STREAM("Sending ERROR Logger Level!");
  } else if (param == "warn" || MessageObj.message =="warn") {
    ss << "WARN";
    ROS_WARN_STREAM("Sending WARN Logger Level!");
  } else if (param == "fatal" || MessageObj.message =="fatal") {
    ss << "FATAL";
    ROS_FATAL_STREAM("Sending FATAL Logger Level!");
  } else if (param == "debug" || MessageObj.message =="debug") {
    ss << "DEBUG";
    ROS_DEBUG_STREAM("Sending DEBUG Logger Level!");
  } else if (param == "info" || MessageObj.message =="info") {
    ss << "INFO";
    ROS_INFO_STREAM("Sending INFO Logger Level!");
    // ss << talker_string << count;
    // msg.data = ss.str();

    // ROS_INFO("%s", msg.data.c_str());

  } else {
    ss << MessageObj.message << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
  }
  msg.data = ss.str();
    // ss << talker_string << count;
    // msg.data = talker_string+" "+std::to_string(count);

    // ROS_INFO_STREAM(msg.data);

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
