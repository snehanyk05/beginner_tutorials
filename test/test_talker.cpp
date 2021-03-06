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
 *@file       test_talker.cpp
 *@author     Sneha Nayak
 *@copyright  MIT License
 *@brief      Talker Node test
 */
#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <beginner_tutorials/changeStringName.h>




/**
 * @brief      Tests whether the service exists and then changes string.
 * @param      testNode         gtest framework type
 * @param      testResponse        name of the test
 */
TEST(testNode, testResponse) {
  // Create ros node handle.
  ros::NodeHandle node;

  // Create service client and check existence.
  ros::ServiceClient client = node.serviceClient
<beginner_tutorials::changeStringName>("change_string");
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));

  beginner_tutorials::changeStringName srv;
    srv.request.change = "Test";
    client.call(srv);
    EXPECT_EQ("Test", srv.response.response);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "testTalker");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
