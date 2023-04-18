#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <actionlib_msgs/GoalStatusArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <actionlib_msgs/GoalID.h>
#include <std_msgs/Empty.h>
#include "exercise2/PlaySound.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;
using namespace cv;

Mat cv_map;
float map_resolution = 0;
geometry_msgs::TransformStamped map_transform;
ros::Time goal_set_time = ros::Time(0.0);
int last_status = -1;
bool allowedNewGoal;
ros::Publisher goal_pub;
ros::ServiceClient sound_client;
ros::Subscriber map_sub;
ros::Subscriber face_marker_sub;
ros::Publisher cancel_pub;
ros::Publisher cmd_vel_pub;
float rate_freq = 0.5;
int x = 0;
int y = 0;
int z = 0;
int i = 0;
bool approaching_face = false;
int stevilo_obrazov = 0;


//x: 243, y: 304 3.
//x: 268, y: 306 4
//x: 309, y: 301
//x: 304, y: 285
//x: 268, y: 275
//x: 272, y: 260
//x: 293, y: 259
//x: 294, y: 232
//x: 267, y: 245
//x: 246, y: 228
//x: 216, y: 220
//x: 233, y: 242
//x: 244, y: 264
//x: 214, y: 275
//x: 240, y: 281

/*
251, y: 299
269, y: 305
287, y: 299
297, y: 287
281, y: 259
267, y: 276
266, y: 248
285, y: 237
248, y: 236
215, y: 227
240, y: 273
218, y: 278
*/

int num_goals = 12;

int points[12][2] = {
    {251, 299},
    {269, 305},
    {287, 299},
    {297, 287},
    {281, 259},
    {267, 276},
    {266, 248},
    {285, 237},
    {248, 236},
    {215, 227},
    {240, 273},
    {218, 278},
};

geometry_msgs::Pose create_pose(double x, double y, double z)
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = 1.0;
    return pose;
}
void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg_map)
{
    int size_x = msg_map->info.width;
    int size_y = msg_map->info.height;

    if ((size_x < 3) || (size_y < 3))
    {
        ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
        return;
    }

    // resize cv image if it doesn't have the same dimensions as the map
    if ((cv_map.rows != size_y) && (cv_map.cols != size_x))
    {
        cv_map = cv::Mat(size_y, size_x, CV_8U);
    }

    map_resolution = msg_map->info.resolution;
    map_transform.transform.translation.x = msg_map->info.origin.position.x;
    map_transform.transform.translation.y = msg_map->info.origin.position.y;
    map_transform.transform.translation.z = msg_map->info.origin.position.z;

    map_transform.transform.rotation = msg_map->info.origin.orientation;

    // tf2::poseMsgToTF(msg_map->info.origin, map_transform);

    const std::vector<int8_t> &map_msg_data(msg_map->data);

    unsigned char *cv_map_data = (unsigned char *)cv_map.data;

    // We have to flip around the y axis, y for image starts at the top and y for map at the bottom
    int size_y_rev = size_y - 1;

    // reconstruct map matrix --> from vector to matrix + flip y
    for (int y = size_y_rev; y >= 0; --y)
    {

        int idx_map_y = size_x * (size_y - y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x)
        {

            int idx = idx_img_y + x;

            switch (map_msg_data[idx_map_y + x])
            {
            case -1:
                cv_map_data[idx] = 127;
                break;

            case 0:
                cv_map_data[idx] = 255;
                break;

            case 100:
                cv_map_data[idx] = 0;
                break;
            }
        }
    }
}

// ROTATE THE ROBOT
void rotate(float direction)
{
    geometry_msgs::Twist twist;
    ros::Rate rateR(2);
    const double angular_speed = 0.5;
    twist.angular.z = angular_speed * direction;

    const double time_for_circle = 2 * M_PI / (angular_speed * direction);
    const ros::Time start_time = ros::Time::now();
    while (ros::Time::now() < start_time + ros::Duration(time_for_circle))
    {
        cmd_vel_pub.publish(twist);
        // ROS_INFO("ROROROORO...");
        rateR.sleep();
    }
    twist.angular.z = 0.0;
    cmd_vel_pub.publish(twist);
    last_status = -1;
}

void nextGoal()
{

    int y = points[i % num_goals][1];
    int x = points[i % num_goals][0];
    // nextGoal(nextX, nextY);

    int v = (int)cv_map.at<unsigned char>(y, x);

    // check if point is reachable
    if (v != 255)
    {
        ROS_WARN("Unable to move to (x: %d, y: %d), not reachable", x, y);
        return;
    }

    geometry_msgs::Point pt;
    geometry_msgs::Point transformed_pt;
    
    if (!approaching_face)
    {
        // ROTATE
        ROS_INFO("Rotating the robot.");
        rotate(1.0);
        // rotate(-1.0);
        ROS_INFO("Finished rotating.");
    } 
    else  
    {
        approaching_face = false;
        ROS_INFO("Not rotating.");
    }

    

    goal_set_time = ros::Time::now();

    ROS_INFO("Goal %d sent to robot!", i);

    // convert point to image coordinate system
    pt.x = (float)x * map_resolution;
    pt.y = (float)(cv_map.rows - y) * map_resolution; // cv_map.rows - y --> because y is flipped
    pt.z = 0.0;

    // transform C.S. because origin is not at 0,0
    tf2::doTransform(pt, transformed_pt, map_transform);

    geometry_msgs::PoseStamped goal;

    goal.header.frame_id = "map";
    goal.pose.orientation.w = 1;
    goal.pose.position.x = transformed_pt.x;
    goal.pose.position.y = transformed_pt.y;
    goal.header.stamp = ros::Time::now();

    ROS_INFO("Moving to (x: %f, y: %f)", transformed_pt.x, transformed_pt.y);
    i++;
    goal_pub.publish(goal);
}

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    // Extract the latest position
    geometry_msgs::Point position = msg->pose.pose.position;

    // Print the position to the console
    ROS_INFO("Latest position: x=%f, y=%f, z=%f", position.x, position.y, position.z);
    x = position.x;
    y = position.y;
    z = position.z;
}

geometry_msgs::Point calculateMidpoint(geometry_msgs::Pose robot_pose, geometry_msgs::Pose face_pose)
{
    double dist = sqrt(pow(robot_pose.position.x - face_pose.position.x, 2) + pow(robot_pose.position.y - face_pose.position.y, 2));

    geometry_msgs::Point midpoint;
    midpoint.x = (robot_pose.position.x + face_pose.position.x) / 2;
    midpoint.y = (robot_pose.position.y + face_pose.position.y) / 2;
    midpoint.z = (robot_pose.position.z + face_pose.position.z) / 2;

    double factor = 0.5;
    midpoint.x += factor * (face_pose.position.x - robot_pose.position.x) / dist;
    midpoint.y += factor * (face_pose.position.y - robot_pose.position.y) / dist;
    midpoint.z += factor * (face_pose.position.z - robot_pose.position.z) / dist;

    return midpoint;
}

void approach_and_greet(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
    visualization_msgs::Marker latestMarker = msg->markers.back();
    geometry_msgs::Pose latestMarkerPose = latestMarker.pose;
    geometry_msgs::Pose latestMarkerPose_map;
    ROS_INFO("JUHU MARKER JE TU");
    stevilo_obrazov++;

    actionlib_msgs::GoalID goal_id;
    goal_id.id = "";
    ROS_INFO("Canceling goal number %d", i);
    i--;
    cancel_pub.publish(goal_id);
    // ROS_INFO("Latest marker point: x=%f, y=%f, z=%f", latestMarkerPose.position.x, latestMarkerPose.position.y, latestMarkerPose.position.z);
    // geometry_msgs::Pose pose = create_pose(x, y, z);
    // geometry_msgs::Point Midpoint = calculateMidpoint(pose, latestMarkerPose);
    // ROS_INFO("Created midpoint message: x=%f, y=%f, z=%f", Midpoint.x, Midpoint.y, Midpoint.z);

    // calculate vector from midpoint to face position
    // geometry_msgs::Vector3 vec;
    // vec.x = latestMarkerPose.position.x - Midpoint.x;
    // vec.y = latestMarkerPose.position.y - Midpoint.y;
    // vec.z = latestMarkerPose.position.z - Midpoint.z;

    // // calculate quaternion from vector
    // tf2::Quaternion quat;
    // quat.setRPY(0, 0, atan2(vec.y, vec.x));

    // // create goal message
    // geometry_msgs::PoseStamped goal;
    // goal.header.frame_id = "map";
    // goal.header.stamp = ros::Time::now();
    // goal.pose.position.x = Midpoint.x;
    // goal.pose.position.y = Midpoint.y;
    // goal.pose.orientation.x = quat.x();
    // goal.pose.orientation.y = quat.y();
    // goal.pose.orientation.z = quat.z();
    // goal.pose.orientation.w = quat.w();
    // goal_pub.publish(goal);

    // ROS_INFO("Approaching and greeting with orientation. w:%lf z:%lf", quat.w(), quat.z());
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    // Wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Set the goal position
    move_base_msgs::MoveBaseGoal goal;
    
    int curr_x = points[i][0];
    int curr_y = points[i][1];
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = (latestMarkerPose.position.x);
    goal.target_pose.pose.position.y = (latestMarkerPose.position.y);
    ROS_INFO("goal position x: %f and goal position y: %f ", goal.target_pose.pose.position.x , goal.target_pose.pose.position.y);
    
    goal.target_pose.pose.orientation.w = 1.0;
    
    // Set a desired distance from the goal
    double desired_distance = 0.3; // meters
    double distance_to_goal = sqrt(pow(goal.target_pose.pose.position.x, 2) + pow(goal.target_pose.pose.position.y, 2));
    double scaling_factor = (distance_to_goal - desired_distance) / distance_to_goal;
    goal.target_pose.pose.position.x *= scaling_factor;
    goal.target_pose.pose.position.y *= scaling_factor;
    // Send the goal to the navigation stack
    ac.sendGoal(goal);
    approaching_face = true;
    ROS_INFO("goal position x: %f and goal position y: %f ", goal.target_pose.pose.position.x , goal.target_pose.pose.position.y);
    ROS_INFO("Approaching the face number: %d", stevilo_obrazov);
    // Wait for the robot to reach the goal
    ac.waitForResult();

    // Check the status of the goal
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The robot has approached the face number: %d", stevilo_obrazov);
    }
    else
    {
        ROS_INFO("The robot has failed to reach the goal");
    }

    // Send a stop goal to the robot

    // Pause the execution of the code for 2 seconds
    ac.cancelGoal();
    ros::Duration(1.0).sleep();

    exercise2::PlaySound srv;
    srv.request.message = "Hello there";
    ROS_INFO("Saying hello");
    sound_client.call(srv);

    ros::Duration(1.0).sleep();
    if (stevilo_obrazov == 3)
    {
        ROS_INFO("Robot has detected every face, now we can stop");
    }

}

void messageCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    // Access the last element in the status_list array and extract the status field
    ros::Rate rate(3);
    int status = 0;
    if (cv_map.empty())
        return;

    // Check status
    if (!msg->status_list.empty())
    {
        status = msg->status_list[0].status;
        //ROS_INFO("Received status: %d, last status: %d", status, last_status);
    }

    // Check if goal has been reached
    if (last_status == -1)
    {
        ROS_INFO("Starting first goal, i=%d!", i);
        allowedNewGoal = true;
    }
    else if (status == 3 && last_status != 3 && last_status != -1)
    {
        ROS_INFO("Goal number %d reached!!", i);
        //i++;
        allowedNewGoal = true;
    }
    else if (status == 3 && last_status == 3 && ros::Time::now() - goal_set_time > ros::Duration(10.0))
    {
        ROS_INFO("Status 3 timeout i=%d!!", i);
        //i++;
        allowedNewGoal = true;
    }

    // Check if timeout
    else if (last_status == 1 && ros::Time::now() - goal_set_time > ros::Duration(20.0))
    {
        ROS_WARN("Goal %d not reached in 20 seconds", i);

        actionlib_msgs::GoalID goal_id;
        goal_id.id = "";
        cancel_pub.publish(goal_id);
        //i++;
        allowedNewGoal = true;
    }

    // ROS_INFO("GOAL %d", allowedNewGoal);
    // ROS_INFO("STATUS %d, %d", status, last_status);

    // Publish new goal if allowed
    if (allowedNewGoal)
    {
        nextGoal();
        allowedNewGoal = false;
    }

    last_status = status;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "map_goals");
    ros::NodeHandle n;

    map_sub = n.subscribe("map", 10, &mapCallback);
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    ros::Subscriber sub = n.subscribe("/move_base/status", 100, &messageCallback);
    cancel_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    face_marker_sub = n.subscribe("face_markers", 10, &approach_and_greet);
    sound_client = n.serviceClient<exercise2::PlaySound>("play_sound");
    namedWindow("Map");
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 100);
    //ros::Subscriber sub_to_pose = n.subscribe("/amcl_pose", 10, &amclPoseCallback);

    // rotate(1);
    actionlib_msgs::GoalID goal_id;
    goal_id.id = "";
    cancel_pub.publish(goal_id);
    allowedNewGoal = true;

    // int i = 0;

    ros::Rate rate(rate_freq);
    while (ros::ok())
    {

        if (!cv_map.empty())
            imshow("Map", cv_map);

        // if (i >= 5) return 0;

        /*

           if (!cv_map.empty()) {
               ROS_INFO("i: %d", i);
               int nextX = points[i][0];
               int nextY = points[i][1];
               nextGoal(nextX,nextY);
               i++;
               sleep(10);

           }
           */
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}