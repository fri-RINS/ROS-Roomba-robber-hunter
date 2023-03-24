#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <actionlib_msgs/GoalStatusArray.h>

using namespace std;
using namespace cv;

Mat cv_map;
float map_resolution = 0;
geometry_msgs::TransformStamped map_transform;
ros::Time goal_set_time = ros::Time(0.0);
int last_status = 0;
ros::Publisher goal_pub;
ros::Subscriber map_sub;
int i = 0;
int points[5][2] = {
    {263, 310},
    {281, 271},
    {263, 229},
    {212, 303},
    {221, 278},
};

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

void nextGoal(int x, int y)
{

    int v = (int)cv_map.at<unsigned char>(y, x);

    // check if point is reachable
    if (v != 255)
    {
        ROS_WARN("Unable to move to (x: %d, y: %d), not reachable", x, y);
        return;
    }

    geometry_msgs::Point pt;
    geometry_msgs::Point transformed_pt;

    // convert point to image coordinate system
    pt.x = (float)x * map_resolution;
    pt.y = (float)(cv_map.rows - y) * map_resolution; // cv_map.rows - y --> because y is flipped
    pt.z = 0.0;

    // transform C.S. because origin is not at 0,0
    tf2::doTransform(pt, transformed_pt, map_transform);

    // geometry_msgs::Point transformed = map_transform * pt;

    geometry_msgs::PoseStamped goal;

    goal.header.frame_id = "map";
    goal.pose.orientation.w = 1;
    goal.pose.position.x = transformed_pt.x;
    goal.pose.position.y = transformed_pt.y;
    goal.header.stamp = ros::Time::now();

    ROS_INFO("Moving to (x: %f, y: %f)", transformed_pt.x, transformed_pt.y);

    goal_pub.publish(goal);
}
void messageCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    // Access the last element in the status_list array and extract the status field
    int status = 0;
    if (!msg->status_list.empty())
    {
        status = msg->status_list[0].status;
    }
    ROS_INFO("Received status: %d", status);

    if (status == 3 && last_status != 3 && i <= 5)
    {
        ROS_INFO("Goal number %d reached!!", i);
        int nextY = points[i][1];
        int nextX = points[i][0];
        nextGoal(nextX, nextY);
        i++;
        goal_set_time = ros::Time::now();
    }
    else if (status == 0 && last_status != 0 && i <= 5)
    {
        int nextY = points[i][1];
        int nextX = points[i][0];
        nextGoal(nextX, nextY);
        i++;
        goal_set_time = ros::Time::now();
        ROS_INFO("First goal sent to robot!");
    }
    else if (last_status == 1 && ros::Time::now() - goal_set_time > ros::Duration(20.0) && i <= 5)
    {
        ROS_WARN("Goal %d not reached in 20 seconds", i);
        int nextY = points[i][1];
        int nextX = points[i][0];
        nextGoal(nextX, nextY);
        i++;
        goal_set_time = ros::Time::now();
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

    namedWindow("Map");

    // int i = 0;
    ros::Rate rate(3);
    while (ros::ok())
    {

        if (!cv_map.empty())
            imshow("Map", cv_map);

        /*   if (i >= 5) return 0;

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