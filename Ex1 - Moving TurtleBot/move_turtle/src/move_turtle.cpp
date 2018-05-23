/*
 * Amir Bagaon
 * 204313100
 * move_turtle.cpp
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <string>

using namespace std;

ros::Publisher velocity_publisher;

void currentPostion();
void move(double speed, double dist);
void rotate(double speed, double target);
double toRadians(double degree);
double toDegrees(double radian);
double const PI = 3.1415;

// Topic messages callback
void poseCallback(const turtlesim::PoseConstPtr& msg)
{
    //Note: the degrees are not perfect because PI is not perfect
    ROS_INFO("x: %.2f, ang': %.2f, ang'(~~Degrees): %.2f", msg->x,
             msg->theta, toDegrees(msg->theta));
}

void rotate (double speed, double target){
    geometry_msgs::Twist msg;
    msg.angular.z =abs(speed);

    double t0 = ros::Time::now().toSec();
    double current_angle = 0;
    ros::Rate loop_rate(1000);
    while(current_angle<target) {
        velocity_publisher.publish(msg);
        double t1 = ros::Time::now().toSec();
        current_angle = speed * (t1-t0);
        loop_rate.sleep();
    }
    msg.angular.z =0;
    velocity_publisher.publish(msg);
}

double toRadians(double degree) {
    return degree * PI / 180.0;
}
double toDegrees(double radian) {
    return radian * 180.0 / PI;
}

void currentPostion() {
    geometry_msgs::Twist vel_msg;
    ros::Rate rate(1);
    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
    rate.sleep();
    ros::spinOnce();
}

void move(double speed, double distance){
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x =abs(speed);
    ros::Rate loop_rate(10000);
    double current_distance = 0;


    double t0 = ros::Time::now().toSec();
    while(current_distance < distance) {
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1-t0);
        loop_rate.sleep();
    }
    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
}

int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "move_turtle");
    string robot_name = string(argv[1]);

    ros::NodeHandle node;

    // A listener for pose
    ros::Subscriber sub = node.subscribe(robot_name + "/pose", 10, poseCallback);

    velocity_publisher = node.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 10);

    ROS_INFO("Starting position:");
    currentPostion();
    move (0.5, 1);
    rotate(toRadians(45), toRadians(45));
    ROS_INFO("Ending position:");
    currentPostion();

}


