/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
#include <fstream>
#include <cmath>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nodelet/nodelet.h>
#include <vector>
#include <quadrotor_msgs/Corrections.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <quadrotor_msgs/SO3Command.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;
using namespace Eigen;
#define PI acos(-1)

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_position;
vector<Vector3d> wp_list;
Vector3d target_pt;
nav_msgs::Path wp_get;
ros::Subscriber position_cmd_sub_;
int k=0;
int flag = 0;
Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
double des_yaw_, des_yaw_dot_;


// void targetWaypointsCallback(const nav_msgs::Path & target);


void targetWaypointsCallback(const nav_msgs::Path & target)
{     
    if( target.poses[0].pose.position.z < -0.1 )
        return;
    target_pt << target.poses[0].pose.position.x,
                 target.poses[0].pose.position.y,
                 target.poses[0].pose.position.z;

}


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void getpointfdb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //ROS_INFO("x: [%f]", msg->pose.position.x);
    //ROS_INFO("y: [%f]", msg->pose.position.y);
    //ROS_INFO("z: [%f]", msg->pose.position.z);
    current_position = *msg;
}

void position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd) {
  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
  kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);
  
  des_yaw_ = cmd->yaw;
  des_yaw_dot_ = cmd->yaw_dot;
  ROS_INFO("x: [%f]", des_pos_(0));
  ROS_INFO("y: [%f]", des_pos_(1));
  ROS_INFO("z: [%f]", des_pos_(2));
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Subscriber target_get = nh.subscribe
            ("/waypoint_generator/waypoints", 10, targetWaypointsCallback);
            
    ros::Subscriber get_point = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, getpointfdb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Subscriber cmd_sub = nh.subscribe("/planning/pos_cmd", 100, 
                                            &position_cmd_callback, ros::TransportHints().tcpNoDelay());

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);
    clock_t startTime,endTime;
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2; 

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0f))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(1.0f))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        ROS_INFO("k: [%f]", k);
        pose.pose.position.x = des_pos_(0);      
        pose.pose.position.y = des_pos_(1);
        pose.pose.position.z = des_pos_(2);
        AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
        AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());
        AngleAxisd yawAngle(des_yaw_, Eigen::Vector3d::UnitZ()); 
        Quaterniond q = yawAngle * pitchAngle * rollAngle;
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


