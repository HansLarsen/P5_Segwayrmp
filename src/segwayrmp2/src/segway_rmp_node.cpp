/*
 * The MIT License (MIT)
 * Copyright (c) 2011 William Woodall <wjwwood@gmail.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a 
 * copy of this software and associated documentation files (the "Software"), 
 * to deal in the Software without restriction, including without limitation 
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 */

#include <iostream>
#include <sstream>
#include <cmath>
#include <memory>

/*
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "segway_rmp/SegwayStatusStamped.h"
*/

#include <chrono>
#include <boost/thread.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rcl/timer.h"
#include <rcl/rcl.h>
#include "rclcpp/logger.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp/utilities.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"

#include "segwayrmp/segwayrmp.h"


//Messages
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "segwayinterfacemsgs/msg/segway_status.hpp"
#include "segwayinterfacemsgs/msg/segway_status_stamped.hpp"

using namespace std::chrono_literals;

class SegwayRMPNode;

static SegwayRMPNode * segwayrmp_node_instance;

static double radians_to_degrees = 180.0 / M_PI;
static double degrees_to_radians = M_PI / 180.0;

// ROS Node class
class SegwayRMPNode : public rclcpp::Node {
public:
    SegwayRMPNode() : Node("dat_rmp_node")
    {
        this->segway_rmp = NULL;
        this->first_odometry = true;
        this->last_forward_displacement = 0.0;
        this->last_yaw_displacement = 0.0;
        this->odometry_x = 0.0;
        this->odometry_y = 0.0;
        this->odometry_w = 0.0;
        this->linear_vel = 0.0;
        this->angular_vel = 0.0;
        this->target_linear_vel = 0.0;
        this->target_angular_vel = 0.0;
        this->initial_integrated_forward_position = 0.0;
        this->initial_integrated_left_wheel_position = 0.0;
        this->initial_integrated_right_wheel_position = 0.0;
        this->initial_integrated_turn_position = 0.0;
        this->count = 0;

        odom_broadcaster = std::shared_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster(this));
    }
    
    ~SegwayRMPNode() {
        this->disconnect();
    }

    
    void disconnect() {
        if (this->segway_rmp != NULL)
            delete this->segway_rmp;
        this->segway_rmp = NULL;
    }
    
    void run() {
        if (this->getParameters()) {
            return;
        }
        
        this->setupSegwayRMP();
        
        this->setupROSComms();

        // Setup keep alive timer
        this->keep_alive_timer = create_wall_timer(std::chrono::milliseconds(50), 
            std::bind(&SegwayRMPNode::keepAliveCallback, this));
        
	this->odometry_reset_start_time = this->now();

        this->connected = false;
        while (rclcpp::ok())  {
            try {
                this->segway_rmp->connect();
                this->connected = true;
            } catch (std::exception& e) {
                std::string e_msg(e.what());
                RCLCPP_ERROR(this->get_logger(),"Exception while connecting to the SegwayRMP, check your cables and power buttons.");
                RCLCPP_ERROR(this->get_logger(),"    %s", e_msg.c_str());
                this->connected = false;
            }
            if (this->spin()) { // ROS is OK, but we aren't connected, wait then try again
                RCLCPP_WARN(this->get_logger(),"Not connected to the SegwayRMP, will retry in 5 seconds...");
                rclcpp::sleep_for(std::chrono::seconds(5));
            }
        }
    }
    
    bool spin() {
        if (rclcpp::ok() && this->connected) {
            RCLCPP_INFO(this->get_logger(),"Segway RMP Ready.");
            while (rclcpp::ok() && this->connected) {
                rclcpp::sleep_for(std::chrono::seconds(1));
            }
        }
        if (rclcpp::ok()) { // Error not shutdown
            return true;
        } else {         // Shutdown
            return false;
        }
    }
    
    /**
     * This method is called at 20Hz.  Each time it sends a movement
     * command to the Segway RMP.
     */
    void keepAliveCallback() {

        if (!this->connected || this->reset_odometry)
          return;

        if (rclcpp::ok()) {
            boost::mutex::scoped_lock lock(this->m_mutex);

            // Update the linear velocity based on the linear acceleration limits
            if (this->linear_vel < this->target_linear_vel) {
                // Must increase linear speed
                if (this->linear_pos_accel_limit == 0.0 
                    || this->target_linear_vel - this->linear_vel < this->linear_pos_accel_limit)
                    this->linear_vel = this->target_linear_vel;
                else
                     this->linear_vel += this->linear_pos_accel_limit; 
            } else if (this->linear_vel > this->target_linear_vel) {
                // Must decrease linear speed
                if (this->linear_neg_accel_limit == 0.0 
                    || this->linear_vel - this->target_linear_vel < this->linear_neg_accel_limit)
                    this->linear_vel = this->target_linear_vel;
                else
                     this->linear_vel -= this->linear_neg_accel_limit; 
            }

            // Update the angular velocity based on the angular acceleration limits
            if (this->angular_vel < this->target_angular_vel) {
                // Must increase angular speed
                if (this->angular_pos_accel_limit == 0.0
                    || this->target_angular_vel - this->angular_vel < this->angular_pos_accel_limit)
                    this->angular_vel = this->target_angular_vel;
                else
                     this->angular_vel += this->angular_pos_accel_limit; 
            } else if (this->angular_vel > this->target_angular_vel) {
                // Must decrease angular speed
                if (this->angular_neg_accel_limit == 0.0 
                    || this->angular_vel - this->target_angular_vel < this->angular_neg_accel_limit)
                    this->angular_vel = this->target_angular_vel;
                else
                     this->angular_vel -= this->angular_neg_accel_limit; 
            }

            RCLCPP_INFO(this->get_logger(),"Sending move command: linear velocity = %f, angular velocity = %f", 
               this->linear_vel, this->angular_vel);

            //if (this->linear_vel == 0 || this->angular_vel == 0) {
            //    RCLCPP_INFO(this->get_logger(),"Sending Segway Command: l=%f a=%f", this->linear_vel, this->angular_vel);
            //}
            try {
                this->segway_rmp->move(this->linear_vel, this->angular_vel);
            } catch (std::exception& e) {
                std::string e_msg(e.what());
                RCLCPP_ERROR(this->get_logger(),"Error commanding Segway RMP: %s", e_msg.c_str());
                this->connected = false;
                this->disconnect();
            }
        }
    }
    
    void handleStatus(segwayrmp::SegwayStatus::Ptr &ss_ptr) {
        if (!this->connected)
            return;
        // Get the time
        rclcpp::Time current_time = this->now();

        this->sss_msg.header.stamp = current_time;
        
        segwayrmp::SegwayStatus &ss = *(ss_ptr);

        // Check if an odometry reset is still required
        if (this->reset_odometry) {
          if ((current_time - this->odometry_reset_start_time) < 0.25s) {
            return; // discard readings for the first 0.25 seconds
          }
          if (fabs(ss.integrated_forward_position) < 1e-3 &&
              fabs(ss.integrated_turn_position) < 1e-3 &&
              fabs(ss.integrated_left_wheel_position) < 1e-3 &&
              fabs(ss.integrated_right_wheel_position) < 1e-3) {
            this->initial_integrated_forward_position = ss.integrated_forward_position;
            this->initial_integrated_left_wheel_position = ss.integrated_left_wheel_position;
            this->initial_integrated_right_wheel_position = ss.integrated_right_wheel_position;
            this->initial_integrated_turn_position = ss.integrated_turn_position;
            RCLCPP_INFO(this->get_logger(),"Integrators reset by Segway RMP successfully");
            this->reset_odometry = false;
          } else if ((current_time - this->odometry_reset_start_time) > std::chrono::duration<double>(this->odometry_reset_duration) ) {
            this->initial_integrated_forward_position = ss.integrated_forward_position;
            this->initial_integrated_left_wheel_position = ss.integrated_left_wheel_position;
            this->initial_integrated_right_wheel_position = ss.integrated_right_wheel_position;
            this->initial_integrated_turn_position = ss.integrated_turn_position;
            RCLCPP_INFO(this->get_logger(),"Integrator reset by Segway RMP failed. Performing software reset"); 
            this->reset_odometry = false;
          } else {
            return; // continue waiting for odometry to be reset
          }
        }

        this->sss_msg.segway.pitch_angle = ss.pitch * degrees_to_radians;
        this->sss_msg.segway.pitch_rate = ss.pitch_rate * degrees_to_radians;
        this->sss_msg.segway.roll_angle = ss.roll * degrees_to_radians;
        this->sss_msg.segway.roll_rate = ss.roll_rate * degrees_to_radians;
        this->sss_msg.segway.left_wheel_velocity = ss.left_wheel_speed;
        this->sss_msg.segway.right_wheel_velocity = ss.right_wheel_speed;
        this->sss_msg.segway.yaw_rate = ss.yaw_rate * degrees_to_radians;
        this->sss_msg.segway.servo_frames = ss.servo_frames;
        this->sss_msg.segway.left_wheel_displacement = 
            ss.integrated_left_wheel_position - this->initial_integrated_left_wheel_position;
        this->sss_msg.segway.right_wheel_displacement = 
            ss.integrated_right_wheel_position - this->initial_integrated_right_wheel_position;
        this->sss_msg.segway.forward_displacement = 
            ss.integrated_forward_position - this->initial_integrated_forward_position;
        this->sss_msg.segway.yaw_displacement = 
            (ss.integrated_turn_position - this->initial_integrated_turn_position) * degrees_to_radians;
        this->sss_msg.segway.left_motor_torque = ss.left_motor_torque;
        this->sss_msg.segway.right_motor_torque = ss.right_motor_torque;
        this->sss_msg.segway.operation_mode = ss.operational_mode;
        this->sss_msg.segway.gain_schedule = ss.controller_gain_schedule;
        this->sss_msg.segway.ui_battery = ss.ui_battery_voltage;
        this->sss_msg.segway.powerbase_battery = ss.powerbase_battery_voltage;
        this->sss_msg.segway.motors_enabled = (bool)(ss.motor_status);
        
        //segway_status_pub->publish(this->sss_msg);
        
        // TODO: possibly spin this off in another thread
        
        // Grab the newest Segway data
        float forward_displacement = 
            (ss.integrated_forward_position - this->initial_integrated_forward_position) * 
            this->linear_odom_scale;
        float yaw_displacement = 
            (ss.integrated_turn_position - this->initial_integrated_turn_position) * 
            degrees_to_radians * this->angular_odom_scale;
        float yaw_rate = ss.yaw_rate * degrees_to_radians;
        
        // Integrate the displacements over time
        // If not the first odometry calculate the delta in displacements
        float vel_x = 0.0;
        float vel_y = 0.0;
        if(!this->first_odometry) {
            float delta_forward_displacement = 
                forward_displacement - this->last_forward_displacement;
            double delta_time = (current_time-this->last_time).seconds();
            // Update accumulated odometries and calculate the x and y components 
            // of velocity
            this->odometry_w = yaw_displacement;
            float delta_odometry_x = 
                delta_forward_displacement * std::cos(this->odometry_w);
            vel_x = delta_odometry_x / delta_time;
            this->odometry_x += delta_odometry_x;
            float delta_odometry_y = 
                delta_forward_displacement * std::sin(this->odometry_w);
            vel_y = delta_odometry_y / delta_time;
            this->odometry_y += delta_odometry_y;
        } else {
            this->first_odometry = false;
        }
        // No matter what update the previouse (last) displacements
        this->last_forward_displacement = forward_displacement;
        this->last_yaw_displacement = yaw_displacement;
        this->last_time = current_time;
        
        // Create a Quaternion from the yaw displacement
        
        tf2::Quaternion q;
        q.setEuler( yaw_displacement, 0, 0 );
        geometry_msgs::msg::Quaternion quat;
        quat.x = q.getX();
        quat.y = q.getY();
        quat.z = q.getZ();
        quat.w = q.getW();
        
        //geometry_msgs::msg::Quaternion quat = tf2::toMsg<tf2::Quaternion, geometry_msgs::msg::Quaternion>( q );
        
        // Publish the Transform odom->base_link
        if (this->broadcast_tf) {
            this->odom_trans.header.stamp = current_time;
            
            this->odom_trans.transform.translation.x = this->odometry_x;
            this->odom_trans.transform.translation.y = this->odometry_y;
            this->odom_trans.transform.translation.z = 0.0;
            this->odom_trans.transform.rotation = quat;
            
            //send the transform
            this->odom_broadcaster->sendTransform(this->odom_trans);
        }
        
        // Publish Odometry
        this->odom_msg.header.stamp = current_time;
        this->odom_msg.pose.pose.position.x = this->odometry_x;
        this->odom_msg.pose.pose.position.y = this->odometry_y;
        this->odom_msg.pose.pose.position.z = 0.0;
        this->odom_msg.pose.pose.orientation = quat;
        this->odom_msg.pose.covariance[0] = 0.00001;
        this->odom_msg.pose.covariance[7] = 0.00001;
        this->odom_msg.pose.covariance[14] = 1000000000000.0;
        this->odom_msg.pose.covariance[21] = 1000000000000.0;
        this->odom_msg.pose.covariance[28] = 1000000000000.0;
        this->odom_msg.pose.covariance[35] = 0.001;
        
        this->odom_msg.twist.twist.linear.x = vel_x;
        this->odom_msg.twist.twist.linear.y = vel_y;
        this->odom_msg.twist.twist.angular.z = yaw_rate;
        
        this->odom_pub->publish(this->odom_msg);
    }
    
    /**
     * This method is called if a motor command is not received
     * within the segway_motor_timeout interval.  It halts the
     * robot for safety reasons.
     */
    void motor_timeoutCallback() {
        boost::mutex::scoped_lock lock(m_mutex);
        //RCLCPP_INFO(this->get_logger(),"Motor command timeout!  Setting target linear and angular velocities to be zero.");
        this->target_linear_vel = 0.0;
        this->target_angular_vel = 0.0;
    }
    
    /**
     * The handler for messages received on the 'cmd_vel' topic.
     */
    void cmd_velCallback(const geometry_msgs::msg::Twist::ConstPtr msg) {
        if (!this->connected)
            return;
        boost::mutex::scoped_lock lock(m_mutex);
        double x = msg->linear.x, z = msg->angular.z;
        if (this->invert_x) {
            x *= -1;
        }
        if (this->invert_z) {
            z *= -1;
        }
        if (this->max_linear_vel != 0.0) {
          if (abs(x) > this->max_linear_vel) {
            x = (x > 0) ? this->max_linear_vel : -this->max_linear_vel;
          }
        }
        if (this->max_angular_vel != 0.0) {
          if (abs(z) > this->max_angular_vel) {
            z = (z > 0) ? this->max_angular_vel : -this->max_angular_vel;
          }
        }
        this->target_linear_vel = x;
        this->target_angular_vel = z * radians_to_degrees; // Convert to degrees

        //RCLCPP_INFO(this->get_logger(),"Received motor command linear vel = %f, angular vel = %f.",
        //    this->target_linear_vel, this->target_angular_vel);

        this->motor_timeout_timer = create_wall_timer(std::chrono::seconds(this->segway_motor_timeout), 
            std::bind(&SegwayRMPNode::motor_timeoutCallback, this));
    }
private:
    // Functions
    void setupROSComms() {
        // Subscribe to command velocities
        cmd_velSubscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1000, std::bind(&SegwayRMPNode::cmd_velCallback, this, std::placeholders::_1));
        //this->cmd_velSubscriber = n->subscribe("cmd_vel", 1000, &SegwayRMPNode::cmd_velCallback, this);
        // Advertise the SegwayStatusStamped
        //segway_status_pub = this->create_publisher<segwayinterfacemsgs::msg::SegwayStatusStamped>("segway_status", 1000);
        //this->segway_status_pub = n->advertise<segway_rmp::SegwayStatusStamped>("segway_status", 1000);

        // Advertise the Odometry Msg
        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1000);
        //this->odom_pub = n->advertise<nav_msgs::Odometry>("odom", 50);
    }

    // Callback wrapper
    void handleStatusWrapper(segwayrmp::SegwayStatus::Ptr ss) {
        this->handleStatus(ss);
    }

    // Message Wrappers
    void handleDebugMessages(const std::string &msg) {RCLCPP_DEBUG(this->get_logger(), msg.c_str());}
    void handleInfoMessages(const std::string &msg) {RCLCPP_DEBUG(this->get_logger(), msg.c_str());}
    void handleErrorMessages(const std::string &msg) {RCLCPP_DEBUG(this->get_logger(), msg.c_str());}

    void setupSegwayRMP() {
        std::stringstream ss;
        ss << "Connecting to Segway RMP via ";
        this->segway_rmp = new segwayrmp::SegwayRMP(this->interface_type, this->segway_rmp_type);
        RCLCPP_INFO(get_logger(), "DatDebug");
        if (this->interface_type_str == "serial") {
            ss << "serial on serial port: " << this->serial_port;
            this->segway_rmp->configureSerial(this->serial_port);
        } else if (this->interface_type_str == "usb") {
            ss << "usb ";
            if (this->usb_selector == "serial_number") {
                ss << "identified by the device serial number: " << this->serial_number;
                this->segway_rmp->configureUSBBySerial(this->serial_number);
            }
            if (this->usb_selector == "description") {
                ss << "identified by the device description: " << this->usb_description;
                RCLCPP_INFO(get_logger(), "DatDebug2");
                this->segway_rmp->configureUSBByDescription(this->usb_description, 460800);
            }
            if (this->usb_selector == "index") {
                ss << "identified by the device index: " << this->usb_index;
                this->segway_rmp->configureUSBByIndex(this->usb_index);
            }
        }
        RCLCPP_INFO(this->get_logger(),"%s", ss.str().c_str());
        
        // Set the instance variable
        segwayrmp_node_instance = this;
        
        // Set callbacks for segway data and messages
        this->segway_rmp->setStatusCallback(std::bind(&SegwayRMPNode::handleStatusWrapper, this, std::placeholders::_1));
        this->segway_rmp->setLogMsgCallback("debug", std::bind(&SegwayRMPNode::handleDebugMessages, this, std::placeholders::_1));
        this->segway_rmp->setLogMsgCallback("info", std::bind(&SegwayRMPNode::handleInfoMessages, this, std::placeholders::_1));
        this->segway_rmp->setLogMsgCallback("error", std::bind(&SegwayRMPNode::handleErrorMessages, this, std::placeholders::_1));

    }
    
    int getParameters() {
        //********************************************************************** configs must be set in the parameter server ****************************************************************************************//
        // In ROS1 a default could be given in the code, this is not the case for ROS2, the params must therefore be set in the server.

        // Get Interface Type
        //this->get_parameter("interface_type", this->interface_type_str); //, std::string("serial"));
        this->interface_type_str = "serial";
        std::cout << this->interface_type_str;
        // Get Configurations based on Interface Type
        if (this->interface_type_str == "serial") {
            this->interface_type = segwayrmp::serial;
            //this->get_parameter("serial_port", this->serial_port); //, std::string("/dev/ttyUSB0"));
            serial_port = "/dev/ttyUSB0";
        } else if (this->interface_type_str == "usb") {
            this->interface_type = segwayrmp::usb;
            //this->get_parameter("usb_selector", this->usb_selector);//, std::string("index"));
            this->usb_selector = "serial_number";
            if (this->usb_selector == "index") {
                this->usb_index = 0;
                //this->get_parameter("usb_index", this->usb_index);//, 0);
            } else if (this->usb_selector == "serial_number") {
                //this->get_parameter("usb_serial_number", this->serial_number);//, std::string("00000000"));
                this->serial_number = "00000239";
                if (this->serial_number == std::string("00000000")) {
                    RCLCPP_WARN(this->get_logger(),"The serial_number parameter is set to the default 00000000, which shouldn't work.");
                }
            } else if (this->usb_selector == "description") {
                usb_description = "Robotic Mobile Platform";
                usb_description = "Future Technology Devices International, Ltd Segway Robotic Mobility Platforms 200";
                //this->get_parameter("usb_description", this->serial_number);//, std::string("Robotic Mobile Platform"));
            } else {
                RCLCPP_ERROR(this->get_logger(),
                    "Invalid USB selector: %s, valid types are 'index', 'serial_number', and 'description'.", 
                    this->usb_selector.c_str());
                return 1;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid interface type: %s, valid interface types are 'serial' and 'usb'.",
                this->interface_type_str.c_str());
            return 1;
        }
        // Get Setup Motor Timeout
        //this->get_parameter("motor_timeout", this->segway_motor_timeout);//, 0.5);
        this->segway_motor_timeout = 1;
        // Get frame id parameter
        //this->get_parameter("frame_id", frame_id); //, std::string("base_link"));
        this->frame_id = "base_link";
        //this->get_parameter("odom_frame_id", odom_frame_id); //, std::string("odom"));
        odom_frame_id = "odom";
        this->sss_msg.header.frame_id = this->frame_id;
        this->odom_trans.header.frame_id = this->odom_frame_id;
        this->odom_trans.child_frame_id = this->frame_id;
        this->odom_msg.header.frame_id = this->odom_frame_id;
        this->odom_msg.child_frame_id = this->frame_id;
        // Get cmd_vel inversion parameters
        this->get_parameter("invert_linear_vel_cmds", invert_x);//, false);
        this->invert_x = false;
        this->get_parameter("invert_angular_vel_cmds", invert_z);//, false);
        this->invert_z = false;
        // Get option for enable/disable tf broadcasting
        this->get_parameter("broadcast_tf", this->broadcast_tf);//, true);
        this->broadcast_tf = true;
        // Get the segway rmp type
        //this->get_parameter("rmp_type", segway_rmp_type_str);//, std::string("200/400"));
        std::string segway_rmp_type_str = "200/400";
        if (segway_rmp_type_str == "200/400") {
            this->segway_rmp_type = segwayrmp::rmp200;
        } else if (segway_rmp_type_str == "50/100") {
            this->segway_rmp_type = segwayrmp::rmp100;
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid rmp type: %s, valid rmp types are '200/400' and '50/100'.",
                segway_rmp_type_str.c_str());
            return 1;
        }

        // Get the linear acceleration limits in m/s^2.  Zero means infinite.
        this->get_parameter("linear_pos_accel_limit", this->linear_pos_accel_limit);//, 0.0);
        this->get_parameter("linear_neg_accel_limit", this->linear_neg_accel_limit);//, 0.0);

        // Get the angular acceleration limits in deg/s^2.  Zero means infinite.
        this->get_parameter("angular_pos_accel_limit", this->angular_pos_accel_limit);//, 0.0);
        this->get_parameter("angular_neg_accel_limit", this->angular_neg_accel_limit);//, 0.0);
        
        // Check for valid acceleration limits
        if (this->linear_pos_accel_limit < 0) {
            RCLCPP_ERROR(this->get_logger(),"Invalid linear positive acceleration limit of %f (must be non-negative).",
                this->linear_pos_accel_limit);
            return 1;
        }
        if (this->linear_neg_accel_limit < 0) {
            RCLCPP_ERROR(this->get_logger(),"Invalid linear negative acceleration limit of %f (must be non-negative).",
                this->linear_neg_accel_limit);
            return 1;
        }
        if (this->angular_pos_accel_limit < 0) {
            RCLCPP_ERROR(this->get_logger(),"Invalid angular positive acceleration limit of %f (must be non-negative).",
                this->angular_pos_accel_limit);
            return 1;
        }
        if (this->angular_neg_accel_limit < 0) {
            RCLCPP_ERROR(this->get_logger(),"Invalid angular negative acceleration limit of %f (must be non-negative).",
                this->angular_neg_accel_limit);
            return 1;
        }

        RCLCPP_INFO(this->get_logger(),"Accel limits: linear: pos = %f, neg = %f, angular: pos = %f, neg = %f.",
            this->linear_pos_accel_limit, this->linear_neg_accel_limit, 
            this->angular_pos_accel_limit, this->angular_neg_accel_limit);

        // Get velocity limits. Zero means no limit
        this->get_parameter("max_linear_vel", this->max_linear_vel);//, 0.0);
        this->get_parameter("max_angular_vel", this->max_angular_vel);//, 0.0);
        
        if (this->max_linear_vel < 0) {
            RCLCPP_ERROR(this->get_logger(),"Invalid max linear velocity limit of %f (must be non-negative).",
                this->max_linear_vel);
            return 1;
        }
 
        if (this->max_angular_vel < 0) {
            RCLCPP_ERROR(this->get_logger(),"Invalid max angular velocity limit of %f (must be non-negative).",
                this->max_angular_vel);
            return 1;
        }

        RCLCPP_INFO(this->get_logger(),"Velocity limits: linear: %f, angular: %f.",
            this->max_linear_vel, this->max_angular_vel); 
        
        // Convert the linear acceleration limits to have units of (m/s^2)/20 since
        // the movement commands are sent to the Segway at 20Hz.
        this->linear_pos_accel_limit /= 20;
        this->linear_neg_accel_limit /= 20;

        // Convert the angular acceleration limits to have units of (deg/s^2)/20 since
        // the movement commands are sent to the Segway at 20Hz.
        this->angular_pos_accel_limit /= 20;
        this->angular_neg_accel_limit /= 20;

        // Get the scale correction parameters for odometry
        this->get_parameter("linear_odom_scale", this->linear_odom_scale);//, 1.0);
        this->get_parameter("angular_odom_scale", this->angular_odom_scale);//, 1.0);

        // Check if a software odometry reset is required
        this->get_parameter("reset_odometry", this->reset_odometry);//, false);
        this->get_parameter("odometry_reset_duration", this->odometry_reset_duration);//, 1.0);
    
        return 0;
    }
    
    rclcpp::TimerBase::SharedPtr keep_alive_timer;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr  cmd_velSubscriber;
    rclcpp::Publisher<segwayinterfacemsgs::msg::SegwayStatusStamped>::SharedPtr segway_status_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
    
    segwayrmp::SegwayRMP * segway_rmp;
    
    std::string interface_type_str;
    segwayrmp::InterfaceType interface_type;
    segwayrmp::SegwayRMPType segway_rmp_type;
    std::string serial_port;
    std::string usb_selector;
    std::string serial_number;
    std::string usb_description;
    int usb_index;
    
    int segway_motor_timeout;
    rclcpp::TimerBase::SharedPtr motor_timeout_timer;
    
    std::string frame_id;
    std::string odom_frame_id;
    bool invert_x, invert_z;
    bool broadcast_tf;
    
    double linear_vel;
    double angular_vel; // The angular velocity in deg/s

    double target_linear_vel;  // The ideal linear velocity in m/s
    double target_angular_vel; // The ideal angular velocity in deg/s

    double linear_pos_accel_limit;  // The max linear acceleration in (m/s^2)/20
    double linear_neg_accel_limit;  // The max linear deceleration in (m/s^2)/20
    double angular_pos_accel_limit; // The max angular acceleration in (deg/s^2)/20
    double angular_neg_accel_limit; // The max angular deceleration in (deg/s^2)/20

    double linear_odom_scale;       // linear odometry scale correction 
    double angular_odom_scale;      // angular odometry scale correction

    double max_linear_vel;  // maximum allowed magnitude of velocity
    double max_angular_vel;
    
    bool connected;
    
    segwayinterfacemsgs::msg::SegwayStatusStamped sss_msg;
    geometry_msgs::msg::TransformStamped odom_trans;
    nav_msgs::msg::Odometry odom_msg;
    
    int count;
    
    bool first_odometry;
    float last_forward_displacement;
    float last_yaw_displacement;
    float odometry_x;
    float odometry_y;
    float odometry_w;
    rclcpp::Time last_time;
    
    boost::mutex m_mutex;

    // Hardware reset of integrators can sometimes fail.
    // These help in performing a software reset.
    bool reset_odometry;
    double odometry_reset_duration;
    rclcpp::Time odometry_reset_start_time;
    double initial_integrated_forward_position;
    double initial_integrated_left_wheel_position;
    double initial_integrated_right_wheel_position;
    double initial_integrated_turn_position;
    
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    SegwayRMPNode segwayrmp_node;
    
    segwayrmp_node.run();
    
    return 0;
}
