/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "qnode.hpp"

#include <geometry_msgs/Twist.h>

#include "cglobal.hpp"

#include <QTime>

// Define function
#define max(a, b) (a > b ? a : b)
#define min(a, b) (a < b ? a : b)

double speed = 0.2, turn = 1.0, translate = 0.2;
double x = 0.0, th = 0.0, y = 0.0;
int count = 0;
double target_speed = 0.0, target_turn = 0.0, target_translate = 0.0;
double control_speed = 0.0, control_turn = 0.0, control_translate = 0.0;
double key_value_bindings[18][3] = {
    {},
    {},
    {1.0, 0.0, 0.0},
    {-1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, -1.0, 0.0},
    {1.0, 1.0, 0.0},
    {1.0, -1.0, 0.0},
    {-1.0, -1.0, 0.0},
    {-1.0, 1.0, 0.0},
    {0.0, 0.0, 1.0},
    {0.0, 0.0, -1.0},
    {1.1, 1.0, 1.0},
    {0.9, 1.0, 1.0},
    {1.0, 1.0, 1.1},
    {1.0, 1.0, 0.9},
    {1.0, 1.1, 1.0},
    {1.0, 0.9, 1.0}
};

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace jupiter_gui_node {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"jupiter_gui_node");
	if ( ! ros::master::check() ) {
//        log(Info,std::string("I sent: "));
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    twist_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    jupiter_state_subscriber = n.subscribe("/jupiter_wheel_state", 10, &QNode::jupiterStateCallback, this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"jupiter_gui_node");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    twist_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    jupiter_state_subscriber = n.subscribe("/jupiter_wheel_state", 10, &QNode::jupiterStateCallback, this);
	start();
	return true;
}

void QNode::run() {
    QTime t;
    ros::Rate loop_rate(50);
    int t_reset = 0;
    int t_reset_value = 120;

    while(ros::ok()) {
        switch(teleop_mode) {
        case NO_TELEOP:
            break;
        case KEYBOARD_TELEOP:
            break;
        case GUI_TELEOP:
            if (key_value == RESET) {
                geometry_msgs::Twist twist;
                twist.linear.x = 100; twist.linear.y = 100; twist.linear.z = t_reset_value;
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 100;
                twist_publisher.publish(twist);
                t_reset ++;
                if ( t_reset == 150) {
                    t_reset_value = -130;
//                    t.start();
//                    while(t.elapsed() < 2000);
                } else if (t_reset == 300) {
                    t_reset_value = 10;
//                    t.start();
//                    while(t.elapsed() < 2000);
                } else if (t_reset == 450){
//                    t.start();
//                    while(t.elapsed() < 2000);
                    key_value = NO_OPERATION;
                    t_reset_value = 120;
                    t_reset = 0;
                }
            } else if (key_value == BRAKE) {
                geometry_msgs::Twist twist;
                twist.linear.x = 1000; twist.linear.y = 1000; twist.linear.z = 0;
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 1000;
                twist_publisher.publish(twist);
            } else {
                if (key_value >= UP && key_value <= RIGHT) {
                    x = key_value_bindings[key_value][0];
                    th = key_value_bindings[key_value][1];
                    y = key_value_bindings[key_value][2];
                    count = 0;
                } else if (key_value >= INCREASE_MAX_SPEEDS && key_value <= DECREASE_ANGULAR_SPEED) {
                    speed = speed * key_value_bindings[key_value][0];
                    turn = turn * key_value_bindings[key_value][1];
                    translate = translate * key_value_bindings[key_value][2];
                    count = 0;
                    Q_EMIT maxSpeed(speed, translate, turn);
                } else if (key_value == STOP) {
                    x = 0.0;
                    th = 0.0;
                    y = 0.0;
                    control_speed = 0.0;
                    control_turn = 0.0;
                    control_translate = 0.0;
                } else if (key_value == NO_OPERATION) {
                    count = count + 1;
                    if (count > 4) {
                        x = 0.0;
                        th = 0.0;
                        y = 0.0;
                    }
                }

                target_speed = speed * x;
                target_turn = turn * th;
                target_translate = translate * y;

                if (target_speed > control_speed)
                    control_speed = min(target_speed, control_speed + 0.02);
                else if (target_speed < control_speed)
                    control_speed = max(target_speed, control_speed - 0.02);
                else
                    control_speed = target_speed;

                if (target_turn > control_turn)
                    control_turn = min(target_turn, control_turn + 0.1);
                else if (target_turn < control_turn)
                    control_turn = max(target_turn, control_turn - 0.1);
                else
                    control_turn = target_turn;

                if (target_translate > control_translate)
                    control_translate = min(target_translate, control_translate + 0.02);
                else if (target_translate < control_translate)
                    control_translate = max(target_translate, control_translate - 0.02);
                else
                    control_translate = target_translate;

                geometry_msgs::Twist twist;
                twist.linear.x = control_speed; twist.linear.y = control_translate; twist.linear.z = 0;
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn;
                twist_publisher.publish(twist);
                Q_EMIT cmdVel(twist.linear.x, twist.linear.y, twist.angular.z);
            }
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::jupiterStateCallback(const jupiter_serial::JupiterWheelState & msg) {
//    logging_model_speed.insertRows(logging_model_speed.rowCount(),1);
//    std::stringstream logging_model_speed_msg;
//    logging_model_speed_msg << "v1: " << msg.speed.wheel_1 / 180 * 3.14159 / 100 << "  \n" << "v2: "
//                            << msg.speed.wheel_2 / 180 * 3.14159 / 100 << "  \n" << "v2: "
//                            << msg.speed.wheel_3  / 180 * 3.14159 / 100 << "  \n" << "v3: "
//                            << msg.speed.wheel_4 / 180 * 3.14159 / 100 ;
//    QVariant new_speed_row(QString(logging_model_speed_msg.str().c_str()));
//    logging_model_speed.setData(logging_model_speed.index(logging_model_speed.rowCount()-1),new_speed_row);
    Q_EMIT wheelSpeedFeedbackUpdated(msg.speed.wheel_1, msg.speed.wheel_2,
                                     msg.speed.wheel_3, msg.speed.wheel_4); // used to readjust the scrollbar

//    logging_model_direct.insertRows(logging_model_direct.rowCount(),1);
//    std::stringstream logging_model_direct_msg;
//    logging_model_direct_msg << "d1: " << msg.direction.wheel_1 << "  " << "d2: " << msg.direction.wheel_2
//                      << "  " << "d3: " << msg.direction.wheel_3 << "  " << "d4: " << msg.direction.wheel_4;
//    QVariant new_direct_row(QString(logging_model_direct_msg.str().c_str()));
//    logging_model_direct.setData(logging_model_direct.index(logging_model_direct.rowCount()-1),new_direct_row);
    Q_EMIT wheelDirectFeedbackUpdated(msg.direction.wheel_1, msg.direction.wheel_2,
                                      msg.direction.wheel_3, msg.direction.wheel_4);

//    logging_model_battery.insertRows(logging_model_battery.rowCount(),1);
//    std::stringstream logging_model_battery_msg;
//    logging_model_battery_msg << "b1: " << msg.battery.battery_1 << "  " << "b2: " << msg.battery.battery_2
//                       << "  " << "b3: " << msg.battery.battery_3 << "  " << "b4: " << msg.battery.battery_4
//                       << "  " << "b5: " << msg.battery.battery_5;
//    QVariant new_battery_row(QString(logging_model_battery_msg.str().c_str()));
//    logging_model_battery.setData(logging_model_battery.index(logging_model_battery.rowCount()-1),new_battery_row);
//    Q_EMIT batteryFeedbackUpdated();
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
                logging_model_msg << "[DEBUG] ["  "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
                logging_model_msg << "[INFO] ["  "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
                logging_model_msg << "[INFO] ["  "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
                logging_model_msg << "[ERROR] ["  "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
                logging_model_msg << "[FATAL] ["  "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    //Q_EMIT aloggingUpdated(); // used to readjust the scrollbar
}

}  // namespace jupiter_gui_node
