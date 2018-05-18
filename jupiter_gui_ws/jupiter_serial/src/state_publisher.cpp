#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <math.h>
#include <string.h>

#include "jupiter_serial/JupiterWheelState.h"

#define FRAME_START_FLAG 0x73    // 定义帧头为0x73
#define FRAME_END_FLAG 0x65      // 定义帧尾为0x65
#define TRANSMISSION_CODE_OF_HW_TO_PC 0x22   // 定义传输码0x22代表从HW硬件控制器发数据到PC上位机
#define FUNCTION_CODE_OF_VELOCITY_POSITION_RECEIVE 0x01   // 定义功能码0x01代表速度、位置接收
#define FUNCTION_CODE_OF_POWER_UPLOAD 0x02                // 定义功能码0x02代表电量上传
#define FUNCTION_CODE_OF_ERROR_MESSAGE_RECEIVE 0xEE       // 定义功能码0xEE代表错误信息接收
#define ERROR_TYPE_OF_MOTOR_RUNNING 0x01   // 定义电机运行错误为0x01
#define ERROR_TYPE_OF_POWER_ABNORMAL 0x02      // 定义电量异常为0x02

using namespace std;
using namespace boost::asio;

//initial position
double x = 0.0;
double y = 0.0;
double th = 0.0;

//velocity
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

unsigned int a = 0;

unsigned char receive_vel_pos[32];        // 定义一个速度、位置信息存储数组
unsigned char receive_power_upload[11];  // 定义一个电量上传信息存储数组
unsigned char receive_motor_error[15];    // 定义一个电机运行错误存储数组
unsigned char receive_power_error[12];    // 定义一个电量异常存储数组

// state[0]表示速度、位置数据接收状态      0:数据接收出错 1:数据接收无误
// state[1]表示电量上传数据接收状态        0:数据接收出错 1:数据接收无误
// state[2]表示电机运行错误数据接收状态    0:数据接收出错 1:数据接收无误
// state[3]表示电量异常数据接收状态        0:数据接收出错 1:数据接收无误
short int state[4] = {0, 0, 0, 0};

// 定义一个联合体（32位的unsigned int，16位的unsigned short int, 8位的unsigned char），用于数据转换
union tran
{
   int i;
   short int j;
   unsigned char c[4];
}v[4], s[4];

// 定义一个串口数据帧结构体
struct Uart1_Control_Frame
{
	unsigned char frame_start_flag;  // 帧头
	unsigned char tranfer_byte; // 传输码
	unsigned char func_byte;    // 功能码
	unsigned char data_size;    // 数据长度
	unsigned char frame_data[50];   // 数据域
	unsigned char checksum_byte;     // 校验码
	unsigned char frame_end_flag;  // 帧尾
} recv_bag;

unsigned char state_machine = 0;  // 表示状态机状态
unsigned char frame_data_index = 0; // 表示帧数据域数组下标
unsigned char recv_byte[1];       //暂存从缓冲区里读到的一个字节，类型需为数组才对应buffer()函数里的参数

// 串口
io_service iosev;
serial_port sp(iosev, "/dev/ttyUSB0");
//serial_port sp(iosev, "/dev/pts/21");

// 函数声明
unsigned char uart1_recv_frame(unsigned char *state_machine, unsigned char *frame_data_index,
					struct Uart1_Control_Frame *recv_bag, unsigned char recv_byte);
unsigned char check_sum(struct Uart1_Control_Frame *recv_bag);
void serialport_init();
void variable_init();
void receive_vel_pos_transform();
void print_uart1_data();

// 主函数
int main(int argc, char** argv) {

    // ROS初始化
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    
    ros::Publisher jupiter_wheel_state_pub = n.advertise<jupiter_serial::JupiterWheelState>("/jupiter_wheel_state", 10);

    ros::Time current_time;
    ros::Time last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(20);
    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    // 串口初始化
    serialport_init();

    // 变量初始化
    variable_init();
    
    while(ros::ok())
    {
		read(sp,buffer(recv_byte));   // 读取缓冲区数据

		if (uart1_recv_frame(&state_machine, &frame_data_index, &recv_bag, recv_byte[0])) {	// 判断帧数据数据是否接收结束
			
			switch (recv_bag.func_byte) { // 协议功能码判断
		        case FUNCTION_CODE_OF_VELOCITY_POSITION_RECEIVE: // 速度、位置接收协议
		            memcpy(receive_vel_pos, recv_bag.frame_data, sizeof(unsigned char)*recv_bag.data_size);
		            state[0] = 1;
		            break;
		        case FUNCTION_CODE_OF_POWER_UPLOAD: // 电量上传协议
		            memcpy(receive_power_upload, recv_bag.frame_data, sizeof(unsigned char)*recv_bag.data_size);
		            state[1] = 1;
		            break;
		        case FUNCTION_CODE_OF_ERROR_MESSAGE_RECEIVE: // 错误信息接收协议
		            if (recv_bag.frame_data[0] == ERROR_TYPE_OF_MOTOR_RUNNING) { // 判断是否为电机运行错误
		                memcpy(receive_motor_error, recv_bag.frame_data, sizeof(unsigned char)*recv_bag.data_size);
		                state[2] = 1;
		            } else if (recv_bag.frame_data[0] == ERROR_TYPE_OF_POWER_ABNORMAL) { // 判断是否为电量异常
						memcpy(receive_power_error, recv_bag.frame_data, sizeof(unsigned char)*recv_bag.data_size);                 
						state[3] = 1;
		            }
		            break;
		    }

			// 判断速度、位置数据接收是否正常并发布话题
			if (state[0]) {
				receive_vel_pos_transform(); // 接收速度、位置转换

				double dt = (current_time - last_time).toSec();
				double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
				double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
				double delta_th = vth * dt;

				x += delta_x;
				y += delta_y;
				th += delta_th;

				geometry_msgs::Quaternion odom_quat;
				odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

				// update transform
				odom_trans.header.stamp = current_time;

				odom_trans.transform.translation.x = x;
				odom_trans.transform.translation.y = y;
				odom_trans.transform.translation.z = 0.0;
				odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

				//filling the odometry
				nav_msgs::Odometry odom;
				odom.header.stamp = current_time;
				odom.header.frame_id = "odom";
				odom.child_frame_id = "base_link";

				// position
				odom.pose.pose.position.x = x;
				odom.pose.pose.position.y = y;
				odom.pose.pose.position.z = 0.0;
				odom.pose.pose.orientation = odom_quat;

				//velocity
				odom.twist.twist.linear.x = vx;
				odom.twist.twist.linear.y = vy;
				odom.twist.twist.linear.z = 0.0;
				odom.twist.twist.angular.x = 0.0;
				odom.twist.twist.angular.y = 0.0;
				odom.twist.twist.angular.z = vth;

				last_time = current_time;

				// publishing the odometry and the new tf
				broadcaster.sendTransform(odom_trans);
				odom_pub.publish(odom);
			}

            jupiter_serial::JupiterWheelState jupiter_wheel_state;

            // speed
            jupiter_wheel_state.speed.wheel_1 = v[0].i;
            jupiter_wheel_state.speed.wheel_2 = v[1].i;
            jupiter_wheel_state.speed.wheel_3 = v[2].i;
            jupiter_wheel_state.speed.wheel_4 = v[3].i;

            // direction
            jupiter_wheel_state.direction.wheel_1 = s[0].j;
            jupiter_wheel_state.direction.wheel_2 = s[1].j;
            jupiter_wheel_state.direction.wheel_3 = s[2].j;
            jupiter_wheel_state.direction.wheel_4 = s[3].j;

            // power
            jupiter_wheel_state.battery.battery_1 = receive_power_upload[0];
            jupiter_wheel_state.battery.battery_2 = receive_power_upload[1];
            jupiter_wheel_state.battery.battery_3 = receive_power_upload[2];
            jupiter_wheel_state.battery.battery_4 = receive_power_upload[3];
            jupiter_wheel_state.battery.battery_5 = receive_power_upload[4];

            jupiter_wheel_state_pub.publish(jupiter_wheel_state);

			print_uart1_data(); // 打印					
		}
    }
    return 0;
}

// 串口初始化
void serialport_init() {
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));
}

// 变量初始化
void variable_init() {
    // Initialize the struct Uart1_Control_Frame recv_bag.
    recv_bag.frame_start_flag = FRAME_START_FLAG; 
    recv_bag.frame_end_flag = FRAME_END_FLAG;  
	// const double degree = M_PI/180;
}

// 串口数据帧接收函数
unsigned char uart1_recv_frame(unsigned char *state_machine, unsigned char *frame_data_index,
					struct Uart1_Control_Frame *recv_bag, unsigned char recv_byte) {
	unsigned char frame_recv_over_flag = 0; // 帧数据接收成功标志
	switch (*state_machine) {
		case 0: // 帧头
			if (recv_byte == FRAME_START_FLAG) {
				*state_machine = 1;
			}
			break;
		case 1: // 传输码
			if (recv_byte == TRANSMISSION_CODE_OF_HW_TO_PC) {
				recv_bag->tranfer_byte = recv_byte;			
				*state_machine = 2;
			} else {
				*state_machine = 0;
			}
			break;			
		case 2: // 功能码
			if (recv_byte == FUNCTION_CODE_OF_VELOCITY_POSITION_RECEIVE 
             	|| recv_byte == FUNCTION_CODE_OF_POWER_UPLOAD 
				|| recv_byte == FUNCTION_CODE_OF_ERROR_MESSAGE_RECEIVE) {
				recv_bag->func_byte = recv_byte;
				*state_machine = 3;
			} else {
				*state_machine = 0;
			}
			break;
		case 3: // 数据长度
			recv_bag->data_size = recv_byte;
			if (recv_bag->data_size != 0) {	// 准备接收数据
				*frame_data_index = 0;    
				*state_machine = 4;
			} else {
				*state_machine = 6;  // 去判断结束符
			}	
			break;
		case 4: // 数据域
			recv_bag->frame_data[*frame_data_index] = recv_byte;
			(*frame_data_index) ++;
			if (*frame_data_index == recv_bag->data_size) { // 判断是否接收完毕 
				*state_machine = 5;
			}		
			break;
		case 5: // 校验码
			if (recv_byte == check_sum(recv_bag)) {
				*state_machine = 6;
			} else {
				*state_machine = 0;
			}
			break;
		case 6: // 帧尾
			if(recv_byte == FRAME_END_FLAG) {
				frame_recv_over_flag = 1; // 帧数据接收成功标志
			}
			*state_machine = 0;
			break;
		default:
			*state_machine = 0;
			break;	
	}	
    return frame_recv_over_flag;	
}

// 和校验函数
unsigned char check_sum(struct Uart1_Control_Frame *recv_bag) {
	unsigned char checksum = 0x00;
	checksum += recv_bag->frame_start_flag;
	checksum += recv_bag->tranfer_byte;
	checksum += recv_bag->func_byte;
	checksum += recv_bag->data_size;
	for (int i = 0; i < (recv_bag->data_size); i++) {
		checksum += recv_bag->frame_data[i];
	}
	return checksum;
}

// 接收速度、位置的转换函数
void receive_vel_pos_transform() {
	for (int i = 0; i < 4; i++) {
		v[i].c[3] = receive_vel_pos[0 + 6 * i];
		v[i].c[2] = receive_vel_pos[1 + 6 * i];
		v[i].c[1] = receive_vel_pos[2 + 6 * i];
		v[i].c[0] = receive_vel_pos[3 + 6 * i];
		s[i].c[1] = receive_vel_pos[4 + 6 * i];
		s[i].c[0] = receive_vel_pos[5 + 6 * i];
	}
}

// 打印串口接收数据函数
void print_uart1_data() {
	// 判断速度、位置数据接收是否正常并打印
    if (state[0]) {
		printf("各电机速度如下：  ");
		printf("v1:%d, v2:%d, v3:%d, v4:%d\n", v[0].i, v[1].i, v[2].i, v[3].i);
		printf("各电机方位如下：  ");
		printf("s1:%d, s2:%d, s3:%d, s4:%d\n", s[0].j, s[1].j, s[2].j, s[3].j);
    }

	// 判断电量上传数据接收是否正常并打印
	if (state[1]) {
		printf("电量情况如下：  ");
		for (int i = 0; i < 5; i++) {
			printf("%d, ", receive_power_upload[i]);
		}
		printf("\n");
	}
	
	// 判断电机运行错误数据接收是否正常并打印
	if (state[2]) {
		printf("电机运行出错！！！  ");
		for (int i = 0; i < 8; i++) {
			printf("%d, ", receive_motor_error[i]);
		}
		printf("\n");
	}
	
	// 判断电量异常数据接收是否正常并打印
	if (state[3]) {
		printf("电量异常！！！   ");
		for (int i = 0; i < 5; i++) {
			printf("%d, ", receive_power_error[i]);
		}
		printf("\n");
	}   
}

