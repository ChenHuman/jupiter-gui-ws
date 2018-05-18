#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <math.h>	


float s1,s2,s3,s4,a1,a2;

//定义传输数据长度
const unsigned char LEN=26;

//定义初始的三个变量，x速度、y速度及z角度
double linear_y,linear_x,angular_z,radius;

//定义一个命名空间，用于后面的write函数
using namespace boost::asio;

//定义传输的io口
io_service io;
serial_port sp(io, "/dev/ttyUSB0");
//serial_port sp(io, "/dev/pts/20");
//定义速度接收→发送程序
void commandReceived (const geometry_msgs::Twist& msgIn);

//定义一个传输数组
unsigned char transform[32];

//定义一个联合体（32位的int，8位的unsigned char），用于数据转换
union velocity
{
   int i;
   unsigned char c[4];
}v[4];

unsigned char len=0;//数据长度

//再定义一个联合体（16位的short int，8位的unsigned char），用于数据转换
union direction
{
   short int i;
   unsigned char c[2];
}d[4];

//将int(32位)的四个轮子速度转换为四个8位的unsigned char类型，并赋予传输数组
void VD2transform()
{
	unsigned char n=4;
	for(unsigned char i = 0; i < 4; i++)
	{
		transform[n] = v[i].c[3];n++;
		transform[n] = v[i].c[2];n++;
		transform[n] = v[i].c[1];n++;
		transform[n] = v[i].c[0];n++;
		transform[n] = d[i].c[1];n++;
		transform[n] = d[i].c[0];n++;
	}
}

//校验和
char checksum(unsigned char *s,char len)
{
	unsigned char *p;
	p=s;
	unsigned char sum;
	for(sum = 0; len > 0; len--)
	{
	sum = sum + (*p);
	p++;
	}
	return sum;
}

std_msgs::Float64 d_joint1, d_joint2, d_joint3, d_joint4;
std_msgs::Float64 v_wheel1, v_wheel2, v_wheel3, v_wheel4;

int main(int argc, char **argv)
{
	//初始化ros
	ros::init(argc, argv, "re_cmd_port");

	ros::NodeHandle nh_pub_joint1, nh_pub_joint2, nh_pub_joint3, nh_pub_joint4;
	ros::NodeHandle nh_pub_wheel1, nh_pub_wheel2, nh_pub_wheel3, nh_pub_wheel4;

	ros::Publisher pub_joint1 = nh_pub_joint1.advertise<std_msgs::Float64>("axis1_position_controller/command", 1000);
	ros::Publisher pub_joint2 = nh_pub_joint2.advertise<std_msgs::Float64>("axis2_position_controller/command", 1000);
	ros::Publisher pub_joint3 = nh_pub_joint3.advertise<std_msgs::Float64>("axis3_position_controller/command", 1000);
	ros::Publisher pub_joint4 = nh_pub_joint4.advertise<std_msgs::Float64>("axis4_position_controller/command", 1000);

	ros::Publisher pub_wheel1 = nh_pub_wheel1.advertise<std_msgs::Float64>("wheel1_velocity_controller/command", 1000);
	ros::Publisher pub_wheel2 = nh_pub_wheel2.advertise<std_msgs::Float64>("wheel2_velocity_controller/command", 1000);
	ros::Publisher pub_wheel3 = nh_pub_wheel3.advertise<std_msgs::Float64>("wheel3_velocity_controller/command", 1000);
	ros::Publisher pub_wheel4 = nh_pub_wheel4.advertise<std_msgs::Float64>("wheel4_velocity_controller/command", 1000);

	ros::Rate loop_rate(20);
	ros::NodeHandle nh;

	ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	//数据长度
	len=LEN;

	//传输数据帧头帧尾及数据长度
	transform[0]='s';//针头
	transform[1]=0x11;//传输码 PC2HW
	transform[2]=0x01;//功能标志位
	transform[3]=len;//数据长度

	transform[31]='e';

	//串口设置（波特率115200,1个停止位，7个数据位） 
	sp.set_option(serial_port::baud_rate(115200));
	sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
	sp.set_option(serial_port::parity(serial_port::parity::none));
	sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
	sp.set_option(serial_port::character_size(8));

	//调用速度接收→发送程序
	ros::Subscriber sub = nh.subscribe("/cmd_vel",1000,&commandReceived) ;
	while(nh.ok())
	{
		pub_joint1.publish(d_joint1);
		pub_joint2.publish(d_joint2);
		pub_joint3.publish(d_joint3);
		pub_joint4.publish(d_joint4);

		pub_wheel1.publish(v_wheel1);
		pub_wheel2.publish(v_wheel2);
		pub_wheel3.publish(v_wheel3);
		pub_wheel4.publish(v_wheel4);
		ros::spinOnce () ;
		loop_rate.sleep();
	}
	sp.close ();
	return 0;
}


void commandReceived (const geometry_msgs::Twist& msgIn)
{
 	const double epsilon = 0.0001; // 数学上常用ε表示任意小的数
	
	//速度由m/s转换到mm/s
	linear_x = msgIn.linear.x*1000;
	linear_y = msgIn.linear.y*1000;
	angular_z =msgIn.angular.z;
	
	radius = abs(angular_z) < epsilon ? 0.0 : linear_x / angular_z;

	printf("linear_x:%f  angular_z:%f\n",msgIn.linear.x,msgIn.angular.z);
	
	/*
	以下为Jupiter大车运动学模型推算公式转换过程，具体情况可参考Jupiter大车运动模型分析文件
	*/
    if ( fabs(msgIn.linear.y) > 600 ) {    // 自定义为锁死状态
        // 发送到底层硬件大车的速度和方向数据
        v[0].i=v[1].i=v[2].i=v[3].i = (int) 0;
        d[0].i = (short int) 45 * 100;
        d[1].i = (short int) -45 * 100;
        d[2].i = d[0].i;
        d[3].i = d[1].i;
        // 发送到仿真大车模型的速度和方向数据
        v_wheel1.data=v_wheel2.data=v_wheel3.data=v_wheel4.data = v[0].i;
        d_joint1.data = M_PI / 4;
        d_joint2.data = -M_PI / 4;
        d_joint3.data = d_joint1.data;
        d_joint4.data = d_joint2.data;
    } else if ( fabs(msgIn.linear.z) > epsilon ) {    // 自定义为过零点情况
        // 发送到底层硬件大车的速度和方向数据
        v[0].i=v[1].i=v[2].i=v[3].i = (int) 0;
        d[0].i=d[1].i=d[2].i=d[3].i = (short int) (msgIn.linear.z * 100);

        // 发送到仿真大车模型的速度和方向数据
        v_wheel1.data=v_wheel2.data=v_wheel3.data=v_wheel4.data = 0;
        d_joint1.data=d_joint2.data=d_joint3.data=d_joint4.data = msgIn.linear.z * M_PI / 180;
	} else if ( fabs(linear_y) > epsilon && fabs(linear_x) < epsilon && fabs(angular_z) < epsilon ) {    // 横着走
		// 发送到底层硬件大车的速度和方向数据
        v[0].i=v[1].i=v[2].i=v[3].i = (int) (linear_y / 152.4 * 180 / M_PI * 100);
        d[0].i=d[1].i=d[2].i=d[3].i = (short int) (atan2(linear_y/fabs(linear_y), 0) / M_PI * 180 * 100);

		// 发送到仿真大车模型的速度和方向数据
        v_wheel1.data=v_wheel2.data=v_wheel3.data=v_wheel4.data = (int) (linear_y / 152.4);
        d_joint1.data=d_joint2.data=d_joint3.data=d_joint4.data = atan2(linear_y/fabs(linear_y), 0);
	} else if ( fabs(linear_y) < epsilon ) {    // 非横着走
		if ( fabs(angular_z) < epsilon ) {    // 角速度为0, 大车直线前进、直线后退
		// 发送到底层硬件大车的速度和方向数据
        v[0].i=v[1].i=v[2].i=v[3].i = (int) (linear_x / 152.4 * 180 / M_PI * 100);
		d[0].i=d[1].i=d[2].i=d[3].i = (short int) 0;

		// 发送到仿真大车模型的速度和方向数据
        v_wheel1.data=v_wheel2.data=v_wheel3.data=v_wheel4.data = (int) (linear_x / 152.4);
		d_joint1.data=d_joint2.data=d_joint3.data=d_joint4.data = 0.0;
		
    } else if ( abs(radius) <= 780.0/2 ) {    // 旋转中心在大车内部
		// 发送到底层硬件大车的速度和方向数据
        v[0].i=v[3].i = (int) (-angular_z * sqrt((780.0/2 - radius) * (780.0/2 - radius) + 1030/2 * 1030/2) / 152.4 * 180 / M_PI * 100); // 1、4车轮角速度
        v[1].i=v[2].i = (int) (angular_z * sqrt((780.0/2 + radius) * (780.0/2 + radius) + 1030/2 * 1030/2) / 152.4 * 180 / M_PI * 100); // 2、3车轮角速度
        d[0].i = (short int) (atan2(-1030.0/2, 780.0/2 - radius) / M_PI * 180 * 100); // 将弧度转化为角度，下同
        d[1].i = (short int) (atan2(1030.0/2, 780.0/2 + radius) / M_PI * 180 * 100);
		d[2].i = -d[1].i;
		d[3].i = -d[0].i;
	
		// 发送到仿真大车模型的速度和方向数据
        v_wheel1.data=v_wheel4.data = (int) (-angular_z * sqrt((780.0/2 - radius) * (780.0/2 - radius) + 1030/2 * 1030/2) / 152.4);
        v_wheel2.data=v_wheel3.data = (int) (angular_z * sqrt((780.0/2 + radius) * (780.0/2 + radius) + 1030/2 * 1030/2) / 152.4);
		d_joint1.data = atan2(-1030.0/2, 780.0/2 - radius);
		d_joint2.data = atan2(1030.0/2, 780.0/2 + radius);
		d_joint3.data = -d_joint2.data;
		d_joint4.data = -d_joint1.data;
    } else {    // 旋转中心在大车外部
		// 发送到底层硬件大车的速度和方向数据
        v[0].i=v[3].i = (int) ((radius/fabs(radius)) * angular_z * sqrt((radius - 780.0/2) * (radius - 780.0/2) + 1030/2 * 1030/2) / 152.4 * 180 / M_PI * 100); // 1、4车轮角速度
        v[1].i=v[2].i = (int) ((radius/fabs(radius)) * angular_z * sqrt((radius + 780.0/2) * (radius + 780.0/2) + 1030/2 * 1030/2) / 152.4 * 180 / M_PI * 100); // 2、3车轮角速度
        d[0].i = (short int) (atan2((radius/fabs(radius)) * 1030.0/2, (radius/fabs(radius)) * (radius - 780.0/2)) / M_PI * 180 * 100); // 将弧度转化为角度，下同
        d[1].i = (short int) (atan2((radius/fabs(radius)) * 1030.0/2, (radius/fabs(radius)) * (radius + 780.0/2)) / M_PI * 180 * 100);
		d[2].i = -d[1].i;
		d[3].i = -d[0].i;
	
		// 发送到仿真大车模型的速度和方向数据
        v_wheel1.data=v_wheel4.data = (int) ((radius/fabs(radius)) * angular_z * sqrt((radius - 780.0/2) * (radius - 780.0/2) + 1030/2 * 1030/2) / 152.4);
        v_wheel2.data=v_wheel3.data = (int) ((radius/fabs(radius)) * angular_z * sqrt((radius + 780.0/2) * (radius + 780.0/2) + 1030/2 * 1030/2) / 152.4);
        d_joint1.data = atan2((radius/fabs(radius)) * 1030.0/2, (radius/fabs(radius)) * (radius - 780.0/2));
        d_joint2.data = atan2((radius/fabs(radius)) * 1030.0/2, (radius/fabs(radius)) * (radius + 780.0/2));
		d_joint3.data = -d_joint2.data;
		d_joint4.data = -d_joint1.data;
    }

	transform[30]=checksum(transform,4+len);//校验和，长度为前四位加数据域长度

	printf("v:%d,%d,%d,%d\nd:%d,%d,%d,%d\n",v[0].i,v[1].i,v[2].i,v[3].i,d[0].i,d[1].i,d[2].i,d[3].i);
	printf("******************************\n");

	//将int(32位)的四个轮子速度转换为四个8位的char类型，并赋予传输数组
	VD2transform();

	//把char类型的transform数组（32位）通过串口写
	write(sp, buffer(transform, 32));
}
