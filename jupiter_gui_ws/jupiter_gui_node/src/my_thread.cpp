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

#include <std_msgs/String.h>
#include <sstream>
#include "my_thread.hpp"

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>

#include <QTime>
#include "cglobal.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace jupiter_gui_node {

/*****************************************************************************
** Implementation
*****************************************************************************/

MyThread::MyThread(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
    {}

MyThread::~MyThread() {
//    if(ros::isStarted()) {
//      ros::shutdown(); // explicitly needed since we use ros::start();
//      ros::waitForShutdown();
//    }
//    terminate();
    wait();
//    quit();
}

void MyThread::init() {
//    ros::init(init_argc,init_argv,"jupiter_gui_node");
//	if ( ! ros::master::check() ) {
//		return false;
//	}
//	ros::start(); // explicitly needed since our nodehandle is going out of scope.
//	ros::NodeHandle n;
//	// Add your ros communications here.
//    twist_publisher = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    start();
//	return true;
}

void MyThread::run() {
//    while(!ros::master::check()) {
//        log(Info,std::string("00000000"));
//    }
//    log(Info,std::string("11111111"));
    FILE *pstr;
    char cmd[128],buff[512];
    pid_t pID;
    int pidnum;
    char *name; /*进程名*/

    char *p = NULL;
    int ret=3;

    QTime t;

    switch (key_value) {
        case START_ROSCORE:
            do {
                name = "rosout";
                p = NULL;
                ret=3;
                memset(cmd,0,sizeof(cmd));
                sprintf(cmd, "ps -ef|grep %s",name);
                pstr=popen(cmd, "r");
        //        if(pstr==NULL)
        //        { return 1; }
                memset(buff,0,sizeof(buff));
                fgets(buff,512,pstr);
                printf("%s\n",buff);
                p=strtok(buff, " ");
                p=strtok(NULL, " ");
                pclose(pstr); //这句是否去掉，取决于当前系统中ps后，进程ID号是否是第一个字段
        //        if(p==NULL)
        //        { return 1; }
        //        //printf( "pid:%s\n",p);
        //        if(strlen(p)==0)
        //        { return 1; }
                if((pidnum=atoi(p))==0);
        //        { return 1; }
                printf("pidnum: %d\n",pidnum);
                pID=(pid_t)pidnum;
                ret=kill(pID,0);
                printf("ret= %d \n",ret);
                if(0==ret)
                log(Info,std::string("roscore started."));
                else
                log(Info,std::string("starting roscore......"));
                t.start();
                while(t.elapsed() < 500);
                //return 0;
            }while(ret != 0);
            quit();
            break;
        case CLOSE_ROSCORE:
            log(Info,std::string("roscore killed."));
            quit();
            break;
        case OPEN_SERIAL_PORT:
            do {
                name = "re_cmd_port";
                p = NULL;
                ret=3;
                memset(cmd,0,sizeof(cmd));
                sprintf(cmd, "ps -ef|grep %s",name);
                pstr=popen(cmd, "r");
        //        if(pstr==NULL)
        //        { return 1; }
                memset(buff,0,sizeof(buff));
                fgets(buff,512,pstr);
                printf("%s\n",buff);
                p=strtok(buff, " ");
                p=strtok(NULL, " ");
                pclose(pstr); //这句是否去掉，取决于当前系统中ps后，进程ID号是否是第一个字段
        //        if(p==NULL)
        //        { return 1; }
        //        //printf( "pid:%s\n",p);
        //        if(strlen(p)==0)
        //        { return 1; }
                if((pidnum=atoi(p))==0);
        //        { return 1; }
                printf("pidnum: %d\n",pidnum);
                pID=(pid_t)pidnum;
                ret=kill(pID,0);
                printf("ret= %d \n",ret);
                if(0==ret)
                log(Info,std::string("re_cmd_vel started."));
                else {
                    log(Info,std::string("starting re_cmd_vel......"));
                    log(Info,std::string("starting state_publisher......"));
                }
                t.start();
                while(t.elapsed() < 500);
                //return 0;
            }while(ret != 0);

            do {
                name = "state_publisher";
                p = NULL;
                ret=3;
                memset(cmd,0,sizeof(cmd));
                sprintf(cmd, "ps -ef|grep %s",name);
                pstr=popen(cmd, "r");
        //        if(pstr==NULL)
        //        { return 1; }
                memset(buff,0,sizeof(buff));
                fgets(buff,512,pstr);
                printf("%s\n",buff);
                p=strtok(buff, " ");
                p=strtok(NULL, " ");
                pclose(pstr); //这句是否去掉，取决于当前系统中ps后，进程ID号是否是第一个字段
        //        if(p==NULL)
        //        { return 1; }
        //        //printf( "pid:%s\n",p);
        //        if(strlen(p)==0)
        //        { return 1; }
                if((pidnum=atoi(p))==0);
        //        { return 1; }
                printf("pidnum: %d\n",pidnum);
                pID=(pid_t)pidnum;
                ret=kill(pID,0);
                printf("ret= %d \n",ret);
                if(0==ret)
                log(Info,std::string("state_publisher started."));
                else
                log(Info,std::string("starting state_publisher......"));
                t.start();
                while(t.elapsed() < 500);
                //return 0;
            }while(ret != 0);
            quit();
            break;
        case CLOSE_SERIAL_PORT:
            log(Info,std::string("re_cmd_vel killed."));
            log(Info,std::string("state_publisher killed."));
            quit();
            break;
    }


}

void MyThread::log( const LogLevel &level, const std::string &msg) {
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
                //logging_model_msg << "[INFO] ["  "]: " << msg;
                logging_model_msg << "neo@Jupiter:~$ " << msg;
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
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace jupiter_gui_node
