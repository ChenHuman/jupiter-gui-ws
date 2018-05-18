/**
 * @file /include/jupiter_gui_node/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef jupiter_gui_node_QNODE_HPP_
#define jupiter_gui_node_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include <geometry_msgs/Twist.h>
#include "jupiter_serial/JupiterWheelState.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace jupiter_gui_node {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
    QStringListModel* speedLoggingModel() { return &logging_model_speed; }
    QStringListModel* directLoggingModel() { return &logging_model_direct; }
    QStringListModel* batteryLoggingModel() { return &logging_model_battery; }

	void log( const LogLevel &level, const std::string &msg);

    void jupiterStateCallback(const jupiter_serial::JupiterWheelState & msg);

Q_SIGNALS:
    void wheelSpeedFeedbackUpdated(double, double, double, double);
    void wheelDirectFeedbackUpdated(double, double, double, double);
    void batteryFeedbackUpdated();
    void rosShutdown();
    void maxSpeed(double,double,double);
    void newState(double, double, double);
    void cmdVel(double, double, double);

private:
	int init_argc;
	char** init_argv;
    ros::Publisher twist_publisher;
    ros::Subscriber jupiter_state_subscriber;
    QStringListModel logging_model;
    QStringListModel logging_model_speed;
    QStringListModel logging_model_direct;
    QStringListModel logging_model_battery;
};

}  // namespace jupiter_gui_node

#endif /* jupiter_gui_node_QNODE_HPP_ */
