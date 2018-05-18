/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "main_window.hpp"

#include "cglobal.hpp"
#include <QDebug>

static bool flag_start_roscore = true;
static bool flag_connect = true;
static bool flag_open_serial_port = true;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace jupiter_gui_node {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
    , my_thread(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    setWindowIcon(QIcon(":/images/jupiter.png"));
    setWindowTitle("Jupiter");
    ui.tab_jupiter_teleop_panel->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    ui.dock_status->setShown(true);
    ui.dock_status->setFeatures(QDockWidget::NoDockWidgetFeatures);

    QObject::connect(&qnode, SIGNAL(maxSpeed(double,double,double)), this, SLOT(updateMaxSpeedDisplay(double,double,double)));
    QObject::connect(&qnode, SIGNAL(cmdVel(double,double,double)), this, SLOT(updateCmdVelDisplay(double,double,double)));
    //QObject::connect(&qnode, SIGNAL(newState(double,double,double)), this, SLOT(updateMaxSpeedDisplay(double,double,double)));
    QObject::connect(&qnode, SIGNAL(wheelSpeedFeedbackUpdated(double, double, double, double)), this, SLOT(wheelSpeedFeedbackUpdateLoggingView(double, double, double, double)));
    QObject::connect(&qnode, SIGNAL(wheelDirectFeedbackUpdated(double, double, double, double)), this, SLOT(wheelDirectFeedbackUpdateLoggingView(double, double, double, double)));
	/*********************
	** Logging
	**********************/
    ui.lv_logging->setModel(my_thread.loggingModel());
    QObject::connect(&my_thread, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
   // ui.lv_wheel_speed_feedback->setModel(qnode.speedLoggingModel());
    //QObject::connect(&qnode, SIGNAL(wheelSpeedFeedbackUpdated(double, double, double, double)), this, SLOT(wheelSpeedFeedbackUpdateLoggingView(double v1, double v2, double v3, double v4)));
//    ui.lv_wheel_direct_feedback->setModel(qnode.directLoggingModel());
//    QObject::connect(&qnode, SIGNAL(wheelDirectFeedbackUpdated()), this, SLOT(wheelDirectFeedbackUpdateLoggingView()));
//    ui.lv_battery_feedback->setModel(qnode.batteryLoggingModel());
//    QObject::connect(&qnode, SIGNAL(batteryFeedbackUpdated()), this, SLOT(batteryFeedbackUpdateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
//    if ( ui.checkbox_remember_settings->isChecked() ) {
//        on_button_connect_clicked(true);
//    }

//    ui.line_edit_master->setEnabled(false);
//    ui.line_edit_host->setEnabled(false);
    ui.checkbox_use_environment->setChecked(true);
    ui.checkbox_use_environment->setEnabled(false);
//    ui.checkbox_remember_settings->setEnabled(false);

    ui.btn_stop->setIcon(QIcon(":/images/stop.png"));
    ui.btn_stop->setIconSize(QSize(100, 100));
    ui.btn_up->setIcon(QIcon(":/images/up.png"));
    ui.btn_up->setIconSize(QSize(70, 70));
    ui.btn_down->setIcon(QIcon(":/images/down.png"));
    ui.btn_down->setIconSize(QSize(70, 70));
    ui.btn_turn_left->setIcon(QIcon(":/images/turn_left.png"));
    ui.btn_turn_left->setIconSize(QSize(60, 60));
    ui.btn_turn_right->setIcon(QIcon(":/images/turn_right.png"));
    ui.btn_turn_right->setIconSize(QSize(60, 60));
    ui.btn_left_forward->setIcon(QIcon(":/images/left_forward.png"));
    ui.btn_left_forward->setIconSize(QSize(80, 80));
    ui.btn_right_forward->setIcon(QIcon(":/images/right_forward.png"));
    ui.btn_right_forward->setIconSize(QSize(80, 80));
    ui.btn_left_back->setIcon(QIcon(":/images/left_back.png"));
    ui.btn_left_back->setIconSize(QSize(80, 80));
    ui.btn_right_back->setIcon(QIcon(":/images/right_back.png"));
    ui.btn_right_back->setIconSize(QSize(80, 80));

    ui.btn_left->setIcon(QIcon(":/images/left.png"));
    ui.btn_left->setIconSize(QSize(70, 70));
    ui.btn_right->setIcon(QIcon(":/images/right.png"));
    ui.btn_right->setIconSize(QSize(70, 70));
    ui.btn_increase_max_speeds->setIcon(QIcon(":/images/increase_max_speeds.png"));
    ui.btn_increase_max_speeds->setIconSize(QSize(120, 80));
    ui.btn_decrease_max_speeds->setIcon(QIcon(":/images/decrease_max_speeds.png"));
    ui.btn_decrease_max_speeds->setIconSize(QSize(120, 80));
    ui.btn_increase_linear_speed->setIcon(QIcon(":/images/increase_linear_speed.png"));
    ui.btn_increase_linear_speed->setIconSize(QSize(120, 60));
    ui.btn_decrease_linear_speed->setIcon(QIcon(":/images/decrease_linear_speed.png"));
    ui.btn_decrease_linear_speed->setIconSize(QSize(120, 65));
    ui.btn_increase_angular_speed->setIcon(QIcon(":/images/increase_angular_speed.png"));
    ui.btn_increase_angular_speed->setIconSize(QSize(120, 60));
    ui.btn_decrease_angular_speed->setIcon(QIcon(":/images/decrease_angular_speed.png"));
    ui.btn_decrease_angular_speed->setIconSize(QSize(120, 60));

    ui.btn_brake->setIcon(QIcon(":/images/brake.png"));
    ui.btn_brake->setIconSize(QSize(281, 121));

    ui.lineEdit_max_linear_x->setText("0.2");
    ui.lineEdit_max_angular_z->setText("1");
    ui.lineEdit_max_linear_y->setText("0.2");

    ui.lineEdit_v1->setText("0");
    ui.lineEdit_v2->setText("0");
    ui.lineEdit_v3->setText("0");
    ui.lineEdit_v4->setText("0");

    ui.lineEdit_d1->setText("0");
    ui.lineEdit_d2->setText("0");
    ui.lineEdit_d3->setText("0");
    ui.lineEdit_d4->setText("0");

    ui.btn_start_roscore->setEnabled(true);
    ui.btn_connect->setEnabled(false);
    ui.btn_open_serial_port->setEnabled(false);
    ui.btn_reset->setEnabled(false);
    ui.btn_test->setEnabled(false);

    ui.btn_start_roscore->setStyleSheet("background-color: rgb(255,255,0);");
    ui.btn_connect->setStyleSheet("background-color: rgb(255,255,0);");
    ui.btn_open_serial_port->setStyleSheet("background-color: rgb(202,235,216);");
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    //close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_btn_connect_clicked(bool check ) {
    if (flag_connect) {
        if (ui.checkbox_use_environment->isChecked() ) {
            if ( !qnode.init("http://127.0.0.1:11311/", "127.0.0.1") ) {
                showNoMasterMessage();
            } else {
                ui.btn_connect->setStyleSheet("background-color: rgb(255,0,0);");
                ui.btn_connect->setText("Close GUI Teleop");
                teleop_mode = GUI_TELEOP;
                ui.btn_reset->setEnabled(true);
                ui.btn_test->setEnabled(false);
                flag_connect = false;
            }
        }
    } else {
        ui.btn_connect->setStyleSheet("background-color: rgb(255,255,0);");
        ui.btn_connect->setText("Open GUI Teleop");
        teleop_mode = NO_TELEOP;
        key_value = NO_OPERATION;
        ui.btn_test->setEnabled(true);
        ui.btn_reset->setEnabled(false);
        flag_connect = true;
    }
//	if ( ui.checkbox_use_environment->isChecked() ) {
//        if ( !qnode.init("http://127.0.0.1:11311/", "127.0.0.1") ) {
//			showNoMasterMessage();
//		} else {
//            ui.btn_connect->setText("disconnect");
//		}
//    }
//    else {
//		if ( ! qnode.init("http://192.168.1.2:11311/",
//				   "192.168.1.3") ) {
//			showNoMasterMessage();
//		} else {
//			ui.button_connect->setEnabled(false);
//			ui.line_edit_master->setReadOnly(true);
//			ui.line_edit_host->setReadOnly(true);
//			ui.line_edit_topic->setReadOnly(true);
//		}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
    /*bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
    ui.line_edit_host->setEnabled(enabled);*/
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.lv_logging->scrollToBottom();
}


//void MainWindow::batteryFeedbackUpdateLoggingView() {
//        ui.lv_battery_feedback->scrollToBottom();
//}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "jupiter_gui_node");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
//    ui.line_edit_master->setText(master_url);
//    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
//    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
//    	ui.line_edit_master->setEnabled(false);
//    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "jupiter_gui_node");
//    settings.setValue("master_url",ui.line_edit_master->text());
//    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
//    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
    QMainWindow::closeEvent(event);
    system("bash -c 'source /opt/ros/indigo/setup.bash;"
           "rosnode kill re_cmd_port state_publisher'&");
}

}  // namespace jupiter_gui_node

void jupiter_gui_node::MainWindow::on_btn_stop_pressed()
{
    key_value = STOP;
}

void jupiter_gui_node::MainWindow::on_btn_stop_released()
{
    key_value = NO_OPERATION;
}

void jupiter_gui_node::MainWindow::on_btn_up_pressed()
{
    key_value = UP;
}

void jupiter_gui_node::MainWindow::on_btn_up_released()
{
    key_value = NO_OPERATION;
}

void jupiter_gui_node::MainWindow::on_btn_down_pressed()
{
    key_value = DOWN;
}

void jupiter_gui_node::MainWindow::on_btn_down_released()
{
    key_value = NO_OPERATION;
}

void jupiter_gui_node::MainWindow::on_btn_turn_left_pressed()
{
    key_value = TURN_LEFT;
}

void jupiter_gui_node::MainWindow::on_btn_turn_left_released()
{
    key_value = NO_OPERATION;
}

void jupiter_gui_node::MainWindow::on_btn_turn_right_pressed()
{
    key_value = TURN_RIGHT;
}

void jupiter_gui_node::MainWindow::on_btn_turn_right_released()
{
    key_value = NO_OPERATION;
}

void jupiter_gui_node::MainWindow::on_btn_left_forward_pressed()
{
    key_value = LEFT_FORWARD;
}

void jupiter_gui_node::MainWindow::on_btn_left_forward_released()
{
    key_value = NO_OPERATION;
}

void jupiter_gui_node::MainWindow::on_btn_right_forward_pressed()
{
    key_value = RIGHT_FORWARD;
}

void jupiter_gui_node::MainWindow::on_btn_right_forward_released()
{
    key_value = NO_OPERATION;
}

void jupiter_gui_node::MainWindow::on_btn_left_back_pressed()
{
    key_value = LEFT_BACK;
}

void jupiter_gui_node::MainWindow::on_btn_left_back_released()
{
    key_value = NO_OPERATION;
}


void jupiter_gui_node::MainWindow::on_btn_right_back_pressed()
{
    key_value = RIGHT_BACK;
}

void jupiter_gui_node::MainWindow::on_btn_right_back_released()
{
    key_value = NO_OPERATION;
}

void jupiter_gui_node::MainWindow::on_btn_left_pressed()
{
    key_value = LEFT;
}

void jupiter_gui_node::MainWindow::on_btn_left_released()
{
    key_value = NO_OPERATION;
}

void jupiter_gui_node::MainWindow::on_btn_right_pressed()
{
    key_value = RIGHT;
}

void jupiter_gui_node::MainWindow::on_btn_right_released()
{
    key_value = NO_OPERATION;
}

void jupiter_gui_node::MainWindow::on_btn_increase_max_speeds_pressed()
{
    key_value = INCREASE_MAX_SPEEDS;
}

void jupiter_gui_node::MainWindow::on_btn_increase_max_speeds_released()
{
    key_value = NO_OPERATION;
}

void jupiter_gui_node::MainWindow::on_btn_decrease_max_speeds_pressed()
{
    key_value = DECREASE_MAX_SPEEDS;
}

void jupiter_gui_node::MainWindow::on_btn_decrease_max_speeds_released()
{
    key_value = NO_OPERATION;
}

void jupiter_gui_node::MainWindow::on_btn_increase_linear_speed_pressed()
{
    key_value = INCREASE_LINEAR_SPEED;
}

void jupiter_gui_node::MainWindow::on_btn_increase_linear_speed_released()
{
    key_value = NO_OPERATION;
}

void jupiter_gui_node::MainWindow::on_btn_decrease_linear_speed_pressed()
{
    key_value = DECREASE_LINEAR_SPEED;
}

void jupiter_gui_node::MainWindow::on_btn_decrease_linear_speed_released()
{
    key_value = NO_OPERATION;
}

void jupiter_gui_node::MainWindow::on_btn_increase_angular_speed_pressed()
{
    key_value = INCREASE_ANGULAR_SPEED;
}

void jupiter_gui_node::MainWindow::on_btn_increase_angular_speed_released()
{
    key_value = NO_OPERATION;
}

void jupiter_gui_node::MainWindow::on_btn_decrease_angular_speed_pressed()
{
    key_value = DECREASE_ANGULAR_SPEED;
}

void jupiter_gui_node::MainWindow::on_btn_decrease_angular_speed_released()
{
    key_value = NO_OPERATION;
}

void jupiter_gui_node::MainWindow::wheelSpeedFeedbackUpdateLoggingView(double v1, double v2, double v3, double v4) {
    QString v_1, v_2, v_3, v_4;
    v_1.setNum(v1);
    v_2.setNum(v2);
    v_3.setNum(v3);
    v_4.setNum(v4);

    ui.lineEdit_v1->setText(v_1);
    ui.lineEdit_v2->setText(v_2);
    ui.lineEdit_v3->setText(v_3);
    ui.lineEdit_v4->setText(v_4);
}

void jupiter_gui_node::MainWindow::wheelDirectFeedbackUpdateLoggingView(double d1, double d2, double d3, double d4) {
    QString d_1, d_2, d_3, d_4;
    d_1.setNum(d1);
    d_2.setNum(d2);
    d_3.setNum(d3);
    d_4.setNum(d4);

    ui.lineEdit_d1->setText(d_1);
    ui.lineEdit_d2->setText(d_2);
    ui.lineEdit_d3->setText(d_3);
    ui.lineEdit_d4->setText(d_4);
}

void jupiter_gui_node::MainWindow::updateMaxSpeedDisplay(double x, double y, double theta)
{
    QString xSpeed, ySpeed, zSpeed;
    xSpeed.setNum(x);
    ySpeed.setNum(y);
    zSpeed.setNum(theta);

    ui.lineEdit_max_linear_x->setText(xSpeed);
    ui.lineEdit_max_linear_y->setText(ySpeed);
    ui.lineEdit_max_angular_z->setText(zSpeed);
}

void jupiter_gui_node::MainWindow::updateCmdVelDisplay(double x, double y, double theta)
{
    QString xSpeed, ySpeed, zSpeed;
    xSpeed.setNum(x);
    ySpeed.setNum(y);
    zSpeed.setNum(theta);

    ui.lineEdit_cur_linear_x->setText(xSpeed);
    ui.lineEdit_cur_linear_y->setText(ySpeed);
    ui.lineEdit_cur_angular_z->setText(zSpeed);
}

void jupiter_gui_node::MainWindow::on_btn_open_serial_port_clicked()
{
    if (flag_open_serial_port) {
        ui.btn_open_serial_port->setStyleSheet("background-color: rgb(255,0,255);");
        ui.btn_open_serial_port->setText("Close Serial Port");
        system("bash -c 'source ~/jupiter_gui_ws/devel/setup.bash;"
               "roslaunch jupiter_serial jupiter_serial.launch;limited:=true'&");
        my_thread.init();
        key_value = OPEN_SERIAL_PORT;
        ui.btn_connect->setEnabled(true);
        ui.btn_test->setEnabled(true);
        flag_open_serial_port = false;
    } else {
        ui.btn_open_serial_port->setText("Open Serial Port");
        ui.btn_open_serial_port->setStyleSheet("background-color: rgb(202,235,216);");
        system("bash -c 'source /opt/ros/indigo/setup.bash;"
               "rosnode kill re_cmd_port state_publisher'&");
        my_thread.init();
        key_value = CLOSE_SERIAL_PORT;

        ui.btn_connect->setStyleSheet("background-color: rgb(255,255,0);");
        ui.btn_connect->setText("Open GUI Teleop");
        teleop_mode = NO_TELEOP;
        key_value = NO_OPERATION;
        flag_connect = true;

        ui.btn_connect->setEnabled(false);
        ui.btn_test->setEnabled(false);

        flag_open_serial_port = true;
    }
}

void jupiter_gui_node::MainWindow::on_btn_start_roscore_clicked()
{
    if (flag_start_roscore) {
        ui.btn_start_roscore->setStyleSheet("background-color: rgb(255,0,0);");
        ui.btn_start_roscore->setText("close roscore");//my_thread.init();
        system("bash -c 'source /opt/ros/indigo/setup.bash;roscore;'&");
        my_thread.init();
        key_value = START_ROSCORE;
        ui.btn_open_serial_port->setEnabled(true);
        flag_start_roscore = false;
    } else {
        ui.btn_start_roscore->setStyleSheet("background-color: rgb(255,255,0);");
        ui.btn_start_roscore->setText("start roscore");
//        system("bash -c 'killall roslaunch; killall roscore'&");
        my_thread.init();
        key_value = CLOSE_ROSCORE;
        flag_start_roscore = true;

        ui.btn_open_serial_port->setText("Open Serial Port");
        ui.btn_open_serial_port->setStyleSheet("background-color: rgb(202,235,216);");
        system("bash -c 'source /opt/ros/indigo/setup.bash;"
               "rosnode kill re_cmd_port state_publisher'&");
        my_thread.init();
        key_value = CLOSE_SERIAL_PORT;
        flag_open_serial_port = true;

        ui.btn_connect->setStyleSheet("background-color: rgb(255,255,0);");
        ui.btn_connect->setText("Open GUI Teleop");
        teleop_mode = NO_TELEOP;
        key_value = NO_OPERATION;
        flag_connect = true;

        ui.btn_connect->setEnabled(false);
        ui.btn_open_serial_port->setEnabled(false);
        ui.btn_reset->setEnabled(false);
        ui.btn_test->setEnabled(false);

    }
}


void jupiter_gui_node::MainWindow::on_btn_reset_clicked()
{
    key_value = RESET;
}

void jupiter_gui_node::MainWindow::on_btn_brake_clicked()
{
    key_value = BRAKE;
}

void jupiter_gui_node::MainWindow::on_btn_test_clicked()
{
    system("gnome-terminal -x bash -c 'source ~/jupiter_gui_ws/devel/setup.bash;"
           "roslaunch jupiter_serial keyboard_teleop.launch;limited:=true'&");
    teleop_mode = KEYBOARD_TELEOP;

    ui.btn_connect->setStyleSheet("background-color: rgb(255,255,0);");
    ui.btn_connect->setText("Open GUI Teleop");
    teleop_mode = NO_TELEOP;
    flag_connect = true;

    ui.btn_reset->setEnabled(false);
    key_value = NO_OPERATION;
}
