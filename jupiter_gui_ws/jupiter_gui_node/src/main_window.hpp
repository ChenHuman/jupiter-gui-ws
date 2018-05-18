/**
 * @file /include/jupiter_gui_node/main_window.hpp
 *
 * @brief Qt based gui for jupiter_gui_node.
 *
 * @date November 2010
 **/
#ifndef jupiter_gui_node_MAIN_WINDOW_H
#define jupiter_gui_node_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "my_thread.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace jupiter_gui_node {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

//    /*********************
//    ** Logging
//    **********************/
//    enum LogLevel {
//             Debug,
//             Info,
//             Warn,
//             Error,
//             Fatal
//     };

//    void log( const LogLevel &level, const std::string &msg);

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/

	void on_actionAbout_triggered();

    void on_btn_connect_clicked(bool check );
    void on_btn_open_serial_port_clicked();
    void on_btn_start_roscore_clicked();
    void on_btn_reset_clicked();
    void on_btn_brake_clicked();
    void on_btn_test_clicked();

	void on_checkbox_use_environment_stateChanged(int state);

    void on_btn_stop_pressed();
    void on_btn_stop_released();
    void on_btn_up_pressed();
    void on_btn_up_released();
    void on_btn_down_pressed();
    void on_btn_down_released();
    void on_btn_turn_left_pressed();
    void on_btn_turn_left_released();
    void on_btn_turn_right_pressed();
    void on_btn_turn_right_released();
    void on_btn_left_forward_pressed();
    void on_btn_left_forward_released();
    void on_btn_right_forward_pressed();
    void on_btn_right_forward_released();
    void on_btn_left_back_pressed();
    void on_btn_left_back_released();
    void on_btn_right_back_pressed();
    void on_btn_right_back_released();

    void on_btn_left_pressed();
    void on_btn_left_released();
    void on_btn_right_pressed();
    void on_btn_right_released();

    void on_btn_increase_max_speeds_pressed();
    void on_btn_increase_max_speeds_released();
    void on_btn_decrease_max_speeds_pressed();
    void on_btn_decrease_max_speeds_released();
    void on_btn_increase_linear_speed_pressed();
    void on_btn_increase_linear_speed_released();
    void on_btn_decrease_linear_speed_pressed();
    void on_btn_decrease_linear_speed_released();
    void on_btn_increase_angular_speed_pressed();
    void on_btn_increase_angular_speed_released();
    void on_btn_decrease_angular_speed_pressed();
    void on_btn_decrease_angular_speed_released();

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void updateMaxSpeedDisplay(double x, double y, double theta);
    void updateCmdVelDisplay(double x, double y, double theta);
    void wheelSpeedFeedbackUpdateLoggingView(double v1, double v2, double v3, double v4);
    void wheelDirectFeedbackUpdateLoggingView(double d1, double d2, double d3, double d4);
//    void batteryFeedbackUpdateLoggingView();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    MyThread my_thread;
//    QStringListModel logging_model;
};

}  // namespace jupiter_gui_node

#endif // jupiter_gui_node_MAIN_WINDOW_H
