#ifndef jupiter_gui_node_MY_THREAD_HPP_
#define jupiter_gui_node_MY_THREAD_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <QThread>
#include <QStringListModel>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace jupiter_gui_node {

/*****************************************************************************
** Class
*****************************************************************************/

class MyThread : public QThread {
    Q_OBJECT
public:
    MyThread(int argc, char** argv );
    virtual ~MyThread();
    void init();
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
    void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
    void loggingUpdated();

private:
    int init_argc;
    char** init_argv;
    QStringListModel logging_model;
};

}  // namespace jupiter_gui_node

#endif /* jupiter_gui_node_MY_THREAD_HPP_ */
