#ifndef BEBOP_CONTROL_GUI_H
#define BEBOP_CONTROL_GUI_H

#include <QWidget>
#include <QString>
#include <QKeyEvent>
#include <QAbstractButton>
#include <QPushButton>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <bebop_msgs/CommonCommonStateBatteryStateChanged.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ui_bebop_control_gui.h"

#define BLOCK_WIDTH 0.9
#define BLOCK_HEIGHT 0.9
static double p_linear = 0.0, p_angular = 0.0, d_linear = 0.0, d_angular = 0.0;

static double vel = 1.0;
static double camera_vel = 1.0;
class bebop_command;
class round_thread;

class bebop_control_gui : public QWidget
{
    Q_OBJECT

public slots:


private slots:
    void arrivePosition(QAbstractButton *button);
    void closeWindowSlot();
    void startRound(QAbstractButton *button);

signals:
    void closeWindowSignal();

public:
    explicit bebop_control_gui(QWidget *parent = 0);
    ~bebop_control_gui();
    void closeWindow();
    void setGoal(double x, double y, double z, double w);

    bebop_command* bebop_cmd;
    round_thread* roundThread;
    nav_msgs::Odometry odom_end;
    nav_msgs::Odometry odom_initial;
    bool isPosition;
    bool isFind;
    bool isAuto;
    int chooseOption;

protected:
    // keyboard control.... this is the only way i managed to do this...
    void keyPressEvent( QKeyEvent * key);
    void keyReleaseEvent( QKeyEvent * key);

private:
    Ui::bebop_control_gui_Class ui;

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    ros::Subscriber battery_sub;
    ros::Subscriber odom_sub;

    cv::Mat conversion_mat_;
    cv::Mat camera_matrix;
    cv::Mat distCo;

    geometry_msgs::Twist cmdSend;
    geometry_msgs::Twist cameraSend;

    //tf::TransformBroadcaster tfBroadcaster;
    tf::TransformListener tfListener;

    void showImage(const sensor_msgs::ImageConstPtr& image_msgs);
    void showBattery(const bebop_msgs::CommonCommonStateBatteryStateChanged battery_msgs);
    void showOdom(const nav_msgs::Odometry odom_msgs);
};

#endif // BEBOP_CONTROL_GUI
