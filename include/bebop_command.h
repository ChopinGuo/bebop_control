#ifndef BEBOP_COMMAND_H
#define BEBOP_COMMAND_H

#include <cmath>
#include <QString>
#include <tf/tf.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include <opencv2/opencv.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#define PI 3.1415926

class bebop_command
{
public:
    bebop_command();
    ~bebop_command();

    //nav_msgs::Odometry* odom_initial;
    double distance;

    void sendTakeoff();
    void sendLand();
    void sendFlatTrim();
    void sendReset();
    void publishCommand(geometry_msgs::Twist cmdT);
    geometry_msgs::Twist setCommand(double roll, double pitch, double yaw, double z, double cay);
    void publishCamera(geometry_msgs::Twist cmdT);

    bool arrivePosition(nav_msgs::Odometry odom_start, nav_msgs::Odometry odom_destination);
    double getDistance(nav_msgs::Odometry odom_start, nav_msgs::Odometry odom_destination);
    double getIncludedAngle(nav_msgs::Odometry odom_start, nav_msgs::Odometry odom_destination);
    void FindQuads(cv::Mat &in_image, std::vector<std::vector<cv::Point> > &quads_out);

    void add_frame(cv::Mat &r,cv::Mat &t);
    void get_transform(cv::Mat &r, cv::Mat &t);
    void listen_once();

private:
    ros::NodeHandle nh_;

    // ros stuff
    ros::Publisher takeoff_pub;
    ros::Publisher land_pub;
    ros::Publisher flattrim_pub;
    ros::Publisher reset_pub;
    ros::Publisher command_pub;
    ros::Publisher camera_pub;

    tf::TransformListener tflistener;
    tf::TransformBroadcaster tfbroadcaster;
    tf::StampedTransform mytransform;

    static pthread_mutex_t send_CS;
};
#endif //BEBOP_COMMAND_H
