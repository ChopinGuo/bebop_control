#include "bebop_command.h"

pthread_mutex_t bebop_command::send_CS = PTHREAD_MUTEX_INITIALIZER;
bebop_command::bebop_command()
{
    distance = 0;
    takeoff_pub  = nh_.advertise<std_msgs::Empty>("bebop/takeoff",1);
    land_pub	 = nh_.advertise<std_msgs::Empty>("bebop/land",1);
    flattrim_pub = nh_.advertise<std_msgs::Empty>("bebop/flattrim",1);
    reset_pub    = nh_.advertise<std_msgs::Empty>("bebop/reset",1);
    command_pub  = nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1);
    camera_pub   = nh_.advertise<geometry_msgs::Twist>("/bebop/camera_control",1);
}

bebop_command::~bebop_command(void)
{

}

geometry_msgs::Twist bebop_command::setCommand(double roll=0.0, double pitch=0.0, double yaw=0.0, double z=0.0, double cay=0.0)
{
    geometry_msgs::Twist cmd;
    cmd.linear.x = pitch;
    cmd.linear.y = roll;
    cmd.linear.z = z;
    cmd.angular.z = yaw;
    cmd.angular.y = cay;
    return cmd;
}

void bebop_command::publishCommand(geometry_msgs::Twist cmdT)
{
    pthread_mutex_lock(&send_CS);
    command_pub.publish(cmdT);
    pthread_mutex_unlock(&send_CS);
}

void bebop_command::publishCamera(geometry_msgs::Twist cmdT)
{
    pthread_mutex_lock(&send_CS);
    camera_pub.publish(cmdT);
    pthread_mutex_unlock(&send_CS);
}

void bebop_command::sendLand()
{
    pthread_mutex_lock(&send_CS);
    land_pub.publish(std_msgs::Empty());
    pthread_mutex_unlock(&send_CS);
}
void bebop_command::sendTakeoff()
{
    pthread_mutex_lock(&send_CS);
    takeoff_pub.publish(std_msgs::Empty());
    pthread_mutex_unlock(&send_CS);
}

void bebop_command::sendFlatTrim()
{
    pthread_mutex_lock(&send_CS);
    flattrim_pub.publish(std_msgs::Empty());
    pthread_mutex_unlock(&send_CS);
}
void bebop_command::sendReset()
{
    pthread_mutex_lock(&send_CS);
    reset_pub.publish(std_msgs::Empty());
    pthread_mutex_unlock(&send_CS);
}

double bebop_command::getDistance(nav_msgs::Odometry odom_start, nav_msgs::Odometry odom_destination)
{
    return sqrt(pow(odom_destination.pose.pose.position.x-odom_start.pose.pose.position.x, 2.0) +
                pow(odom_destination.pose.pose.position.y-odom_start.pose.pose.position.y, 2.0));
}

double bebop_command::getIncludedAngle(nav_msgs::Odometry odom_start, nav_msgs::Odometry odom_destination)
{
    return atan2((odom_destination.pose.pose.position.y-odom_start.pose.pose.position.y),
                 (odom_destination.pose.pose.position.x-odom_start.pose.pose.position.x));
}

bool bebop_command::arrivePosition(nav_msgs::Odometry odom_start, nav_msgs::Odometry odom_destination)
{
    double height_now = odom_destination.pose.pose.position.z - odom_start.pose.pose.position.z;
    double distance_now = getDistance(odom_start, odom_destination);
    double includedAngle_now = tf::getYaw(odom_start.pose.pose.orientation) -getIncludedAngle(odom_start, odom_destination);
    double turnAngle_now = tf::getYaw(odom_start.pose.pose.orientation) - odom_destination.pose.pose.orientation.w*PI/180.0;
    qDebug("position>>>>>>>x:%0.6f y:%0.6f z:%0.6f yaw:%0.6f inAn:%0.6f dis:%0.6f", odom_start.pose.pose.position.x,
           odom_start.pose.pose.position.y, odom_start.pose.pose.position.z,
           tf::getYaw(odom_start.pose.pose.orientation), includedAngle_now, distance_now);

    if(fabs(height_now) < 0.1)
    {
        if(distance_now <= 0.08)
        {
            if(fabs(turnAngle_now) <= 0.04364)
            {
                publishCommand(setCommand(0.0, 0.0, 0.0, 0.0, 0.0));
                return true;
            }else{
                publishCommand(setCommand(0.0, 0.0, -2.0*turnAngle_now/PI, 0.0, 0.0));
                return false;
            }
        }else {
            if(fabs(includedAngle_now) <= 0.04364)
            {
                publishCommand(setCommand(0.0, (distance_now/distance*0.2)<=0.05?0.05:(distance_now/distance*0.2), 0.0, 0.0, 0.0));
            }else{
                publishCommand(setCommand(0.0, 0.0, -2.0*includedAngle_now/PI, 0.0, 0.0));
            }
            return false;
        }
    }else{
        publishCommand(setCommand(0.0, 0.0, 0.0, height_now/odom_destination.pose.pose.position.z>0.5?0.5:height_now/odom_destination.pose.pose.position.z, 0.0));
    }
}

bool comPoint(const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
{
    return cv::contourArea(a)>cv::contourArea(b);
}

void bebop_command::FindQuads(cv::Mat &in_image, std::vector<std::vector<cv::Point> > &quads_out)
{
    int height = in_image.rows, width = in_image.cols;
    std::vector<std::vector<cv::Point> > contours, top_contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat img_gray = in_image.clone();
    cv::Mat kernel;
    int kernel_sz = 3;
    kernel = cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size(kernel_sz*2+1, kernel_sz*2+1), cv::Point(kernel_sz, kernel_sz));
    //    cv::adaptiveThreshold(img_gray, img_gray, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 9, 0);
    cv::morphologyEx(img_gray, img_gray, CV_MOP_OPEN, kernel);
    cv::morphologyEx(img_gray, img_gray, CV_MOP_CLOSE, kernel);
    //    cv::morphologyEx(img_gray, img_gray, CV_MOP_GRADIENT, kernel);
    cv::threshold(img_gray, img_gray, 48, 255, CV_THRESH_BINARY);

    cv::Canny(img_gray, img_gray, 32, 32);
    cv::findContours(	img_gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
    std::vector<std::vector<cv::Point> > hull(contours.size());
    for (int i=0; i<contours.size(); ++i)
    {
        if (hierarchy[i][2]<0 || contours[i].size() < 150)
            continue;

        convexHull(cv::Mat(contours[i]), hull[i], true);
        std::vector<cv::Point> quadra_corners;
        bool isClosed = true;
        approxPolyDP(hull[i], quadra_corners, 11, isClosed);
        top_contours.push_back(contours[i]);
        cv::Rect rect = cv::boundingRect(contours[i]);
        if (quadra_corners.size() == 4)
        {
            quads_out.push_back(quadra_corners);
        }
    }
    std::sort(top_contours.begin(), top_contours.end(), comPoint);
    std::sort(quads_out.begin(), quads_out.end(), comPoint);
    cv::Mat contours_img(height, width, CV_8UC1);
    contours_img *= 0;
    for(int i=0; i<top_contours.size(); ++i)
    {
        cv::drawContours(contours_img, top_contours, i, 255);
    }

    std::vector<std::vector<cv::Point> > quads_temp = quads_out;
    quads_out.clear();
    for(int i=0; i<quads_temp.size(); ++i)
    {
        uint8_t e = 0;
        cv::Point center(0, 0);
        for (int j = 0; j < quads_temp[i].size(); ++j)
        {
            center.x += quads_temp[i][j].x;
            center.y += quads_temp[i][j].y;
        }
        center.x /= quads_temp[i].size();
        center.y /= quads_temp[i].size();
        std::vector<cv::Point> quad_temp(4);
        for(int j=0; j<quads_out[i].size(); ++j)
        {
            if (quads_temp[i][j].x < center.x && quads_temp[i][j].y < center.y)
            {
                e |= 0x01;
                quad_temp[0].x = quads_temp[i][j].x;
                quad_temp[0].y = quads_temp[i][j].y;
            }
            else if (quads_temp[i][j].x < center.x && quads_temp[i][j].y > center.y)
            {
                e |= 0x02;
                quad_temp[1].x = quads_temp[i][j].x;
                quad_temp[1].y = quads_temp[i][j].y;
            }
            else if (quads_temp[i][j].x > center.x && quads_temp[i][j].y > center.y)
            {
                e |= 0x04;
                quad_temp[2].x = quads_temp[i][j].x;
                quad_temp[2].y = quads_temp[i][j].y;
            }
            else if (quads_temp[i][j].x > center.x && quads_temp[i][j].y < center.y)
            {
                e |= 0x08;
                quad_temp[3].x = quads_temp[i][j].x;
                quad_temp[3].y = quads_temp[i][j].y;
            }
        }
        if (e == 15)
        {
            quads_out.push_back(quad_temp);
        }
    }
}

void bebop_command::add_frame(cv::Mat &r, cv::Mat &t)
{

    tf::Transform temp;
    tf::Quaternion q;
    temp.setOrigin(tf::Vector3(t.at<double>(0,0), t.at<double>(0,1),t.at<double>(0,2)));
    q.setRPY(r.at<double>(0,0),r.at<double>(0,1),r.at<double>(0,2));
    temp.setRotation(q);
    tfbroadcaster.sendTransform(tf::StampedTransform(temp,
                                ros::Time::now(),
                                std::string("/camera_optical"),
                                std::string("/mark_pos")));
}

void bebop_command::get_transform(cv::Mat &r, cv::Mat &t)
{
    listen_once();

    tf::Quaternion q = mytransform.getRotation();

    double angle = q.getAngle();

    tf::Vector3  axis = q.getAxis();
    r.at<double>(0,0) = axis[0] * angle;
    r.at<double>(0,1) = axis[1] * angle;
    r.at<double>(0,2) = axis[2] * angle;

    t.at<double>(0,0) = mytransform.getOrigin().x();
    t.at<double>(0,1)= mytransform.getOrigin().y();
    t.at<double>(0,2) = mytransform.getOrigin().z();


}

void bebop_command::listen_once()
{
    bool need_run = true;
    try{
        tflistener.waitForTransform("/mark_pos", "/odom", ros::Time(0),ros::Duration(0.2));
        tflistener.lookupTransform("/mark_pos","/odom" ,ros::Time(0),mytransform);
        need_run = false;
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("Error---%s",ex.what());
        ros::Duration(0.2).sleep();
    }
}
