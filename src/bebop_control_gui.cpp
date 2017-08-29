#include "bebop_command.h"
#include "round_thread.h"
#include "bebop_control_gui.h"
#include <stdlib.h>

bebop_control_gui::bebop_control_gui(QWidget *parent) :
    QWidget(parent),it(nh)
{
    ui.setupUi(this);
    isPosition = false;
    isFind = false;
    chooseOption = 0;

    cmdSend.linear.x = 0;
    cmdSend.linear.y = 0;
    cmdSend.linear.z = 0;
    cmdSend.angular.z = 0;

    cameraSend.angular.y = 0;
    cameraSend.angular.z = 0;

    std::string camera_calib_path;
//    camera_calib_path = nh.param<std::string>("camera_calib_path", "/home/unaware/bebop_ws/src/bebop_point/data/bebop2_low_camera_calib.yaml");
    camera_calib_path = "/home/unaware/bebop_ws/src/bebop_point/data/ost.yaml";
    cv::FileStorage camera_cfg(camera_calib_path, cv::FileStorage::READ);
    camera_cfg["camera_matrix"] >> camera_matrix;
    camera_cfg["distortion_coefficients"] >> distCo;

    p_linear = double(camera_cfg["p_linear"]);
    p_angular = double(camera_cfg["p_angular"]);
    d_linear = double(camera_cfg["d_linear"]);

    QObject::connect(this, SIGNAL(closeWindowSignal()), this, SLOT(closeWindowSlot()));
    image_sub = it.subscribe("/bebop/image_raw", 1, &bebop_control_gui::showImage, this);
    battery_sub = nh.subscribe("/bebop/states/common/CommonState/BatteryStateChanged", 1, &bebop_control_gui::showBattery, this);
    odom_sub = nh.subscribe("/bebop/odom", 1, &bebop_control_gui::showOdom, this);
}

bebop_control_gui::~bebop_control_gui()
{

}

void bebop_control_gui::keyPressEvent( QKeyEvent * key)
{
    if(!key->isAutoRepeat())
    {
        if(key->key() == Qt::Key_Z)
        {
            bebop_cmd->sendTakeoff();
        }else if(key->key() == Qt::Key_X)
        {
            bebop_cmd->sendLand();
        }else if(key->key() == Qt::Key_N)
        {
            bebop_cmd->sendReset();
        }else if(key->key() == Qt::Key_M)
        {
            bebop_cmd->sendFlatTrim();
        }else if(key->key() == Qt::Key_Escape)
        {
            this->closeWindow();
        }else{
            bool flag = true;
            switch (key->key()) {
            case Qt::Key_W:
                cmdSend.linear.x +=vel;
                break;
            case Qt::Key_S:
                cmdSend.linear.x -=vel;
                break;
            case Qt::Key_A:
                cmdSend.linear.y +=vel;
                break;
            case Qt::Key_D:
                cmdSend.linear.y -=vel;
                break;
            case Qt::Key_I:
                cmdSend.linear.z +=vel;
                break;
            case Qt::Key_K:
                cmdSend.linear.z -=vel;
                break;
            case Qt::Key_J:
                cmdSend.angular.z +=vel;
                break;
            case Qt::Key_L:
                cmdSend.angular.z -=vel;
                break;
            default:
                flag = false;
                break;
            }
            if(flag)
                bebop_cmd->publishCommand(cmdSend);

            switch (key->key()) {
            case Qt::Key_8:
                cameraSend.angular.y += camera_vel;
                break;
            case Qt::Key_2:
                cameraSend.angular.y -= camera_vel;
                break;
            case Qt::Key_4:
                cameraSend.angular.z -= camera_vel;
                break;
            case Qt::Key_6:
                cameraSend.angular.z += camera_vel;
                break;
            default:
                flag = true;
                break;
            }
            if(!flag)
                bebop_cmd->publishCamera(cameraSend);
        }
    }
}

void bebop_control_gui::keyReleaseEvent( QKeyEvent * key)
{
    if(!key->isAutoRepeat())
    {
        bool flag = true;
        switch (key->key()) {
        case Qt::Key_W:
            cmdSend.linear.x -=vel;
            break;
        case Qt::Key_S:
            cmdSend.linear.x +=vel;
            break;
        case Qt::Key_A:
            cmdSend.linear.y -=vel;
            break;
        case Qt::Key_D:
            cmdSend.linear.y +=vel;
            break;
        case Qt::Key_I:
            cmdSend.linear.z -=vel;
            break;
        case Qt::Key_K:
            cmdSend.linear.z +=vel;
            break;
        case Qt::Key_J:
            cmdSend.angular.z -=vel;
            break;
        case Qt::Key_L:
            cmdSend.angular.z +=vel;
            break;
        default:
            flag = false;
            break;
        }
        if(flag)
            bebop_cmd->publishCommand(cmdSend);
    }
}

void bebop_control_gui::showImage(const sensor_msgs::ImageConstPtr& image_msgs)
{
    cv::Mat img;
    std::vector<std::vector<cv::Point> > quads;
    cv::Mat img_gray;
    //tvec 平移向量 相机坐标系 z前2 x右0 y下1
    //rvec 旋转向量
    cv::Mat rvec, tvec;
    std::vector<cv::Mat> bgr;
    cv::Mat y;
    int mark_flag=0;
    try
    {
        // First let cv_bridge do its magic
        //cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msgs, sensor_msgs::image_encodings::RGB8);
        //conversion_mat_ = cv_ptr->image;

        cv_bridge::CvImagePtr cv_img_ptr = cv_bridge::toCvCopy(image_msgs, "bgr8");
        cv_img_ptr->image.copyTo(img);
        cv::cvtColor(img, img_gray, CV_BGR2GRAY);
        cv::split(img, bgr);
        cv::addWeighted(bgr[2], 0.5, bgr[1], 0.5, 0.0, y);
        cv::addWeighted(y, 1.0, bgr[0], -1.0, 0.0, y);
        bebop_cmd->FindQuads(y, quads);
        if (quads.size() > 0)
        {
            float  L1 =pow(pow((quads[0][0].x-quads[0][1].x),2)+pow((quads[0][0].y-quads[0][1].y),2),0.5);
            float  L2 =pow(pow((quads[0][1].x-quads[0][2].x),2)+pow((quads[0][1].y-quads[0][2].y),2),0.5);
            float  L3 =pow(pow((quads[0][2].x-quads[0][3].x),2)+pow((quads[0][2].y-quads[0][3].y),2),0.5);
            float  L4 =pow(pow((quads[0][3].x-quads[0][1].x),2)+pow((quads[0][3].y-quads[0][1].y),2),0.5);
            /////////////////////////////////////////////////////////////////////////////////////
            if(((L1/L3)>0.8||(L1/L3)<1.2)&&((L2/L4)>0.8||(L2/L4)<1.2))
                //////////////////////////////////////////////////////////////////////////////////////
            {
                std::vector<cv::Point3f> opoints;
                std::vector<cv::Point2f> ipoints;
                opoints.push_back(cv::Point3d(-para_boardLength[para_nowRound-1]/2, -para_boardLength[para_nowRound-1]/2, 0));
                opoints.push_back(cv::Point3d(-para_boardLength[para_nowRound-1]/2, para_boardLength[para_nowRound-1]/2, 0));
                opoints.push_back(cv::Point3d(para_boardLength[para_nowRound-1]/2, para_boardLength[para_nowRound-1]/2, 0));
                opoints.push_back(cv::Point3d(para_boardLength[para_nowRound-1]/2, -para_boardLength[para_nowRound-1]/2, 0));

                ipoints.push_back(cv::Point2f(quads[0][0].x, quads[0][0].y));
                ipoints.push_back(cv::Point2f(quads[0][1].x, quads[0][1].y));
                ipoints.push_back(cv::Point2f(quads[0][2].x, quads[0][2].y));
                ipoints.push_back(cv::Point2f(quads[0][3].x, quads[0][3].y));
                cv::solvePnP(cv::Mat(opoints), cv::Mat(ipoints), camera_matrix, distCo, rvec, tvec);
                if(mark_flag==0)
                {
                    mark_flag=tvec.at<double>(0, 3);
                }
                if(tvec.at<double>(0, 3)-mark_flag<0.4)
                {
                    mark_flag=tvec.at<double>(0, 3);

                    isFind = true;

                    tf2::Quaternion q;
                    q.setRPY(rvec.at<double>(0, 0), rvec.at<double>(0, 1), rvec.at<double>(0, 2));
                    geometry_msgs::PoseStamped qua_came;
                    qua_came.header.frame_id = "camera_optical";
                    qua_came.header.stamp = ros::Time(0);
                    qua_came.pose.position.x = tvec.at<double>(0, 0);
                    qua_came.pose.position.y = tvec.at<double>(0, 1);
                    qua_came.pose.position.z = tvec.at<double>(0, 2);
                    qua_came.pose.orientation.x = q.x();
                    qua_came.pose.orientation.y = q.y();
                    qua_came.pose.orientation.z = q.z();
                    qua_came.pose.orientation.w = q.w();

                    geometry_msgs::PoseStamped qua_odom;
                    try {
                        tfListener.transformPose("odom", qua_came, qua_odom);
                    }
                    catch (tf2::TransformException e) {
                        ROS_ERROR("Received an exception trying to transform a point from \"camera_optical\" to \"odom\": %s", e.what());
                    }

                    /*
                    bebop_cmd->add_frame(rvec, tvec);
                    cv::Mat de_rvec(1,3,CV_64FC1),de_tvec(1,3,CV_64FC1);
                    bebop_cmd->get_transform(de_rvec, de_tvec);
                    */
                    ui.position_x_2->setText(QString::number(qua_odom.pose.position.x));
                    ui.position_y_2->setText(QString::number(qua_odom.pose.position.y));
                    ui.position_z_2->setText(QString::number(qua_odom.pose.position.z));
                    ui.position_yaw_2->setText(QString::number(tf::getYaw(qua_odom.pose.orientation)));

                    double an = para_boardDriection[para_nowRound-1];

                    //circle
                    if (chooseOption == 1)
                    {
                        if(!isPosition){
                            //                    setGoal(odom_initial.pose.pose.position.x + sqrt(pow(tvec.at<double>(0, 2)-1.9, 2.0) + pow(tvec.at<double>(0, 0),2.0))*cos(tf::getYaw(odom_initial.pose.pose.orientation) - atan2(tvec.at<double>(0, 0),tvec.at<double>(0, 2)-1.9)),
                            //                            odom_initial.pose.pose.position.y + sqrt(pow(tvec.at<double>(0, 2)-1.9, 2.0) + pow(tvec.at<double>(0, 0),2.0))*sin(tf::getYaw(odom_initial.pose.pose.orientation) - atan2(tvec.at<double>(0, 0),tvec.at<double>(0, 2)-1.9)),
                            //                            odom_initial.pose.pose.position.z-tvec.at<double>(0, 1) + 0.5,
                            //                            (tf::getYaw(odom_initial.pose.pose.orientation) - rvec.at<double>(0, 1))*180/PI);

                            setGoal(qua_odom.pose.position.x-2.0*cos(an),
                                    qua_odom.pose.position.y-2.0*sin(an),
                                    odom_initial.pose.pose.position.z - tvec.at<double>(0, 1) + 0.5,
                                    an*180/PI);

                            qDebug("initial>>>>>>>>>>>>>>x:%0.6f y:%0.6f z:%0.6f w:%0.6f", odom_initial.pose.pose.position.x, odom_initial.pose.pose.position.y, odom_initial.pose.pose.position.z, tf::getYaw(odom_initial.pose.pose.orientation));
                            qDebug("destina>>>>>>>>>>>>>>>x:%0.6f y:%0.6f z:%0.6f w:%0.6f", odom_end.pose.pose.position.x, odom_end.pose.pose.position.y, odom_end.pose.pose.position.z, tf::getYaw(odom_initial.pose.pose.orientation) - atan2(tvec.at<double>(0, 0),tvec.at<double>(0, 2)-1.9));
                            isPosition= true;
                        }
                        //board
                    }else if(chooseOption == 2) {
                        if(!isPosition){
/*
                            setGoal(odom_initial.pose.pose.position.x + sqrt(pow(tvec.at<double>(0, 2)+0.1, 2.0) + pow(tvec.at<double>(0, 0),2.0))*cos(tf::getYaw(odom_initial.pose.pose.orientation) - atan2(tvec.at<double>(0, 0),tvec.at<double>(0, 2)+0.1)),
                                    odom_initial.pose.pose.position.y + sqrt(pow(tvec.at<double>(0, 2)+0.1, 2.0) + pow(tvec.at<double>(0, 0),2.0))*sin(tf::getYaw(odom_initial.pose.pose.orientation) - atan2(tvec.at<double>(0, 0),tvec.at<double>(0, 2)+0.1)),
                                    odom_initial.pose.pose.position.z,
                                    (tf::getYaw(odom_initial.pose.pose.orientation) - rvec.at<double>(0, 1))*180/PI);
*/

                    setGoal(qua_odom.pose.position.x,
                            qua_odom.pose.position.y,
                            odom_initial.pose.pose.position.z,
                            (tf::getYaw(odom_initial.pose.pose.orientation) - rvec.at<double>(0, 1))*180/PI);

                            qDebug("initial>>>>>>>>>>>>>>x:%0.6f y:%0.6f z:%0.6f w:%0.6f", odom_initial.pose.pose.position.x, odom_initial.pose.pose.position.y, odom_initial.pose.pose.position.z, tf::getYaw(odom_initial.pose.pose.orientation) - rvec.at<double>(0, 1));
                            qDebug("destina>>>>>>>>>>>>>>>x:%0.6f y:%0.6f z:%0.6f w:%0.6f", odom_end.pose.pose.position.x, odom_end.pose.pose.position.y, odom_end.pose.pose.position.z, tf::getYaw(odom_initial.pose.pose.orientation) - rvec.at<double>(0, 1));
                            isPosition= true;
                        }
                    }else{
           //             qDebug("----->x:%f y:%f z:%f p:%f e:%f", tvec.at<double>(0,2),tvec.at<double>(0,0),tvec.at<double>(0,1), rvec.at<double>(0,1), tf::getYaw(odom_initial.pose.pose.orientation) - rvec.at<double>(0, 1));

                    }
                }
            }
        }
    }
    catch(cv_bridge::Exception& e)  //异常处理
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //mark quad
    for (int i=0; i<quads.size() && i<1; ++i)
    {
        for (int j=0; j<quads.at(i).size(); ++j)
        {
            cv::circle(img, quads.at(i).at(j), 4, cv::Scalar(0, 0, 255), 2);
        }
    }

    cv::cvtColor(img, conversion_mat_, CV_BGR2RGB);
    //resize ui.image_lab
    ui.image_lab->resize(conversion_mat_.cols*3/4, conversion_mat_.rows*3/4);
    QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.cols*conversion_mat_.channels(), QImage::Format_RGB888);
    ui.image_lab->setPixmap(QPixmap::fromImage(image).scaled(conversion_mat_.cols*3/4, conversion_mat_.rows*3/4,Qt::KeepAspectRatio));
    cv::waitKey(30);
}

void bebop_control_gui::showBattery(const bebop_msgs::CommonCommonStateBatteryStateChanged battery_msgs)
{
    ui.Battery_info->setText(QString::number(battery_msgs.percent, 10) + "%");
}

void bebop_control_gui::setGoal(double x, double y, double z, double w)
{
    odom_end.pose.pose.position.x = x;
    odom_end.pose.pose.position.y = y;
    odom_end.pose.pose.position.z = z;
    odom_end.pose.pose.orientation.w = w;
    bebop_cmd->distance = bebop_cmd->getDistance(odom_initial, odom_end);
    qDebug("aim---->x:%0.6f y:%0.6f z:%0.6f w:%0.6f", x, y, z, w);
}

void bebop_control_gui::showOdom(const nav_msgs::Odometry odom_msgs)
{
    ui.position->setText("\t"   + QString::number(odom_msgs.pose.pose.position.x,'f', 6) +
                         "\n\t" + QString::number(odom_msgs.pose.pose.position.y,'f', 6) +
                         "\n\t" + QString::number(odom_msgs.pose.pose.position.z,'f', 6) + "\n");

    ui.linear->setText("\t"   + QString::number(odom_msgs.twist.twist.linear.x,'f', 6) +
                       "\n\t" + QString::number(odom_msgs.twist.twist.linear.y,'f', 6) +
                       "\n\t" + QString::number(odom_msgs.twist.twist.linear.z,'f', 6) + "\n");

    ui.angular->setText("\t"   + QString::number(odom_msgs.twist.twist.angular.x,'f', 6) +
                        "\n\t" + QString::number(odom_msgs.twist.twist.angular.y,'f', 6) +
                        "\n\t" + QString::number(odom_msgs.twist.twist.angular.z,'f', 6) + "\n");

    ui.orientation_lab->setText("orientation:\n\t" + QString::number(tf::getYaw(odom_msgs.pose.pose.orientation), 'f', 6) + "\n");

    if(isPosition)
    {
        if(bebop_cmd->arrivePosition(odom_msgs, odom_end))
        {
            chooseOption = 0;
            isPosition = false;
            qDebug(">>>>>>>>>>>>>Have arrived at the specified position!");
        }
    }else {
        odom_initial = odom_msgs;
    }
}

void bebop_control_gui::arrivePosition(QAbstractButton *button)
{
    if(ui.button_position->button(QDialogButtonBox::Ok) == button)
    {
        qDebug("--------> Ok");
        qDebug("odom_initial>>>>>>>>>>>>>>x:%0.6f y:%0.6f z:%0.6f", odom_initial.pose.pose.position.x, odom_initial.pose.pose.position.y, odom_initial.pose.pose.position.z);
        qDebug("destination>>>>>>>>>>>>>>>x:%0.6f y:%0.6f z:%0.6f", ui.position_x->text().toDouble(), ui.position_y->text().toDouble(),ui.position_z->text().toDouble());
        setGoal(ui.position_x->text().toDouble(), ui.position_y->text().toDouble(), ui.position_z->text().toDouble(), ui.position_yaw->text().toDouble());
        isPosition = true;
    }else if(ui.button_position->button(QDialogButtonBox::Cancel) == button) {
        isPosition = false;
        bebop_cmd->publishCommand(bebop_cmd->setCommand(0.0, 0.0, 0.0, 0.0, 0.0));
        qDebug("--------> Cancel");
    }
}

void bebop_control_gui::startRound(QAbstractButton *button)
{
    if(ui.button_round->button(QDialogButtonBox::Ok) == button)
    {
        qDebug("Round--------> Ok");
 //       bebop_cmd->publishCamera(bebop_cmd->setCommand(0.0, 0.0, 0.0, 0.0, 1.0));
 //       bebop_cmd->publishCamera(bebop_cmd->setCommand(0.0, 0.0, 0.0, 0.0, 0.0));
 //       chooseOption = 2;
        roundThread->start();
    }else{
//        bebop_cmd->publishCamera(bebop_cmd->setCommand(0.0, 0.0, 0.0, 0.0, 1.0));
//        bebop_cmd->publishCommand(bebop_cmd->setCommand(0.0, 0.0, 0.0, 0.0, 0.0));
        chooseOption = 0;
        isPosition = false;
        roundThread->exit();
        qDebug("Round--------> Cancel");
    }
}

void bebop_control_gui::closeWindowSlot()
{
    ros::shutdown();
    closeWindow();
}

void bebop_control_gui::closeWindow()
{
    emit closeWindowSignal();
}
