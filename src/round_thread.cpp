#include "round_thread.h"
#include "bebop_command.h"
#include "bebop_control_gui.h"

round_thread::round_thread()
{
    para_nowRound = 1;
}

round_thread::~round_thread()
{

}

void round_thread::run()
{
    /*******1*******//*
    //bebop_cmd->publishCamera(bebop_cmd->setCommand(0.0, 0.0, 0.0, 0.0, 0.0));
    bebop_cmd->sendFlatTrim();
    QThread::msleep(100);
    bebop_cmd->sendTakeoff();
    QThread::msleep(3000);
    bebop_control->setGoal(bebop_control->odom_initial.pose.pose.position.x, bebop_control->odom_initial.pose.pose.position.y, bebop_control->odom_initial.pose.pose.position.z, para_direction[para_nowRound - 1]);
    bebop_control->isPosition = true;
    while(bebop_control->isPosition)
        QThread::sleep(1);
*/
//    para_nowRound++;


    /*******2*******/
    //bebop_cmd->publishCamera(bebop_cmd->setCommand(0.0, 0.0, 0.0, 0.0, -45.0));
    while(!bebop_control->isFind){
        bebop_cmd->publishCommand(bebop_cmd->setCommand(0.0, 0.05, 0.0, 0.0, 0.0));
    }
    qDebug(">>>>>>1");
    bebop_control->isFind = false;
    bebop_cmd->publishCommand(bebop_cmd->setCommand(0.0, 0.0, 0.0, 0.0, 0.0));
    //bebop_cmd->publishCamera(bebop_cmd->setCommand(0.0, 0.0, 0.0, 0.0, 1.0));
    //bebop_cmd->publishCamera(bebop_cmd->setCommand(0.0, 0.0, 0.0, 0.0, 0.0));
    bebop_control->chooseOption = 2;
    QThread::sleep(3);
    while(bebop_control->isPosition)
        QThread::sleep(1);
    qDebug(">>>>>>2");
    bebop_cmd->sendLand();
    QThread::sleep(3);
    bebop_cmd->sendTakeoff();
    bebop_control->setGoal(bebop_control->odom_initial.pose.pose.position.x, bebop_control->odom_initial.pose.pose.position.y, bebop_control->odom_initial.pose.pose.position.z, para_direction[para_nowRound - 1]);
    bebop_control->isPosition = true;
    while(bebop_control->isPosition)
        QThread::sleep(1);
    para_nowRound++;
    /*******3*******/

    while(!bebop_control->isFind){
        bebop_cmd->publishCommand(bebop_cmd->setCommand(0.0, 0.05, 0.0, 0.0, 0.0));
    }
    bebop_control->isFind = false;
    bebop_cmd->publishCommand(bebop_cmd->setCommand(0.0, 0.0, 0.0, 0.0, 0.0));
    //bebop_cmd->publishCamera(bebop_cmd->setCommand(0.0, 0.0, 0.0, 0.0, 1.0));
    //bebop_cmd->publishCamera(bebop_cmd->setCommand(0.0, 0.0, 0.0, 0.0, 0.0));
    bebop_control->chooseOption = 1;
    QThread::sleep(3);
    while(bebop_control->isPosition)
        QThread::sleep(1);

    bebop_control->setGoal(bebop_control->odom_initial.pose.pose.position.x + 2*cos(para_boardDriection[para_nowRound-1]), bebop_control->odom_initial.pose.pose.position.y + 2*sin(para_boardDriection[para_nowRound-1]), bebop_control->odom_initial.pose.pose.position.z, para_direction[para_nowRound-2]);
    bebop_control->isPosition = true;
    while(bebop_control->isPosition)
        QThread::sleep(1);
    para_nowRound++;

    /*******4*******/
bebop_cmd->sendLand();

    /*******5*******/

    /*******6*******/

    /*******7*******/

    /*******8*******/

    /*******9*******/

    /*******10*******/

}

void round_thread::doId_board()
{
    bebop_cmd->publishCamera(bebop_cmd->setCommand(0.0, 0.0, 0.0, 0.0, 0.0));
    bebop_cmd->sendTakeoff();

}

void round_thread::doOb_circle()
{

}
