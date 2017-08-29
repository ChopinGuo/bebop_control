#ifndef ROUND_THREAD_H
#define ROUND_THREAD_H
#include <QThread>

static int para_nowRound = 1;
static double para_boardLength[9] = { 0.9,  0.9, 1.2,  0.9,  0.9, 1.2, 1.2,  0.9, 1.2};
static bool para_isBoard[9]       = {true,false,true,false,false,true,true,false,true};
static double para_direction[9]   = { 2.042,  2.301, -1.298,  0.0,  0.0, 0.0, 0.0,  0.0, 0.0};
static double para_boardDriection[9]   = { 1.269,  2.610, 0.0,  0.0,  0.0, 0.0, 0.0,  0.0, 0.0};

class bebop_command;
class bebop_control_gui;
class round_thread : public QThread
{

public:
    round_thread();
    ~round_thread();

    bebop_command* bebop_cmd;
    bebop_control_gui* bebop_control;

    void run();
    void doId_board();
    void doOb_circle();

private:

};
#endif
