 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include "bebop_control_gui.h"
#include "bebop_command.h"
#include "round_thread.h"

#include <QtGui>
#include <QApplication>
#include "ros/ros.h"

// this global var is used in getMS(ros::Time t) to convert to a consistent integer timestamp used internally pretty much everywhere.
// kind of an artifact from Windows-Version, where only that was available / used.
unsigned int ros_header_timestamp_base = 0;

int main(int argc, char *argv[])
{
    std::cout << "Starting bebop_control_gui Node" << std::endl;

	// ROS
    ros::init(argc, argv, "bebop_control");

    bebop_command bebop_cmd;
    round_thread round_th;

    // UI
    QApplication a(argc, argv);
    bebop_control_gui w;

    //bebop_cmd.odom_initial = &w.odom_initial;
    w.bebop_cmd = &bebop_cmd;
    w.roundThread = &round_th;
    round_th.bebop_cmd = &bebop_cmd;
    round_th.bebop_control = &w;

    // start
    w.show();
    ros::spin();
    if(!ros::ok())
        w.closeWindow();
    // wait until windows closed....
    int ec = a.exec();

    std::cout << "Exiting bebop_control_gui Node" << std::endl;

    return ec;
}
