/*
 * teleop_pr2_keyboard
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Kevin Watts

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_C 0x63
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_O 0x6f
#define KEYCODE_P 0x70
#define KEYCODE_H 0x6a
#define KEYCODE_L 0x6c

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45
#define KEYCODE_P_CAP 0x50
#define KEYCODE_L_CAP 0x4c

class TeleopUAVKeyboard
{
private:
    double walk_vel, run_vel, yaw_rate, yaw_rate_run, vertical_vel;
    geometry_msgs::Twist cmd;
    std_msgs::Float64 gripper;
    ros::NodeHandle n_;
    ros::Publisher vel_pub_;
    ros::Publisher grip_pub_;
    bool teleopUGV;

public:
    void init()
    {
        cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
        gripper.data = 0;

        vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        grip_pub_ = n_.advertise<std_msgs::Float64>("/r_gripper_controller/command", 1);
        if (!n_.getParam("teleopUGV", teleopUGV))
            puts("fail to load the param");
        ros::NodeHandle n_private("~");
        n_private.param("walk_vel", walk_vel, 1.0);
        n_private.param("run_vel", run_vel, 4.0);
        n_private.param("yaw_rate", yaw_rate, 1.0);
        n_private.param("yaw_run_rate", yaw_rate_run, 1.5);
        n_private.param("vertical_vel", vertical_vel, 1.0);
    }

    ~TeleopUAVKeyboard()   { }
    void keyboardLoop();
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pr2_base_keyboard");

    TeleopUAVKeyboard tpk;
    tpk.init();

    signal(SIGINT, quit);

    tpk.keyboardLoop();

    return(0);
}

void TeleopUAVKeyboard::keyboardLoop()
{
    char c;
    bool dirty = false;
    bool dirtygripper = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    if (teleopUGV)
    {
        puts("Reading from keyboard");
        puts("---------------------------");
        puts("Use 'WS' for move forward and backward");
        puts("Use 'QE' to yaw");
        puts("press 'OC' to open and close the gripper");
        puts("Press 'Shift' to run at fast speed");
    }
    else
    {
        puts("Use 'WASD' to horizontal translate");
        puts("Use 'QE' to yaw");
        puts("Use 'PL' to up/down");
        puts("Use 'H' to hover");
        puts("Press 'Shift' to run");
    }


    for (;;)
    {
        // get the next event from the keyboard
        if (read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        cmd.linear.x = cmd.linear.y = cmd.angular.z = cmd.linear.z = 0;

        switch (c)
        {
        // Walking
        case KEYCODE_W:
            cmd.linear.x = walk_vel;
            dirty = true;
            break;
        case KEYCODE_S:
            cmd.linear.x = - walk_vel;
            dirty = true;
            break;
        case KEYCODE_A:
            cmd.linear.y = walk_vel;
            dirty = true;
            break;
        case KEYCODE_D:
            cmd.linear.y = - walk_vel;
            dirty = true;
            break;
        case KEYCODE_Q:
            cmd.angular.z = yaw_rate;
            dirty = true;
            break;
        case KEYCODE_E:
            cmd.angular.z = - yaw_rate;
            dirty = true;
            break;
        case KEYCODE_P:
            cmd.linear.z = vertical_vel;
            dirty = true;
            break;
        case KEYCODE_L:
            cmd.linear.z = - vertical_vel;
            dirty = true;
            break;
        case KEYCODE_H:
            dirty = true;
            break;



            // Running
        case KEYCODE_W_CAP:
            cmd.linear.x = run_vel;
            dirty = true;
            break;
        case KEYCODE_S_CAP:
            cmd.linear.x = - run_vel;
            dirty = true;
            break;
        case KEYCODE_A_CAP:
            cmd.linear.y = run_vel;
            dirty = true;
            break;
        case KEYCODE_D_CAP:
            cmd.linear.y = - run_vel;
            dirty = true;
            break;
        case KEYCODE_Q_CAP:
            cmd.angular.z = yaw_rate_run;
            dirty = true;
            break;
        case KEYCODE_E_CAP:
            cmd.angular.z = - yaw_rate_run;
            dirty = true;
            break;
            // Gripper
        case KEYCODE_O:
            gripper.data += 0.01;
            gripper.data = gripper.data > 0.5?0.5:gripper.data;
            dirtygripper = true;
            break;
        case KEYCODE_C:
            gripper.data -= 0.01;
            gripper.data = gripper.data < 0?0:gripper.data;
            dirtygripper = true;
            dirty = true;
            break;
        }

        if (dirty == true)
        {
            vel_pub_.publish(cmd);
        }
        if (dirtygripper == true)
        {
            grip_pub_.publish(gripper);
        }
    }
}
