#ifndef MOTOR_DRIVER_TESTER_H
#define MOTOR_DRIVER_TESTER_H

#include <ros/ros.h>

#include "motor_driver_msgs/MotorCommand.h"

#include <termios.h>

#include "ftxui/component/container.hpp"
#include "ftxui/component/checkbox.hpp"
#include "ftxui/screen/string.hpp"
#include "ftxui/screen/color.hpp"

class Motor_driver_tester
{
public:
    Motor_driver_tester(ros::NodeHandle &n)
      : publisher_motor_command_(n.advertise<motor_driver_msgs::MotorCommand>("motor_command",1000))
       {
         HideEcho(true);
         ClearTerminal();
       }
       ~Motor_driver_tester()
       {
         HideEcho(false);
         for(int i=0;i<4;i++)
         {
           ResetAtMotorCommand(i);
         }
         DrawTUI();
         Publisher();
         ClearTerminal();
       }

    void HideEcho(bool value);

    void ClearTerminal();

    int ReturnInputKey();

    void DrawTUI();

    int GetInputKey();

    void InputMotorCommand(int menu_number, int key_input);

    void ResetAtMotorCommand(int menu_number);

    int DoJoin();

    int Publisher();

    void Spin();

private:
    ros::Publisher publisher_motor_command_;
    motor_driver_msgs::MotorCommand motor_command_;

    int menu_number_ = 0;
    int state_mute_ = 0;
    int state_join_ = 0;

    struct termios org_term_;
    struct termios new_term_;


};
#endif // MOTOR_DRIVER_TESTER_H
