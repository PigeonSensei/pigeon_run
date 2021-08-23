#ifndef MOTOR_DRIVER_TESTER_H
#define MOTOR_DRIVER_TESTER_H

#include <ros/ros.h>

#include "motor_driver_msgs/MotorCommand.h"
#include "encoder_msgs/EncoderCount.h"

#include "encoder_msgs/ResetEncoderCount.h"

#include <termios.h>

#include "ftxui/component/container.hpp"
#include "ftxui/component/checkbox.hpp"
#include "ftxui/screen/string.hpp"
#include "ftxui/screen/color.hpp"

class Motor_driver_tester
{
public:
    Motor_driver_tester(ros::NodeHandle &n)
      : publisher_motor_command_(n.advertise<motor_driver_msgs::MotorCommand>("motor_command",1000)),
        subscriber_encoder_count_(n.subscribe("encoder_count", 100, &Motor_driver_tester::EncoderCountCallback, this)),
        service_client_reset_encoder_count_(n.serviceClient<encoder_msgs::ResetEncoderCount>("resetEncoderCount"))
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

    void EncoderCountCallback(const encoder_msgs::EncoderCount &data);

    void HideEcho(bool value);

    void ClearTerminal();

    int ReturnInputKey();

    void DrawTUI();

    int GetInputKey();

    void InputMotorCommand(int menu_number, int key_input);

    void ResetAtMotorCommand(int menu_number);

    int DoJoin();

    int DoJoinOpposite();

    void ResetEncoderCountServiceCall();

    int Publisher();

    void Spin();

private:

    ros::Publisher publisher_motor_command_;
    ros::Subscriber subscriber_encoder_count_;
    ros::ServiceClient service_client_reset_encoder_count_;


    motor_driver_msgs::MotorCommand motor_command_;
    encoder_msgs::EncoderCount encoder_count_;
    encoder_msgs::ResetEncoderCount reset_encoder_count_;

    int menu_number_ = 0;
    int state_mute_ = 0;
    int state_join_ = 0;
    int state_encoder_reset_ = 0;

    struct termios org_term_;
    struct termios new_term_;


};
#endif // MOTOR_DRIVER_TESTER_H
