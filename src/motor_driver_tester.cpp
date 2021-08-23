#include "ros/ros.h"

#include "pigeon_run/motor_driver_tester.h"

void Motor_driver_tester::EncoderCountCallback(const encoder_msgs::EncoderCount &data)
{
  encoder_count_ = data;
}

void Motor_driver_tester::HideEcho(bool value)
{
  if(value == true)
  {
    tcgetattr(STDIN_FILENO, &org_term_);

    new_term_ = org_term_;

    new_term_.c_lflag &= ~(ECHO | ICANON);

    new_term_.c_cc[VMIN] = 0;
    new_term_.c_cc[VTIME] = 0;

    tcsetattr(STDIN_FILENO, TCSANOW, &new_term_);
  }
  else tcsetattr(STDIN_FILENO, TCSANOW, &org_term_);

  return;
}

void Motor_driver_tester::ClearTerminal()
{
  printf("\033[2J\033[1;1H");
  return;
}

int Motor_driver_tester::ReturnInputKey()
{
  char input_key = 0;

  if(read(STDIN_FILENO, &input_key, 1) != 1) input_key = 0;

  else
  {
    char dummy;
    while (read(STDIN_FILENO, &dummy, 1) == 1) ;
  }

  return input_key;
}

void Motor_driver_tester::DrawTUI()
{

  double gauge_command_L = 0.0;
  double gauge_command_R = 0.0;
  double negative_gauge_command_L = 1.0;
  double negative_gauge_command_R = 1.0;

  if(motor_command_.command_L > 0)
  {
    gauge_command_L = (double)motor_command_.command_L/100;
    negative_gauge_command_L = 1.0;
  }
  else if (motor_command_.command_L < 0) {
    negative_gauge_command_L = 1 - (-1 * (double)motor_command_.command_L/100);
  }
  else if (motor_command_.command_L == 0) {
    gauge_command_L = 0.0;
    negative_gauge_command_L = 1.0;
  }

  if(motor_command_.command_R > 0)
  {
    gauge_command_R = (double)motor_command_.command_R/100;
    negative_gauge_command_R = 1.0;
  }
  else if (motor_command_.command_R < 0) {
    negative_gauge_command_R = 1 - (-1 * (double)motor_command_.command_R/100);
  }
  else if (motor_command_.command_R == 0) {
    gauge_command_R = 0.0;
    negative_gauge_command_R = 1.0;
  }

  // -------- style 조건문 --------------
  auto style_0 = (menu_number_ == 0) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
  auto style_1 = (menu_number_ == 1) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
  auto style_2 = (menu_number_ == 2) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
  auto style_3 = (menu_number_ == 3) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
  auto style_4 = (menu_number_ == 4) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);


  auto limit_style_0 = (gauge_command_L >= 1.0) ? (menu_number_ == 0) ? color(ftxui::Color::RedLight) | ftxui::dim : color(ftxui::Color::RedLight) : (menu_number_ == 0) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
  auto limit_style_1 = (gauge_command_R >= 1.0) ? (menu_number_ == 1) ? color(ftxui::Color::RedLight) | ftxui::dim : color(ftxui::Color::RedLight) : (menu_number_ == 1) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);

  auto negative_limit_style_0 = (negative_gauge_command_L == 0.0) ? (menu_number_ == 0) ? color(ftxui::Color::Red) | ftxui::dim : color(ftxui::Color::Red) : (menu_number_ == 0) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
  auto negative_limit_style_1 = (negative_gauge_command_R == 0.0) ? (menu_number_ == 1) ? color(ftxui::Color::Red) | ftxui::dim : color(ftxui::Color::Red) : (menu_number_ == 1) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);

  ftxui::Element Document =
  ftxui::vbox
  ({
    // -------- Top panel --------------
    ftxui::hbox
    ({
      ftxui::hbox
      ({
        ftxui::text(L"Motor Driver Tester") | ftxui::bold | ftxui::center, ftxui::separator(),
      }),
      ftxui::text(L"motor_command")| color(ftxui::Color::Red) | ftxui::center, ftxui::separator(),
      ftxui::text(L"ver 0.3")| ftxui::bold,
    }),
    ftxui::separator(),
    // -------- motor_command Menu --------------
    ftxui::hbox
    ({
      ftxui::vbox
      ({
        ftxui::text(L"-"),
        ftxui::text(L"-"),
      }) | ftxui::bold,ftxui::separator(),
      ftxui::vbox
      ({
        ftxui::gauge(negative_gauge_command_L) | ftxui::inverted | negative_limit_style_0,
        ftxui::gauge(negative_gauge_command_R) | ftxui::inverted | negative_limit_style_1,
      }) | ftxui::flex, ftxui::separator(),
      ftxui::vbox
      ({
        ftxui::text(L"command_L") | style_0 | ftxui::center,
        ftxui::text(L"command_R") | style_1 | ftxui::center,
      }) | ftxui::bold,ftxui::separator(),
      ftxui::vbox
      ({
        ftxui::gauge(gauge_command_L) | limit_style_0,
        ftxui::gauge(gauge_command_R) | limit_style_1,
      }) | ftxui::flex, ftxui::separator(),
      ftxui::vbox
      ({
        ftxui::text(L"+"),
        ftxui::text(L"+"),
      }),
    }) | ftxui::flex, ftxui::separator(),
    // -------- motor_command panel --------------
    ftxui::hbox
    ({
      ftxui::vbox
      ({
        ftxui::hbox
        ({
          ftxui::text(L"command_L  : "),
          ftxui::text(ftxui::to_wstring(std::to_string(motor_command_.command_L))),
        }),
        ftxui::hbox
        ({
          ftxui::text(L"command_R  : "),
          ftxui::text(ftxui::to_wstring(std::to_string(motor_command_.command_R))),
        }),
      }) | ftxui::bold, ftxui::separator(),
      ftxui::vbox
      ({
        ftxui::hbox
        ({
          ftxui::text(L"EncoderCount_L  : "),
          ftxui::text(ftxui::to_wstring(std::to_string(encoder_count_.Count[0]))),
        }),
        ftxui::hbox
        ({
          ftxui::text(L"EncoderCount_R  : "),
           ftxui::text(ftxui::to_wstring(std::to_string(encoder_count_.Count[1]))),
        }),
      }) | ftxui::bold, ftxui::separator(),
      ftxui::vbox
      ({
        state_mute_ == 0 ? ftxui::text(L"Mute ☐ ") | style_2 : ftxui::text(L"Mute ▣ ") | style_2 ,
        state_join_ == 0 ? ftxui::text(L"Join ☐ ☐ ") | style_3 : state_join_ == 1 ? ftxui::text(L"Join ▣ ☐ ") | style_3 : ftxui::text(L"Join ▣ ▣ ") | style_3,
        state_encoder_reset_ == 0 ? ftxui::text(L"ResetEncoder ☐ ") | style_4 : ftxui::text(L"ResetEncoder ▣ ") | style_4 ,
      })
    }) | ftxui::flex,
  });

  Document = border(Document);
  auto screen = ftxui::Screen::Create(ftxui::Dimension::Full(), ftxui::Dimension::Fit(Document));

  Render(screen, Document);

  ClearTerminal();
  screen.Print();
  return;

}

int Motor_driver_tester::GetInputKey() // 키 입력 함수
{
  int minimum_menu_number = 0;
  int maximum_menu_number = 4;

  int key_value = ReturnInputKey();

  // INPUT W
  if(key_value == 119 | key_value == 87){
    menu_number_ = menu_number_ - 1;
    if(menu_number_ < minimum_menu_number ) menu_number_ = minimum_menu_number;
  }

  // INPUT S
  if(key_value == 115 | key_value == 83)
  {
    menu_number_ = menu_number_ + 1;
    if(menu_number_ > maximum_menu_number ) menu_number_ = maximum_menu_number;
  }

  //INPUT D
  if(key_value == 100 | key_value == 68) InputMotorCommand(menu_number_, key_value);

  //INPUT A
  if(key_value == 97 | key_value == 65) InputMotorCommand(menu_number_, key_value);

  //INPUT X
  if(key_value == 120 | key_value == 88) ResetAtMotorCommand(menu_number_);

  //INPUT Z
  if(key_value == 122 | key_value == 90)
  {
    for(int i=0;i<4;i++)
    {
      ResetAtMotorCommand(i);
    }

  }


  return 0;

}

void Motor_driver_tester::InputMotorCommand(int menu_number, int key_value)
{
//----------------- command_L -----------------//
  if(menu_number == 0)
  {
    if(key_value == 100 | key_value == 68)
    {
      motor_command_.command_L = motor_command_.command_L + 1;
      if(motor_command_.command_L > 100) motor_command_.command_L = 100;
    }
     if(key_value == 97 | key_value == 65 )
     {
       motor_command_.command_L = motor_command_.command_L - 1;
       if(motor_command_.command_L < -100) motor_command_.command_L = -100;
     }
  }

//----------------- command_R -----------------//
  if(menu_number == 1)
  {
    if(key_value == 100 | key_value == 68)
    {
      motor_command_.command_R = motor_command_.command_R + 1;
      if(motor_command_.command_R > 100) motor_command_.command_R = 100;
    }

    if(key_value == 97 | key_value == 65)
    {
      motor_command_.command_R = motor_command_.command_R - 1;
      if(motor_command_.command_R < -100) motor_command_.command_R = -100;
    }
  }

//----------------- Mute -----------------//
  if(menu_number == 2)
  {
    if(key_value == 100 | key_value == 68) state_mute_ = 1;
    if(key_value == 97 | key_value == 65) state_mute_ = 0;
  }

//----------------- Join -----------------//
  if(menu_number == 3)
  {
    if(key_value == 100 | key_value == 68)
    {
      state_join_++;
      if(state_join_ > 2) state_join_ = 2;
    }
    if(key_value == 97 | key_value == 65)
    {
      state_join_--;
      if(state_join_ < 0) state_join_ = 0;
    }
  }

//----------------- Encoder Reset -----------------//
  if(menu_number == 4)
  {
    if(key_value == 100 | key_value == 68) state_encoder_reset_ = 1;
  }
}

void Motor_driver_tester::ResetAtMotorCommand(int menu_number) // 현재 항목 리셋 함수
{
//----------------- command_L -----------------//
  if(menu_number == 0) motor_command_.command_L = 0;

//----------------- command_R -----------------//
  if(menu_number == 1) motor_command_.command_R = 0;

//----------------- Mute -----------------//
  if(menu_number == 2) state_mute_ = 0;

//----------------- Join -----------------//
  if(menu_number == 3) state_join_ = 0;

//----------------- Join -----------------//
  if(menu_number == 4) state_encoder_reset_ = 0;
}

int Motor_driver_tester::DoJoin()
{

  if(menu_number_ == 0) motor_command_.command_R = motor_command_.command_L;
  if(menu_number_ == 1) motor_command_.command_L = motor_command_.command_R;

  if(motor_command_.command_L != motor_command_.command_R)
  {
    if(abs(motor_command_.command_L) > abs(motor_command_.command_R)) motor_command_.command_L = motor_command_.command_R;
    else if(abs(motor_command_.command_L) < abs(motor_command_.command_R)) motor_command_.command_R = motor_command_.command_L;
    else if(abs(motor_command_.command_L) == abs(motor_command_.command_R)) motor_command_.command_R = motor_command_.command_L;
  }

  return 0;
}

int Motor_driver_tester::DoJoinOpposite()
{
  if(menu_number_ == 0) motor_command_.command_R = (-1)*motor_command_.command_L;
  if(menu_number_ == 1) motor_command_.command_L = (-1)*motor_command_.command_R;

  if(motor_command_.command_L != motor_command_.command_R | motor_command_.command_L == motor_command_.command_R)
  {
    if(abs(motor_command_.command_L) > abs(motor_command_.command_R)) motor_command_.command_L = (-1)*motor_command_.command_R;
    else if(abs(motor_command_.command_L) < abs(motor_command_.command_R)) motor_command_.command_R = (-1)*motor_command_.command_L;
    else if(abs(motor_command_.command_L) == abs(motor_command_.command_R)) motor_command_.command_R = (-1)*motor_command_.command_L;
  }

  return 0;
}

void Motor_driver_tester::ResetEncoderCountServiceCall()
{
  reset_encoder_count_.request.command = true;
  service_client_reset_encoder_count_.call(reset_encoder_count_);
}

int Motor_driver_tester::Publisher() // 토픽 퍼블리시 함수
{
  publisher_motor_command_.publish(motor_command_);
  return 0;
}

void Motor_driver_tester::Spin() // 전체 흐름 제어 함수
{
  DrawTUI();
  GetInputKey();

  if(state_encoder_reset_ == 1)
  {
    ResetEncoderCountServiceCall();
    state_encoder_reset_ = 0;
  }

  if(state_join_ == 1) DoJoin();
  if(state_join_ == 2) DoJoinOpposite();

  if(state_mute_ == 0) Publisher();
  else if(state_mute_ == 1)
  {
    motor_command_.command_L = 0;
    motor_command_.command_R = 0;
    Publisher();
  }
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_driver_tester_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(60);

  Motor_driver_tester motor_driver_tester(n);

  while (ros::ok())
  {
    motor_driver_tester.Spin();
    loop_rate.sleep();
    ros::spinOnce();

  }
  return 0;

}
