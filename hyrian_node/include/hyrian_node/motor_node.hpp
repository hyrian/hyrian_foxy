#ifndef MOTOR_NODE_H
#define MOTOR_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include <pigpiod_if2.h>
#include <fstream>

// #define motor1_dir 6
// #define motor1_pwm 13

#define motor1_dir 19
#define motor1_pwm 26
#define motor1_encA 23
#define motor1_encB 24 //왼쪽 모터가 1

#define motor2_dir 6
#define motor2_pwm 13
#define motor2_encA 27
#define motor2_encB 17  //오른쪽 모터가 2

#define PI 3.141592

using namespace std::chrono_literals;
using std::placeholders::_1;

// LoadParameters
void LoadParameters(void);
int pwm_range;
int pwm_frequency;
int pwm_limit;
double control_cycle;
int acceleration_ratio;
double wheel_radius;
double robot_radius;
int encoder_resolution;
double wheel_round;
double robot_round;

// InitMotors
int InitMotors(void);
int pinum;
int current_pwm1;
int current_pwm2;
bool current_direction1;
bool current_direction2;
int acceleration;

double left_rpm;
double right_rpm;
double left_speed;
double right_speed;
double linear_vel1; 
double linear_vel2;

double raw_odom_1;
double raw_odom_2;


bool target_dir1 = true;
bool target_dir2 = true;

//LOW-PASS FILTER
float dzn_lower_limit = 8.0;
float dzn_upper_limit = 60.0;

double coeff_ext_filter = 0.5;
double rpm_value2_fold = 0.0;

//PID Control
double target_pwm1 = 0.0;
double target_pwm2 = 0.0;

double topic_rpm_value1 = 0.0;
double topic_rpm_value2 = 0.0;

// #define TIME 0.0666666667
double pidControl1 = 0.0;
double p_gain1 = 0.14;
double i_gain1 = 0.0;
double d_gain1 = 0.03;
double derivative1 = 0.0;
double lastderivative1 = 0.0;
double last_input1 = 0.0;
double pControl1 = 0.0;
double iControl1 = 0.0;
double dControl1 = 0.0;

double pidControl2 = 0.0;
double p_gain2 = 0.14;
double i_gain2 = 0.0;
double d_gain2 = 0.03;
double derivative2 = 0.0;
double lastderivative2 = 0.0;
double last_input2 = 0.0;
double pControl2 = 0.0;
double iControl2 = 0.0;
double dControl2 = 0.0;

// SetInterrupts
void Interrupt_Setiing(void);
volatile int encoder_count_1;
volatile int encoder_count_2;
volatile int encoder_count_1A;
volatile int encoder_count_1B;
volatile int encoder_count_2A;
volatile int encoder_count_2B;
volatile int speed_count_1;
volatile int speed_count2;
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
int SumMotor1Encoder();
int SumMotor2Encoder();
void InitEncoders(void);
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Initialize(void);

// Controller
void MotorController(int motor_num, bool direction, int pwm);
void AccelController(int motor_num, bool direction, int desired_pwm);

// Example
bool switch_direction;
int theta_distance_flag;
void SwitchTurn(int pwm1, int pwm2);
void ThetaTurn(double theta, int pwm);
void DistanceGo(double distance, int pwm);
void ThetaTurnDistanceGo(double theta, int turn_pwm, double distance, int go_pwm);

// Utility
int LimitPwm(int pwm);
double rpm_value1;
double rpm_value2;
void CalculateRpm();
void InfoMotors();
class RosCommunicator : public rclcpp::Node
{
public:
  RosCommunicator();


private:
  void TimerCallback();
  void TeleopCallback(const std_msgs::msg::Int64MultiArray::SharedPtr msg);
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void pidGainCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void DirectMotorControl();  
  void PidMotorControl();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pid_check_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pid_gain_;
  size_t count_;


};

#endif // MOTOR_NODE_H