#include <hyrian_node/motor_node.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"


bool direction_flag1 = true;
bool direction_flag2 = true;


void LoadParameters(void)
{
  std::ifstream inFile("/home/ubuntu/feature_ws/src/hyrian_foxy/hyrian_node/data/motor_input.txt");
  if (!inFile.is_open())
  {
    RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "Unable to open the file");
    return;
  }

  int i = 0;
  std::size_t found;
  for (std::string line; std::getline(inFile, line);)
  {
    found = line.find("=");

    switch (i)
    {
    case 0:
      pwm_range = atof(line.substr(found + 2).c_str());
      break;
    case 1:
      pwm_frequency = atof(line.substr(found + 2).c_str());
      break;
    case 2:
      pwm_limit = atof(line.substr(found + 2).c_str());
      break;
    case 3:
      control_cycle = atof(line.substr(found + 2).c_str());
      break;
    case 4:
      acceleration_ratio = atof(line.substr(found + 2).c_str());
      break;
    case 5:
      wheel_radius = atof(line.substr(found + 2).c_str());
      break;
    case 6:
      robot_radius = atof(line.substr(found + 2).c_str());
      break;
    case 7:
      encoder_resolution = atof(line.substr(found + 2).c_str());
      break;
      // case :  = atof(line.substr(found+2).c_str()); break;
    }
    i += 1;
  }
  inFile.close();
}

int InitMotors(void)
{
  pinum = pigpio_start(NULL, NULL);

  if (pinum < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "Setup failed");
    RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "pinum is %d", pinum);
    return 1;
  }

  set_mode(pinum, motor1_dir, PI_OUTPUT);
  set_mode(pinum, motor2_dir, PI_OUTPUT);
  set_mode(pinum, motor1_pwm, PI_OUTPUT);
  set_mode(pinum, motor2_pwm, PI_OUTPUT);
  set_mode(pinum, motor1_encA, PI_INPUT);
  set_mode(pinum, motor1_encB, PI_INPUT);
  set_mode(pinum, motor2_encA, PI_INPUT);
  set_mode(pinum, motor2_encB, PI_INPUT);

  gpio_write(pinum, motor1_dir, PI_LOW);
  gpio_write(pinum, motor2_dir, PI_LOW);

  set_PWM_range(pinum, motor1_pwm, pwm_range);
  set_PWM_range(pinum, motor2_pwm, pwm_range);
  set_PWM_frequency(pinum, motor1_pwm, pwm_frequency);
  set_PWM_frequency(pinum, motor2_pwm, pwm_frequency);
  set_PWM_dutycycle(pinum, motor1_pwm, 0);
  set_PWM_dutycycle(pinum, motor1_pwm, 0);

  set_pull_up_down(pinum, motor1_encA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor1_encB, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_encA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_encB, PI_PUD_DOWN);

  current_pwm1 = 0;
  current_pwm2 = 0;

  current_direction1 = true;
  current_direction2 = true;

  acceleration = pwm_limit / (acceleration_ratio);

  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Setup Fin");
  return 0;
}

void SetInterrupts(void)
{
  callback(pinum, motor1_encA, EITHER_EDGE, Interrupt1A);
  callback(pinum, motor1_encB, EITHER_EDGE, Interrupt1B);
  callback(pinum, motor2_encA, EITHER_EDGE, Interrupt2A);
  callback(pinum, motor2_encB, EITHER_EDGE, Interrupt2B);
}

void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick) //left
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor1_dir) == true)
    encoder_count_1A--;
  else
    encoder_count_1A++;
  speed_count_1++;
}

void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick) 
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor1_dir) == true)
    encoder_count_1B--;
  else
    encoder_count_1B++;
  speed_count_1++;
}

void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick) 
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor2_dir) == true)
    encoder_count_2A--;
  else
    encoder_count_2A++;
  speed_count2++;
}

void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor2_dir) == true)
    encoder_count_2B--;
  else
    encoder_count_2B++;
  speed_count2++;
}

int SumMotor1Encoder()
{
  encoder_count_1 = encoder_count_1A + encoder_count_1B;
  return encoder_count_1;
}

int SumMotor2Encoder()
{
  encoder_count_2 = encoder_count_2A + encoder_count_2B;
  return encoder_count_2;
}

void InitEncoders(void)
{
  encoder_count_1 = 0;
  encoder_count_2 = 0;
  encoder_count_1A = 0;
  encoder_count_1B = 0;
  encoder_count_2A = 0;
  encoder_count_2B = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pid_check_;
rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pid_gain_;

void Initialize(void)
{
  LoadParameters();
  InitMotors();
  InitEncoders();
  SetInterrupts();

  wheel_round = 2 * PI * wheel_radius;
  robot_round = 2 * PI * robot_radius;

  switch_direction = true;
  theta_distance_flag = 0;

  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_range %d", pwm_range);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_frequency %d", pwm_frequency);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_limit %d", pwm_limit);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "control_cycle %f", control_cycle);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "acceleration_ratio %d", acceleration_ratio);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Initialize Complete");

  printf("\033[2J");
}

void MotorController(int motor_num, bool direction, int pwm)
{


  int local_pwm = LimitPwm(pwm);

  if (motor_num == 1) 
  {
    if (direction == true) 
    {
      gpio_write(pinum, motor1_dir, PI_LOW);
      set_PWM_dutycycle(pinum, motor1_pwm, local_pwm);
      current_pwm1 = local_pwm;
      current_direction1 = true; 
      linear_vel1 = ((2 * PI * wheel_radius * rpm_value1) / 60);  

    }
    else if (direction == false) 
    {
      gpio_write(pinum, motor1_dir, PI_HIGH);
      set_PWM_dutycycle(pinum, motor1_pwm, local_pwm);
      current_pwm1 = local_pwm;
      current_direction1 = false;
      linear_vel1 = -((2 * PI * wheel_radius * rpm_value1) / 60.0); 

    }
  }

  else if (motor_num == 2) 
  {
    if (direction == true)
    {
      gpio_write(pinum, motor2_dir, PI_LOW);
      set_PWM_dutycycle(pinum, motor2_pwm, local_pwm);
      current_pwm2 = local_pwm;
      current_direction2 = true;
      linear_vel2 = -((2 * PI * wheel_radius * rpm_value2) / 60.0);  

    }
    else if (direction == false) 
    {
      gpio_write(pinum, motor2_dir, PI_HIGH);
      set_PWM_dutycycle(pinum, motor2_pwm, local_pwm);
      current_pwm2 = local_pwm;
      current_direction2 = false;
      linear_vel2 = ((2 * PI * wheel_radius * rpm_value2) / 60.0);

    }
  }
}

int LimitPwm(int pwm)
{
  int output;
  if (pwm > pwm_limit * 2)
  {
    output = pwm_limit;
    RCLCPP_WARN(rclcpp::get_logger("motor_node"), "pwm too fast!!!");
  }
  else if (pwm > pwm_limit)
    output = pwm_limit;
  else if (pwm < 0)
  {
    output = 0;
    RCLCPP_WARN(rclcpp::get_logger("motor_node"), "trash value!!!");
  }
  else
    output = pwm;
  return output;
}

void CalculateRpm()
{
  rpm_value1 = (speed_count_1 * (60 * control_cycle)) / (encoder_resolution * 4);
  speed_count_1 = 0;
  rpm_value2 = (speed_count2 * (60 * control_cycle)) / (encoder_resolution * 4);
  speed_count2 = 0;

  // // //LOw-pass filter
  // if(fabs(rpm_value2) < dzn_lower_limit || fabs(rpm_value2) > dzn_upper_limit) {rpm_value2 = 0;}
  // else
  // {
  //     if( rpm_value2 > 0){rpm_value2 -= dzn_lower_limit;}
  //     else{rpm_value2 += dzn_lower_limit;}
      
  //     rpm_value2 = (1 - coeff_ext_filter) * rpm_value2_fold + coeff_ext_filter * rpm_value2;
  //     rpm_value2_fold = rpm_value2;
  // }
}


void pidControlSystem1(double &goal, double &curr, double &pControl, double &iControl, double &dControl, double &p_gain, double &i_gain, double &d_gain, double &pidControl, double &derivative, double &lastderivative, double &last_input)
{
  double error = goal - curr;

  pControl = p_gain * error;


  double ACC_ERROR_MIN = -50.0;
  double ACC_ERROR_MAX = 50.0;

  iControl += (error * i_gain) * TIME;
  iControl = std::min(std::max(iControl, ACC_ERROR_MIN), ACC_ERROR_MAX);

  double filter = 7.9577e-3;

  derivative = (goal - last_input) / TIME;
  derivative = lastderivative + (TIME / (filter + TIME)) * (derivative - lastderivative);
  last_input = goal;
  lastderivative = derivative;

  dControl = d_gain * derivative;

  double ACC_ERROR_MIN_d = -50.0;
  double ACC_ERROR_MAX_d = 50.0;
  dControl = std::min(std::max(dControl, ACC_ERROR_MIN_d), ACC_ERROR_MAX_d);
  
  pidControl = pControl + iControl + dControl;
}

void pidControlSystem2(double &goal, double &curr, double &pControl, double &iControl, double &dControl, double &p_gain, double &i_gain, double &d_gain, double &pidControl, double &derivative, double &lastderivative, double &last_input)
{
  double error = goal - curr;

  pControl = p_gain * error;


  double ACC_ERROR_MIN = -50.0;
  double ACC_ERROR_MAX = 50.0;

  iControl += (error * i_gain) * TIME;
  iControl = std::min(std::max(iControl, ACC_ERROR_MIN), ACC_ERROR_MAX);

  double filter = 7.9577e-3;

  derivative = (goal - last_input) / TIME;
  derivative = lastderivative + (TIME / (filter + TIME)) * (derivative - lastderivative);
  last_input = goal;
  lastderivative = derivative;

  dControl = d_gain * derivative;

  double ACC_ERROR_MIN_d = -50.0;
  double ACC_ERROR_MAX_d = 50.0;
  dControl = std::min(std::max(dControl, ACC_ERROR_MIN_d), ACC_ERROR_MAX_d);
  
  pidControl = pControl + iControl + dControl;
}

void InfoMotors()
{
  CalculateRpm();
  printf("\033[2J");
  printf("\033[1;1H");
  printf("Encoder1A : %5d    ||  Encoder2A : %5d\n", encoder_count_1A, encoder_count_2A);
  printf("Encoder1B : %5d    ||  Encoder2B : %5d\n", encoder_count_1B, encoder_count_2B);
  printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", rpm_value1, rpm_value2);
  printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_pwm1, current_pwm2);
  printf("DIR1 :%11s    ||  DIR2 :%11s\n", current_direction1 ? "CW" : "CCW", current_direction2 ? "CW" : "CCW");
  printf("ACC  :%11.0d\n", acceleration);
  printf("wheel_radius : %11.0f\n", wheel_radius);
  printf("robot_radius  :%11.0f\n", robot_radius);
  printf("encoder_resolution  :%11.0d\n", encoder_resolution);
  printf("pwm_frequency  :%11.0d\n", pwm_frequency);
  printf("control_cycle  :%11.0f\n", control_cycle);
  printf("linear_vel1 : %10.0f    ||  linear_vel2 : %10.0f\n", linear_vel1, linear_vel2);
  // printf("odom_l: %10.0f || odom_r: %10.0f\n",odo[0], odo[1]);
  printf("left_speed: %10.0f || right_speed: %10.0f\n",left_speed, right_speed);

  printf("encoder_count_1B  :%5d\n", encoder_count_1B);

  // printf("linear : %10.0f    ||  angular : %10.0f\n", vw[0], vw[1]);

  printf("\n");

  if (current_direction1 == true)
  {
    topic_rpm_value1 = rpm_value1;
  }
  else
  {
    topic_rpm_value1 = -1 * rpm_value1;
  }

  if (current_direction2 == false)
  {
    topic_rpm_value2 = rpm_value2;
  }
  else
  {
    topic_rpm_value2 = -1 * rpm_value2;
  }

}


RosCommunicator::RosCommunicator()
    : Node("tutorial_ros2_motor"), count_(0)
{

  publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("motor_packet", 10); // Publisher 생성
  timer_ = this->create_wall_timer(
      100ms, std::bind(&RosCommunicator::TimerCallback, this));
  
  sub_vel = this->create_subscription<geometry_msgs::msg::Twist>("motor_input", 10, std::bind(&RosCommunicator::cmdCallback, this, std::placeholders::_1));

  pub_pid_check_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("rpm", 10);
  sub_pid_gain_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("pid_gain", 10, std::bind(&RosCommunicator::pidGainCallback, this, std::placeholders::_1));
}

void RosCommunicator::TimerCallback()
{

  double vw[2];
  vw[0] = (linear_vel1 - linear_vel2) / (robot_radius * 2) * 1000; //angular
  vw[1] = (linear_vel1 + linear_vel2) / 2.0; //linear vel

  double encod[2];
  encod[0] = SumMotor1Encoder();
  encod[1] = SumMotor2Encoder();

  double odo[2];
  odo[0] = (encod[0] / (encoder_resolution * 4)) * wheel_round; // 1번
  odo[1] = (encod[1] / (encoder_resolution * 4)) * wheel_round; //2번

  double pwml = current_pwm1; //1번
  double pwmr = current_pwm2; //2번

  std_msgs::msg::Float64MultiArray motor_data; 
  motor_data.data = std::vector<double>{vw[0], vw[1], encod[0], encod[1], odo[0], odo[1], pwml, pwmr};

  publisher_->publish(motor_data);
  InfoMotors();
  // DirectMotorControl();
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {topic_rpm_value1, topic_rpm_value2, left_rpm, right_rpm};
  pub_pid_check_->publish(msg);
  PidMotorControl();


}

void RosCommunicator::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double linear_vel = msg->linear.x;
  double angular_vel = msg->angular.z;

  double left_speed = linear_vel - (angular_vel * robot_radius * 0.001);
  double right_speed = linear_vel + (angular_vel * robot_radius * 0.001);

  left_rpm = (left_speed * 60) / ( 2 * PI * wheel_radius);
  right_rpm = (right_speed * 60) / (2 * PI * wheel_radius);
}

void RosCommunicator::DirectMotorControl()
{

  // 왼쪽 모터 제어
  if(left_rpm < 0)
  {
    MotorController(2, true, -left_rpm); // 회전 방향을 반대로 하고, 회전 속도는 절댓값으로 설정
  }
  else if (left_rpm > 0)
  {
    MotorController(2, false, left_rpm); // 회전 방향과 회전 속도를 그대로 설정
  }

  else if(left_rpm == 0)
  {
    MotorController(2, false, 0);
    left_rpm = 0;
  }

  // 오른쪽 모터 제어
  if(right_rpm < 0)
  {
    MotorController(1, false, -right_rpm); // 회전 방향을 반대로 하고, 회전 속도는 절댓값으로 설정
  }

  else if (right_rpm >0)
  {
    MotorController(1, true, right_rpm); // 회전 방향과 회전 속도를 그대로 설정
  }

  else if(right_rpm == 0)
  {
    MotorController(1, true, 0);
    right_rpm = 0;
  }
}

void RosCommunicator::PidMotorControl()
{

  // 왼쪽
  if(left_rpm < 0)
  { 
    left_rpm = abs(left_rpm);
    pidControlSystem2(left_rpm, rpm_value1, pControl2, iControl2, dControl2, p_gain2, i_gain2, d_gain2, pidControl2, derivative2, lastderivative2, last_input2);

    target_pwm2 +=pidControl2;
    // target_pwm1 = abs(target_pwm1);
    // target_pwm1 = abs(pidControl1);

    // MotorController(2, true, target_pwm2); // 회전 방향을 반대로 하고, 회전 속도는 절댓값으로 설정
    MotorController(1, false, target_pwm2); // 회전 방향을 반대로 하고, 회전 속도는 절댓값으로 설정


  }
  else if (left_rpm > 0)
  {

    pidControlSystem2(left_rpm, rpm_value1, pControl2, iControl2, dControl2, p_gain2, i_gain2, d_gain2, pidControl2, derivative2, lastderivative2, last_input2);
    target_pwm2 +=pidControl2;
    

    MotorController(1, true, target_pwm2); // 회전 방향과 회전 속도를 그대로 설정
    // MotorController(2, false, target_pwm2); // 회전 방향과 회전 속도를 그대로 설정
  }

  else if(left_rpm == 0)
  {
    MotorController(1, true, 0);
    target_pwm2 = 0;
  }

  // 오른쪽 
  if(right_rpm < 0)
  {  
    right_rpm = abs(right_rpm);
    pidControlSystem1(right_rpm, rpm_value2, pControl1, iControl1, dControl1, p_gain1, i_gain1, d_gain1, pidControl1, derivative1, lastderivative1, last_input1);
    target_pwm1 +=pidControl1;
    // target_pwm2 = abs(target_pwm2 + pidControl2);
    // target_pwm2 = abs(target_pwm2);
    MotorController(2, true, target_pwm1); // 회전 방향과 회전 속도를 그대로 설정

    // MotorController(1, false, target_pwm1); // 회전 방향을 반대로 하고, 회전 속도는 절댓값으로 설정
  }

  else if (right_rpm >0)
  {
    pidControlSystem1(right_rpm, rpm_value2, pControl1, iControl1, dControl1, p_gain1, i_gain1, d_gain1, pidControl1, derivative1, lastderivative1, last_input1);
    target_pwm1 += pidControl1;

    MotorController(2, false, target_pwm1); // 회전 방향과 회전 속도를 그대로 설정

    // MotorController(1, true, target_pwm1); // 회전 방향과 회전 속도를 그대로 설정
  }

  else if(right_rpm == 0)
  {
    MotorController(2, false, 0);
    // MotorController(1, true, 0);
    target_pwm1 = 0;
  }
}

void RosCommunicator::pidGainCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 6)
  {
    RCLCPP_ERROR(this->get_logger(), "Received data size is not correct for pid_gain");
    return;
  }
  // target_rpm1 = msg->data[0];
  // target_rpm1 = msg->data[1];

  p_gain1 = msg->data[0];
  i_gain1 = msg->data[1];
  d_gain1 = msg->data[2];
  p_gain2 = msg->data[3];
  i_gain2 = msg->data[4];
  d_gain2 = msg->data[5];
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  Initialize();
  rclcpp::spin(std::make_shared<RosCommunicator>());

  rclcpp::shutdown();
  MotorController(1, true, 0);
  MotorController(2, true, 0);
  return 0;
}