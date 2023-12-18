// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
// This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
// For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
// To test this example, please follow the commands below.
//
// Open terminal #1
// $ ros2 run dynamixel_sdk_examples read_write_node
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 1000}"
// $ ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
//
// Author: Will Son
*******************************************************************************/

#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"




#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
// #include "dynamixel_sdk_custom_interfaces/srv/get_gesture.hpp"
#include "hyrian_interfaces/srv/get_gesture.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_srvs/srv/set_bool.hpp"

#include "hyrian_motion/read_write_node.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_POSITION 132


#define DXL1_ID                         0
#define DXL2_ID                         1
// Protocol version



#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"




dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;


uint8_t dxl_error = 0;
uint32_t velocity_value = 10;

int dxl_comm_result = COMM_TX_FAIL;

std::string gesture;


class GestureNode : public rclcpp::Node
{
public:
  GestureNode(): Node("gesture_node")
  {
    service_ = this->create_service<hyrian_interfaces::srv::GetGesture>(
      "gesture_service",
      std::bind(&GestureNode::handle_service, this, std::placeholders::_1, std::placeholders::_2)
    );
  }
private:
  void handle_service(
    const std::shared_ptr<hyrian_interfaces::srv::GetGesture::Request> request,
    std::shared_ptr<hyrian_interfaces::srv::GetGesture::Response> response)
  {
    (void)request;  // 일단 요청 사용 x


      // Position Value of X series is 4 byte data.
      // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
      gesture = request->gesture;  // Convert int32 -> uint32

      if (gesture == "Greeting") { //임시로 지정
    for (int a = 200; a <= 1700; a += 20) {
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            (uint8_t)0,
            ADDR_GOAL_POSITION,
            (uint32_t)a,
            &dxl_error
        );
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            (uint8_t)1,
            ADDR_GOAL_POSITION,
            (uint32_t)a,
            &dxl_error
        );
        std::chrono::milliseconds timespan_hello_2(5);
        std::this_thread::sleep_for(timespan_hello_2);
    }
      RCLCPP_INFO(this->get_logger(), "while loop");
    for (int a = 1700; a >= 200; a -= 20) {
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            (uint8_t)0,
            ADDR_GOAL_POSITION,
            (uint32_t)a,
            &dxl_error
        );
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            (uint8_t)1,
            ADDR_GOAL_POSITION,
            (uint32_t)a,
            &dxl_error
        );
        std::chrono::milliseconds timespan_hello_2(5);
        std::this_thread::sleep_for(timespan_hello_2);
    }


      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", 1, 1000);
      }
    }






     if (gesture=="start") {

      std::chrono::milliseconds timespan_start_6(6000);
      std::this_thread::sleep_for(timespan_start_6);
      for (int a = 200; a <= 1700; a += 20) {
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            (uint8_t)0,
            ADDR_GOAL_POSITION,
            (uint32_t)a,
            &dxl_error
        );
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            (uint8_t)1,
            ADDR_GOAL_POSITION,
            (uint32_t)a,
            &dxl_error
        );
        std::chrono::milliseconds timespan_start_2(5);
        std::this_thread::sleep_for(timespan_start_2);
    }



      for (int a = 1700; a >= 200; a -= 20) {
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            (uint8_t)0,
            ADDR_GOAL_POSITION,
            (uint32_t)a,
            &dxl_error
        );
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            (uint8_t)1,
            ADDR_GOAL_POSITION,
            (uint32_t)a,
            &dxl_error
        );
        std::chrono::milliseconds timespan_hello_2(5);
        std::this_thread::sleep_for(timespan_hello_2);
    }



     for (int a = 200; a <= 1700; a += 20) {
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            (uint8_t)0,
            ADDR_GOAL_POSITION,
            (uint32_t)a,
            &dxl_error
        );
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            (uint8_t)1,
            ADDR_GOAL_POSITION,
            (uint32_t)a,
            &dxl_error
        );
        std::chrono::milliseconds timespan_start_2(5);
        std::this_thread::sleep_for(timespan_start_2);
    }

    for (int a = 1700; a >= 200; a -= 20) {
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            (uint8_t)0,
            ADDR_GOAL_POSITION,
            (uint32_t)a,
            &dxl_error
        );
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            (uint8_t)1,
            ADDR_GOAL_POSITION,
            (uint32_t)a,
            &dxl_error
        );
        std::chrono::milliseconds timespan_hello_2(5);
        std::this_thread::sleep_for(timespan_hello_2);
    }




      //std::chrono::milliseconds timespan_2(6000);
      std::this_thread::sleep_for(timespan_start_6);
      for (int a = 200; a <= 1700; a += 20) {

        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            (uint8_t)1,
            ADDR_GOAL_POSITION,
            (uint32_t)a,
            &dxl_error
        );
        std::chrono::milliseconds timespan_start_2(5);
        std::this_thread::sleep_for(timespan_start_2);
    }

      std::this_thread::sleep_for(timespan_start_6);
      for (int a = 1700; a >= 200; a -= 20) {

        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            (uint8_t)1,
            ADDR_GOAL_POSITION,
            (uint32_t)a,
            &dxl_error
        );
        std::chrono::milliseconds timespan_hello_2(5);
        std::this_thread::sleep_for(timespan_hello_2);
    }


      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", 1, 1000);
      }
    }

    if (gesture=="handup") {


      for (int a = 200; a <= 1700; a += 20) {

        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            (uint8_t)1,
            ADDR_GOAL_POSITION,
            (uint32_t)a,
            &dxl_error
        );
        std::chrono::milliseconds timespan_start_2(5);
        std::this_thread::sleep_for(timespan_start_2);
    }
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", 1, 1000);
      }
    }
      if (gesture=="handdown") {


      for (int a =1700; a >= 240; a -= 20) {

        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            (uint8_t)1,
            ADDR_GOAL_POSITION,
            (uint32_t)a,
            &dxl_error
        );
        std::chrono::milliseconds timespan_start_2(5);
        std::this_thread::sleep_for(timespan_start_2);
    }
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", 1, 1000);
      }
    }



      response->finish = "happy";
    }




  rclcpp::Service<hyrian_interfaces::srv::GetGesture>::SharedPtr service_;
};



void setupDynamixel(uint8_t dxl_id)
{
  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    3,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Position Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );




  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }
}





int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  std::chrono::milliseconds timespan_1(2000);
  std::chrono::milliseconds timespan_2(2000);
  // Open Serial Port
  dxl_comm_result = portHandler->openPort();                                   
  if (dxl_comm_result == false) {

    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
  }


  setupDynamixel(BROADCAST_ID);
  rclcpp::init(argc, argv);
  auto gesture_node = std::make_shared<GestureNode>();

  rclcpp::spin(gesture_node);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );


  return 0;
}
