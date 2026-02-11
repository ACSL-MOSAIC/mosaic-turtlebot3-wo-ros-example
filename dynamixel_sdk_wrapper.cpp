// Copyright 2019 ROBOTIS CO., LTD.
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
//
// Author: Darby Lim

#include "dynamixel_sdk_wrapper.hpp"

#include <algorithm>
#include <iostream>
#include <string>

using robotis::turtlebot3::DynamixelSDKWrapper;

DynamixelSDKWrapper::DynamixelSDKWrapper(const Device &device)
  : device_(device) {
  if (init_dynamixel_sdk_handlers() == false) {
    std::cout << "Failed to initialize SDK handlers" << std::endl;
    return;
  } else {
    std::cout << "Success to initilize SDK handlers" << std::endl;
  }
}

DynamixelSDKWrapper::~DynamixelSDKWrapper() {
  portHandler_->closePort();
}

bool DynamixelSDKWrapper::is_connected_to_device() {
  uint8_t data[2];
  return this->read_register(device_.id, 0, 2, &data[0]);
}

void DynamixelSDKWrapper::init_read_memory(const uint16_t &start_addr, const uint16_t &length) {
  read_memory_.start_addr = start_addr;
  read_memory_.length = length;
  read_memory_.data = &read_data_[0];
}

void DynamixelSDKWrapper::read_data_set() {
  const char *log = NULL;
  bool ret = this->read_register(
    device_.id,
    read_memory_.start_addr,
    read_memory_.length,
    &read_data_buffer_[0],
    &log);

  if (ret == false) {
    std::cout << "DynamixelSDKWrapper Failed to read[" << log << "]" << std::endl;
  } else {
    std::lock_guard<std::mutex> lock(read_data_mutex_);
    std::copy(read_data_buffer_, read_data_buffer_ + READ_DATA_SIZE, read_data_);
    std::cout << "DynamixelSDKWrapper Succeeded to read" << std::endl;
  }
}

bool DynamixelSDKWrapper::set_data_to_device(
  const uint16_t &addr,
  const uint16_t &length,
  uint8_t *get_data,
  std::string *msg) {
  const char *log = nullptr;
  bool ret = false;

  std::lock_guard<std::mutex> lock(write_data_mutex_);
  ret = write_register(device_.id, addr, length, get_data, &log);

  if (ret == true) {
    *msg = "Succeeded to write data";
    return true;
  } else {
    *msg = "Failed to write data" + std::string(log);
    return false;
  }

  return ret;
}

bool DynamixelSDKWrapper::init_dynamixel_sdk_handlers() {
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_.usb_port.c_str());
  packetHandler_ =
      dynamixel::PacketHandler::getPacketHandler(static_cast<int>(device_.protocol_version));

  if (portHandler_->openPort()) {
    std::cout << "DynamixelSDKWrapper Succeeded to open the port(" << device_.usb_port.c_str() << ")" << std::endl;
  } else {
    std::cout << "DynamixelSDKWrapper Failed to open the port(" << device_.usb_port.c_str() << ")!" << std::endl;
    return false;
  }

  if (portHandler_->setBaudRate(static_cast<int>(device_.baud_rate))) {
    std::cout << "DynamixelSDKWrapper Succeeded to change the baudrate!" << std::endl;
  } else {
    std::cout << "DynamixelSDKWrapper Failed to change the baudrate(" << device_.baud_rate << ")!" << std::endl;
    return false;
  }

  return true;
}

bool DynamixelSDKWrapper::read_register(
  uint8_t id,
  uint16_t address,
  uint16_t length,
  uint8_t *data_basket,
  const char **log) {
  std::lock_guard<std::mutex> lock(sdk_mutex_);

  int32_t dxl_comm_result = COMM_RX_FAIL;
  uint8_t dxl_error = 0;

  dxl_comm_result = packetHandler_->readTxRx(
    portHandler_,
    id,
    address,
    length,
    data_basket,
    &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    if (log != NULL) { *log = packetHandler_->getTxRxResult(dxl_comm_result); }
    return false;
  } else if (dxl_error != 0) {
    if (log != NULL) { *log = packetHandler_->getRxPacketError(dxl_error); }
    return false;
  } else {
    return true;
  }

  return false;
}

bool DynamixelSDKWrapper::write_register(
  uint8_t id,
  uint16_t address,
  uint16_t length,
  uint8_t *data,
  const char **log) {
  std::lock_guard<std::mutex> lock(sdk_mutex_);

  int32_t dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  dxl_comm_result = packetHandler_->writeTxRx(
    portHandler_,
    id,
    address,
    length,
    data,
    &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    if (log != NULL) { *log = packetHandler_->getTxRxResult(dxl_comm_result); }
    return false;
  } else if (dxl_error != 0) {
    if (log != NULL) { *log = packetHandler_->getRxPacketError(dxl_error); }
    return false;
  } else {
    return true;
  }

  return false;
}
