#include "sensor.h"
#include "../duco.h"
#include <vector>

namespace esphome {
namespace duco {

static const char *const TAG = "duco sensor";

void DucoCo2Sensor::setup() {}

void DucoCo2Sensor::update() {
  DucoMessage message;
  message.function = 0x10;
  message.data = {0x01, address_, 0x00, 0x49, 0x04};
  this->parent_->send(message, this);
}

float DucoCo2Sensor::get_setup_priority() const {
  // After DUCO
  return setup_priority::BUS - 2.0f;
}

// STATIC GLOBALS:
// Initialize static variables (place this in your .cpp file outside the function)
uint16_t DucoCo2Sensor::global_last_value = 0;
uint8_t DucoCo2Sensor::global_last_sensor_id = 0;
uint32_t DucoCo2Sensor::last_dup_log_at_ = 0;

// Map sensor address to instance for routing forwarded data
static DucoCo2Sensor* co2_sensor_map[256] = {nullptr};

void DucoCo2Sensor::receive_response(const DucoMessage &message) {
  if (message.function == 0x12) {
    // Parse CO2 value from response
    uint16_t co2_value = (message.data[5] << 8) + message.data[4];
    // Identify which sensor address the data came from
    uint8_t resp_addr = message.data[1];
    uint32_t now = millis();

    // If the response address is different, route to the correct sensor
    if (resp_addr != this->address_) {
      DucoCo2Sensor* other = co2_sensor_map[resp_addr];
      if (other) {
        ESP_LOGW(TAG, "Forwarded CO2: %u ppm for sensor 0x%02X (via 0x%02X)", co2_value, resp_addr, this->address_);
        // Publish on the correct sensor
        if (co2_value <= 10000 && co2_value >= 300) {
          other->publish_state(co2_value);
        }
        // Update history for the correct sensor and global
        other->last_value_ = co2_value;
        DucoCo2Sensor::global_last_value = co2_value;
        DucoCo2Sensor::global_last_sensor_id = resp_addr;
      } else {
        ESP_LOGW(TAG, "Received CO2 for unknown sensor: 0x%02X", resp_addr);
      }
      // Stop waiting (for the sensor that sent us here) and exit
      this->parent_->stop_waiting(message.id);
      return;
    }

    //--- START DEBUG (as before) ---
    // Create a "Hex Dump" of the raw data to see the full packet
    char raw_data_str[30];
    snprintf(raw_data_str, sizeof(raw_data_str), "%02X %02X %02X %02X %02X %02X %02X %02X",
             message.data[0], message.data[1], message.data[2], message.data[3],
             message.data[4], message.data[5], message.data[6], message.data[7]);

    // --- DEBUG 1: JUMP DETECTION (Is it a swap?) ---
    if (this->last_value_ != 0 && abs(co2_value - this->last_value_) > 200) {
      if (now - this->last_jump_log_at_ > 300000) {
        // Special check: Did we just jump to exactly what the OTHER sensor reported?
        const char* type = (co2_value == global_last_value) ? "CROSS-TALK JUMP" : "SUDDEN JUMP";
        ESP_LOGW("DEBUG", "[%s] Sensor Addr: 0x%02X | Msg ID: 0x%02X | Val: %u | Raw: [%s]", 
                 type, this->address_, message.id, co2_value, raw_data_str);
        this->last_jump_log_at_ = now;
      }
    }

    // --- DEBUG 2: MIRRORING DETECTION ---
    if (co2_value == global_last_value && this->address_ != global_last_sensor_id) {
      if (now - last_dup_log_at_ > 300000) {
        ESP_LOGW("DEBUG", "[MIRRORING] Current Sensor Addr: 0x%02X | Msg ID: 0x%02X | Val: %u | Raw: [%s]", 
                 this->address_, message.id, co2_value, raw_data_str);
        last_dup_log_at_ = now;
      }
    }

    // Update history
    this->last_value_ = co2_value;
    global_last_value = co2_value;
    global_last_sensor_id = this->address_;

    // Original logic: publish state if in valid range
    if (co2_value <= 10000 && co2_value >= 300) {
      publish_state(co2_value);
    }
    this->parent_->stop_waiting(message.id);
  }
}

void DucoCo2Sensor::set_address(uint8_t address) { 
  this->address_ = address; 
  co2_sensor_map[address] = this;
}

void DucoHumiditySensor::setup() {}

void DucoHumiditySensor::update() {
  DucoMessage message;
  message.function = 0x10;
  message.data = {0x01, address_, 0x00, 0x49, 0x04};
  this->parent_->send(message, this);
}

float DucoHumiditySensor::get_setup_priority() const {
  // After DUCO
  return setup_priority::BUS - 2.0f;
}

void DucoHumiditySensor::receive_response(const DucoMessage &message) {
  if (message.function == 0x12) {
    uint16_t rh_value = (message.data[7] << 8) + message.data[6];
    if (rh_value <= 10000)
      publish_state(rh_value / 100.0);
    this->parent_->stop_waiting(message.id);
  }
}

void DucoHumiditySensor::set_address(uint8_t address) { this->address_ = address; }

void DucoTemperatureSensor::setup() {}

void DucoTemperatureSensor::update() {
  DucoMessage message;
  message.function = 0x10;
  message.data = {0x01, address_, 0x00, 0x49, 0x04};
  this->parent_->send(message, this);
}

float DucoTemperatureSensor::get_setup_priority() const {
  // After DUCO
  return setup_priority::BUS - 2.0f;
}

void DucoTemperatureSensor::receive_response(const DucoMessage &message) {
  if (message.function == 0x12) {
    uint16_t temp_value = (message.data[3] << 8) + message.data[2];
    if (temp_value <= 1000)
      publish_state(temp_value / 10.0);
    this->parent_->stop_waiting(message.id);
  }
}

void DucoTemperatureSensor::set_address(uint8_t address) { this->address_ = address; }

void DucoBoxTemperatureSensor::setup() {}

void DucoBoxTemperatureSensor::update() {
  DucoMessage message;
  message.function = 0x24;
  message.data = {0x00, type_, 0x09};
  this->parent_->send(message, this);
}

float DucoBoxTemperatureSensor::get_setup_priority() const {
  // After DUCO
  return setup_priority::BUS - 2.0f;
}

void DucoBoxTemperatureSensor::receive_response(const DucoMessage &message) {
  if (message.function == 0x26) {
    int16_t temp_value = (message.data[4] << 8) + message.data[3];
    if (temp_value <= 1000 && temp_value > -1000)
      publish_state(temp_value / 10.0);
    this->parent_->stop_waiting(message.id);
  }
}

void DucoBoxTemperatureSensor::set_type(uint8_t type) { this->type_ = type; }

void DucoBypassSensor::setup() {}

void DucoBypassSensor::update() {
  DucoMessage message;
  message.function = 0x24;
  message.data = {0x00, 0x10, 0x09};
  this->parent_->send(message, this);
}

float DucoBypassSensor::get_setup_priority() const {
  // After DUCO
  return setup_priority::BUS - 2.0f;
}

void DucoBypassSensor::receive_response(const DucoMessage &message) {
  if (message.function == 0x26) {
    uint16_t bypass_value = message.data[3];
    if (bypass_value <= 100)
      publish_state(bypass_value);
    this->parent_->stop_waiting(message.id);
  }
}

void DucoFilterRemainingSensor::setup() {}

void DucoFilterRemainingSensor::update() {
  DucoMessage message;
  message.function = 0x24;
  message.data = {0x00, 0x30, 0x09};
  this->parent_->send(message, this);
}

float DucoFilterRemainingSensor::get_setup_priority() const {
  // After DUCO
  return setup_priority::BUS - 2.0f;
}

void DucoFilterRemainingSensor::receive_response(const DucoMessage &message) {
  if (message.function == 0x26) {
    uint8_t filter_remaining = message.data[3];
    publish_state(filter_remaining);
    this->parent_->stop_waiting(message.id);
  }
}

void DucoFlowLevelSensor::setup() {}

void DucoFlowLevelSensor::update() {
  DucoMessage message;
  message.function = 0x0c;
  message.data = {0x02, 0x01};
  this->parent_->send(message, this);
}

float DucoFlowLevelSensor::get_setup_priority() const {
  // After DUCO
  return setup_priority::BUS - 2.0f;
}

void DucoFlowLevelSensor::receive_response(const DucoMessage &message) {
  if (message.function == 0x0e) {
    uint8_t flow_level = message.data[2];
    publish_state(flow_level);
    this->parent_->stop_waiting(message.id);
  }
}

void DucoStateTimeRemainingSensor::setup() {}

void DucoStateTimeRemainingSensor::update() {
  DucoMessage message;
  message.function = 0x0c;
  message.data = {0x02, 0x01};
  this->parent_->send(message, this);
}

float DucoStateTimeRemainingSensor::get_setup_priority() const {
  // After DUCO
  return setup_priority::BUS - 2.0f;
}

void DucoStateTimeRemainingSensor::receive_response(const DucoMessage &message) {
  if (message.function == 0x0e) {
    uint16_t time_remaining = (message.data[13] << 8) + message.data[12];
    publish_state(time_remaining);
    this->parent_->stop_waiting(message.id);
  }
}

}  // namespace duco
}  // namespace esphome
