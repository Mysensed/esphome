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

// void DucoCo2Sensor::receive_response(const DucoMessage &message) {
//   if (message.function == 0x12) {
//     // ---- ADD THIS DEBUG LINE ----
//     ESP_LOGD("duco_co2", "CO2 message received: id=0x%X data[0..7]=%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X",
//              message.id,
//              message.data.size() > 0 ? message.data[0] : 0,
//              message.data.size() > 1 ? message.data[1] : 0,
//              message.data.size() > 2 ? message.data[2] : 0,
//              message.data.size() > 3 ? message.data[3] : 0,
//              message.data.size() > 4 ? message.data[4] : 0,
//              message.data.size() > 5 ? message.data[5] : 0,
//              message.data.size() > 6 ? message.data[6] : 0,
//              message.data.size() > 7 ? message.data[7] : 0
//     );
//     // ------------------------------
//     uint16_t co2_value = (message.data[5] << 8) + message.data[4];
//     // only publish the state if the co2 value is below 10000 or above 300
//     // otherwise the value is likely invalid
//     if (co2_value <= 10000 && co2_value >= 300)
//       publish_state(co2_value);

//     this->parent_->stop_waiting(message.id);
//   }
// }

void DucoCo2Sensor::receive_response(const DucoMessage &message) {
  if (message.function != 0x12)
    return;

  uint32_t now = millis();

  // ---- DEBUG: basic message info ----
  ESP_LOGD("duco_co2",
           "RX CO2 message: id=0x%02X data_len=%d",
           message.id,
           message.data.size());

  // ---- DEBUG: dump raw data (max 16 bytes) ----
  char hexbuf[3 * 16] = {0};
  size_t pos = 0;
  for (size_t i = 0; i < message.data.size() && i < 16; i++) {
    pos += snprintf(hexbuf + pos, sizeof(hexbuf) - pos,
                    "%02X%s",
                    message.data[i],
                    (i + 1 < message.data.size()) ? "." : "");
  }
  ESP_LOGD("duco_co2", "RX raw data: %s", hexbuf);

  // ---- Sanity check ----
  if (message.data.size() < 6) {
    ESP_LOGW("duco_co2",
             "CO2 message too short (len=%d) from id=0x%02X",
             message.data.size(),
             message.id);
    return;
  }

  // ---- Decode CO2 ----
  uint16_t co2_value = (message.data[5] << 8) | message.data[4];

  ESP_LOGD("duco_co2",
           "Decoded CO2: %u ppm (id=0x%02X)",
           co2_value,
           message.id);

  // ============================================================
  // DEBUG STATE
  // ============================================================
  static uint16_t last_co2_by_id[256] = {0};
  static uint32_t last_seen_ms_by_id[256] = {0};
  static uint32_t last_dup_warn_ms_by_id[256] = {0};

  static uint16_t last_co2_global = 0;
  static uint8_t  last_id_global = 0;
  static uint32_t last_global_ms = 0;

  static uint32_t last_jump_warn_ms = 0;
  static uint32_t last_forward_warn_ms = 0;

  // ============================================================
  // DEBUG 1: same-ID duplicate values (baseline / heartbeat)
  // ============================================================
  if (last_co2_by_id[message.id] == co2_value) {
    if (now - last_dup_warn_ms_by_id[message.id] > 300000) { // 5 min
      ESP_LOGW("duco_co2",
               "Duplicate CO2 value %u ppm from same id=0x%02X",
               co2_value,
               message.id);
      last_dup_warn_ms_by_id[message.id] = now;
    }
  }

  last_co2_by_id[message.id] = co2_value;
  last_seen_ms_by_id[message.id] = now;

  // ============================================================
  // DEBUG 2: forwarded / proxied data detection (Case C)
  // Same CO2 value from DIFFERENT IDs within short time
  // ============================================================
  if (last_co2_global == co2_value &&
      last_id_global != message.id &&
      (now - last_global_ms) < 30000) {

    if (now - last_forward_warn_ms > 300000) { // 5 min
      ESP_LOGW("duco_co2",
               "Possible forwarded CO2 value: %u ppm from id=0x%02X "
               "(previously id=0x%02X, dt=%lu ms)",
               co2_value,
               message.id,
               last_id_global,
               (unsigned long)(now - last_global_ms));
      last_forward_warn_ms = now;
    }
  }

  // ============================================================
  // DEBUG 3: implausible interleaving jumps (A/B switching)
  // ============================================================
  if (last_co2_global != 0) {
    uint32_t dt = now - last_global_ms;
    int diff = abs((int)co2_value - (int)last_co2_global);

    if (dt < 60000 && diff > 300) {
      if (now - last_jump_warn_ms > 300000) { // 5 min
        ESP_LOGW("duco_co2",
                 "Implausible CO2 jump: %u -> %u ppm in %lu ms "
                 "(id=0x%02X, prev_id=0x%02X)",
                 last_co2_global,
                 co2_value,
                 (unsigned long)dt,
                 message.id,
                 last_id_global);
        last_jump_warn_ms = now;
      }
    }
  }

  // ---- Update global history ----
  last_co2_global = co2_value;
  last_id_global = message.id;
  last_global_ms = now;

  // ---- Publish (UNCHANGED behavior) ----
  if (co2_value >= 300 && co2_value <= 10000) {
    publish_state(co2_value);
  } else {
    ESP_LOGW("duco_co2",
             "CO2 value %u ppm out of range, not published (id=0x%02X)",
             co2_value,
             message.id);
  }

  this->parent_->stop_waiting(message.id);
}



void DucoCo2Sensor::set_address(uint8_t address) { this->address_ = address; }

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
    // only publish the state if the co2 value is below 10000
    // otherwise the value is likely invalid
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
    // only publish the state if the co2 value is below 10000
    // otherwise the value is likely invalid
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
    // only publish the state if the temperature value is reasonable
    // otherwise the value is likely invalid
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
    // only publish the state if the co2 value is below 10000
    // otherwise the value is likely invalid
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
