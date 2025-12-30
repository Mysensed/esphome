#include "sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace duco {

static const char *const TAG = "duco sensor";

// ---- shared debug state (optional) ----
uint16_t DucoCo2Sensor::global_last_value = 0;
uint8_t  DucoCo2Sensor::global_last_sensor_id = 0;
uint32_t DucoCo2Sensor::last_dup_log_at_ = 0;

// ------------------------------------------------------------

void DucoCo2Sensor::set_address(uint8_t address) {
  this->address_ = address;
}

void DucoCo2Sensor::setup() {
  ESP_LOGD(TAG, "Setting up CO2 sensor at address 0x%02X", this->address_);
}

float DucoCo2Sensor::get_setup_priority() const {
  return setup_priority::DATA;
}

void DucoCo2Sensor::update() {
  // Request CO2 value from this specific sensor
  this->parent_->request_sensor_value(this, this->address_);
}

void DucoCo2Sensor::receive_response(const DucoMessage &message) {
  // Function 0x12 = CO2 response
  if (message.function != 0x12)
    return;

  uint8_t reported_addr = message.data[1];

  // ------------------------------------------------------------------
  // 🔒 STRICT ADDRESS CHECK (CORE FIX)
  //
  // Forwarded RF frames do NOT preserve origin sensor identity.
  // Therefore we must only accept responses that match our address.
  // ------------------------------------------------------------------
  if (reported_addr != this->address_) {
    ESP_LOGW(TAG,
      "[IGNORED] CO2 response for addr 0x%02X delivered to sensor 0x%02X (Msg ID 0x%02X)",
      reported_addr,
      this->address_,
      message.id
    );

    // IMPORTANT: always stop waiting to avoid stalling the bus
    this->parent_->stop_waiting(message.id);
    return;
  }

  // DUCO CO2 value is little-endian in bytes 4+5
  uint16_t value = (message.data[5] << 8) | message.data[4];
  uint32_t now = millis();

  // ------------------------------------------------------------------
  // Optional debug: sudden jumps (per-sensor)
  // ------------------------------------------------------------------
  if (this->last_value_ != 0) {
    uint16_t diff = abs((int)value - (int)this->last_value_);
    if (diff > 300 && (now - this->last_jump_log_at_) > 10000) {
      ESP_LOGW(TAG,
        "[SUDDEN JUMP] Sensor 0x%02X | Old: %u | New: %u | Raw: [%02X %02X %02X %02X %02X %02X %02X %02X]",
        this->address_,
        this->last_value_,
        value,
        message.data[0], message.data[1], message.data[2], message.data[3],
        message.data[4], message.data[5], message.data[6], message.data[7]
      );
      this->last_jump_log_at_ = now;
    }
  }

  // ------------------------------------------------------------------
  // Optional debug: mirrored values between sensors
  // ------------------------------------------------------------------
  if (value == global_last_value &&
      this->address_ != global_last_sensor_id &&
      (now - last_dup_log_at_) > 10000) {

    ESP_LOGW(TAG,
      "[MIRRORING] Sensor 0x%02X mirrors sensor 0x%02X | Value: %u",
      this->address_,
      global_last_sensor_id,
      value
    );
    last_dup_log_at_ = now;
  }

  // ------------------------------------------------------------------
  // Publish value (sanity bounds)
  // ------------------------------------------------------------------
  this->last_value_ = value;
  global_last_value = value;
  global_last_sensor_id = this->address_;

  if (value >= 300 && value <= 10000) {
    this->publish_state(value);
  } else {
    ESP_LOGW(TAG,
      "[OUT OF RANGE] Sensor 0x%02X reported %u ppm",
      this->address_,
      value
    );
  }

  // Always stop waiting after handling response
  this->parent_->stop_waiting(message.id);
}

}  // namespace duco
}  // namespace esphome
