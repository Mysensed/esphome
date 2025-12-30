#include "sensor.h"
#include "duco.h"
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
