#pragma once

#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "../duco.h"

namespace esphome {
namespace duco {

class DucoCo2Sensor : public DucoDevice, public PollingComponent, public sensor::Sensor {
 public:
  void setup() override;
  void update() override;

  float get_setup_priority() const override;

  void receive_response(const DucoMessage &message) override;

  void set_address(uint8_t address);

 protected:
  uint8_t address_;
  
  // --- New Debug Variables ---
  
  // These are unique to EACH sensor
  uint16_t last_value_ = 0;           // Remembers what this sensor last said
  uint32_t last_jump_log_at_ = 0;    // Remembers when we last complained about a jump

  // 'static' means these are SHARED between Sensor A and Sensor B
  static uint16_t global_last_value;      // The last value any sensor reported
  static uint8_t global_last_sensor_id;   // Which sensor reported it
  static uint32_t last_dup_log_at_;       // When we last complained about duplicates
};

class DucoHumiditySensor : public DucoDevice, public PollingComponent, public sensor::Sensor {
 public:
  void setup() override;
  void update() override;

  float get_setup_priority() const override;

  void receive_response(const DucoMessage &message) override;

  void set_address(uint8_t address);

 protected:
  uint8_t address_;
};

class DucoTemperatureSensor : public DucoDevice, public PollingComponent, public sensor::Sensor {
 public:
  void setup() override;
  void update() override;

  float get_setup_priority() const override;

  void receive_response(const DucoMessage &message) override;

  void set_address(uint8_t address);

 protected:
  uint8_t address_;
};

class DucoBoxTemperatureSensor : public DucoDevice, public PollingComponent, public sensor::Sensor {
 public:
  void setup() override;
  void update() override;

  float get_setup_priority() const override;

  void receive_response(const DucoMessage &message) override;

  void set_type(uint8_t type);

 protected:
  uint8_t address_;
  uint8_t type_;
};

class DucoBypassSensor : public DucoDevice, public PollingComponent, public sensor::Sensor {
 public:
  void setup() override;
  void update() override;

  float get_setup_priority() const override;

  void receive_response(const DucoMessage &message) override;
};

class DucoFilterRemainingSensor : public DucoDevice, public PollingComponent, public sensor::Sensor {
 public:
  void setup() override;
  void update() override;

  float get_setup_priority() const override;

  void receive_response(const DucoMessage &message) override;
};

class DucoFlowLevelSensor : public DucoDevice, public PollingComponent, public sensor::Sensor {
 public:
  void setup() override;
  void update() override;

  float get_setup_priority() const override;

  void receive_response(const DucoMessage &message) override;
};

class DucoStateTimeRemainingSensor : public DucoDevice, public PollingComponent, public sensor::Sensor {
 public:
  void setup() override;
  void update() override;

  float get_setup_priority() const override;

  void receive_response(const DucoMessage &message) override;
};

}  // namespace duco
}  // namespace esphome
