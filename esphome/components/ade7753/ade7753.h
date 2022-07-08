#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace ade7753 {

class ADE7753 : public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                                 spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_2MHZ>, public PollingComponent {
 public:
  void set_voltage_sensor(sensor::Sensor *voltage_sensor) { voltage_sensor_ = voltage_sensor; }
  void set_current_a_sensor(sensor::Sensor *current_a_sensor) { current_a_sensor_ = current_a_sensor; }
  void set_active_power_a_sensor(sensor::Sensor *active_power_a_sensor) {
    active_power_a_sensor_ = active_power_a_sensor;
  }


  void setup() override {
    this->set_timeout(100, [this]() {
      this->ade_write_8_(0x0010, 0x04);
      this->ade_write_8_(0x00FE, 0xAD);
      this->ade_write_16_(0x0120, 0x0030);
      this->is_setup_ = true;
    });
  }

  void dump_config() override;

  void update() override;

 protected:
  spi::ErrorCode ade_write_8_(uint16_t reg, uint8_t value) {
    std::vector<uint8_t> data;
    data.push_back(reg >> 8);
    data.push_back(reg >> 0);
    data.push_back(value);
    return write(data.data(), data.size());
  }
  spi::ErrorCode ade_write_16_(uint16_t reg, uint16_t value) {
    std::vector<uint8_t> data;
    data.push_back(reg >> 8);
    data.push_back(reg >> 0);
    data.push_back(value >> 8);
    data.push_back(value >> 0);
    return write(data.data(), data.size());
  }
  spi::ErrorCode ade_write_32_(uint16_t reg, uint32_t value) {
    std::vector<uint8_t> data;
    data.push_back(reg >> 8);
    data.push_back(reg >> 0);
    data.push_back(value >> 24);
    data.push_back(value >> 16);
    data.push_back(value >> 8);
    data.push_back(value >> 0);
    return write(data.data(), data.size());
  }
  spi::ErrorCode ade_read_32_(uint16_t reg, uint32_t *value) {
    uint8_t reg_data[2];
    reg_data[0] = reg >> 8;
    reg_data[1] = reg >> 0;
    spi::ErrorCode err = write(reg_data, 2);
    if (err != spi::ERROR_OK)
      return err;
    uint8_t recv[4];
    err = read(recv, 4);
    if (err != spi::ERROR_OK)
      return err;
    *value = 0;
    *value |= ((uint32_t) recv[0]) << 24;
    *value |= ((uint32_t) recv[1]) << 16;
    *value |= ((uint32_t) recv[2]) << 8;
    *value |= ((uint32_t) recv[3]);
    return spi::ERROR_OK;
  }

  bool is_setup_{false};
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_a_sensor_{nullptr};
  sensor::Sensor *active_power_a_sensor_{nullptr};
};

}  // namespace ade7753
}  // namespace esphome
