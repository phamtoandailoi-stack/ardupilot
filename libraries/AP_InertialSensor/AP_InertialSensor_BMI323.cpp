/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The BMI323 register sequence and board-axis mapping are based on the
 * MadFlight BMI323 driver (MIT licence, copyright 2026 madflight.com) that was
 * validated on the same F450 flight-controller hardware.
 */

#include "AP_InertialSensor_BMI323.h"

#if AP_INERTIALSENSOR_ENABLED

#include <AP_Math/AP_Math.h>

namespace {

constexpr uint8_t REG_CHIP_ID = 0x00;
constexpr uint8_t REG_ACC_DATA_X = 0x03;
constexpr uint8_t REG_INT_STATUS_INT1 = 0x0D;
constexpr uint8_t REG_ACC_CONF = 0x20;
constexpr uint8_t REG_GYR_CONF = 0x21;
constexpr uint8_t REG_IO_INT_CTRL = 0x38;
constexpr uint8_t REG_INT_CONF = 0x39;
constexpr uint8_t REG_INT_MAP2 = 0x3B;
constexpr uint8_t REG_CMD = 0x7E;

constexpr uint8_t CHIP_ID = 0x43;
constexpr uint16_t CMD_SOFT_RESET = 0xDEAF;

// Normal mode, 800 Hz, +/-16 g and +/-2000 dps.  These exact settings are
// already proven on this board by its MadFlight firmware.
constexpr uint16_t ACC_CONF_800HZ_16G = 0x40BB;
constexpr uint16_t GYR_CONF_800HZ_2000DPS = 0x404B;
constexpr uint16_t IO_INT_CTRL_INT1 = 0x0005;
constexpr uint16_t INT_MAP2_ACC_GYR_DRDY_INT1 = 0x0500;

constexpr uint16_t BACKEND_SAMPLE_RATE = 800;
constexpr uint32_t BACKEND_PERIOD_US = 1000000UL / BACKEND_SAMPLE_RATE;
constexpr uint8_t HARDWARE_INIT_MAX_TRIES = 5;

}

extern const AP_HAL::HAL &hal;

AP_InertialSensor_BMI323::AP_InertialSensor_BMI323(
    AP_InertialSensor &imu,
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
    enum Rotation rotation) :
    AP_InertialSensor_Backend(imu),
    _dev(std::move(dev)),
    _rotation(rotation)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_BMI323::probe(
    AP_InertialSensor &imu,
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
    enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    auto *sensor = NEW_NOTHROW AP_InertialSensor_BMI323(imu, std::move(dev), rotation);
    if (sensor == nullptr) {
        return nullptr;
    }

    if (!sensor->hardware_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_InertialSensor_BMI323::hardware_init()
{
    hal.scheduler->delay(10);
    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    for (uint8_t attempt = 0; attempt < HARDWARE_INIT_MAX_TRIES; attempt++) {
        uint16_t chip_id = 0;

        // The first read after power-up selects SPI. BMI323 then requires one
        // dummy byte before returning the 16-bit little-endian register.
        (void)read_register16(REG_CHIP_ID, chip_id);
        hal.scheduler->delay(1);
        if (!read_register16(REG_CHIP_ID, chip_id) || uint8_t(chip_id) != CHIP_ID) {
            hal.scheduler->delay(5);
            continue;
        }

        if (!write_register16(REG_CMD, CMD_SOFT_RESET)) {
            continue;
        }
        hal.scheduler->delay(5);

        // Soft reset returns the interface to its power-on state.
        (void)read_register16(REG_CHIP_ID, chip_id);
        hal.scheduler->delay(2);
        if (!read_register16(REG_CHIP_ID, chip_id) || uint8_t(chip_id) != CHIP_ID) {
            continue;
        }

        if (!write_register16(REG_INT_MAP2, 0) ||
            !write_register16(REG_ACC_CONF, ACC_CONF_800HZ_16G) ||
            !write_register16(REG_GYR_CONF, GYR_CONF_800HZ_2000DPS) ||
            !write_register16(REG_INT_CONF, 0) ||
            !write_register16(REG_IO_INT_CTRL, IO_INT_CTRL_INT1) ||
            !write_register16(REG_INT_MAP2, INT_MAP2_ACC_GYR_DRDY_INT1)) {
            continue;
        }
        hal.scheduler->delay(100);

        uint16_t acc_conf = 0;
        uint16_t gyr_conf = 0;
        uint16_t int_ctrl = 0;
        uint16_t int_map = 0;
        if (read_register16(REG_ACC_CONF, acc_conf) &&
            read_register16(REG_GYR_CONF, gyr_conf) &&
            read_register16(REG_IO_INT_CTRL, int_ctrl) &&
            read_register16(REG_INT_MAP2, int_map) &&
            acc_conf == ACC_CONF_800HZ_16G &&
            gyr_conf == GYR_CONF_800HZ_2000DPS &&
            int_ctrl == IO_INT_CTRL_INT1 &&
            int_map == INT_MAP2_ACC_GYR_DRDY_INT1) {
            // Clear a pending data-ready event before periodic sampling starts.
            (void)read_register16(REG_INT_STATUS_INT1, chip_id);
            _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
            return true;
        }
    }

    DEV_PRINTF("BMI323: failed to initialise or verify registers\n");
    return false;
}

bool AP_InertialSensor_BMI323::read_register16(uint8_t reg, uint16_t &value)
{
    const uint8_t tx[4] { uint8_t(reg | 0x80U), 0, 0, 0 };
    uint8_t rx[sizeof(tx)] {};
    if (!_dev->transfer_fullduplex(tx, rx, sizeof(tx))) {
        return false;
    }
    value = uint16_t(rx[2]) | (uint16_t(rx[3]) << 8);
    return true;
}

bool AP_InertialSensor_BMI323::write_register16(uint8_t reg, uint16_t value)
{
    const uint8_t tx[3] {
        uint8_t(reg & 0x7FU),
        uint8_t(value),
        uint8_t(value >> 8)
    };
    if (!_dev->transfer(tx, sizeof(tx), nullptr, 0)) {
        return false;
    }
    hal.scheduler->delay_microseconds(2);
    return true;
}

bool AP_InertialSensor_BMI323::read_motion6(int16_t raw[6])
{
    uint8_t tx[14] { uint8_t(REG_ACC_DATA_X | 0x80U) };
    uint8_t rx[sizeof(tx)] {};
    if (!_dev->transfer_fullduplex(tx, rx, sizeof(tx))) {
        return false;
    }

    // rx[1] is the BMI323 SPI dummy byte.
    for (uint8_t i = 0; i < 6; i++) {
        raw[i] = int16_t(uint16_t(rx[2 + 2*i]) |
                         (uint16_t(rx[3 + 2*i]) << 8));
    }
    return true;
}

void AP_InertialSensor_BMI323::start()
{
    if (!_imu.register_accel(_accel_instance, BACKEND_SAMPLE_RATE,
                             _dev->get_bus_id_devtype(DEVTYPE_INS_BMI323)) ||
        !_imu.register_gyro(_gyro_instance, BACKEND_SAMPLE_RATE,
                            _dev->get_bus_id_devtype(DEVTYPE_INS_BMI323))) {
        return;
    }

    set_accel_orientation(_accel_instance, _rotation);
    set_gyro_orientation(_gyro_instance, _rotation);

    _periodic_handle = _dev->register_periodic_callback(
        BACKEND_PERIOD_US,
        FUNCTOR_BIND_MEMBER(&AP_InertialSensor_BMI323::read_sensor, void));
}

void AP_InertialSensor_BMI323::read_sensor()
{
    int16_t raw[6];
    if (!read_motion6(raw)) {
        _inc_accel_error_count(_accel_instance);
        _inc_gyro_error_count(_gyro_instance);
        return;
    }

    const float accel_scale = (16.0f * GRAVITY_MSS) / 32768.0f;
    const float gyro_scale = radians(2000.0f) / 32768.0f;

    // Preserve the native-to-airframe conversion verified with MadFlight.
    // ArduPilot applies the hwdef ROTATION_* and calibration after this.
    Vector3f accel(-raw[0], raw[1], raw[2]);
    Vector3f gyro(raw[3], -raw[4], -raw[5]);
    accel *= accel_scale;
    gyro *= gyro_scale;

    _rotate_and_correct_accel(_accel_instance, accel);
    _notify_new_accel_raw_sample(_accel_instance, accel);
    _rotate_and_correct_gyro(_gyro_instance, gyro);
    _notify_new_gyro_raw_sample(_gyro_instance, gyro);
}

bool AP_InertialSensor_BMI323::update()
{
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);
    return true;
}

#endif // AP_INERTIALSENSOR_ENABLED
