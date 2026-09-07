/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/*
 * Bosch BMI323 SPI backend.
 *
 * BMI323 has 16-bit registers and inserts a dummy byte after an SPI read
 * address.  That protocol is different enough from BMI270 to require its own
 * backend.
 */
class AP_InertialSensor_BMI323 : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation = ROTATION_NONE);

    void start() override;
    bool update() override;

private:
    AP_InertialSensor_BMI323(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                             enum Rotation rotation);

    bool hardware_init();
    bool read_register16(uint8_t reg, uint16_t &value);
    bool write_register16(uint8_t reg, uint16_t value);
    bool read_motion6(int16_t raw[6]);
    void read_sensor();

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    enum Rotation _rotation;
    uint8_t _accel_instance;
    uint8_t _gyro_instance;
    AP_HAL::Device::PeriodicHandle _periodic_handle;
};
