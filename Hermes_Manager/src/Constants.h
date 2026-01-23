#pragma once
#include <Arduino.h>

namespace Const {
    
    namespace Encoder {
        constexpr uint8_t WIRE_0_SDA = 21;
        constexpr uint8_t WIRE_0_SCL = 22;

        constexpr uint8_t WIRE_1_SDA = 16;
        constexpr uint8_t WIRE_1_SCL = 17;

        constexpr uint8_t AS5600_ADDR = 0x36;
        constexpr uint32_t I2C_CLOCK_HZ = 400000;
    }

    namespace Motor {

        constexpr float SUPPLY_VOLTAGE = 12.0f;
        constexpr float VOLTAGE_LIMIT = 2.0f;
        constexpr float VELOCITY_LIMIT = 20.0f;

        constexpr int MOTOR_0_POLE_PAIRS = 7;
        constexpr int MOTOR_1_POLE_PAIRS = 7;

        constexpr uint8_t MOTOR_0_U = 25;
        constexpr uint8_t MOTOR_0_V = 26;
        constexpr uint8_t MOTOR_0_W = 27;
        constexpr uint8_t MOTOR_0_EN = 13;

        constexpr uint8_t MOTOR_1_U = 32;
        constexpr uint8_t MOTOR_1_V = 33;
        constexpr uint8_t MOTOR_1_W = 34;
        constexpr uint8_t MOTOR_1_EN = 23;
    }

    namespace Serial {
        constexpr uint32_t BAUD = 115200;
    }

    namespace Safety {
        constexpr bool DEFAULT_ENABLED_ON_BOOT = false;
        constexpr bool ENABLE_ACTIVE_ON_HIGH = true;
        constexpr uint32_t INDEPENDENT_CMD_TIMEOUT_MS = 0;
    }

}