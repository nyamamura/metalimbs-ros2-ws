#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <iostream>
#include <numeric>

#include "serialize.hpp"

#pragma pack(1)

namespace metalimbs_hardware::command
{

struct Command {
    virtual std::deque<char> serialize() = 0;
};

struct ReadCurrent : public Command {
    struct Result {
        uint8_t size;
        int16_t current;  // mA
        uint8_t sum;
    };

    std::deque<char> serialize() { return ::metalimbs_hardware::serialize(command, motor_id); }

    static constexpr uint8_t command = 0x02;
    uint8_t motor_id;
};

struct ReadTemperature : public Command {
    struct Result {
        uint8_t size;
        int16_t temperature;  // 0.01 degrees of Celsius
        uint8_t sum;
    };

    std::deque<char> serialize() { return ::metalimbs_hardware::serialize(command, motor_id); }

    static constexpr uint8_t command = 0x03;
    uint8_t motor_id;
};

struct ReadPosition : public Command {
    struct Result {
        uint8_t size;
        int16_t angle;  // 0.01 deg
        uint8_t sum;
    };

    std::deque<char> serialize() { return ::metalimbs_hardware::serialize(command, motor_id); }

    static constexpr uint8_t command = 0x04;
    uint8_t motor_id;
};

struct ReadEncoder : public Command {
    struct Result {
        uint8_t size;
        int32_t angle;  // pulses
        uint8_t sum;
    };


    std::deque<char> serialize() { return ::metalimbs_hardware::serialize(command, motor_id); }

    static constexpr uint8_t command = 0x05;
    uint8_t motor_id;
};

struct SetPosition : public Command {
    struct Result {
    };

    std::deque<char> serialize() { return ::metalimbs_hardware::serialize(command, motor_id, angle); }

    static constexpr uint8_t command = 0x06;
    uint8_t motor_id;
    int16_t angle;  // 0.01 deg
};

struct SetPositionWithTime {
    struct Result {
    };

    std::deque<char> serialize() { return ::metalimbs_hardware::serialize(command, motor_id, angle, time_to_reach); }

    static constexpr uint8_t command = 0x0c;
    uint8_t motor_id;
    int16_t angle;           // 0.01 deg
    uint16_t time_to_reach;  // ms
};

struct SetAllPosition : public Command {
    struct Result {
    };

    std::deque<char> serialize() { return ::metalimbs_hardware::serialize(command, positions); }

    static constexpr uint8_t command = 0x07;
    std::array<int16_t, 7> positions;
};

struct SetAllPositionWithTime : public Command {
    struct Result {
    };

    std::deque<char> serialize() { return ::metalimbs_hardware::serialize(command, positions, time_to_reach); }

    static constexpr uint8_t command = 0x0d;
    std::array<int16_t, 7> positions;
    int16_t time_to_reach;  // ms
};

struct ReadAllTemperatures : public Command {
    struct Result {
        std::array<int16_t, 7> temperatures;
    };

    std::deque<char> serialize() { return ::metalimbs_hardware::serialize(command); }

    static constexpr uint8_t command = 0x0a;
};

struct ReadAllPosition : public Command {
    struct Result {
        std::array<int16_t, 7> positions;  // 0.01 deg
    };

    std::deque<char> serialize() { return ::metalimbs_hardware::serialize(command); }

    static constexpr uint8_t command = 0x08;
};

struct SetTorqueState : public Command {
    struct Result {
    };

    std::deque<char> serialize() { return ::metalimbs_hardware::serialize(command, state); }

    static constexpr uint8_t command = 0x01;
    bool state;
};

struct Reset : public Command {
    struct Result {
    };
    std::deque<char> serialize() { return ::metalimbs_hardware::serialize(command); }
    static constexpr uint8_t command = 0x1c;
};

// TODO(watanabe): implement Hand Position Controller

}  // namespace metalimbs_hardware::command

#pragma pack()
