#include <gtest/gtest.h>

#include "metalimbs_hardware/command.hpp"

namespace metalimbs_hardware::command
{

TEST(CommandTest, ReadCurrentTest)
{
    ReadCurrent command;
    command.motor_id = 0x01;
    const auto bytes = command.serialize();
    EXPECT_EQ(4, bytes.size());

    EXPECT_EQ((char)0x04, bytes[0]);
    EXPECT_EQ((char)0x02, bytes[1]);
    EXPECT_EQ((char)0x01, bytes[2]);
    EXPECT_EQ((char)0x07, bytes[3]);
}
TEST(CommandTest, ReadTemperatureTest)
{
    ReadTemperature command;
    command.motor_id = 0x01;
    const auto bytes = command.serialize();
    EXPECT_EQ(4, bytes.size());

    EXPECT_EQ((char)0x04, bytes[0]);
    EXPECT_EQ((char)0x03, bytes[1]);
    EXPECT_EQ((char)0x01, bytes[2]);
    EXPECT_EQ((char)0x08, bytes[3]);
}

TEST(CommandTest, ReadPositionTest)
{
    ReadPosition command;
    command.motor_id = 0x01;
    const auto bytes = command.serialize();
    EXPECT_EQ(4, bytes.size());

    EXPECT_EQ((char)0x04, bytes[0]);
    EXPECT_EQ((char)0x04, bytes[1]);
    EXPECT_EQ((char)0x01, bytes[2]);
    EXPECT_EQ((char)0x09, bytes[3]);
}

TEST(CommandTest, ReadEncoderTest)
{
    ReadEncoder command;
    command.motor_id = 0x01;
    const auto bytes = command.serialize();
    EXPECT_EQ(4, bytes.size());

    EXPECT_EQ((char)0x04, bytes[0]);
    EXPECT_EQ((char)0x05, bytes[1]);
    EXPECT_EQ((char)0x01, bytes[2]);
    EXPECT_EQ((char)0x0a, bytes[3]);
}

TEST(CommandTest, SetPositionTest)
{
    SetPosition command;
    command.motor_id = 0x01;
    command.angle = 2000;
    const auto bytes = command.serialize();
    EXPECT_EQ(6, bytes.size());

    constexpr uint8_t expected[8] = {0x06, 0x06, 0x01, 0x07, 0xd0, 0xe4};
    for (int i = 0; i < 6; ++i) {
        EXPECT_EQ((char)expected[i], bytes[i]) << "i: " << i;
    }
}

TEST(CommandTest, SetPositionWithTimeTest)
{
    SetPositionWithTime command;
    command.motor_id = 0x02;
    command.angle = 0;
    command.time_to_reach = 2000;
    const auto bytes = command.serialize();
    EXPECT_EQ(8, bytes.size());

    constexpr uint8_t expected[8] = {
        0x08, 0x0c, 0x02, 0x00, 0x00, 0x07, 0xd0, 0xed};
    for (int i = 0; i < 8; ++i) {
        EXPECT_EQ((char)expected[i], bytes[i]) << "i: " << i;
    }
}

TEST(CommandTest, SetAllPositionTest)
{
    SetAllPosition command;
    command.positions[0] = 2000;
    command.positions[1] = 2000;
    command.positions[2] = 2000;
    command.positions[3] = 2000;
    command.positions[4] = 2000;
    command.positions[5] = 2000;
    command.positions[6] = 2000;
    const auto bytes = command.serialize();
    EXPECT_EQ(17, bytes.size());

    constexpr uint8_t expected[17] = {
        0x11, 0x07, 0x07, 0xd0,
        0x07, 0xd0, 0x07, 0xd0,
        0x07, 0xd0, 0x07, 0xd0,
        0x07, 0xd0, 0x07, 0xd0, 0xf9};

    for (int i = 0; i < 17; ++i) {
        EXPECT_EQ((char)expected[i], bytes[i]) << "i: " << i;
    }
}

TEST(CommandTest, SetAllPositionWithTimeTest)
{
    SetAllPositionWithTime command;
    command.positions[0] = 0;
    command.positions[1] = 0;
    command.positions[2] = 0;
    command.positions[3] = 0;
    command.positions[4] = 0;
    command.positions[5] = 2000;
    command.positions[6] = 0;
    command.time_to_reach = 2560;
    const auto bytes = command.serialize();
    EXPECT_EQ(19, bytes.size());

    constexpr uint8_t expected[19] = {
        0x13, 0x0d, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x07, 0xd0, 0x00, 0x00,
        0x0a, 0x00, 0x01};

    for (int i = 0; i < 19; ++i) {
        EXPECT_EQ((char)expected[i], bytes[i]) << "i: " << i;
    }
}

TEST(CommandTest, SetTorqueStateTest)
{
    SetTorqueState command;
    command.state = true;
    const auto bytes = command.serialize();
    EXPECT_EQ(4, bytes.size());

    EXPECT_EQ(0x04, bytes[0]);
    EXPECT_EQ(0x01, bytes[1]);
    EXPECT_EQ(0x01, bytes[2]);
    EXPECT_EQ(0x06, bytes[3]);
}

}  // namespace metalimbs_hardware::command