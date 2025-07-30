#include <array>

#include <gtest/gtest.h>

#include "metalimbs_hardware/serialize.hpp"

namespace metalimbs_hardware::impl
{
template <class T>
struct not_std_array_class {
};
TEST(SerializeTest, IsStdArrayTest)
{
    EXPECT_TRUE((impl::is_std_array_v<std::array<char, 2>>));
    EXPECT_FALSE((impl::is_std_array_v<not_std_array_class<char>>));
    EXPECT_FALSE(impl::is_std_array_v<int>);
}

TEST(SerializeTest, GetSizeTest)
{
    EXPECT_EQ(sizeof(float), (get_size_v<float>));
    EXPECT_EQ(sizeof(int) + sizeof(double), (get_size_v<int, double>));
    EXPECT_EQ(2 * sizeof(int) + sizeof(double), (get_size_v<int, double, int>));
}

TEST(SerializeTest, SerializePrimitiveElementTest)
{
    {
        int16_t value = 2560;
        std::deque<char> bytes = serialize(value);
        EXPECT_EQ(2, bytes.size());
        EXPECT_EQ(static_cast<uint8_t>((value & 0xff00) >> 8), bytes[0]);
        EXPECT_EQ(static_cast<uint8_t>(value & 0x00ff), bytes[1]);
    }
    {
        union {
            int32_t value = 123;
            char bytes[4];
        } data;
        std::deque<char> bytes = serialize(data.value);
        EXPECT_EQ(4, bytes.size());
        EXPECT_EQ(data.bytes[0], bytes[3]);
        EXPECT_EQ(data.bytes[1], bytes[2]);
        EXPECT_EQ(data.bytes[2], bytes[1]);
        EXPECT_EQ(data.bytes[3], bytes[0]);
    }
    {
        union {
            uint16_t value = 123;
            char bytes[2];
        } data;
        std::deque<char> bytes = serialize(data.value);
        EXPECT_EQ(2, bytes.size());
        EXPECT_EQ(data.bytes[0], bytes[1]);
        EXPECT_EQ(data.bytes[1], bytes[0]);
    }
}

TEST(SerializeTest, SerializeArrayElementTest)
{
    {
        std::array<char, 4> data = {'a', 'b', 'c', 'd'};
        std::deque<char> bytes = serialize(data);
        EXPECT_EQ(4, bytes.size());
        EXPECT_EQ(data[0], bytes[0]);
        EXPECT_EQ(data[1], bytes[1]);
        EXPECT_EQ(data[2], bytes[2]);
        EXPECT_EQ(data[3], bytes[3]);
    }
}

TEST(SerializeTest, SerializeMultipleElementsTest)
{
    {
        union {
            struct {
                int32_t v1 = 123;
                uint16_t v2 = 321;
            } value;
            char bytes[6];
        } data{};
        std::deque<char> bytes = serialize(data.value.v1, data.value.v2);

        EXPECT_EQ(6, bytes.size());
        EXPECT_EQ(data.bytes[0], bytes[3]);
        EXPECT_EQ(data.bytes[1], bytes[2]);
        EXPECT_EQ(data.bytes[2], bytes[1]);
        EXPECT_EQ(data.bytes[3], bytes[0]);
        EXPECT_EQ(data.bytes[4], bytes[5]);
        EXPECT_EQ(data.bytes[5], bytes[4]);
    }
}

}  // namespace metalimbs_hardware::impl
