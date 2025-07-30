#pragma once

#include <algorithm>
#include <array>
#include <deque>
#include <numeric>

#pragma pack(1)
namespace metalimbs_hardware
{

namespace impl
{

template <class T>
struct is_std_array : public std::false_type {
};
template <class T, std::size_t size>
struct is_std_array<std::array<T, size>> : public std::true_type {
};
template <class T>
inline constexpr bool is_std_array_v = is_std_array<T>::value;

template <class Head, class... Args>
struct get_size {
    static constexpr uint8_t size = sizeof(Head) + get_size<Args...>::size;
};
template <class T>
struct get_size<T> {
    static constexpr uint8_t size = sizeof(T);
};
template <class... Args>
inline constexpr uint8_t get_size_v = get_size<Args...>::size;

template <class T>
std::deque<char> serialize(const T& value)
{
    if constexpr (sizeof(T) == 1) {
        return std::deque<char>{*reinterpret_cast<const char*>(&value)};
    } else if constexpr (is_std_array_v<T>) {
        std::deque<char> bytes;
        for (const auto& element : value) {
            const std::deque<char> element_bytes = serialize(element);
            bytes.insert(bytes.end(), element_bytes.begin(), element_bytes.end());
        }
        return bytes;
    } else {
        std::deque<char> bytes{reinterpret_cast<const char*>(&value), reinterpret_cast<const char*>(&value + 1)};
        std::reverse(bytes.begin(), bytes.end());
        return bytes;
    }
}

template <class Head, class... Args>
std::deque<char> serialize(Head head, Args... args)
{
    std::deque<char> bytes = serialize(head);
    const std::deque<char> rest_result = serialize(args...);
    bytes.insert(bytes.end(), rest_result.begin(), rest_result.end());
    return bytes;
}
}  // namespace impl

template <class... Args>
std::deque<char> serialize(Args... args)
{
    std::deque<char> bytes = impl::serialize(args...);
    // size, commandの2バイトを含む
    bytes.emplace_front(impl::get_size_v<Args...> + 2);
    bytes.emplace_back(std::reduce(bytes.begin(), bytes.end(), uint8_t{0},
        [](char a, char b) { return static_cast<uint8_t>(a) + static_cast<uint8_t>(b); }));
    return bytes;
}

}  // namespace metalimbs_hardware
#pragma pack()
