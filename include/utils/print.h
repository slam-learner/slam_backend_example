#pragma once

#include <iostream>

namespace utils {

template <class T, class = void>
struct is_streamable : std::false_type {};

template <class T>
struct is_streamable<T, std::void_t<decltype(std::declval<std::ostream&>() << std::declval<std::decay_t<T>>())>>
    : std::true_type {};

template <class T>
inline constexpr bool is_streamable_v = is_streamable<T>::value;

template <class... Args>
struct all_streamable : std::conjunction<is_streamable<Args>...> {};

template <class... Args>
inline constexpr bool all_streamable_v = all_streamable<Args...>::value;

template <class... Args>
void print(const Args&... args) {
    if constexpr (all_streamable_v<Args...>) {
        constexpr int argc{sizeof...(Args)};
        int cnt{0};
        ((std::cout << args << (++cnt == argc ? "" : " ")), ...);
        std::cout << '\n';
    } else {
        static_assert(all_streamable_v<Args...>, "All arguments must be streamable");
    }
}

template <class... Args>
void print_wo_space(const Args&... args) {
    if constexpr (all_streamable_v<Args...>) {
        ((std::cout << args), ...);
        std::cout << '\n';
    } else {
        static_assert(all_streamable_v<Args...>, "All arguments must be streamable");
    }
}

template <class... Args>
void print_wo_endl(const Args&... args) {
    if constexpr (all_streamable_v<Args...>) {
        constexpr int argc{sizeof...(Args)};
        int cnt{0};
        ((std::cout << args << (++cnt == argc ? "" : " ")), ...);
    } else {
        static_assert(all_streamable_v<Args...>, "All arguments must be streamable");
    }
}

}  // namespace utils
