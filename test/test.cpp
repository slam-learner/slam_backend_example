#include "utils/print.h"
#include "utils/timer.h"

long long fibonacci(int n) {
    if (n <= 1) {
        return n;
    }

    long long fib_0 = 0;
    long long fib_1 = 1;
    for (int i = 2; i <= n; ++i) {
        long long fib_n = fib_0 + fib_1;
        fib_0 = fib_1;
        fib_1 = fib_n;
    }

    return fib_1;
}

auto main() -> int {
    constexpr int n = 40;

    utils::print("testing fibonacci with n =", n);
    utils::Timer::Evaluate([]() { fibonacci(n); }, "Fibonacci");
    utils::Timer::PrintAll();

    return 0;
}
