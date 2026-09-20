#include "ps2_gpio_async.hpp"

#include <array>
#include <cassert>
#include <cstdio>
#include <vector>

namespace
{
core::callback<void()> timer_callback;
bool armed = false;
bool run_timer = true;
bool thread_context = true;
bool cmd = true;
bool clk = true;
bool cs = false;
bool fail_gpio = false;
bool fail_timer = false;
unsigned next_delay = 0;
unsigned time_us = 0;
unsigned cmd_time = 0;
unsigned low_time = 0;
unsigned high_time = 0;
unsigned interrupts = 0;
unsigned blocking_waits = 0;
std::vector<bool> sent;
std::array<unsigned char, 9> response{0xff, 0x73, 0x5a, 0xaf, 0x39, 0x80, 0x00, 0xff, 0x01};
std::size_t read_bit = 0;

void reset_bus()
{
    time_us = cmd_time = low_time = high_time = interrupts = blocking_waits = 0;
    sent.clear();
    read_bit = 0;
    assert(!armed && !cs && clk);
}
}

namespace bsp::gpio
{
types::status write(output id, bool high)
{
    if (fail_gpio) return types::status::error;
    if (id == output::cmd)
    {
        cmd = high;
        cmd_time = time_us;
    }
    if (id == output::clk)
    {
        if (!high)
        {
            assert(cs && clk);
            assert(time_us - cmd_time == 5);
            sent.push_back(cmd);
            low_time = time_us;
        }
        else if (!clk)
        {
            high_time = time_us;
        }
        clk = high;
    }
    return types::status::ok;
}

types::status set_active(output id, bool active)
{
    assert(id == output::cs);
    cs = active;
    return types::status::ok;
}

types::status read(input, bool& high)
{
    assert(cs && !clk && time_us - low_time == 5);
    high = (response[read_bit / 8] & (1U << (read_bit % 8))) != 0;
    ++read_bit;
    return types::status::ok;
}
}

namespace bsp::timer
{
types::status init(channel, core::callback<void()> callback)
{
    timer_callback = callback;
    return types::status::ok;
}
types::status arm(channel, std::uint32_t delay)
{
    if (fail_timer) return types::status::error;
    assert(!armed);
    armed = true;
    next_delay = delay;
    return types::status::ok;
}
void cancel(channel) { armed = false; }
}

UINT tx_semaphore_create(TX_SEMAPHORE* sem, CHAR*, ULONG count)
{
    sem->count = static_cast<unsigned>(count);
    return TX_SUCCESS;
}
UINT tx_semaphore_delete(TX_SEMAPHORE*) { return TX_SUCCESS; }
UINT tx_semaphore_put(TX_SEMAPHORE* sem) { ++sem->count; return TX_SUCCESS; }
UINT tx_semaphore_get(TX_SEMAPHORE* sem, ULONG wait)
{
    if (wait != TX_NO_WAIT)
    {
        ++blocking_waits;
        // Simulated ISR execution while the caller is suspended in ThreadX.
        while (run_timer && armed && sem->count == 0)
        {
            assert(++interrupts <= 218);
            time_us += next_delay;
            armed = false;
            timer_callback();
        }
    }
    if (sem->count == 0) return 1U;
    --sem->count;
    return TX_SUCCESS;
}
TX_THREAD* tx_thread_identify()
{
    static TX_THREAD thread;
    return thread_context ? &thread : nullptr;
}

int main()
{
    using types::status;
    remoter::ps2_gpio_async transport{bsp::gpio::output::cmd, bsp::gpio::input::data,
        bsp::gpio::output::clk, bsp::gpio::output::cs, bsp::timer::channel::test};
    const std::array<unsigned char, 9> query{1, 0x42, 0, 0x55, 0xaa, 0, 1, 0x80, 0xff};
    std::array<unsigned char, 9> rx{};
    assert(transport.transfer(query.data(), rx.data(), 9) == status::not_initialized);
    assert(transport.init() == status::ok);
    assert(transport.init() == status::ok);
    assert(transport.transfer(nullptr, rx.data(), 9) == status::invalid_arg);
    thread_context = false;
    assert(transport.transfer(query.data(), rx.data(), 9) == status::invalid_context);
    thread_context = true;

    for (int repeat = 0; repeat < 2; ++repeat)
    {
        reset_bus();
        assert(transport.transfer(query.data(), rx.data(), 9) == status::ok);
        assert(rx == response && sent.size() == 72 && read_bit == 72);
        for (unsigned bit = 0; bit < 72; ++bit)
            assert(sent[bit] == ((query[bit / 8] & (1U << (bit % 8))) != 0));
        assert(time_us == 1120 && time_us - high_time == 25);
        assert(interrupts == 218 && blocking_waits == 1);
        assert(!armed && !cs && clk);
    }

    reset_bus();
    run_timer = false;
    assert(transport.transfer(query.data(), rx.data(), 9) == status::error);
    assert(!armed && !cs && clk);
    const auto saved_rx = rx;
    timer_callback(); // A stale ISR cannot touch a returned stack buffer.
    assert(rx == saved_rx);
    run_timer = true;

    reset_bus();
    fail_timer = true;
    assert(transport.transfer(query.data(), rx.data(), 9) == status::error);
    assert(!armed && !cs && clk);
    fail_timer = false;

    reset_bus();
    fail_gpio = true;
    assert(transport.transfer(query.data(), rx.data(), 9) == status::error);
    assert(!armed && !cs);
    fail_gpio = false;

    reset_bus();
    assert(transport.transfer(query.data(), rx.data(), 9) == status::ok);
    std::puts("PS2 timer transport: duplex/LSB/timing/suspend/timeout/error/reuse passed");
}
