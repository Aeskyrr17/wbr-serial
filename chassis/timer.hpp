#pragma once

#include <cstdint>

namespace timing
{

// Counts consecutive calls; the caller determines the duration of each call.
class timer
{
public:
    explicit constexpr timer(std::uint32_t limit) noexcept : limit_(limit)
    {
    }

    constexpr bool update(bool condition = true) noexcept
    {
        if (!condition)
        {
            reset();
            return false;
        }
        if (count_ < limit_)
        {
            ++count_;
        }
        return count_ >= limit_;
    }

    constexpr void reset() noexcept
    {
        count_ = 0;
    }

    [[nodiscard]] constexpr std::uint32_t count() const noexcept
    {
        return count_;
    }

private:
    std::uint32_t limit_;
    std::uint32_t count_ = 0;
};

} // namespace timing
