#include "cmd.hpp"

#include <algorithm>
#include <cmath>

namespace chassis::cmd
{
motion_req map_ps2(const remotes::ps2::sample& rc, bool online, std::uint32_t now) noexcept
{
    motion_req req{};
    req.received_tick = now;
    req.online = online;
    const bool up = rc.buttons.up;
    const bool down = rc.buttons.down;
    req.len_rate = (static_cast<int>(up) - static_cast<int>(down)) * mapping::len_rate;
    static bool move = false;
    static bool wait_release = false; // 是否正在等待两个键都松开
    const bool l1 = rc.buttons.l1;
    const bool l2 = rc.buttons.l2;
    if (!req.online)
    {
        move = false;
        wait_release = true;
    }
    else if (wait_release)
    {
        // Consume one action until both keys are released.
        if (!l1 && !l2)
            wait_release = false;
    }
    else if (move && (l1 || l2))
    {
        move = false;
        wait_release = true;
    }
    else if (!move && l1 && l2)
    {
        move = true;
        wait_release = true;
    }
    static bool spin = false;
    static bool circle_held = false;
    const bool circle = rc.buttons.circle;
    if (!req.online || !move)
        spin = false;
    else if (circle && !circle_held)
        spin = !spin;
    circle_held = circle;
    req.move = move;
    req.spin = spin;
    req.v = std::fabs(rc.ly) > mapping::deadzone ? rc.ly * mapping::max_v : 0.0f;
    req.dyaw = std::fabs(rc.rx) > mapping::deadzone ? -rc.rx * mapping::max_dyaw : 0.0f;
    return req;
}
}
