#include "command.hpp"

#include <algorithm>
#include <cmath>

namespace robot::command
{
motion_request map_can(const io::command& raw, std::uint32_t now) noexcept
{
    motion_request out{};
    out.source = input_source::can;
    out.received_tick = raw.received_tick;
    out.online = raw.sequence != 0U && now - raw.received_tick <= mapping::offline_ticks;
    out.move = out.online && raw.move && !raw.gimbal_only;
    out.v = std::clamp(raw.vx * mapping::can_v_scale, -mapping::max_v, mapping::max_v);
    out.dyaw = std::clamp(raw.vy * mapping::can_dyaw_scale,
                         -mapping::max_dyaw, mapping::max_dyaw);
    return out;
}

motion_request map_ps2(const remoter::state& rc, std::uint32_t now) noexcept
{
    motion_request out{};
    out.source = input_source::ps2;
    out.received_tick = now;
    out.online = !rc.offline && rc.active_source == remoter::source::ps2;
    const bool up = remoter::is_held(rc.ps2_buttons, remoter::ps2_button::up);
    const bool down = remoter::is_held(rc.ps2_buttons, remoter::ps2_button::down);
    out.len_rate = (static_cast<int>(up) - static_cast<int>(down)) * mapping::len_rate;
    static bool move = false;
    static bool wait_release = false; // 是否正在等待两个键都松开
    const bool l1 = remoter::is_held(rc.ps2_buttons, remoter::ps2_button::l1);
    const bool l2 = remoter::is_held(rc.ps2_buttons, remoter::ps2_button::l2);
    if (!out.online)
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
    const bool circle = remoter::is_held(rc.ps2_buttons, remoter::ps2_button::circle);
    if (!out.online || !move)
        spin = false;
    else if (circle && !circle_held)
        spin = !spin;
    circle_held = circle;
    out.move = move;
    out.spin = spin;
    out.v = std::fabs(rc.left_y) > mapping::deadzone ? rc.left_y * mapping::max_v : 0.0f;
    out.dyaw = std::fabs(rc.right_x) > mapping::deadzone ? -rc.right_x * mapping::max_dyaw : 0.0f;
    return out;
}
}
