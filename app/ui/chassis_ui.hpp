#pragma once

#include "command.hpp"
#include "robot_io.hpp"
#include "robot_types.hpp"
#include "ui.hpp"

namespace robot::display
{

class chassis_ui
{
public:
    bool init() noexcept;
    void update(const robot_state& state, const io::gimbal_ui& gimbal,
                const command::referee_summary& referee) noexcept;

private:
    int cap_ = -1, front_ = -1;
    int velocity_ = -1, length_ = -1, distance_ = -1;
    int aim_rect_ = -1, robot_rect_ = -1, rune_rect_ = -1;
    int aim_ = -1, aimer_ = -1, target_ = -1;
    int normal_ = -1, jump_ = -1, stair_ = -1, fly_ = -1;
    int alert_ = -1, armor1_ = -1, armor2_ = -1, armor3_ = -1, armor4_ = -1;
    int lbase_ = -1, rbase_ = -1, lleg_ = -1, rleg_ = -1;
    bool created_ = false;
};

} // namespace robot::display
