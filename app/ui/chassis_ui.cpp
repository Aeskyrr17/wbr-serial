#include "chassis_ui.hpp"

#include "chassis_model.hpp"

#include <cmath>

namespace robot::display
{
namespace
{
constexpr float rad_to_deg = 180.0f / chassis::model::pi;
int angle(float degrees) noexcept
{
    int result = static_cast<int>(std::fmod(degrees, 360.0f));
    return result < 0 ? result + 360 : result;
}
}

bool chassis_ui::init() noexcept
{
    return ui::canvas::instance().init() == types::status::ok;
}

void chassis_ui::update(const robot_state& state, const io::gimbal_ui& gimbal,
                        const command::referee_summary& referee) noexcept
{
    auto& canvas = ui::canvas::instance();
    if (gimbal.reset)
    {
        canvas.remove_all();
        created_ = false;
    }
    if (!created_ && referee.robot_id != 0U)
    {
        canvas.set_sender_receiver_id(referee.robot_id,
                                      static_cast<std::uint16_t>(referee.robot_id + 256U));
        cap_ = canvas.create_line(30, ui::object_color::yellow, 2, 695, 300, 1155, 300);
        front_ = canvas.create_arc(10, ui::object_color::yellow, 2, 960, 540, 90, 90, 330, 30);
        velocity_ = canvas.create_float(2, ui::object_color::yellow, 2, 120, 750, 20, 0.0f);
        length_ = canvas.create_float(2, ui::object_color::yellow, 2, 120, 800, 20, 0.0f);
        distance_ = canvas.create_float(2, ui::object_color::yellow, 2, 120, 700, 20, 0.0f);

        aim_rect_ = canvas.create_rect(3, ui::object_color::white, 3, 695, 700, 1155, 330);
        robot_rect_ = canvas.create_rect(3, ui::object_color::white, 3, 695, 710, 765, 760);
        rune_rect_ = canvas.create_rect(3, ui::object_color::white, 3, 775, 710, 865, 760);
        aim_ = canvas.create_circle(3, ui::object_color::green, 3, 695, 330, 15);
        aimer_ = canvas.create_circle(3, ui::object_color::orange, 3, 960, 540, 80);
        target_ = canvas.create_int(2, ui::object_color::yellow, 3, 710, 685, 15, 0);
        (void)canvas.create_string(2, ui::object_color::white, 3, 705, 745, 20, "AIM");
        (void)canvas.create_string(2, ui::object_color::white, 3, 785, 745, 20, "RUNE");

        (void)canvas.create_string(3, ui::object_color::yellow, 4, 1500, 660, 20, "NORMAL");
        (void)canvas.create_string(3, ui::object_color::yellow, 4, 1500, 610, 20, "JUMP");
        (void)canvas.create_string(3, ui::object_color::yellow, 4, 1500, 560, 20, "STAIR");
        (void)canvas.create_string(3, ui::object_color::yellow, 4, 1500, 510, 20, "FLY");
        normal_ = canvas.create_circle(3, ui::object_color::yellow, 5, 1700, 650, 15);
        jump_ = canvas.create_circle(3, ui::object_color::yellow, 5, 1700, 600, 15);
        stair_ = canvas.create_circle(3, ui::object_color::yellow, 5, 1700, 550, 15);
        fly_ = canvas.create_circle(3, ui::object_color::yellow, 5, 1700, 500, 15);

        alert_ = canvas.create_string(3, ui::object_color::team, 5, 800, 800, 25,
                                      "TURN ON SPIN");
        armor1_ = canvas.create_line(3, ui::object_color::team, 5, 920, 830, 1000, 830);
        armor2_ = canvas.create_line(3, ui::object_color::team, 5, 1300, 500, 1300, 580);
        armor3_ = canvas.create_line(3, ui::object_color::team, 5, 920, 250, 1000, 250);
        armor4_ = canvas.create_line(3, ui::object_color::team, 5, 620, 500, 620, 580);

        lbase_ = canvas.create_line(3, ui::object_color::yellow, 6, 1450, 800, 1600, 800);
        rbase_ = canvas.create_line(3, ui::object_color::yellow, 6, 1650, 800, 1800, 800);
        lleg_ = canvas.create_line(3, ui::object_color::yellow, 6, 1525, 800, 1525, 750);
        rleg_ = canvas.create_line(3, ui::object_color::yellow, 6, 1725, 800, 1725, 750);

        canvas.set_visible(rune_rect_, false); canvas.set_visible(robot_rect_, false);
        canvas.set_visible(front_, false); canvas.set_visible(aim_, false);
        canvas.set_visible(normal_, false); canvas.set_visible(jump_, false);
        canvas.set_visible(stair_, false); canvas.set_visible(fly_, false);
        canvas.set_visible(alert_, false); canvas.set_visible(armor1_, false);
        canvas.set_visible(armor2_, false); canvas.set_visible(armor3_, false);
        canvas.set_visible(armor4_, false);
        created_ = rleg_ >= 0;
    }
    if (created_)
    {
        io::supercap_feedback cap{};
        io::tof_sample tof{};
        (void)io::read_supercap(cap); (void)io::read_tof(tof);
        const float relative = std::remainder(state.yaw_feedback - chassis::model::yaw_offset_front,
                                              2.0f * chassis::model::pi);
        canvas.set_visible(front_, true);
        canvas.move_p2_to(cap_, 695 + static_cast<int>(cap.remained_energy * 0.245f), 300);
        canvas.set_start_angle(front_, angle(330.0f - relative * rad_to_deg));
        canvas.set_end_angle(front_, angle(30.0f - relative * rad_to_deg));
        canvas.set_float(velocity_, std::fabs(state.v));
        canvas.set_float(length_, state.len * 100.0f);
        canvas.set_float(distance_, tof.valid ? tof.distance_cm : 0.0f);

        canvas.set_visible(rune_rect_, gimbal.aim_rune);
        canvas.set_visible(robot_rect_, !gimbal.aim_rune);
        canvas.set_int(target_, gimbal.aim_target);
        const bool acquired = gimbal.aim_target != 0U;
        canvas.set_color(aim_rect_, acquired ? ui::object_color::magenta : ui::object_color::white);
        canvas.set_visible(aim_, acquired);
        if (acquired) canvas.move_to(aim_, 695 + static_cast<int>(gimbal.aim_x * 4.6f),
                                     330 + static_cast<int>(gimbal.aim_y * 3.7f));
        canvas.set_color(aimer_, gimbal.bullet_frequency ? ui::object_color::orange
                                                        : ui::object_color::yellow);
        canvas.set_visible(normal_, state.normal);
        canvas.set_visible(jump_, state.jump != jump_stage::dont);
        canvas.set_visible(stair_, state.stairing);
        canvas.set_visible(fly_, state.flying);

        constexpr float half_base = 75.0f, leg_len = 100.0f;
        constexpr int body_x = 1525, body_y = 800;
        const float cp = std::cos(-state.pitch), sp = std::sin(-state.pitch);
        canvas.move_to(lbase_, body_x + static_cast<int>(half_base * cp),
                       body_y - static_cast<int>(half_base * sp));
        canvas.move_p2_to(lbase_, body_x - static_cast<int>(half_base * cp),
                          body_y + static_cast<int>(half_base * sp));
        canvas.move_to(rbase_, body_x + 200 + static_cast<int>(half_base * cp),
                       body_y - static_cast<int>(half_base * sp));
        canvas.move_p2_to(rbase_, body_x + 200 - static_cast<int>(half_base * cp),
                          body_y + static_cast<int>(half_base * sp));
        canvas.move_p2_to(lleg_, body_x + static_cast<int>(leg_len * std::sin(state.pitch + state.lalpha)),
                          body_y - static_cast<int>(leg_len * std::cos(state.pitch + state.lalpha)));
        canvas.move_p2_to(rleg_, body_x + 200 + static_cast<int>(leg_len * std::sin(state.pitch + state.ralpha)),
                          body_y - static_cast<int>(leg_len * std::cos(state.pitch + state.ralpha)));

        const bool armor_hit = referee.hurt_reason == 0U && referee.hurt_armor_id != 0U;
        canvas.set_visible(armor1_, armor_hit && referee.hurt_armor_id == 1U);
        canvas.set_visible(armor2_, armor_hit && referee.hurt_armor_id == 2U);
        canvas.set_visible(armor3_, armor_hit && referee.hurt_armor_id == 3U);
        canvas.set_visible(armor4_, armor_hit && referee.hurt_armor_id == 4U);
        canvas.set_visible(alert_, armor_hit && !state.spinning);
    }
    canvas.update();
}

} // namespace robot::display
