#include "cfg.hpp"
#include "chassis.hpp"

#include <algorithm>
#include <cmath>

namespace chassis
{
/**
 * @brief 被动离地状态
 */
void controller::run_airborne(const inputs& in, step_out& out) noexcept
{
    if (!l_contact_.air_seen && in.left.offground) //进入空中状态
    {
        l_contact_.air_seen = true;
        l_contact_.dlen_peak = 0.0f;
        l_contact_.touch_timer.reset();
        l_contact_.touchdown = false;
    }
    if (!r_contact_.air_seen && in.right.offground)
    {
        r_contact_.air_seen = true;
        r_contact_.dlen_peak = 0.0f;
        r_contact_.touch_timer.reset();
        r_contact_.touchdown = false;
    }
    if (!l_contact_.touchdown) //空中状态
    {
        l_contact_.dlen_peak = std::max(l_contact_.dlen_peak, in.left.dlen);
        const bool contact =  (in.left.dlen < cfg::touchdown_compress_rate || //腿长变短速率于阈值，说明落地腿在被压缩
            (in.left.len < cfg::Lmax - cfg::touchdown_len_margin &&           //仍在伸腿过程
             l_contact_.dlen_peak > cfg::touchdown_peak_rate &&               //伸长峰值速度大于阈值 //TODO 有点不明白这一项
             l_contact_.dlen_peak - in.left.dlen > cfg::touchdown_rate_drop));//伸腿加速度减小
        if (l_contact_.touch_timer.update(contact))
        {
            l_contact_.touchdown = true;
        }
    }
    if (!r_contact_.touchdown)
    {
        r_contact_.dlen_peak = std::max(r_contact_.dlen_peak, in.right.dlen);

        const bool contact = (in.right.dlen < cfg::touchdown_compress_rate ||
            (in.right.len < cfg::Lmax - cfg::touchdown_len_margin &&
             r_contact_.dlen_peak > cfg::touchdown_peak_rate &&
             r_contact_.dlen_peak - in.right.dlen > cfg::touchdown_rate_drop));
        if (r_contact_.touch_timer.update(contact))
        {
            r_contact_.touchdown = true;
        }
    }

    out.ref.x = in.obs.x;
    out.ref.v = in.obs.v;
    out.ref.yaw = in.obs.yaw;
    out.ref.dyaw = in.obs.dyaw;
    out.ref.l_alpha = in.imu.pitch; //为了保持机身的垂直，不跟踪绝对的alpha = 0
    out.ref.l_dalpha = 0.0f;
    out.ref.r_alpha = in.imu.pitch;
    out.ref.r_dalpha = 0.0f;
    out.ref.pitch = in.obs.pitch;
    out.ref.dpitch = in.obs.dpitch;
    out.mode = lqr_mode::low;
    const auto torque = lqr_.update(out.mode, in.left.len, in.right.len, in.obs, out.ref);
    out.force.l_tp = torque[2]; //TODO:为了让空中给的摆角不要那么剧烈，降低了力矩，可以继续调
    out.force.r_tp = torque[3]; //TODO:为了让空中给的摆角不要那么剧烈，降低了力矩，可以继续调
    out.motor.l_tw = 0.0f;
    out.motor.r_tw = 0.0f;

    if (l_contact_.touchdown || r_contact_.touchdown) //任意一条腿落地就切lqr //TODO：是否需要改成都落地才切
    {
        update_leg_lqr(in, out);
    }
    if (!l_contact_.touchdown)
    {
        // out.force.l_f = cfg::air_leg_force; //? direct force output for airborne leg
        len_ramp_.set_value(in.left.len);     //? set leg length to max for airborne leg
        len_ramp_.set_step(0.0009f); //! 考虑是否要大一点步长
        out.force.l_f = len_pid_control(l_len_, in.left, cfg::Lmax) - in.left.spring.fs;
    }
    if (!r_contact_.touchdown)
    {
        // out.force.r_f = cfg::air_leg_force;
        len_ramp_.set_value(in.right.len);
        len_ramp_.set_step(0.0009f);
        update_leg_pid(in, out);
    }

    if (in.left.alpha >= 0.8f || in.right.alpha >= 0.8f) //空中摆角太大失衡relax
    {
        reset_phi_control();
        out.fsm_next = chassis_fsm::relax;
    }
    else if (l_contact_.touchdown && r_contact_.touchdown)
    {
        settle_count_ = 0;
        target_len_ = cfg::Lmin;
        len_ramp_.set_value(in.left.len);
        len_ramp_.set_step(0.0009f);
        out.fsm_next = chassis_fsm::normal;
    }

    auto& debug = out.debug;
    if (!l_contact_.touchdown && r_contact_.touchdown)
    {
        debug.l_len = {l_len_.ref, l_len_.fdb, l_len_.result, true};
    }
    std::copy(torque.begin(), torque.end(), debug.lqr_output);
    const auto& raw = lqr_.raw_output();
    std::copy(raw.begin(), raw.end(), debug.lqr_raw);
    debug.lqr_active = true;
}

} // namespace chassis
