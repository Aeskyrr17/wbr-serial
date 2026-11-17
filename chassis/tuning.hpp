#pragma once

namespace chassis
{
struct pid_param
{
    volatile float kp;
    volatile float ki;
    volatile float kd;
};

// Runtime parameters adjusted through the debugger; not observation snapshots.
struct robot_tuning_t
{
    // Enables automatic joint startup and control output.
    volatile bool motor_output_enabled = true;
    volatile float alpha_trim = 0.03f;
    volatile float spin_alpha_trim = -0.03f; // Legacy SPIN equilibrium offset.
    pid_param len_startup{5000.0f, 0.0f, -100.0f};
    pid_param len_run{2000.0f, 0.0f, -400.0f};
    pid_param roll{0.5f, 0.0f, 0.0f};
    pid_param phi_recover{20.0f, 0.0f, 10.0f};
    pid_param phi_flatten_fast{10.0f, 0.0f, 5.0f};
    pid_param phi_flatten_slow{5.0f, 0.0f, 10.0f};
    pid_param phi_neutral{30.0f, 0.0f, 10.0f};
};

inline robot_tuning_t tuning{};
} // namespace chassis
