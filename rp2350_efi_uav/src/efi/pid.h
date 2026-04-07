/**
 * @file    pid.h / pid.cpp
 * @brief   Generic PID controller – used for IAC (Idle Air Control)
 *          and optionally boost control.
 *
 * Anti-windup: integrator clamping (output saturation method).
 * Derivative:  filtered d/dt on measurement (not error) to avoid
 *              derivative kick on setpoint change.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float Kp;           // proportional gain
    float Ki;           // integral gain (units: per second)
    float Kd;           // derivative gain (units: seconds)
    float out_min;      // output clamp minimum
    float out_max;      // output clamp maximum
    float dt;           // sample time [s] – fixed for this instance

    // Internal state
    float integrator;
    float prev_measurement; // for derivative on measurement
    float d_filter;         // IIR filter state for derivative
    float d_alpha;          // IIR coefficient (0 = no filter, 1 = frozen)
} PID_t;

/**
 * Initialise a PID controller.
 * @param pid       pointer to PID_t
 * @param Kp, Ki, Kd  gains
 * @param out_min/max  output saturation limits
 * @param dt        sample time [s]
 * @param d_alpha   derivative IIR filter coefficient (try 0.7)
 */
void pid_init(PID_t *pid,
              float Kp, float Ki, float Kd,
              float out_min, float out_max,
              float dt, float d_alpha);

/**
 * Run one PID update.
 * @param pid         controller instance
 * @param setpoint    desired value
 * @param measurement current process variable
 * @returns           control output
 */
float pid_update(PID_t *pid, float setpoint, float measurement);

/** Reset integrator and derivative state (e.g. on enable). */
void pid_reset(PID_t *pid);

#ifdef __cplusplus
}
#endif


/* ─── IMPLEMENTATION (single-header convenience) ─────────────
 *  Include only once in a .cpp file.  Guard with PID_IMPL.
 * ─────────────────────────────────────────────────────────── */
#ifdef PID_IMPL

#include <string.h>

void pid_init(PID_t *pid,
              float Kp, float Ki, float Kd,
              float out_min, float out_max,
              float dt, float d_alpha) {
    pid->Kp = Kp;  pid->Ki = Ki;  pid->Kd = Kd;
    pid->out_min = out_min;  pid->out_max = out_max;
    pid->dt = dt;
    pid->d_alpha = d_alpha;
    pid_reset(pid);
}

void pid_reset(PID_t *pid) {
    pid->integrator      = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->d_filter        = 0.0f;
}

float pid_update(PID_t *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;

    // Proportional
    float P = pid->Kp * error;

    // Integral (with anti-windup via clamping after sum)
    pid->integrator += pid->Ki * error * pid->dt;

    // Derivative on measurement (avoids kick)
    float d_raw = -(measurement - pid->prev_measurement) / pid->dt;
    pid->d_filter = pid->d_alpha * pid->d_filter + (1.0f - pid->d_alpha) * d_raw;
    float D = pid->Kd * pid->d_filter;
    pid->prev_measurement = measurement;

    // Sum
    float out = P + pid->integrator + D;

    // Clamp output and back-calculate integrator (anti-windup)
    if (out > pid->out_max) {
        out = pid->out_max;
        pid->integrator -= (out - pid->out_max);
    } else if (out < pid->out_min) {
        out = pid->out_min;
        pid->integrator -= (out - pid->out_min);
    }

    // Clamp integrator independently
    if (pid->integrator > pid->out_max) pid->integrator = pid->out_max;
    if (pid->integrator < pid->out_min) pid->integrator = pid->out_min;

    return out;
}

#endif /* PID_IMPL */
