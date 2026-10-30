#include "rc_control.h"
#include "cfg_rc.h"
#include "cfg_motor.h"
#include "ibus.h"
#include "motion.h"

_Static_assert(CFG_RC_THROTTLE_CH >= 1U && CFG_RC_THROTTLE_CH <= IBUS_CHANNEL_COUNT, "Throttle channel");
_Static_assert(CFG_RC_STEERING_CH >= 1U && CFG_RC_STEERING_CH <= IBUS_CHANNEL_COUNT, "Steering channel");
_Static_assert(CFG_RC_ENABLE_CH >= 1U && CFG_RC_ENABLE_CH <= IBUS_CHANNEL_COUNT, "Enable channel");
_Static_assert(CFG_RC_THROTTLE_CH != CFG_RC_STEERING_CH && CFG_RC_ENABLE_CH != CFG_RC_THROTTLE_CH &&
               CFG_RC_ENABLE_CH != CFG_RC_STEERING_CH, "RC channels must differ");
_Static_assert(CFG_RC_MAX_SPEED_HZ > 0 && CFG_RC_MAX_SPEED_HZ <= CFG_PUL_MAX_HZ, "RC speed limit");
_Static_assert(CFG_RC_PIVOT_MAX_HZ >= 0 && CFG_RC_PIVOT_MAX_HZ <= CFG_RC_MAX_SPEED_HZ, "RC pivot limit");
_Static_assert(CFG_RC_INPUT_MIN_US + CFG_RC_DEADBAND_US < CFG_RC_INPUT_CENTER_US &&
               CFG_RC_INPUT_CENTER_US + CFG_RC_DEADBAND_US < CFG_RC_INPUT_MAX_US, "RC deadband");

static RcControlStatus s_status;
static u8 s_seen_frame;
static u8 s_neutral_timing;
static u32 s_last_sequence;
static u32 s_last_frame_ms;
static u32 s_neutral_since_ms;

static s32 Rc_Abs(s32 value) { return value < 0 ? -value : value; }

static u8 Rc_InputValid(u16 value)
{
    return (u8)(value >= CFG_RC_INPUT_VALID_MIN_US && value <= CFG_RC_INPUT_VALID_MAX_US);
}

static s32 Rc_Axis(u16 raw, u8 reverse)
{
    s32 delta;
    s32 value;
    if (raw < CFG_RC_INPUT_MIN_US) { raw = CFG_RC_INPUT_MIN_US; }
    if (raw > CFG_RC_INPUT_MAX_US) { raw = CFG_RC_INPUT_MAX_US; }
    delta = (s32)raw - (s32)CFG_RC_INPUT_CENTER_US;
    if (delta > (s32)CFG_RC_DEADBAND_US)
    {
        value = (delta - (s32)CFG_RC_DEADBAND_US) * 1000 /
                (s32)(CFG_RC_INPUT_MAX_US - CFG_RC_INPUT_CENTER_US - CFG_RC_DEADBAND_US);
    }
    else if (delta < -(s32)CFG_RC_DEADBAND_US)
    {
        value = (delta + (s32)CFG_RC_DEADBAND_US) * 1000 /
                (s32)(CFG_RC_INPUT_CENTER_US - CFG_RC_INPUT_MIN_US - CFG_RC_DEADBAND_US);
    }
    else { value = 0; }
    return reverse != 0U ? -value : value;
}

static void Rc_Stop(RcState state, u8 requireLow)
{
    Motion_StopImmediate();
    s_status.state = state;
    s_status.target_left = 0;
    s_status.target_right = 0;
    s_neutral_timing = 0;
    if (requireLow != 0U) { s_status.need_enable_low = 1; }
}

void RcControl_Init(u32 nowMs)
{
    s_status.state = RC_WAIT_SIGNAL;
    s_status.throttle_us = 0;
    s_status.steering_us = 0;
    s_status.enable_us = 0;
    s_status.throttle = 0;
    s_status.steering = 0;
    s_status.target_left = 0;
    s_status.target_right = 0;
    s_status.frame_age_ms = 0;
    s_status.need_enable_low = 1;
    s_seen_frame = 0;
    s_neutral_timing = 0;
    s_last_sequence = 0;
    s_last_frame_ms = nowMs;
    s_neutral_since_ms = nowMs;
    Motion_StopImmediate();
}

void RcControl_Tick(u32 nowMs)
{
    IbusFrame frame;
    u8 fresh;
    u8 continuous;
    s32 base;
    s32 turn;
    s32 peak;
    s32 left;
    s32 right;

    if (Ibus_GetLatest(&frame) == 0U)
    {
        Rc_Stop(RC_WAIT_SIGNAL, 1);
        return;
    }
    /* RX may finish between the caller reading its tick and this snapshot. */
    if ((u32)(frame.received_ms - nowMs) <= 1U) { nowMs = frame.received_ms; }
    s_status.frame_age_ms = nowMs - frame.received_ms;
    if (s_status.frame_age_ms >= CFG_RC_FRAME_TIMEOUT_MS)
    {
        Rc_Stop(RC_FAILSAFE, 1);
        return;
    }
    fresh = (u8)(s_seen_frame == 0U || frame.sequence != s_last_sequence);
    continuous = (u8)(s_seen_frame != 0U &&
                      (u32)(frame.received_ms - s_last_frame_ms) <= CFG_RC_NEUTRAL_MAX_GAP_MS);
    if (fresh != 0U)
    {
        s_seen_frame = 1;
        s_last_sequence = frame.sequence;
        s_last_frame_ms = frame.received_ms;
    }
    s_status.throttle_us = frame.channel[CFG_RC_THROTTLE_CH - 1U];
    s_status.steering_us = frame.channel[CFG_RC_STEERING_CH - 1U];
    s_status.enable_us = frame.channel[CFG_RC_ENABLE_CH - 1U];
    if (Rc_InputValid(s_status.throttle_us) == 0U || Rc_InputValid(s_status.steering_us) == 0U ||
        Rc_InputValid(s_status.enable_us) == 0U)
    {
        Rc_Stop(RC_FAILSAFE, 1);
        return;
    }
    s_status.throttle = Rc_Axis(s_status.throttle_us, CFG_RC_THROTTLE_REVERSE);
    s_status.steering = Rc_Axis(s_status.steering_us, CFG_RC_STEERING_REVERSE);

    /* Low is an immediate stop and the required acknowledgement after startup
     * or any failsafe. Mid-position does not count as observing low. */
    if (s_status.enable_us < CFG_RC_ENABLE_HIGH_US)
    {
        if (s_status.enable_us <= CFG_RC_ENABLE_LOW_US) { s_status.need_enable_low = 0; }
        Rc_Stop(RC_DISARMED, 0);
        return;
    }
    if (s_status.need_enable_low != 0U)
    {
        Rc_Stop(RC_DISARMED, 0);
        return;
    }

    if (s_status.state != RC_STOPPED && s_status.state != RC_DRIVING)
    {
        s_status.state = RC_WAIT_NEUTRAL;
        if (s_status.throttle != 0 || s_status.steering != 0)
        {
            s_neutral_timing = 0;
        }
        else if (fresh != 0U)
        {
            if (s_neutral_timing == 0U || continuous == 0U)
            {
                s_neutral_since_ms = frame.received_ms;
                s_neutral_timing = 1;
            }
            if ((u32)(frame.received_ms - s_neutral_since_ms) >= CFG_RC_NEUTRAL_HOLD_MS)
            {
                s_status.state = RC_STOPPED;
                s_neutral_timing = 0;
            }
        }
        /* Still no output on the exact frame that completes arming. */
        Motion_StopImmediate();
        return;
    }

    if (fresh != 0U)
    {
        base = s_status.throttle * CFG_RC_MAX_SPEED_HZ / 1000;
        /* Preserve steering authority through zero throttle for a slow pivot.
         * The floor joins continuously to proportional steering at higher
         * speed. Positive steering always means nose right, also in reverse. */
        turn = Rc_Abs(base);
        if (turn < CFG_RC_PIVOT_MAX_HZ) { turn = CFG_RC_PIVOT_MAX_HZ; }
        turn = turn * s_status.steering / 1000;
        left = base + turn;
        right = base - turn;
        peak = Rc_Abs(left);
        if (Rc_Abs(right) > peak) { peak = Rc_Abs(right); }
        if (peak > CFG_RC_MAX_SPEED_HZ)
        {
            left = (s32)((int64_t)left * CFG_RC_MAX_SPEED_HZ / peak);
            right = (s32)((int64_t)right * CFG_RC_MAX_SPEED_HZ / peak);
        }
        s_status.target_left = left;
        s_status.target_right = right;
        Motion_SetTarget(left, right);
    }
    /* 非阻塞推进斜坡。
     * 注意这里叫 Motion_SlewTick() 而不是原作者的 Motion_Tick(nowMs)：
     * 本工程的 Motion_Tick(void) 已经是"掉线看门狗"，两者同名在 C 里
     * 无法共存，故把遥控这一路改名，语义不变（按 CFG_SCURVE_SAMPLE_MS
     * 节拍把当前速度朝目标推一步，绝不阻塞）。 */
    Motion_SlewTick(nowMs);
    Motion_GetSpeed(&left, &right);
    s_status.state = (left == 0 && right == 0 && s_status.target_left == 0 &&
                      s_status.target_right == 0) ? RC_STOPPED : RC_DRIVING;
}

void RcControl_GetStatus(RcControlStatus *out)
{
    if (out != 0) { *out = s_status; }
}

const char *RcControl_StateName(RcState state)
{
    switch (state)
    {
        case RC_WAIT_SIGNAL:  return "WAIT_SIGNAL";
        case RC_DISARMED:     return "DISARMED";
        case RC_WAIT_NEUTRAL: return "WAIT_NEUTRAL";
        case RC_STOPPED:      return "STOPPED";
        case RC_DRIVING:      return "DRIVING";
        case RC_FAILSAFE:     return "FAILSAFE";
        default:             return "UNKNOWN";
    }
}
