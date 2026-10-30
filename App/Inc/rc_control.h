#ifndef APP_RC_CONTROL_H
#define APP_RC_CONTROL_H

#include "types.h"

typedef enum
{
    RC_WAIT_SIGNAL = 0,
    RC_DISARMED,
    RC_WAIT_NEUTRAL,
    RC_STOPPED,
    RC_DRIVING,
    RC_FAILSAFE
} RcState;

typedef struct
{
    RcState state;
    u16 throttle_us;
    u16 steering_us;
    u16 enable_us;
    s32 throttle;       /* -1000..1000; positive = forward. */
    s32 steering;       /* -1000..1000; positive = turn nose right. */
    s32 target_left;
    s32 target_right;
    u32 frame_age_ms;
    u8 need_enable_low;
} RcControlStatus;

/* Main context only. Init after Motion_Init; call Tick every main-loop pass.
 * No delays, logging, HAL headers, or interrupt-side motor commands. */
void RcControl_Init(u32 nowMs);
void RcControl_Tick(u32 nowMs);
void RcControl_GetStatus(RcControlStatus *out);
const char *RcControl_StateName(RcState state);

#endif /* APP_RC_CONTROL_H */
