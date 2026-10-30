#ifndef SERVICE_IBUS_H
#define SERVICE_IBUS_H

#include "types.h"

#define IBUS_CHANNEL_COUNT 14U
#define IBUS_FRAME_SIZE    32U

typedef struct
{
    u16 channel[IBUS_CHANNEL_COUNT]; /* CH1 = channel[0]. */
    u32 received_ms;
    u32 sequence;
} IbusFrame;

typedef struct
{
    u32 valid_frames;
    u32 bad_checksums;
    u32 discarded_bytes;
} IbusStats;

/* Init before starting UART RX. FeedByte runs in the RX interrupt;
 * GetLatest / GetStats copy consistent snapshots in a short critical section.
 * A bad frame NEVER refreshes received_ms. No logs/motor calls in the ISR. */
void Ibus_Init(void);
void Ibus_FeedByte(u8 byte);
u8   Ibus_GetLatest(IbusFrame *out);
void Ibus_GetStats(IbusStats *out);

#endif /* SERVICE_IBUS_H */
