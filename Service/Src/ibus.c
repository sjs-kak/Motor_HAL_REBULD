#include "ibus.h"
#include "osal.h"

/* Standard servo frame: 20 40 + 14 little-endian u16 + little-endian checksum.
 * checksum = 0xFFFF - sum(bytes[0..29]). Independent implementation; format
 * reference: https://github.com/bmellink/IBusBM (protocol description). */
#define IBUS_BYTE_GAP_MS 5U

static u8 s_buffer[IBUS_FRAME_SIZE];
static u8 s_used;
static u32 s_last_byte_ms;
static volatile u8 s_has_frame;
static volatile IbusFrame s_latest;
static volatile IbusStats s_stats;

void Ibus_Init(void)
{
    u32 key = Osal_EnterCritical();
    u32 i;
    s_used = 0;
    s_last_byte_ms = 0;
    s_has_frame = 0;
    s_latest.received_ms = 0;
    s_latest.sequence = 0;
    for (i = 0; i < IBUS_CHANNEL_COUNT; ++i) { s_latest.channel[i] = 0; }
    s_stats.valid_frames = 0;
    s_stats.bad_checksums = 0;
    s_stats.discarded_bytes = 0;
    Osal_ExitCritical(key);
}

/* Keep a possible partial header after corruption/insertion/deletion.
 * Bounded by 32 bytes, including when payload contains 20 40. */
static void Ibus_Resync(void)
{
    u8 start;
    u8 i;
    for (start = 1; start < s_used; ++start)
    {
        if (s_buffer[start] == 0x20U &&
            (start == (u8)(s_used - 1U) || s_buffer[start + 1U] == 0x40U))
        {
            break;
        }
    }
    s_stats.discarded_bytes += start;
    for (i = start; i < s_used; ++i) { s_buffer[i - start] = s_buffer[i]; }
    s_used = (u8)(s_used - start);
}

void Ibus_FeedByte(u8 byte)
{
    u32 now = Osal_GetTickMs();
    u16 checksum = 0xFFFFU;
    u16 received;
    u32 i;

    if (s_used != 0U && (u32)(now - s_last_byte_ms) >= IBUS_BYTE_GAP_MS)
    {
        s_stats.discarded_bytes += s_used;
        s_used = 0;
    }
    s_last_byte_ms = now;
    if (s_used == 0U && byte != 0x20U)
    {
        ++s_stats.discarded_bytes;
        return;
    }
    s_buffer[s_used++] = byte;
    if (s_used == 2U && s_buffer[1] != 0x40U)
    {
        Ibus_Resync();
        return;
    }
    if (s_used != IBUS_FRAME_SIZE) { return; }

    for (i = 0; i < IBUS_FRAME_SIZE - 2U; ++i) { checksum -= s_buffer[i]; }
    received = (u16)((u16)s_buffer[30] | ((u16)s_buffer[31] << 8));
    if (checksum != received)
    {
        ++s_stats.bad_checksums;
        Ibus_Resync();
        return;
    }
    for (i = 0; i < IBUS_CHANNEL_COUNT; ++i)
    {
        s_latest.channel[i] = (u16)((u16)s_buffer[2U + i * 2U] |
                                   ((u16)s_buffer[3U + i * 2U] << 8));
    }
    s_latest.received_ms = now;
    ++s_latest.sequence;
    ++s_stats.valid_frames;
    s_has_frame = 1;
    s_used = 0;
}

u8 Ibus_GetLatest(IbusFrame *out)
{
    u32 key;
    u32 i;
    u8 available;
    if (out == 0) { return 0; }
    key = Osal_EnterCritical();
    available = s_has_frame;
    for (i = 0; i < IBUS_CHANNEL_COUNT; ++i) { out->channel[i] = s_latest.channel[i]; }
    out->received_ms = s_latest.received_ms;
    out->sequence = s_latest.sequence;
    Osal_ExitCritical(key);
    return available;
}

void Ibus_GetStats(IbusStats *out)
{
    u32 key;
    if (out == 0) { return; }
    key = Osal_EnterCritical();
    out->valid_frames = s_stats.valid_frames;
    out->bad_checksums = s_stats.bad_checksums;
    out->discarded_bytes = s_stats.discarded_bytes;
    Osal_ExitCritical(key);
}
