/*
 * custom_status_screen.c — Dumb bitmap display + system status for left side.
 *
 * GATT service UUID : 00001523-1212-efde-1523-785feabcd123
 *
 * Characteristic 0x1525 — WriteWithoutResponse
 *   Receives 1bpp portrait frames (68×160, 1440 bytes) in chunks:
 *   [2B offset LE][2B total LE][data]
 *   On complete frame: decode + rotate 90° CW into LVGL canvas.
 *   Chunks land in a ping-pong buffer (frame_bufs[write_idx]); on frame
 *   completion write_idx/read_idx swap under irq_lock so a new frame's
 *   chunks never overwrite the buffer flush_canvas is currently decoding.
 *   Every 20th drawn frame logs received/drawn/backlog counters at INFO
 *   for diagnosing display lag vs. BLE delivery.
 *
 * Characteristic 0x1526 — Read + Notify, 4 bytes
 *   Byte 0:
 *     bit 0     : USB HID active (1 = host USB HID connected)
 *     bits [3:1]: active BLE profile index 0–4 (display as 1–5)
 *     bits [7:4]: reserved, always 0
 *   Byte 1:
 *     bits [4:0]: bonded mask — bit i set if BLE profile i has a bonded peer
 *                 (regardless of whether it's connected right now)
 *     bits [7:5]: reserved, always 0
 *   Byte 2:
 *     active layer index (zmk_keymap_highest_layer_active()), 0-based, the
 *     highest-numbered currently-active layer per ZMK's own definition of
 *     "the active layer" for stacked/momentary layers.
 *   Byte 3:
 *     zmk_wpm_get_state(), clamped to 0-255. Raw pass-through, no smoothing
 *     or "hold last nonzero" logic - ZMK's own ~5s windowed average already
 *     decays to 0 when idle, a real "not typing" signal we deliberately
 *     don't suppress (see build_status_bytes()).
 *   Notifies on BLE connect, profile/layer/WPM change, USB/endpoint change.
 *   Battery level: use standard BAS 0x180F / 0x2A19 (ZMK provides it).
 *
 * Characteristic 0x1527 — Write (WriteWithResponse), cell-grid protocol v1.1
 *   Kept alongside 0x1525 for A/B comparison during migration; 0x1525 is
 *   untouched. See zmk-companion docs/cell_grid_protocol.md for the full
 *   spec. Message types, dispatched on byte 0:
 *     0x01 LAYOUT    — run-length list of (tier_id, repeat) rows, ≤16
 *                      entries, tier_id resolved against CELL_TIERS[].
 *                      Defines the active page's row structure; rare.
 *     0x04 LAYOUT_v2 — same shape, entries carry explicit (w, h, repeat)
 *                      instead of tier_id — no shared tier table needed.
 *                      Both variants populate the same layout_rows[].
 *     0x02 CELL      — one changed cell's packed 1bpp bitmap for
 *                      (row_index, col_index) in the current LAYOUT.
 *                      bitmap_len is uint16 LE (bytes[3..4]; bitmap data
 *                      starts at byte[5]) — widened from a single byte to
 *                      support tiers whose packed bitmap exceeds 255 bytes.
 *     0x03 CLEAR     — blank the canvas (fill black + full invalidate).
 *   Firmware is stateless beyond canvas_buf + the current LAYOUT's row
 *   geometry/Y-offsets; it never tracks per-cell content. Out-of-range
 *   LAYOUT/LAYOUT_v2/CELL messages are rejected (logged, ignored) rather
 *   than applied.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <stdbool.h>
#include <lvgl.h>
#include <string.h>

#include <zmk/ble.h>
#include <zmk/usb.h>
#include <zmk/endpoints.h>
#include <zmk/keymap.h>
#include <zmk/wpm.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/wpm_state_changed.h>

#ifndef ZMK_BLE_PROFILE_COUNT
#define ZMK_BLE_PROFILE_COUNT 5
#endif

LOG_MODULE_REGISTER(kbd_display, CONFIG_ZMK_LOG_LEVEL);

#if !IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_PERIPHERAL)

/* ── Bitmap display dimensions ───────────────────────────────────────────── */

#define SRC_W        68
#define SRC_H       160
#define SRC_STRIDE    9   /* ceil(68/8) */
#define FRAME_BYTES 1440  /* SRC_STRIDE × SRC_H */
#define DST_W       160   /* LVGL landscape */
#define DST_H        68

/* ── Buffers ─────────────────────────────────────────────────────────────── */

/* Ping-pong pair: chunks always land in frame_bufs[write_idx]; on frame
 * completion write_idx/read_idx swap under irq_lock so flush_canvas never
 * decodes a buffer that's still being written by a new incoming frame. */
static uint8_t    frame_bufs[2][FRAME_BYTES];
static volatile uint8_t write_idx = 0;
static volatile uint8_t read_idx  = 1;
static lv_color_t canvas_buf[DST_W * DST_H];

static uint32_t frames_received;
static uint32_t frames_drawn;

/* ── LVGL canvas ─────────────────────────────────────────────────────────── */

static lv_obj_t *ble_canvas;
static lv_obj_t *attached_screen;

static void create_canvas(lv_obj_t *screen)
{
    ble_canvas = lv_canvas_create(screen);
    lv_canvas_set_buffer(ble_canvas, canvas_buf, DST_W, DST_H,
                         LV_IMG_CF_TRUE_COLOR);
    lv_obj_set_pos(ble_canvas, 0, 0);
    /* lv_color_black() physically renders as the panel's "on/bright" state
     * on this display — see decode_and_rotate()/cell_blit() below. Fill
     * with white so the canvas starts in the physically-off/black state
     * before the first frame/cell arrives. */
    lv_canvas_fill_bg(ble_canvas, lv_color_white(), LV_OPA_COVER);
    lv_obj_move_foreground(ble_canvas);
    attached_screen = screen;
}

/* ── Decode 1bpp portrait + rotate 90° CW → RGB565 landscape ────────────── */

/* Panel is physically inverted relative to LVGL's color constants: calling
 * lv_color_black() renders as bright/on, lv_color_white() as dark/off.
 * Confirmed against zmk-companion's app-side content: bit=1 (lit) must
 * show as the panel's on/bright state, so bit=1 -> lv_color_black() here. */
static void decode_and_rotate(const uint8_t *src)
{
    for (uint16_t py = 0; py < SRC_H; py++) {
        for (uint16_t px = 0; px < SRC_W; px++) {
            uint8_t byte = src[py * SRC_STRIDE + (px >> 3)];
            int     lit  = (byte >> (7 - (px & 7))) & 1;
            uint16_t dx  = (DST_W - 1) - py;
            uint16_t dy  = px;
            canvas_buf[dy * DST_W + dx] = lit
                ? lv_color_black()
                : lv_color_white();
        }
    }
}

static void flush_canvas(struct k_work *work)
{
    if (!ble_canvas) return;
    decode_and_rotate(frame_bufs[read_idx]);
    lv_obj_invalidate(ble_canvas);

    frames_drawn++;
    if (frames_drawn % 20 == 0) {
        LOG_INF("kbd_display: received=%u drawn=%u (backlog=%d)",
                frames_received, frames_drawn,
                (int)(frames_received - frames_drawn));
    }
}

static K_WORK_DEFINE(flush_work, flush_canvas);

/* ── Status bytes ────────────────────────────────────────────────────────── */

static void build_status_bytes(uint8_t out[4])
{
    uint8_t flags = 0;
#if IS_ENABLED(CONFIG_ZMK_USB)
    if (zmk_usb_get_conn_state() == ZMK_USB_CONN_HID) {
        flags |= BIT(0);
    }
#endif
    flags |= (uint8_t)((zmk_ble_active_profile_index() & 0x07) << 1);

    uint8_t bonded = 0;
    for (uint8_t i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        if (!zmk_ble_profile_is_open(i)) {
            bonded |= BIT(i);
        }
    }

    out[0] = flags;
    out[1] = bonded;
    /* zmk_keymap_highest_layer_active(): index of the highest-numbered
     * currently-active layer, ZMK's own definition of "the active layer"
     * for stacked/momentary layers. Truncated to uint8_t; ZMK caps layer
     * count well under 256 so this never wraps in practice. */
    out[2] = (uint8_t)zmk_keymap_highest_layer_active();
    /* Raw zmk_wpm_get_state(), no smoothing/hold-last-value logic here on
     * purpose: ZMK's own windowed average already decays to 0 within its
     * ~5s window when idle, which is a real, free "not typing" signal.
     * Suppressing that zero app-side (to keep the last nonzero reading on
     * screen) would silently reintroduce the same stale-value-shown-as-live
     * problem the custom-token staleness balloons exist to warn about, for
     * no benefit weighed against the added state. Clamped since ZMK's
     * int state is unbounded in principle; the field is only a byte. */
    int wpm = zmk_wpm_get_state();
    out[3] = (uint8_t)CLAMP(wpm, 0, 255);
}

/* ── Cell-grid protocol (0x1527) ─────────────────────────────────────────── */

typedef struct {
    uint8_t w, h;   /* cell pixel size */
    uint8_t cols;   /* max columns for this tier on a 68px-wide row */
    uint8_t bytes;  /* packed 1bpp bytes/cell = ceil(w/8) * h */
} cell_tier_t;

#define TIER_COUNT 7
static const cell_tier_t CELL_TIERS[TIER_COUNT] = {
    /* 0 small_impar  */ {  6, 10, 11, 10 },
    /* 1 small_par    */ {  8, 13,  8, 13 },
    /* 2 medium_impar */ {  9, 15,  7, 30 },
    /* 3 medium_par   */ { 11, 20,  6, 40 },
    /* 4 large_impar  */ { 13, 22,  5, 44 },
    /* 5 large_par    */ { 16, 28,  4, 56 },
    /* 6 micro        */ {  2,  2, 34,  2 },
};

/* Rows are stored as resolved geometry, not a tier index, so LAYOUT (v1,
 * tier_id-based) and LAYOUT_v2 (explicit W/H-based) can share one table:
 * v1 resolves tier_id -> {w,h,cols,bytes} once at parse time; v2 already
 * has those values. CELL never needs to know which LAYOUT variant produced
 * the row it targets. */
typedef struct {
    uint8_t  w, h;
    uint8_t  cols;
    uint16_t bytes; /* packed 1bpp bytes/cell; wire field (CELL bitmap_len)
                      * is uint16 LE, so this must not be narrower */
    uint16_t y;
} layout_row_t;

#define MAX_LAYOUT_ROWS 80
static layout_row_t layout_rows[MAX_LAYOUT_ROWS];
static uint8_t       layout_row_count;

/* Stage one physical row into tmp[*row] at cumulative Y *y, expanding
 * bounds/overflow checks shared by both LAYOUT and LAYOUT_v2. Returns
 * false (and logs) if the row would not fit. */
static bool layout_stage_row(layout_row_t *tmp, uint16_t *row, uint16_t *y,
                              uint8_t w, uint8_t h, uint8_t cols, uint16_t bytes)
{
    if (w == 0 || w > SRC_W || h == 0) {
        LOG_WRN("cell_grid: invalid cell geometry (w=%u h=%u)", w, h);
        return false;
    }
    if (*row >= MAX_LAYOUT_ROWS || (uint32_t)*y + h > SRC_H) {
        LOG_WRN("cell_grid: LAYOUT exceeds %dpx or %d-row cap "
                "(row=%u y=%u h=%u)", SRC_H, MAX_LAYOUT_ROWS, *row, *y, h);
        return false;
    }
    tmp[*row].w     = w;
    tmp[*row].h     = h;
    tmp[*row].cols  = cols;
    tmp[*row].bytes = bytes;
    tmp[*row].y     = *y;
    *y  += h;
    (*row)++;
    return true;
}

/* Deferred, coalesced invalidate: CELL writes land directly in canvas_buf
 * from the BT RX thread, but the actual LVGL invalidate is deferred to the
 * system workqueue. Multiple CELL writes for the same logical update
 * (e.g. two digits of a clock tick, two separate ATT writes) collapse into
 * one k_work_submit — if the work is still PENDING when the next CELL
 * arrives, resubmitting is a no-op, so LVGL's ~33ms render cycle never
 * observes a torn intermediate state because nothing was marked dirty yet.
 * This does not close the (much narrower) window where the work is already
 * RUNNING when a new CELL lands — Zephyr reschedules it to run again after
 * the current pass, but the in-flight invalidate can still fire on a
 * partially-updated canvas_buf. A fully race-free design would double-
 * buffer per logical update the way 0x1525's ping-pong does; this is the
 * cheap fix for the dominant case, not a proof of zero tearing. */
static void cell_invalidate_handler(struct k_work *work)
{
    if (ble_canvas) {
        lv_obj_invalidate(ble_canvas);
    }
}

static K_WORK_DEFINE(cell_invalidate_work, cell_invalidate_handler);

/* Blit a cell (portrait source coords) into canvas_buf with the same
 * 90° CW rotation decode_and_rotate() uses. Invalidate is deferred —
 * see cell_invalidate_handler(). */
static void cell_blit(uint16_t x0, uint16_t y0, uint8_t w, uint8_t h,
                       const uint8_t *bitmap)
{
    uint8_t stride = (uint8_t)((w + 7) / 8);
    for (uint8_t ly = 0; ly < h; ly++) {
        uint16_t py = y0 + ly;
        for (uint8_t lx = 0; lx < w; lx++) {
            uint16_t px   = x0 + lx;
            uint8_t  byte = bitmap[ly * stride + (lx >> 3)];
            int      lit  = (byte >> (7 - (lx & 7))) & 1;
            uint16_t dx   = (DST_W - 1) - py;
            uint16_t dy   = px;
            /* Panel-inverted, see decode_and_rotate() above. */
            canvas_buf[dy * DST_W + dx] = lit
                ? lv_color_black()
                : lv_color_white();
        }
    }

    k_work_submit(&cell_invalidate_work);
}

static void handle_layout(const uint8_t *p, uint16_t len)
{
    if (len < 2) {
        LOG_WRN("cell_grid: LAYOUT too short (len=%u)", len);
        return;
    }
    uint8_t entry_count = p[1];
    if (entry_count > 16 || (uint16_t)(2 + entry_count * 2) > len) {
        LOG_WRN("cell_grid: malformed LAYOUT (entry_count=%u len=%u)",
                entry_count, len);
        return;
    }

    static layout_row_t tmp[MAX_LAYOUT_ROWS];
    uint16_t row = 0;
    uint16_t y   = 0;

    for (uint8_t i = 0; i < entry_count; i++) {
        uint8_t tier_id = p[2 + i * 2];
        uint8_t repeat  = p[3 + i * 2];
        if (tier_id >= TIER_COUNT || repeat < 1 || repeat > 80) {
            LOG_WRN("cell_grid: malformed LAYOUT entry %u (tier=%u repeat=%u)",
                    i, tier_id, repeat);
            return;
        }
        const cell_tier_t *tier = &CELL_TIERS[tier_id];
        for (uint8_t r = 0; r < repeat; r++) {
            if (!layout_stage_row(tmp, &row, &y,
                                   tier->w, tier->h, tier->cols, tier->bytes)) {
                return;
            }
        }
    }

    memcpy(layout_rows, tmp, row * sizeof(tmp[0]));
    layout_row_count = (uint8_t)row;
}

/* LAYOUT_v2 (0x04): same wire shape as LAYOUT but entries carry explicit
 * (w, h, repeat) instead of a tier_id — firmware no longer needs to share
 * a hardcoded tier table with the app. LAYOUT (0x01, tier_id-based) stays
 * supported for older app builds; both populate the same layout_rows[]. */
static void handle_layout_v2(const uint8_t *p, uint16_t len)
{
    if (len < 2) {
        LOG_WRN("cell_grid: LAYOUT_v2 too short (len=%u)", len);
        return;
    }
    uint8_t entry_count = p[1];
    if (entry_count > 16 || (uint16_t)(2 + entry_count * 3) > len) {
        LOG_WRN("cell_grid: malformed LAYOUT_v2 (entry_count=%u len=%u)",
                entry_count, len);
        return;
    }

    static layout_row_t tmp[MAX_LAYOUT_ROWS];
    uint16_t row = 0;
    uint16_t y   = 0;

    for (uint8_t i = 0; i < entry_count; i++) {
        uint8_t w      = p[2 + i * 3];
        uint8_t h      = p[3 + i * 3];
        uint8_t repeat = p[4 + i * 3];
        if (w == 0 || w > SRC_W || h == 0 || h > SRC_H ||
            repeat < 1 || repeat > MAX_LAYOUT_ROWS) {
            LOG_WRN("cell_grid: malformed LAYOUT_v2 entry %u (w=%u h=%u repeat=%u)",
                    i, w, h, repeat);
            return;
        }
        uint8_t  cols  = (uint8_t)(SRC_W / w);
        /* w<=SRC_W(68), h<=SRC_H(160) already bound this to <=1440, well
         * within uint16_t — no separate overflow check needed now that
         * CELL's bitmap_len wire field is 2 bytes, not 1. */
        uint16_t bytes = (uint16_t)(((w + 7) / 8) * h);
        for (uint8_t r = 0; r < repeat; r++) {
            if (!layout_stage_row(tmp, &row, &y, w, h, cols, bytes)) {
                return;
            }
        }
    }

    memcpy(layout_rows, tmp, row * sizeof(tmp[0]));
    layout_row_count = (uint8_t)row;
}

static void handle_cell(const uint8_t *p, uint16_t len)
{
    if (len < 5) {
        LOG_WRN("cell_grid: CELL too short (len=%u)", len);
        return;
    }
    uint8_t  row_index  = p[1];
    uint8_t  col_index  = p[2];
    uint16_t bitmap_len = (uint16_t)p[3] | ((uint16_t)p[4] << 8);

    if (row_index >= layout_row_count) {
        LOG_WRN("cell_grid: CELL row_index %u out of range (count=%u)",
                row_index, layout_row_count);
        return;
    }
    const layout_row_t *row = &layout_rows[row_index];

    /* 32-bit arithmetic: bitmap_len is attacker/peer-controlled up to
     * 65535, and 5 + bitmap_len would wrap a uint16_t back below len. */
    if (col_index >= row->cols || bitmap_len != row->bytes ||
        (uint32_t)5 + bitmap_len > (uint32_t)len) {
        LOG_WRN("cell_grid: CELL out of range (row=%u col=%u cols=%u "
                "bitmap_len=%u expected=%u)", row_index, col_index,
                row->cols, bitmap_len, row->bytes);
        return;
    }

    uint16_t x0 = (uint16_t)col_index * row->w;
    cell_blit(x0, row->y, row->w, row->h, p + 5);
}

static void handle_clear(void)
{
    /* 0xFF bytes -> RGB565 0xFFFF (lv_color_white()'s value), which is the
     * physically-off/dark state on this panel — see decode_and_rotate(). */
    memset(canvas_buf, 0xFF, sizeof(canvas_buf));
    if (ble_canvas) {
        lv_obj_invalidate(ble_canvas);
    }
}

static ssize_t on_cell_grid_write(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  const void *buf, uint16_t len,
                                  uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(offset); ARG_UNUSED(flags);

    if (len < 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    const uint8_t *p = (const uint8_t *)buf;
    switch (p[0]) {
    case 0x01:
        handle_layout(p, len);
        break;
    case 0x02:
        handle_cell(p, len);
        break;
    case 0x04:
        handle_layout_v2(p, len);
        break;
    case 0x03:
        handle_clear();
        break;
    default:
        LOG_WRN("cell_grid: unknown msg_type 0x%02x", p[0]);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    return (ssize_t)len;
}

/* ── GATT callbacks ──────────────────────────────────────────────────────── */

static ssize_t on_bitmap_write(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               const void *buf, uint16_t len,
                               uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn); ARG_UNUSED(attr); ARG_UNUSED(offset); ARG_UNUSED(flags);

    if (len < 4) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    const uint8_t *p      = (const uint8_t *)buf;
    uint16_t chunk_offset = (uint16_t)(p[0] | (p[1] << 8));
    uint16_t total        = (uint16_t)(p[2] | (p[3] << 8));
    const uint8_t *data   = p + 4;
    uint16_t data_len     = len - 4;

    if (total != FRAME_BYTES) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    if (chunk_offset + data_len > FRAME_BYTES) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    memcpy(frame_bufs[write_idx] + chunk_offset, data, data_len);
    if (chunk_offset + data_len == FRAME_BYTES) {
        frames_received++;
        /* Only rotate the ping-pong pair if the consumer is done with the
         * buffer it would become read_idx for. k_work_submit() on a
         * still-pending/running item is a silent no-op (see
         * cell_invalidate_handler() above for the same property), so
         * without this guard the previous unconditional swap could hand
         * flush_canvas a buffer that a subsequent frame's chunks are
         * concurrently overwriting -> torn frame, not just a dropped one.
         * Dropping the frame here (frames_received keeps counting it,
         * frames_drawn does not) is the same "backlog" the periodic log
         * already reports; it just no longer corrupts frame_bufs[]. */
        if (k_work_busy_get(&flush_work) != 0) {
            return (ssize_t)len;
        }
        unsigned int key = irq_lock();
        uint8_t completed = write_idx;
        write_idx = read_idx;
        read_idx  = completed;
        irq_unlock(key);
        k_work_submit(&flush_work);
    }
    return (ssize_t)len;
}

static ssize_t on_status_read(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset)
{
    uint8_t status[4];
    build_status_bytes(status);
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             status, sizeof(status));
}

static void on_status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (value == BT_GATT_CCC_NOTIFY) {
        /* Client just subscribed — send current state immediately */
        uint8_t status[4];
        build_status_bytes(status);
        bt_gatt_notify(NULL, attr - 1, status, sizeof(status));
    }
}

/* ── GATT service ────────────────────────────────────────────────────────── */

#define KBD_DISPLAY_SVC_UUID \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE( \
        0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123ULL))
#define KBD_BITMAP_CHAR_UUID \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE( \
        0x00001525, 0x1212, 0xefde, 0x1523, 0x785feabcd123ULL))
#define KBD_STATUS_CHAR_UUID \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE( \
        0x00001526, 0x1212, 0xefde, 0x1523, 0x785feabcd123ULL))
#define KBD_CELLGRID_CHAR_UUID \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE( \
        0x00001527, 0x1212, 0xefde, 0x1523, 0x785feabcd123ULL))

BT_GATT_SERVICE_DEFINE(keyboard_display_svc,
    BT_GATT_PRIMARY_SERVICE(KBD_DISPLAY_SVC_UUID),
    /* 0x1525 — bitmap write (attrs[1]=decl, attrs[2]=value) */
    BT_GATT_CHARACTERISTIC(KBD_BITMAP_CHAR_UUID,
        BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE, NULL, on_bitmap_write, NULL),
    /* 0x1526 — status read+notify (attrs[3]=decl, attrs[4]=value, attrs[5]=CCC) */
    BT_GATT_CHARACTERISTIC(KBD_STATUS_CHAR_UUID,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ, on_status_read, NULL, NULL),
    BT_GATT_CCC(on_status_ccc_changed,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    /* 0x1527 — cell-grid write (attrs[6]=decl, attrs[7]=value) */
    BT_GATT_CHARACTERISTIC(KBD_CELLGRID_CHAR_UUID,
        BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE, NULL, on_cell_grid_write, NULL),
);

/* ── ZMK event listener — notifies 0x1526 on profile/USB change ─────────── */

static int status_event_listener(const zmk_event_t *eh)
{
    uint8_t status[4];
    build_status_bytes(status);
    bt_gatt_notify(NULL, &keyboard_display_svc.attrs[4], status, sizeof(status));
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(kbd_status, status_event_listener);
ZMK_SUBSCRIPTION(kbd_status, zmk_ble_active_profile_changed);
ZMK_SUBSCRIPTION(kbd_status, zmk_usb_conn_state_changed);
ZMK_SUBSCRIPTION(kbd_status, zmk_endpoint_changed);
ZMK_SUBSCRIPTION(kbd_status, zmk_layer_state_changed);
ZMK_SUBSCRIPTION(kbd_status, zmk_wpm_state_changed);

/* ── BLE link diagnostics + latency fix ──────────────────────────────────────
 *
 * Session history: the 0x1525 GATT handler and its consumer are both
 * effectively free (a memcpy per chunk, a ~11k-iteration decode once/frame),
 * confirmed via kbd_display's received/drawn/backlog counter staying at
 * backlog=0 during a live repro. The companion app's own send path was also
 * measured clean (~40-90ms per frame, steady, no growth). Neither explains
 * the ~2-3s visual display lag the user confirmed while watching the panel
 * during that same clean-looking capture - so the gap is on the physical
 * BLE link, after the app's WriteWithoutResponse call returns and before
 * this peripheral's radio actually notices the data.
 *
 * A live connect log measured interval=15ms (fine) but latency=30: this
 * peripheral is allowed to skip listening for up to 30 consecutive
 * connection events before it must listen again, i.e. new data can sit
 * unseen for up to (latency+1)*interval =~465ms in the worst case, and if
 * that resync cost is paid repeatedly across a frame's chunks rather than
 * once, it plausibly compounds into the reported multi-second lag.
 * on_connected() below requests latency=0 to remove that stall, at the
 * cost of the radio no longer being allowed to sleep between events while
 * connected (battery trade-off, accepted for this test).
 *
 * A first attempt also requested 2M PHY on connect; that failed on this
 * hardware (HCI status 0x1a, unsupported remote feature - logged as
 * `Failed LE Set PHY (-5)`) and has been removed. PHY is still logged
 * read-only for visibility. */

static void log_conn_params(struct bt_conn *conn, const char *why)
{
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) != 0 || info.type != BT_CONN_TYPE_LE) {
        return;
    }
#if defined(CONFIG_BT_USER_PHY_UPDATE)
    uint8_t tx_phy = info.le.phy ? info.le.phy->tx_phy : 0;
    uint8_t rx_phy = info.le.phy ? info.le.phy->rx_phy : 0;
#else
    uint8_t tx_phy = 0, rx_phy = 0;
#endif
    LOG_INF("kbd_display: %s interval=%u.%02ums latency=%u timeout=%ums phy=tx%u/rx%u",
             why,
             (info.le.interval * 125) / 100, (info.le.interval * 125) % 100,
             info.le.latency, info.le.timeout * 10, tx_phy, rx_phy);
}

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) return;
    log_conn_params(conn, "connected,");

    /* Measured on this hardware: interval=15ms (fine) but latency=30,
     * i.e. the peripheral may sleep through up to 30 consecutive
     * connection events before it's required to listen again. A
     * WriteWithoutResponse burst that lands while we're deep in that
     * skip cycle can sit unseen for up to (latency+1)*interval =~465ms
     * before our radio next listens - and if that resync cost is paid
     * per chunk rather than once per frame, it plausibly adds up to the
     * multi-second display lag reported against the companion app.
     * Requesting latency=0 trades idle battery life (radio must respond
     * every interval instead of sleeping) for eliminating that stall
     * entirely. Interval range kept close to the observed 15ms so this
     * is purely a latency change, not an interval renegotiation. */
    static const struct bt_le_conn_param low_latency_param =
        BT_LE_CONN_PARAM_INIT(6, 12, 0, 400);
    bt_conn_le_param_update(conn, &low_latency_param);
}

static void on_le_param_updated(struct bt_conn *conn, uint16_t interval,
                                uint16_t latency, uint16_t timeout)
{
    ARG_UNUSED(latency); ARG_UNUSED(timeout);
    log_conn_params(conn, "params updated,");
}

BT_CONN_CB_DEFINE(kbd_display_conn_cb) = {
    .connected        = on_connected,
    .le_param_updated = on_le_param_updated,
};

/* ── Canvas lifecycle ────────────────────────────────────────────────────── */

static void ensure_timer_cb(lv_timer_t *timer)
{
    lv_obj_t *screen = lv_scr_act();
    if (!screen) return;
    if (ble_canvas && attached_screen == screen) {
        lv_obj_move_foreground(ble_canvas);
        return;
    }
    if (ble_canvas) {
        lv_obj_del(ble_canvas);
        ble_canvas = NULL;
    }
    create_canvas(screen);
}

static void add_canvas_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(add_canvas_work, add_canvas_fn);

static void add_canvas_fn(struct k_work *work)
{
    lv_obj_t *screen = lv_scr_act();
    if (!screen) {
        k_work_schedule(&add_canvas_work, K_MSEC(500));
        return;
    }
    create_canvas(screen);
    lv_timer_create(ensure_timer_cb, 1000, NULL);
}

static int kbd_ble_display_init(void)
{
    k_work_schedule(&add_canvas_work, K_MSEC(3000));
    return 0;
}
SYS_INIT(kbd_ble_display_init, APPLICATION, 99);

#endif /* !CONFIG_ZMK_SPLIT_BLE_ROLE_PERIPHERAL */
