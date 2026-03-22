/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          3M MicroTouch Serial emulation.
 *
 * Authors: Cacodemon345, mourix
 *
 *          Copyright 2026 Cacodemon345
 */

/* Reference: https://www.touchwindow.com/mm5/drivers/mtsctlrm.pdf */

/* TODO:
    - Properly implement GP/SP commands (formats are not documented at all, like anywhere; no dumps yet).
    - Figure out undocumented commands used by Touchware Windows drivers as well.
    - Add additional SMT2/3 formats as we currently only support Tablet, Hex and Dec.
    - Add Mode Polled.
    - Add UV/NM commands.
    - Add Calibrate Raw.
*/
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/mouse.h>
#include <86box/serial.h>
#include <86box/plat.h>
#include <86box/fifo8.h>
#include <86box/fifo.h>
#include <86box/video.h>
#include <86box/nvr.h>

#define NVR_SIZE      16


enum mtouch_formats {
    FORMAT_BIN        = 1,
    FORMAT_BIN_STREAM = 2,
    FORMAT_DEC        = 3,
    FORMAT_HEX        = 4,
    FORMAT_RAW        = 5,
    FORMAT_TABLET     = 6,
    FORMAT_ZONE       = 7,
};

enum mtouch_modes {
    MODE_DOWNUP   = 1,
    MODE_INACTIVE = 2,
    MODE_POINT    = 3,
    MODE_STREAM   = 4,
};

enum mtouch_touch_states {
    TOUCH_NONE    = 0,
    TOUCH_DOWN    = 1,
    TOUCH_CONT    = 2,
    TOUCH_LIFTOFF = 3,
};

enum mtouch_cal_types {
    CAL_NONE        = 0,
    CAL_NEW         = 1,
    CAL_EXTENDED    = 2,
    CAL_INTERACTIVE = 3,
};

enum mtouch_cal_points {
    CAL_PT_DONE        = 0,
    CAL_PT_UPPER_RIGHT = 1,
    CAL_PT_LOWER_LEFT  = 2,
};

static const char *mtouch_identity[] = {
    "A30100", /* SMT2 Serial / SMT3(R)V */
    "A40100", /* SMT2 PCBus */
    "P50100", /* TouchPen 4(+) */
    "Q10100", /* SMT3(R) Serial */
};

typedef struct mouse_microtouch_t {
    char       cmd[256];
    double     abs_x, abs_x_old, abs_y, abs_y_old;
    float      scale_x, scale_y, off_x, off_y;
    int        but, but_old;
    int        baud_rate, cmd_pos;
    uint8_t    format, mode;
    uint8_t    id, cal_point, pen_mode;
    uint8_t    cal_type;
    bool       mode_status, soh;
    bool       in_reset, reset;
    uint8_t   *nvr;
    char       nvr_path[64];
    serial_t  *serial;
    Fifo8      resp;
    pc_timer_t host_to_serial_timer;
    pc_timer_t reset_timer;
} mouse_microtouch_t;

static mouse_microtouch_t *mtouch_inst = NULL;

static void
mtouch_nvr_save(mouse_microtouch_t *dev)
{
    FILE *fp;

    fp = nvr_fopen(dev->nvr_path, "wb");
    if (fp) {
        fwrite(dev->nvr, 1, NVR_SIZE, fp);
        fclose(fp);
    }
}

static void
mtouch_nvr_write(mouse_microtouch_t *dev, float scale_x, float scale_y, float off_x, float off_y)
{
    memcpy(&dev->nvr[0], &scale_x, 4);
    memcpy(&dev->nvr[4], &scale_y, 4);
    memcpy(&dev->nvr[8], &off_x, 4);
    memcpy(&dev->nvr[12], &off_y, 4);
}

static void
mtouch_nvr_read(mouse_microtouch_t *dev)
{
    memcpy(&dev->scale_x, &dev->nvr[0], 4);
    memcpy(&dev->scale_y, &dev->nvr[4], 4);
    memcpy(&dev->off_x, &dev->nvr[8], 4);
    memcpy(&dev->off_y, &dev->nvr[12], 4);

    pclog("MT NVR CAL: scale_x=%f, scale_y=%f, off_x=%f, off_y=%f\n", dev->scale_x, dev->scale_y, dev->off_x, dev->off_y);
}

static void
mtouch_nvr_init(mouse_microtouch_t *dev)
{
    FILE *fp;

    /* Allocate and initialize the EEPROM */
    dev->nvr = (uint8_t *) calloc(1, NVR_SIZE);
    if (!dev->nvr)
        fatal("mtouch_nvr_init(): Failed to allocate NVR\n");

    fp = nvr_fopen(dev->nvr_path, "rb");
    if (fp) {
        if (fread(dev->nvr, 1, NVR_SIZE, fp) != NVR_SIZE)
            fatal("mtouch_nvr_init(): Error reading data\n");
        fclose(fp);
    } else
        mtouch_nvr_write(dev, 1, 1, 0, 0);
}

/* Push a framed <SOH>payload<CR> response */
static void
mtouch_respond(mouse_microtouch_t *dev, const char *payload)
{
    fifo8_push(&dev->resp, 0x01);
    fifo8_push_all(&dev->resp, (uint8_t *) payload, strlen(payload));
    fifo8_push(&dev->resp, 0x0D);
}

/* Process calibration touch point */
static void
mtouch_calibrate(mouse_microtouch_t *dev)
{
    /* CN inverts the upper-right ack for Dec/Hex (Table 7) */
    bool invert_ack = dev->cal_point == CAL_PT_UPPER_RIGHT &&
                      dev->cal_type == CAL_NEW &&
                      (dev->format == FORMAT_DEC || dev->format == FORMAT_HEX);

    /* Reject if touch is outside expected quadrant (CI skips) */
    if (dev->cal_type != CAL_INTERACTIVE &&
        ((dev->cal_point == CAL_PT_LOWER_LEFT && (dev->abs_x > 0.25 || dev->abs_y < 0.75)) ||
         (dev->cal_point == CAL_PT_UPPER_RIGHT && (dev->abs_x < 0.75 || dev->abs_y > 0.25)))) {
        mtouch_respond(dev, invert_ack ? "1" : "0");
        return;
    }

    dev->cal_point = (dev->cal_point == CAL_PT_UPPER_RIGHT) ? CAL_PT_DONE : CAL_PT_UPPER_RIGHT;
    mtouch_respond(dev, invert_ack ? "0" : "1");

    if (dev->cal_point == CAL_PT_DONE) {
        double inset = (dev->cal_type == CAL_EXTENDED) ? 0.125 : 0.0;

        /* CI auto-swaps if the points were touched in reverse order */
        if (dev->cal_type == CAL_INTERACTIVE &&
            (dev->abs_x_old > dev->abs_x || dev->abs_y_old < dev->abs_y)) {
            double tmp;
            tmp = dev->abs_x_old; dev->abs_x_old = dev->abs_x; dev->abs_x = tmp;
            tmp = dev->abs_y_old; dev->abs_y_old = dev->abs_y; dev->abs_y = tmp;
        }

        dev->scale_x  = (1.0 - 2.0 * inset) / (dev->abs_x - dev->abs_x_old);
        dev->off_x    = inset - dev->scale_x * dev->abs_x_old;
        dev->scale_y  = (2.0 * inset - 1.0) / (dev->abs_y - dev->abs_y_old);
        dev->off_y    = (1.0 - inset) - dev->scale_y * dev->abs_y_old;
        dev->cal_type = CAL_NONE;

        pclog("MT NEW CAL: scale_x=%f, scale_y=%f, off_x=%f, off_y=%f\n", dev->scale_x, dev->scale_y, dev->off_x, dev->off_y);
        mtouch_nvr_write(dev, dev->scale_x, dev->scale_y, dev->off_x, dev->off_y);
        mtouch_nvr_save(dev);
    }

    dev->abs_x_old = dev->abs_x;
    dev->abs_y_old = dev->abs_y;
}

/* Determine touch state and drive transmissions */
static void
mtouch_handle_touch(mouse_microtouch_t *dev)
{
    uint8_t  touch_state;
    double   x, y;

    if (dev->but && !dev->but_old)
        touch_state = TOUCH_DOWN;
    else if (dev->but && dev->but_old)
        touch_state = TOUCH_CONT;
    else if (!dev->but && dev->but_old)
        touch_state = TOUCH_LIFTOFF;
    else
        touch_state = TOUCH_NONE;

    if (dev->mode == MODE_INACTIVE)
        return;

    /* Calibrate */
    if (dev->cal_point || touch_state == TOUCH_NONE) {
        if (touch_state == TOUCH_LIFTOFF)
            mtouch_calibrate(dev);
        dev->but_old = dev->but;
        return;
    }

    /* Filter events ignored by mode */
    if ((touch_state == TOUCH_CONT && dev->mode != MODE_STREAM) ||
        (touch_state == TOUCH_LIFTOFF && dev->mode == MODE_POINT)) {
        dev->but_old = dev->but;
        return;
    }

    if (touch_state == TOUCH_LIFTOFF) {
        x = dev->abs_x_old;
        y = dev->abs_y_old;
    } else {
        x = dev->abs_x;
        y = dev->abs_y;
    }

    switch (dev->format) {
        case FORMAT_TABLET: {
            uint8_t  status;
            uint16_t tx, ty;

            /* bit 7 = sync, bit 6 = touching, bit 5 = pen, bits 0-1 = buttons */
            status = 0x80 | ((touch_state != TOUCH_LIFTOFF) ? 0x40 : 0x00);
            if (dev->pen_mode == 2)
                status |= 0x20 | ((touch_state != TOUCH_LIFTOFF) ? (dev->but & 3) : 0);

            tx = (uint16_t)(16383.0 * x);
            ty = (uint16_t)(16383.0 * (1.0 - y));

            fifo8_push(&dev->resp, status);
            fifo8_push(&dev->resp, tx & 0x7F);
            fifo8_push(&dev->resp, (tx >> 7) & 0x7F);
            fifo8_push(&dev->resp, ty & 0x7F);
            fifo8_push(&dev->resp, (ty >> 7) & 0x7F);
            break;
        }
        case FORMAT_DEC: case FORMAT_HEX: {
            char        buffer[16];
            const char *fmt   = (dev->format == FORMAT_DEC) ? "%03d,%03d\r" : "%03X,%03X\r";
            uint16_t    c_max = (dev->format == FORMAT_DEC) ? 999 : 1023;
            uint8_t     status;

            if (!dev->mode_status)
                status = 0x01;
            else if (touch_state == TOUCH_DOWN)
                status = 0x19;
            else if (touch_state == TOUCH_CONT)
                status = 0x1c;
            else
                status = 0x18;

            fifo8_push(&dev->resp, status);
            snprintf(buffer, sizeof(buffer), fmt, (uint16_t)(c_max * x), (uint16_t)(c_max * (1.0 - y)));
            fifo8_push_all(&dev->resp, (uint8_t *) buffer, strlen(buffer));
            break;
        }
        case FORMAT_BIN: case FORMAT_BIN_STREAM: case FORMAT_RAW: case FORMAT_ZONE:
            pclog("MT: unsupported format %d\n", dev->format);
            break;
        default:
            break;
    }

    dev->abs_x_old = dev->abs_x;
    dev->abs_y_old = dev->abs_y;
    dev->but_old = dev->but;
}

/* Parse and execute firmware command */
static void
mtouch_process_command(mouse_microtouch_t *dev)
{
    dev->cmd[strcspn(dev->cmd, "\r")] = '\0';
    pclog("MT Command: %s\n", dev->cmd);

    if (dev->cmd[0] == 'C' && (dev->cmd[1] == 'I' || dev->cmd[1] == 'N' || dev->cmd[1] == 'X')) {
        dev->scale_x  = 1; /* Calibrate Interactive/New/Extended */
        dev->scale_y  = 1;
        dev->off_x    = 0;
        dev->off_y    = 0;
        dev->cal_point = CAL_PT_LOWER_LEFT;

        if (dev->cmd[1] == 'I')
            dev->cal_type = CAL_INTERACTIVE;
        else if (dev->cmd[1] == 'N')
            dev->cal_type = CAL_NEW;
        else
            dev->cal_type = CAL_EXTENDED;
    }
    else if (dev->cmd[0] == 'C' && dev->cmd[1] == 'R') { /* Calibrate Raw */
        pclog("MT: Calibrate Raw not implemented\n");
    }
    else if (dev->cmd[0] == 'F' && dev->cmd[1] == 'B') {
        if (dev->cmd[2] == 'S') { /* Format Binary Stream */
            dev->format = FORMAT_BIN_STREAM;
            dev->mode_status = true;
        } else { /* Format Binary */
            dev->format = FORMAT_BIN;
            dev->mode_status = true;
        }
    }
    else if (dev->cmd[0] == 'F' && dev->cmd[1] == 'D') { /* Format Decimal */
        dev->format = FORMAT_DEC;
        dev->mode_status = false;
    }
    else if (dev->cmd[0] == 'F' && dev->cmd[1] == 'H') { /* Format Hexadecimal */
        dev->format = FORMAT_HEX;
        dev->mode_status = false;
    }
    else if (dev->cmd[0] == 'F' && dev->cmd[1] == 'O') { /* Finger Only */
        dev->pen_mode = 1;
    }
    else if (dev->cmd[0] == 'F' && dev->cmd[1] == 'R') { /* Format Raw */
        dev->format    = FORMAT_RAW;
        dev->mode      = MODE_INACTIVE;
        dev->cal_point = CAL_PT_DONE;
    }
    else if (dev->cmd[0] == 'F' && dev->cmd[1] == 'T') { /* Format Tablet */
        dev->format = FORMAT_TABLET;
    }
    else if (dev->cmd[0] == 'F' && dev->cmd[1] == 'Z') { /* Format Zone */
        dev->format = FORMAT_ZONE;
        dev->mode = MODE_STREAM;
    }
    else if (dev->cmd[0] == 'G' && dev->cmd[1] == 'P' && dev->cmd[2] == '1') { /* Get Parameter Block 1 */
        mtouch_respond(dev, "A");
        fifo8_push_all(&dev->resp, (uint8_t *) "0000000000000000000000000\r", 26);
    }
    else if (dev->cmd[0] == 'M' && dev->cmd[1] == 'D' && dev->cmd[2] == 'U') { /* Mode Down/Up */
        dev->mode = MODE_DOWNUP;
    }
    else if (dev->cmd[0] == 'M' && dev->cmd[1] == 'I') { /* Mode Inactive */
        dev->mode = MODE_INACTIVE;
    }
    else if (dev->cmd[0] == 'M' && dev->cmd[1] == 'P') { /* Mode Point */
        dev->mode = MODE_POINT;
    }
    else if (dev->cmd[0] == 'M' && dev->cmd[1] == 'T') { /* Mode Status */
        dev->mode_status = true;
    }
    else if (dev->cmd[0] == 'M' && dev->cmd[1] == 'S') { /* Mode Stream */
        dev->mode = MODE_STREAM;
    }
    else if (dev->cmd[0] == 'O' && dev->cmd[1] == 'I') { /* Output Identity */
        mtouch_respond(dev, mtouch_identity[dev->id]);
        return;
    }
    else if (dev->cmd[0] == 'O' && dev->cmd[1] == 'S') { /* Output Status */
        mtouch_respond(dev, dev->reset ? "\x40\x60" : "\x40\x40");
        return;
    }
    else if (dev->cmd[0] == 'P') {
        if (strlen(dev->cmd) == 2) { /* Pen */
            if (dev->cmd[1] == 'F') dev->pen_mode = 3;      /* Pen or Finger */
            else if (dev->cmd[1] == 'O') dev->pen_mode = 2; /* Pen Only */
        }
        else if (strlen(dev->cmd) == 5) { /* Serial Options */
            if      (dev->cmd[4] == '1') dev->baud_rate = 19200;
            else if (dev->cmd[4] == '2') dev->baud_rate = 9600;
            else if (dev->cmd[4] == '3') dev->baud_rate = 4800;
            else if (dev->cmd[4] == '4') dev->baud_rate = 2400;
            else if (dev->cmd[4] == '5') dev->baud_rate = 1200;

            timer_stop(&dev->host_to_serial_timer);
            timer_on_auto(&dev->host_to_serial_timer, (1000000. / dev->baud_rate) * 10);
        }
    }
    else if (dev->cmd[0] == 'R') { /* Reset */
        dev->in_reset  = true;
        dev->cal_point = CAL_PT_DONE;
        dev->cal_type  = CAL_NONE;
        dev->pen_mode = 3;

        if (dev->cmd[1] == 'D') { /* Restore Defaults */
            dev->mode = MODE_STREAM;
            dev->mode_status = false;

            if (dev->id < 2) {
                dev->format = FORMAT_DEC;
            } else {
                dev->format = FORMAT_TABLET;
            }
        }

        timer_on_auto(&dev->reset_timer, 500. * 1000.);
        return;
    }
    else if (dev->cmd[0] == 'S' && dev->cmd[1] == 'P' && dev->cmd[2] == '1') { /* Set Parameter Block 1 */
        mtouch_respond(dev, "A");
        return;
    }
    else if (dev->cmd[0] == 'U' && dev->cmd[1] == 'T') { /* Unit Type */
        mtouch_respond(dev, (dev->id == 2) ? "TP****00" : "QM****00");
        return;
    }

    mtouch_respond(dev, "0");
}

/* Handle byte received from the host */
static void
mtouch_write(UNUSED(serial_t *serial), void *priv, uint8_t data)
{
    mouse_microtouch_t *dev = (mouse_microtouch_t *) priv;

    if (data == '\x1') {
        dev->soh = true;
    }
    else if (dev->soh) {
        if (data != '\r') {
            if (dev->cmd_pos < sizeof(dev->cmd) - 2) {
                dev->cmd[dev->cmd_pos++] = data;
            }
        } else {
            dev->soh = false;

            if (!dev->cmd_pos) {
                return;
            }

            dev->cmd[dev->cmd_pos++] = data;
            dev->cmd_pos = 0;
            mtouch_process_command(dev);
        }
    }
}

/* Send next byte to the host */
static void
mtouch_write_to_host(void *priv)
{
    mouse_microtouch_t *dev = (mouse_microtouch_t *) priv;

    if (dev->serial == NULL)
        goto no_write_to_machine;
    if ((dev->serial->type >= SERIAL_16550) && dev->serial->fifo_enabled) {
        if (fifo_get_full(dev->serial->rcvr_fifo)) {
            goto no_write_to_machine;
        }
    } else {
        if (dev->serial->lsr & 1) {
            goto no_write_to_machine;
        }
    }
    if (dev->in_reset) {
        goto no_write_to_machine;
    }
    if (fifo8_num_used(&dev->resp)) {
        serial_write_fifo(dev->serial, fifo8_pop(&dev->resp));
    }
    else {
        mtouch_handle_touch(dev);
    }

no_write_to_machine:
    timer_on_auto(&dev->host_to_serial_timer, (1000000.0 / (double) dev->baud_rate) * (double) (1 + 8 + 1));
}

/* Poll absolute coordinates and button state */
static int
mtouch_poll(void *priv)
{
    mouse_microtouch_t *dev = (mouse_microtouch_t *) priv;

    dev->but = mouse_get_buttons_ex();
    mouse_get_abs_coords(&dev->abs_x, &dev->abs_y);

    /* Adjust coordinates for overscan */
    if (enable_overscan && mouse_tablet_in_proximity > 0) {
        int index = mouse_tablet_in_proximity - 1;

        dev->abs_x *= monitors[index].mon_unscaled_size_x - 1;
        dev->abs_y *= monitors[index].mon_efscrnsz_y - 1;

        if (dev->abs_x <= (monitors[index].mon_overscan_x / 2.)) {
            dev->abs_x = (monitors[index].mon_overscan_x / 2.);
        }
        if (dev->abs_y <= (monitors[index].mon_overscan_y / 2.)) {
            dev->abs_y = (monitors[index].mon_overscan_y / 2.);
        }
        dev->abs_x -= (monitors[index].mon_overscan_x / 2.);
        dev->abs_y -= (monitors[index].mon_overscan_y / 2.);
        dev->abs_x = dev->abs_x / (double) monitors[index].mon_xsize;
        dev->abs_y = dev->abs_y / (double) monitors[index].mon_ysize;
    }

    dev->abs_x = dev->scale_x * dev->abs_x + dev->off_x;
    dev->abs_y = dev->scale_y * dev->abs_y + dev->off_y;

    if (dev->abs_x >= 1.0) dev->abs_x = 1.0;
    if (dev->abs_y >= 1.0) dev->abs_y = 1.0;
    if (dev->abs_x <= 0.0) dev->abs_x = 0.0;
    if (dev->abs_y <= 0.0) dev->abs_y = 0.0;

    return 0;
}

static void
mtouch_poll_global(void)
{
    mtouch_poll(mtouch_inst);
}

static void
mtouch_reset_complete(void *priv)
{
    mouse_microtouch_t *dev = (mouse_microtouch_t *) priv;

    dev->reset = true;
    dev->in_reset = false;
    mtouch_respond(dev, "0");
}

void *
mtouch_init(UNUSED(const device_t *info))
{
    mouse_microtouch_t *dev = calloc(1, sizeof(mouse_microtouch_t));

    dev->serial = serial_attach(device_get_config_int("port"), NULL, mtouch_write, dev);
    dev->baud_rate = 9600;
    serial_set_cts(dev->serial, 1);
    serial_set_dsr(dev->serial, 1);
    serial_set_dcd(dev->serial, 1);

    fifo8_create(&dev->resp, 256);
    timer_add(&dev->host_to_serial_timer, mtouch_write_to_host, dev, 0);
    timer_add(&dev->reset_timer, mtouch_reset_complete, dev, 0);
    timer_on_auto(&dev->host_to_serial_timer, (1000000. / dev->baud_rate) * 10);
    dev->id          = device_get_config_int("identity");
    dev->pen_mode    = 3;
    dev->mode        = MODE_STREAM;

    snprintf(dev->nvr_path, sizeof(dev->nvr_path), "mtouch_%s.nvr", mtouch_identity[dev->id]);
    mtouch_nvr_init(dev);
    mtouch_nvr_read(dev);

    if (dev->id < 2) { /* legacy controllers */
        dev->format = FORMAT_DEC;
    } else {
        dev->format = FORMAT_TABLET;
    }

    mouse_input_mode = device_get_config_int("crosshair") + 1;
    mouse_set_buttons(2);
    mouse_set_poll(mtouch_poll, dev);
    mouse_set_poll_ex(mtouch_poll_global);
    mtouch_inst = dev;

    return dev;
}

void
mtouch_close(void *priv)
{
    mouse_microtouch_t *dev = (mouse_microtouch_t *) priv;

    fifo8_destroy(&dev->resp);
    /* Detach serial port from the mouse */
    if (dev->serial && dev->serial->sd) {
        memset(dev->serial->sd, 0, sizeof(serial_device_t));
    }

    if (dev->nvr != NULL)
        free(dev->nvr);

    free(dev);
    mtouch_inst = NULL;
}

static const device_config_t mtouch_config[] = {
  // clang-format off
    {
        .name           = "port",
        .description    = "Serial Port",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "COM1", .value = 0 },
            { .description = "COM2", .value = 1 },
            { .description = "COM3", .value = 2 },
            { .description = "COM4", .value = 3 },
            { .description = ""                 }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "identity",
        .description    = "Controller",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "A3 - SMT2 Serial / SMT3(R)V", .value = 0 },
            { .description = "A4 - SMT2 PCBus",             .value = 1 },
            { .description = "P5 - TouchPen 4(+)",          .value = 2 },
            { .description = "Q1 - SMT3(R) Serial",         .value = 3 },
            { .description = ""                                        }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "crosshair",
        .description    = "Show Crosshair",
        .type           = CONFIG_BINARY,
        .default_string = NULL,
        .default_int    = 1,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

const device_t mouse_mtouch_device = {
    .name          = "3M MicroTouch (Serial)",
    .internal_name = "microtouch_touchpen",
    .flags         = DEVICE_COM,
    .local         = 0,
    .init          = mtouch_init,
    .close         = mtouch_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = mtouch_config
};
