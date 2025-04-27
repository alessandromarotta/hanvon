/*
* =====================================================================================
*
*       Filename:  hanvon-libusb.c
*
*    Description:  libusb Hanvon tablet driver (Userspace)
*
*        Version:  0.2 (Corrected)
*        Created:  08/17/2020 04:05:14 PM
*       Revision:  2025-04-27
*       Compiler:  gcc
*
*  Maintained by:  scuti@teknik.io
*                  surkeh@protonmail.com
*      Corrections: GitHub Copilot
*
* =====================================================================================
*/

// Use ##__VA_ARGS__ for portability with zero arguments
#define DEBUG(msg,...) fprintf(stderr,"%s(%d): " msg "\n", __FILE__,__LINE__, ##__VA_ARGS__)

#include <stdio.h>
#include <stdlib.h>
#include <string.h> // For memset, strerror
#include <errno.h>  // For error codes like EINVAL
#include <signal.h> // For signal handling
#include <unistd.h> // For pause() or sleep() if needed
#include <math.h>   // For abs
#include <endian.h> // For htobe16/be16toh (if needed, currently using manual shifts)

#include <libusb-1.0/libusb.h>
#include <libevdev/libevdev.h>
#include <libevdev/libevdev-uinput.h>

// Define standard button codes if not included by libevdev headers directly
// These might vary slightly depending on your kernel headers version
#ifndef BTN_0
#define BTN_0 0x100
#endif
#ifndef BTN_1
#define BTN_1 0x101
#endif
#ifndef BTN_2
#define BTN_2 0x102
#endif
#ifndef BTN_3
#define BTN_3 0x103
#endif
#ifndef BTN_4
#define BTN_4 0x104
#endif
#ifndef BTN_5
#define BTN_5 0x105
#endif
#ifndef BTN_6
#define BTN_6 0x106
#endif
#ifndef BTN_7
#define BTN_7 0x107
#endif
#ifndef BTN_MIDDLE
#define BTN_MIDDLE 0x112
#endif


#define VENDOR_ID_HANVON        0x0b57
#define PRODUCT_ID_AM3M         0x8528
#define PRODUCT_ID_AM0806       0x8502
#define PRODUCT_ID_AM0605       0x8503
#define PRODUCT_ID_AM1107       0x8505
#define PRODUCT_ID_AM1209       0x8501
#define PRODUCT_ID_RL0604       0x851f
#define PRODUCT_ID_RL0504       0x851d
#define PRODUCT_ID_GP0806       0x8039
#define PRODUCT_ID_GP0806B      0x8511
#define PRODUCT_ID_GP0605       0x8512
#define PRODUCT_ID_GP0605A      0x803a
#define PRODUCT_ID_GP0504       0x8037
#define PRODUCT_ID_NXS1513      0x8030
#define PRODUCT_ID_GP0906       0x8521
#define PRODUCT_ID_APPIV0906    0x8532

#define AM_PACKET_LEN           10
#define AM_RESOLUTION           40 // Dots per mm? Check kernel driver or specs
#define AM_WHEEL_THRESHOLD      4

// Default max coordinates (check per device if necessary)
#define AM_MAX_ABS_X            0x27DE
#define AM_MAX_ABS_Y            0x1CFE
#define AM_MAX_TILT_X           0x3F // Check if signed or unsigned
#define AM_MAX_TILT_Y           0x7F // Check if signed or unsigned
#define AM_MAX_PRESSURE         0x400 // 1024 levels

// APPIV0906 specific max coordinates
#define APPIV_MAX_ABS_X         0x5750
#define APPIV_MAX_ABS_Y         0x3692 // Kernel driver uses this, not 0x5750

// Message types from device
#define BUTTON_EVENT_GP         0x01 // General purpose button/wheel event
#define PEN_EVENT               0x02 // Pen movement/status event
#define BUTTON_EVENT_0906       0x0C // Specific button event for GP0906/APPIV0906

// Button mappings
static int lbuttons[]={BTN_0,BTN_1,BTN_2,BTN_3};   /* reported on most AM/GP */
static int rbuttons[]={BTN_4,BTN_5,BTN_6,BTN_7};   /* reported on AM1107/AM1209 */

// Structure overlay for PEN_EVENT (Use with caution, verify offsets)
// Note: Endianness of shorts depends on device protocol (often Big Endian)
struct hanvon_pen_message {
    unsigned char msgtype;      // Should be PEN_EVENT (0x02)
    unsigned char status;       // Contains flags for proximity, touch, buttons
    unsigned char x_hi;         // X coordinate high byte
    unsigned char x_lo;         // X coordinate low byte
    unsigned char y_hi;         // Y coordinate high byte
    unsigned char y_lo;         // Y coordinate low byte
    unsigned char pressure_hi;  // Pressure high byte (often only low byte used or shifted)
    unsigned char pressure_lo;  // Pressure low byte
    unsigned char tilt_x;       // Tilt X value
    unsigned char tilt_y;       // Tilt Y value
};

// GLOBAL state (RETAINS SINGLE DEVICE LIMITATION)
// TODO: Refactor to handle multiple devices using a context struct list
static libusb_device_handle *g_dev_handle = NULL;
static struct libevdev_uinput *g_uidev = NULL;
static struct libevdev *g_evdev = NULL;
static struct libusb_transfer *g_tx = NULL;
static unsigned char g_buffer[AM_PACKET_LEN];
static volatile sig_atomic_t g_running = 1; // Flag for main loop termination
static int g_wheel_position = 0; // Global wheel position state

// Forward declarations
int init_ctrl(struct libusb_device *dev, struct libevdev **evdev, struct libevdev_uinput **uidev);
void callback_default (struct libusb_transfer *tx);


// Finds the first supported Hanvon device in the list
int find_device(libusb_device **list, unsigned int count) {
    // Check for invalid count (though unsigned can't be < 0)
    if (list == NULL) {
        return -1;
    }

    struct libusb_device_descriptor desc;
    for (unsigned int i = 0; i < count; i++) {
        if (list[i] == NULL) continue; // Skip null entries if any

        int rc = libusb_get_device_descriptor(list[i], &desc);
        if (rc < 0) {
            DEBUG("Failed to get device descriptor for device %u", i);
            continue; // Skip devices we can't query
        }

        if (desc.idVendor == VENDOR_ID_HANVON) {
            switch(desc.idProduct) {
                // Add all known supported product IDs here
                case PRODUCT_ID_AM3M:
                case PRODUCT_ID_AM0806:
                case PRODUCT_ID_AM0605:
                case PRODUCT_ID_AM1107:
                case PRODUCT_ID_AM1209:
                case PRODUCT_ID_RL0604:
                case PRODUCT_ID_RL0504:
                case PRODUCT_ID_GP0806:
                case PRODUCT_ID_GP0806B:
                case PRODUCT_ID_GP0605:
                case PRODUCT_ID_GP0605A:
                case PRODUCT_ID_GP0504:
                case PRODUCT_ID_NXS1513:
                case PRODUCT_ID_GP0906:
                case PRODUCT_ID_APPIV0906:
                    DEBUG("Found supported Hanvon device %04x:%04x at index %u", desc.idVendor, desc.idProduct, i);
                    return i; // Return index of the first found supported device
                default:
                    DEBUG("Found unsupported Hanvon device %04x:%04x", desc.idVendor, desc.idProduct);
                    break; // Continue searching
            }
        }
    }
    return -1; // No supported device found
}

// Helper to display packet data for debugging
void display_packets(const unsigned char* buf, size_t len) {
    fprintf(stderr, "Packet: ");
    for(size_t i = 0; i < len; i++) {
        fprintf(stderr,"0x%02x ", buf[i]); // Use %02x for consistent byte formatting
    }
    fprintf(stderr,"\n"); // Use newline instead of carriage return
}

// Helper function to report button events
static inline void report_buttons( struct libevdev_uinput *ud,
                                   int buttons[], // Array of BTN_ codes
                                   size_t num_buttons, // Size of the buttons array
                                   unsigned char data) // Byte containing button flags
{
    int err = 0;
    // Check specific pattern for AM/GP buttons (data[2] or data[4])
    if ((data & 0xf0) == 0xa0) {
        // These seem to map to buttons 1, 2, 3 in the array (index 1, 2, 3)
        // Ensure array bounds are checked
        if (num_buttons > 1) {
            err = libevdev_uinput_write_event(ud, EV_KEY, buttons[1], !!(data & 0x02)); // Use !! for bool
            if(err) { DEBUG("err reporting button %d: %d", buttons[1], err); }
        }
        if (num_buttons > 2) {
            err = libevdev_uinput_write_event(ud, EV_KEY, buttons[2], !!(data & 0x04));
            if(err) { DEBUG("err reporting button %d: %d", buttons[2], err); }
        }
        if (num_buttons > 3) {
            err = libevdev_uinput_write_event(ud, EV_KEY, buttons[3], !!(data & 0x08));
            if(err) { DEBUG("err reporting button %d: %d", buttons[3], err); }
        }
    } else if (data <= 0x3f) {   /* slider/wheel area active */
        // Calculate delta relative to the last known position
        int delta = data - g_wheel_position;

        // Handle wrap-around (e.g., if wheel goes from 0x3f to 0x00 or vice-versa)
        // This simple logic might need adjustment based on actual wheel behavior
        if (abs(delta) > (0x3f / 2)) { // Heuristic for wrap-around
             if (delta > 0) delta -= 0x40; // Wrapped from low to high
             else delta += 0x40; // Wrapped from high to low
        }

        // Report only if movement exceeds threshold (prevents jitter)
        // Note: Kernel driver reports if abs(delta) < threshold, which seems inverted.
        // Reporting if delta is non-zero might be simpler unless thresholding is needed.
        if (delta != 0) { // Report any change
        // if (abs(delta) >= AM_WHEEL_THRESHOLD) { // Report if change meets threshold
            err = libevdev_uinput_write_event(ud, EV_REL, REL_WHEEL, delta);
            if(err) { DEBUG("err reporting wheel: %d", err); }
            g_wheel_position = data; // Update position only after reporting
        }
    }
    // Note: Button 0 (index 0) seems unhandled here, might be eraser/tool button handled elsewhere
}


// Main callback function to handle incoming USB interrupt data
void callback_default (struct libusb_transfer *tx) {
    // Check transfer status first
    if (tx->status != LIBUSB_TRANSFER_COMPLETED) {
        DEBUG("Transfer failed or cancelled: %s (%d)", libusb_error_name(tx->status), tx->status);
        // Do not resubmit if cancelled or failed critically
        // Resources (like tx itself) might be freed elsewhere based on status
        return;
    }

    unsigned char *data = tx->buffer;
    struct libevdev_uinput *ud = tx->user_data;
    int err = 0;

    // Ensure user_data (uidev) is valid
    if (!ud) {
        DEBUG("Error: uinput device handle is NULL in callback.");
        // Cannot report events, maybe try resubmitting? Risky.
        goto resubmit; // Try resubmitting anyway, but log the error
    }

    // Optional: Display raw packet data for debugging
    // display_packets(data, tx->actual_length);

    // Ensure we have enough data (at least 1 byte for msgtype)
    if (tx->actual_length < 1) {
        DEBUG("Received empty or too short packet (%d bytes)", tx->actual_length);
        goto resubmit;
    }

    // Process based on message type (first byte)
    switch(data[0]) {
        case BUTTON_EVENT_GP: // General buttons/wheel (AM/GP series)
            // Check length for safety
            if (tx->actual_length < 5) {
                 DEBUG("BUTTON_EVENT_GP packet too short (%d bytes)", tx->actual_length);
                 break;
            }
            // Left side buttons/wheel use data[2]
            if(data[1] == 0x55) {
                report_buttons(ud, lbuttons, sizeof(lbuttons)/sizeof(lbuttons[0]), data[2]);
            }
            // Right side buttons/wheel use data[4] (AM1107, AM1209)
            if(data[3] == 0xAA) {
                report_buttons(ud, rbuttons, sizeof(rbuttons)/sizeof(rbuttons[0]), data[4]);
            }
            break;

        case PEN_EVENT: // Pen movement/status
            // Check length for safety
            if (tx->actual_length < AM_PACKET_LEN) { // Expect full packet length
                 DEBUG("PEN_EVENT packet too short (%d bytes)", tx->actual_length);
                 break;
            }
            // data[1] contains status flags:
            // 0x80: Pen near sensor (in proximity)
            // 0x10: Pen lifted but was near (sometimes combined with 0x80?)
            // 0x01: Pen touching surface (BTN_LEFT)
            // 0x02: Pen side button pressed (BTN_RIGHT)
            // 0x04: Pen side button 2 pressed (BTN_MIDDLE, if exists)
            // 0x20: Eraser end active (BTN_TOOL_RUBBER) - Check if device supports this

            // Report tool type (Pen or Eraser)
            // Assuming data[1] & 0x20 indicates eraser, otherwise pen
            // Note: Kernel driver uses lbuttons[0] (BTN_0) for eraser? Verify this.
            // Let's report BTN_TOOL_PEN vs BTN_TOOL_RUBBER based on 0x20 flag
            int tool_type = (data[1] & 0x20) ? BTN_TOOL_RUBBER : BTN_TOOL_PEN;
            err = libevdev_uinput_write_event(ud, EV_KEY, tool_type, !!(data[1] & (0x80 | 0x10 | 0x01))); // Report tool active if near or touching
            if(err) { DEBUG("err reporting tool type: %d", err); }

            // Report proximity/movement only if pen is near or touching
            if (data[1] & (0x80 | 0x10 | 0x01)) {
                // Coordinates are typically Big Endian (check device specifics)
                uint16_t x_raw = ((uint16_t)data[2] << 8) | data[3];
                uint16_t y_raw = ((uint16_t)data[4] << 8) | data[5];

                // TODO: Handle potential Little Endian for APPIV0906 if needed
                // struct libusb_device_descriptor desc; // Need device context here
                // libusb_get_device_descriptor(libusb_get_device(tx->dev_handle), &desc);
                // if (desc.idProduct == PRODUCT_ID_APPIV0906) {
                //     x_raw = ((uint16_t)data[3] << 8) | data[2]; // LE
                //     y_raw = ((uint16_t)data[5] << 8) | data[4]; // LE
                // }

                err = libevdev_uinput_write_event(ud, EV_ABS, ABS_X, x_raw);
                if(err) { DEBUG("err reporting ABS_X: %d", err); }
                err = libevdev_uinput_write_event(ud, EV_ABS, ABS_Y, y_raw);
                if(err) { DEBUG("err reporting ABS_Y: %d", err); }

                // Pressure (often uses lower 10 bits of a 16-bit field, check device)
                // Kernel driver uses get_unaligned_be16(&data[6]) >> 6
                // This implies data[6] is high byte, data[7] is low byte, using top 10 bits.
                uint16_t pressure_raw = ((uint16_t)data[6] << 8) | data[7];
                // Scale pressure if needed, kernel driver doesn't scale here but sets max to 0x400 (1024)
                // The >> 6 effectively scales it to 0-1023.
                err = libevdev_uinput_write_event(ud, EV_ABS, ABS_PRESSURE, pressure_raw >> 6);
                if(err) { DEBUG("err reporting ABS_PRESSURE: %d", err); }

                // Tilt (check range and sign)
                // Kernel driver uses data[7] & 0x3f for Tilt X, data[8] for Tilt Y
                // This suggests Tilt X might be 6-bit, Tilt Y 8-bit.
                // Assuming they are unsigned for now. Max values set in init_ctrl.
                err = libevdev_uinput_write_event(ud, EV_ABS, ABS_TILT_X, data[8]); // Kernel uses data[7] & 0x3f? Check byte order/meaning
                if(err) { DEBUG("err reporting ABS_TILT_X: %d", err); }
                err = libevdev_uinput_write_event(ud, EV_ABS, ABS_TILT_Y, data[9]); // Kernel uses data[8]? Check byte order/meaning
                if(err) { DEBUG("err reporting ABS_TILT_Y: %d", err); }
            }

            // Report pen touch (BTN_LEFT) based on 0x01 flag
            err = libevdev_uinput_write_event(ud, EV_KEY, BTN_TOUCH, !!(data[1] & 0x01)); // Use BTN_TOUCH for surface contact
            if(err) { DEBUG("err reporting BTN_TOUCH: %d", err); }

            // Report pen side buttons (BTN_RIGHT, BTN_MIDDLE) based on 0x02, 0x04 flags
            err = libevdev_uinput_write_event(ud, EV_KEY, BTN_STYLUS, !!(data[1] & 0x02)); // Use BTN_STYLUS for first side button
            if(err) { DEBUG("err reporting BTN_STYLUS: %d", err); }
            // Check for a second side button flag if applicable (e.g., 0x04)
            // err = libevdev_uinput_write_event(ud, EV_KEY, BTN_STYLUS2, !!(data[1] & 0x04));
            // if(err) { DEBUG("err reporting BTN_STYLUS2: %d", err); }

            break;

        case BUTTON_EVENT_0906: // Specific buttons for GP0906/APPIV0906
             // Check length for safety
            if (tx->actual_length < 4) { // Need at least 4 bytes based on kernel driver
                 DEBUG("BUTTON_EVENT_0906 packet too short (%d bytes)", tx->actual_length);
                 break;
            }
            // Kernel driver uses data[3] for button flags
            // Map these flags to appropriate BTN_ codes (using lbuttons for consistency?)
            err = libevdev_uinput_write_event(ud, EV_KEY, lbuttons[0], !!(data[3] & 0x01)); if(err) { DEBUG("err: %d",err); }
            err = libevdev_uinput_write_event(ud, EV_KEY, lbuttons[1], !!(data[3] & 0x02)); if(err) { DEBUG("err: %d",err); }
            err = libevdev_uinput_write_event(ud, EV_KEY, lbuttons[2], !!(data[3] & 0x04)); if(err) { DEBUG("err: %d",err); }
            err = libevdev_uinput_write_event(ud, EV_KEY, lbuttons[3], !!(data[3] & 0x08)); if(err) { DEBUG("err: %d",err); }
            // APPIV0906 might have more buttons (up to BTN_7 in kernel driver)
            // Check if data[3] contains more flags or if they are in other bytes
            // Example if flags continue in data[3]:
            // err = libevdev_uinput_write_event(ud, EV_KEY, rbuttons[0], !!(data[3] & 0x10)); if(err) { DEBUG("err: %d",err); } // BTN_4
            // err = libevdev_uinput_write_event(ud, EV_KEY, rbuttons[1], !!(data[3] & 0x20)); if(err) { DEBUG("err: %d",err); } // BTN_5
            // err = libevdev_uinput_write_event(ud, EV_KEY, rbuttons[2], !!(data[3] & 0x40)); if(err) { DEBUG("err: %d",err); } // BTN_6
            // err = libevdev_uinput_write_event(ud, EV_KEY, rbuttons[3], !!(data[3] & 0x80)); if(err) { DEBUG("err: %d",err); } // BTN_7
            break;

        default:
            DEBUG("Unknown message type received: 0x%02x", data[0]);
            // Optional: display_packets(data, tx->actual_length);
            break;
    }

    // Send SYN_REPORT to signal end of event batch
    err = libevdev_uinput_write_event(ud, EV_SYN, SYN_REPORT, 0);
    if (err != 0) {
        DEBUG("Error writing EV_SYN: %d (%s)", err, strerror(-err));
    }

resubmit:
    // Resubmit the transfer for the next interrupt packet
    // Only if the program is still supposed to be running
    if (g_running && g_dev_handle != NULL) { // Check handle too
        err = libusb_submit_transfer(tx);
        if (err != 0) {
            DEBUG("Error resubmitting transfer: %s (%d)", libusb_error_name(err), err);
            // If resubmit fails, the device might stop reporting.
            // Consider closing the device handle or attempting recovery.
            // For now, just log the error. The loop in main will continue.
        }
    } else {
         DEBUG("Not resubmitting transfer (running=%d, handle=%p)", g_running, (void*)g_dev_handle);
         // If not resubmitting, the transfer object might need freeing,
         // but libusb often handles this internally or it's done during cleanup.
    }
}

// Initializes the libevdev device based on USB device descriptor
int init_ctrl(struct libusb_device *dev,
            struct libevdev **evdev_out,
            struct libevdev_uinput **uidev_out) {

    struct input_absinfo abs; // Use stack allocation for absinfo
    memset(&abs, 0, sizeof(abs)); // Important: Initialize the struct

    printf("Initializing evdev controls...\n");
    g_wheel_position = 0; // Reset global wheel position

    if (dev == NULL) {
        DEBUG("init_ctrl called with NULL device");
        return -EINVAL; // Invalid argument
    }

    int rc = 0; // Use standard Linux error codes (negative)
    struct libusb_device_descriptor desc;
    rc = libusb_get_device_descriptor(dev, &desc);
    if (rc < 0) {
        DEBUG("Failed to get device descriptor: %s", libusb_error_name(rc));
        return -EIO; // Input/output error
    }

    *evdev_out = libevdev_new();
    if (!*evdev_out) {
        DEBUG("Failed to create evdev device: %s", strerror(errno));
        return -ENOMEM; // Out of memory
    }
    struct libevdev *evdev = *evdev_out; // Use local variable for convenience

    // --- Set common device properties ---
    libevdev_set_name(evdev, "Hanvon Tablet (Userspace)"); // Generic name first
    libevdev_set_id_vendor(evdev, desc.idVendor);
    libevdev_set_id_product(evdev, desc.idProduct);
    libevdev_set_id_bustype(evdev, BUS_USB);
    libevdev_set_id_version(evdev, desc.bcdDevice);
    libevdev_enable_property(evdev, INPUT_PROP_POINTER); // Indicate it's a pointer device
    // INPUT_PROP_DIRECT for absolute screen mapping, INPUT_PROP_POINTING_STICK if relative
    // Choose based on typical tablet usage (absolute)
    libevdev_enable_property(evdev, INPUT_PROP_DIRECT);

    // --- Enable common event types and codes ---
    libevdev_enable_event_type(evdev, EV_SYN);
    libevdev_enable_event_code(evdev, EV_SYN, SYN_REPORT, NULL);

    libevdev_enable_event_type(evdev, EV_KEY);
    // Common tool types and buttons
    libevdev_enable_event_code(evdev, EV_KEY, BTN_TOOL_PEN, NULL);
    libevdev_enable_event_code(evdev, EV_KEY, BTN_TOOL_RUBBER, NULL); // Enable eraser tool if supported
    libevdev_enable_event_code(evdev, EV_KEY, BTN_TOUCH, NULL);      // Pen touch/tap
    libevdev_enable_event_code(evdev, EV_KEY, BTN_STYLUS, NULL);     // First pen side button
    libevdev_enable_event_code(evdev, EV_KEY, BTN_STYLUS2, NULL);    // Second pen side button (if exists)

    libevdev_enable_event_type(evdev, EV_ABS);
    // Absolute axes: X, Y, Pressure, Tilt X, Tilt Y
    // Configure X axis
    abs.minimum = 0;
    abs.maximum = (desc.idProduct == PRODUCT_ID_APPIV0906) ? APPIV_MAX_ABS_X : AM_MAX_ABS_X;
    abs.resolution = AM_RESOLUTION; // Dots per mm (needs verification)
    abs.fuzz = 4; // Adjust if needed based on jitter
    abs.flat = 0; // Adjust if needed
    // abs.value = 0; // Initial value not needed here
    rc = libevdev_enable_event_code(evdev, EV_ABS, ABS_X, &abs);
    if (rc < 0) { DEBUG("Failed to enable ABS_X: %s", strerror(-rc)); goto error_free_evdev; }

    // Configure Y axis
    abs.maximum = (desc.idProduct == PRODUCT_ID_APPIV0906) ? APPIV_MAX_ABS_Y : AM_MAX_ABS_Y;
    // Keep other abs settings same as X (resolution, fuzz, flat)
    rc = libevdev_enable_event_code(evdev, EV_ABS, ABS_Y, &abs);
    if (rc < 0) { DEBUG("Failed to enable ABS_Y: %s", strerror(-rc)); goto error_free_evdev; }

    // Configure Pressure axis
    abs.maximum = AM_MAX_PRESSURE;
    abs.resolution = 0; // Resolution typically 0 for pressure/tilt
    abs.fuzz = 0;
    abs.flat = 0;
    rc = libevdev_enable_event_code(evdev, EV_ABS, ABS_PRESSURE, &abs);
    if (rc < 0) { DEBUG("Failed to enable ABS_PRESSURE: %s", strerror(-rc)); goto error_free_evdev; }

    // Configure Tilt X axis
    abs.maximum = AM_MAX_TILT_X; // Check if signed range needed (-64 to 63?)
    // abs.minimum = -64; // Example if signed
    rc = libevdev_enable_event_code(evdev, EV_ABS, ABS_TILT_X, &abs);
    if (rc < 0) { DEBUG("Failed to enable ABS_TILT_X: %s", strerror(-rc)); goto error_free_evdev; }

    // Configure Tilt Y axis
    abs.maximum = AM_MAX_TILT_Y; // Check if signed range needed (-64 to 63?)
    // abs.minimum = -64; // Example if signed
    rc = libevdev_enable_event_code(evdev, EV_ABS, ABS_TILT_Y, &abs);
    if (rc < 0) { DEBUG("Failed to enable ABS_TILT_Y: %s", strerror(-rc)); goto error_free_evdev; }

    // --- Enable relative events (Wheel) ---
    // Only enable if device likely has it (e.g., AM1107, AM1209)
    // Or enable generally and let events be ignored if device doesn't send them
    libevdev_enable_event_type(evdev, EV_REL);
    rc = libevdev_enable_event_code(evdev, EV_REL, REL_WHEEL, NULL);
    if (rc < 0) {
        DEBUG("Failed to enable REL_WHEEL: %s (Ignoring, might not be critical)", strerror(-rc));
        // Don't fail init, just log the warning
    }

    // --- Enable device-specific buttons ---
    switch(desc.idProduct) {
        // Models with 4 left buttons (BTN_0 to BTN_3)
        case PRODUCT_ID_AM3M:
        case PRODUCT_ID_AM0806:
        case PRODUCT_ID_AM0605:
        case PRODUCT_ID_GP0806:
        case PRODUCT_ID_GP0806B:
        case PRODUCT_ID_GP0605:
        case PRODUCT_ID_GP0605A:
        case PRODUCT_ID_GP0504:
        case PRODUCT_ID_NXS1513:
        case PRODUCT_ID_GP0906: // Also uses lbuttons mapping
            libevdev_enable_event_code(evdev, EV_KEY, BTN_0, NULL);
            libevdev_enable_event_code(evdev, EV_KEY, BTN_1, NULL);
            libevdev_enable_event_code(evdev, EV_KEY, BTN_2, NULL);
            libevdev_enable_event_code(evdev, EV_KEY, BTN_3, NULL);
            break;

        // Models with 8 buttons (4 left: BTN_0-3, 4 right: BTN_4-7)
        case PRODUCT_ID_AM1107:
        case PRODUCT_ID_AM1209:
            libevdev_enable_event_code(evdev, EV_KEY, BTN_0, NULL);
            libevdev_enable_event_code(evdev, EV_KEY, BTN_1, NULL);
            libevdev_enable_event_code(evdev, EV_KEY, BTN_2, NULL);
            libevdev_enable_event_code(evdev, EV_KEY, BTN_3, NULL);
            libevdev_enable_event_code(evdev, EV_KEY, BTN_4, NULL);
            libevdev_enable_event_code(evdev, EV_KEY, BTN_5, NULL);
            libevdev_enable_event_code(evdev, EV_KEY, BTN_6, NULL);
            libevdev_enable_event_code(evdev, EV_KEY, BTN_7, NULL);
            break;

        // APPIV0906 specific buttons (based on kernel driver)
        case PRODUCT_ID_APPIV0906:
            // Pen buttons already enabled (BTN_STYLUS, BTN_STYLUS2 if needed)
            // Kernel driver maps 0x04 to BTN_MIDDLE, 0x08 to BTN_0? Verify.
            libevdev_enable_event_code(evdev, EV_KEY, BTN_MIDDLE, NULL); // Third pen button?
            libevdev_enable_event_code(evdev, EV_KEY, BTN_0, NULL);      // Fourth pen button?

            // Tablet buttons (map to BTN_1 through BTN_7 based on kernel driver)
            libevdev_enable_event_code(evdev, EV_KEY, BTN_1, NULL); // Flag 0x01
            libevdev_enable_event_code(evdev, EV_KEY, BTN_2, NULL); // Flag 0x02
            libevdev_enable_event_code(evdev, EV_KEY, BTN_3, NULL); // Flag 0x04
            libevdev_enable_event_code(evdev, EV_KEY, BTN_4, NULL); // Flag 0x08
            libevdev_enable_event_code(evdev, EV_KEY, BTN_5, NULL); // Flag 0x10
            libevdev_enable_event_code(evdev, EV_KEY, BTN_6, NULL); // Flag 0x20
            libevdev_enable_event_code(evdev, EV_KEY, BTN_7, NULL); // Flag 0x40
            break;

        // Rollick models - check button layout if different
        case PRODUCT_ID_RL0604:
        case PRODUCT_ID_RL0504:
            // Assuming 4 buttons like GP series for now
            libevdev_enable_event_code(evdev, EV_KEY, BTN_0, NULL);
            libevdev_enable_event_code(evdev, EV_KEY, BTN_1, NULL);
            libevdev_enable_event_code(evdev, EV_KEY, BTN_2, NULL);
            libevdev_enable_event_code(evdev, EV_KEY, BTN_3, NULL);
            break;

        default:
            // No specific extra buttons known for this device
            break;
   }

    // --- Set specific device name ---
    switch(desc.idProduct) {
        // Cases are in ID order for readability
        case PRODUCT_ID_NXS1513:    libevdev_set_name(evdev, "Hanvon Nilox NXS1513"); break;
        case PRODUCT_ID_GP0504:     libevdev_set_name(evdev, "Hanvon Graphicpal 0504"); break;
        case PRODUCT_ID_GP0806:     libevdev_set_name(evdev, "Hanvon Graphicpal 0806"); break;
        case PRODUCT_ID_GP0605A:    libevdev_set_name(evdev, "Hanvon Graphicpal 0605A"); break;
        case PRODUCT_ID_AM1209:     libevdev_set_name(evdev, "Hanvon ArtMaster AM1209"); break;
        case PRODUCT_ID_AM0806:     libevdev_set_name(evdev, "Hanvon ArtMaster AM0806"); break;
        case PRODUCT_ID_AM0605:     libevdev_set_name(evdev, "Hanvon ArtMaster AM0605"); break;
        case PRODUCT_ID_AM1107:     libevdev_set_name(evdev, "Hanvon Art Master AM1107"); break; // Space intentional?
        case PRODUCT_ID_GP0806B:    libevdev_set_name(evdev, "Hanvon Graphicpal 0806B"); break;
        case PRODUCT_ID_GP0605:     libevdev_set_name(evdev, "Hanvon Graphicpal 0605"); break;
        case PRODUCT_ID_RL0504:     libevdev_set_name(evdev, "Hanvon Rollick 0504"); break;
        case PRODUCT_ID_RL0604:     libevdev_set_name(evdev, "Hanvon Rollick 0604"); break;
        case PRODUCT_ID_GP0906:     libevdev_set_name(evdev, "Hanvon Graphicpal 0906"); break;
        case PRODUCT_ID_AM3M:       libevdev_set_name(evdev, "Hanvon Art Master III"); break; // Space intentional?
        case PRODUCT_ID_APPIV0906:  libevdev_set_name(evdev, "Hanvon Art Painter Pro APPIV0906"); break;
        default: libevdev_set_name(evdev, "Hanvon Tablet (Unknown Model)"); break; // Fallback name
    }

    // --- Create the uinput device ---
    rc = libevdev_uinput_create_from_device(evdev, LIBEVDEV_UINPUT_OPEN_MANAGED, uidev_out);
    if (rc < 0) {
        DEBUG("Failed to create uinput device: %s (%d)", strerror(-rc), rc);
        goto error_free_evdev;
    }

    printf("Initialized controls for %04x:%04x, uinput node: %s\n",
           desc.idVendor, desc.idProduct, libevdev_uinput_get_devnode(*uidev_out));

    // Success
    // No need to free abs, it's on the stack
    return 0;

error_free_evdev:
    libevdev_free(evdev);
    *evdev_out = NULL; // Ensure caller sees NULL on error
    return rc; // Return the negative error code
}

// Signal handler for graceful shutdown
void sigterm_handler(int signum) {
    DEBUG("Received signal %d, initiating shutdown...", signum);
    g_running = 0; // Signal the main loop to exit
}

// Hotplug callback function
// NOTE: This callback and the global state management **DO NOT** correctly
//       handle multiple connected Hanvon devices. It assumes only one.
//       Refactoring is needed for multi-device support.
int hotplug_callback(struct libusb_context *ctx, struct libusb_device *dev,
                     libusb_hotplug_event event, void *user_data) {

    struct libusb_device_descriptor desc;
    int rc;

    rc = libusb_get_device_descriptor(dev, &desc);
    if (rc < 0) {
        DEBUG("Hotplug: Failed to get descriptor for event %d", event);
        return 0; // Ignore event if descriptor fails
    }

    DEBUG("Hotplug event: %s for device %04x:%04x",
           event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED ? "ARRIVED" : "LEFT",
           desc.idVendor, desc.idProduct);

    if (LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED == event) {
        // --- SINGLE DEVICE CHECK ---
        if (g_dev_handle != NULL) {
             DEBUG("INFO: Another Hanvon device is already active. Ignoring new device %04x:%04x.", desc.idVendor, desc.idProduct);
             return 0; // Ignore if already handling one
        }
        // --- END SINGLE DEVICE CHECK ---

        // Check if the device is one we specifically support via find_device logic
        // (find_device checks product ID list)
        libusb_device *devs_list[] = {dev};
        if (find_device(devs_list, 1) < 0) {
            // find_device already printed a message if vendor matched but product didn't
            // No need to print again unless vendor didn't match (which hotplug filter should prevent)
            return 0; // Not a supported product ID
        }

        DEBUG("Supported device %04x:%04x arrived. Attempting to open...", desc.idVendor, desc.idProduct);
        rc = libusb_open(dev, &g_dev_handle);
        if (rc != LIBUSB_SUCCESS) {
            DEBUG("Error opening device %04x:%04x: %s", desc.idVendor, desc.idProduct, libusb_error_name(rc));
            g_dev_handle = NULL; // Ensure handle is NULL on failure
            return 0; // Non-fatal, just couldn't open this one
        }

        // Detach kernel driver if active on interface 0
        rc = libusb_kernel_driver_active(g_dev_handle, 0);
        if (rc == 1) {
            DEBUG("Kernel driver active on interface 0. Detaching...");
            rc = libusb_detach_kernel_driver(g_dev_handle, 0);
            if (rc != LIBUSB_SUCCESS) {
                DEBUG("Error detaching kernel driver: %s. Closing device.", libusb_error_name(rc));
                libusb_close(g_dev_handle);
                g_dev_handle = NULL;
                return 0;
            }
        } else if (rc < 0 && rc != LIBUSB_ERROR_NOT_SUPPORTED) {
             DEBUG("Error checking kernel driver status: %s. Closing device.", libusb_error_name(rc));
             libusb_close(g_dev_handle);
             g_dev_handle = NULL;
             return 0;
        }


        // Claim interface 0 (usually the one with the interrupt endpoint)
        // Check lsusb -v output for bInterfaceNumber if unsure
        rc = libusb_claim_interface(g_dev_handle, 0);
        if (rc != LIBUSB_SUCCESS) {
             DEBUG("Error claiming interface 0: %s", libusb_error_name(rc));
             // Attempt to re-attach kernel driver before closing
             libusb_attach_kernel_driver(g_dev_handle, 0);
             libusb_close(g_dev_handle);
             g_dev_handle = NULL;
             return 0;
        }
        DEBUG("Interface 0 claimed successfully.");

        // Initialize evdev/uinput controls
        rc = init_ctrl(dev, &g_evdev, &g_uidev);
        if (rc < 0) {
            DEBUG("Error: Could not initialize controls for the device (%d).", rc);
            // Clean up interface claim and handle
            libusb_release_interface(g_dev_handle, 0);
            libusb_attach_kernel_driver(g_dev_handle, 0); // Reattach kernel driver
            libusb_close(g_dev_handle);
            g_dev_handle = NULL;
            // evdev might be partially created, uidev should be NULL if create failed
            if (g_evdev) {
                libevdev_free(g_evdev);
                g_evdev = NULL;
            }
            g_uidev = NULL; // Ensure uidev is NULL
            return 0; // Non-fatal
        }

        // Allocate the interrupt transfer
        g_tx = libusb_alloc_transfer(0);
        if (!g_tx) {
             DEBUG("Error allocating transfer");
             // Cleanup uinput/evdev, interface, handle
             libevdev_uinput_destroy(g_uidev); // Managed uinput also frees evdev
             g_uidev = NULL; g_evdev = NULL;
             libusb_release_interface(g_dev_handle, 0);
             libusb_attach_kernel_driver(g_dev_handle, 0);
             libusb_close(g_dev_handle);
             g_dev_handle = NULL;
             return 0;
        }

        // Find the interrupt IN endpoint address (usually 0x81)
        // TODO: Dynamically find the endpoint instead of hardcoding
        const int ENDPOINT_ADDR = 0x81; // bEndpointAddress from lsusb -v for Interface 0

        // Fill the interrupt transfer request
        libusb_fill_interrupt_transfer(
                g_tx,
                g_dev_handle,
                ENDPOINT_ADDR,
                g_buffer,           // Use global buffer
                AM_PACKET_LEN,      // Max packet length
                callback_default,   // The callback function
                g_uidev,            // Pass uidev as user_data to callback
                0                   // Timeout 0 = no timeout (recommended for interrupt)
        );

        // Submit the first transfer
        rc = libusb_submit_transfer(g_tx);
        if (rc != LIBUSB_SUCCESS) {
            DEBUG("Error submitting initial transfer: %s", libusb_error_name(rc));
            // Cleanup everything allocated so far
            libusb_free_transfer(g_tx); g_tx = NULL;
            libevdev_uinput_destroy(g_uidev); g_uidev = NULL; g_evdev = NULL;
            libusb_release_interface(g_dev_handle, 0);
            libusb_attach_kernel_driver(g_dev_handle, 0);
            libusb_close(g_dev_handle); g_dev_handle = NULL;
        } else {
            DEBUG("Device %04x:%04x initialized and first transfer submitted.", desc.idVendor, desc.idProduct);
        }

    } else if (LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT == event) {
        // --- SINGLE DEVICE CHECK ---
        // Check if the device leaving is the one we are currently handling.
        // This requires comparing the libusb_device* pointers.
        if (g_dev_handle != NULL && libusb_get_device(g_dev_handle) == dev) {
            DEBUG("Handling departure of active device %04x:%04x.", desc.idVendor, desc.idProduct);

            // 1. Cancel any pending transfers associated with this device handle
            if (g_tx) {
                DEBUG("Cancelling active transfer...");
                rc = libusb_cancel_transfer(g_tx);
                if (rc != LIBUSB_SUCCESS && rc != LIBUSB_ERROR_NOT_FOUND) {
                    DEBUG("Error cancelling transfer: %s", libusb_error_name(rc));
                }
                // The transfer callback will eventually run with status CANCELLED.
                // We still need to free the transfer structure itself.
                libusb_free_transfer(g_tx);
                g_tx = NULL;
            }

            // 2. Destroy the uinput device (this also frees the associated evdev device)
            if (g_uidev) {
                DEBUG("Destroying uinput device...");
                libevdev_uinput_destroy(g_uidev);
                g_uidev = NULL;
                g_evdev = NULL; // evdev is freed by uinput destroy
            }

            // 3. Release the interface
            DEBUG("Releasing interface 0...");
            rc = libusb_release_interface(g_dev_handle, 0);
             if (rc != LIBUSB_SUCCESS && rc != LIBUSB_ERROR_NO_DEVICE) {
                 DEBUG("Error releasing interface: %s", libusb_error_name(rc));
             }

            // 4. Re-attach kernel driver (best effort)
            DEBUG("Attempting to re-attach kernel driver...");
            rc = libusb_attach_kernel_driver(g_dev_handle, 0);
            if (rc != LIBUSB_SUCCESS && rc != LIBUSB_ERROR_NO_DEVICE && rc != LIBUSB_ERROR_NOT_SUPPORTED && rc != LIBUSB_ERROR_BUSY) {
                 DEBUG("Error re-attaching kernel driver: %s", libusb_error_name(rc));
            }

            // 5. Close the device handle
            DEBUG("Closing device handle...");
            libusb_close(g_dev_handle);
            g_dev_handle = NULL;

            DEBUG("Device %04x:%04x cleanup complete.", desc.idVendor, desc.idProduct);

        } else {
             DEBUG ("INFO: Device %04x:%04x left, but it wasn't the active device or no device was active.", desc.idVendor, desc.idProduct);
        }
        // --- END SINGLE DEVICE CHECK ---

    } else {
        DEBUG ("Unhandled hotplug event: %d", event);
    }
    return 0; // Return 0 to continue receiving hotplug events
}


int main() {
    int rc;

    // Initialize libusb
    rc = libusb_init(NULL);
    if (rc < 0) {
        fprintf(stderr, "Failed to initialize libusb: %s\n", libusb_error_name(rc));
        return EXIT_FAILURE;
    }
    DEBUG("libusb initialized.");

    // Setup signal handler for graceful shutdown (SIGINT, SIGTERM)
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = sigterm_handler;
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    DEBUG("Signal handlers registered.");

    // Check for hotplug capability
    if (!libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
        fprintf(stderr, "Error: libusb hotplug not supported on this system.\n");
        // Optional: Implement manual device scanning loop here if hotplug isn't available
        libusb_exit(NULL);
        return EXIT_FAILURE;
    }
    DEBUG("libusb hotplug capability detected.");

    // Register hotplug callback
    libusb_hotplug_callback_handle callback_handle;
    rc = libusb_hotplug_register_callback(
        NULL, // Context (NULL for default)
        LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED |
        LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
        LIBUSB_HOTPLUG_ENUMERATE, // Enumerate existing devices on startup
        VENDOR_ID_HANVON,         // Filter by Vendor ID
        LIBUSB_HOTPLUG_MATCH_ANY, // Match any Product ID (find_device will filter further)
        LIBUSB_HOTPLUG_MATCH_ANY, // Match any Device Class
        hotplug_callback,         // The callback function
        NULL,                     // User data (could pass device list pointer here for multi-device)
        &callback_handle          // Handle for deregistration
    );
    if (rc != LIBUSB_SUCCESS) {
        fprintf(stderr, "Error registering hotplug callback: %s\n", libusb_error_name(rc));
        libusb_exit(NULL);
        return EXIT_FAILURE;
    }
    DEBUG("Hotplug callback registered. Waiting for events...");

    // Event handling loop
    while (g_running) {
        // Use blocking wait with timeout for events
        // Timeout allows checking g_running flag periodically
        struct timeval tv = {1, 0}; // 1 second timeout
        rc = libusb_handle_events_timeout_completed(NULL, &tv, NULL);
        if (rc < 0) {
            if (rc == LIBUSB_ERROR_INTERRUPTED) {
                // Interrupted by signal (likely SIGINT/SIGTERM), loop condition will handle exit
                DEBUG("libusb event handling interrupted.");
                continue;
            }
            // Other errors
            fprintf(stderr, "Error during libusb event handling: %s\n", libusb_error_name(rc));
            // Decide if this is fatal or recoverable
            // If fatal, maybe set g_running = 0;
        }
        // Loop continues if g_running is still 1
    }

    DEBUG("Exiting event loop.");

    // --- Cleanup before exiting ---

    // Deregister hotplug callback
    libusb_hotplug_deregister_callback(NULL, callback_handle);
    DEBUG("Hotplug callback deregistered.");

    // Final cleanup for the active device if the loop was terminated
    // while a device was still active (similar logic as DEVICE_LEFT event)
    if (g_dev_handle != NULL) {
         DEBUG("Cleaning up active device before exit...");
         if (g_tx) {
             libusb_cancel_transfer(g_tx);
             // Note: Might need a short wait or event handle loop here
             // to ensure cancellation completes before freeing.
             // However, libusb_exit should handle pending cancellations.
             libusb_free_transfer(g_tx); g_tx = NULL;
         }
         if (g_uidev) {
             libevdev_uinput_destroy(g_uidev); g_uidev = NULL; g_evdev = NULL;
         }
         // Interface release and kernel driver attach are best-effort on exit
         libusb_release_interface(g_dev_handle, 0);
         libusb_attach_kernel_driver(g_dev_handle, 0);
         libusb_close(g_dev_handle); g_dev_handle = NULL;
         DEBUG("Active device cleanup complete.");
    }

    // Exit libusb
    libusb_exit(NULL);
    DEBUG("libusb exited.");

    printf("Hanvon userspace driver finished.\n");
    return EXIT_SUCCESS;
}
