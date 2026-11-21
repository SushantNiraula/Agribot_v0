#ifndef PICO_USB_TRANSPORT_H
#define PICO_USB_TRANSPORT_H

#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include <uxr/client/transport.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- The missing clock function ROS 2 needs for message timestamps ---
int clock_gettime(clockid_t unused, struct timespec *tp) {
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

bool pico_usb_transport_open(struct uxrCustomTransport * transport) { return true; }
bool pico_usb_transport_close(struct uxrCustomTransport * transport) { return true; }

size_t pico_usb_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err) {
    for (size_t i = 0; i < len; ++i) {
        putchar_raw(buf[i]);
    }
    return len;
}

size_t pico_usb_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
    uint64_t start_time_us = time_us_64();
    size_t bytes_read = 0;
    while(bytes_read < len && (time_us_64() - start_time_us) < (timeout * 1000)) {
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            buf[bytes_read++] = (uint8_t)c;
        }
    }
    return bytes_read;
}

#ifdef __cplusplus
}
#endif

#endif // PICO_USB_TRANSPORT_H
