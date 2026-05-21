#ifndef ESP32_SERIAL_TRANSPORT_H
#define ESP32_SERIAL_TRANSPORT_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <uxr/client/transport.h>

#ifdef __cplusplus
extern "C" {
#endif

bool esp32_serial_open(struct uxrCustomTransport *transport);
bool esp32_serial_close(struct uxrCustomTransport *transport);
size_t esp32_serial_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t esp32_serial_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

#ifdef __cplusplus
}
#endif

#endif
