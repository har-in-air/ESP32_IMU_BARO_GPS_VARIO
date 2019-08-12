#ifndef BTSPP_H_
#define BTSPP_H_

#ifdef __cplusplus
extern "C" {
#endif

bool btspp_init(const char *deviceName);
size_t btspp_writeBuf(const uint8_t *buffer, size_t size);
size_t btspp_print(const char *buffer, size_t size);

#ifdef __cplusplus
}
#endif

#endif

