// Mode Manager: top-level state machine (header stub)
#ifndef MODE_H
#define MODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MODE_INIT = 0,
    MODE_IDLE,
    MODE_ENABLED,
    MODE_FAULT,
} ModeState;

void      Mode_Init(void);
void      Mode_Update(uint32_t now_ms);
void      Mode_RequestEnable(uint8_t enable);
void      Mode_RequestClearFault(void);
ModeState Mode_GetState(void);

#ifdef __cplusplus
}
#endif

#endif // MODE_H

