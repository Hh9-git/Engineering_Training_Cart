#ifndef BSP_KEY_H
#define BSP_KEY_H

#include "main.h"

#define BUTTON_NOT_PRESSED      0
#define BUTTON_PRESSED          1

typedef enum
{
    NO_DETECT,
    DETECTING,
    DETECTED
}KeyState;

int8_t BSP_UserKey_Detect(void);

#endif /* BSP_KEY_H */