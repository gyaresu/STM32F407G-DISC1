#ifndef _BUFFER_POOL_
#define _BUFFER_POOL_

#include "stdint.h"
#include "stdlib.h"
#include "model.h"

#define BUFFER_POOL_SIZE		10

typedef enum _status_ {BUSY = 0x00, FREE = 0xFF} STATUS;

typedef struct _car_control_buffer_ {
	Car payload;
	struct _car_control_buffer_ *next;
	STATUS status;
} CCBuffer;

CCBuffer *cc_buffer_init(CCBuffer **start_pos, uint8_t start_index);

#endif
