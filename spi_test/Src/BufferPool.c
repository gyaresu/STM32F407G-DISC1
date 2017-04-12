#include "BufferPool.h"

CCBuffer *cc_buffer_init(CCBuffer **start_pos, uint8_t start_index)
{
	*start_pos = malloc(sizeof(CCBuffer));
	(*start_pos)->status = FREE;
	
	if(start_index != BUFFER_POOL_SIZE)
	{
		return cc_buffer_init( &((*start_pos)->next), start_index+1);
	}
	return *start_pos;
}
