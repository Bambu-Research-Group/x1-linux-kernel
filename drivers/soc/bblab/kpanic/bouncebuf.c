/*
 * Generic bounce buffer implementation
 *
 * Copyright (C) 2012 Marek Vasut <marex@denx.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>


#include "common.h"
#include "bouncebuf.h"
#include "memalign.h"


#if 1
// copied from arch/arm/mm/dma.h

#define dmac_map_area			__glue(_CACHE,_dma_map_area)
#define dmac_unmap_area 		__glue(_CACHE,_dma_unmap_area)

/*
 * These are private to the dma-mapping API.  Do not use directly.
 * Their sole purpose is to ensure that data held in the cache
 * is visible to DMA, or data written by DMA to system memory is
 * visible to the CPU.
 */
extern void dmac_map_area(const void *, size_t, int);
extern void dmac_unmap_area(const void *, size_t, int);

#endif


static int addr_aligned(struct bounce_buffer *state)
{
	const ulong align_mask = ARCH_DMA_MINALIGN - 1;

	/* Check if start is aligned */
	if ((ulong)state->user_buffer & align_mask) {
		debug("Unaligned buffer address %p\n", state->user_buffer);
		return 0;
	}

	/* Check if length is aligned */
	if (state->len != state->len_aligned) {
		debug("Unaligned buffer length %zu\n", state->len);
		return 0;
	}

	/* Aligned */
	return 1;
}

int bounce_buffer_start(struct bounce_buffer *state, void *data,
			size_t len, unsigned int flags)
{
	state->user_buffer = data;
	state->bounce_buffer = data;
	state->len = len;
	state->len_aligned = roundup(len, ARCH_DMA_MINALIGN);
	state->flags = flags;

	if (!addr_aligned(state)) {
		state->bounce_buffer = memalign(ARCH_DMA_MINALIGN,
						state->len_aligned);
		if (!state->bounce_buffer)
			return -ENOMEM;

		if (state->flags & GEN_BB_READ)
			memcpy(state->bounce_buffer, state->user_buffer,
				state->len);
	}

	/*
	 * Flush data to RAM so DMA reads can pick it up,
	 * and any CPU writebacks don't race with DMA writes
	 */
#if 0
	flush_dcache_range((unsigned long)state->bounce_buffer,
				(unsigned long)(state->bounce_buffer) +
					state->len_aligned);
#endif
	//__cpuc_flush_dcache_area((void *)state->bounce_buffer, state->len_aligned);
	dmac_flush_range((void *)state->bounce_buffer, (void *)(state->bounce_buffer) + state->len_aligned);

	return 0;
}

int bounce_buffer_stop(struct bounce_buffer *state)
{
	if (state->flags & GEN_BB_WRITE) {
		/* Invalidate cache so that CPU can see any newly DMA'd data */
#if 0
		invalidate_dcache_range((unsigned long)state->bounce_buffer,
					(unsigned long)(state->bounce_buffer) +
						state->len_aligned);
#endif
		dmac_map_area((void *)state->bounce_buffer, state->len_aligned, DMA_FROM_DEVICE);
	}

	if (state->bounce_buffer == state->user_buffer)
		return 0;

	if (state->flags & GEN_BB_WRITE)
		memcpy(state->user_buffer, state->bounce_buffer, state->len);

	free(state->bounce_buffer);

	return 0;
}
