/* DMA API benchmark
 * Copyright(c) 2016 Andrew Lunn <andrew@lunn.ch>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <asm/io.h>
#include <asm/page.h>

#define MAX_PAGES 1024

struct device dev;

static struct page *pages[MAX_PAGES];
static dma_addr_t dmas[MAX_PAGES];
static dma_addr_t buffs[MAX_PAGES * 2];

static char discard[1500];

/* Allocate a page per two buffers, and add the pointers to the ring structure */
static int alloc_ring(struct device *dev, int num_bufs)
{
	int err;
	int i;

	if (num_bufs & 1)
		return -EINVAL;

	if (num_bufs > MAX_PAGES * 2)
		return -EINVAL;

	for (i = 0; i < num_bufs/2; i++) {
		pages[i] = dev_alloc_page();
		if (!pages[i])
			return -ENOMEM;

		dmas[i] = dma_map_page(dev, pages[i], 0, PAGE_SIZE, DMA_FROM_DEVICE);
		err = dma_mapping_error(dev, dmas[i]);
		if (err)
			return err;

		buffs[i * 2] = dmas[i];
		buffs[i * 2 + 1] = dmas[i] + 2048;
	}
	return 0;
}

static void free_ring(struct device *dev, int num_bufs)
{
	int i;

	for (i = 0; i < num_bufs/2; i++) {
		dma_unmap_page(dev, dmas[i], PAGE_SIZE, DMA_FROM_DEVICE);
		__free_page(pages[i]);
	}
}

/* Give all but the first buffer to the device */
static void prime_ring(struct device *dev, int num_bufs)
{
	int i;

	dma_sync_single_range_for_device(dev, dmas[0], 2048, 2048,
					 DMA_FROM_DEVICE);

	for (i = 1; i < num_bufs/2; i++) {
		dma_sync_single_range_for_device(dev, dmas[i], 0,
						 2048, DMA_FROM_DEVICE);

		dma_sync_single_range_for_device(dev, dmas[i], 2048,
						 2048, DMA_FROM_DEVICE);
	}
}

static void give_to_device(struct device *dev, int num_buf)
{
	dma_sync_single_range_for_device(dev, dmas[num_buf / 2],
					 2048 * (num_buf & 1), 2048,
					 DMA_FROM_DEVICE);
}

static void take_from_device(struct device *dev, int num_buf)
{

	dma_sync_single_range_for_cpu(dev, dmas[num_buf / 2],
				      2048 * (num_buf & 1), 2048,
				      DMA_FROM_DEVICE);

	memcpy(discard, (void *)buffs[num_buf], 1500);
}

static void run_ring_once(struct device *dev, int num_bufs)
{

	int i;

	for (i = 0; i < num_bufs - 1; i++) {
		give_to_device(dev, i);
		take_from_device(dev, i + 1);
	}
}

static void run_ring(struct device *dev, int num_bufs)
{
	unsigned long elapsed;
	unsigned long start;
	unsigned long now;
	int count = 0;

	start = jiffies;

	do {
		run_ring_once(dev, num_bufs);
		count += (num_bufs - 1);
		now = jiffies;
		elapsed = now - start;
	} while (elapsed < (HZ * 2));

	dev_info(dev, "%3d: %d\n", num_bufs, count);
}

static __exit void rx_benchmark_release(struct device *dev)
{
}

static int __init rx_benchmark_init(void)
{
	int err;
	int i;

	memset(&dev, 0, sizeof(dev));

	dev_set_name(&dev, "rx_benchmark");
	dev.release = rx_benchmark_release;

	err = device_register(&dev);
	if (err)
		return err;

	for (i = 4; i <= 128; i += 2) {
		err = alloc_ring(&dev, i);
		if (err)
			goto out_unregister;

		prime_ring(&dev, i);
		run_ring(&dev, i);
		free_ring(&dev, i);
		schedule();
	}

out_unregister:
	return 0;
}

static void __exit rx_benchmark_exit(void)
{
	device_unregister(&dev);
}

module_init(rx_benchmark_init);
module_exit(rx_benchmark_exit);
MODULE_AUTHOR("Andrew Lunn (andrew@lunn.chl)");
MODULE_LICENSE("GPL v2");
