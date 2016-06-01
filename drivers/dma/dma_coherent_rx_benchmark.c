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
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/io.h>
#include <asm/page.h>

#define MAX_BUFS 1024

struct device dev;
static void *bufs[MAX_BUFS];
dma_addr_t dma_handles[MAX_BUFS];

static struct dma_pool *pool;

static char discard[1500];

static int alloc_ring(struct device *dev, int num_bufs)
{
	int i;

	if (num_bufs > MAX_BUFS)
		return -EINVAL;

	pool = dma_pool_create("RX Pool", dev, 4, 2048, 0);
	if (!pool)
		return -ENOMEM;

	for (i = 0; i < num_bufs; i++) {
		bufs[i] = dma_pool_alloc(pool, GFP_KERNEL, &dma_handles[i]);
		if (!bufs[i])
			return -ENOMEM;
	}
	return 0;
}

static void free_ring(struct device *dev, int num_bufs)
{
	int i;

	for (i = 0; i < num_bufs; i++)
		dma_pool_free(pool, bufs[i], dma_handles[i]);

	dma_pool_destroy(pool);
}

static void run_ring_once(struct device *dev, int num_bufs)
{
	int i;

	for (i = 0; i < num_bufs; i++)
		memcpy(discard, bufs[i], 2048);
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
		count += num_bufs;
		now = jiffies;
		elapsed = now - start;
	} while (elapsed < (HZ * 2));

	dev_info(dev, "%3d: %d\n", num_bufs, count);
}

static __exit void rx_benchmark_release(struct device *dev)
{
}

static int __init rx_coherent_benchmark_init(void)
{
	int err;
	int i;

	memset(&dev, 0, sizeof(dev));

	dev_set_name(&dev, "rx_benchmark");
	dev.release = rx_benchmark_release;
	dev.coherent_dma_mask = DMA_BIT_MASK(32);

	err = device_register(&dev);
	if (err)
		return err;

	for (i = 4; i <= 128; i += 2) {
		err = alloc_ring(&dev, i);
		if (err)
			goto out_unregister;

		run_ring(&dev, i);
		free_ring(&dev, i);
		schedule();
	}

out_unregister:
	return 0;
}

static void __exit rx_coherent_benchmark_exit(void)
{
	device_unregister(&dev);
}

module_init(rx_coherent_benchmark_init);
module_exit(rx_coherent_benchmark_exit);
MODULE_AUTHOR("Andrew Lunn (andrew@lunn.ch)");
MODULE_LICENSE("GPL v2");
