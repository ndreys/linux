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
#include <linux/genalloc.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <asm/page.h>

#define MAX_BUFS 1024

struct device dev;
static void *bufs[MAX_BUFS];
dma_addr_t dma_handles[MAX_BUFS];

static struct gen_pool *pool;

static char discard[1500];

static int alloc_ring(struct device *dev, int num_bufs)
{
	struct platform_device *pdev;
	struct device_node *node;
	int ret;
	int i;

	if (num_bufs > MAX_BUFS)
		return -EINVAL;

	node = of_find_compatible_node(NULL, NULL, "mmio-sram");
	if (!node) {
		dev_warn(dev, "failed to find sram node\n");
		return -ENODEV;
	}

	pdev = of_find_device_by_node(node);
	if (!pdev) {
		dev_warn(dev, "failed to find sram device\n");
		ret = -ENODEV;
		goto put_node;
	}

	pool = gen_pool_get(&pdev->dev, NULL);
	if (!pool) {
		dev_warn(dev, "ocram pool unavailable\n");
		ret = -ENODEV;
		goto put_node;
	}

	for (i = 0; i < num_bufs; i++) {
		bufs[i] = gen_pool_dma_alloc(pool, 2048, &dma_handles[i]);
		dev_info(dev, "%p %x\n", bufs[i], dma_handles[i]);
		if (!bufs[i]) {
			dev_warn(dev, "Out of pool memory\n");
			ret = -ENOMEM;
			goto put_node;
		}
	}
	return 0;

put_node:
	of_node_put(node);

	return ret;
}

static void free_ring(struct device *dev, int num_bufs)
{
	int i;

	for (i = 0; i < num_bufs; i++)
		gen_pool_free(pool, (unsigned long)bufs[i], 2048);
}

/* Give all but the first buffer to the device */
static void prime_ring_sync(struct device *dev, int num_bufs)
{
	int i;

	for (i = 1; i < num_bufs - 1; i++)
		dma_sync_single_range_for_device(dev, dma_handles[i], 0,
						 2048, DMA_FROM_DEVICE);
}

static void give_to_device(struct device *dev, int num_buf)
{
	dma_sync_single_range_for_device(dev, dma_handles[num_buf],
					 0, 2048, DMA_FROM_DEVICE);
}

static void take_from_device(struct device *dev, int num_buf)
{

	dma_sync_single_range_for_cpu(dev, dma_handles[num_buf],
				      0, 2048, DMA_FROM_DEVICE);

	memcpy(discard, (void *)bufs[num_buf], 1500);
}


static void run_ring_once(struct device *dev, int num_bufs)
{
	int i;

	for (i = 0; i < num_bufs - 1; i++)
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
		count += (num_bufs - 1);
		now = jiffies;
		elapsed = now - start;
	} while (elapsed < (HZ * 2));

	dev_info(dev, "%3d: %d\n", num_bufs, count);
}

static void run_ring_sync_once(struct device *dev, int num_bufs)
{
	int i;

	for (i = 0; i < num_bufs - 1; i++) {
		give_to_device(dev, i);
		take_from_device(dev, i + 1);
	}
}

static void run_ring_sync(struct device *dev, int num_bufs)
{
	unsigned long elapsed;
	unsigned long start;
	unsigned long now;
	int count = 0;

	start = jiffies;

	do {
		run_ring_sync_once(dev, num_bufs);
		count += num_bufs;
		now = jiffies;
		elapsed = now - start;
	} while (elapsed < (HZ * 2));

	dev_info(dev, "sync: %3d: %d\n", num_bufs, count);
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

	for (i = 4; i <= 16; i += 2) {
		err = alloc_ring(&dev, i);
		if (err)
			goto out_unregister;

		run_ring(&dev, i);
		free_ring(&dev, i);
		schedule();
	}

	for (i = 4; i <= 16; i += 2) {
		err = alloc_ring(&dev, i);
		if (err)
			goto out_unregister;

		prime_ring_sync(&dev, i);
		run_ring_sync(&dev, i);
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
