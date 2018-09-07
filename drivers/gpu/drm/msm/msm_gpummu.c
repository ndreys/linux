/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "msm_drv.h"
#include "msm_mmu.h"

struct msm_gpummu {
	struct msm_mmu base;
	struct msm_gpu *gpu;
	dma_addr_t pt_base;
	uint32_t *table;
};
#define to_msm_gpummu(x) container_of(x, struct msm_gpummu, base)

#define VA_START SZ_16M
#define VA_RANGE (0xfff * SZ_64K)
#define MMU_PAGE_SIZE SZ_4K
#define TABLE_SIZE (sizeof(uint32_t) * VA_RANGE / MMU_PAGE_SIZE)

static int msm_gpummu_attach(struct msm_mmu *mmu, const char * const *names,
		int cnt)
{
	return 0;
}

static void msm_gpummu_detach(struct msm_mmu *mmu, const char * const *names,
		int cnt)
{
}

static void update_pt(struct msm_mmu *mmu, uint64_t iova,
		struct sg_table *sgt, unsigned len, unsigned prot)
{
	struct msm_gpummu *gpummu = to_msm_gpummu(mmu);
	unsigned idx = (iova - VA_START) / MMU_PAGE_SIZE;
	struct scatterlist *sg;
	unsigned i, j;

	for_each_sg(sgt->sgl, sg, sgt->nents, i) {
		dma_addr_t addr = sg->dma_address;
		for (j = 0; j < sg->length / MMU_PAGE_SIZE; j++, idx++) {
			gpummu->table[idx] = prot ? addr | prot : 0;
			addr += MMU_PAGE_SIZE;
		}
	}
}

static int msm_gpummu_map(struct msm_mmu *mmu, uint64_t iova,
		struct sg_table *sgt, unsigned len, int prot)
{
	unsigned p = 0;
	if (prot & IOMMU_WRITE)
		p |= 1;
	if (prot & IOMMU_READ)
		p |= 2;
	update_pt(mmu, iova, sgt, len, p);
	return 0;
}

static int msm_gpummu_unmap(struct msm_mmu *mmu, uint64_t iova,
		struct sg_table *sgt, unsigned len)
{
	update_pt(mmu, iova, sgt, len, 0);
	return 0;
}

static void msm_gpummu_destroy(struct msm_mmu *mmu)
{
	struct msm_gpummu *gpummu = to_msm_gpummu(mmu);

	dma_free_attrs(mmu->dev, TABLE_SIZE, gpummu->table, gpummu->pt_base,
		DMA_ATTR_FORCE_CONTIGUOUS);

	kfree(gpummu);
}

static const struct msm_mmu_funcs funcs = {
		.attach = msm_gpummu_attach,
		.detach = msm_gpummu_detach,
		.map = msm_gpummu_map,
		.unmap = msm_gpummu_unmap,
		.destroy = msm_gpummu_destroy,
};

struct msm_mmu *msm_gpummu_new(struct device *dev, struct msm_gpu *gpu)
{
	struct msm_gpummu *gpummu;

	gpummu = kzalloc(sizeof(*gpummu), GFP_KERNEL);
	if (!gpummu)
		return ERR_PTR(-ENOMEM);

	gpummu->table = dma_alloc_attrs(dev, TABLE_SIZE + 32, &gpummu->pt_base,
		GFP_KERNEL | __GFP_ZERO, DMA_ATTR_FORCE_CONTIGUOUS);
	if (!gpummu->table) {
		kfree(gpummu);
		return ERR_PTR(-ENOMEM);
	}

	gpummu->gpu = gpu;
	msm_mmu_init(&gpummu->base, dev, &funcs);

	return &gpummu->base;
}

void msm_gpummu_params(struct msm_mmu *mmu, dma_addr_t *pt_base,
		dma_addr_t *tran_error)
{
	dma_addr_t base = to_msm_gpummu(mmu)->pt_base;

	*pt_base = base;
	*tran_error = base + TABLE_SIZE; /* 32-byte aligned */
}
