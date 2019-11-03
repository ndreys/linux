/* SPDX-License-Identifier: GPL-2.0 */
/*
 * CAAM public-level include definitions for the JobR backend
 *
 * Copyright 2008-2011 Freescale Semiconductor, Inc.
 */

#ifndef JR_H
#define JR_H

/* Prototypes for backend-level services exposed to APIs */
typedef void (*caam_jr_cbk)(struct device *dev, u32 *desc, u32 status,
			    void *areq);

struct device *caam_jr_alloc(void);
void caam_jr_free(struct device *rdev);
int caam_jr_enqueue(struct device *dev, u32 *desc, caam_jr_cbk cbk,
		    void *areq);

#endif /* JR_H */
