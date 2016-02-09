/*
 * Copyright 2016 Pengutronix
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL TUNGSTEN GRAPHICS AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef _UAPI_IMX_DRM_H_
#define _UAPI_IMX_DRM_H_

#include "drm.h"

struct drm_imx_gem_new {
	__u64 size;           /* in */
	__u32 flags;          /* in, dummy for now, no valid values */
	__u32 handle;         /* out */
};

#define DRM_IMX_GEM_NEW	0x00

#define DRM_IOCTL_IMX_GEM_NEW      DRM_IOWR(DRM_COMMAND_BASE + DRM_IMX_GEM_NEW, struct drm_imx_gem_new)

#endif /* _UAPI_IMX_DRM_H_ */
