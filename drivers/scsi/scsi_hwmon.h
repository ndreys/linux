/* SPDX-License-Identifier: GPL-2.0 */
#include <scsi/scsi_device.h>

#ifdef CONFIG_SCSI_HWMON

int scsi_hwmon_probe(struct scsi_device *sdev);

#else

static inline int scsi_hwmon_probe(struct scsi_device *sdev)
{
	return 0;
}

#endif
