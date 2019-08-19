
#include <linux/debugfs.h>

#include "global2.h"

static int mv88e6xx_index_reg_get(void *data, u64 *val)
{
	struct mv88e6xxx_chip *chip = data;

	*val = chip->index_reg;
	return 0;
}

static int mv88e6xx_index_reg_set(void *data, u64 val)
{
	struct mv88e6xxx_chip *chip = data;
	int ret;

	pr_crit("%s %lx\n", __func__, (unsigned long)chip);

	mv88e6xxx_reg_lock(chip);
	ret = mv88e6xxx_g2_write(chip, MV88E6XXX_G2_IMP_DEBUG, val);
	mv88e6xxx_reg_unlock(chip);
	if (ret)
		return ret;

	if (val & MV88E6XXX_G2_IMP_DEBUG_UPDATE)
		return 0;

	mv88e6xxx_reg_lock(chip);
	ret = mv88e6xxx_g2_read(chip, MV88E6XXX_G2_IMP_DEBUG,
				&chip->index_reg);
	mv88e6xxx_reg_unlock(chip);

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(mv88e6xx_index_reg_fops,
			 mv88e6xx_index_reg_get,
			 mv88e6xx_index_reg_set,
			 "%llu\n");

void mv88e6xxx_register_imp_debugfs(struct mv88e6xxx_chip *chip)
{

	pr_crit("%s %lx\n", __func__, (unsigned long)chip);

	chip->debugfs_dir = debugfs_create_dir("switchy", NULL);

	if (IS_ERR(chip->debugfs_dir)) {
		dev_warn(chip->dev, "Failed to create IMP debugfs dir\n");
		return;
	}

	debugfs_create_file_unsafe("index_reg", 0666, chip->debugfs_dir,
				   chip, &mv88e6xx_index_reg_fops);
}
