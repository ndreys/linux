
#include <linux/bitfield.h>
#include <linux/debugfs.h>

#include "global2.h"

static int
mv88e6xxx_index_reg_io(struct mv88e6xxx_chip *chip, u8 offset, u8 *data)
{
	u16 debug;
	int ret;

	debug = FIELD_PREP(MV88E6XXX_G2_IMP_DEBUG_POINTER, offset);

	mv88e6xxx_reg_lock(chip);

	if (data) {
		debug |= MV88E6XXX_G2_IMP_DEBUG_UPDATE;
		debug |= FIELD_PREP(MV88E6XXX_G2_IMP_DEBUG_DATA, *data);
	}

	ret = mv88e6xxx_g2_write(chip, MV88E6XXX_G2_IMP_DEBUG, debug);
	if (ret || data)
		goto unlock;

	ret = mv88e6xxx_g2_read(chip, MV88E6XXX_G2_IMP_DEBUG, &debug);
	if (ret)
		goto unlock;

	ret = debug;

unlock:
	mv88e6xxx_reg_unlock(chip);

	return ret;
}

static int
mv88e6xxx_imp_read_index_reg(struct mv88e6xxx_chip *chip, u8 offset, u8 *value)
{
	int ret = mv88e6xxx_index_reg_io(chip, offset, NULL);

	if (ret < 0)
		return ret;

	*value = (u8)ret;

	return 0;
}

 static int
mv88e6xxx_imp_write_index_reg(struct mv88e6xxx_chip *chip, u8 offset, u8 value)
{
	return mv88e6xxx_index_reg_io(chip, offset, &value);
}

static ssize_t mv88e6xxx_imp_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct mv88e6xxx_chip *chip = file->private_data;
	loff_t avail = file_inode(file)->i_size;
	loff_t pos = *ppos;
	loff_t i;

	if (pos < 0)
		return -EINVAL;
	if (pos >= avail)
		return 0;

	count = min_t(size_t, count, avail - pos);

	for (i = 0; i < count; i++) {
		int ret;
		u8 data;

		ret = mv88e6xxx_imp_read_index_reg(chip, pos + i, &data);
		if (ret)
			return ret;

		if (copy_to_user(buf, &data, sizeof(data)))
			return -EFAULT;
	}

	*ppos = pos + i;
	return count;
}

static const struct file_operations mv88e6xxx_imp_debugfs_fops = {
	.owner   = THIS_MODULE,
	.open    = simple_open,
	.read    = mv88e6xxx_imp_read,
	.llseek  = default_llseek,
};

void mv88e6xxx_register_imp_debugfs(struct mv88e6xxx_chip *chip)
{
	chip->debugfs_dir = debugfs_create_dir("switchy", NULL);

	if (IS_ERR(chip->debugfs_dir)) {
		dev_warn(chip->dev, "Failed to create IMP debugfs dir\n");
		return;
	}

	debugfs_create_file_size("debug", 0664, chip->debugfs_dir, chip,
				 &mv88e6xxx_imp_debugfs_fops, 128);
}
