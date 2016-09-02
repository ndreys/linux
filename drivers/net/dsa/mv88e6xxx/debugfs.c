#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int _mv88e6xxx_pvt_wait(struct mv88e6xxx_chip *chip)
{
	return mv88e6xxx_wait(chip, REG_GLOBAL2, GLOBAL2_PVT_ADDR,
			      GLOBAL2_PVT_ADDR_BUSY);
}

static int _mv88e6xxx_pvt_cmd(struct mv88e6xxx_chip *chip, int src_dev,
			      int src_port, u16 op)
{
	u16 reg = op;
	int err;

	/* 9-bit Cross-chip PVT pointer: with GLOBAL2_MISC_5_BIT_PORT cleared,
	 * source device is 5-bit, source port is 4-bit.
	 */
	reg |= (src_dev & 0x1f) << 4;
	reg |= (src_port & 0xf);

	err = _mv88e6xxx_reg_write(chip, REG_GLOBAL2, GLOBAL2_PVT_ADDR, reg);
	if (err)
		return err;

	return _mv88e6xxx_pvt_wait(chip);
}

static int _mv88e6xxx_pvt_read(struct mv88e6xxx_chip *chip, int src_dev,
			       int src_port, u16 *data)
{
	int ret;

	ret = _mv88e6xxx_pvt_wait(chip);
	if (ret < 0)
		return ret;

	ret = _mv88e6xxx_pvt_cmd(chip, src_dev, src_port,
				 GLOBAL2_PVT_ADDR_OP_READ);
	if (ret < 0)
		return ret;

	ret = _mv88e6xxx_reg_read(chip, REG_GLOBAL2, GLOBAL2_PVT_DATA);
	if (ret < 0)
		return ret;

	*data = ret;

	return 0;
}

static int _mv88e6xxx_pvt_write(struct mv88e6xxx_chip *chip, int src_dev,
				int src_port, u16 data)
{
	int err;

	err = _mv88e6xxx_pvt_wait(chip);
	if (err)
		return err;

	err = _mv88e6xxx_reg_write(chip, REG_GLOBAL2, GLOBAL2_PVT_DATA, data);
	if (err)
		return err;

	return _mv88e6xxx_pvt_cmd(chip, src_dev, src_port,
				  GLOBAL2_PVT_ADDR_OP_WRITE_PVLAN);
}

static int mv88e6xxx_g1_get_age_time(struct mv88e6xxx_chip *chip,
				     unsigned int *msecs)
{
	u8 age_time;
	u16 val;
	int err;

	err = mv88e6xxx_read(chip, REG_GLOBAL, GLOBAL_ATU_CONTROL, &val);
	if (err)
		return err;

	/* AgeTime is 11:4 bits */
	age_time = (val & 0xff0) >> 4;
	*msecs = age_time * chip->info->age_time_coeff;

	return 0;
}

static int mv88e6xxx_regs_show(struct seq_file *s, void *p)
{
	struct mv88e6xxx_chip *chip = s->private;
	int port, reg, ret;
	u16 val;

	seq_puts(s, "    GLOBAL GLOBAL2 SERDES   ");
	for (port = 0; port < chip->info->num_ports; port++)
		seq_printf(s, " %2d  ", port);
	seq_puts(s, "\n");

	mutex_lock(&chip->reg_lock);

	for (reg = 0; reg < 32; reg++) {
		seq_printf(s, "%2x:", reg);

		ret = _mv88e6xxx_reg_read(chip, REG_GLOBAL, reg);
		if (ret < 0)
			goto unlock;
		seq_printf(s, "  %4x  ", ret);

		ret = _mv88e6xxx_reg_read(chip, REG_GLOBAL2, reg);
		if (ret < 0)
			goto unlock;
		seq_printf(s, "  %4x  ", ret);

		if (reg == PHY_PAGE) {
			/* There is no paging for registers 22 */
			seq_puts(s, "  XXXX  ");
		} else {
			ret = mv88e6xxx_serdes_read(chip, reg, &val);
			if (ret < 0)
				goto unlock;
			seq_printf(s, "  %4x  ", val);
		}

		/* Port regs 0x1a-0x1f are reserved in 6185 family */
		if (mv88e6xxx_6185_family(chip) && reg > 25) {
			for (port = 0; port < chip->info->num_ports; ++port)
				seq_printf(s, "%4c ", '-');
			seq_puts(s, "\n");
			continue;
		}

		for (port = 0; port < chip->info->num_ports; ++port) {
			ret = _mv88e6xxx_reg_read(chip, REG_PORT(port), reg);
			if (ret < 0)
				goto unlock;

			seq_printf(s, "%4x ", ret);
		}

		seq_puts(s, "\n");
	}

	ret = 0;
unlock:
	mutex_unlock(&chip->reg_lock);

	return ret;
}

static ssize_t mv88e6xxx_regs_write(struct file *file, const char __user *buf,
				    size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct mv88e6xxx_chip *chip = s->private;
	char cmd[32], name[32] = { 0 };
	unsigned int port, reg, val;
	int ret;

	if (count > sizeof(name) - 1)
		return -EINVAL;

	if (copy_from_user(cmd, buf, sizeof(cmd)))
		return -EFAULT;

	ret = sscanf(cmd, "%s %x %x", name, &reg, &val);
	if (ret != 3)
		return -EINVAL;

	if (reg > 0x1f || val > 0xffff)
		return -ERANGE;

	mutex_lock(&chip->reg_lock);

	if (strcasecmp(name, "GLOBAL") == 0)
		ret = _mv88e6xxx_reg_write(chip, REG_GLOBAL, reg, val);
	else if (strcasecmp(name, "GLOBAL2") == 0)
		ret = _mv88e6xxx_reg_write(chip, REG_GLOBAL2, reg, val);
	else if (strcasecmp(name, "SERDES") == 0)
		ret = mv88e6xxx_serdes_write(chip, reg, val);
	else if (kstrtouint(name, 10, &port) == 0 &&
		 port < chip->info->num_ports)
		ret = _mv88e6xxx_reg_write(chip, REG_PORT(port), reg, val);
	else
		ret = -EINVAL;

	mutex_unlock(&chip->reg_lock);

	return ret < 0 ? ret : count;
}

static int mv88e6xxx_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6xxx_regs_show, inode->i_private);
}

static const struct file_operations mv88e6xxx_regs_fops = {
	.open   = mv88e6xxx_regs_open,
	.read   = seq_read,
	.write  = mv88e6xxx_regs_write,
	.llseek = no_llseek,
	.release = single_release,
	.owner  = THIS_MODULE,
};

static int mv88e6xxx_age_time_show(struct seq_file *s, void *p)
{
	struct mv88e6xxx_chip *chip = s->private;
	unsigned int msecs;
	int err;

	mutex_lock(&chip->reg_lock);
	err = mv88e6xxx_g1_get_age_time(chip, &msecs);
	mutex_unlock(&chip->reg_lock);
	if (err)
		return err;

	seq_printf(s, "%d\n", msecs);

	return 0;
}

static int mv88e6xxx_age_time_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6xxx_age_time_show, inode->i_private);
}

static const struct file_operations mv88e6xxx_age_time_fops = {
	.open   = mv88e6xxx_age_time_open,
	.read   = seq_read,
	.llseek = no_llseek,
	.release = single_release,
	.owner  = THIS_MODULE,
};

static int mv88e6xxx_atu_show(struct seq_file *s, void *p)
{
	struct mv88e6xxx_chip *chip = s->private;
	struct mv88e6xxx_atu_entry addr = {
		.mac = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff },
	};
	const char *state;
	int fid, i, err;

	seq_puts(s, " FID  MAC Addr                  State         Trunk?  DPV/Trunk ID\n");

	mutex_lock(&chip->reg_lock);

	err = _mv88e6xxx_atu_mac_write(chip, addr.mac);
	if (err)
		goto unlock;

	for (fid = 0; fid < mv88e6xxx_num_databases(chip); ++fid) {
		do {
			err = _mv88e6xxx_atu_getnext(chip, fid, &addr);
			if (err)
				goto unlock;

			if (addr.state == GLOBAL_ATU_DATA_STATE_UNUSED)
				break;

			/* print ATU entry */
			seq_printf(s, "%4d", addr.fid);

			seq_printf(s, "  %.2x", addr.mac[0]);
			for (i = 1; i < ETH_ALEN; ++i)
				seq_printf(s, ":%.2x", addr.mac[i]);

			if (is_multicast_ether_addr(addr.mac)) {
				switch (addr.state) {
				case 0xf:
					/* GLOBAL_ATU_DATA_STATE_MC_PRIO_OVER */
					state = "MC_STATIC_PO";
					break;
				case 0xe:
					/* GLOBAL_ATU_DATA_STATE_MC_MGMT */
					state = "MC_STATIC_MGMT_PO";
					break;
				case 0xd:
					state = "MC_STATIC_NRL_PO";
					break;
				case 0xc:
					state = "MC_STATIC_POLICY_PO";
					break;
				case 0x7:
					/* GLOBAL_ATU_DATA_STATE_MC_STATIC */
					state = "MC_STATIC";
					break;
				case 0x6:
					state = "MC_STATIC_MGMT";
					break;
				case 0x5:
					/* GLOBAL_ATU_DATA_STATE_MC_NONE_RATE */
					state = "MC_STATIC_NRL";
					break;
				case 0x4:
					state = "MC_STATIC_POLICY";
					break;
				case 0xb: case 0xa: case 0x9: case 0x8:
					/* Reserved for future use */
				case 0x3: case 0x2: case 0x1:
					/* Reserved for future use */
				case 0x0:
					/* GLOBAL_ATU_DATA_STATE_UNUSED */
				default:
					state = "???";
					break;
				}
			} else {
				switch (addr.state) {
				case 0xf:
					/* GLOBAL_ATU_DATA_STATE_UC_PRIO_OVER */
					state = "UC_STATIC_PO";
					break;
				case 0xe:
					/* GLOBAL_ATU_DATA_STATE_UC_STATIC */
					state = "UC_STATIC";
					break;
				case 0xd:
					/* GLOBAL_ATU_DATA_STATE_UC_MGMT */
					state = "UC_STATIC_MGMT_PO";
					break;
				case 0xc:
					state = "UC_STATIC_MGMT";
					break;
				case 0xb:
					state = "UC_STATIC_NRL_PO";
					break;
				case 0xa:
					state = "UC_STATIC_NRL";
					break;
				case 0x9:
					state = "UC_STATIC_POLICY_PO";
					break;
				case 0x8:
					state = "UC_STATIC_POLICY";
					break;
				case 0x7:
					state = "Age 7 (newest)";
					break;
				case 0x6:
					state = "Age 6";
					break;
				case 0x5:
					state = "Age 5";
					break;
				case 0x4:
					state = "Age 4";
					break;
				case 0x3:
					state = "Age 3";
					break;
				case 0x2:
					state = "Age 2";
					break;
				case 0x1:
					state = "Age 1 (oldest)";
					break;
				case 0x0:
					/* GLOBAL_ATU_DATA_STATE_UNUSED */
				default:
					state = "???";
					break;
				}
			}

			seq_printf(s, "  %19s", state);

			if (addr.trunk) {
				seq_printf(s, "       y  %d",
					   addr.portv_trunkid);
			} else {
				seq_puts(s, "       n ");
				for (i = 0; i < chip->info->num_ports; ++i)
					seq_printf(s, " %c",
						   addr.portv_trunkid & BIT(i) ?
						   48 + i : '-');
			}

			seq_puts(s, "\n");
		} while (!is_broadcast_ether_addr(addr.mac));
	}

unlock:
	mutex_unlock(&chip->reg_lock);

	return err;
}

static ssize_t mv88e6xxx_atu_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct mv88e6xxx_chip *chip = s->private;
	char cmd[64];
	unsigned int fid;
	int ret;

	if (copy_from_user(cmd, buf, sizeof(cmd)))
		return -EFAULT;

	if (kstrtouint(cmd, 10, &fid))
		return -EINVAL;

	if (fid >= mv88e6xxx_num_databases(chip))
		return -ERANGE;

	mutex_lock(&chip->reg_lock);
	ret = _mv88e6xxx_atu_flush(chip, fid, true);
	mutex_unlock(&chip->reg_lock);

	return ret < 0 ? ret : count;
}

static int mv88e6xxx_atu_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6xxx_atu_show, inode->i_private);
}

static const struct file_operations mv88e6xxx_atu_fops = {
	.open   = mv88e6xxx_atu_open,
	.read   = seq_read,
	.write   = mv88e6xxx_atu_write,
	.llseek = no_llseek,
	.release = single_release,
	.owner  = THIS_MODULE,
};

static int mv88e6xxx_default_vid_show(struct seq_file *s, void *p)
{
	struct mv88e6xxx_chip *chip = s->private;
	u16 pvid;
	int i, err;

	seq_puts(s, " Port  DefaultVID\n");

	mutex_lock(&chip->reg_lock);

	for (i = 0; i < chip->info->num_ports; ++i) {
		err = _mv88e6xxx_port_pvid_get(chip, i, &pvid);
		if (err)
			break;

		seq_printf(s, "%4d  %d\n", i, pvid);
	}

	mutex_unlock(&chip->reg_lock);

	return err;
}

static ssize_t mv88e6xxx_default_vid_write(struct file *file,
					   const char __user *buf, size_t count,
					   loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct mv88e6xxx_chip *chip = s->private;
	char cmd[32];
	unsigned int port, pvid;
	int ret;

	if (copy_from_user(cmd, buf, sizeof(cmd)))
		return -EFAULT;

	ret = sscanf(cmd, "%u %u", &port, &pvid);
	if (ret != 2)
		return -EINVAL;

	if (port >= chip->info->num_ports || pvid > 0xfff)
		return -ERANGE;

	mutex_lock(&chip->reg_lock);
	ret = _mv88e6xxx_port_pvid_set(chip, port, pvid);
	mutex_unlock(&chip->reg_lock);

	return ret < 0 ? ret : count;
}

static int mv88e6xxx_default_vid_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6xxx_default_vid_show, inode->i_private);
}

static const struct file_operations mv88e6xxx_default_vid_fops = {
	.open		= mv88e6xxx_default_vid_open,
	.read		= seq_read,
	.write		= mv88e6xxx_default_vid_write,
	.llseek		= no_llseek,
	.release	= single_release,
	.owner		= THIS_MODULE,
};

static int mv88e6xxx_fid_show(struct seq_file *s, void *p)
{
	struct mv88e6xxx_chip *chip = s->private;
	u16 fid;
	int i, err;

	seq_puts(s, " Port  FID\n");

	mutex_lock(&chip->reg_lock);

	for (i = 0; i < chip->info->num_ports; ++i) {
		err = _mv88e6xxx_port_fid_get(chip, i, &fid);
		if (err)
			break;

		seq_printf(s, "%4d  %d\n", i, fid);
	}

	mutex_unlock(&chip->reg_lock);

	return err;
}

static int mv88e6xxx_fid_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6xxx_fid_show, inode->i_private);
}

static const struct file_operations mv88e6xxx_fid_fops = {
	.open		= mv88e6xxx_fid_open,
	.read		= seq_read,
	.llseek		= no_llseek,
	.release	= single_release,
	.owner		= THIS_MODULE,
};

static int mv88e6xxx_state_show(struct seq_file *s, void *p)
{
	struct mv88e6xxx_chip *chip = s->private;
	int i, ret;

	/* header */
	seq_puts(s, " Port  Mode\n");

	mutex_lock(&chip->reg_lock);

	/* One line per input port */
	for (i = 0; i < chip->info->num_ports; ++i) {
		seq_printf(s, "%4d ", i);

		ret = _mv88e6xxx_reg_read(chip, REG_PORT(i), PORT_CONTROL);
		if (ret < 0)
			goto unlock;

		ret &= PORT_CONTROL_STATE_MASK;
		seq_printf(s, " %s\n", mv88e6xxx_port_state_names[ret]);
		ret = 0;
	}

unlock:
	mutex_unlock(&chip->reg_lock);

	return ret;
}

static int mv88e6xxx_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6xxx_state_show, inode->i_private);
}

static const struct file_operations mv88e6xxx_state_fops = {
	.open		= mv88e6xxx_state_open,
	.read		= seq_read,
	.llseek		= no_llseek,
	.release	= single_release,
	.owner		= THIS_MODULE,
};

static int mv88e6xxx_8021q_mode_show(struct seq_file *s, void *p)
{
	struct mv88e6xxx_chip *chip = s->private;
	int i, ret;

	/* header */
	seq_puts(s, " Port  Mode\n");

	mutex_lock(&chip->reg_lock);

	/* One line per input port */
	for (i = 0; i < chip->info->num_ports; ++i) {
		seq_printf(s, "%4d ", i);

		ret = _mv88e6xxx_reg_read(chip, REG_PORT(i), PORT_CONTROL_2);
		if (ret < 0)
			goto unlock;

		ret &= PORT_CONTROL_2_8021Q_MASK;
		seq_printf(s, " %s\n", mv88e6xxx_port_8021q_mode_names[ret]);
		ret = 0;
	}

unlock:
	mutex_unlock(&chip->reg_lock);

	return ret;
}

static int mv88e6xxx_8021q_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6xxx_8021q_mode_show, inode->i_private);
}

static const struct file_operations mv88e6xxx_8021q_mode_fops = {
	.open		= mv88e6xxx_8021q_mode_open,
	.read		= seq_read,
	.llseek		= no_llseek,
	.release	= single_release,
	.owner		= THIS_MODULE,
};

static int mv88e6xxx_vlan_table_show(struct seq_file *s, void *p)
{
	struct mv88e6xxx_chip *chip = s->private;
	int i, j, ret;

	/* header */
	seq_puts(s, " Port");
	for (i = 0; i < chip->info->num_ports; ++i)
		seq_printf(s, " %2d", i);
	seq_puts(s, "\n");

	mutex_lock(&chip->reg_lock);

	/* One line per input port */
	for (i = 0; i < chip->info->num_ports; ++i) {
		seq_printf(s, "%4d ", i);

		ret = _mv88e6xxx_reg_read(chip, REG_PORT(i), PORT_BASE_VLAN);
		if (ret < 0)
			goto unlock;

		/* One column per output port */
		for (j = 0; j < chip->info->num_ports; ++j)
			seq_printf(s, "  %c", ret & BIT(j) ? '*' : '-');
		seq_puts(s, "\n");
	}

	ret = 0;
unlock:
	mutex_unlock(&chip->reg_lock);

	return ret;
}

static int mv88e6xxx_vlan_table_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6xxx_vlan_table_show, inode->i_private);
}

static const struct file_operations mv88e6xxx_vlan_table_fops = {
	.open		= mv88e6xxx_vlan_table_open,
	.read		= seq_read,
	.llseek		= no_llseek,
	.release	= single_release,
	.owner		= THIS_MODULE,
};

static int mv88e6xxx_pvt_show(struct seq_file *s, void *p)
{
	struct mv88e6xxx_chip *chip = s->private;
	int port, src_dev, src_port;
	u16 pvlan;
	int err;

	if (mv88e6xxx_6185_family(chip))
		return -ENODEV;

	/* header */
	seq_puts(s, " Dev Port PVLAN");
	for (port = 0; port < chip->info->num_ports; ++port)
		seq_printf(s, " %2d", port);
	seq_puts(s, "\n");

	mutex_lock(&chip->reg_lock);

	/* One line per external port */
	for (src_dev = 0; src_dev < 32; ++src_dev) {
		if (src_dev >= chip->ds->dst->pd->nr_chips)
			break;

		if (src_dev == chip->ds->index)
			continue;

		seq_puts(s, "\n");
		for (src_port = 0; src_port < 16; ++src_port) {
			if (src_port >= DSA_MAX_PORTS)
				break;

			err = _mv88e6xxx_pvt_read(chip, src_dev, src_port,
						  &pvlan);
			if (err)
				goto unlock;

			seq_printf(s, "  %d   %2d   %03hhx ", src_dev, src_port,
				   pvlan);

			/* One column per internal output port */
			for (port = 0; port < chip->info->num_ports; ++port)
				seq_printf(s, "  %c",
					   pvlan & BIT(port) ? '*' : '-');
			seq_puts(s, "\n");
		}
	}

unlock:
	mutex_unlock(&chip->reg_lock);

	return err;
}

static ssize_t mv88e6xxx_pvt_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct mv88e6xxx_chip *chip = s->private;
	const u16 mask = (1 << chip->info->num_ports) - 1;
	char cmd[32];
	unsigned int src_dev, src_port, pvlan;
	int ret;

	if (copy_from_user(cmd, buf, sizeof(cmd)))
		return -EFAULT;

	if (sscanf(cmd, "%d %d %x", &src_dev, &src_port, &pvlan) != 3)
		return -EINVAL;

	if (src_dev >= 32 || src_port >= 16 || pvlan & ~mask)
		return -ERANGE;

	mutex_lock(&chip->reg_lock);
	ret = _mv88e6xxx_pvt_write(chip, src_dev, src_port, pvlan);
	mutex_unlock(&chip->reg_lock);

	return ret < 0 ? ret : count;
}

static int mv88e6xxx_pvt_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6xxx_pvt_show, inode->i_private);
}

static const struct file_operations mv88e6xxx_pvt_fops = {
	.open		= mv88e6xxx_pvt_open,
	.read		= seq_read,
	.write		= mv88e6xxx_pvt_write,
	.llseek		= no_llseek,
	.release	= single_release,
	.owner		= THIS_MODULE,
};

static int mv88e6xxx_vtu_show(struct seq_file *s, void *p)
{
	struct mv88e6xxx_chip *chip = s->private;
	int port, ret = 0, vid = GLOBAL_VTU_VID_MASK; /* first or lowest VID */

	seq_puts(s, " VID  FID  SID");
	for (port = 0; port < chip->info->num_ports; ++port)
		seq_printf(s, " %2d", port);
	seq_puts(s, "\n");

	mutex_lock(&chip->reg_lock);

	ret = _mv88e6xxx_vtu_vid_write(chip, vid);
	if (ret < 0)
		goto unlock;

	do {
		struct mv88e6xxx_vtu_stu_entry next = { 0 };

		ret = _mv88e6xxx_vtu_getnext(chip, &next);
		if (ret < 0)
			goto unlock;

		if (!next.valid)
			break;

		seq_printf(s, "%4d %4d   %2d", next.vid, next.fid, next.sid);
		for (port = 0; port < chip->info->num_ports; ++port) {
			switch (next.data[port]) {
			case GLOBAL_VTU_DATA_MEMBER_TAG_UNMODIFIED:
				seq_puts(s, "  =");
				break;
			case GLOBAL_VTU_DATA_MEMBER_TAG_UNTAGGED:
				seq_puts(s, "  u");
				break;
			case GLOBAL_VTU_DATA_MEMBER_TAG_TAGGED:
				seq_puts(s, "  t");
				break;
			case GLOBAL_VTU_DATA_MEMBER_TAG_NON_MEMBER:
				seq_puts(s, "  x");
				break;
			default:
				seq_puts(s, " ??");
				break;
			}
		}
		seq_puts(s, "\n");

		vid = next.vid;
	} while (vid < GLOBAL_VTU_VID_MASK);

unlock:
	mutex_unlock(&chip->reg_lock);

	return ret;
}

static ssize_t mv88e6xxx_vtu_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct mv88e6xxx_chip *chip = s->private;
	struct mv88e6xxx_vtu_stu_entry entry = { 0 };
	bool valid = true;
	char cmd[64], tags[12]; /* DSA_MAX_PORTS */
	int vid, fid, sid, port, ret;

	if (copy_from_user(cmd, buf, sizeof(cmd)))
		return -EFAULT;

	/* scan 12 chars instead of num_ports to avoid dynamic scanning... */
	ret = sscanf(cmd, "%d %d %d %c %c %c %c %c %c %c %c %c %c %c %c", &vid,
		     &fid, &sid, &tags[0], &tags[1], &tags[2], &tags[3],
		     &tags[4], &tags[5], &tags[6], &tags[7], &tags[8], &tags[9],
		     &tags[10], &tags[11]);
	if (ret == 1)
		valid = false;
	else if (ret != 3 + chip->info->num_ports)
		return -EINVAL;

	entry.vid = vid;
	entry.valid = valid;

	if (valid) {
		entry.fid = fid;
		entry.sid = sid;
		/* Note: The VTU entry pointed by VID will be loaded but not
		 * considered valid until the STU entry pointed by SID is valid.
		 */

		for (port = 0; port < chip->info->num_ports; ++port) {
			u8 tag;

			switch (tags[port]) {
			case 'u':
				tag = GLOBAL_VTU_DATA_MEMBER_TAG_UNTAGGED;
				break;
			case 't':
				tag = GLOBAL_VTU_DATA_MEMBER_TAG_TAGGED;
				break;
			case 'x':
				tag = GLOBAL_VTU_DATA_MEMBER_TAG_NON_MEMBER;
				break;
			case '=':
				tag = GLOBAL_VTU_DATA_MEMBER_TAG_UNMODIFIED;
				break;
			default:
				return -EINVAL;
			}

			entry.data[port] = tag;
		}
	}

	mutex_lock(&chip->reg_lock);
	ret = _mv88e6xxx_vtu_loadpurge(chip, &entry);
	mutex_unlock(&chip->reg_lock);

	return ret < 0 ? ret : count;
}

static int mv88e6xxx_vtu_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6xxx_vtu_show, inode->i_private);
}

static const struct file_operations mv88e6xxx_vtu_fops = {
	.open		= mv88e6xxx_vtu_open,
	.read		= seq_read,
	.write		= mv88e6xxx_vtu_write,
	.llseek		= no_llseek,
	.release	= single_release,
	.owner		= THIS_MODULE,
};

static int mv88e6xxx_stats_show(struct seq_file *s, void *p)
{
	struct mv88e6xxx_chip *chip = s->private;
	int stat, port;
	int err = 0;

	seq_puts(s, "          Statistic  ");
	for (port = 0; port < chip->info->num_ports; port++)
		seq_printf(s, " Port %2d ", port);
	seq_puts(s, "\n");

	mutex_lock(&chip->reg_lock);

	for (stat = 0; stat < ARRAY_SIZE(mv88e6xxx_hw_stats); stat++) {
		struct mv88e6xxx_hw_stat *hw_stat = &mv88e6xxx_hw_stats[stat];

		if (!mv88e6xxx_has_stat(chip, hw_stat))
			continue;

		seq_printf(s, "%19s: ", hw_stat->string);
		for (port = 0 ; port < chip->info->num_ports; port++) {
			u64 value;

			err = _mv88e6xxx_stats_snapshot(chip, port);
			if (err)
				goto unlock;

			value = _mv88e6xxx_get_ethtool_stat(chip, hw_stat,
							    port);
			seq_printf(s, "%8llu ", value);
		}
		seq_puts(s, "\n");
	}

unlock:
	mutex_unlock(&chip->reg_lock);

	return err;
}

static int mv88e6xxx_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6xxx_stats_show, inode->i_private);
}

static const struct file_operations mv88e6xxx_stats_fops = {
	.open   = mv88e6xxx_stats_open,
	.read   = seq_read,
	.llseek = no_llseek,
	.release = single_release,
	.owner  = THIS_MODULE,
};

static int mv88e6xxx_device_map_show(struct seq_file *s, void *p)
{
	struct mv88e6xxx_chip *chip = s->private;
	int target, ret;

	seq_puts(s, "Target Port\n");

	mutex_lock(&chip->reg_lock);
	for (target = 0; target < 32; target++) {
		ret = _mv88e6xxx_reg_write(
			chip, REG_GLOBAL2, GLOBAL2_DEVICE_MAPPING,
			target << GLOBAL2_DEVICE_MAPPING_TARGET_SHIFT);
		if (ret < 0)
			goto out;
		ret = _mv88e6xxx_reg_read(chip, REG_GLOBAL2,
					  GLOBAL2_DEVICE_MAPPING);
		seq_printf(s, "  %2d   %2d\n", target,
			   ret & GLOBAL2_DEVICE_MAPPING_PORT_MASK);
	}
out:
	mutex_unlock(&chip->reg_lock);

	return 0;
}

static int mv88e6xxx_device_map_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6xxx_device_map_show, inode->i_private);
}

static const struct file_operations mv88e6xxx_device_map_fops = {
	.open   = mv88e6xxx_device_map_open,
	.read   = seq_read,
	.llseek = no_llseek,
	.release = single_release,
	.owner  = THIS_MODULE,
};

/* Must be called with SMI lock held */
static int _mv88e6xxx_scratch_wait(struct mv88e6xxx_chip *chip)
{
	return mv88e6xxx_wait(chip, REG_GLOBAL2, GLOBAL2_SCRATCH_MISC,
			      GLOBAL2_SCRATCH_BUSY);
}

static int mv88e6xxx_scratch_show(struct seq_file *s, void *p)
{
	struct mv88e6xxx_chip *chip = s->private;
	int reg, ret;

	seq_puts(s, "Register Value\n");

	mutex_lock(&chip->reg_lock);
	for (reg = 0; reg < 0x80; reg++) {
		ret = _mv88e6xxx_reg_write(
			chip, REG_GLOBAL2, GLOBAL2_SCRATCH_MISC,
			reg << GLOBAL2_SCRATCH_REGISTER_SHIFT);
		if (ret < 0)
			goto out;

		ret = _mv88e6xxx_scratch_wait(chip);
		if (ret < 0)
			goto out;

		ret = _mv88e6xxx_reg_read(chip, REG_GLOBAL2,
					  GLOBAL2_SCRATCH_MISC);
		seq_printf(s, "  %2x   %2x\n", reg,
			   ret & GLOBAL2_SCRATCH_VALUE_MASK);
	}
out:
	mutex_unlock(&chip->reg_lock);

	return 0;
}

static int mv88e6xxx_scratch_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6xxx_scratch_show, inode->i_private);
}

static const struct file_operations mv88e6xxx_scratch_fops = {
	.open   = mv88e6xxx_scratch_open,
	.read   = seq_read,
	.llseek = no_llseek,
	.release = single_release,
	.owner  = THIS_MODULE,
};

static char *mv88e6xxx_tcam_frame_type_str(int frame_type)
{
	switch (frame_type) {
	case GLOBAL3_P0_KEY1_FRAME_TYPE_NORNAL:
		return "Frame type: Nornal";
	case GLOBAL3_P0_KEY1_FRAME_TYPE_DSA:
		return "frame type: DSA";
	case GLOBAL3_P0_KEY1_FRAME_TYPE_PROVIDER:
		return "frame type: Provider";
	default:
		return "frame type: Unknown";
	}
}

static int mv88e6xxx_tcam_show_entry(struct mv88e6xxx_chip *chip,
				     struct seq_file *s, int entry,
				     struct mv88e6xxx_tcam_data *data)
{
	int err, i, value;
	u8 octet, mask;

	seq_puts(s, "      Dst          Src          Tag      Type Data\n");
	seq_printf(s, "Entry %3d\n", entry);
	seq_puts(s, "Octet:");
	for (i = 0; i < 48; i++) {
		/* -Dst-------Src-------Tag--------Eth Type----Data-- */
		if (i == 6 || i == 12 || i == 16 || i == 18 || i == 26 ||
		    i == 34 || i == 42)
			seq_puts(s, " ");

		err = mv88e6xxx_tcam_get_match(chip, data, i, &octet, &mask);
		if (err)
			return err;
		seq_printf(s, "%02x", octet);
	}
	seq_puts(s, "\n");

	seq_puts(s, "Mask: ");
	for (i = 0; i < 48; i++) {
		/* -Dst-------Src-------Tag--------Eth Type----Data-- */
		if (i == 6 || i == 12 || i == 16 || i == 18 || i == 26 ||
		    i == 34 || i == 42)
			seq_puts(s, " ");

		err = mv88e6xxx_tcam_get_match(chip, data, i, &octet, &mask);
		if (err)
			return err;
		seq_printf(s, "%02x", mask);
	}
	seq_puts(s, "\n");

	mv88e6xxx_tcam_get(chip, data, MV88E6XXX_P0_KEY1_FRAME_TYPE,
			   &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "%s ", mv88e6xxx_tcam_frame_type_str(value));

	mv88e6xxx_tcam_get(chip, data, MV88E6XXX_P0_KEY2_SRC_PORT_VECTOR,
			   &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "Source port vector: %x ", value);

	mv88e6xxx_tcam_get(chip, data, MV88E6XXX_P0_KEY3_PPRI, &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "Provider priority: %d ", value);

	mv88e6xxx_tcam_get(chip, data, MV88E6XXX_P0_KEY4_PVID, &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "Provider VLAN ID: %d ", value);

	mv88e6xxx_tcam_get(chip, data, MV88E6XXX_P2_ACTION1_INTERRUPT,
			   &value);
	seq_printf(s, "Interrupt: %d ",
		   value == GLOBAL3_P2_ACTION1_INTERRUPT);

	mv88e6xxx_tcam_get(chip, data, MV88E6XXX_P2_ACTION1_INC_TCAM_COUNTER,
			   &value);
	seq_printf(s, "Inc TCAM counter: %d ",
		   value == GLOBAL3_P2_ACTION1_INC_TCAM_COUNTER);

	mv88e6xxx_tcam_get(chip, data, MV88E6XXX_P2_ACTION1_VID, &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "VID: %d ", value);

	mv88e6xxx_tcam_get(chip, data, MV88E6XXX_P2_ACTION2_FLOW_ID, &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "Flow ID: %d ",
			   value - GLOBAL3_P2_ACTION2_FLOW_ID_0);

	mv88e6xxx_tcam_get(chip, data, MV88E6XXX_P2_ACTION2_QPRI, &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "Queue priority: %d ",
			   value - GLOBAL3_P2_ACTION2_QPRI_0);

	mv88e6xxx_tcam_get(chip, data, MV88E6XXX_P2_ACTION2_FPRI, &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "Priority: %d ",
			   value - GLOBAL3_P2_ACTION2_FPRI_0);

	mv88e6xxx_tcam_get(chip, data, MV88E6XXX_P2_ACTION3_DST_PORT_VECTOR,
			   &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "Destination port vector: %x ", value);

	mv88e6xxx_tcam_get(chip, data, MV88E6XXX_P2_ACTION4_FRAME_ACTION,
			   &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED) {
		seq_printf(s, "Frame Action: %x ", value);
		if (value & GLOBAL3_P2_ACTION4_FRAME_ACTION_SRC_IS_TAGGED)
			seq_puts(s, "SRC_IS_TAGGED ");
		if (value & GLOBAL3_P2_ACTION4_FRAME_ACTION_PVID)
			seq_puts(s, "PVID ");
		if (value & GLOBAL3_P2_ACTION4_FRAME_ACTION_MGMT)
			seq_puts(s, "MGMT ");
		if (value & GLOBAL3_P2_ACTION4_FRAME_ACTION_SNOOP)
			seq_puts(s, "SNOOP ");
		if (value & GLOBAL3_P2_ACTION4_FRAME_ACTION_POLICY_MIRROR)
			seq_puts(s, "POLICY_MIRROR ");
		if (value & GLOBAL3_P2_ACTION4_FRAME_ACTION_POLICY_TRAP)
			seq_puts(s, "POLICY_TRAP ");
		if (value & GLOBAL3_P2_ACTION4_FRAME_ACTION_SANRL)
			seq_puts(s, "SaNRL ");
		if (value & GLOBAL3_P2_ACTION4_FRAME_ACTION_DANRL)
			seq_puts(s, "DaNRL ");
	}

	mv88e6xxx_tcam_get(chip, data, MV88E6XXX_P2_ACTION4_LOAD_BALANCE,
			   &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "Load balance: %d", value);

	mv88e6xxx_tcam_get(chip, data, MV88E6XXX_P2_DEBUG_PORT, &value);
	seq_printf(s, "Debug Port: %d ", value);
	mv88e6xxx_tcam_get(chip, data, MV88E6XXX_P2_DEBUG_HIT, &value);
	seq_printf(s, "Debug Hit %x\n", value);

	return 0;
}

static int mv88e6xxx_tcam_all_show(struct seq_file *s, void *p)
{
	struct mv88e6xxx_chip *chip = s->private;
	struct mv88e6xxx_tcam_data data;
	int err, entry;

	for (entry = 0; entry < 255; entry++) {
		err = mv88e6xxx_tcam_read(chip, entry, &data);
		if (err)
			return err;

		err = mv88e6xxx_tcam_show_entry(chip, s, entry, &data);
		if (err)
			return err;
	}

	return 0;
}

static ssize_t mv88e6xxx_tcam_all_write(struct file *file,
					const char __user *buf,
					size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct mv88e6xxx_chip *chip = s->private;
	struct mv88e6xxx_tcam_data data;

	memset(&data, 0, sizeof(data));

	mv88e6xxx_tcam_flush_all(chip);
	mv88e6xxx_tcam_port_enable(chip, 0);
	mv88e6xxx_tcam_port_enable(chip, 1);

	/* Destination - Broadcast address */
	mv88e6xxx_tcam_set_match(chip, &data, 0, 0xff, 0xff);
	mv88e6xxx_tcam_set_match(chip, &data, 1, 0xff, 0xff);
	mv88e6xxx_tcam_set_match(chip, &data, 2, 0xff, 0xff);
	mv88e6xxx_tcam_set_match(chip, &data, 3, 0xff, 0xff);
	mv88e6xxx_tcam_set_match(chip, &data, 4, 0xff, 0xff);
	mv88e6xxx_tcam_set_match(chip, &data, 5, 0xff, 0xff);

	/* Source Port 0 */
	mv88e6xxx_tcam_set(chip, &data, MV88E6XXX_P0_KEY2_SRC_PORT_VECTOR,
			   (1 << 0));

	/* Destination port None, i.e. drop */
	mv88e6xxx_tcam_set(chip, &data, MV88E6XXX_P2_ACTION3_DST_PORT_VECTOR,
			   0);

	mv88e6xxx_tcam_load_entry(chip, 42, &data);

	memset(&data, 0, sizeof(data));

	/* Source 00:26:55:d2:27:a9 */
	mv88e6xxx_tcam_set_match(chip, &data, 6, 0x00, 0xff);
	mv88e6xxx_tcam_set_match(chip, &data, 7, 0x26, 0xff);
	mv88e6xxx_tcam_set_match(chip, &data, 8, 0x55, 0xff);
	mv88e6xxx_tcam_set_match(chip, &data, 9, 0xd2, 0xff);
	mv88e6xxx_tcam_set_match(chip, &data, 10, 0x27, 0xff);
	mv88e6xxx_tcam_set_match(chip, &data, 11, 0xa9, 0xff);

	/* Ether Type 0x0806 - ARP */
	mv88e6xxx_tcam_set_match(chip, &data, 16, 0x08, 0xff);
	mv88e6xxx_tcam_set_match(chip, &data, 17, 0x06, 0xff);

	/* ARP Hardware Type 1  - Ethernet */
	mv88e6xxx_tcam_set_match(chip, &data, 18, 0x00, 0xff);
	mv88e6xxx_tcam_set_match(chip, &data, 19, 0x01, 0xff);

	/* ARP protocol Type 0x0800 - IP */
	mv88e6xxx_tcam_set_match(chip, &data, 20, 0x08, 0xff);
	mv88e6xxx_tcam_set_match(chip, &data, 21, 0x00, 0xff);

	/* Operation 2 - reply */
	mv88e6xxx_tcam_set_match(chip, &data, 24, 0x00, 0xff);
	mv88e6xxx_tcam_set_match(chip, &data, 25, 0x02, 0xff);

	/* Source Port 1 */
	mv88e6xxx_tcam_set(chip, &data, MV88E6XXX_P0_KEY2_SRC_PORT_VECTOR,
			   (1 << 1));

	/* Destination port None, i.e. drop */
	mv88e6xxx_tcam_set(chip, &data, MV88E6XXX_P2_ACTION3_DST_PORT_VECTOR,
			   0);

	mv88e6xxx_tcam_load_entry(chip, 43, &data);

	return count;
}

static int mv88e6xxx_tcam_all_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6xxx_tcam_all_show, inode->i_private);
}

static const struct file_operations mv88e6xxx_tcam_all_fops = {
	.open   = mv88e6xxx_tcam_all_open,
	.read   = seq_read,
	.write  = mv88e6xxx_tcam_all_write,
	.llseek = no_llseek,
	.release = single_release,
	.owner  = THIS_MODULE,
};

static int mv88e6xxx_tcam_show(struct seq_file *s, void *p)
{
	struct mv88e6xxx_chip *chip = s->private;
	struct mv88e6xxx_tcam_data data;
	int err, entry = 0;

	while (1) {
		err = mv88e6xxx_tcam_get_next(chip, &entry, &data);
		if (err)
			return err;

		if (entry == 0xff)
			break;

		err = mv88e6xxx_tcam_show_entry(chip, s, entry, &data);
		if (err)
			return err;
	}

	return 0;
}

static int mv88e6xxx_tcam_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6xxx_tcam_show, inode->i_private);
}

static const struct file_operations mv88e6xxx_tcam_fops = {
	.open   = mv88e6xxx_tcam_open,
	.read   = seq_read,
	.llseek = no_llseek,
	.release = single_release,
	.owner  = THIS_MODULE,
};

static void mv88e6xxx_init_debugfs(struct mv88e6xxx_chip *chip)
{
	char *name;

	name = kasprintf(GFP_KERNEL, "mv88e6xxx.%d", chip->ds->index);
	chip->dbgfs = debugfs_create_dir(name, NULL);
	kfree(name);

	debugfs_create_file("regs", S_IRUGO | S_IWUSR, chip->dbgfs, chip,
			    &mv88e6xxx_regs_fops);

	debugfs_create_file("age_time", S_IRUGO, chip->dbgfs, chip,
			    &mv88e6xxx_age_time_fops);

	debugfs_create_file("atu", S_IRUGO | S_IWUSR, chip->dbgfs, chip,
			    &mv88e6xxx_atu_fops);

	debugfs_create_file("default_vid", S_IRUGO | S_IWUSR, chip->dbgfs, chip,
			    &mv88e6xxx_default_vid_fops);

	debugfs_create_file("fid", S_IRUGO, chip->dbgfs, chip,
			    &mv88e6xxx_fid_fops);

	debugfs_create_file("state", S_IRUGO, chip->dbgfs, chip,
			    &mv88e6xxx_state_fops);

	debugfs_create_file("8021q_mode", S_IRUGO, chip->dbgfs, chip,
			    &mv88e6xxx_8021q_mode_fops);

	debugfs_create_file("vlan_table", S_IRUGO, chip->dbgfs, chip,
			    &mv88e6xxx_vlan_table_fops);

	debugfs_create_file("pvt", S_IRUGO | S_IWUSR, chip->dbgfs, chip,
			    &mv88e6xxx_pvt_fops);

	debugfs_create_file("vtu", S_IRUGO | S_IWUSR, chip->dbgfs, chip,
			    &mv88e6xxx_vtu_fops);

	debugfs_create_file("stats", S_IRUGO, chip->dbgfs, chip,
			    &mv88e6xxx_stats_fops);

	debugfs_create_file("device_map", S_IRUGO, chip->dbgfs, chip,
			    &mv88e6xxx_device_map_fops);

	debugfs_create_file("scratch", S_IRUGO, chip->dbgfs, chip,
			    &mv88e6xxx_scratch_fops);

	if (mv88e6xxx_has(chip, MV88E6XXX_FLAG_TCAM)) {
		debugfs_create_file("tcam-all", S_IRUGO, chip->dbgfs, chip,
				    &mv88e6xxx_tcam_all_fops);
		debugfs_create_file("tcam", S_IRUGO, chip->dbgfs, chip,
				    &mv88e6xxx_tcam_fops);
	}
}
