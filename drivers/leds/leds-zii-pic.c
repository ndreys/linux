/*
 * zii-pic-leds.c - driver for PIC LEDs on Zodiac Inflight Innovations
 * platforms.
 *
 * (C) 2016-2017, Nikita Yushchenko <nikita.yoush@cogentembedded.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/zii-pic.h>
#include <linux/leds.h>

enum zii_led_id {
	ZII_FRONT_PANEL_LED = 0x0,
	ZII_MOOD_LED = 0x1,
	ZII_READING_LED = 0x2,

	ZII_LEDS_NR,
};

enum zii_led_state {
	ZII_LED_STATE_OFF = 0x0,
	ZII_LED_STATE_ON = 0x1,
};

enum zii_led_channel {
	CHANNEL_A = 0,
	CHANNEL_R = 1,
	CHANNEL_G = 2,
	CHANNEL_B = 3,

	CHANNELS_NR
};

static const char *channel_names[] = {
	[CHANNEL_A] = "brightness",
	[CHANNEL_R] = "red",
	[CHANNEL_G] = "green",
	[CHANNEL_B] = "blue",
};

static const char *led_names[] = {
	[ZII_FRONT_PANEL_LED] = "front",
	[ZII_MOOD_LED] = "mood",
	[ZII_READING_LED] = "reading",
};

#define ALL_CHANNELS	(BIT(CHANNEL_A) | BIT(CHANNEL_R) | \
			 BIT(CHANNEL_G) | BIT(CHANNEL_B))


static const u8 channel_masks[] = {
	[ZII_FRONT_PANEL_LED] = ALL_CHANNELS,
	[ZII_MOOD_LED] = ALL_CHANNELS,
	[ZII_READING_LED] = BIT(CHANNEL_G),
};

struct zii_pic_leds {
	struct zii_pic *zp;
	struct {
		u8 v[CHANNELS_NR];
		struct led_classdev ld[CHANNELS_NR];
		struct mutex mutex;
	} l[ZII_LEDS_NR];
};

static void zl_decode(struct led_classdev *led_dev,
		struct zii_pic_leds **zleds_ret,
		enum zii_led_id *id_ret,
		enum zii_led_channel *ch_ret)
{
	struct zii_pic_leds *zleds;
	enum zii_led_id id;
	enum zii_led_channel ch;

	zleds = dev_get_drvdata(led_dev->dev->parent);
	for (id = 0; id < ZII_LEDS_NR; id++) {
		for (ch = 0; ch < CHANNELS_NR; ch++) {
			if (led_dev == &zleds->l[id].ld[ch]) {
				*zleds_ret = zleds;
				*id_ret = id;
				*ch_ret = ch;
				return;
			}
		}
	}

	BUG();
}

static inline void zl_lock_init(struct zii_pic_leds *zleds, enum zii_led_id id)
{
	mutex_init(&zleds->l[id].mutex);
}

static inline void zl_lock(struct zii_pic_leds *zleds, enum zii_led_id id)
{
	mutex_lock(&zleds->l[id].mutex);
}

static inline void zl_unlock(struct zii_pic_leds *zleds, enum zii_led_id id)
{
	mutex_unlock(&zleds->l[id].mutex);
}

#define CMD_LEDS	0x28
#define RSP_LEDS	0x68

static int zl_fetch(struct zii_pic_leds *zleds, enum zii_led_id id)
{
	u8 cmd[] = { 0, id, 0, 0, 0, 0, 0, 0, 0 };
	u8 rsp[5];
	int ret;

	ret = zii_pic_exec(zleds->zp, CMD_LEDS, cmd, sizeof(cmd),
			RSP_LEDS, rsp, sizeof(rsp));
	if (ret)
		return ret;

	if (rsp[0] == ZII_LED_STATE_OFF) {
		zleds->l[id].v[CHANNEL_A] = 0;
		zleds->l[id].v[CHANNEL_R] = 0;
		zleds->l[id].v[CHANNEL_G] = 0;
		zleds->l[id].v[CHANNEL_B] = 0;
	} else {
		zleds->l[id].v[CHANNEL_A] = rsp[1];
		zleds->l[id].v[CHANNEL_R] = rsp[2];
		zleds->l[id].v[CHANNEL_G] = rsp[3];
		zleds->l[id].v[CHANNEL_B] = rsp[4];
	}

	return 0;
}

static int zl_apply(struct zii_pic_leds *zleds, enum zii_led_id id)
{
	enum zii_led_state state;
	u8 a, r, g, b;
	u8 cmd[9];

	a = zleds->l[id].v[CHANNEL_A];
	r = zleds->l[id].v[CHANNEL_R];
	g = zleds->l[id].v[CHANNEL_G];
	b = zleds->l[id].v[CHANNEL_B];
	state = (a | r | g | b) ? ZII_LED_STATE_ON : ZII_LED_STATE_OFF;

	cmd[0] = 1;
	cmd[1] = id;
	cmd[2] = state;
	cmd[3] = 0;
	cmd[4] = 0;
	cmd[5] = a;
	cmd[6] = r;
	cmd[7] = g;
	cmd[8] = b;

	return zii_pic_exec(zleds->zp, CMD_LEDS, cmd, sizeof(cmd),
			RSP_LEDS, NULL, 0);
}

static int zl_brightness_set(struct led_classdev *led_dev,
		enum led_brightness value)
{
	struct zii_pic_leds *zleds;
	enum zii_led_id id;
	enum zii_led_channel ch;
	int ret;

	zl_decode(led_dev, &zleds, &id, &ch);
	zl_lock(zleds, id);
	zleds->l[id].v[ch] = value;
	ret = zl_apply(zleds, id);
	zl_unlock(zleds, id);

	return ret;
}

static enum led_brightness zl_brightness_get(struct led_classdev *led_dev)
{
	struct zii_pic_leds *zleds;
	enum zii_led_id id;
	enum zii_led_channel ch;
	enum led_brightness ret;

	zl_decode(led_dev, &zleds, &id, &ch);
	zl_lock(zleds, id);
	zl_fetch(zleds, id);
	ret = zleds->l[id].v[ch];
	zl_unlock(zleds, id);

	return ret;
}

static int zii_pic_leds_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zii_pic *zp = zii_pic_parent(dev);
	struct zii_pic_leds *zleds;
	enum zii_led_id id;
	enum zii_led_channel ch;
	struct led_classdev *ld;
	int ret;

	if (!zp)
		return -EINVAL;
	if (zp->hw_id != ZII_PIC_HW_ID_RDU2)
		return -ENODEV;		/* currently only RDU2 is supported */

	zleds = devm_kzalloc(dev, sizeof(*zleds), GFP_KERNEL);
	if (!zleds)
		return -ENOMEM;
	zleds->zp = zp;
	dev_set_drvdata(dev, zleds);

	for (id = 0; id < ZII_LEDS_NR; id++) {

		zl_lock_init(zleds, id);

		zl_lock(zleds, id);
		ret = zl_fetch(zleds, id);
		zl_unlock(zleds, id);
		if (ret) {
			dev_warn(dev, "could not fetch led %d - disabling\n",
					id);
			continue;
		}

		for (ch = 0; ch < CHANNELS_NR; ch++) {

			if (!(channel_masks[id] & (1 << ch))) {
				zleds->l[id].v[ch] = 255;
				continue;
			}

			ld = &(zleds->l[id].ld[ch]);

			if (channel_masks[id] & ~(1 << ch))
				ld->name = devm_kasprintf(dev, GFP_KERNEL,
					"%s:%s",
					led_names[id], channel_names[ch]);
			else
				ld->name = led_names[id];

			ld->brightness = zleds->l[id].v[ch];
			ld->max_brightness = 255;

			ld->brightness_set_blocking = zl_brightness_set;
			ld->brightness_get = zl_brightness_get;

			ret = devm_led_classdev_register(dev, ld);
			if (ret) {
				dev_err(dev, "could not register %s led\n",
						ld->name);
				return ret;
			}
		}

		zl_lock(zleds, id);
		ret = zl_apply(zleds, id);
		zl_unlock(zleds, id);
		if (ret) {
			dev_warn(dev, "could not apply settings for led %d\n",
					id);
		}
	}

	return 0;
}

static const struct of_device_id zii_pic_leds_of_match[] = {
	{ .compatible = "zii,pic-leds" },
	{}
};

static struct platform_driver zii_pic_leds_driver = {
	.probe = zii_pic_leds_probe,
	.driver = {
		.name = ZII_PIC_NAME_LEDS,
		.of_match_table = zii_pic_leds_of_match,
	},
};
module_platform_driver(zii_pic_leds_driver);

MODULE_DEVICE_TABLE(of, zii_pic_leds_of_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Yushchenko <nikita.yoush@cogentembedded.com>");
MODULE_DESCRIPTION("ZII PIC LEDs driver");
