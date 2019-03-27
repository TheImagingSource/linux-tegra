#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include <linux/of.h>
#include <linux/of_device.h>


#define TIS_FPDLINK_FLAG_VERIFY 1

struct tis_fpdlink {
	struct regmap *deser_regmap;
	struct regmap *ser_regmap;
	struct i2c_client *ser_client;
	struct dentry *fpdlink_dir;
	struct dentry *deser_d;
	struct dentry *ser_d;
	u8 debug_addr;

	u8 slave_addr;
	u8 slave_map_addr;
	u8 ser_map_addr;
	u8 clkout_ctrl1;
	u8 clkout_ctrl2;
};

static const struct regmap_config tis_fpdlink_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.cache_type	= REGCACHE_RBTREE,
	.use_single_rw	= true,
};

struct fpdlink_reg {
	char addr;
	char val;
	unsigned int flags;
};

static const struct of_device_id tis_fpdlink_of_match [] = {
	{ .compatible = "tis,tis-fpdlink" },
	{ }
};

static const struct fpdlink_reg fpdlink_deser_reglist[] = {
	{0x0d, 0x09, 0x0}, // IO_CTL ( 0x9 = Detect IO Voltage level)
	{0x0c, 0x6d, 0x0}, // RX_PORT_CTL (0x01 = Port 0 Enable, LOCK on Port 0, PASS on Port 0)
	{0x7d, 0x05, 0x0}, // PORT_CONFIG (0x05 = Enable Watchdog, Assert PASS after 1 Frame)
	{0x0f, 0x7f, 0x0}, // GPIO_INPUT_CTL,
	{0x10, 0x0, 0x0}, // GPIO0_PIN_CTL,
	{0x11, 0x0, 0x0}, // GPIO1_PIN_CTL,
	{0x15, 0x13, 0x0}, // GPIO5_PIN_CTL,
	{0x4c, 0x01, 0x0}, // FPD3_PORT_SEL,

	{0x59, 0x82, TIS_FPDLINK_FLAG_VERIFY}, // DATAPATH_CTL1 ( 0x82 = Disable loading from FC; Forward two GPIOs )
	{0x6e, 0x10, TIS_FPDLINK_FLAG_VERIFY}, // BC_GPIO_CTL0 ( 0x10 = GPIO0 on BC_GPIO0 and GPIO1 on BC_GPIO1 )

	{0x58, 0x5e, 0x0}, // BCC_CONFIG
	{0x5c, 0x30, 0x0}, // SER_ALIAS_ID

	{0x1f, 0x00, 0x0}, // CSI_PLL_CTL (0x0 = 1.6gps, 0x2 = 800Mbps)
	{0x33, 0x21, 0x0}, // CSI_CTL ( 0x21 = 2 lanes, CSI Enable, Continuous Clock Disable )
	{0x21, 0x01, 0x0}, // FWD_CTL2 ( 0x1 = round robin forward )
	{0x20, 0x00, 0x0}, // FWD_CTL1 ( 0x20 = disable rx port 1 / 0x00 = forward both ports )
	{0x6d, 0x7c, 0x0}, // Configures port to coax mode and FPD III to CSI mode

	{ },
};

static const struct fpdlink_reg fpdlink_ser_reglist[] = {
	// {0x06, 0x41, 0x0},
	// {0x07, 0x19, 0x0},
	{0x0b, 0x19, 0x0}, // SCL_HIGH_TIME
	{0x0c, 0x19, 0x0}, // SCL_LOW_TIME
	{0x0d, 0x30, 0x0}, // LOCAL_GPIO_DATA (0x30 = Enable remote GPIO0 & GPIO1 )
	{0x0e, 0x3c, 0x0}, // GPIO_INPUT_CTL: GPIO0 & GPIO1 Output Enable, GPIO_0 & GPIO1 Input Disable
	{0x33, 0x0, 0x0}, // DATAPATH_CTL1 ( 0x0 = No forward of GPIO from serializer )
	{0x02, 0x13, 0x0}, // GENERAL_CFG ( 0xb = 2-lane config, CRC enable, 1.8V Mode)

	// {0x0d, 0x0, 0x0},
	// {0x0e, 0x03, 0x0},
	{ },
};

// 1948 x 1096
// 0x79c x 0x448
static const struct fpdlink_reg fpdlink_ser_patgen[] = {
	{0xB0, 0x00, 0x0}, // # Indirect Pattern Gen Registers
	{0xB1, 0x01, 0x0}, // # PGEN_CTL
	{0xB2, 0x01, 0x0}, //
	{0xB1, 0x02, 0x0}, // # PGEN_CFG
	{0xB2, 0x33, 0x0}, //
	{0xB1, 0x03, 0x0}, // # PGEN_CSI_DI
	{0xB2, 0x24, 0x0}, //
	{0xB1, 0x04, 0x0}, // # PGEN_LINE_SIZE1
	{0xB2, 0x07, 0x0}, //
	{0xB1, 0x05, 0x0}, // # PGEN_LINE_SIZE0
	{0xB2, 0x9c, 0x0}, //
	{0xB1, 0x06, 0x0}, // # PGEN_BAR_SIZE1
	{0xB2, 0x01, 0x0}, //
	{0xB1, 0x07, 0x0}, // # PGEN_BAR_SIZE0
	{0xB2, 0xE0, 0x0}, //
	{0xB1, 0x08, 0x0}, // # PGEN_ACT_LPF1
	{0xB2, 0x04, 0x0}, //
	{0xB1, 0x09, 0x0}, // # PGEN_ACT_LPF0
	{0xB2, 0x48, 0x0}, //
	{0xB1, 0x0A, 0x0}, // # PGEN_TOT_LPF1
	{0xB2, 0x04, 0x0}, //
	{0xB1, 0x0B, 0x0}, // # PGEN_TOT_LPF0
	{0xB2, 0x48, 0x0}, //
	{0xB1, 0x0C, 0x0}, // # PGEN_LINE_PD1
	{0xB2, 0x0C, 0x0}, //
	{0xB1, 0x0D, 0x0}, // # PGEN_LINE_PD0
	{0xB2, 0x67, 0x0}, //
	{0xB1, 0x0E, 0x0}, // # PGEN_VBP
	{0xB2, 0x21, 0x0}, //
	{0xB1, 0x0F, 0x0}, // # PGEN_VFP
	{0xB2, 0x0A, 0x0}, //
	{},
};

static ssize_t tis_fpdlink_deser_debug_read(struct file *fp,
				char __user *ubuf, size_t cnt, loff_t *ppos)
{
	struct tis_fpdlink *priv = file_inode(fp)->i_private;
		char *buf;
	ssize_t st;
	int err;
	u32 val;

	if (*ppos)
		return 0;

	buf = kzalloc(cnt, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	err = regmap_read(priv->deser_regmap, priv->debug_addr, &val);
	if (err)
		pr_err("%s:i2c read failed, %x = %x\n",
			__func__, 0x4, val);
	if (!err)
	{
		snprintf(buf, cnt, "0x%x\n", val);
		st = copy_to_user(ubuf, buf, cnt);
		cnt = strlen(buf);
	}
	kfree(buf);
	if (st)
		return st;
	return cnt;
}

static ssize_t tis_fpdlink_deser_debug_write(struct file *fp,
				const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	struct tis_fpdlink *priv = file_inode(fp)->i_private;
		char *buf;
	ssize_t st;
	int err;

	buf = kzalloc(cnt, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	st = copy_from_user(buf, ubuf, cnt);
	if (!st)
	{
		unsigned int val;
		sscanf(buf, "0x%x", &val);
		err = regmap_write(priv->deser_regmap, priv->debug_addr, val);
		if (err)
			pr_err("%s:i2c write failed, %x = %x\n",
				__func__, priv->debug_addr, val);
	}
	kfree(buf);
	if (err)
		return err;

	return cnt;
}

static ssize_t tis_fpdlink_ser_debug_read(struct file *fp,
				char __user *ubuf, size_t cnt, loff_t *ppos)
{
	struct tis_fpdlink *priv = file_inode(fp)->i_private;
		char *buf;
	ssize_t st;
	int err;
	unsigned int val;

	if (*ppos)
		return 0;

	buf = kzalloc(cnt, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	err = regmap_read(priv->ser_regmap, priv->debug_addr, &val);
	if (err)
		pr_err("%s:i2c read failed, %x = %x\n",
			__func__, 0x4, val);
	if (!err)
	{
		snprintf(buf, cnt, "0x%x\n", val);
		st = copy_to_user(ubuf, buf, cnt);
		cnt = strlen(buf);
	}
	kfree(buf);
	if (st)
		return st;
	return cnt;
}

static ssize_t tis_fpdlink_ser_debug_write(struct file *fp,
				const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	struct tis_fpdlink *priv = file_inode(fp)->i_private;
		char *buf;
	ssize_t st;
	int err;

	buf = kzalloc(cnt, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	st = copy_from_user(buf, ubuf, cnt);
	if (!st)
	{
		unsigned int val;
		sscanf(buf, "0x%x", &val);
		err = regmap_write(priv->ser_regmap, priv->debug_addr, val);
		if (err)
			pr_err("%s:i2c write failed, %x = %x\n",
				__func__, priv->debug_addr, val);
	}
	kfree(buf);
	if (err)
		return err;

	return cnt;
}

static struct file_operations tis_fpdlink_deser_data_ops = {
	.owner = THIS_MODULE,
	.read = tis_fpdlink_deser_debug_read,
	.write = tis_fpdlink_deser_debug_write,
};

static struct file_operations tis_fpdlink_ser_data_ops = {
	.owner = THIS_MODULE,
	.read = tis_fpdlink_ser_debug_read,
	.write = tis_fpdlink_ser_debug_write,
};

static struct i2c_board_info ds90ub953_serializer_info = {
	I2C_BOARD_INFO("ds90ub953", 0x18),
};


static struct dentry *tis_fpdlink_debug_create(unsigned short addr, struct tis_fpdlink *priv)
{
	struct dentry *fpdlink_dir;
	struct dentry *retval;
	char dirname[32];

	snprintf(dirname, sizeof(dirname), "tis-fpdlink-%x", addr);

	fpdlink_dir = debugfs_create_dir(dirname, NULL);
	if (!fpdlink_dir)
	{
		pr_err("%s: failed to create debug dir\n", __func__);
		return NULL;
	}

	retval = debugfs_create_x8("addr", S_IRUGO | S_IWUGO, fpdlink_dir, &priv->debug_addr);
	if (!retval)
		goto error_out;

	priv->deser_d = debugfs_create_file_size("data-deser", S_IRUGO | S_IWUGO, fpdlink_dir,
		priv, &tis_fpdlink_deser_data_ops, strlen("0x00\n")+1);
	if (!priv->deser_d)
		goto error_out;

	priv->ser_d = debugfs_create_file_size("data-ser", S_IRUGO | S_IWUGO, fpdlink_dir,
		priv, &tis_fpdlink_ser_data_ops, strlen("0x00\n")+1);
	if (!priv->ser_d)
		goto error_out;

	return fpdlink_dir;

	error_out:
	debugfs_remove_recursive(fpdlink_dir);
	return NULL;
}

static int tis_fpdlink_parse_dt(struct i2c_client *client, struct tis_fpdlink *priv)
{
	struct device_node *node = client->dev.of_node;
	const struct of_device_id *match;
	int err;

	match = of_match_device(tis_fpdlink_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return -EINVAL;
	}

	err = of_property_read_u8(node, "clkout_ctrl1", &priv->clkout_ctrl1);
	if (err < 0)
	{
		dev_err(&client->dev, "Failed to read clkout_ctrl1\n");
		return err;
	}

	err = of_property_read_u8(node, "clkout_ctrl2", &priv->clkout_ctrl2);
	if (err < 0)
	{
		dev_err(&client->dev, "Failed to read clkout_ctrl2\n");
		return err;
	}

	err = of_property_read_u8(node, "slave_addr", &priv->slave_addr);
	if (err < 0)
	{
		dev_err(&client->dev, "Failed to read slave_addr\n");
		return err;
	}

	err = of_property_read_u8(node, "slave_map_addr", &priv->slave_map_addr);
	if (err < 0)
	{
		dev_err(&client->dev, "Failed to read slave_map_addr\n");
		return err;
	}

	err = of_property_read_u8(node, "ser_map_addr", &priv->ser_map_addr);
	if (err < 0)
	{
		dev_err(&client->dev, "Failed to read ser_map_addr\n");
		return err;
	}

	return 0;
}


static int parse_additional_config(struct regmap *dev_regmap, struct device_node *node, const char* propname)
{
	int datalen;
	u8 *data;
	int ret;
	int i;

	if (!of_get_property(node, propname, &datalen)) {
		return -ENOENT;
	}

	data = kmalloc(datalen, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = of_property_read_u8_array(node, propname, data, datalen);
	if (ret) {
		pr_err("%s:dt property read failed: %s\n",
			__func__, propname);
		kfree(data);
		return ret;
	}

	if (datalen & 1){
		pr_err("%s: length of property '%s' should be even\n",
			__func__, propname);
	}

	for( i = 0; i < datalen; i+= 2) {
		pr_info("%s: write %x = %x\n", __func__, data[i], data[i+1]);
		ret = regmap_write(dev_regmap, data[i], data[i+1]);
		if (ret)
			pr_err("%s:i2c write failed, %x = %x\n",
				__func__, data[i], data[i+1]);
	}

	kfree(data);
	return 0;
}

static int tis_fpdlink_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct tis_fpdlink *priv;
	const struct fpdlink_reg *	reg;
	struct i2c_board_info ser_board_info;
	int wait_count = 10;
	int ret;

	dev_info(&client->dev, "Probing TIS_FPDLink\n");

	priv = devm_kzalloc(&client->dev,
			sizeof(struct tis_fpdlink),
			GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	ret = tis_fpdlink_parse_dt(client, priv);
	if (ret)
		return ret;

	priv->deser_regmap = devm_regmap_init_i2c(client, &tis_fpdlink_regmap_config);
	if (IS_ERR(priv->deser_regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->deser_regmap));
		return -ENODEV;
	}

	i2c_set_clientdata(client, priv);

	dev_info(&client->dev, "Initializing DESERIALIZER\n");
	for (reg = fpdlink_deser_reglist; reg->addr != 0; reg++) {
		int err;
		dev_info(&client->dev, "Writing deser: 0x%x = 0x%x\n", reg->addr, reg->val);
		err = regmap_write(priv->deser_regmap, reg->addr, reg->val);
		if (err)
			pr_err("%s:i2c write failed, %x = %x\n",
				__func__, reg->addr, reg->val);
		if (reg->flags & TIS_FPDLINK_FLAG_VERIFY)
		{
			int retry_count;
			for (retry_count = 0; retry_count < 5; retry_count++)
			{
				unsigned int val;
				int err;
				err = regmap_read(priv->deser_regmap, reg->addr, &val);
				if (err)
					pr_err("%s:i2c read failed, %x = %x\n",
						__func__, 0x4, val);
				if (val != reg->val){
					err = regmap_write(priv->deser_regmap, reg->addr, reg->val);
					if (err)
						pr_err("%s:i2c write failed, %x = %x\n",
							__func__, reg->addr, reg->val);

				} else {
					break;
				}
				usleep_range(30000, 31000);
			}
			if (retry_count == 5)
			{
				dev_info(&client->dev, "Repeatedly failed to set register!\n");
			}
		}
	}

	// Set up serializer I2C address mapping
	dev_info(&client->dev, "Mapping serializer to 0x%x\n", priv->ser_map_addr << 1);
	regmap_write(priv->deser_regmap, 0x5c, priv->ser_map_addr << 1);
	dev_info(&client->dev, "Mapping slave 0x%x to 0x%x\n", priv->slave_addr << 1, priv->slave_map_addr << 1);
	regmap_write(priv->deser_regmap, 0x5d, priv->slave_addr << 1);
	regmap_write(priv->deser_regmap, 0x65, priv->slave_map_addr << 1);
	// {0x5d, 0x20, 0x0}, // SlaveID[0],
	// {0x65, 0x20, 0x0}, // SlaveAlias[0],


	dev_info(&client->dev, "Waiting for link\n");
	for( ; wait_count >= 0; wait_count--)
	{
		unsigned int val;
		int err;
		usleep_range(100000, 200000);
		err = regmap_read(priv->deser_regmap, 0x04, &val);
		if (err)
			pr_err("%s:i2c read failed, %x = %x\n",
				__func__, 0x4, val);
		if ( val & (1<<2))
		{
			dev_info(&client->dev, "Got LOCK\n");
			break;
		}
	}

	memcpy(&ser_board_info, &ds90ub953_serializer_info, sizeof(struct i2c_board_info));
	ser_board_info.addr = priv->ser_map_addr;

	priv->ser_client = i2c_new_device(client->adapter, &ser_board_info);
	if (!priv->ser_client)
	{
		dev_err(&client->dev, "Instantiating of serializer failed\n");
		return -ENODEV;
	}

	priv->ser_regmap = devm_regmap_init_i2c(priv->ser_client, &tis_fpdlink_regmap_config);
	if (IS_ERR(priv->ser_regmap)) {
		dev_err(&priv->ser_client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->ser_regmap));
		return -ENODEV;
	}

	dev_info(&client->dev, "Initializing SERIALIZER\n");

	// Set CLKOUT config
	dev_info(&client->dev, "CLKOUT_CONFIG: 0x6 = 0x%x  0x7 = 0x%x\n", priv->clkout_ctrl1, priv->clkout_ctrl2);
	ret = regmap_write(priv->ser_regmap, 0x06, priv->clkout_ctrl1);
	if (ret)
		pr_err("%s:i2c write failed, %x = %x\n",
			__func__, 0x06, priv->clkout_ctrl1);
	ret = regmap_write(priv->ser_regmap, 0x07, priv->clkout_ctrl2);
	if (ret)
		pr_err("%s:i2c write failed, %x = %x\n",
			__func__, 0x07, priv->clkout_ctrl2);


	for (reg = fpdlink_ser_reglist; reg->addr != 0; reg++) {
		int err;
		dev_info(&client->dev, "Writing ser: 0x%x = 0x%x\n", reg->addr, reg->val);
		err = regmap_write(priv->ser_regmap, reg->addr, reg->val);
		if (err)
			pr_err("%s:i2c write failed, %x = %x\n",
				__func__, reg->addr, reg->val);
	}

	parse_additional_config(priv->ser_regmap, client->dev.of_node, "ser_i2c_setup");

	// for (reg = fpdlink_ser_patgen; reg->addr != 0; reg++) {
	// 	int err;
	// 	dev_info(&client->dev, "Writing ser: 0x%x = 0x%x\n", reg->addr, reg->val);
	// 	err = regmap_write(priv->ser_regmap, reg->addr, reg->val);
	// 	if (err)
	// 		pr_err("%s:i2c write failed, %x = %x\n",
	// 			__func__, reg->addr, reg->val);
	// }

	priv->fpdlink_dir = tis_fpdlink_debug_create(client->addr, priv);

	return 0;
}


static int tis_fpdlink_remove(struct i2c_client *client)
{
	struct tis_fpdlink *priv;

	dev_info(&client->dev, "Removing TIS_FPDLink\n");

	priv = (struct tis_fpdlink *)i2c_get_clientdata(client);

	if (priv->fpdlink_dir)
		debugfs_remove_recursive(priv->fpdlink_dir);

	i2c_unregister_device(priv->ser_client);

	return 0;
}



static const struct i2c_device_id tis_fpdlink_id[] = {
	{ "tis-fpdlink", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tis_fpdlink_id);

/*
* I2C related structure
*/
static struct i2c_driver tis_fpdlink_i2c_driver = {
	.driver = {
		.name = "tis-fpdlink",
		.owner = THIS_MODULE,
		.of_match_table = tis_fpdlink_of_match,
	},
	.probe = tis_fpdlink_probe,
	.remove = tis_fpdlink_remove,
	.id_table = tis_fpdlink_id,
};

module_i2c_driver(tis_fpdlink_i2c_driver);

MODULE_DESCRIPTION("FDP Link driver for The Imaging Source FPD Link camera connector");
MODULE_AUTHOR("The Imaging Source Europe GmbH");
MODULE_LICENSE("GPL v2");