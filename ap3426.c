#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>

/***************************************************************

Faile Name  : ap3426.c

Descriptionz     : AP3426 Linux Driver

***************************************************************/

#define AP3426_CNT 1

#define AP3426_NAME "ap3426"


#define AP3426_ADDR     0X1E /* AP3426 I2C Address   */

/* AP3426 Register */

#define AP3426_SYSTEMCONG 0x00 /* System Control     */

#define AP3426_INTSTATUS 0X01 /* Interrupt Flag      */

#define AP3426_INTCLEAR 0X02 /* INT Control          */

#define AP3426_WAITITME 0X06 /* Wait time            */

#define AP3426_IRDATALOW 0x0A /* IR Data Low          */

#define AP3426_IRDATAHIGH 0x0B /* IR Data High        */

#define AP3426_ALSDATALOW 0x0C /* ALS Data Low        */

#define AP3426_ALSDATAHIGH 0X0D /* ALS Data High       */

#define AP3426_PSDATALOW 0X0E /* PS Data Low          */

#define AP3426_PSDATAHIGH 0X0F /* PS Data High         */


 

struct ap3426_dev 

{

 dev_t devid;   

 struct cdev cdev;  

 struct class *class; 

 struct device *device; 

 struct device_node *nd; 

 int major;   

 void *private_data; 

 unsigned short ir, als, ps;  

};

 

static struct ap3426_dev ap3426dev;



static int ap3426_read_regs(struct ap3426_dev *dev, u8 reg, void *val, int len)

{

 int ret;

 struct i2c_msg msg[2];

 struct i2c_client *client = (struct i2c_client *)dev->private_data;

 

 msg[0].addr = client->addr;   

 msg[0].flags = 0;     

 msg[0].buf = &reg;    

 msg[0].len = 1;      



 msg[1].addr = client->addr;   

 msg[1].flags = I2C_M_RD;   

 msg[1].buf = val;     

 msg[1].len = len;     

 

 ret = i2c_transfer(client->adapter, msg, 2);

 if(ret == 2) {

  ret = 0;

 } else {

  printk("i2c rd failed=%d reg=%06x len=%d\n",ret, reg, len);

  ret = -EREMOTEIO;

 }

 return ret;

}



static s32 ap3426_write_regs(struct ap3426_dev *dev, u8 reg, u8 *buf, u8 len)

{

 u8 b[256];

 struct i2c_msg msg;

 struct i2c_client *client = (struct i2c_client *)dev->private_data;

 

 b[0] = reg;     

 memcpy(&b[1],buf,len);  

  

 msg.addr = client->addr; 

 msg.flags = 0;    

 

 msg.buf = b;    

 msg.len = len + 1;   

 

 return i2c_transfer(client->adapter, &msg, 1);

}

 

static unsigned char ap3426_read_reg(struct ap3426_dev *dev, u8 reg)

{

 u8 data = 0;

 

 ap3426_read_regs(dev, reg, &data, 1);

 return data;

 

#if 0

 struct i2c_client *client = (struct i2c_client *)dev->private_data;

 return i2c_smbus_read_byte_data(client, reg);

#endif

}



static void ap3426_write_reg(struct ap3426_dev *dev, u8 reg, u8 data)

{

 u8 buf = 0;

 buf = data;

 ap3426_write_regs(dev, reg, &buf, 1);

}



void ap3426_readdata(struct ap3426_dev *dev)

{

    unsigned char i =0;

    unsigned char buf[6];

 

    for(i = 0; i < 6; i++) 

    {

        buf[i] = ap3426_read_reg(dev, AP3426_IRDATALOW + i); 

    }

 

    if(buf[0] & 0X80)  

  dev->ir = 0;     

  else     

  dev->ir = ((unsigned short)buf[1] << 2) | (buf[0] & 0X03);    

 

 dev->als = ((unsigned short)buf[3] << 8) | buf[2];   

 

    if(buf[4] & 0x40) 

  dev->ps = 0;                 

 else     

  dev->ps = ((unsigned short)(buf[5] & 0X3F) << 4) | (buf[4] & 0X0F); 

}

 

static int ap3426_open(struct inode *inode, struct file *filp)

{

 filp->private_data = &ap3426dev;

 

 ap3426_write_reg(&ap3426dev, AP3426_SYSTEMCONG, 0x04);  

 mdelay(50);              

 ap3426_write_reg(&ap3426dev, AP3426_SYSTEMCONG, 0X03);  

 return 0;

}



static ssize_t ap3426_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)

{

 short data[3];

 long err = 0;

 

 struct ap3426_dev *dev = (struct ap3426_dev *)filp->private_data;

 

 ap3426_readdata(dev);

 

 data[0] = dev->ir;

 data[1] = dev->als;

 data[2] = dev->ps;

 err = copy_to_user(buf, data, sizeof(data));

 return 0;

}

 

static int ap3426_release(struct inode *inode, struct file *filp)

{

 return 0;

}

 

static const struct file_operations ap3426_ops = {

 .owner = THIS_MODULE,

 .open = ap3426_open,

 .read = ap3426_read,

 .release = ap3426_release,

};

 

static int ap3426_probe(struct i2c_client *client, const struct i2c_device_id *id)

{

 

 if (ap3426dev.major) {

  ap3426dev.devid = MKDEV(ap3426dev.major, 0);

  register_chrdev_region(ap3426dev.devid, AP3426_CNT, AP3426_NAME);

 } else {

  alloc_chrdev_region(&ap3426dev.devid, 0, AP3426_CNT, AP3426_NAME);

  ap3426dev.major = MAJOR(ap3426dev.devid);

 }

 

 cdev_init(&ap3426dev.cdev, &ap3426_ops);

 cdev_add(&ap3426dev.cdev, ap3426dev.devid, AP3426_CNT);

 

 ap3426dev.class = class_create(THIS_MODULE, AP3426_NAME);

 if (IS_ERR(ap3426dev.class)) 

 {

  return PTR_ERR(ap3426dev.class);

 }

 

 ap3426dev.device = device_create(ap3426dev.class, NULL, ap3426dev.devid, NULL, AP3426_NAME);

 if (IS_ERR(ap3426dev.device)) 

 {

  return PTR_ERR(ap3426dev.device);

 }

 

 ap3426dev.private_data = client;

 

 return 0;

}

 

static int ap3426_remove(struct i2c_client *client)

{

 cdev_del(&ap3426dev.cdev);

 unregister_chrdev_region(ap3426dev.devid, AP3426_CNT);

 

 device_destroy(ap3426dev.class, ap3426dev.devid);

 class_destroy(ap3426dev.class);

 return 0;

}



static const struct i2c_device_id ap3426_id[] = {

 {"dyna,ap3426", 0},  

 {}

};

 

static const struct of_device_id ap3426_of_match[] = {

 { .compatible = "dyna,ap3426" },

 { /* Sentinel */ }

};

 

static struct i2c_driver ap3426_driver = {

 .probe = ap3426_probe,

 .remove = ap3426_remove,

 .driver = {

   .owner = THIS_MODULE,

      .name = "ap3426",

      .of_match_table = ap3426_of_match, 

     },

 .id_table = ap3426_id,

};



static int __init ap3426_init(void)

{

 int ret = 0;

 

 ret = i2c_add_driver(&ap3426_driver);

 return ret;

}



static void __exit ap3426_exit(void)

{

 i2c_del_driver(&ap3426_driver);

}



module_init(ap3426_init);

module_exit(ap3426_exit);

MODULE_LICENSE("GPL");

MODULE_AUTHOR("Brian Chiu<brian.chiu@dyna-image.com>");

