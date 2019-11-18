/* ---------------------------------------------- */
/* #if DM_DM_CONF_INSTEAD_OF_DTS_AND_BE_DRVMODULE */
/*  #undef DMA3_P0_MTKHDR                         */
/*  #define DMA3_P0_MTKHDR 0                      */
/* #endif                                         */
/* ---------------------------------------------- */

#if DMA3_P0_MTKHDR
#include <linux/cache.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/cache.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
 
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#endif

/* sxtechs */
#if DMA3_P0_MTKHDR
#include <mach/gpio_const.h>
//#include "../../../../drivers/spi/mediatek/mt6735/mt_spi.h"
#include "../../../spi/mediatek/mt6735/mt_spi.h"
#include "../../../spi/mediatek/mt6735/mt_spi.h"
//#include "../../../spi/mediatek/mt6755/mt_spi.h"
//#include "../../../spi/mediatek/mt6755/mt_spi.h"
//#include <linux/ctldbg.h>
#include <mt-plat/mt_gpio.h>
#endif

/*3p*/
#if DMA3_P0_MTKHDR
#if 0
//#include <mach/mt_spi.h>
#include <mach/mt_gpio.h>
#include "cust_gpio_usage.h" 
#include <linux/gpio.h>
#include <mach/eint.h>
#include <cust_eint.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#endif
#endif