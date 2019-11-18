
// 0----------------------------------------------------------------------------------------------------------

#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
   /* this no need register board information ! */
#else
static struct spi_board_info dm9051_spi_board_devs[] __initdata = {
	[0] = {
	.modalias = "faraday,dm9051",
	.max_speed_hz = DRV_MAX_SPEED_HZ,
	.bus_num = DM_CONF_SPI_BUS_NUMBER, //0,
	.chip_select = DM_CONF_SPI_CHIP_SELECT, //0,
	.mode = SPI_MODE_3,
#if DRV_INTERRUPT_1	  
	.irq = DM_CONF_INTERRUPT_IRQ, //3,
#endif		
	},
};
#endif


#if DMA3_P6_DRVSTATIC
 /* Use "spi_register_board_info(dm9051_spi_board_devs, ARRAY_SIZE(dm9051_spi_board_devs));" directly */
#else
 /* Use "dm9051_spi_register_board_info(dm9051_spi_board_devs, ARRAY_SIZE(dm9051_spi_board_devs));" dynamically */
#endif

#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
   /* this no need register board information ! */
#else
#if DMA3_P6_DRVSTATIC
 /* Joseph 20151030 */
 extern int spi_register_board_info(struct spi_board_info const *info, unsigned n);
#else
 /* Joseph: find/delete/new */
 static unsigned verbose = 3;
 module_param(verbose, uint, 0);
 MODULE_PARM_DESC(verbose,
 "0 silent, >0 show gpios, >1 show devices, >2 show devices before (default=3)");

 static struct spi_device *spi_device;

 static void dm9051_device_spi_delete(struct spi_master *master, unsigned cs)
 {
	struct device *dev;
	char str[32];

	snprintf(str, sizeof(str), "%s.%u", dev_name(&master->dev), cs);

	dev = bus_find_device_by_name(&spi_bus_type, NULL, str);
	if (dev) {
		if (verbose)
			pr_info(DRVNAME_9051": Deleting %s\n", str);
		device_del(dev);
	}
 }
 static int dm9051_spi_register_board_info(struct spi_board_info *spi, unsigned n)
 {
      /* Joseph_20151030: 'n' is always const is 1, in this design */

	struct spi_master *master;

	master = spi_busnum_to_master(spi->bus_num);
	if (!master) {
		pr_err(DRVNAME_9051 ":  spi_busnum_to_master(%d) returned NULL\n",
								spi->bus_num);
		return -EINVAL;
	}
	/* make sure it's available */
	dm9051_device_spi_delete(master, spi->chip_select);
	spi_device = spi_new_device(master, spi);
	put_device(&master->dev);
	if (!spi_device) {
		pr_err(DRVNAME_9051 ":    spi_new_device() returned NULL\n");
		return -EPERM;
	}
	return 0;
 }

 static void dm9051_spi_unregister_board(void)
 {
	//----------------------
	//[#ifdef MODULE #endif]
	//----------------------
	if (spi_device) {
		device_del(&spi_device->dev);
		kfree(spi_device);
	}
 }
#endif
#endif

static
void conf_spi_print(void)
{
#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
	/* DTS, this speed setting is in the dts file, not in the driver ! */
#else
	printk("[ *dm9051  ] CONFIG SPI speed[= %d]\n", dm9051_spi_board_devs[0].max_speed_hz);
#endif	
}

void conf_spi_board(void)
{
/* ------------------------------------------------------------------------------------------- */
/* #if DM_DM_CONF_INSTEAD_OF_DTS_AND_BE_DRVMODULE                                              */
/*   dm9051_spi_register_board_info(dm9051_spi_board_devs, ARRAY_SIZE(dm9051_spi_board_devs)); */
/* #else                                                                                       */
/* #endif                                                                                      */
/* ------------------------------------------------------------------------------------------- */

  #if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
   /* this no need register board information ! */
  #else
    #if DMA3_P6_DRVSTATIC
    //  spi_register_board_info(dm9051_spi_board_devs, ARRAY_SIZE(dm9051_spi_board_devs));
    #else
      dm9051_spi_register_board_info(dm9051_spi_board_devs, ARRAY_SIZE(dm9051_spi_board_devs));
    #endif
  #endif
}

void unconf_spi_board(void)
{
/* ---------------------------------------------- */
/* #if DM_DM_CONF_INSTEAD_OF_DTS_AND_BE_DRVMODULE */
/*     dm9051_spi_unregister_board();             */
/* #else                                          */
/* #endif                                         */
/* ---------------------------------------------- */

  #if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
   /* this no need register board information ! */
  #else
    #if DMA3_P6_DRVSTATIC
    #else
      dm9051_spi_unregister_board();
    #endif
  #endif
}