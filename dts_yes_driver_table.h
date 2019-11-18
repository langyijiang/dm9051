/* 3p6s.s */
asmlinkage __visible int printkr(const char *fmt, ...){
  return 0; 
}
EXPORT_SYMBOL(printkr);
/* 3p6s.e */

#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE || 1
// || 1, Definitety Allow 
static struct of_device_id dm9051_match_table[] = {
	{
		.compatible = "davicom,dm9051",
	},
	{}
};
#endif

struct spi_device_id dm9051_spi_id_table = {"faraday,dm9051", 0}; //DRVNAME_9051
static int dm9051_probe(struct spi_device *spi);
static int dm9000_drv_remove(struct spi_device *spi);
#if DMA3_P4_KT
static int dm9000_drv_suspend(struct spi_device *spi, pm_message_t state);
static int dm9000_drv_resume(struct spi_device *spi);
#endif

static struct spi_driver dm9051_driver = {
	.driver	= {
		.name  = DRVNAME_9051, //"dm9051"
		.owner = THIS_MODULE,
        #if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE || 1
        // || 1, Definitety Allow 
        .of_match_table = dm9051_match_table,
        .bus = &spi_bus_type,
        #endif
	},
	.probe   = dm9051_probe,
	.remove  = /*__devexit_p*/(dm9000_drv_remove),
	.id_table = &dm9051_spi_id_table,
#if DMA3_P4_KT
/*3p*/	.suspend = dm9000_drv_suspend,
/*3p*/	.resume = dm9000_drv_resume,
#endif
};
