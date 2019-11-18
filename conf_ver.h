 /* [configuration definition] */

//(0) Optional configuration: 
//#define DM_CONF_VSPI //(Note: virtual spi interface)
//(1) Optional configuration: 
//#define DM_CONF_MAX_SPEED_HZ	(50 * 1000 *1000)  //(chnage spi speed to 50MHz)
//#define DM_CONF_INTERRUPT //(Note: Not available for be less efficient!)
//#define DM_CONF_INTERRUPT_IRQ	3 //(Note: When available, Set IRQ number by this const definition!)
//#define DM_CONF_MODULE //{change to kernel driver module}
//(2) Optional configuration:
//#define MTK_CONF_YES
//#define MTK_CONF_SPI_DMA_YES
//(3) while configure to interrupt mode, It's recommand to add DM_CONF_PHYPOLL
//#define DM_CONF_PHYPOLL

 /* [configuration code below] */

#define DM_CONF_MAX_SPEED_HZ	20000000  //31200000 	//(62 * 1000 * 1000 + 500 * 1000) = 62500000, Fail
						//(33 * 1000 * 1000 + 000 * 1000) = 33000000, Fail
						//(31 * 1000 * 1000 + 200 * 1000) = 31200000, OK
						//(27 * 1000 *1000) //(20 * 1000 *1000) 
#define DM_CONF_SPI_BUS_NUMBER	0
#define DM_CONF_SPI_CHIP_SELECT	0
//#define DM_CONF_MODULE
//#define DM_CONF_INTERRUPT
//#define DM_CONF_INTERRUPT_IRQ	80
//#define MTK_CONF_YES
//#define MTK_CONF_SPI_DMA_YES
#define DM_CONF_MODULE
#define DM_CONF_PHYPOLL
