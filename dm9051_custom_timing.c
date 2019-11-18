/* ---------------------------------------------- */
/* #if DM_DM_CONF_INSTEAD_OF_DTS_AND_BE_DRVMODULE */
/* #undef DMA3_P1_MTKGPIO                         */
/* #define DMA3_P1_MTKGPIO 0                      */
/* #undef DMA3_P1_MTKSETUP                        */
/* #define DMA3_P1_MTKSETUP 0                     */
/* #endif                                         */
/* ---------------------------------------------- */

void SPI_PARAM_Set(struct board_info *db)
{
#if DMA3_P1_MTKSETUP
        struct mt_chip_conf *spi_par = (struct mt_chip_conf *) db->spidev->controller_data; // added 2015-12-3
		if(!spi_par){
			printk("[dm95_spi] spi config fail");
			return;
		}
		spi_par->setuptime = 15; 
		spi_par->holdtime = 15; 
		spi_par->high_time = 10;       //10--6m   15--4m   20--3m  30--2m  [ 60--1m 120--0.5m  300--0.2m]
		spi_par->low_time = 10;
		spi_par->cs_idletime = 20; 

		spi_par->rx_mlsb = 1; 
		spi_par->tx_mlsb = 1;		 
		spi_par->tx_endian = 0;
		spi_par->rx_endian = 0;

		spi_par->cpol = 0;
		spi_par->cpha = 0;
 #if DMA3_P2_MSEL_MOD
		spi_par->com_mod = DMA_TRANSFER;
 #else
		spi_par->com_mod = FIFO_TRANSFER;
 #endif

		spi_par->pause = 0;
		spi_par->finish_intr = 1;
		spi_par->deassert = 0;
#endif
}
void SPI_SPI_Setup(struct board_info *db) //(struct spi_device *spi)
{    
		SPI_PARAM_Set(db);

#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
		/* While DTS modle, Define spi max speed in the DTS file */
#else
		db->spidev->max_speed_hz= dm9051_spi_board_devs[0].max_speed_hz;	
#endif
		db->spidev->mode = SPI_MODE_3;
		db->spidev->bits_per_word = 8;
		
		if(spi_setup(db->spidev)){
			printk("[dm95_spi] spi_setup fail\n");
			return;
		}
}