/* ----------------------------------------------------------------------------- */
/* ----------------------------------------------                                */
/* *** We like no-limitation "inblk & outblk" ***                                */
/* ----------------------------------------------                                */
/* #undef DMA3_P2_RSEL_1024F                                                     */
/* #undef DMA3_P2_RSEL_32F                                                       */
/* #undef DMA3_P2_RSEL_1F                                                        */
/* #undef DMA3_P2_TSEL_1024F                                                     */
/* #undef DMA3_P2_TSEL_32F                                                       */
/* #undef DMA3_P2_TSEL_1F                                                        */
/* #define DMA3_P2_RSEL_1024F	0 // RD_MACH: FIFO model (1024 bytes-limitation) */
/* #define DMA3_P2_RSEL_32F	0 // RD_MACH: FIFO model (32 bytes-limitation)       */
/* #define DMA3_P2_RSEL_1F	0 // RD_MACH: FIFO model (1 byte-limitation)         */
/* #define DMA3_P2_TSEL_1024F	0 // TX_MACH: FIFO model (1024 bytes-limitation) */
/* #define DMA3_P2_TSEL_32F	0 // TX_MACH: FIFO model (32 bytes-limitation)       */
/* #define DMA3_P2_TSEL_1F	0 // TX_MACH: FIFO model (1 byte-limitation)         */
/* ----------------------------------------------------------------------------- */

static int SubNetwork_SPI_Init(struct board_info *db, int enable)
{
	mutex_lock(&db->addr_lock);
	if(enable){		
        SPI_GPIO_Setup(db); //mt_dm9051_pinctrl_init(db->spidev); //or, SPI_GPIO_Set(1);
        SPI_SPI_Setup(db);
	}
	mutex_unlock(&db->addr_lock);
	return 0;
}

void dwrite_1024_Limitation(board_info_t *db, u8 *txb, u8 *trxb, int len);
void dwrite_32_Limitation(board_info_t *db, u8 *txb, u8 *trxb, int len);
void dwrite_1_Limitation(board_info_t *db, u8 *txb, u8 *trxb, int len);

static int Custom_SPI_Write(board_info_t *db, u8 *buff, unsigned len) 
{
#if DMA3_P2_TSEL_1024F
	dwrite_1024_Limitation(db, buff, NULL, len);
	return 1;
#elif DMA3_P2_TSEL_32F
    memcpy(&db->TxDatBuf[1], buff, len);
    dwrite_32_Limitation(db, db->TxDatBuf, NULL, len);
	return 1;
#elif DMA3_P2_TSEL_1F
	memcpy(&db->TxDatBuf[1], buff, len);
	dwrite_1_Limitation(db, db->TxDatBuf, NULL, len);
	return 1;
#else
	return 0;
#endif
}

void dread_1024_Limitation(board_info_t *db, u8 *trxb, int len);
void dread_32_Limitation(board_info_t *db, u8 *trxb, int len);
void dread_1_Limitation(board_info_t *db, u8 *trxb, int len);

static int Custom_SPI_Read(board_info_t *db, u8 *buff, unsigned len)
{	
#if DMA3_P2_RSEL_1024F
	dread_1024_Limitation(db, buff, (int)len);
	return 1;
#elif DMA3_P2_RSEL_32F
	dread_32_Limitation(db, buff, (int)len);
	return 1;
#elif DMA3_P2_RSEL_1F
	dread_1_Limitation(db, buff, (int)len);
	return 1;
#else
	return 0;
#endif
}

static int INNODev_sync(board_info_t *db)
{
	int ret;
	
#ifdef DM_CONF_VSPI
	return 0;
#endif	
	mutex_lock(&db->sublcd_mtkspi_mutex);
	
	ret= spi_sync(db->spidev, &db->spi_dmsg1);
		
	mutex_unlock(&db->sublcd_mtkspi_mutex);
	if(ret){
		printk("[dm95_spi] spi.sync fail ret= %d, should check", ret);
		if(ret == -ETIME){
		}
	}		 
	return ret;
}

/* routines for sending block to chip */
void dwrite_1024_Limitation(board_info_t *db, u8 *txb, u8 *trxb, int len)
{
	int blkLen;
	struct spi_transfer *xfer;
	int const pkt_count = (len + 1)/ CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	int const remainder = (len + 1)% CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	
	if((len + 1)>CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES){
		//(1)
		blkLen= CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count - 1;
		printk("9Tx tbf=0x%p,rbf=0x%p)\n", db->tmpTxPtr, db->tmpRxPtr);

	    wbuff_u8(DM_SPI_WR | DM_SPI_MWCMD, db->tmpTxPtr); //'RD_LEN_ONE'
		xfer= &db->spi_dxfer1;

#if 1
      //memcpy(db->tmpTxPtr, txb, RD_LEN_ONE + blkLen);
        memcpy(db->tmpTxPtr+1, txb, blkLen);
		xfer->tx_buf = db->tmpTxPtr; //txb;
		xfer->rx_buf = db->tmpRxPtr; //NULL; //tmpRxPtr; //trxb; ((( When DMA 'NULL' is not good~~~
#else
#endif

		xfer->len = RD_LEN_ONE + blkLen; // minus 1, so real all is 1024 * n
		spi_message_init(&db->spi_dmsg1);

		spi_message_add_tail(&db->spi_dxfer1, &db->spi_dmsg1);
		if(INNODev_sync(db))
			printk("[dm95_spi]INNO txERR1: len= %d of %d, txbuf=0x%p,rxbuf=0x%p",blkLen,len,xfer->tx_buf,xfer->rx_buf); //return INNO_GENERAL_ERROR;

		//(2)	
		blkLen= remainder;
	  //printk("   9Tx_Rem(%d ... ", remainder); printk("tbf=0x%p,rbf=0x%p)", db->tmpTxPtr, db->tmpRxPtr); printk("\n");
		
		xfer= &db->spi_dxfer1;
#if 1
        memcpy(db->tmpTxPtr+1, &txb[CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count - 1], remainder);
		xfer->tx_buf = db->tmpTxPtr; //&txb[CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count - 1]; // has been minus 1
		xfer->rx_buf = db->tmpRxPtr; //NULL; //tmpRxPtr; //trxb; (((  'NULL' is not good~~~
#else
#endif

		xfer->len = RD_LEN_ONE + remainder; // when calc, it plus 1
		spi_message_init(&db->spi_dmsg1);

		spi_message_add_tail(&db->spi_dxfer1, &db->spi_dmsg1);
		if(INNODev_sync(db))
			printk("[dm95_spi]INNO txERR2: len=%d of %d, txbuf=0x%p,rxbuf=0x%p",blkLen,len,xfer->tx_buf,xfer->rx_buf); //return INNO_GENERAL_ERROR;
	} else {
	  //printk("   9Tx_Sma_(%d ... ", len); printk("tbf=0x%p,rbf=0x%p)", db->tmpTxPtr, db->tmpRxPtr); printk("\n");
	  //wbuff_u8(DM_SPI_WR | DM_SPI_MWCMD, txb);
		wbuff_u8(DM_SPI_WR | DM_SPI_MWCMD, db->tmpTxPtr);
		spi_message_init(&db->spi_dmsg1);
		xfer= &db->spi_dxfer1;
#if 1
      //memcpy(db->tmpTxPtr, txb, RD_LEN_ONE + len);
        memcpy(db->tmpTxPtr+1, txb, len);
		xfer->tx_buf = db->tmpTxPtr; //txb;
		xfer->rx_buf = db->tmpRxPtr; //NULL; //tmpRxPtr; //trxb; ((( again When DMA 'NULL' is not good~~~
#endif
		xfer->len = RD_LEN_ONE + len;
		spi_message_add_tail(&db->spi_dxfer1, &db->spi_dmsg1);
		if(INNODev_sync(db))
			printk("[dm95_spi]INNO ERROR: len=%d, txbuf=0x%p,rxbuf=0x%p",len,xfer->tx_buf,xfer->rx_buf); //return INNO_GENERAL_ERROR;
	}
}

void dwrite_32_Limitation(board_info_t *db, u8 *txb, u8 *trxb, int len)
{
  int n;
  unsigned counter, not_process;
  
  not_process= len;
  for (n= 0; n< len; n += 31){	// n += 'counter' is the exact calc, But we use n += 31 can get the same looping..	
  		if (not_process <= 31)
  		  counter= not_process;
  		else 
  		  counter= 31;
  		//counter++; // counter= 32; or less
  		//But: //counter= 31; or less
		wbuff_u8(DM_SPI_WR | DM_SPI_MWCMD, txb);
		xrdbuff_u8(db, txb, NULL, counter);
		not_process -= counter;
		txb += counter;
  }
}

void dwrite_1_Limitation(board_info_t *db, u8 *txb, u8 *trxb, int len)
{
  int n;
  for (n=0; n< len; n++){
		wbuff_u8(DM_SPI_WR | DM_SPI_MWCMD, txb);
		xrdbuff_u8(db, txb, NULL, 1);
		txb++;
  }
}

void dread_1024_Limitation(board_info_t *db, u8 *trxb, int len)
{
	struct spi_transfer *xfer;
	u8 txb[1];
	int const pkt_count = (len + 1) / CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	int const remainder = (len + 1) % CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	if((len + 1)>CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES){	
		int blkLen;
		wbuff_u8(DM_SPI_RD | DM_SPI_MRCMD, txb);
		//(1)
		blkLen= CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count - 1;

	    printkr("dm9rx_EvenPar_OvLimit(%d ... ", blkLen);
	    printkr("txbf=0x%p,rxbf=0x%p)\n", db->tmpTxPtr, db->tmpRxPtr);
    
		spi_message_init(&db->spi_dmsg1);
		xfer= &db->spi_dxfer1;
        memcpy(db->tmpTxPtr, txb, 2);
        //memcpy(db->tmpRxPtr, trxb, RD_LEN_ONE + (CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count) - 1);
		xfer->tx_buf = db->tmpTxPtr; //txb;
		xfer->rx_buf = db->tmpRxPtr; //trxb;
		xfer->len = RD_LEN_ONE + (CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count) - 1;  // minus 1, so real all is 1024 * n
		spi_message_add_tail(&db->spi_dxfer1, &db->spi_dmsg1);
		if(INNODev_sync(db/*&db->spi_dmsg1*/))
			printk("[dm95_spi]INNO1 ERROR: len=%d, txbuf=0x%p,rxbuf=0x%p",len,txb,trxb); //return INNO_GENERAL_ERROR;
        memcpy(trxb, db->tmpRxPtr, RD_LEN_ONE + (CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count) - 1);
		//(2)	
		blkLen= remainder;
		printkr("dm9rx_EvenPar_OvRemainder(%d ... ", blkLen);
		printkr("txbf=0x%p,rxbf=0x%p)\n", db->tmpTxPtr, db->tmpRxPtr);

		spi_message_init(&db->spi_dmsg1);
		xfer= &db->spi_dxfer1;
      //memcpy(db->tmpTxPtr, txb, 2);
      //memcpy(db->tmpRxPtr, db->TxDatBuf, RD_LEN_ONE + remainder);
		xfer->tx_buf = db->tmpTxPtr; //txb;
		xfer->rx_buf = db->tmpRxPtr; //db->TxDatBuf;
		xfer->len = RD_LEN_ONE + remainder; // when calc, it plus 1
		spi_message_add_tail(&db->spi_dxfer1, &db->spi_dmsg1);
		if(INNODev_sync(db/*&db->spi_dmsg1*/))
			printk("[dm95_spi]INNO2 ERROR: len=%d, txbuf=0x%p,rxbuf=0x%p",len,txb,xfer->rx_buf); //return INNO_GENERAL_ERROR;

        memcpy(trxb + (CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count), &db->tmpRxPtr[1], remainder);
	}
	else{
		printkr("dm9rx_smal_(%d ... ", len);
		printkr("txbf=0x%p,rxbf=0x%p)\n", db->tmpTxPtr, db->tmpRxPtr);

		wbuff_u8(DM_SPI_RD | DM_SPI_MRCMD, txb);
		spi_message_init(&db->spi_dmsg1);
		xfer= &db->spi_dxfer1;

#if 1
	  wbuff_u8(DM_SPI_RD | DM_SPI_MRCMD, db->tmpTxPtr);
#else
         memcpy(db->tmpTxPtr, txb, 2);
        //memcpy(db->tmpRxPtr, 
#endif
		xfer->tx_buf = db->tmpTxPtr; //txb;
		xfer->rx_buf = db->tmpRxPtr; //trxb;
		xfer->len = RD_LEN_ONE + len;
		spi_message_add_tail(&db->spi_dxfer1, &db->spi_dmsg1);
		if(INNODev_sync(db))
			printk("[dm95_spi]INNO ERROR: len=%d, txbuf=0x%p,rxbuf=0x%p",len,txb,trxb); //return INNO_GENERAL_ERROR;

		printkr("dm9rx_smal_tx_cmd(%x) ... \n", db->tmpTxPtr[0]);

		memcpy(trxb, db->tmpRxPtr, RD_LEN_ONE + len);                
		//dread_32_Limitation(db, trxb, len);
	}
} //printkr

void dread_32_Limitation(board_info_t *db, u8 *trxb, int len)
{
  u8 txb[1];
  
  int n;
  unsigned counter, not_process;
  
  wbuff_u8(DM_SPI_RD | DM_SPI_MRCMD, txb);
  trxb++;
  
  not_process= len;
  for (n= 0; n< len; n += 31){	// n += 'counter' is the exact calc, But we use n += 31 can get the same looping..	
	if (not_process <= 31)
	  counter= not_process;
	else 
	  counter= 31;
  	
  	xrdbuff_u8(db, txb, db->TxDatBuf, counter);
  	memcpy(trxb + n, &db->TxDatBuf[1], counter);
  	not_process -= counter;
  }
}

void dread_1_Limitation(board_info_t *db, u8 *trxb, int len)
{
  int n;
  u8 txb[1];
  u8 test_buf[6]; //len is 4
  wbuff_u8(DM_SPI_RD | DM_SPI_MRCMD, txb);
  trxb++;
  for (n=0; n< len; n++){
  	xrdbuff_u8(db, txb, test_buf, 1);
  	*trxb++= test_buf[1]; 
  }
}
