/* 
 * drivers/net/dm9051.c (dm9r.c )
 * Copyright 2014 Davicom Semiconductor,Inc.
 *
 * 	This program is free software; you can redistribute it and/or
 * 	modify it under the terms of the GNU General Public License
 * 	as published by the Free Software Foundation; either version 2
 * 	of the License, or (at your option) any later version.
 *
 * 	This program is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 	GNU General Public License for more details.
 *
 *	http://www.davicom.com.tw
 *	2014/03/11  Joseph CHANG  v1.0  Create	
 *	2014/05/05  Joseph CHANG  v1.01  Start from Basic functions	
 *	2015/06/08  Joseph CHANG  v1.02  Formal naming (Poll version)	
 *  Ver: Step1.3 dma: mt_ DMA.. (20151203)
 *  Ver: Step1.3p1 DMA3_PNs design for platforms' config (20151223)
  * Ver: [3p6s]
  * Ver: 3p6ss
  * Ver: 3p11sss (kmalloc re-placement)
  *      Remove: drwbuff_u8()
  * Ver: V1.1
  * Ver: V1.2  Default as static 'dm9051.o'
  * Ver: V1.5b customization (2017/10/17)
  * Ver: V1.69.3 Support interrupt mode and mdio mutex_lock
  * Ver: V1.69.5 Start Poll: Was in dm9015_probe / Is in dm9051_open,
  *      So can be exactly more stable (2018/04/26)
  * Ver: V1.69.6 Support phy_poll, It's helpful while select interrupt mode,
  *      For if no rx packets that still can well maintain the link-status. 
  *      (2018/07/02)
  * Ver: V1.69.7 Better reliability in TX/RX operation 
  *      Coerce to show debug message, do device link-down and then link-up.
  *      (2019/03/15) 
  * Ver: V1.69.7_SaveCPU1  
  *      (2019/10/24)
  * Ver: V1.69.7_SaveCPU2, Fine tune again polling CPU save and schedule_weight.  
  *      (2019/10/29)
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/cache.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include "dm9051.h"
#include <linux/of_irq.h>

#include "conf_ver.h"

#include "def_generation.h"
#include "def_const_opt.h"

#include "dts_yes_driver_table.h"
#include "dts_no_spi_brd_reg.c"
#include "dm9051_custom_head.h"
/*#include "dm9051_custom_gpio.c"*/
/*#include "dm9051_custom_timing.c"*/
#include "dm9051_custom_main.h" /* [2 lines] */

/* Board/System/Debug information/definition ---------------- */
#define DS_NUL							0
#define DS_POLL							1
#define DS_IDLE							2
#define CCS_NUL							0
//#define CCS_PROBE						1

#define DBG_TO_FINAL_ADD_1_INDEED 		1

#define DISP_GAP						25
#define TOTAL_SUPP_LIMIT	  			75

#define RXM_WrtPTR				0		// Write Read RX FIFO Buffer Writer Pointer
#define MD_ReadPTR				1		// Write Read RX FIFO Buffer Read Pointer

static u8 uRxDone_tx_found = 0;
int control_display_tx = 0;
unsigned tx_array[2];
int tx_i = 0;
static u8 disp1514_flg = 0;
u8 flg_rxdbg = 0;
u16 xSupp_Limit = 0; //75

/* DM9051 chip logic ---------------------------- */

u16 dm9051_add_calc(u16 data_wr_addr, u16 len)
{
	data_wr_addr += len;
	if (data_wr_addr >= 0x0c00)
	  data_wr_addr -= 0x0c00;
	return data_wr_addr;
}

void dumphex_fifo(char *typ, unsigned len, u8 *pbff, unsigned start_loc) //dm9051_dumphex
{
	unsigned i;
	u16 adr= start_loc; //0;
	i=0;
	printk("dumphex_fifo:('%s') PktLen %d : loc %d", typ, start_loc, len); //": %x" , (unsigned int) pbff
	while (len--){
		if (!(i%8)) printk(" ");
		if (!(i%16)) printk("\n%04x ", adr);
		i++;
		printk(" %02x", *pbff++);
		adr++;
	}
	printk("\n");
}

/* DM9051 basic structures ---------------------------- */

struct rx_ctl_mach {
	__le16	RxLen;
	u16		ERRO_counter; /*cycFOUND_counter*/
	u16		RXBErr_counter;
	u16		LARGErr_counter;
	u16		StatErr_counter; /* new */
    u16		FIFO_RST_counter; /*(preRST_counter) / cycRST_counter*/
    
	u16		rxbyte_counter;
	u16		rxbyte_counter0;
	
	u16		rxbyte_counter0_to_prt;
#if 0	
	u16		rx_brdcst_counter; 
	u16		rx_multi_counter;
#endif 	
	u16		rx_unicst_counter;
	u8		rxbyte;
	u8		isbyte; // ISR Register data
	//u8		dummy_pad;
};

static void bcprobe_rst_info_clear(struct rx_ctl_mach *pbc)
{
	pbc->ERRO_counter= 
	pbc->RXBErr_counter= 
	pbc->LARGErr_counter= 
	pbc->StatErr_counter=
	pbc->FIFO_RST_counter= 0;
}
static void bcopen_rx_info_clear(struct rx_ctl_mach *pbc)
{
	pbc->rxbyte_counter= 
	pbc->rxbyte_counter0= 
	
	pbc->rxbyte_counter0_to_prt= 
#if 0	
	pbc->rx_brdcst_counter= 
	pbc->rx_multi_counter= 
#endif	
	pbc->rx_unicst_counter= 0;
	
	pbc->isbyte= 0xff; // Special Pattern
}
static void bcrdy_rx_info_clear(struct rx_ctl_mach *pbc)
{
	pbc->rxbyte_counter= 
	pbc->rxbyte_counter0= 0;
}

struct tx_state_mach {
	u16		prob_cntStopped;
	char	local_cntTXREQ;
	char	pad_0;
	u16		local_cntLOOP; // for trace looptrace_rec[]
#if DRV_TRACE_XLOOP
	#define NUMLPREC  16
	struct loop_tx {
	  u16 	looptrace_rec[NUMLPREC];  	// 20140522
	  int	loopidx;					// 20140522
	} dloop_tx;
#endif
};

#if DRV_TRACE_XLOOP	
void dloop_init(struct loop_tx *pt)
{
	int i;
    for (i=0; i<NUMLPREC; i++)
    {
      pt->looptrace_rec[i]= 0; // means no_value
    }
    pt->loopidx= 0;
}

void dloop_step(struct loop_tx *pt, u16 n)
{
	int i;
	if (pt->loopidx==NUMLPREC)
	{
    	pt->loopidx--; //63
    	//shift
	  	for (i=1; i<NUMLPREC; i++)
	  	  pt->looptrace_rec[i-1]= pt->looptrace_rec[i];
	}
    pt->looptrace_rec[pt->loopidx]= n;
    pt->loopidx++;
}

void dloop_diap_ALL(struct loop_tx *pt)
{
	int i;
	//display ALL: refer to rec_s.u.b_d.i.s.p(pdrec, tx_space);
	printk("TrXmitLP[%d]", pt->loopidx);
	if (!pt->loopidx) {
		printk("\n");
		return;
	}
	
	for (i=0; i<pt->loopidx; i++)
	{
	  if (!(i%8)) printk(" ");
	  else printk(",");
	  printk("%d", pt->looptrace_rec[i]); //rec_rxdisp(prx, i);
	  //if (pt->looptrace_rec[i]=='w') printk(" ");
	}
	printk("\n");
	
	dloop_init(pt);
}
#endif

/* use ethtool to change the level for any given device */
static struct {
	u32 msg_enable;
} debug = { -1 };

/* Structure/enum declaration ------------------------------- */
/*
 * struct board_info - dm9051 spi driver private data
 * @netdev: The network device we're bound to
 * @spidev: The spi device we're bound to.
 * @lock: Lock to ensure that the device is not accessed when busy.
 * @statelock: Lock on this structure for tx list.
 * @mii: The MII state information for the mii calls.
 * @rxctrl: RX settings for @rxctrl_work.
 //@tx_work: Work queue for tx packets (eliminated)
 * @rxctrl_work: Work queue for updating RX mode and multicast lists
 * @txq: Queue of packets for transmission.
 * @spi_msg1: pre-setup SPI transfer with one message, @spi_xfer1.
 * @spi_msg2: "pre-setup SPI transfer with two messages, @spi_xfer2".
 * @rxd: Space for receiving SPI data, in DMA-able space.
 * @txd: Space for transmitting SPI data, in DMA-able space.
 * @msg_enable: "The message flags controlling driver output (see ethtool)".
 * @rc_ccr: Cached copy of KS_CCR.
 *
 * The @lock ensures that the chip is protected when certain operations are
 * in progress. When the read or write packet transfer is in progress, most
 * of the chip registers are not ccessible until the transfer is finished and
 * the DMA has been de-asserted.
 *
 * The @statelock is used to protect information in the structure which may
 * need to be accessed via several sources, such as the network driver layer
 * or one of the work queues.
 *
 * We align the buffers we may use for rx/tx to ensure that if the SPI driver
 * wants to DMA map them, it will not have any problems with data the driver
 * modifies.
 */	

typedef struct board_info {
	u8 tmpTxPtr[CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES];
	u8 tmpRxPtr[CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES];

	struct rx_ctl_mach	  bC;
	struct tx_state_mach  bt;
	struct spi_device	*spidev;
	struct sk_buff_head	txq;
	struct spi_message	spi_msg1;
	struct spi_transfer	spi_xfer1;
	struct spi_message	spi_dmsg1; //Uesd when is 'SPI_DMA_MTK'
	struct spi_transfer	spi_dxfer1; //Uesd when is 'SPI_DMA_MTK'

	struct mutex	 	addr_lock;	/* dm9051's lock;*/	/* phy and eeprom access lock */
	struct net_device   *ndev; /* dm9051's netdev */

	u8					driver_state;
	u8					chip_code_state;
	
	u16 				rwregs[2];
	u16 				DISPLAY_rwregs[2];
	
	u16 DERFER_calc;
	u16 DERFER_rwregs[2];
	u16 DERFER_calc1;
	u16 DERFER_rwregs1[2];
	
	u16 				tx_rwregs[2];
	u16 				tx_qs;
	
	int					tx_eq;
	int					tx_err;
	
	u16 				rx_count;
	
	u8					imr_all;
	u8					rcr_all;
	int					link;

	unsigned int		in_suspend :1;
	unsigned int		wake_supported :1;
	int					ip_summed;

	struct mutex	 	sublcd_mtkspi_mutex; //Uesd when is 'SPI_DMA_MTK'
#if 1	
	spinlock_t			statelock; /* state lock;*/
#endif

	struct mii_if_info 	mii;
	u8					rxd[8] ____cacheline_aligned; //aS R1 for R2.R3
	u8					txd[8];
	u32					msg_enable ____cacheline_aligned;

	struct delayed_work	 phy_poll;
//.#if DRV_INTERRUPT_1 | DRV_POLL_0
	struct delayed_work	rx_work; //"INT_or_poll_Work;"
  //struct work_struct	rx._work;
//.#endif
	#if DRV_INTERRUPT_1
	struct work_struct	tx_work;
	#endif
	struct work_struct	rxctrl_work;

	u8  TxDatBuf[SPI_SYNC_TRANSFER_BUF_LEN];
	u8  Enter_count;
} board_info_t;

/* debug code */
int DM9051_rxdbg(board_info_t *db);
//static void dm9051_reset(board_info_t * db);
//static void dm9051_fifo_reset(u8 state, u8 *hstr, board_info_t *db);
//static void dm9051_fifo_reset_statistic(board_info_t *db);
//static void dm9051_INTPschedule_isr(board_info_t *db);
//void read_tx_mrr(board_info_t *db, u16 *ptrmrr); // used 'dm9051_display_get_txrwr_xx'
//u16 dm9051_rx_cap(board_info_t *db);
//void dm9051_fifo_ERRO(u8 rxbyte, board_info_t *db);

void driver_dtxt_init(board_info_t *db)	{
	// func: dtxt_init(&db->bt.dtxt_tx); eliminated
}
void driver_dtxt_step(board_info_t *db, char c){
     // func: dtxt_step(&db->bt.dtxt_tx, c); eliminated
}
void driver_dtxt_disp(board_info_t *db){ //'=dm9051_queue_show'   
	// func: dtxt_diap_ALL(&db->bt.dtxt_tx); eliminated
}

void driver_dloop_init(board_info_t *db){
	#if DRV_TRACE_XLOOP	
	dloop_init(&db->bt.dloop_tx);
	#endif
}
void driver_dloop_step(board_info_t *db, u16 n)
{
	#if DRV_TRACE_XLOOP
	dloop_step(&db->bt.dloop_tx, n);
	#endif
}
void driver_dloop_disp(board_info_t *db){
	#if DRV_TRACE_XLOOP
	dloop_diap_ALL(&db->bt.dloop_tx);
	#endif
}

void driver_display_tx_cap(char *hdtitle, board_info_t *db, int item_n, int nTx /*, u16 txcalc*/)
{	
	if (control_display_tx<=250)
	{     
	     if (item_n!=2){ 
	       //printk("dm9( %s ) ", hdtitle);
	       //printk("txWpRp ./%02x", // (TXQ %d.%d%c)
		   //    db->tx_rwregs[1]);  // txcalc>>8, txcalc&0xFF, '%',
	       //printk("\n");
	       
	       //.printk("dm9( %s ) txWp ./%02x(txMDR.in)\n", hdtitle, db->tx_rwregs[1]); //"diff %d ", db->tx_qs
	     }
	     if (item_n==2){
	       //printk("   dm9( %s %d eq %d err %d ) ", hdtitle, nTx, db->tx_eq, db->tx_err);
	       //printk("txWpRp .%02x.%d bytes/%02x", // (TXQ %d.%d%c)
		   //    db->tx_rwregs[0], db->tx_qs, 	// txcalc>>8, txcalc&0xFF, '%',
		   //    db->tx_rwregs[1]);
	       //printk("\n");

	      if (!(control_display_tx % DISP_GAP) && (xSupp_Limit < TOTAL_SUPP_LIMIT))
	      {
	       printk("\n");
	       if (tx_i==1)
	         printk("[%3d] dm9( %s %d) nTx %d ( eq %d err %d )\n", control_display_tx, 
			   hdtitle, tx_array[0], nTx, db->tx_eq, db->tx_err); 
			   //" txWp ./%02x", db->tx_rwregs[1]
	       else if (tx_i==2)
	         printk("[%3d] dm9( %s %d %d) nTx %d ( eq %d err %d )\n", control_display_tx, 
			   hdtitle, tx_array[0], tx_array[1], nTx, db->tx_eq, db->tx_err);
	       else if (tx_i > 2)
	         printk("[%3d] dm9( %s %d %d...) nTx %d ( eq %d err %d )\n", control_display_tx, 
			   hdtitle, tx_array[0], tx_array[1], nTx, db->tx_eq, db->tx_err);
	       else
	         printk("[%3d] dm9( %s check-condition!! ) nTx %d ( eq %d err %d )\n", control_display_tx, 
			   hdtitle, nTx, db->tx_eq, db->tx_err); 
			   //"diff %d ", db->tx_qs
			   //"(no txMDR_reset)"
	       }
	       tx_i = 0;
	     }
	 }
}

/* 
 *   DM9051 basic spi_sync call ---------------------------- 
 */

#define RD_LEN_ONE	1

void wbuff(unsigned op, __le16 *txb)
{
  //op= DM_SPI_WR | reg | val
	txb[0] = cpu_to_le16(op);
}

void wbuff_u8(u8 op, u8 *txb)
{
	txb[0]= op;
}

void xwrbyte(board_info_t * db, __le16 *txb)
{
	struct spi_transfer *xfer = &db->spi_xfer1;
	struct spi_message *msg = &db->spi_msg1;
	int ret;
	
#ifdef DM_CONF_VSPI
	return;
#endif	
	
	xfer->tx_buf = txb;
	xfer->rx_buf = NULL;
	xfer->len = 2;

	ret = spi_sync(db->spidev, msg);
	if (ret < 0)
		netdev_err(db->ndev, "spi_.sync()failed (xwrbyte 0x%04x)\n", txb[0]);
}

void xrdbyte(board_info_t * db, __le16 *txb, u8 *trxb)
{
	struct spi_transfer *xfer= &db->spi_xfer1;
	struct spi_message *msg= &db->spi_msg1;
	int ret;
	
		xfer->tx_buf = txb;
		xfer->rx_buf = trxb;
		xfer->len = 2;

		ret = spi_sync(db->spidev, msg);
		if (ret < 0)
			netdev_err(db->ndev, "spi_.sync()fail (xrd.byte 0x%04x) ret= %d\n", txb[0], ret);
//u8	return trxb[1];
}

void xrdbuff_u8(board_info_t *db, u8 *txb, u8 *trxb, unsigned len) //xwrbuff
{
	struct spi_transfer *xfer = &db->spi_xfer1;
	struct spi_message *msg = &db->spi_msg1;
	int ret;
	
#ifdef DM_CONF_VSPI
		trxb[1]= 0;
        return;
#endif	
		//(One byte)
        xfer->tx_buf = txb;
        xfer->rx_buf = trxb;
        xfer->len = RD_LEN_ONE + len;
		ret = spi_sync(db->spidev, msg);
		if (ret < 0){
	    	printk("9051().e out.in.dump_fifo4, %d BYTES, ret=%d\n", len, ret); //"%dth byte", i
			printk(" <failed.e>\n");
		}
//u8	return trxb[1];
}

#include "dm9051_custom_irq.c"
#include "dm9051_custom_gpio.c"
#include "dm9051_custom_timing.c"
#include "dm9051_custom_main.c"

/*
 *   Read a byte from spi port
 */
static u8 iior(board_info_t *db, /*int*/ unsigned reg)
{ 
	__le16 *txb = (__le16 *)db->txd;
	u8 *trxb = db->rxd;
#ifdef DM_CONF_VSPI
	return 0;
#endif	
	wbuff(DM_SPI_RD | reg, txb);
	xrdbyte(db, txb, trxb);
	return trxb[1];
}

static u8 ior(board_info_t *db, /*int*/ unsigned reg)
{ 
	__le16 *txb = (__le16 *)db->txd;
	u8 *trxb = db->rxd;
	
	if (reg==DM9051_ISR
	/* x */
	    || reg==DM9051_PAR || reg==(DM9051_PAR+1) || reg==(DM9051_PAR+2)
		|| reg==(DM9051_PAR+3) || reg==(DM9051_PAR+4) || reg==(DM9051_PAR+5)
	/* x */
		|| reg==DM9051_MRRL || reg==DM9051_MRRH
		|| reg==DM_SPI_MRCMDX
#if DBG_TO_FINAL_ADD_1_INDEED	
		|| reg==DM9051_MWRL || reg==DM9051_MWRH
		|| reg==0x22 || reg==0x23
		|| reg==0x24 || reg==0x25
		|| reg==DM9051_FCR || reg==DM9000_EPCR 
		|| reg==DM9000_EPDRH || reg==DM9000_EPDRL
		|| reg==DM9051_TCR
#endif
	)
		return iior(db, reg);
		
#ifdef DM_CONF_VSPI
	if (reg==DM9051_PIDL) {
	  wbuff(DM_SPI_RD | reg, txb);
	  xrdbyte(db, txb, trxb); //trxb[1]= 0x51;
	}
	else if (reg==DM9051_PIDH) {
	  wbuff(DM_SPI_RD | reg, txb);
	  xrdbyte(db, txb, trxb); //trxb[1]= 0x90;
	}
	else trxb[1]= 0;
	printk("dm9rdreg.MOSI.p: VSPI[%02x][..]\n", db->txd[0]);
	printk("dm9rdreg.MISO.e: VSPI[..][%02x]\n", trxb[1]);
	return trxb[1];
#endif	
	wbuff(DM_SPI_RD | reg, txb);
	printk("dm9rdreg.MOSI.p: [%02x][..]\n", db->txd[0]);
	xrdbyte(db, txb, trxb);
	printk("dm9rdreg.MISO.e: [..][%02x]\n", trxb[1]);
	return trxb[1];
}

/*
 *   Write a byte to spi port
 */
static void iiow(board_info_t *db, /*int*/ unsigned reg, /*int*/ unsigned val)
{
	__le16 txb[2];
	wbuff(DM_SPI_WR| reg | val<<8, txb); //org: wrbuff(reg, val, txb)
	xwrbyte(db, txb);
}

static void iow(board_info_t *db, /*int*/ unsigned reg, /*int*/ unsigned val)
{
	__le16 txb[2];

	//iiow(db, reg, val);	
	if (reg!=DM9051_TCR &&reg!=DM9051_TXPLL &&reg!=DM9051_TXPLH
#if DBG_TO_FINAL_ADD_1_INDEED	
	    && reg!=DM9051_ISR &&reg!=DM9051_MWRL &&reg!=DM9051_MWRH
	    &&reg!=DM9000_EPAR &&reg!=DM9000_EPCR
	 /* && reg!=DM9051_FCR */ /* 'DM9000_EPDRL'/'DM9000_EPDRH' by iiow() */
#endif
	)
	  printk("iow[%02X %02X]\n", /*txb[0]&0x7f*/ reg, /*txb[0]>>8*/ val & 0xff); // eg: "iow [7E 82]"
	  // Include: DM9000_GPR, ... etc

	wbuff(DM_SPI_WR| reg | val<<8, txb); //wrbuff(reg, val, txb)
	xwrbyte(db, txb);
}

/*static unsigned int
dm9000_read_locked(board_info_t *db, int reg) --SPI CAN NOT 'spin_lock_irqsave'
{
	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(&db->lock, flags);
	ret = ior(db, reg);
	spin_unlock_irqrestore(&db->lock, flags);

	return ret;
}*/
/*static unsigned int dm9051_read_mutex(board_info_t *db, int reg)
{
	unsigned int ret;
    mutex_lock(&db->addr_lock);
	ret = iior(db, reg);
    mutex_unlock(&db->addr_lock);
	return ret;
}*/
static unsigned int dm9051_read_mutex(board_info_t *db, int reg)
{
	unsigned int ret;
    mutex_lock(&db->addr_lock);
	ret = iior(db, reg);
    mutex_unlock(&db->addr_lock);
	return ret;
}

/* Common cap calc usage */

void read_rwr(board_info_t *db, u16 *ptrwr)
{
	*ptrwr= ior(db, 0x24); //v.s. 'DM9051_MRRL'
	*ptrwr |= (u16)ior(db, 0x25) << 8;  //v.s. 'DM9051_MRRH'
}
void read_mrr(board_info_t *db, u16 *ptrmrr)
{
	*ptrmrr= ior(db, DM9051_MRRL);
	*ptrmrr |= (u16)ior(db, DM9051_MRRH) << 8; 
}

void read_tx_rwr(board_info_t *db, u16 *ptrwr)
{
	*ptrwr= ior(db, 0x22); //v.s. 'DM9051_MWRL'
	*ptrwr |= (u16)ior(db, 0x23) << 8;  //v.s. 'DM9051_MWRH'
}
void read_tx_mrr(board_info_t *db, u16 *ptrmrr)
{
	*ptrmrr= ior(db, DM9051_MWRL); // is '0x7A'
	*ptrmrr |= (u16)ior(db, DM9051_MWRH) << 8; // is '0x7B'
}

u16 dm9051_diff(u16 rwregs0, u16 rwregs1)
{
	u16 bcalc;
	if (rwregs0>=rwregs1)
		bcalc= rwregs0 - rwregs1;
	else
		bcalc= 0x3400 + rwregs0 - rwregs1; //= 0x4000 - rwregs[1] + (rwregs[0] - 0xc00)
	return bcalc;
}

u16 dm9051_calc(u16 rwregs0, u16 rwregs1)
{
	u32 digiCalc;
	u32 digi, dotdigi;
	
	if (rwregs0>=rwregs1)
		digiCalc= rwregs0 - rwregs1;
	else
		digiCalc= 0x3400 + rwregs0 - rwregs1; //= 0x4000 - rwregs[1] + (rwregs[0] - 0xc00)
		
	digiCalc *= 100;
	digi= digiCalc / 0x3400;
	
	dotdigi= 0;
	digiCalc -= digi * 0x3400;
	if (digiCalc>=0x1a00) dotdigi= 5;
	
	return /*value=*/ ((digi << 8) + dotdigi);
}

u16 dm9051_txcalc(board_info_t *db, u16 rwregs0, u16 rwregs1)
{
	u32 digiCalc;
	u32 digi, dotdigi;
	
	if (rwregs0>=rwregs1)
		digiCalc= rwregs0 - rwregs1;
	else
		digiCalc= 0x0c00 + rwregs0 - rwregs1; 
		
	db->tx_rwregs[0]= rwregs0; //regs[0]; // save in 'tx_xxx'
	db->tx_rwregs[1]= rwregs1; //regs[1];
	
	db->tx_qs= (u16) digiCalc;
		
	digiCalc *= 100;
	digi= digiCalc / 0x0c00;
	
	dotdigi= 0;
	digiCalc -= digi * 0x0c00;
	if (digiCalc>=0x0600) dotdigi= 5;
	
	return /*value=*/ ((digi << 8) + dotdigi);
}

u16 dm9051_rx_cap(board_info_t *db)
{
	u16 rwregs[2];
	read_rwr(db, &rwregs[0]);
	read_mrr(db, &rwregs[1]);
	db->rwregs[RXM_WrtPTR]= rwregs[0]; // save in 'rx_cap'
	db->rwregs[MD_ReadPTR]= rwregs[1];
	return dm9051_calc(rwregs[0], rwregs[1]);   
}
u16 dm9051_tx_cap(board_info_t *db)
{
	u16 regs[2];
	read_tx_rwr(db, &regs[0]);
	read_tx_mrr(db, &regs[1]);
	return dm9051_txcalc(db, regs[0], regs[1]);  
}

u16 dm9051_rx_cap_lock(board_info_t *db)
{
	u16 ret;
	  mutex_lock(&db->addr_lock);
	  ret= dm9051_rx_cap(db);
	  mutex_unlock(&db->addr_lock);
	  return ret;
}

/* [Common, ior/iow usage] */

/*
 *  disp
 */	
void dm9051_fifo_show_flatrx(char *dstr, board_info_t *db)
{
	u16 rwregs[2];	     
	u16 calc;
	read_rwr(db, &rwregs[0]);
	read_mrr(db, &rwregs[1]);
	calc= dm9051_calc(rwregs[0], rwregs[1]);
	
	/* Show rx-occupied state */
	if (dstr) printk("%s: ", dstr);
	printk(" rxWrRd .%04x/%04x (RO %d.%d%c)\n", rwregs[0], rwregs[1], 
		calc>>8, calc&0xff, '%');
}

/*
 *  set mac
 */	
static int dm9051_write_mac_addr(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
	int i;

	for (i = 0; i < ETH_ALEN; i++)
	  iow(db, DM9051_PAR+i, dev->dev_addr[i]);

	return 0;
}

static void dm9051_init_mac(board_info_t *db)
{
	struct net_device *dev = db->ndev;

	random_ether_addr(dev->dev_addr);
#if 1
	dev->dev_addr[0]= 0x00;
	dev->dev_addr[1]= 0xff;
	dev->dev_addr[2]= 0x00;
	dev->dev_addr[3]= 0x00;
	dev->dev_addr[4]= 0x90;
	dev->dev_addr[5]= 0x51;
#endif
	//printk("[dm9051.init_mac() %02X %02X %02X  %02X %02X %02X\n", dev->dev_addr[0],
	  //dev->dev_addr[1], dev->dev_addr[2], dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

	mutex_lock(&db->addr_lock);
	dm9051_write_mac_addr(dev);
	mutex_unlock(&db->addr_lock);

}

static void dm9051_set_mac(board_info_t *db)
{
	struct net_device *dev = db->ndev;
	//printk("[dm9051.set_mac() %02X %02X %02X  %02X %02X %02X\n", dev->dev_addr[0],
	  //dev->dev_addr[1], dev->dev_addr[2], dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

	mutex_lock(&db->addr_lock);
	dm9051_write_mac_addr(dev);
	mutex_unlock(&db->addr_lock);
}

/*
 *  Set DM9051 multicast address
 */
static void
dm9000_hash_table_unlocked(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
	struct netdev_hw_addr *ha;
	int i, oft;
	u32 hash_val;
	u16 hash_table[4];
	u8 rcr = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;

	for (i = 0, oft = DM9051_PAR; i < 6; i++, oft++)
		iiow(db, oft, dev->dev_addr[i]);

	/* Clear Hash Table */
	for (i = 0; i < 4; i++)
		hash_table[i] = 0x0;

	/* broadcast address */
	hash_table[3] = 0x8000;

	if (dev->flags & IFF_PROMISC)
		rcr |= RCR_PRMSC;

	if (dev->flags & IFF_ALLMULTI)
		rcr |= RCR_ALL;

	/* the multicast address in Hash Table : 64 bits */
	netdev_for_each_mc_addr(ha, dev) {
		hash_val = ether_crc_le(6, ha->addr) & 0x3f;
		hash_table[hash_val / 16] |= (u16) 1 << (hash_val % 16);
	}

	/* Write the hash table */
	for (i = 0, oft = DM9000_MAR; i < 4; i++) {
		iiow(db, oft++, hash_table[i]);
		iiow(db, oft++, hash_table[i] >> 8);
	}

	iow(db, DM9051_RCR, rcr);
	db->rcr_all= rcr;
/*
//TEST
	db->rcr_all |= RCR_PRMSC | IFF_ALLMULTI;
	printk("Test db->rcr_all from %02x to %02x\n", rcr, db->rcr_all);
*/	
}

static void 
dm9000_hash_table(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
	mutex_lock(&db->addr_lock);
	dm9000_hash_table_unlocked(dev);	
    mutex_unlock(&db->addr_lock);
}

static
void read_intcr_print(board_info_t *db)
{
	unsigned  rdv;
	unsigned char *int_pol;
	rdv= iior(db, DM9051_INTCR);
	int_pol= "active high";
	if (rdv&0x01)
	  int_pol= "active low";
	printk("ior[REG39H][%02x] (b0: %d)(%s)\n", rdv, rdv&0x01, int_pol);
}

void read_isr_print(board_info_t *db)
{
	unsigned  dat= ior(db, DM9051_ISR);
	printk("[dm9051.probe.ISR.MBNDRY_STS] data[= 0x%02x]\n", dat);
}

/*
 *  Read a word data from EEPROM
 */
/*
	Caller's example:
	u8 tmp[2];
	dm9051_read_eeprom(db, i, tmp); // i=0, 1, 2, ... 63
 */
static void
dm9051_read_eeprom(board_info_t *db, int offset, u8 *to)
{
	mutex_lock(&db->addr_lock);

	//unsigned long flags;	
	//spin_lock_irqsave(&db->statelock, flags);
	iiow(db, DM9000_EPAR, offset);
	iiow(db, DM9000_EPCR, EPCR_ERPRR);
	//spin_unlock_irqrestore(&db->statelock, flags);
	
	//dm9051_msleep(db, 1);		/* Wait read complete */
	//= 
	while ( iior(db, DM9000_EPCR) & EPCR_ERRE) ;
	
	//spin_lock_irqsave(&db->statelock, flags);
	iiow(db, DM9000_EPCR, 0x0);	
	to[0] = iior(db, DM9000_EPDRL);
	to[1] = iior(db, DM9000_EPDRH);
	//spin_unlock_irqrestore(&db->statelock, flags);
	
	mutex_unlock(&db->addr_lock);
}

/*
 * Write a word data to SROM
 */
static void
dm9051_write_eeprom(board_info_t *db, int offset, u8 *data)
{
	mutex_lock(&db->addr_lock);
	
	iiow(db, DM9000_EPAR, offset);
	iiow(db, DM9000_EPDRH, data[1]);
	iiow(db, DM9000_EPDRL, data[0]);
	iiow(db, DM9000_EPCR, EPCR_WEP | EPCR_ERPRW);
	
	//dm9051_msleep(db, 1);		/* Wait read complete */
	//= 
	while ( iior(db, DM9000_EPCR) & EPCR_ERRE) ;
	
	iow(db, DM9000_EPCR, 0);
	
	mutex_unlock(&db->addr_lock);
}

#define DM9051_PHY		0x40	/* PHY address 0x01 */

//SPI:
// do before: mutex_lock(&db->addr_lock); | (spin_lock_irqsave(&db->statelock,flags);)
// do mid: spin_unlock_irqrestore(&db->statelock,flags);, spin_lock_irqsave(&db->statelock,flags);
// do after: (spin_unlock_irqrestore(&db->statelock,flags);) | mutex_unlock(&db->addr_lock);
static int dm9051_phy_read(struct net_device *dev, int phy_reg_unused, int reg)
{
	board_info_t *db = netdev_priv(dev);
	int ret;

	/* Fill the phyxcer register into REG_0C */
	iiow(db, DM9000_EPAR, DM9051_PHY | reg);
	iiow(db, DM9000_EPCR, EPCR_ERPRR | EPCR_EPOS);	/* Issue phyxcer read command */

	//dm9051_msleep(db, 1);		/* Wait read complete */
	//= 
	while ( ior(db, DM9000_EPCR) & EPCR_ERRE) ;

	iiow(db, DM9000_EPCR, 0x0);	/* Clear phyxcer read command */
	/* The read data keeps on REG_0D & REG_0E */
	ret = (ior(db, DM9000_EPDRH) << 8) | ior(db, DM9000_EPDRL);
	return ret;
}

static void dm9051_phy_write(struct net_device *dev,
		 int phyaddr_unused, int reg, int value)
{
	board_info_t *db = netdev_priv(dev);

	printk("iowPHY[%02d %04x]\n", reg, value);
	/* Fill the phyxcer register into REG_0C */
	iow(db, DM9000_EPAR, DM9051_PHY | reg);
	/* Fill the written data into REG_0D & REG_0E */
	iiow(db, DM9000_EPDRL, value);
	iiow(db, DM9000_EPDRH, value >> 8);
	iow(db, DM9000_EPCR, EPCR_EPOS | EPCR_ERPRW);	/* Issue phyxcer write command */

	//dm9051_msleep(db, 1);		/* Wait write complete */
	//= 
	while ( ior(db, DM9000_EPCR) & EPCR_ERRE) ;

	iow(db, DM9000_EPCR, 0x0);	/* Clear phyxcer write command */
}

static int dm9051_phy_read_lock(struct net_device *dev, int phy_reg_unused, int reg)
{
	int val;
	board_info_t *db = netdev_priv(dev);
	mutex_lock(&db->addr_lock);
	val= dm9051_phy_read(dev, 0, reg); //
	mutex_unlock(&db->addr_lock);
	return val;
}
static void dm9051_phy_write_lock(struct net_device *dev, int phyaddr_unused, int reg, int value)
{
	board_info_t *db = netdev_priv(dev);
	mutex_lock(&db->addr_lock);
	dm9051_phy_write(dev, 0, reg, value);
	mutex_unlock(&db->addr_lock);
}

/* NetDevice (Linux style type) */
/*
static void dm_show_carrier(board_info_t *db,
				unsigned carrier, unsigned nsr)
{
	struct net_device *ndev = db->ndev;
	unsigned ncr = dm9051_read_mutex(db, DM9051_NCR);

	if (carrier)
		printk("%s: link up, %dMbps, %s-duplex, no LPA\n",
			 ndev->name, (nsr & NSR_SPEED) ? 10 : 100,
			 (ncr & NCR_FDX) ? "full" : "half");
	else
	  //dev_info(&db->spidev->dev, "%s: link down\n", ndev->name);
		printk("%s: link down\n", ndev->name);
}*/

#ifdef DM_CONF_PHYPOLL
static void
dm_netdevice_carrier(board_info_t *db)
{
	struct net_device *dev = db->ndev;
	unsigned nsr; 
	int link;	
    nsr= dm9051_read_mutex(db, DM9051_NSR);
    
	link= !!(nsr & 0x40); //& NSR_LINKST
	db->link= link;       //Rasp-save
	
	//Add	
	db->Enter_count++;
	if (db->Enter_count > 26) db->Enter_count = 26;
	//.if (control_display_tx<=250)
	//.{
	//.  printk("[DM9051.phy_poll] Enter %d (checking nsr %x) ...\n", db->Enter_count, nsr);
	//.}
	
	if (netif_carrier_ok(dev) != link) {
		if (link)
		  netif_carrier_on(dev);
		else
		  netif_carrier_off(dev);
		printk("[DM9051.phy_poll] Link Status is: %d\n", link);
	}
}
#endif

/*
static void
dm_netdevice_carrier(board_info_t *db)
{
	struct net_device *ndev = db->ndev;
	unsigned old_carrier = netif_carrier_ok(ndev) ? 1 : 0;
	unsigned new_carrier;
	unsigned nsr; 

    nsr= dm9051_read_mutex(db, DM9051_NSR);
	new_carrier= (nsr & NSR_LINKST) ? 1 : 0;
#if 1	
// action more (also in dm9051_continue_poll( ))
	db->link= (nsr & NSR_LINKST) ? 1 : 0;       //Rasp-save
#endif
	//printk("ior[%02x][%02x]\n", DM9051_NSR, nsr);

    if (new_carrier)
    	;
    else {
    	dm_show_carrier(db, new_carrier, nsr); // Added more here!! 'nsr' no-used in it!
		printk("ior[%02x][%02x]\n", DM9051_NSR, nsr);
		
	#if 0
		//function: dm9051_software_auto(void) {}
		int	phy20;
    	mutex_lock(&db->addr_lock);
		phy20= dm9051_phy_read(dev, 0, 20);
		
		//. if (!(phy20&0x10)) // HP Auto-MDIX is Enable
		//. {
			  if (phy20&0x20) //MDIX //OR &0x80
			  {
				printk("( dm_poll_phy nsr=0x%02x phy20= 0x%04x MDIX down) %d ++ \n", 
				  nsr, phy20, db->bC.rxbyte_counter0_to_prt);
			    printk("( dm_poll_phy WR phy20= 0x%04x [chg to MDI])\n", 0x10);
			    dm9051_phy_write(dev, 0, 20, 0x10);
			  }
			  else // if () MDI
			  {
				printk("( dm_poll_phy nsr=0x%02x phy20= 0x%04x MDI down) %d ++ \n", 
				  nsr, phy20, db->bC.rxbyte_counter0_to_prt);
			    printk("( dm_poll_phy WR phy20= 0x%04x [chg to MDIX])\n", 0x30);
			    dm9051_phy_write(dev, 0, 20, 0x30);
			  }
		//. }
		mutex_unlock(&db->addr_lock);
	#endif
	}
	
	if (old_carrier != new_carrier) {
	    if (new_carrier) //(netif_msg_link(db)), use 'new_carrier' is better!!
			dm_show_carrier(db, new_carrier, nsr);

		if (new_carrier)
			netif_carrier_on(ndev);
		else
			netif_carrier_off(ndev);
#if 1	
		// action more
		printk("[DM9051.poll] Link Status is: %d\n", db->link);
#endif
	}
	
#if 0
// Note: [In spi driver] No acceptable to call 'mii_check_media()'
	/-*if (db->flags & DM9000_PLATF_SIMPLE_PHY &&
	    !(db->flags & DM9000_PLATF_EXT_PHY)) {
		unsigned nsr = dm9000_read_locked(db, DM9000_NSR);
		unsigned old_carrier = netif_carrier_ok(ndev) ? 1 : 0;
		unsigned new_carrier;

		new_carrier = (nsr & NSR_LINKST) ? 1 : 0;

		if (old_carrier != new_carrier) {
			if (netif_msg_link(db)) //NOT_ by 'netif_msg_link()'
				dm9000_show_carrier(db, new_carrier, nsr);

			if (!new_carrier)
				netif_carrier_off(ndev);
			else
				netif_carrier_on(ndev);
		}
	} else
		mii_check_media(&db->mii, netif_msg_link(db), 0);*-/
#endif
}
*/

/*#include "dm9051_custom_main.c"*/

/* ethtool ops block (to be "dm9051_ethtool.c") */

static inline board_info_t *to_dm9051_board(struct net_device *dev)
{
	return netdev_priv(dev);
}

static void dm9051_get_drvinfo(struct net_device *dev,
			       struct ethtool_drvinfo *info)
{
	strcpy(info->driver, CARDNAME_9051);
	strcpy(info->version, DRV_VERSION);
	strlcpy(info->bus_info, dev_name(dev->dev.parent), sizeof(info->bus_info));
}

static void dm9000_set_msglevel(struct net_device *dev, u32 value)
{
	board_info_t *dm = to_dm9051_board(dev);
	dm->msg_enable = value;
}

static u32 dm9000_get_msglevel(struct net_device *dev)
{
	board_info_t *dm = to_dm9051_board(dev);
	return dm->msg_enable;
}

static int dm9000_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	board_info_t *dm = to_dm9051_board(dev);
	mii_ethtool_gset(&dm->mii, cmd);
	return 0;
}

static int dm9000_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	board_info_t *dm = to_dm9051_board(dev);
	return mii_ethtool_sset(&dm->mii, cmd);
}

static int dm9000_nway_reset(struct net_device *dev)
{
	board_info_t *dm = to_dm9051_board(dev);
	return mii_nway_restart(&dm->mii);
}

static u32 dm9000_get_link(struct net_device *dev)
{
	board_info_t *dm = to_dm9051_board(dev);
	return (u32)dm->link;
}

#define DM_EEPROM_MAGIC		(0x444D394B)

static int dm9000_get_eeprom_len(struct net_device *dev)
{
	return 128;
}

static int dm9000_get_eeprom(struct net_device *dev,
			     struct ethtool_eeprom *ee, u8 *data)
{
	board_info_t *dm = to_dm9051_board(dev);
	int offset = ee->offset;
	int len = ee->len;
	int i;

	/* EEPROM access is aligned to two bytes */
	if ((len & 1) != 0 || (offset & 1) != 0)
		return -EINVAL;

	ee->magic = DM_EEPROM_MAGIC;

	for (i = 0; i < len; i += 2)
		dm9051_read_eeprom(dm, (offset + i) / 2, data + i);
	return 0;
}

static int dm9000_set_eeprom(struct net_device *dev,
			     struct ethtool_eeprom *ee, u8 *data)
{
	board_info_t *dm = to_dm9051_board(dev);
	int offset = ee->offset;
	int len = ee->len;
	int i;

	/* EEPROM access is aligned to two bytes */
	if ((len & 1) != 0 || (offset & 1) != 0)
		return -EINVAL;

	if (ee->magic != DM_EEPROM_MAGIC)
		return -EINVAL;

	for (i = 0; i < len; i += 2)
		dm9051_write_eeprom(dm, (offset + i) / 2, data + i);
	return 0;
}

/*
static void dm9000_get_wol(struct net_device *dev, struct ethtool_wolinfo *w)
{
	board_info_t *dm = to_dm9000_board(dev);

	memset(w, 0, sizeof(struct ethtool_wolinfo));

	// note, we could probably support wake-phy too 
	w->supported = dm->wake_supported ? WAKE_MAGIC : 0;
	w->wolopts = dm->wake_state;
}

static int dm9000_set_wol(struct net_device *dev, struct ethtool_wolinfo *w)
{
	board_info_t *dm = to_dm9000_board(dev);
	unsigned long flags;
	u32 opts = w->wolopts;
	u32 wcr = 0;

	if (!dm->wake_supported)
		return -EOPNOTSUPP;

	if (opts & ~WAKE_MAGIC)
		return -EINVAL;

	if (opts & WAKE_MAGIC)
		wcr |= WCR_MAGICEN;

	mutex_lock(&dm->addr_lock);

	iow(dm, DM9000_WCR, wcr);

	mutex_unlock(&dm->addr_lock);

	if (dm->wake_state != opts) {
		// change in wol state, update IRQ state

		if (!dm->wake_state)
			irq_set_irq_wake(dm->irq_wake, 1);
		else if (dm->wake_state && !opts)
			irq_set_irq_wake(dm->irq_wake, 0);
	}

	dm->wake_state = opts;
	return 0;
}*/

static const struct ethtool_ops dm9051_ethtool_ops = {
	.get_drvinfo		= dm9051_get_drvinfo,
	.get_settings		= dm9000_get_settings,
	.set_settings		= dm9000_set_settings,
	.get_msglevel		= dm9000_get_msglevel,
	.set_msglevel		= dm9000_set_msglevel,
	.nway_reset			= dm9000_nway_reset,
	.get_link			= dm9000_get_link,
/*TBD	
	.get_wol		= dm9000_get_wol,
	.set_wol		= dm9000_set_wol,
*/
 	.get_eeprom_len		= dm9000_get_eeprom_len,
 	.get_eeprom			= dm9000_get_eeprom,
 	.set_eeprom			= dm9000_set_eeprom,
};

/* Init (& reset) chip */

static void
dm9051_reset(board_info_t * db)
{
	__le16 txb[2];
	
	  mdelay(2); // delay 2 ms any need before NCR_RST (20170510)
	
	  wbuff(DM_SPI_WR| DM9051_NCR | NCR_RST<<8, txb); //org: wrbuff(DM9051_NCR, NCR_RST, txb)
	  xwrbyte(db, txb);
	  mdelay(1);
	
	//iiow(db, 0x5e, 0x80);
	//=  
	  wbuff(DM_SPI_WR| 0x5e | 0x80<<8, txb); //org: wrbuff(DM9051_NCR, NCR_RST, txb)
	  xwrbyte(db, txb);
	  //.printk("iow[%02x %02x].[Set.MBNDRY.BYTEBNDRY]\n", 0x5e, 0x80);
	mdelay(1);
}

// ------------------------------------------------------------------------------
// state: 0 , fill '90 90 90 ...' e.g. dm9051_fi.fo_re.set(0, "fifo-clear0", db);
//		  1 , RST
//        2 , dump 'fifo-data...'
//		 11 , RST-Silent
// hstr:  String 'hstr' to print-out
//        NULL (no 'hstr' print)
// Tips: If (state==1 && hstr!=NULL)
//        Do increase the RST counter
// ------------------------------------------------------------------------------
static void dm9051_fifo_reset(u8 state, u8 *hstr, board_info_t *db)
{		  		 
     if (state==11)
     {
     		if (hstr)
	     	  ++db->bC.FIFO_RST_counter;
	      //printk("%s %d\n", hstr, db->bC.FIFO_RST_counter);
	     	printk("%s Len %d %d\n", hstr, db->bC.RxLen, db->bC.FIFO_RST_counter);
		  	dm9051_reset(db);	
			iiow(db, DM9051_FCR, FCR_FLOW_ENABLE);	/* Flow Control */
			iiow(db, DM9051_PPCR, PPCR_SETTING); /* Fully Pause Packet Count */
	     	iiow(db, DM9051_IMR, IMR_PAR | IMR_PTM | IMR_PRM);
	     	//iiow(db, DM9051_RCR, RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN);
	     	  iiow(db, DM9051_RCR, db->rcr_all);
			bcopen_rx_info_clear(&db->bC);
	     	return; 
     }
     if (state==1)
     {
     		u8 pk;
     		if (hstr)
     		{
	     	  ++db->bC.FIFO_RST_counter;
	     	  //printk("%s LenNotYeh %d %d\n", hstr, db->bC.RxLen, db->bC.FIFO_RST_counter); 
	     	  printk("%s rxb %02x rst_cnt %d\n", hstr, db->bC.rxbyte, db->bC.FIFO_RST_counter);
	    	}
		  	dm9051_reset(db);
	#if 1
			iiow(db, DM9051_FCR, FCR_FLOW_ENABLE);	/* Flow Control */
			if (hstr)
			  iiow(db, DM9051_PPCR, PPCR_SETTING); /* Fully Pause Packet Count */
			else
			{
			  pk= ior(db, DM9051_PPCR);
			  iow(db, DM9051_PPCR, PPCR_SETTING); /* Fully Pause Packet Count */
			}
	     	iiow(db, DM9051_IMR, IMR_PAR | IMR_PTM | IMR_PRM);
	     	//iiow(db, DM9051_RCR, RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN);
	     	  iiow(db, DM9051_RCR, db->rcr_all);
	#endif
			bcopen_rx_info_clear(&db->bC);
	     	return; 
	 }
     if (state==2){
	    printk("------- ---end--- -------\n");
	    printk("\n");
	    printk("Hang.System.JJ..\n");
	    while (1);
     }
     return; 
}

static void dm9051_fifo_reset_statistic(board_info_t *db)
{		
	printk("ERO.Rxb&LargEr\n");
	printk("%d .%d %d\n", 
		db->bC.ERRO_counter, db->bC.RXBErr_counter, db->bC.LARGErr_counter);
	if (db->bC.StatErr_counter)
		printk("StatEr %d\n", db->bC.StatErr_counter);
}

void dm9051_fifo_ERRO(u8 rxbyte, board_info_t *db)
{	
	u16 calc;
	if (db->bC.rxbyte_counter==5 || db->bC.rxbyte_counter0==(NUMRXBYTECOUNTER-1)) {
		 calc= dm9051_rx_cap(db);
	     db->bC.RXBErr_counter++;
	     printk("\n");
	     printk("( Rxb %d ) %d ++ \n", db->bC.RXBErr_counter, db->bC.rxbyte_counter0_to_prt);
	     printk("rxb=%02X rxWrRd .%02x/%02x (RO %d.%d%c)\n", rxbyte, 
	       db->rwregs[0], db->rwregs[1],
	       calc>>8, calc&0xFF, '%');
	     if (!(db->bC.RXBErr_counter%5))
	     {
	       driver_dtxt_disp(db);
	       driver_dloop_disp(db);
	     }
	     dm9051_fifo_reset(1, "dmfifo_reset( RxbEr )", db);
	     dm9051_fifo_reset_statistic(db);
		 printk("\n");
	}
	return;
} 

/*
 * Initialize dm9051 board
 */
static void dm9051_init_dm9051(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
	int	phy4;

	iiow(db, DM9051_GPCR, GPCR_GEP_CNTL);	/* Let GPIO0 output */
	
	/* dm9051_reset(db); */

/* DBG_20140407 */
  phy4= dm9051_phy_read(dev, 0, MII_ADVERTISE);	
  dm9051_phy_write(dev, 0, MII_ADVERTISE, phy4 | ADVERTISE_PAUSE_CAP);	/* dm95 flow-control RX! */	
  dm9051_phy_read(dev, 0, MII_ADVERTISE);

	/* Program operating register */
	iow(db, DM9000_TCR, 0);	        /* TX Polling clear */
	iiow(db, DM9000_BPTR, 0x3f);	/* Less 3Kb, 200us */
	iiow(db, DM9000_SMCR, 0);        /* Special Mode */
	/* clear TX status */
	iiow(db, DM9051_NSR, NSR_WAKEST | NSR_TX2END | NSR_TX1END);
	iow(db, DM9051_ISR, ISR_CLR_STATUS); /* Clear interrupt status */

	/* Init Driver variable */
	db->imr_all = IMR_PAR | IMR_PTM | IMR_PRM;
	db->rcr_all= RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;

	/*
	 * (Set address filter table) 
	 * After.call.ndo_open
	 * "kernel_call.ndo_set_multicast_list.later".
	 *   'dm9000_hash_table_unlocked'(dev);
	*/
    //(1)
    dm9051_fifo_reset(1, NULL, db); // 'NULL' for reset FIFO, and no increase the RST counter
}

/* Schedule (misc-item) block.s */
/* Schedule starter (trigger functions) */
/*  
static void 
dm_schedule_phy(board_info_t *db)
{  
  //schedule_delayed_work(&db->phy_poll, HZ * 2); to be 3 seconds instead
	schedule_delayed_work(&db->phy_poll, HZ * 3);
}*/

#ifdef DM_CONF_PHYPOLL 
static void 
dm_schedule_phy(board_info_t *db)
{  
  //schedule_delayed_work(&db->phy._poll, HZ * 2); to be 3 seconds instead
  //schedule_delayed_work(&db->phy._poll, HZ * 3);
	schedule_delayed_work(&db->phy_poll, HZ * 2);
}
#endif

/* ops reg-func, also play as schedule starter */

static void 
dm9051_set_multicast_list_schedule(struct net_device *dev)
{
	/* spin_lock/spin_unlock(&db->statelock);  no need */
	board_info_t *db = netdev_priv(dev);
	schedule_work(&db->rxctrl_work);
}

/* schedule reg-functions */

static void 
dm_phy_poll(struct work_struct *w)
{ 
	/*
	struct delayed_work *dw = to_delayed_work(w);
	board_info_t *db = container_of(dw, board_info_t, phy._poll);
	struct net_device *ndev = db->ndev;

    dm_netdevice_carrier(db);

	if (netif_running(ndev))
	  dm_schedule_phy(db);
	*/
#ifdef DM_CONF_PHYPOLL
	struct delayed_work *dw = to_delayed_work(w);
	board_info_t *db = container_of(dw, board_info_t, phy_poll);
	struct net_device *dev = db->ndev;
	
	dm_netdevice_carrier(db);
	if (netif_running(dev))
	  dm_schedule_phy(db);
#endif
}

static void 
dm9000_hash_table_work(struct work_struct *work)
{
	board_info_t *db = container_of(work, board_info_t, rxctrl_work);
	struct net_device *dev = db->ndev;

/*.	printk("dm95 [ndo_set_rx_mode ].s\n");*/
    dm9000_hash_table(dev);
}

/* schedule end. */
/* Schedule (misc-item) block.e */

/* Common etc usage (Chk data, Display, ...) */

static bool dm9051_chk_data(board_info_t *db, u8 *rdptr, int RxLen)
{
	struct net_device *dev = db->ndev;
	u16 calc;
#if 1
    if (rdptr[12]==0x08 && rdptr[13]==0x00)
      ; // Not to log, This is IP packet data
    else
    if (RxLen!=(1518)){ // (1514+4)
     //Display Debug RX-pkt	
     //printk("[RX.found.s]\n");
     calc= dm9051_rx_cap(db);
     //printk("dm9.hdrRd.s.%02x/%02x(RO %d.%d%c)\n", db->DERFER_rwregs[0], db->DERFER_rwregs[1], DERFER_calc>>8, DERFER_calc&0xFF, '%');
     if (RxLen >= (CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES-1))
      printkr(" OvLen=%d.", RxLen);
     else
      printkr(" SmallLen=%d.", RxLen);
     printkr("dm9rx.e.%02x/%02x(RO %d.%d%c) ", db->rwregs[0], db->rwregs[1], calc>>8, calc&0xFF, '%'); //dataWrRd
     printkr("rx(%02x %02x %02x %02x %02x. ", rdptr[0], rdptr[1],rdptr[2],rdptr[3],rdptr[4]); //"%02x %02x %02x" ,rdptr[3],rdptr[4],rdptr[5]
     printkr("%02x %02x .. ", rdptr[6], rdptr[7]);
     printkr("%02x %02x\n", rdptr[12],rdptr[13]);
    }
#endif 

#if 0	
	u16 sum_u6,sum_v6;	
 		//u8 *skbdata; //= rdptr
		sum_u6= rdptr[0]+rdptr[1]+rdptr[2]+rdptr[3]+rdptr[4]+rdptr[5];
		sum_v6= dev->dev_addr[1]+dev->dev_addr[2]+dev->dev_addr[3]+dev->dev_addr[4]+dev->dev_addr[5]+dev->dev_addr[6]; //BUG [1~6],NEED [0~5]
		
		if (sum_u6==(0xff+0xff+0xff+0xff+0xff+0xff)) // 0x5FA
		   db->bC.rx_brdcst_counter++;
		if (rdptr[0]&1) //'skb->data[0]'
			db->bC.rx_multi_counter++;
			
		//disp..change_location..			        
        if (sum_u6!=(0xff+0xff+0xff+0xff+0xff+0xff))
        {
          if (sum_u6==sum_v6)
          	db->bC.rx_unicst_counter++;
          else if (rdptr[0]&1) //'skb->data[0]'
			; //read_mrr(db, &ckRXcurr);
          else
          {
            //"[ERRO.found.s]"
          }
        }
#endif
	if (rdptr[0]!=dev->dev_addr[0] || rdptr[1]!=dev->dev_addr[1] || rdptr[2]!=dev->dev_addr[2])
	{
		if (rdptr[0]&1) //'skb->data[0]'
		{
		 	#if 0
			db->bC.rx_multi_counter++;
		 	#endif
            return true;
		}
		else
		{
            //"[ERRO.found.s]"
            calc= dm9051_rx_cap(db);
            printk("\n");
            printk("rWrRd.%02x/%02x(RO %d.%d%c) ", db->rwregs[0], db->rwregs[1], calc>>8, calc&0xFF, '%');
            printk("fifo_rst(.s %02x %02x %02x %02x %02x %02x\n", rdptr[0], rdptr[1],rdptr[2],rdptr[3],rdptr[4],rdptr[5]);
            calc= dm9051_rx_cap(db);
            printk("rWrRd.%02x/%02x(RO %d.%d%c) ", db->rwregs[0], db->rwregs[1], calc>>8, calc&0xFF, '%');
            printk("fifo_should %02x %02x %02x %02x %02x %02x\n", dev->dev_addr[0], dev->dev_addr[1],dev->dev_addr[2],
              dev->dev_addr[3],dev->dev_addr[4],dev->dev_addr[5]);
			db->bC.ERRO_counter++;
			printk("( ERO %d ) %d ++ \n", db->bC.ERRO_counter, db->bC.rxbyte_counter0_to_prt);
			dm9051_fifo_reset(11, "dmfifo_reset( 11 )", db); //~= dm9051_fifo_reset(11, ...)
            calc= dm9051_rx_cap(db);
            printk("rWrRd.%02x/%02x(RO %d.%d%c) ", db->rwregs[0], db->rwregs[1], calc>>8, calc&0xFF, '%');
            printk("fifo_rst(.e %02x %02x %02x %02x %02x %02x\n", rdptr[0], rdptr[1],rdptr[2],rdptr[3],rdptr[4],rdptr[5]);
            dm9051_fifo_reset_statistic(db);
			printk("\n");
            return false;
		}
	}
	db->bC.rx_unicst_counter++;
    return true;
}  

#if DM9051_CONF_TX
static void dm9051_tx_chk(struct net_device *dev, u8 *wrptr)
{
#if 0
    printk("dm9.tx_packets %lu ", dev->stats.tx_packets);
    printk("tx(%02x %02x %02x %02x %02x %02x ", wrptr[0], wrptr[1],wrptr[2],wrptr[3],wrptr[4],wrptr[5]);
    printk("%02x %02x   %02x %02x %02x %02x ", wrptr[6], wrptr[7],wrptr[8],wrptr[9],wrptr[10],wrptr[11]);
    printk("%02x %02x\n", wrptr[12],wrptr[13]);
#endif
}
#endif

void dm9051_display_get_txrwr_sectloop(void)
{
	if (control_display_tx<=250 && !(control_display_tx % DISP_GAP))
	  printk("  ");
}

void dm9051_display_get_txrwr_triploop(board_info_t *db, int nTx, unsigned len)
{
  //.AddCalc= dm9051_add_calc(db->tx_rwregs[1], len);
	u16 regs, AddCalc= dm9051_add_calc(db->tx_rwregs[1], len);
    read_tx_mrr(db, &regs);
    
    if (tx_i<=1) // tx_array[0], tx_array[1]. (to be displaied.)
      tx_array[tx_i] = len;
    // tx_array[0],tx_array[1], [2], [3], [4], [5], [6],...
    tx_i++; 
    
	if (len==(1514)){
     //.printk("]");
     //.=
        if (regs==AddCalc){
			if (control_display_tx<=250)
			{  
			  //printk(" [%d./%02x..", len, regs);
			  //printk("E]");
			  //printk(" dm9(OK. %d).", len); 
			  //"\n"
			  disp1514_flg = 1;
			  //.printk(" pkt1514..");
			}
          db->tx_eq++;
        }else{
          printk(" [%d./%02x..", len, regs);
          printk("].ERR(should %02x)", AddCalc);
          db->tx_err++;
        }
	}
    if (len!=(1514)){
     //.printk(")");
     //.=
        if (regs==AddCalc){
			if (control_display_tx<=250)
			{  
				if (disp1514_flg)
				{
					disp1514_flg = 0;
					//..printk(" dm9(1514) dm9(OK.Tx_len %d)\n", len); // " ./%02x..", regs
				}
				else
				//printk(" (9Tx_ss %d./%02x..", len, regs);
				//printk("E)");
				  ; //..printk(" dm9(OK.Tx_len %d)\n", len); // " ./%02x..", regs
			}  
          db->tx_eq++;
        }else{
          printk(" (9Tx_ss %d./%02x..", len, regs);
          printk(").ERR(should %02x)", AddCalc);
          db->tx_err++;
        }
    }
      
    /* Next 'rwregs[1]' */
    db->tx_rwregs[1]= regs;
    
  #if 0  
    //if (!(nTx%5)) {
    //    printk("\n");
	//	dm9051_display_get_txrwr_sectloop();
    //}
  #endif
}
void dm9051_display_get_txrwr_endloop(int nTx)
{
	//if (nTx==1) printk(" %d pkt\n", nTx); else printk(" %d pkts\n", nTx);
	
	if (control_display_tx==250) {
		if (xSupp_Limit < TOTAL_SUPP_LIMIT) printk("\ndm9.Temp-Stop-Tx-Disp-Mode\n\n");
		printk("\n"); 
		//printk("(%3d)\n", control_display_tx); 
	}
		
	control_display_tx++;
	
	//if ((control_display_tx > 250) && !(control_display_tx % DISP_GAP)) {
	//	printk("\n"); 
	//	printk("(%3d)\n", control_display_tx); 
	//}
	if (control_display_tx==500) {
		if (xSupp_Limit < TOTAL_SUPP_LIMIT) printk("\ndm9.Restart-Tx-Disp-Mode\n\n");
		control_display_tx = 0; // init a display period.
	}
}

/* [RX/TX (misc-item) block] */

#if DM9051_CONF_TX
static void dm9051_outblk(board_info_t *db, u8 *buff, unsigned len) //(txp->data, tx_len)
{
	//WHY?
	//SYNC_len= ALIGN(len, 4); // [DA got problem! Think about the chip can not synchronous for TX DATA BYTES]
	//int SYNC_len= len;
	if (len>DM9051_PKT_MAX) printk("[WARN9051: Large TX packet is present!!!\n");

//idea- no copy
	if (Custom_SPI_Write(db, buff, len))
		return;
	else {
		memcpy(&db->TxDatBuf[1], buff, len);
		wbuff_u8(DM_SPI_WR | DM_SPI_MWCMD, db->TxDatBuf);
		xrdbuff_u8(db, db->TxDatBuf, NULL, len);
	}
}
#endif
		
static void dm9051_inblk(board_info_t *db, u8 *buff, unsigned len)
{
	// Note: Read into &buff[1]...
	u8 txb[1];
	
	if (len==1){
		wbuff_u8(DM_SPI_RD | DM_SPI_MRCMD, txb);
		xrdbuff_u8(db, txb, buff, 1);
		return;
	}
	
	if (Custom_SPI_Read(db, buff, len))
		return;
	else {
		wbuff_u8(DM_SPI_RD | DM_SPI_MRCMD, txb);
		xrdbuff_u8(db, txb, buff, len);
		return;
	}
}
static void dm9051_dumpblk(board_info_t *db, unsigned len)
{
	unsigned i;
	u8 rxb[2]; // Note: One shift...
	u8 txb[1];
	wbuff_u8(DM_SPI_RD | DM_SPI_MRCMD, txb);
	for(i=0; i<len; i++)
	  xrdbuff_u8(db, txb, rxb, 1);
}

#if DM9051_CONF_TX 	
static void opening_wake_queue1(struct net_device *dev) //( u8 flag)
{
	board_info_t *db= netdev_priv(dev);
	if (db->bt.prob_cntStopped)
	{
		db->bt.prob_cntStopped= 0;
		netif_wake_queue(dev);
	}
}

static void toend_stop_queue1(struct net_device *dev, u16 stop_cnt)
{
	board_info_t *db= netdev_priv(dev);
	if (stop_cnt<NUM_QUEUE_TAIL)
		return; // true;
	if (stop_cnt==NUM_QUEUE_TAIL)
	{
	  	netif_stop_queue(dev);
		return; // true;
	}
	//.wrong path, but anyhow call stop for it
	netif_stop_queue(dev);
	printk("[.wrong path]: WARN, anyhow call stop for it .. ");
	printk("(cntStop %d)\n", db->bt.prob_cntStopped);
	driver_dtxt_disp(db); // OPTIONAL CALLED
	return; // false;
}
#endif

/* routines for packing to use read/write blk */
static void dm9051_rd_rxhdr(board_info_t *db, u8 *buff, unsigned len)
{	
	u8 test_buf[12]; //len is 4
	//dm9051_inblk(db, test_buf, len);
	//memcpy(buff, test_buf + 1, len); 
	
	dm9051_inblk(db, test_buf, 1);
	buff[0]= test_buf[1];
	dm9051_inblk(db, test_buf, 1);
	buff[1]= test_buf[1];
	dm9051_inblk(db, test_buf, 1);
	buff[2]= test_buf[1];
	dm9051_inblk(db, test_buf, 1);
	buff[3]= test_buf[1];
}

static void dm9051_disp_hdr_s_new(board_info_t *db)
{
	u16 calc= dm9051_rx_cap(db);
	//printk("dm9.hdrRd.s.%02x/%02x(RO %d.%d%c)\n", db->rwregs[0], db->rwregs[1], calc>>8, calc&0xFF, '%'); //hdrWrRd
	db->DERFER_rwregs[RXM_WrtPTR]= db->rwregs[0];
	db->DERFER_rwregs[MD_ReadPTR]= db->rwregs[1];
	db->DERFER_calc= calc;
}
static void dm9051_disp_hdr_e_new(board_info_t *db)
{
	//u16 calc= dm9051_rx_cap(db);
	//printk("hdrWrRd.e.%02x/%02x(RO %d.%d%c) rxLen(0x%x)= %d\n", db->rwregs[0], db->rwregs[1], calc>>8, calc&0xFF, '%', 
	//  rxlen, rxlen);
	u16 calc= dm9051_rx_cap(db);
	db->DERFER_rwregs1[RXM_WrtPTR]= db->rwregs[0];
	db->DERFER_rwregs1[MD_ReadPTR]= db->rwregs[1];
	db->DERFER_calc1= calc;
} 

#if 1	
struct dm9051_rxhdr0 { //old
	u8	RxPktReady;
	u8	RxStatus;
	__le16	RxLen;
} __packed;

struct spi_rxhdr { //new
	u8	padb;
	u8	spiwb;
	struct dm9051_rxhdr {
	  u8	RxPktReady;
	  u8	RxStatus;
	  __le16	RxLen;
	} rxhdr;
} __packed;

#define TOTAL_COUNT_STEPorg			111
#define TOTAL_COUNT_STEP			(TOTAL_COUNT_STEPorg + (TOTAL_COUNT_STEPorg>>1))
#define TOTAL_COUNT_QUICK		 	81000
//#define TOTAL_COUNT_DISPLAY_IRQ	900000 // yes-need div 10
//#define TOTAL_COUNT_DISPLAY_IRQ	 90000 // div 10, yes-need div 2
//#define TOTAL_COUNT_DISPLAY_IRQ	 45000 // div 2
#define TOTAL_COUNT_DISPLAY_IRQ	 	45000
#define TOTAL_COUNT_CYCLE	  		90000000

u32 xTotalACC_IRQ = 0, xTotalACC_IRQ_EXPIRE = TOTAL_COUNT_DISPLAY_IRQ;
u16 xTotal_SKIP_DISP_num= 25; //xTotal_SKIP_DISP_num= 0; (NO Check)

int DM9051_Control_RXDbg(void)
{
	xTotalACC_IRQ += TOTAL_COUNT_STEP;
	if (xTotalACC_IRQ < TOTAL_COUNT_DISPLAY_IRQ*2)
		xTotalACC_IRQ += TOTAL_COUNT_QUICK;
	
	//polling
	if (xTotalACC_IRQ > xTotalACC_IRQ_EXPIRE)
	{
		xTotalACC_IRQ_EXPIRE += TOTAL_COUNT_DISPLAY_IRQ;
		
		if (xSupp_Limit >= TOTAL_SUPP_LIMIT) //75
			return 0; //75
		xSupp_Limit++; //75
		
		return 1;
	}
	return 0;
}

//return false: err
//return true: OK
bool DM9051_RX_Check(board_info_t *db, u8 *rdptr, int RxLen)
{
	if (!xTotal_SKIP_DISP_num)
		return true; // (NO Check)
		
	if (xTotal_SKIP_DISP_num > 1) {
		xTotal_SKIP_DISP_num--;
		return true; // (no need check), when begin period.
	}
	
	if (DM9051_Control_RXDbg() && (control_display_tx > 250))
	{
		if (!DM9051_rxdbg(db)) //PRINTK
			return false;
	}
	
	return dm9051_chk_data(db, rdptr, RxLen);
	//=
	//if (!dm9051_chk_data(db, rdptr, RxLen))
	//	return false;
	//return true;
	
}

// return 0: err
// return 1: OK
int DM9051_rxdbg(board_info_t *db)
{
#if 1 //rxdbg stop.
	int ret = 1;
	return ret;
#else	
	int ret = 0;
	u16 calc;
	u8 rxbyte;
	calc= dm9051_rx_cap(db);	
	rxbyte= ior(db, DM_SPI_MRCMDX);	/* Dummy read */
	rxbyte= ior(db, DM_SPI_MRCMDX);	/* Dummy read */
	
	printk("\n");
	if (db->rwregs[0] == db->rwregs[1])
	{
		printk("(%3d) dg [rwr,mdr][ %4x %4x] rxb %02x EQU  wp= rdp\n", 
			control_display_tx, db->rwregs[0], db->rwregs[1], rxbyte);
		//printk(" dg [rwr,mdr][ %4x %4x] rxb %02x", db->rwregs[0], db->rwregs[1], rxbyte);
		//printk(" EQU  wp= rdp\n");
		
		if (!rxbyte)
			ret = 1;
	}	
	else
	{
		u16 dl;
		if (db->rwregs[0] < db->rwregs[1])
		{
			db->rwregs[0] += 0x4000;
			db->rwregs[0] -=  0xc00;
		}
		dl = db->rwregs[0] - db->rwregs[1];
		
		printk("(%3d) dg [rwr,mdr][ %4x %4x] rxb %02x NEQ (RO %d.%d%c %3x=%4d)\n", 
			control_display_tx, db->rwregs[0], db->rwregs[1], rxbyte,
			calc>>8, calc&0xFF, '%',
			dl, dl);
		//printk(" dg [rwr,mdr][ %4x %4x] rxb %02x", db->rwregs[0], db->rwregs[1], rxbyte);
		//printk(" NEQ (RO %d.%d%c", calc>>8, calc&0xFF, '%');
		//printk(" %3x=%4d)\n", dl, dl);
		
		flg_rxdbg = 1;
		
		if (rxbyte==1)
			ret = 1;
	}
	//printk("\n");
	
	//maintain
	if (xTotalACC_IRQ > TOTAL_COUNT_CYCLE)
	{
		xTotalACC_IRQ -= TOTAL_COUNT_CYCLE;
		xTotalACC_IRQ_EXPIRE = TOTAL_COUNT_DISPLAY_IRQ;
	}
	return ret;
#endif
}

/*
 *  Received a packet and pass to upper layer
 */	
static void dm9000_rx(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
#if 1	
	struct dm9051_rxhdr rxhdr;
#endif	
#if 0	
	struct spi_rxhdr spihdr; //new
#endif	
	struct sk_buff *skb;
	u8 rxbyte, *rdptr;
	bool GoodPacket;
	int RxLen;
	u16 calc;
	
	db->rx_count= 0;

	/* Get most updated data */
	rxbyte= ior(db, DM_SPI_MRCMDX);	/* Dummy read */
	rxbyte= ior(db, DM_SPI_MRCMDX);	/* Dummy read */
	db->bC.rxbyte = rxbyte;
		
	do {
		db->bC.RxLen = 0; // store.s
		// if ( rxbyte & DM9000_PKT_ERR) 
		// if (!(rxbyte & DM9000_PKT_RDY))
		// if ( rxbyte != DM9000_PKT_RDY)
		if ( rxbyte != DM9051_PKT_RDY)
		{
			if ( rxbyte == 0x00 ) {
				
			    db->bC.rxbyte_counter0++;
	
			    calc= dm9051_rx_cap(db); // get db->rwregs[0] & db->rwregs[1]
			    if (db->bC.rxbyte_counter0>1 || (calc>>8)>NUM_RX_FIFO_FULL){ // '50'
			         printk("ISRByte 0x%02x, rxWrRd .%04x/%04x (rxbyt==00 (%d ++) @ conti_cntr %d) RO %d.%d%c\n", 
			           db->bC.isbyte, db->rwregs[0], db->rwregs[1],
			           db->bC.rxbyte_counter0_to_prt, db->bC.rxbyte_counter0,
			     	   calc>>8, calc&0xFF, '%');			     	   
	     	   
				     db->bC.rxbyte_counter0_to_prt += 1;
			         printk("RXB_00Hs_or_FIFO_pre_full ");
			         driver_dtxt_disp(db);
			    }
			    else
				     db->bC.rxbyte_counter0_to_prt += 1;
			} else
			    db->bC.rxbyte_counter++; // insteaded (FFH, or not_01H)
			
		    
			dm9051_fifo_ERRO(rxbyte, db); // (00H/FFH/not_01H) // {Operate RST if continue 'n' 0x00 read.}
			return;
		} /* Status check: this byte must be 0xff, 0 or 1 */
		
		/* rxbyte_counter/rxbyte_counter0 */
		bcrdy_rx_info_clear(&db->bC);
 
		/* A packet ready now  & Get status/length */
		GoodPacket = true;
		
		
//	struct net_device *dev = db->ndev;
//	u16 calc= dm9051_rx_cap(db);
//    printk("hdrWrRd.s.%02x/%02x(RO %d.%d%c)\n", db->rwregs[0], db->rwregs[1], calc>>8, calc&0xFF, '%');
//    printk("hdrWrRd.e.%02x/%02x(RO %d.%d%c) rxlen= ...\n", db->rwregs[0], db->rwregs[1], calc>>8, calc&0xFF, '%');
        
		  dm9051_rd_rxhdr(db, (u8 *)&rxhdr, sizeof(rxhdr));
		
		  RxLen = le16_to_cpu(rxhdr.RxLen);
		  db->bC.RxLen = le16_to_cpu(rxhdr.RxLen); // store
        
		/*
		 * [LARGE THAN 1536 or less than 64]!"
		 */
		 if (RxLen > DM9051_PKT_MAX || RxLen < 0x40) {

			u16 calc= dm9051_rx_cap(db);
			db->bC.LARGErr_counter++;
			
	     	printk("\n");
	     	printk("( LargEr %d / %d ) LargeLen=%d (RO %d.%d%c)\n", 
	        	db->bC.LARGErr_counter, (db->bC.FIFO_RST_counter+1), 
	     		RxLen, calc>>8, calc&0xFF, '%');
	     		
	        dm9051_fifo_reset(1, "dmfifo_reset( LargEr )", db);
	        dm9051_fifo_reset_statistic(db);
			printk("\n");
		    return;
		 }

		 /* Packet Status check, 'RSR_PLE' happen, but take it not error!! 20150609JJ */		 
		 /* rxhdr.RxStatus is identical to RSR register. */
		if (rxhdr.RxStatus & (RSR_FOE | RSR_CE | RSR_AE |
				      RSR_RWTO |
				      RSR_LCS | RSR_RF)) {
			GoodPacket = false;
			
	 		rdptr= (u8 *)&rxhdr;
     		printk("\n");
	 		printk("<!GoodPacket-rxbyte&rxhdr %02x & %02x %02x %02x %02x>\n", rxbyte, rdptr[0], rdptr[1], rdptr[2], rdptr[3]);
	 		
			if (rxhdr.RxStatus & RSR_FOE) 
				dev->stats.rx_fifo_errors++;
			if (rxhdr.RxStatus & RSR_CE) 
				dev->stats.rx_crc_errors++;
			if (rxhdr.RxStatus & RSR_RF) 
				dev->stats.rx_length_errors++;
				
			db->bC.StatErr_counter++;
	     	printk("\n");
	     	printk("( StatEr %d / %d ) StatEr=0x%02x", db->bC.StatErr_counter, (db->bC.FIFO_RST_counter+1), 
	     		rdptr[1]);
	     	printk("\n");
            dm9051_fifo_reset(1, "[!GoodPacket - StatusErr]", db);
            dm9051_fifo_reset_statistic(db);
            printk("\n");
		    return;
		}

		/* Move data from DM9051 */
		if ((skb = dev_alloc_skb(RxLen + 4)) == NULL)  {
            printk("dm9051 [Warn] [!ALLOC %d rx.len space]\n", RxLen);
            printk("\n");
            /* dump-data */
            dm9051_dumpblk(db, RxLen);
		    return;
		}

	    /* 
	     *  We note that "#define NET_IP_ALIGN  2"
	     *
	     *	Move data from DM9051 
		 *  (Linux skb->len IS LESS than 4, because it = RxLen - 4)
		 */
		/* Increase the headroom of an empty &skb_buff by            *
		 * reducing the tail room. Only allowed for an empty buffer. */ 
		skb_reserve(skb, 2);
		/* A pointer to the first byte is returned */
		rdptr = (u8 *) skb_put(skb, RxLen - 4);  
		
		/* Read received packet from RX SRAM */
	  //dm9051_rd_rxdata(db, rdptr-1, RxLen);
	  //=
		dm9051_inblk(db, rdptr-1, RxLen);
		
		if (!DM9051_RX_Check(db, rdptr, RxLen))
			return;
		//instead=
		//.	if (!dm9051_chk_data(db, rdptr, RxLen))
		//.		return;
    
		dev->stats.rx_bytes += RxLen;
    
		/* Pass to upper layer */
		skb->protocol = eth_type_trans(skb, dev);
		if (dev->features & NETIF_F_RXCSUM) {
			if ((((rxbyte & 0x1c) << 3) & rxbyte) == 0)
				skb->ip_summed = CHECKSUM_UNNECESSARY;
			else
				skb_checksum_none_assert(skb);
		}
		netif_rx(skb);
		dev->stats.rx_packets++;
		db->rx_count++;
		
		/* Get most updated data */
		rxbyte= ior(db, DM_SPI_MRCMDX);	/* Dummy read */
		rxbyte= ior(db, DM_SPI_MRCMDX);	/* Dummy read */ //rxbyte = 0x01; //. readb(db->io_data);
	db->bC.rxbyte = rxbyte;

	} while (rxbyte == DM9051_PKT_RDY); // CONSTRAIN-TO: (rxbyte != XX)
	
	if (uRxDone_tx_found && flg_rxdbg)
	{
		//.if (control_display_tx <= 250) 	// only show below in non-Tx-Disp-Mode
		//.	uRxDone_tx_found = 0;			// skip
			
		if (uRxDone_tx_found && flg_rxdbg)
		{
			u16 calc= dm9051_rx_cap(db);
			if (db->rwregs[0] == db->rwregs[1])
				printk(
					"         --after dm9.rxDone-xmit- [ %04x %04x] rxb %02x EQU  wp= rdp\n", 
					db->rwregs[0], db->rwregs[1],
					rxbyte);
			else {
				u16 bcalc = dm9051_diff(db->rwregs[0], db->rwregs[1]);
				printk(
					"         --after dm9.rxDone-xmit- [ %04x %04x] rxb %02x (RO %2d.%d%c %dbytes)\n", 
					db->rwregs[0], db->rwregs[1], rxbyte,
					calc>>8, calc%0xFF, '%', 
					bcalc);
			}
			
			if ( (calc>>8) <=1) {
				uRxDone_tx_found = 0;
				flg_rxdbg = 0;
			}
		}
	}
}

static irqreturn_t 
dm9051_isr_ext(int irq, void *dev_id, int flag)
{
	struct net_device *dev = dev_id;
	board_info_t *db = netdev_priv(dev);
	u16 calc;
	u16 rwregs[2];
	int int_status;
	u8 rxbyte;
	
  #if DRV_INTERRUPT_1	
	  mutex_lock(&db->addr_lock);
	  iiow(db, DM9051_IMR, IMR_PAR); // Disable all interrupts 
  #elif DRV_POLL_0
	  mutex_lock(&db->addr_lock);
  #endif

//printk("[dm9051.isr extend.s:\n");

	// dm9051_isr(irq, dev_id); =
	// Get DM9051 interrupt status 
	db->bC.isbyte= ior(db, DM9051_ISR);	
	int_status = db->bC.isbyte; // Got ISR
	iiow(db, DM9051_ISR, int_status);	// Clear ISR status 

	// Received the coming packet 
	calc= dm9051_rx_cap(db);
	
//if (db->Enter_count <15)	
//printk("[dm9051.isr extending. rwregs[0]/rwregs[1].s: %04x %04x\n", db->rwregs[0], db->rwregs[1]);

	if (db->rwregs[0]==db->rwregs[1])
	{
	  //;
	  if (int_status & ISR_ROS) 
	  	printk("( Impossible rwregs[0]==rwregs[1] overflow ) %d ++ \n", db->bC.rxbyte_counter0_to_prt);
	}
	else
	{
		rwregs[0]= db->rwregs[0];
		rwregs[1]= db->rwregs[1];
		//if (int_status & ISR_PRS)
		//	dm9000_rx(dev);
		//=
#if 1
		rxbyte= ior(db, DM_SPI_MRCMDX);	/* Dummy read */
		rxbyte= ior(db, DM_SPI_MRCMDX);	/* Dummy read */
	db->bC.rxbyte = rxbyte;

if (db->Enter_count <15)	
printk("[dm9051.isr extending rxbyte.s %02x:\n", rxbyte);	
	
		if ( rxbyte == DM9051_PKT_RDY)
		{
			if (int_status & ISR_PRS)
			{
				dm9000_rx(dev);
				//3p6s
				//if (db->rx_count>=5) /*typical, if (db->rx_count==1)*/
				//  printk("%2d.ISRByte NORM%02x rxb%02x (add %d to %lu) dtx %d\n", 
				//    flag, int_status, rxbyte, db->rx_count, dev->stats.rx_packets,
				//    db->bt.prob_cntStopped);
				if (db->rx_count==0)
				  printk("%2d.ISRByte 0x%02x NORMAL-but-Zero-pkt rxb%02x ( ??? %d to %lu)\n", 
				    flag, int_status, rxbyte, db->rx_count, dev->stats.rx_packets);
			}
			else
			{
				dm9000_rx(dev);
				//3p6s
				//if (db->rx_count>=5) /*typical, if (db->rx_count==1)*/
				  //printk("%2d.ISRByte what%02x rxb01 (add %d to %lu) dtx %d\n", 
				  //  flag, int_status, db->rx_count, dev->stats.rx_packets,
				  //  db->bt.prob_cntStopped);
			}
			
			if (db->bt.prob_cntStopped && ((calc>>8) >= 12) ) 
			{
			  uRxDone_tx_found = 1;
			  //.if (flg_rxdbg)
			  //.{
			  //.u16 bcalc = dm9051_diff(rwregs[0], rwregs[1]);
			  //.printk("%2d.rxb01 --dm9.rxDone xmit- .%04x/%04x (RO %2d.%d%c) %dbytes\n", flag, 
			  //.  rwregs[0], rwregs[1], calc>>8, calc&0xFF, '%', bcalc);
			  //.} 
			}
		}
		//.else
		//.{
		//.	if (int_status & ISR_PRS)
		//.	{
		//.		dm9000_rx(dev);
		//.		printk("%d.ISRByte 0x%02x warnning rxbyt=0x%02x (add %d to %lu)\n", 
		//.		  flag, int_status, rxbyte, db->rx_count, dev->stats.rx_packets);
		//.	}
		//.}
		else
		{
			if (db->bt.prob_cntStopped) 
			  printk("%2d. dm9( .%04x/%04x no-rx tx fnd - RO %d.%d%c rxb%02x)\n", 
			    flag, rwregs[0], rwregs[1], calc>>8, calc&0xFF, '%', rxbyte);
		}
		
if (db->Enter_count <15)
printk("[dm9051.isr extending rxbyte.e %02x:\n", rxbyte);	
#endif	
	}
	
//if (db->Enter_count <15)
//printk("[dm9051.isr extending. rwregs[0]/rwregs[1].e: %04x %04x \n", db->rwregs[0], db->rwregs[1]);

	/* Receive Overflow */
	if (int_status & ISR_ROS){
#if 1
//early fifo_reset
		db->bC.RXBErr_counter++;
		printk("dm9.( Rxb %d ) %d ++ \n", db->bC.RXBErr_counter, db->bC.rxbyte_counter0_to_prt);
		if (!(db->bC.RXBErr_counter%5))
	     {
	       driver_dtxt_disp(db);
	       driver_dloop_disp(db);
	     }
#endif		
		printk(" db_isbyte 0x%02x (%d ++)\n", db->bC.isbyte, db->bC.rxbyte_counter0_to_prt);
		printk(" int_status 0x%02x", int_status);
		dm9051_fifo_show_flatrx(" [ERR-ISR] (recieve overflow)", db);
#if 1
//early fifo_reset
		dm9051_fifo_reset(1, "dm9.dmfifo_reset( RxbEr )", db);
	    dm9051_fifo_reset_statistic(db);
#endif
		printk("\n");
	}
	
//printk("[dm9051.isr extend.e:\n");

  #if DRV_INTERRUPT_1
  #elif DRV_POLL_0
    mutex_unlock(&db->addr_lock);
  #endif

	//	if (int_status & ISR_LNKCHNG)
	//	  schedule_delayed_work(&db->phy._poll, 1); // fire a link-change request 

  #if DRV_INTERRUPT_1
	iiow(db, DM9051_IMR, db->imr_all); // Re-enable interrupt mask 
    mutex_unlock(&db->addr_lock);
  #elif DRV_POLL_0
  #endif	

	return IRQ_HANDLED; //[Here! 'void' is OK]
}
#endif

/* TX */

#if DM9051_CONF_TX
static void
dm9051_continue_xmit_inRX(board_info_t *db) //dm9051_continue_poll_xmit
{
		    struct net_device *dev = db->ndev;
		    int nTx= 0;
		    //.u16 txcalc;
		    db->bt.local_cntTXREQ= 0;
		    db->bt.local_cntLOOP= 0;
			while(!skb_queue_empty(&db->txq)) // JJ-20140225, When '!empty'
			{
				  struct sk_buff *tx_skb;
				  //  =dm9051_send_packet(db);	
				  while( ior(db, DM9051_TCR) & TCR_TXREQ ) 
				    driver_dtxt_step(db, 'B');
		        
				  if (!nTx) {
				    //.txcalc= 
				    dm9051_tx_cap(db);
				    driver_display_tx_cap("txFirst.", db, 1, nTx /*, txcalc*/);
				    //.dm9051_display_get_txrwr_sectloop();
				  }
		    
				  tx_skb = skb_dequeue(&db->txq);
				  if (tx_skb != NULL) {
					  	
					        if(db->bt.local_cntTXREQ==2)
					        {
					           while( ior(db, DM9051_TCR) & TCR_TXREQ ) 
					             driver_dtxt_step(db, 'Z');
					           db->bt.local_cntTXREQ= 0;
					        }

						    nTx++;

						    dm9051_outblk(db, tx_skb->data, tx_skb->len);
						    dm9051_display_get_txrwr_triploop(db, nTx, tx_skb->len); // do-prt "   9Tx_ON_small(%d ... "
						    iow(db, DM9051_TXPLL, tx_skb->len);
						    iow(db, DM9051_TXPLH, tx_skb->len >> 8);
						    iow(db, DM9051_TCR, TCR_TXREQ);
						    dev->stats.tx_bytes += tx_skb->len;
						    dev->stats.tx_packets++;
						    /* done tx */
					        #if 1
						    dm9051_tx_chk(dev, tx_skb->data);
					        #endif
						    dev_kfree_skb(tx_skb);	
						    driver_dtxt_step(db, '1');
				            db->bt.local_cntTXREQ++;
				            db->bt.local_cntLOOP++;
				            #if 0
				            /* supplement as MCU uip driver */
				            /* Is ping size can only small size ? */ 
				            //Reset TX FIFO pointer
				            iiow(db, 0x55, 0x02); 
				            #endif
				  } //if
			} //while
		
			driver_dtxt_step(db, 'w');
			driver_dloop_step(db, db->bt.local_cntLOOP);
		    
		    //.txcalc= 
		    dm9051_tx_cap(db);
		    driver_display_tx_cap("txDone", db, 2, nTx /*, txcalc*/);
			#if 1
			dm9051_display_get_txrwr_endloop(nTx);
			#endif
}
#endif


/**
 * schwrk_loop_xmit_inRx - process tx packet(s)
 *
 * This is called when a packet has been scheduled for
 * transmission and need to be sent to the device.
 */

/*
static void schwrk_loop_xmit_inRx(board_info_t *db)
{
}
*/

/* RX */

int rx_continue_rx(struct net_device *dev, board_info_t *db, int flag)
{
	u16 calc;
	if (db->bt.prob_cntStopped) { //(1/2)
      if (db->DISPLAY_rwregs[0]!=db->rwregs[0] || db->DISPLAY_rwregs[1]!=db->rwregs[1]) {
        db->DISPLAY_rwregs[0]= db->rwregs[0];
        db->DISPLAY_rwregs[1]= db->rwregs[1];
        calc= dm9051_rx_cap_lock(db); 
        if (db->rwregs[0]!=db->rwregs[1] && flg_rxdbg) 
          printk("%2d.break --dm9.next.rxb##-  .%04x/%04x (RO %2d.%d%c)\n", flag-1, 
			db->rwregs[0], db->rwregs[1], calc>>8, calc&0xFF, '%'); 
      }
	}

    if (db->bt.prob_cntStopped) { //(2/2)
		//.uRxDone_tx_found = 1;
		return 0; //false //break;
	}
	dm9051_isr_ext(dev->irq, dev, flag);
	return 1; //true;
}

/* Schedule starter (trigger functions) */

static void 
dm9051_INTPschedule_isr(board_info_t *db)
{
    /* 0, Because @delay: number of jiffies to wait or 0 for immediate execution */
    schedule_delayed_work(&db->rx_work, 0); 
}

static void
dm9051_INTPschedule_weight(board_info_t *db, unsigned long delay)
{
	static int sd_weight = 0;
	
	//urgent .Has_ReadPkt
	//if (db->DERFER_rwregs[MD_ReadPTR] != db->DERFER_rwregs1[MD_ReadPTR]) {
	//	printk("[dm9 Warn No_ReadPkt0 : MDWA 0x%x mdra 0x%x (RO %d.%d%c)] equ.\n", 
	//			db->DERFER_rwregs[RXM_WrtPTR], db->DERFER_rwregs[MD_ReadPTR], 
	//			db->DERFER_calc>>8, db->DERFER_calc&0xff, '%');
	//	printk("[dm9 Warn No_ReadPkt1 : MDWA 0x%x mdra 0x%x (RO %d.%d%c)]\n", 
	//			db->DERFER_rwregs1[RXM_WrtPTR], db->DERFER_rwregs1[MD_ReadPTR], 
	//			db->DERFER_calc1>>8, db->DERFER_calc1&0xff, '%');
	//} else {
	//}
	if (db->DERFER_rwregs[MD_ReadPTR] != db->DERFER_rwregs1[MD_ReadPTR]) {
		schedule_delayed_work(&db->rx_work, 0); 
		return;
	}
		
	//good.all.readout
	if (db->DERFER_rwregs1[RXM_WrtPTR] == db->DERFER_rwregs1[MD_ReadPTR]) { //THis is also 'db->DERFER_calc1>>8 == 0'
		//mdwa = 0;
		schedule_delayed_work(&db->rx_work, delay); 
		return;
	}
			
	sd_weight++;
	if (!(sd_weight%5)) /* "slower" by more delay_work(delay) */
	/*if (!(sd_weight%6)) */ {
		
		//warn.(NoWrPkt)But_Read_CutCut (too slow read or rx-pointer.Err)
		if ((db->DERFER_calc1>>8) > 5) {
			sd_weight = 0;
			
			#if 0
			//static u16 mdwa = 0;
			//if (mdwa != db->DERFER_rwregs1[0]) {
			//	printk("[dm9 Warn ReadPkt1 (ng-remain >5) : MDRA 0x%x mdwa 0x%x (RO %d.%d%c)]\n", db->DERFER_rwregs1[1], db->DERFER_rwregs1[0], db->DERFER_calc1>>8, db->DERFER_calc1&0xff, '%');
			//	mdwa = db->DERFER_rwregs1[0];
			//}
			#endif
			
			dm9051_fifo_reset(1, "dm9 (RxPoint.Err)", db);
			dm9051_fifo_reset_statistic(db);
			
			schedule_delayed_work(&db->rx_work, 1);  //or 'delay'
			return;
		}
		
	
		//normal
		if (sd_weight>=6000) /*(sd_weight>=5000) in disp no adj*/ 
			sd_weight = 0;
			
		if ( sd_weight == 0 && (db->DERFER_calc1>>8 )!= 0) // fewer disp
			printk("-[dm9 SaveCPU for: MDWA 0x%x (RO %d.%d%c)]-\n", db->DERFER_rwregs1[RXM_WrtPTR], db->DERFER_calc1>>8, db->DERFER_calc1&0xff, '%');
		
		schedule_delayed_work(&db->rx_work, delay);  // slower ,
		return;
	}
	schedule_delayed_work(&db->rx_work, 0); 
}

//************************************************************//
/* Not used interrupt-related function: */
/* This function make it less efficient for performance */
/* Since the ISR not RX direct, but use a schedule-work */
/* So that polling is better in using its poll schedule-work */
#if DRV_INTERRUPT_1
static irqreturn_t dm951_irq(int irq, void *pw)
{
	board_info_t *db = pw;
//r	disable_irq_nosync(irq);
  #if DRV_INTERRUPT_1	
    disable_irq_nosync(irq);
  #endif
    dm9051_INTPschedule_isr(db); //dm9051_continue_poll() //schedule_work(&db->rx._work); //new 'dm9051_INTPschedule_isr'
	return IRQ_HANDLED;
}
#endif
//************************************************************//

static void dm9051_tx(board_info_t *db)
{
	struct net_device *dev = db->ndev;
#if DM9051_CONF_TX
	if (db->bt.prob_cntStopped)  // This is more exactly right!!
	{
		  #if LOOP_XMIT
		    mutex_lock(&db->addr_lock);
		    dm9051_continue_xmit_inRX(db); //=dm9051_continue_poll_xmit
		    opening_wake_queue1(dev); 
		    mutex_unlock(&db->addr_lock);
		  #endif //LOOP_XMIT
	}
#endif //DM9051_CONF_TX
}

/* schedule reg-functions */

static void dm9051_continue_poll(struct work_struct *work) //old. dm9051_INTP_isr
{
	struct delayed_work *dw = to_delayed_work(work);
	board_info_t *db = container_of(dw, board_info_t, rx_work);
	struct net_device *dev = db->ndev;

	u8 nsr;
	int link;	
	
	if (db->Enter_count <26)
	  printk("[DM9051.Interrupt] Enter %d (checking) ...\n", db->Enter_count++);
	
	mutex_lock(&db->addr_lock);
	dm9051_disp_hdr_s_new(db);
	nsr= iior(db, DM9051_NSR); 
	mutex_unlock(&db->addr_lock);	
	//JJ-Add
	link= !!(nsr & 0x40); //& NSR_LINKST
	db->link= link;       //Rasp-save
	//Add	
	if (netif_carrier_ok(dev) != link) {
		if (link)
		{
		  xSupp_Limit = 0; //75
		  netif_carrier_on(dev);
		}
		else
		  netif_carrier_off(dev);
		printk("[DM9051.continue_poll] Link Status is: %d\n", link);
	}
	
#if DM9051_CONF_TX
	if (db->bt.prob_cntStopped)  // This is more exactly right!!
	{
	  //if (control_display_tx<=250 && !(control_display_tx % DISP_GAP))
	  //{  
	  //  printk("\n");
	  //  printk("[%3d] DM9051.poll. Has nTx %d\n", control_display_tx, db->bt.prob_cntStopped);
	  //}
	  dm9051_tx(db); // tx.ing in_RX
	}
#endif	
	
	do {
	
   //.printk("dm9<DBG (to dm.9051_isr_EXT).s>\n");
	  if (!rx_continue_rx(dev, db, 1)) break;
	  if (!rx_continue_rx(dev, db, 2)) break;
	  if (!rx_continue_rx(dev, db, 3)) break;
	  if (!rx_continue_rx(dev, db, 4)) break;
	  if (!rx_continue_rx(dev, db, 5)) break;
	  if (!rx_continue_rx(dev, db, 6)) break;
	  if (!rx_continue_rx(dev, db, 7)) break;
	  if (!rx_continue_rx(dev, db, 8)) break;
	  if (!rx_continue_rx(dev, db, 9)) break;
	  if (!rx_continue_rx(dev, db, 10)) break;
	  if (!rx_continue_rx(dev, db, 11)) break;
	  if (!rx_continue_rx(dev, db, 12)) break;
    if (db->bt.prob_cntStopped) {       
      u16 calc;
      if (db->DISPLAY_rwregs[0]!=db->rwregs[0] || db->DISPLAY_rwregs[1]!=db->rwregs[1]) {
        db->DISPLAY_rwregs[0]= db->rwregs[0];
        db->DISPLAY_rwregs[1]= db->rwregs[1];
        calc= dm9051_rx_cap_lock(db); 
        if (db->rwregs[0]!=db->rwregs[1] && flg_rxdbg)
          printk("%2d.break --dm9.check.next_read.rxb .%04x/%04x (RO %2d.%d%c)\n",  
			13, db->rwregs[0], db->rwregs[1], calc>>8, calc&0xFF, '%'); 
      }
	}

	} while (0);

  #if DRV_INTERRUPT_1	
	enable_irq(db->ndev->irq);
  #elif DRV_POLL_0
  
	mutex_lock(&db->addr_lock);
    dm9051_disp_hdr_e_new(db);
	mutex_unlock(&db->addr_lock);	
//#define DM_TIMER_EXPIRE0a (HZ / 10)	//=[10]
//#define DM_TIMER_EXPIRE0b  msecs_to_jiffies(150) //=[15]
#define DM_TIMER_EXPIRE1    1  // 2 //15
#define DM_TIMER_EXPIRE2    0  //10
#define DM_TIMER_EXPIRE3    0
	if (db->DERFER_rwregs[RXM_WrtPTR] == db->DERFER_rwregs1[RXM_WrtPTR]) {
		//printk("[schedule_delayed %d slowest1 for: 0x%x == 0x%x]\n", DM_TIMER_EXPIRE, db->DERFER_rwregs[0] , db->DERFER_rwregs1[0] );
		//printk("[schedule_delayed %lu slowest1 for: 0x%x == 0x%x]\n", DM_TIMER_EXPIRE1, db->DERFER_rwregs[0] , db->DERFER_rwregs1[0] );
		//schedule_delayed_work(&db->rx_work, DM_TIMER_EXPIRE1); // slower ,
		dm9051_INTPschedule_weight(db, DM_TIMER_EXPIRE1);
	} else {
		if ((db->DERFER_calc1>>8) < 50) {
			//if ((db->DERFER_calc1>>8) > 5) 
			// printk("[schedule_delayed slow for: RWPAs %x %x (RO %d.%d%c)]\n", db->DERFER_rwregs[0], db->DERFER_rwregs1[0], db->DERFER_calc1>>8, db->DERFER_calc1&0xff, '%');
			schedule_delayed_work(&db->rx_work, DM_TIMER_EXPIRE2); // slow ,
		} else {
			//printk("[schedule_delayed faster for: RWPA 0x%x (RO %d.%d%c)]\n", db->DERFER_rwregs1[0], db->DERFER_calc1>>8, db->DERFER_calc1&0xff, '%');
			schedule_delayed_work(&db->rx_work,  DM_TIMER_EXPIRE3); // faster ,dm9051_INTPschedule_isr(db); 
    		/*old is schedule_work(&db->rx._work); */
    	}
    }
  #endif
}

/* ops reg-func */
#if DRV_INTERRUPT_1
static void dm9051_tx_work(struct work_struct *work)
{
	board_info_t *db = container_of(work, board_info_t, rxctrl_work);
	dm9051_tx(db);
}
#endif

static netdev_tx_t 
dm9051_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);

	//dm9000_dbg(db, 3, "%s:\n", __func__);
	
	spin_lock(&db->statelock);//mutex_lock(&db->addr_lock);
#if DM9051_CONF_TX
	toend_stop_queue1(dev, db->bt.prob_cntStopped++ );

	skb_queue_tail(&db->txq, skb); // JJ: a skb add to the tail of the list '&db->txq'
	driver_dtxt_step(db, '0'); //driver_dtxt_step(db, 'q'); // Normal
#elif 0
	//(db->outblk), ...dev_kfree_skb(skb);
#endif
	spin_unlock(&db->statelock);//mutex_unlock(&db->addr_lock);
	#if DRV_INTERRUPT_1
	schedule_work(&db->tx_work);
	#endif
	
	return NETDEV_TX_OK;
}

static void dm9051_open_code(struct net_device *dev, board_info_t *db) // v.s. dm9051_probe_code()
{
    /* Note: Reg 1F is not set by reset */
    iow(db, DM9000_GPR, 0);	/* REG_1F bit0 activate phyxcer */
    mdelay(1); /* delay needs by DM9051 */ 
	
    /* Initialize DM9051 board */
    dm9051_reset(db);
	dm9051_init_dm9051(dev);
}

#if 1
/**
 * Open network device
 * Called when the network device is marked active, such as a user executing
 * 'ifconfig up' on the device.
 */
static int
dm9051_open(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
    printk("\n");
	printk("[dm951_open].s\n");

  #if 1
	if (netif_msg_ifup(db))
		dev_dbg(&db->spidev->dev, "enabling %s\n", dev->name);
  #endif
	
	mutex_lock(&db->addr_lock); //Note: must 

	   if (db->chip_code_state==CCS_NUL)
	     dm9051_open_code(dev, db);

	   read_intcr_print(db);

  #if 0
     //(1)
     //[    6.644346] Unable to handle kernel NULL pointer dereference at virtual address 00000000
     //[    6.652369] pgd = dbe30000
     //     ...
     //[    6.675053] PC is at 0x0
     //[    6.677572] LR is at mii_link_ok+0x1c/0x38
     //[    6.681640] pc : [<00000000>]    lr : [<c04058f0>]    psr: 60000013

     //
     //  [In spi driver] No acceptable to call 'mii_check_media()'
     //

     // mii_check_media(&db->mii, netif_msg_link(db), 1);

     //(2)
     //[Which one is mandotory?]
     //If is 'netif_carrier_on(dev);' 
     // WILL solve others by move to a poll_phy (work)
  #endif

  #if DM9051_CONF_TX
     printk("[  sum  ] \n");
     printk("[  add  ] skb_queue_head_init: Design, Not, Only is in _probe()\n");
     printk("[  add  ] HERE, Test call skb_queue_head_init(), in _open()\n");
    //[Init.] [Re-init compare to ptobe.]
	 skb_queue_head_init(&db->txq); 
	 db->tx_eq= 0;
	 db->tx_err= 0;

     //[Should operated in _probe()]
     // Or that you can only call 'netif_stop_queue(dev)' before this operate (i.e. in _probe()),
     // And that you can not call 'netif_wake_queue(dev)' before this operate (i.e. in _probe()).
     printk("[  sum  ] \n");
     printk("[  sum  ] netif_start_queue: ?\n");
     printk("[  add  ] Yes, call netif_start_queue(), in _open()\n");
	 netif_start_queue(dev);
  #endif	

	 /* Init driver variable */
	
  #if 1
  #if DRV_POLL_0
	/* Flexable, So can start in Probe or here! */
	if (db->driver_state==DS_POLL)
	  ;
	else {
	   db->driver_state= DS_POLL;
	  //org=
	   dm9051_INTPschedule_isr(db); 
	   //=schedule_delayed_work(&db->rx._work, 0); 
       //schedule_work(&db->rx._work);

       //printk("[  sum  ] netif_carrier_on: ?\n");
       //printk("[  add  ] Yes, call netif_carrier_on()\n");
       //netif_carrier_on(dev);
	  
	  //add=
	  // netif_carrier_on(dev); have handled inside 'db->rx._work's hook-function.
	}
  #endif
  #ifdef DM_CONF_PHYPOLL
	dm_schedule_phy(db);
  #endif
  #endif
  //dm_schedule_phy(db); open 所以,目前版本沒有啟動此工作! ('db->link' 已有處理!)

	mutex_unlock(&db->addr_lock);

  #if DM9051_CONF_TX
    opening_wake_queue1(dev);
  #endif	
    printk("[dm9.open].e (%s)\n", DRV_VERSION);
    printk("[dm9.open].d DM9051_SPI: Mode %s\n", MSTR_MOD_VERSION);
    printk("[dm9.open].d DM9051_RD: Burst size %s\n", RD_MODEL_VERSION);
    printk("[dm9.open].d DM9051_TX: Burst size %s\n", WR_MODEL_VERSION);
    printk("\n");
    db->Enter_count =0;
	return 0;
} 
// "DRV.DM9: SPI_MODE: SPI_Master DMA model.."
// "DRV.DM9: SPI_MODE: SPI_Master FIFO model.."
// "DM9051_RD: Burst size (no-limitation)"
// "DM9051_TX: Burst size (no-limitation)"

/**
 * dm951_net_stop - close network device
 * @dev: The device being closed.
 *
 * Called to close down a network device which has been active. Cancell any
 * work, shutdown the RX and TX process and then place the chip into a low
 * power state whilst it is not being used.
 */
static int dm9000_stop(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);

	/* "kernel_call.ndo_set_multicast_list.first". */
	/* Then.call.ndo_stop                          */

	
	printk("[dm951_stop].s\n");
	db->driver_state= DS_IDLE;
	db->chip_code_state= CCS_NUL;
	
  #if DRV_INTERRUPT_1 | DRV_POLL_0
	/*
	mutex_lock(&db->addr_lock);
	cancel_delayed_work_sync(&db->rx._work); //flush_work(&db->rx._work);
	mutex_unlock(&db->addr_lock);
	*/
	cancel_delayed_work_sync(&db->rx_work); //flush_work(&db->rx_work);
  #endif
	//cancel_delayed_work_sync(&db->phy._poll);
#ifdef DM_CONF_PHYPOLL
	cancel_delayed_work_sync(&db->phy_poll);
#endif

	toend_stop_queue1(dev, db->bt.prob_cntStopped= NUM_QUEUE_TAIL); //ending_stop_queue1(dev);
	  
	  #if 0
	  //[.Temp commented]
	  mutex_lock(&db->addr_lock);
		
	  	/* stop any outstanding work */
	  	flush_work(&db->tx_work); // [.Temp commented] -- This is temp commented!
	  	flush_work(&db->rxctrl_work);
	
		/* ensure any queued tx buffers are dumped */
		while (!skb_queue_empty(&db->txq)) {
			struct sk_buff *txb = skb_dequeue(&db->txq);
	
			netif_dbg(db, ifdown, db->ndev,
				  "%s: freeing txb %p\n", __func__, txb);
	
			dev_kfree_skb(txb);
		}
		
	  mutex_unlock(&db->addr_lock);
	  #endif
	    
	//JJ-Count-on
		netif_carrier_off(dev);

	   /* dm9051_shutdown(dev) */
	   mutex_lock(&db->addr_lock);
	   printk("[dm951_stop].m\n");
	   dm9051_phy_write(dev, 0, MII_BMCR, BMCR_RESET);	/* PHY RESET */
	   iow(db, DM9000_GPR, 0x01);	/* Power-Down PHY */
	   iow(db, DM9051_IMR, IMR_PAR);	/* Disable all interrupt */
	   iow(db, DM9051_RCR, RCR_RX_DISABLE);	/* Disable RX */
       mutex_unlock(&db->addr_lock);
	   return 0;
}

//static int dm9051_ioctl(struct net_device *dev, struct ifreq *req, int cmd)
//{
//	board_info_t *dm = to_dm9051_board(dev);
//
//	if (!netif_running(dev))
//		return -EINVAL;
//
//	return generic_mii_ioctl(&dm->mii, if_mii(req), cmd, NULL);
//}

/*static int dm9000_set_features(struct net_device *dev,
	netdev_features_t features)
{
	board_info_t *dm = to_dm9000_board(dev);
	netdev_features_t changed = dev->features ^ features;
	unsigned long flags;

	if (!(changed & NETIF_F_RXCSUM))
		return 0;

	spin_lock_irqsave(&dm->lock, flags);
	iow(dm, DM9000_RCSR, (features & NETIF_F_RXCSUM) ? RCSR_CSUM : 0);
	spin_unlock_irqrestore(&dm->lock, flags);

	return 0;
}*/

/* ops */

static const struct net_device_ops dm9051_netdev_ops = {
	.ndo_open		= dm9051_open,
	.ndo_stop		= dm9000_stop,
	.ndo_start_xmit		= dm9051_start_xmit,
//>	.ndo_tx_timeout			= dm9000_timeout,
	.ndo_set_rx_mode = dm9051_set_multicast_list_schedule,
//. .ndo_do_ioctl		= dm9051_ioctl,
	.ndo_change_mtu		= eth_change_mtu,
//.	.ndo_set_features		= dm9000_set_features,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,	
#ifdef CONFIG_NET_POLL_CONTROLLER
	//.ndo_poll_controller 	=
#endif
};
#endif

//void dm9051_probe_code(struct net_device *dev, board_info_t *db)
//{
//    /* Note: Reg 1F is not set by reset */
//    iow(db, DM9000_GPR, 0);	/* REG_1F bit0 activate phyxcer */
//    mdelay(1); /* delay needs by DM9051 */ 
//	
//    /* Initialize DM9051 board */
//    dm9051_reset(db);
//	dm9051_init_dm9051(dev);
//}

/*
 * Search DM9051 board, allocate space and register it
 */
static int /* __devinit */ 
dm9051_probe(struct spi_device *spi)
{
	struct board_info *db;
	struct net_device *ndev;
    unsigned  chipid;
	int i;
	const unsigned char *mac_src;
	int ret = 0;

    printk("[ *dm9051  ] {_probe.s}\n");

	ndev = alloc_etherdev(sizeof(struct board_info));
	if (!ndev) {
		dev_err(&spi->dev, "failed to alloc ethernet device\n");
		return -ENOMEM;
	}
	/* setup board info structure */
	db = netdev_priv(ndev);

	mutex_init(&db->addr_lock);
	mutex_init(&db->sublcd_mtkspi_mutex);
	
	db->ndev = ndev;
	db->spidev = spi;

       spi->bits_per_word = 8;
 	   SubNetwork_SPI_Init(db, 1); // spi, // dma

    db->driver_state= DS_NUL;
    db->chip_code_state= CCS_NUL;
	
	db->DISPLAY_rwregs[0]= 0;
    db->DISPLAY_rwregs[1]= 0;
	db->link= 0;
	
#if 1
#if DM9051_CONF_TX
	driver_dtxt_init(db);
	driver_dloop_init(db);
#endif
#endif
	
	/* ERRO_counter/RXBErr_counter/LARGErr_counter/StatErr_counter/FIFO_RST_counter */
	bcprobe_rst_info_clear(&db->bC);
	/* rx_brdcst_counter/rx_multi_counter/rx_unicst_counter.rxbyte_counter/rxbyte_counter0/rxbyte_counter0_to_prt */
	bcopen_rx_info_clear(&db->bC); 
	
#if 1
#if DM9051_CONF_TX
	toend_stop_queue1(ndev, db->bt.prob_cntStopped= NUM_QUEUE_TAIL); //ending_stop_queue1(ndev);
#endif
#endif
	
#if 1	
	spin_lock_init(&db->statelock); // used in 'dm9051' 'start' 'xmit'
#endif
	#if DRV_INTERRUPT_1
	INIT_WORK(&db->tx_work, dm9051_tx_work);
	#endif
	INIT_WORK(&db->rxctrl_work, dm9000_hash_table_work);
#if DRV_INTERRUPT_1 | DRV_POLL_0
    INIT_DELAYED_WORK(&db->rx_work, dm9051_continue_poll); // old. 'dm9051_INTP_isr()' by "INIT_WORK"
#endif
	INIT_DELAYED_WORK(&db->phy_poll, dm_phy_poll);

	/* initialise pre-made spi transfer messages */
	spi_message_init(&db->spi_msg1);
	spi_message_add_tail(&db->spi_xfer1, &db->spi_msg1);

	/* setup mii state */
	db->mii.dev	     = ndev;
	db->mii.phy_id_mask  = 1;   //db->mii.phy_id_mask  = 0x1f;
	db->mii.reg_num_mask = 0xf; //db->mii.reg_num_mask = 0x1f;
	db->mii.phy_id		= 1,
	db->mii.mdio_read    = dm9051_phy_read_lock;
	db->mii.mdio_write   = dm9051_phy_write_lock;

	skb_queue_head_init(&db->txq); //[Init.]
    
	SET_NETDEV_DEV(ndev, &spi->dev);

#if 1
    /*
	 * No need: db->dev = &pdev->dev;            
     * May need: dev_set_drvdata(&spi->dev, db); 
     */
    dev_set_drvdata(&spi->dev, db);
#endif

	dm9051_reset(db);

	/* Get chip ID */
    conf_spi_print();
	for (i = 0; i < 8; i++) {
      chipid= ior(db, DM9051_PIDL);
	  chipid |= (unsigned)ior(db, DM9051_PIDH) << 8; //ior(db, );

	  if (chipid == (DM9051_ID>>16) || chipid == (DM9000_ID>>16))
	    break;
	  printk("dm9: read wrong id 0x%04x\n", chipid);
	}
	printk("[ *dm9051  ] Device ID %x\n", chipid);
	if (chipid != (DM9051_ID>>16) && chipid != (DM9000_ID>>16)) {
	  dev_err(&spi->dev, "failed to read device ID\n");
	  ret = -ENODEV;
	  goto err_id;
	}
//.	if (chipid == (DM9000_ID>>16)) printk("[dm9051.ior_chipid(): 0x%04x\n", chipid);
//.	if (chipid == (DM9051_ID>>16)) printk("[dm9051.ior_chipid(): 0x%04x\n", chipid);
	
    printk("[dm9051.dump_eeprom():");
	for (i = 0; i < 64; i++) {
		dm9051_read_eeprom(db, i, db->TxDatBuf);
		if (!(i%8)) printk("\n ");
		if (!(i%4)) printk(" ");
		printk(" %02x %02x", db->TxDatBuf[0], db->TxDatBuf[1]);
	}
	printk("\n");
	
#if 1
	db->TxDatBuf[0]= 0x08;
	db->TxDatBuf[1]= 0x00; 
	dm9051_write_eeprom(db, (12 + 0) / 2, db->TxDatBuf);
    printk("[dm9051.write_eeprom():  WORD[%d]= %02x %02x\n",
    	(12 + 0) / 2, db->TxDatBuf[0], db->TxDatBuf[1]);
#endif 
#if 1
	db->TxDatBuf[0]= 0x80;
	db->TxDatBuf[1]= 0x41; //  0x0180 | (1<<14), DM9051 E1 (old) set WORD7.D14=1 to 'HP Auto-MDIX enable'
	dm9051_write_eeprom(db, (14 + 0) / 2, db->TxDatBuf);
    printk("[dm9051.write_eeprom():  WORD[%d]= %02x %02x\n",
    	(14 + 0) / 2, db->TxDatBuf[0], db->TxDatBuf[1]);
#endif

    printk("[dm9051.dump_eeprom():");
	for (i = 0; i < 16; i++) {
		dm9051_read_eeprom(db, i, db->TxDatBuf);
		if (!(i%8)) printk("\n ");
		if (!(i%4)) printk(" ");
		printk(" %02x %02x", db->TxDatBuf[0], db->TxDatBuf[1]);
	}
	printk("\n");
	
	/* The node address from the attached EEPROM(already loaded into the chip), first. */
    mac_src = "eeprom2chip";
   
    printk("[dm9051.ior_mac().a:");
	for (i = 0; i < 6; i++) {
      ndev->dev_addr[i]= ior(db, DM9051_PAR+i);
      printk(" %02x", ndev->dev_addr[i]);
	}
	printk(" (%s)\n", mac_src);
    
    
    printk("[dm9051.check valid ether addr.s:\n");
    
	/* The node address by the laboratory fixed (if previous not be valid) */
    if (!is_valid_ether_addr(ndev->dev_addr)) {

	    mac_src = "lab_fixed";
	    dm9051_init_mac(db);
	
	    printk("dm9051.ior_mac().b[drv_default]:");
		for (i = 0; i < 6; i++) {
	      chipid= ior(db, DM9051_PAR+i);
	      printk(" %02x", chipid);
		}
		printk(" (%s)\n", mac_src);

#if 1	
		mac_src = "random";
		eth_hw_addr_random(ndev); // vs. random_ether_addr(dev->dev_addr);
		dm9051_set_mac(db);

	    printk("dm9051.ior_mac().c[random_mac]:");
		for (i = 0; i < 6; i++) {
	      chipid= ior(db, DM9051_PAR+i);
	      printk(" %02x", chipid);
		}
		printk(" (%s)\n", mac_src);
#endif
	}
    printk("[dm9051.check valid ether addr.e:\n");

#if DRV_INTERRUPT_1	
    
/*r	ret = request_irq(spi->irq, dm951_irq, IRQF_TRIGGER_LOW,
				   ndev->name, db); */

	spi->irq= request_irq_no();
	if (!spi->irq) {
		printk("dm9051 failed to get irq_no\n");
		goto err_irq;
	}
	ret = request_irq(spi->irq, dm951_irq, IRQF_TRIGGER_NONE, 
				ndev->name, db);

	/*r  ret = request_threaded_irq(spi->irq, NULL, dm951_irq,
	  			   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	  			   ndev->name, db); */
	printk("[DBG] dm9051 request irq %d, ret= %d\n", spi->irq, ret);
	if (ret < 0) {
		printk("dm9051 failed to get irq\n");
		goto err_irq;
	}
#endif

	ndev->if_port = IF_PORT_100BASET;
	ndev->irq = spi->irq;	

	/*
     * _DBG_.	ether_setup(ndev); --driver system function (no this is OK..?)
     */
	ndev->netdev_ops	= &dm9051_netdev_ops;
    //SET_ETHTOOL_OPS(ndev, &dm9051_ethtool_ops);
    //SET_ETHTOOL_OPS(ndev, &dm9051_ethtool_ops);
    //ndev->ethtool_ops = &dm9051_ethtool_ops;
#if DMA3_P3_KT
      SET_ETHTOOL_OPS(ndev, &dm9051_ethtool_ops); /*3p*/  
#else
      ndev->ethtool_ops = &dm9051_ethtool_ops;
#endif

    printk("[dm9051.check register netdev.s:\n");
	ret = register_netdev(ndev);
    printk("[dm9051.check register netdev.e:\n");

    if (ret) {
		dev_err(&spi->dev, "failed to register network device\n");
        printk("[  dm9051  ] dm9051_probe {failed to register network device}\n");
        goto err_netdev;
    }

    #if 1
    //[TEST NOT TO BE IN PROBE...]
    /* */
    //db->chip_code_state= CCS_NUL;
    /* */
    //db->chip_code_state= CCS_PROBE;
    //dm9051_probe_code(ndev, db);
    #endif

    printk("[dm9051.check INTschedule_isr.s:\n");
#if 1
    db->driver_state= DS_NUL;
	#if DRV_POLL_0
//;[Add_for_a_test!]
//; org=
//x	db->driver_state= DS_POLL;
//x	schedule_delayed_work(&db->rx_work, 5*HZ); // DRV_POLL_0, 40*Hz, 15*Hz, [5*Hz] zhengzhou,mobiletek,20170301, {change for Boot time}
	#endif
#endif	
    printk("[dm9051.check INTschedule_isr.e:\n");

	read_isr_print(db);
    printk("%s: dm951 is bus_num %d, chip_select %d {%s}\n", 
           ndev->name,
    	   spi->master->bus_num, 
    	   spi->chip_select, DRV_VERSION);
	printk("%s: dm9051spi at isNO_IRQ %d MAC: %pM (%s)\n", // (%s)
		   ndev->name,
		   ndev->irq,
		   ndev->dev_addr, mac_src);
    printk("[*dm9.probe].e (%s)\n", DRV_VERSION);
    printk("[*dm9.probe].d DM9051_SPI: Mode %s\n", MSTR_MOD_VERSION);
    printk("[*dm9.probe].d DM9051_RD: Burst size %s\n", RD_MODEL_VERSION);
    printk("[*dm9.probe].d DM9051_TX: Burst size %s\n", WR_MODEL_VERSION);
    db->Enter_count =0;

	return 0;

err_netdev:

#if DRV_INTERRUPT_1
	free_irq(spi->irq, db); 
#endif

err_id:

#if DRV_INTERRUPT_1
err_irq:
#endif
	printk("[ *dm9051  ] {_probe.e}not found (%d)\n", ret);

	free_netdev(ndev);

	return ret;
} // ['mutex_init' only in 'probe']
  
//**************************************************************************************
#if DMA3_P4_KT
/*3p*/
static int dm9000_drv_suspend(struct spi_device *spi, pm_message_t state)
{
    board_info_t *db = dev_get_drvdata(&spi->dev);
    struct net_device *ndev = db->ndev;
	if (ndev) {
		db->in_suspend = 1;
		if (!netif_running(ndev)) {
		    netif_device_detach(ndev);
            dm9000_stop(ndev);
	    }
	}
	return 0;
}

static int dm9000_drv_resume(struct spi_device *spi)
{
    board_info_t *db = dev_get_drvdata(&spi->dev);
    struct net_device *ndev = db->ndev;
	if (ndev) {
		if (netif_running(ndev)) {
            dm9051_open(ndev);
			netif_device_attach(ndev);
		}

		db->in_suspend = 0;
	}
	return 0;
}
#endif

//--------------------------------------------------------------------------------------

static int
dm9000_drv_remove(struct spi_device *spi)
{
    board_info_t *db = dev_get_drvdata(&spi->dev);
	unregister_netdev(db->ndev);
#if DRV_INTERRUPT_1
   	free_irq(spi->irq, db);
#endif
	free_netdev(db->ndev);
	return 0;
}

static int __init  
dm9051_init(void)
{
	printk("\n");
	printk("%s Driver\n", 
		CARDNAME_9051);
	printk("%s Driver, V%s (%s)\n", 
		CARDNAME_9051, 
		DRV_VERSION, str_drv_xmit_type);
	//printk("%s Driver, %s\n", 
	//CARDNAME_9051, MASTER_MODEL_VERSION);
    printk("%s, SPI %s\n", CARDNAME_9051, MSTR_MOD_VERSION);
    printk("%s, RD %s\n", CARDNAME_9051, RD_MODEL_VERSION);
    printk("%s, TX %s\n", CARDNAME_9051, WR_MODEL_VERSION);
  #if 1
	printk("dm9r Driver, modalias in dm9051.h= %s\n", DRVNAME_9051);
	printk("dm9r Driver, modalias in dm9r.c= %s\n", dm9051_driver.driver.name);
  #endif		

	//conf_spi_board();
	return spi_register_driver(&dm9051_driver);
}

static void /*__exit*/ dm9051_cleanup(void)
{
	unconf_spi_board();
	spi_unregister_driver(&dm9051_driver);
}

module_init(dm9051_init);
module_exit(dm9051_cleanup);

MODULE_DESCRIPTION("Davicom DM9051 network driver");
MODULE_AUTHOR("Joseph CHANG <joseph_chang@davicom.com.tw>");
MODULE_LICENSE("GPL");
module_param_named(debug, debug.msg_enable, int, 0);
MODULE_PARM_DESC(debug, "Debug verbosity level (0=none, ..., ffff=all)");
MODULE_ALIAS("spi:dm9051");
