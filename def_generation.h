/**  Lab: Options control flags (The usage for the customer who have especial requirments ) */
#define DM_DM_CONF_INSTEAD_OF_DTS_AND_BE_DRVMODULE  	0
#define DM_DM_CONF_RARE_PROJECTS_DTS_USAGE		0

#if DM_DM_CONF_INSTEAD_OF_DTS_AND_BE_DRVMODULE

 #undef DM_CONF_MAX_SPEED_HZ
 #define DM_CONF_MAX_SPEED_HZ (20 * 1000 *1000)
 #undef DM_CONF_MODULE
 #define DM_CONF_MODULE

 #undef MTK_CONF_YES

 #undef DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
 #define DM_DM_CONF_RARE_PROJECTS_DTS_USAGE 0

#endif


/* -------------- */
/* Generation: dm */
/* -------------- */

/**  max spi speed: 20MHz [default] */
/**  speed update: chnage with new define const */
#ifndef DM_CONF_MAX_SPEED_HZ
 #define DRV_MAX_SPEED_HZ (20 * 1000 *1000)  // (20MHz)
#else /* DM_CONF_MAX_SPEED_HZ */
 #define DRV_MAX_SPEED_HZ DM_CONF_MAX_SPEED_HZ
#endif /* DM_CONF_MAX_SPEED_HZ */

/**  process: polling [default] */
#ifndef DM_CONF_INTERRUPT
  #define DRV_INTERRUPT_1    0
  #define DRV_POLL_0         1
#else /* DM_CONF_INTERRUPT */
/*#error "Sorry the sm9051 spi does not support IRQ interrupt! Because it is less performance!"*/
  #define DRV_INTERRUPT_1    1
  #define DRV_POLL_0         0
#endif /* DM_CONF_INTERRUPT */

/**  0: dm9051.ko (module) */
/**  1: dm9051.o (static) [default] */
#ifndef DM_CONF_MODULE
 #define DMA3_P6_DRVSTATIC  1  
#else /* DM_CONF_MODULE */
 #define DMA3_P6_DRVSTATIC  0  
#endif /* DM_CONF_MODULE */



#if DRV_INTERRUPT_1
#define DRV_VERSION	\
	"4.9.28-KT.INTWrk-1.69.7_saveCPU2_Pollphy"
#endif
#if DRV_POLL_0
#define DRV_VERSION	\
	"4.9.28-V7+.2018.POLL-mt_dma-1.69.7_saveCPU2_Pollphy"
#endif


#define DRV_TRACE_XLOOP						1
#define LOOP_XMIT						1
#define SCH_XMIT						0 

#if LOOP_XMIT
static char *str_drv_xmit_type= \
    "LOOP_XMIT";
#endif
#if SCH_XMIT
static char *str_drv_xmit_type= \
    "sch_xmit";
#endif

/* --------------- */
/* Generation: mtk */
/* --------------- */

#ifdef MTK_CONF_YES
 #ifdef MTK_CONF_SPI_DMA_YES
   #define DMA3_P0_MTKHDR  		1
   #define DMA3_P1_MTKGPIO  	1
   #define DMA3_P1_MTKSETUP		1

   #define DMA3_P2_MSEL_MOD		1 // SPI_MODE, 0: FIFO, 1: DMA (conjunction with 1024,32,1 bytes or no-limitation)
  #define DMA3_P2_RSEL_1024F	1 // RD_MACH: FIFO model (1024 bytes-limitation)
  #define DMA3_P2_RSEL_32F		0 // RD_MACH: FIFO model (32 bytes-limitation)
  #define DMA3_P2_RSEL_1F		0 // RD_MACH: FIFO model (1 byte-limitation)
  #define DMA3_P2_TSEL_1024F	1 // TX_MACH: FIFO model (1024 bytes-limitation)
  #define DMA3_P2_TSEL_32F		0 // TX_MACH: FIFO model (32 bytes-limitation)
  #define DMA3_P2_TSEL_1F		0 // TX_MACH: FIFO model (1 byte-limitation)
 #else
   #define DMA3_P0_MTKHDR  		1
   #define DMA3_P1_MTKGPIO  	1
   #define DMA3_P1_MTKSETUP		1

   #define DMA3_P2_MSEL_MOD		0
  #define DMA3_P2_RSEL_1024F	0 // RD_MACH: FIFO model (1024 bytes-limitation)
  #define DMA3_P2_RSEL_32F		1 // RD_MACH: FIFO model (32 bytes-limitation)
  #define DMA3_P2_RSEL_1F		0 // RD_MACH: FIFO model (1 byte-limitation)
  #define DMA3_P2_TSEL_1024F	0 // TX_MACH: FIFO model (1024 bytes-limitation)
  #define DMA3_P2_TSEL_32F		1 // TX_MACH: FIFO model (32 bytes-limitation)
  #define DMA3_P2_TSEL_1F		0 // TX_MACH: FIFO model (1 byte-limitation)
 #endif
#else /* MTK_CONF_YES */
  #define DMA3_P0_MTKHDR  		0
  #define DMA3_P1_MTKGPIO  		0
  #define DMA3_P1_MTKSETUP		0

  #define DMA3_P2_MSEL_MOD		0 // SPI_MODE, 0: FIFO, 1: DMA (conjunction with 1024,32,1 bytes or no-limitation)
 #define DMA3_P2_RSEL_1024F		0 // RD_MACH: FIFO model (1024 bytes-limitation)
 #define DMA3_P2_RSEL_32F		1 // RD_MACH: FIFO model (32 bytes-limitation)
 #define DMA3_P2_RSEL_1F		0 // RD_MACH: FIFO model (1 byte-limitation)
 #define DMA3_P2_TSEL_1024F		0 // TX_MACH: FIFO model (1024 bytes-limitation)
 #define DMA3_P2_TSEL_32F		1 // TX_MACH: FIFO model (32 bytes-limitation)
 #define DMA3_P2_TSEL_1F		0 // TX_MACH: FIFO model (1 byte-limitation)
#endif /* MTK_CONF_YES */

/* SE - System Environment setup (P0/P1/P3/P4/P6) */
/* PA - Performance Adjustment setup (P5) */
/* -------------------------------------- */
 #define DMA3_P3_KT		  	0 // keep 0 is OK
 #define DMA3_P4_KT		  	0 // keep 0 is OK

/* ------------------------------- */
/* - ReadWrite.configuration.table */
/* ------------------------------- */

//SPI_MODE
#if DMA3_P2_MSEL_MOD
 #define MSTR_MOD_VERSION		"(SPI_Master is DMA mode..)"   //enhance
#else
 #define MSTR_MOD_VERSION		"(SPI_Master is FIFO mode..)"  //def
#endif

//RD_MACH
#if DMA3_P2_RSEL_1024F
  #define RD_MODEL_VERSION		"(1024 bytes-limitation)"
#elif DMA3_P2_RSEL_32F
  #define RD_MODEL_VERSION		"(32 bytes-limitation)"	   //test.OK
#elif DMA3_P2_RSEL_1F
  #define RD_MODEL_VERSION		"(1 byte-limitation)"
#else
  #define RD_MODEL_VERSION		"(no-limitation)"		   //best
#endif
//TX_MACH
#if DMA3_P2_TSEL_1024F
  #define WR_MODEL_VERSION		"(1024 bytes-limitation)"
#elif DMA3_P2_TSEL_32F
  #define WR_MODEL_VERSION		"(32 bytes-limitation)"	   //tobe.test.again
#elif DMA3_P2_TSEL_1F
  #define WR_MODEL_VERSION		"(1 byte-limitation)"	   //tobe.test.again
#else
  #define WR_MODEL_VERSION		"(no-limitation)"		   //best
#endif