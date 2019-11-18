// Local compiler-option
/*  RX is basic essential */
/*  TX is made optional('DM9051_CONF_TX') */

#define DM9051_CONF_TX   				1

#if DM9051_CONF_TX
#define NUM_QUEUE_TAIL					0xFFFE   //(2) //(5)//(65534= 0xFFFE)MAX_QUEUE_TAIL  
#endif