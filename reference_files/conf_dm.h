 /* [configuration definition] */

// Optional configuration: 
//#define DM_CONF_MAX_SPEED_HZ	(50 * 1000 *1000)  //(chnage spi speed to 50MHz)
//#define DM_CONF_INTERRUPT //(Note: Not available for be less efficient!)
//#define DM_CONF_INTERRUPT_IRQ	3 //(Note: When available, Set IRQ number by this const definition!)
//#define DM_CONF_MODULE //{change to kernel driver module}

//#define DM_CONF_VSPI
#define DM_CONF_MAX_SPEED_HZ	(20 * 1000 *1000)
#define DM_CONF_INTERRUPT
#define DM_CONF_INTERRUPT_IRQ	3
#define DM_CONF_MODULE