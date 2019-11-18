/* ---------------------------------------------- */
/* #if DM_DM_CONF_INSTEAD_OF_DTS_AND_BE_DRVMODULE */
/* #undef DMA3_P1_MTKGPIO                         */
/* #define DMA3_P1_MTKGPIO 0                      */
/* #endif                                         */
/* ---------------------------------------------- */

/* Below optional is either [PinControl] or [PinDefine] by the customer user */ 
/* Either [PinControl] or [PinDefine] is valid only when 'DMA3_P1_MTKGPIO' is 1 */
/*  [PinControl]
     #define  MT_GPIO_PINCTRL_NEW  1
     #define  MT_GPIO_PINCTRL      0
     #define  MT_GPIO_PINDEF       0
    [PinDefine]
     #define  MT_GPIO_PINCTRL_NEW  0
     #define  MT_GPIO_PINCTRL      1
     #define  MT_GPIO_PINDEF       0
    [PinDefine]
     #define  MT_GPIO_PINCTRL_NEW  0
     #define  MT_GPIO_PINCTRL      0
     #define  MT_GPIO_PINDEF       1
*/
//#define  MT_GPIO_PINCTRL_NEW  1
//#define  MT_GPIO_PINCTRL      0
//#define  MT_GPIO_PINDEF       0
#define  MT_GPIO_PINCTRL_NEW  0
#define  MT_GPIO_PINCTRL      1
#define  MT_GPIO_PINDEF       0

#if MT_GPIO_PINCTRL && DMA3_P1_MTKGPIO
static struct pinctrl_state *dm9051_rst_high;
static struct pinctrl_state *dm9051_rst_low;
static struct pinctrl_state *dm9051_en_high;
static struct pinctrl_state *dm9051_en_low;
static struct pinctrl *dm9051_pinctrl1;
#endif

static int mt_dm9051_pinctrl_init(struct spi_device *pdev)
{
	int retval = 0; 

#if MT_GPIO_PINCTRL_NEW && DMA3_P1_MTKGPIO
	struct device_node *avin_np= of_find_compatible_node(NULL, NULL, "mediatek,avin_no_work");
	unsigned int GPIO_AVIN_EN= of_get_named_gpio(avin_np, "net_spi_en", 0);;
	printk("mt_dm9051_pinctrl_init \n");
	gpio_direction_output(GPIO_AVIN_EN,1);

	mdelay(5);
#endif

#if MT_GPIO_PINCTRL && DMA3_P1_MTKGPIO
	printk("mt_dm9051_pinctrl_init \n");
	dm9051_pinctrl1 = devm_pinctrl_get(&pdev->dev);

	if (IS_ERR(dm9051_pinctrl1)) {
		printk("Cannot find pinctrl! \n");
		retval = PTR_ERR(dm9051_pinctrl1);
		return 0;
	}
	dm9051_rst_low = pinctrl_lookup_state(dm9051_pinctrl1, "dm9051_rst0");
	if (IS_ERR(dm9051_rst_low))
	{
		printk("Cannot find dm9051_rst_low pinctrl! \n");
		return 0;
	}
	dm9051_rst_high = pinctrl_lookup_state(dm9051_pinctrl1, "dm9051_rst1");
	if (IS_ERR(dm9051_rst_high))
	{
		printk("Cannot find dm9051_rst_high pinctrl! \n");
		return 0;
	}
#endif

#if MT_GPIO_PINCTRL && DMA3_P1_MTKGPIO
	#define LOOK_EN_LOW "spi_switch_en0"
	#define LOOK_EN_HIGH "spi_switch_en1"
	dm9051_en_low = pinctrl_lookup_state(dm9051_pinctrl1, LOOK_EN_LOW);
	if (IS_ERR(dm9051_en_low))
	{
		printk("Cannot find en_low pinctrl! \n");
		return 0;
	}
	dm9051_en_high = pinctrl_lookup_state(dm9051_pinctrl1, LOOK_EN_HIGH);
	if (IS_ERR(dm9051_en_high))
	{
		printk("Cannot find en_high pinctrl! \n");
		return 0;
	}
#endif

#if MT_GPIO_PINCTRL && DMA3_P1_MTKGPIO
	pinctrl_select_state(dm9051_pinctrl1, dm9051_en_high);
	mdelay(100);
#endif

#if MT_GPIO_PINCTRL && DMA3_P1_MTKGPIO
	pinctrl_select_state(dm9051_pinctrl1, dm9051_rst_low);
	mdelay(300);
	pinctrl_select_state(dm9051_pinctrl1, dm9051_rst_high);
	mdelay(100);
#endif
	return retval;
}


/*3p*/
//typedef struct board_info board_info_t;
//#define	INNO_GENERAL_ERROR	1

int SPI_GPIO_Set(int enable)
{
#if MT_GPIO_PINDEF && DMA3_P1_MTKGPIO
	if(enable)
		{
			mt_set_gpio_mode(GPIO_SPI_CS_PIN, 1);
			mt_set_gpio_pull_enable(GPIO_SPI_CS_PIN, GPIO_PULL_ENABLE);
			mt_set_gpio_pull_select(GPIO_SPI_CS_PIN, GPIO_PULL_UP);
			
			mt_set_gpio_mode(GPIO_SPI_SCK_PIN, 1);
			mt_set_gpio_pull_enable(GPIO_SPI_SCK_PIN, GPIO_PULL_ENABLE);
			mt_set_gpio_pull_select(GPIO_SPI_SCK_PIN, GPIO_PULL_DOWN);
			
			mt_set_gpio_mode(GPIO_SPI_MISO_PIN, 1);
			mt_set_gpio_pull_enable(GPIO_SPI_MISO_PIN, GPIO_PULL_ENABLE);
			mt_set_gpio_pull_select(GPIO_SPI_MISO_PIN, GPIO_PULL_DOWN);
			
			mt_set_gpio_mode(GPIO_SPI_MOSI_PIN, 1);
			mt_set_gpio_pull_enable(GPIO_SPI_MOSI_PIN, GPIO_PULL_ENABLE);
			mt_set_gpio_pull_select(GPIO_SPI_MOSI_PIN, GPIO_PULL_DOWN);
			//sublcd_msg("CMMB GPIO CS SPI PIN mode:num:%d, %d,out:%d, dir:%d,pullen:%d,pullup%d",GPIO_SPI_CS_PIN,mt_get_gpio_mode(GPIO_SPI_CS_PIN),mt_get_gpio_out(GPIO_SPI_CS_PIN),
			//mt_get_gpio_dir(GPIO_SPI_CS_PIN),mt_get_gpio_pull_enable(GPIO_SPI_CS_PIN),mt_get_gpio_pull_select(GPIO_SPI_CS_PIN));    
		}
	else
		{
			mt_set_gpio_mode(GPIO_SPI_CS_PIN, 0);
			mt_set_gpio_dir(GPIO_SPI_CS_PIN, GPIO_DIR_IN);
			mt_set_gpio_pull_enable(GPIO_SPI_CS_PIN, GPIO_PULL_DISABLE);
			
			mt_set_gpio_mode(GPIO_SPI_SCK_PIN, 0);
			mt_set_gpio_dir(GPIO_SPI_SCK_PIN, GPIO_DIR_IN);
			mt_set_gpio_pull_enable(GPIO_SPI_SCK_PIN, GPIO_PULL_DISABLE);
			
			mt_set_gpio_mode(GPIO_SPI_MISO_PIN, 0);
			mt_set_gpio_dir(GPIO_SPI_MISO_PIN, GPIO_DIR_IN);
			mt_set_gpio_pull_enable(GPIO_SPI_MISO_PIN, GPIO_PULL_DISABLE);
			
			mt_set_gpio_mode(GPIO_SPI_MOSI_PIN, 0);
			mt_set_gpio_dir(GPIO_SPI_MOSI_PIN, GPIO_DIR_IN);
			mt_set_gpio_pull_enable(GPIO_SPI_MOSI_PIN, GPIO_PULL_DISABLE);
		}
#endif
	return 0;
}

static
void SPI_GPIO_Setup(struct board_info *db)
{
	mt_dm9051_pinctrl_init(db->spidev);
	SPI_GPIO_Set(1);
}
/* */
