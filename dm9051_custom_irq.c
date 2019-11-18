
#ifdef DM_CONF_INTERRUPT
#if DRV_INTERRUPT_1	

#ifdef MTK_CONF_YES	
static unsigned int mt_dm9051_irq_no(void)
{
	#define IRQ_NODE_NAME "mediatek,mt6735-spi_int"
	unsigned int irq_no= 0; //int ret;
	u32 ints[2] = {0, 0};
	static unsigned int gpiopin, debounce;
	struct device_node *irq_node;
		irq_node = of_find_compatible_node(NULL, NULL, IRQ_NODE_NAME);

	if (irq_node) {
		of_property_read_u32_array(irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		gpiopin = ints[0];
		debounce = ints[1];
		
		irq_no = irq_of_parse_and_map(irq_node, 0); //spi->irq
		
		printk("[USB_CHECK]:  switch_data->irq = %d, gpioin = %d, debounce = %d\n", irq_no, ints[0], ints[1]);
		
		if (irq_no) {
			printk("[USB_CHECK]: irq_of_parse_and_map fail!!\n");
	//		goto err_detect_irq_num_failed;
		}
	
	} else {
			printk("[USB_CHECK]: usb_id_enable_pin --null irq node!!\n");
		//	goto err_detect_irq_node_failed;
	}
	return irq_no;
}
#endif

static unsigned int request_irq_no(void)
{
	unsigned int irq_no;
	
	#ifdef MTK_CONF_YES	
	  irq_no= mt_dm9051_irq_no();
	#else
	  irq_no= DM_CONF_INTERRUPT_IRQ;	
	#endif	
	
	printk("[DBG] getting dm9051 request irq %d\n", irq_no);
	return irq_no;
}

#endif
#endif