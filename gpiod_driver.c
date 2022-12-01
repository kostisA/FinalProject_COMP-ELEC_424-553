#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
/**
* Author: Konstantinos Alexopoulos
* Class: COMP 424 ELEC 424/553 001
* Final Project: Autonomous Car
* Fall 2022
*
* Interrupt code drawn from:
* https://github.com/Johannes4Linux/Linux_Driver_Tutorial/blob/main/11_gpio_irq/gpio_irq.c
**/

# include <linux/timekeeping.h>
# include <linux/ktime.h>




// Init variables
unsigned int irq_number;
static struct gpio_desc *my_enc;
static volatile u64 prev_time;
static volatile u64 curr_time;

// Interrupt Handler
static irq_handler_t gpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs) {
	printk("irq_handler_t: Button pressed & interrupt triggered!\n");
	

   
	// Ensure time difference between interrupts is greater than 1 ms 
	curr_time = ktime_get_ns();
	u64 elapsed_time = curr_time - prev;
	if (elapsed_time > 1000000) {
		prev_time = curr_time;
		printk("irq_handler - timing written\n");

	} else {
		printk("irq_handler - encoder timing too short!\n");
	}
	
	return (irq_handler_t) IRQ_HANDLED; 
}


// probe function 
static int enc_probe(struct platform_device *pdev)
{
	struct gpio_desc *my_btn;
	struct device *dev;
	dev = &pdev->dev;
	
	printk("enc_probe - RUNNING\n");

	// Get button
	my_btn = gpiod_get_index(dev, "userbutton", 0, GPIOD_IN);
	if(IS_ERR(my_btn)) {
		printk("enc_probe - could not gpiod_get_index 0 for btn\n");
		return -1;
	}
	
	// Setup interrupt & debounce
	irq_number = gpiod_to_irq(my_btn);
	gpiod_set_debounce(my_btn, 1000000);
	
	if(request_irq(irq_number, (irq_handler_t) gpio_irq_handler, IRQF_TRIGGER_RISING, "my_gpio_irq", NULL) != 0){
		printk("Error!\nCan not request interrupt nr.: %d\n", irq_number);
		return -1;
	}
	
	prev = ktime_get_ns();
	
	printk("enc_probe - SUCCESS\n");
	return 0;
}

// remove function
static int enc_remove(struct platform_device *pdev)
{
	free_irq(irq_number, NULL);
	printk("enc_remove - Freed interrupt & Removing\n");
	return 0;
}

static struct of_device_id matchy_match[] = {
    { .compatible = "hello", },
	{},
};

// platform driver object
static struct platform_driver adam_driver = {
	.probe	 = enc_probe,
	.remove	 = enc_remove,
	.driver	 = {
	       .name  = "The Rock: this name doesn't even matter",
	       .owner = THIS_MODULE,
	       .of_match_table = matchy_match,
	},
};
module_platform_driver(adam_driver);

MODULE_DESCRIPTION("424\'s finest");
MODULE_AUTHOR("GOAT");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:adam_driver");
