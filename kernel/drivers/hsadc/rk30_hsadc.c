/* drivers/hsadc/chips/rk30_hsadc.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
*/
#include <linux/blkdev.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/stat.h>
#include <linux/delay.h>
#include <linux/irq.h>

#include <mach/board.h>
#include <mach/io.h>
#include <mach/gpio.h>
#include <asm/dma.h>
#include <mach/dma-pl330.h>
#include <asm/scatterlist.h>

#include <mach/cru.h>

#include "rk30_hsadc.h"

struct hsadc_client {
	int time;
	int result;
	void (*callback)(struct hsadc_client *, void *, int);
	void *callback_param;

	struct hsadc_host_t *hsadc_host;
};

struct hsadc_request {
	int result;
	void (*callback)(struct hsadc_client *, void *, int);
	void *callback_param;
	struct hsadc_client *client;
	/* Used in case of sync requests */
	struct completion completion;
};
struct hsadc_host_t;
struct hsadc_ops {
	void (*start)(struct hsadc_host_t *);
	void (*stop)(struct hsadc_host_t *);
	int (*read)(struct hsadc_host_t *);
};
	
struct hsadc_host_t {
	struct device *dev;
	int is_suspended;
	struct mutex queue_mutex;
	struct hsadc_client *cur;
	const struct hsadc_ops *ops;
	unsigned long		private[0];
};

static inline void *hsadc_priv(struct hsadc_host_t *hsadc_host)
{
	return (void *)hsadc_host->private;
}

struct rk29_hsadc_device {
	int			 		irq;
	struct clk *		clk_ahb;
	struct clk *		clk_hsadc;
	resource_size_t  reg_start;	
	void __iomem		*regs;	
	struct resource		*ioarea;
	enum dma_ch              dmach;	
	dma_addr_t		dma_addr;
	struct resource		*dma_res;
	struct hsadc_host_t		*hsadc_host;
};

static struct rk29_dma_client rk29_dma_hsadc_client = {
        .name = "rk29-dma-hsadc",
};
static struct hsadc_host_t *g_hsadc = NULL;

//#define RK29_HSADC_IRQ_REQUEST

//debug config
static int rk29_hsadc_debug = 1;
#define dprintk if (rk29_hsadc_debug) printk

extern int hsadc_dma_kmalloc_buf_head; 

static void rk29_hsadc_dma_cleanup(struct hsadc_host_t *hsadc_host)
{
	struct rk29_hsadc_device *hsadc_dev  = hsadc_priv(hsadc_host);
}

static void rk29_hsadc_stop_dma(struct hsadc_host_t *hsadc_host)
{
	struct rk29_hsadc_device *hsadc_dev  = hsadc_priv(hsadc_host);
	
	rk29_dma_ctrl(hsadc_dev->dmach, RK29_DMAOP_STOP);
	rk29_hsadc_dma_cleanup(hsadc_host);
}

/* This function is called by the DMA driver from tasklet context. */
static void rk29_hsadc_dma_complete(void *arg, int size, enum rk29_dma_buffresult result)  ///(int chn, dma_irq_type_t type, void *arg)
{
	struct rk29_hsadc_device *hsadc_dev  = hsadc_priv(g_hsadc);
	unsigned int			i, dma_len=0, *src, *dest;
	int ret = 0;
	dma_addr_t		dest_dma_addr;;	

#if 0			
	 dest_dma_addr = virt_to_dma(NULL, hsadc_dma_kmalloc_buf_head);
    	 ret = rk29_dma_enqueue(hsadc_dev->dmach, g_hsadc, dest_dma_addr, 1024*2048);  
#endif
	dprintk("%s: hsadc dma complete!!!!!!!!!!!!!!!!!!!\n",__FUNCTION__);
}

static void rk29_hsadc_start(struct hsadc_host_t *hsadc_host)
{
	struct rk29_hsadc_device *hsadc_dev  = hsadc_priv(hsadc_host);
	unsigned int			i, dma_len=0, *src, *dest;
	int ret = 0,temp;
	dma_addr_t		dest_dma_addr;

      //select hsadc_ext clock
	temp = readl(RK30_CRU_BASE + 0x9c);		
	temp = (temp & ~(0x30 | (0x3<< 20)));
       writel(temp | (0x2<< 4) | (0x3<< 20), RK30_CRU_BASE + 0x9c);
	dsb();	
			
	/* HSADC control register */
	iowrite32((HSADC_ALMOST_FULL_LEVEL80 | HSADC_ALMOST_EMPTY_LEVEL32 | HSADC_SELECT_MPEG_TRANSPORT_STREAM_INPUT | HSADC_SINGLE_CHANNEL_DATA_INPUT
	| HSADC_VALID_INTERFACE_ENABLE  | HSADC_SYNC_INTERFACE_ENABLE
                   | HSADC_STORE_MODE_8BIT | HSADC_ALMOST_FULL_DMA_REQUEST | HSADC_MOST_SIGNIFICANT_BIT_NOT_NEGATION | HSADC_STORE_TO_LOW_8_10_BIT
                   | HSADC_DATA_BUS_WIDTH_8_BIT | HSADC_NOT_SELECT_GPS | HSADC_INTERFACE_UNIT_ENABLE),  hsadc_dev->regs + HSADC_CTRL);
       /* HSADC interrupt enable/mask register */
	iowrite32((HSADC_EMPTY_INTERRUPT_DISABLE | HSADC_EMPTY_INTERRUPT_DISABLE),  hsadc_dev->regs + HSADC_IER);
       hsadc_dev->dma_addr = (hsadc_dev->reg_start + HSADC_DATA);
	dest_dma_addr = virt_to_dma(NULL, hsadc_dma_kmalloc_buf_head);
	
	memset(hsadc_dma_kmalloc_buf_head, 0, 1024*2048);	

       ret = rk29_dma_devconfig(hsadc_dev->dmach, RK29_DMASRC_HW, (unsigned long )(hsadc_dev->dma_addr));	   
	 ret = rk29_dma_config(hsadc_dev->dmach, 4, 1);
	 //ret = rk29_dma_set_buffdone_fn(hsadc_dev->dmach, rk29_hsadc_dma_complete);	 
	 ret = rk29_dma_setflags(hsadc_dev->dmach, RK29_DMAF_CIRCULAR);	//RK29_DMAF_AUTOSTART  	//RK29_DMAF_CIRCULAR	
        ret = rk29_dma_ctrl(hsadc_dev->dmach, RK29_DMAOP_FLUSH);
    	 ret = rk29_dma_enqueue(hsadc_dev->dmach, hsadc_host, dest_dma_addr, 1024*2048);  
	 ret = rk29_dma_ctrl(hsadc_dev->dmach, RK29_DMAOP_START);	

	dprintk("%s[%d]\n",__FUNCTION__,__LINE__);		
}

static void rk29_hsadc_stop(struct hsadc_host_t *hsadc_host)
{
	struct rk29_hsadc_device *hsadc_dev  = hsadc_priv(hsadc_host);
	/* HSADC control register */	
	iowrite32((HSADC_INTERFACE_UNIT_DISABLE),  hsadc_dev->regs + HSADC_CTRL);
       /* HSADC interrupt enable/mask register */
	iowrite32((HSADC_EMPTY_INTERRUPT_DISABLE | HSADC_FULL_INTERRUPT_DISABLE),  hsadc_dev->regs + HSADC_IER);   
}

static u32 rk29_hsadc_read(struct hsadc_host_t *hsadc_host)
{
	struct rk29_hsadc_device *hsadc_dev  = hsadc_priv(hsadc_host);
       dma_addr_t src, dst,addr;
	 unsigned int hsadc_isr = 0, hsadc_ier = 0, hsadc_data = 0, hsadc_ctrl = 0;

	rk29_dma_getposition(hsadc_dev->dmach, &src, &dst);
	
       addr = dma_to_virt(NULL, dst);	
	   	   
#if 0  
       /* inputerrupt status register */	
	hsadc_isr = ioread32(hsadc_dev->regs + HSADC_ISR);	
	hsadc_ier = ioread32(hsadc_dev->regs + HSADC_IER);	
	//hsadc_data = readl(hsadc_dev->regs + HSADC_DATA);	
	hsadc_ctrl = ioread32(hsadc_dev->regs + HSADC_CTRL);       	
        dprintk("%s[%d]:dst dma addr:%x,  hsadc CTRL register: %x, hsadc ISR register: %x, IER register: %x\n",__FUNCTION__,__LINE__, addr, hsadc_ctrl, hsadc_isr, hsadc_ier);		
#endif
       //dprintk("hsadc_dev->dmach:%x---",  hsadc_dev->dmach);

	return addr;
}

//------------------------------------------
int rk29_hsadc_start_transmit(void)
{
    rk29_hsadc_start(g_hsadc);
	
    return 0;
}
EXPORT_SYMBOL(rk29_hsadc_start_transmit);

int rk29_hsadc_stop_transmit(void)
{
    int i;
    int diff_flag = 0;	
    char *data_addr = hsadc_dma_kmalloc_buf_head;
    char data;
	
    rk29_hsadc_stop(g_hsadc);
    rk29_hsadc_stop_dma(g_hsadc);

    dprintk( "%s[%d]: !!!!!!!!!!!!!!the hsadc data!!!!!!!!!!!!!!!!!!\n",__FUNCTION__,__LINE__);		
    for(i = 0; i < 200; i++)
    {
        data = data_addr[i+1024];
        dprintk( "0x%x ",  data);	       
    }
	
    return 0;
}
EXPORT_SYMBOL(rk29_hsadc_stop_transmit);

int rk29_hsadc_get_cur_transmit_addr(void)
{
   int addr =0;
   
    addr = rk29_hsadc_read(g_hsadc);
	
    return addr;
}
EXPORT_SYMBOL(rk29_hsadc_get_cur_transmit_addr);
//------------------------------------------

static irqreturn_t rk29_hsadc_irq(int irq, void *data)
{
	struct rk29_hsadc_device *hsadc_dev = data;

	dprintk( "%s[%d]\n",__FUNCTION__,__LINE__);	
	
	return IRQ_HANDLED;
}
static const struct hsadc_ops rk29_hsadc_ops = {
	.start		= rk29_hsadc_start,
	.stop		= rk29_hsadc_stop,
	.read		= rk29_hsadc_read,
};

static struct hsadc_host_t *hsadc_alloc_host(int extra, struct device *dev)
{
	struct hsadc_host_t *hsadc_host;
	
	hsadc_host = kzalloc(sizeof(struct hsadc_host_t) + extra, GFP_KERNEL);
	if (!hsadc_host)
		return NULL;
	hsadc_host->dev = dev;
	g_hsadc = hsadc_host;
	
	return hsadc_host;
}

static void hsadc_free_host(struct hsadc_host_t *hsadc_host)
{
	kfree(hsadc_host);
	hsadc_host = NULL;
	return;
}

static int __devinit rk29_hsadc_probe(struct platform_device *pdev)
{
	struct hsadc_host_t *hsadc_host= NULL;
	struct rk29_hsadc_device *hsadc_dev;
	struct resource *res, *dma_res;
	int ret;

	dprintk("%s[%d]\n",__FUNCTION__,__LINE__);	

	hsadc_host = hsadc_alloc_host(sizeof(struct rk29_hsadc_device), &pdev->dev);
	if (!hsadc_host)
		return -ENOMEM;
	
	mutex_init(&hsadc_host->queue_mutex);
	hsadc_host->dev = &pdev->dev;
	hsadc_host->is_suspended = 0;
	hsadc_host->ops = &rk29_hsadc_ops;
	
	hsadc_dev= hsadc_priv(hsadc_host);
	hsadc_dev->hsadc_host = hsadc_host;
	
#ifdef RK29_HSADC_IRQ_REQUEST	
	//hsadc irq
	hsadc_dev->irq = platform_get_irq(pdev, 0);
	if (hsadc_dev->irq <= 0) {
		printk("failed to get hsadc irq\n");
		ret = -ENOENT;
		goto err_alloc;
	}

	ret = request_irq(hsadc_dev->irq, rk29_hsadc_irq, 0, pdev->name, NULL);
	if (ret < 0) {
		printk("failed to attach hsadc irq\n");
		goto err_alloc;
	}	
#endif	

	//hsadc ahb clock
	hsadc_dev->clk_ahb = clk_get(NULL, "hclk_hsadc");
	if (IS_ERR(hsadc_dev->clk_ahb)) {
		printk("failed to get hclk_hsadc clock\n");
		ret = PTR_ERR(hsadc_dev->clk_ahb);
		goto err_irq;
	}
	clk_enable(hsadc_dev->clk_ahb);	
	
       clk_set_parent(clk_get(NULL, "hsadc"), clk_get(NULL, "hsadc_ext"));	
	clk_set_parent(clk_get(NULL, "hsadc"), clk_get(NULL, "hsadc_ext"));	
	clk_set_parent(clk_get(NULL, "hsadc"), clk_get(NULL, "hsadc_ext"));	
	//hsadc  clock
	hsadc_dev->clk_hsadc= clk_get(NULL, "hsadc");
	if (IS_ERR(hsadc_dev->clk_hsadc)) {
		printk("failed to get hsadc clock\n");
		ret = PTR_ERR(hsadc_dev->clk_hsadc);
		goto err_ahb_clk;
	}
	clk_enable(hsadc_dev->clk_hsadc);	
	
       //hsadc register IO memory resource
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		printk("cannot find IO resource\n");
		ret = -ENOENT;
		goto err_clk;
	}
	hsadc_dev->reg_start = res->start;
	hsadc_dev->ioarea = request_mem_region(res->start, (res->end - res->start) + 1,  pdev->name);
	if(hsadc_dev->ioarea == NULL) {
		printk("cannot request IO\n");
		ret = -ENXIO;
		goto err_clk;
	}
	hsadc_dev->regs = ioremap(res->start, (res->end - res->start) + 1);
	if (!hsadc_dev->regs) {
		printk("cannot map IO\n");
		ret = -ENXIO;
		goto err_ioarea;
	}
	
       //hsadc dma    
	hsadc_dev->dma_res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (hsadc_dev->dma_res == NULL) {
		printk("Unable to get hsadc dma resource\n");
		ret = -ENOENT;
		goto err_iomap;
	}	
       hsadc_dev->dmach = hsadc_dev->dma_res->start + 0;   
	if(rk29_dma_request(hsadc_dev->dmach, &rk29_dma_hsadc_client, NULL) < 0) {
		printk("cannot request dma channel: %d \n", hsadc_dev->dmach);
		ret = -ENXIO;
		goto err_iomap;
	}	
	
	platform_set_drvdata(pdev, hsadc_dev);
	dprintk("rk29 hsadc: driver initialized\n");

	return 0;

 err_iomap:
	iounmap(hsadc_dev->regs);

 err_ioarea:
	release_mem_region(res->start,  (res->end - res->start) + 1);  //releae resource hsadc_dev->ioarea

 err_clk:
 	clk_put(hsadc_dev->clk_hsadc);
err_ahb_clk:	
	clk_put(hsadc_dev->clk_ahb);
			

 err_irq:
 #ifdef RK29_HSADC_IRQ_REQUEST		
	free_irq(hsadc_dev->irq, hsadc_dev);
 #endif

 err_alloc:
	hsadc_free_host(hsadc_dev->hsadc_host);
	
	return ret;
}

static int __devexit rk29_hsadc_remove(struct platform_device *pdev)
{
	struct rk29_hsadc_device *hsadc_dev = platform_get_drvdata(pdev);

       rk29_dma_free(hsadc_dev->dmach, &rk29_dma_hsadc_client);
	iounmap(hsadc_dev->regs);
	release_resource(hsadc_dev->ioarea);
	kfree(hsadc_dev->ioarea);
	clk_disable(hsadc_dev->clk_ahb);
	clk_put(hsadc_dev->clk_ahb);
	clk_disable(hsadc_dev->clk_hsadc);
	clk_put(hsadc_dev->clk_hsadc);		
 #ifdef RK29_HSADC_IRQ_REQUEST		
	free_irq(hsadc_dev->irq, hsadc_dev);
 #endif	
	hsadc_free_host(hsadc_dev->hsadc_host);

	return 0;
}

#ifdef CONFIG_PM
static int rk29_hsadc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rk29_hsadc_device *hsadc_dev = platform_get_drvdata(pdev);

	hsadc_dev->hsadc_host->is_suspended = 1;
	return 0;
}

static int rk29_hsadc_resume(struct platform_device *pdev)
{
	struct rk29_hsadc_device *hsadc_dev = platform_get_drvdata(pdev);

	hsadc_dev->hsadc_host->is_suspended = 0;
	return 0;
}

#else
#define rk29_hsadc_suspend NULL
#define rk29_hsadc_resume NULL

#endif

static struct platform_driver rk29_hsadc_driver = {
	.driver		= {
		.name	= "rk30-hsadc",
		.owner	= THIS_MODULE,
	},
	.probe		= rk29_hsadc_probe,
	.remove		= __devexit_p(rk29_hsadc_remove),
	.suspend	= rk29_hsadc_suspend,
	.resume		= rk29_hsadc_resume,
};

static int __init rk29_hsadc_init(void)
{
	return platform_driver_register(&rk29_hsadc_driver);
}

static void __exit rk29_hsadc_exit(void)
{
	platform_driver_unregister(&rk29_hsadc_driver);
}
int  rk29_pid_filter_ctrl(uint8_t onoff)
{

       return 0;		
}
EXPORT_SYMBOL(rk29_pid_filter_ctrl);

int rk29_pid_filter_set(uint8_t id, uint16_t pid, uint8_t onoff)
{
       return 0;	
}
EXPORT_SYMBOL(rk29_pid_filter_set);

module_init(rk29_hsadc_init);
module_exit(rk29_hsadc_exit);

MODULE_DESCRIPTION("Driver for HSADC");
MODULE_AUTHOR("aiyoujun, ayj@rock-chips.com");
MODULE_LICENSE("GPL");
