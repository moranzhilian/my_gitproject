/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2010-03-29     Bernard      remove interrupt Tx and DMA Rx mode
 * 2012-02-08     aozima       update for F4.
 * 2012-07-28     aozima       update for ART board.
 */

#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"

#include <rtdevice.h>


/* ------------------------------------ */
/*   modify  usart2  */
#define UART2_GPIO_TX       GPIO_Pin_9
#define UART2_TX_PIN_SOURCE GPIO_PinSource9

#define UART2_GPIO_RX       GPIO_Pin_10
#define UART2_RX_PIN_SOURCE GPIO_PinSource10

#define UART2_GPIO          GPIOA

#define UART2_GPIO_RCC      RCC_AHB1Periph_GPIOA
#define RCC_APBPeriph_UART2 RCC_APB2Periph_USART1

#define UART2_TX_DMA        DMA1_Channel4
#define UART2_RX_DMA        DMA1_Channel5

/* ------------------------------------ */


/*******************************************************/
#define USARTx                             USART1

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define USARTx_CLK                         RCC_APB2Periph_USART1
#define USARTx_CLOCKCMD                    RCC_APB2PeriphClockCmd
#define USARTx_BAUDRATE                    115200  //串口波特率

#define USARTx_RX_GPIO_PORT                GPIOA
#define USARTx_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define USARTx_RX_PIN                      GPIO_Pin_10
#define USARTx_RX_AF                       GPIO_AF_USART1
#define USARTx_RX_SOURCE                   GPIO_PinSource10

#define USARTx_TX_GPIO_PORT                GPIOA
#define USARTx_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define USARTx_TX_PIN                      GPIO_Pin_9
#define USARTx_TX_AF                       GPIO_AF_USART1
#define USARTx_TX_SOURCE                   GPIO_PinSource9

/************************************************************/

///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(USARTx, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}


void USARTx_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
		
  RCC_AHB1PeriphClockCmd(USARTx_RX_GPIO_CLK|USARTx_TX_GPIO_CLK, ENABLE);

  /* 使能 USART 时钟 */
  USARTx_CLOCKCMD(USARTx_CLK, ENABLE);
  
  /* GPIO初始化 */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* 配置Tx引脚为复用功能  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin =  USARTx_TX_PIN  ;  
  GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);

  /* 配置Rx引脚为复用功能 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin =  USARTx_RX_PIN;
  GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);
  
 /* 连接 PXx 到 USARTx_Tx*/
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT,USARTx_RX_SOURCE,USARTx_RX_AF);

  /*  连接 PXx 到 USARTx__Rx*/
  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT,USARTx_TX_SOURCE,USARTx_TX_AF);
  
  /* 配置串DEBUG_USART 模式 */
  /* 波特率设置：DEBUG_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = USARTx_BAUDRATE;
  /* 字长(数据位+校验位)：8 */
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  /* 停止位：1个停止位 */
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* 校验位选择：不使用校验 */
  USART_InitStructure.USART_Parity = USART_Parity_Even;
  /* 硬件流控制：不使用硬件流 */
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  /* USART模式控制：同时使能接收和发送 */
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  /* 完成USART初始化配置 */
  USART_Init(USARTx, &USART_InitStructure); 
	
  /* 使能串口 */
  USART_Cmd(USARTx, ENABLE);
	
	while(1)
	{
		fputc('A');
	}
}


/* STM32 uart driver */
struct stm32_uart
{
    USART_TypeDef *uart_device;
    IRQn_Type irq;
};

static rt_err_t stm32_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct stm32_uart *uart;
    USART_InitTypeDef USART_InitStructure;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct stm32_uart *)serial->parent.user_data;

    if (cfg->baud_rate == BAUD_RATE_9600)
        USART_InitStructure.USART_BaudRate = 9600;
    else if (cfg->baud_rate == BAUD_RATE_115200)
        USART_InitStructure.USART_BaudRate = 115200;

    if (cfg->data_bits == DATA_BITS_8)
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;

    if (cfg->stop_bits == STOP_BITS_1)
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    else if (cfg->stop_bits == STOP_BITS_2)
        USART_InitStructure.USART_StopBits = USART_StopBits_2;

    USART_InitStructure.USART_Parity = USART_Parity_Even;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(uart->uart_device, &USART_InitStructure);

	
    /* Enable USART */
    USART_Cmd(uart->uart_device, ENABLE);

    return RT_EOK;
}

static rt_err_t stm32_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        UART_DISABLE_IRQ(uart->irq);
        /* disable interrupt */
        USART_ITConfig(uart->uart_device, USART_IT_RXNE, DISABLE);
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        UART_ENABLE_IRQ(uart->irq);
        /* enable interrupt */
        USART_ITConfig(uart->uart_device, USART_IT_RXNE, ENABLE);
        break;
    }

    return RT_EOK;
}

static int stm32_putc(struct rt_serial_device *serial, char c)
{
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;
	
		/* 发送一个字节数据到串口 */
		uart->uart_device->DR = c;
	
		/* 等待发送完毕 */
    while (!(uart->uart_device->SR & USART_FLAG_TXE));
    

    return 1;
}

static int stm32_getc(struct rt_serial_device *serial)
{
    int ch;
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    ch = -1;
    if (uart->uart_device->SR & USART_FLAG_RXNE)
    {
        ch = uart->uart_device->DR & 0xff;
    }

    return ch;
}

static const struct rt_uart_ops stm32_uart_ops =
{
    stm32_configure,
    stm32_control,
    stm32_putc,
    stm32_getc,
};




/* UART1 device driver structure */
struct stm32_uart uart1 =
{
    USART1,
    USART1_IRQn,
};
struct rt_serial_device serial1;

void USART1_IRQHandler(void)
{
    struct stm32_uart *uart;

    uart = &uart1;

    /* enter interrupt */
    rt_interrupt_enter();
    if (USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
        rt_hw_serial_isr(&serial1, RT_SERIAL_EVENT_RX_IND);
    }
    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}


static void RCC_Configuration(void)
{
    /* Enable UART GPIO clocks */
    RCC_AHB1PeriphClockCmd(UART2_GPIO_RCC, ENABLE);
    /* Enable UART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
}

static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* Configure USART2 Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = UART2_GPIO_RX | UART2_GPIO_TX;
    GPIO_Init(UART2_GPIO, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(UART2_GPIO, UART2_TX_PIN_SOURCE, GPIO_AF_USART2);
		
    GPIO_PinAFConfig(UART2_GPIO, UART2_RX_PIN_SOURCE, GPIO_AF_USART2);
}

static void NVIC_Configuration(struct stm32_uart *uart)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = uart->irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

int stm32_hw_usart_init(void)
{
    struct stm32_uart *uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
	
    RCC_Configuration();
    GPIO_Configuration();

    uart = &uart1;

    serial1.ops    = &stm32_uart_ops;
    serial1.config = config;

    NVIC_Configuration(&uart1);

    /* register UART1 device */
    rt_hw_serial_register(&serial1,
                          "uart2",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
		
    return 0;
}
INIT_BOARD_EXPORT(stm32_hw_usart_init);
