#include <rtthread.h>
#include <bsp_led.h>

/* ================================================================================================ */
/*  静态线程的 线程堆栈*/
static rt_uint8_t led1_stack[512];

/* 静态线程的 线程控制块 */
static struct rt_thread led1_thread;



/* ================================================================================================ */
void static_thread_entry(void* parameter);
void dynamic_thread_entry(void* parameter);
extern void cpu_usage_init(void);
extern void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor);
/* ================================================================================================ */
void demo_thread_creat(void)
{
    rt_err_t result;
    /* 动态线程的 线程控制块指针 */
    rt_thread_t led2_thread;

    LED_GPIO_Config();
		cpu_usage_init();
	
    /* 创建静态线程 ： 优先级 20 ，时间片 2个系统滴答 */
    result = rt_thread_init(&led1_thread,
                            "led1",
                            static_thread_entry, RT_NULL,
                            (rt_uint8_t*)&led1_stack[0], sizeof(led1_stack), 20, 8);
    if (result == RT_EOK)
    {
        rt_thread_startup(&led1_thread);
    }

		
    /* 创建动态线程 ： 堆栈大小512 bytes ，优先级 21 ，时间片 2个系统滴答 */
    led2_thread = rt_thread_create("led2",
                                   dynamic_thread_entry, RT_NULL,
                                   512, 20, 8);

    if (led2_thread != RT_NULL)
        rt_thread_startup(led2_thread);

}



void static_thread_entry(void* parameter)
{

    /* 无限循环*/
    while (1)
    {
        rt_kprintf(" 静态线程运行 \r\n");
				LED_RED;
//				rt_thread_delay( RT_TICK_PER_SECOND/2 ); /* 等待0.5s，让出cpu权限，切换到其他线程 */
				LED_GREEN;
//				rt_thread_delay( RT_TICK_PER_SECOND/2 ); /* 等待0.5s，让出cpu权限，切换到其他线程 */
				LED_BLUE;			
				rt_thread_yield();
    }
}

void dynamic_thread_entry(void* parameter)
{
		volatile rt_uint32_t count=0;
	  rt_uint8_t major, minor;
		char buf[20];
	
    while (1)
    {
        rt_kprintf(" 动态线程运行 \r\n");
				LED_RGBOFF;
				rt_thread_delay( RT_TICK_PER_SECOND*3); /* 等待0.5s，让出cpu权限，切换到其他线程 */
				
			  if (++count >= 4)
        {
            count = 0;
            cpu_usage_get(&major, &minor);

            rt_sprintf(buf,"%d.%d%%",major,minor);
            rt_kprintf(" CPU usage is: %s \r\n", buf);
        }
			
    }
}

