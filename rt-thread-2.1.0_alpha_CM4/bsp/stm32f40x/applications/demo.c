#include <rtthread.h>
#include <bsp_led.h>

/* ================================================================================================ */
/*  ��̬�̵߳� �̶߳�ջ*/
static rt_uint8_t led1_stack[512];

/* ��̬�̵߳� �߳̿��ƿ� */
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
    /* ��̬�̵߳� �߳̿��ƿ�ָ�� */
    rt_thread_t led2_thread;

    LED_GPIO_Config();
		cpu_usage_init();
	
    /* ������̬�߳� �� ���ȼ� 20 ��ʱ��Ƭ 2��ϵͳ�δ� */
    result = rt_thread_init(&led1_thread,
                            "led1",
                            static_thread_entry, RT_NULL,
                            (rt_uint8_t*)&led1_stack[0], sizeof(led1_stack), 20, 8);
    if (result == RT_EOK)
    {
        rt_thread_startup(&led1_thread);
    }

		
    /* ������̬�߳� �� ��ջ��С512 bytes �����ȼ� 21 ��ʱ��Ƭ 2��ϵͳ�δ� */
    led2_thread = rt_thread_create("led2",
                                   dynamic_thread_entry, RT_NULL,
                                   512, 20, 8);

    if (led2_thread != RT_NULL)
        rt_thread_startup(led2_thread);

}



void static_thread_entry(void* parameter)
{

    /* ����ѭ��*/
    while (1)
    {
        rt_kprintf(" ��̬�߳����� \r\n");
				LED_RED;
//				rt_thread_delay( RT_TICK_PER_SECOND/2 ); /* �ȴ�0.5s���ó�cpuȨ�ޣ��л��������߳� */
				LED_GREEN;
//				rt_thread_delay( RT_TICK_PER_SECOND/2 ); /* �ȴ�0.5s���ó�cpuȨ�ޣ��л��������߳� */
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
        rt_kprintf(" ��̬�߳����� \r\n");
				LED_RGBOFF;
				rt_thread_delay( RT_TICK_PER_SECOND*3); /* �ȴ�0.5s���ó�cpuȨ�ޣ��л��������߳� */
				
			  if (++count >= 4)
        {
            count = 0;
            cpu_usage_get(&major, &minor);

            rt_sprintf(buf,"%d.%d%%",major,minor);
            rt_kprintf(" CPU usage is: %s \r\n", buf);
        }
			
    }
}

