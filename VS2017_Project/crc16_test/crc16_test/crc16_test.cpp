// crc16_test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"

void check_crc_func(void);

uint8 cfg_table0[] =
{
	0xff,
	0xfe,
	0x14,
	0x02,
	0x52,
};


int main()
{
	check_crc_func();

	uint16 crc_base = 0xFFFF;
	uint16 cur_crc_val;

	cur_crc_val = xchip_crc_check16(crc_base, cfg_table0, 5);

	cout << "crc_result :" << hex<<cur_crc_val << endl;
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门提示: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件

































/* ======================================================================================================================= */

#if 0
#define CHECK_NUM			 0x1189

uint8 xchip_get_max_bit(uint16 val)
{
	uint8 i = 0;
	while (val > 1)
	{
		val >>= 1;
		i++;
	}

	return i;
}


uint16 xchip_check_CRC_positive(uint8 i)
{
	uint8 val = i;
	uint8 step = 0;
	uint16 calc_val;

	if (val == 0)
	{
		return 0;
	}

	while (val > 0)
	{
		uint8 bit_i = xchip_get_max_bit(val);

		val &= ~(1 << bit_i);   //去掉最高位

		val = val ^ (CHECK_NUM >> (16 - bit_i));  //除去最高位之后 异或 CHECK_NUM

		//	printf("bit_i = 0x%x , val=0x%x\n", bit_i, val);

		if (step == 0)
		{
			calc_val = (CHECK_NUM << bit_i) & 0xFFFF;
		}
		else
		{
			calc_val ^= (CHECK_NUM << bit_i) & 0xFFFF;
		}

		step++;
	}

	return calc_val;
}
#endif

/* ======================================================================================================================= */