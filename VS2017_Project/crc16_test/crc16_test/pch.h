// 入门提示: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件

#ifndef PCH_H
#define PCH_H

#include <iostream>
#include <stdio.h>

using namespace std;

// TODO: 添加要在此处预编译的标头
typedef unsigned int	uint32;
typedef unsigned short	uint16;
typedef unsigned char	uint8;


extern const unsigned short crc16_table[256];


uint32 crc_check16(uint8* data, uint32 length);

uint32 xchip_crc_check16(unsigned short Input, uint8* data, uint32 length);


#endif //PCH_H
