/**********************************************************************************************************
 * @文件     Type_conversion.c
 * @说明     数据类型转化
 * @作者     YuyingJin
 * @网站     https://yuyingjin0111.github.io/
 * @日期     2018 ~
**********************************************************************************************************/
#include "Type_conversion.h"

void FloatToByte(float floatNum,unsigned char* byteArry)
{
	int i;
	char* pchar=(char*)&floatNum;
	for(i=0;i<sizeof(float);i++)
	{
		*byteArry=*pchar;
		pchar++;
		byteArry++;
	}
}


void IntToByte(int intNum,unsigned char* byteArry)
{
	int i;
	char* pchar=(char*)&intNum;
	for(i=0;i<sizeof(int);i++)
	{
		*byteArry=*pchar;
		pchar++;
		byteArry++;
	}
}


void UintToByte(unsigned int uintNum,unsigned char* byteArry)
{
	int i;
	char* pchar=(char*)&uintNum;
	for(i=0;i<sizeof(unsigned int);i++)
	{
		*byteArry=*pchar;
		pchar++;
		byteArry++;
	}
}


float Hex_To_Decimal(unsigned char *Byte,int num)
{
	int i;
	char cByte[4];
	for (i=0;i<num;i++)
	{
	cByte[i] = Byte[i];
	}
	float pfValue=*(float*)&cByte;
	return pfValue;
}


int Hex_To_Int(unsigned char *Byte,int num)
{
	int i;
	char cByte[2];
	for (i=0;i<num;i++)
	{
		cByte[i] = Byte[i];
	}
	short int pfValue=*(int*)&cByte;
	return pfValue;
}


unsigned int Hex_To_Uint(unsigned char *Byte,int num)
{
	int i;
	char cByte[2];
	for (i=0;i<num;i++)
	{
		cByte[i] = Byte[i];
	}
	unsigned int pfValue=*(unsigned int*)&cByte;
	return pfValue;
}


void arrycat(u8 *dst,u8 index,u8 *src,u8 len)
{
	u8 i=0;
	for(i=0;i<len;i++)
	{
		*(dst+index+i)=*(src+i);
	}
}


float Asc_to_f(volatile unsigned char *str)
{
  signed char temp,flag1,flag2; 
  float value,count;
  flag1 = 1;
  flag2 = 0;
  value = 0;
  count = 1;
  temp = *str;
  while(((*str>='0')&&(*str<='9'))||(*str=='-')||(*str=='.')) 
  { 
			temp=*str++;
			if(temp=='-')
			{ 
					if(flag1)
					flag1=-1;
					else
					return(0x00); 
			}
			else if(temp=='.')
			{ 
				  flag2=1;	  
	    }
		  else
			{ 
		      value=value*10+(temp&0x0f);
          if(flag2)
		    	count*=0.1f;
		  }
  }
  value*=count*flag1; 
  return(value);
}


