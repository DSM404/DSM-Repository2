#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include  "MSCAN.h" 

unsigned char ASCII[10]="0123456789"; //ASCII码表


long int wtime1=0,wtime2=0;            //记次数打印
unsigned char CAN0SendData[8]={0,0,0,0,0,0,0,0};    //  CAN0发送数据缓存区                    
unsigned char CAN0RecdData[8]={0,0,0,0,0,0,0,0};	  //  CAN0接收数据缓存区
//unsigned short int hw_SendID=(0x520)<<1;  //CAN0发送ID
unsigned short int hw_RecID;                //CAN0接收ID缓存区
/*相关函数声明*/
void MCU_Init(void);
void AD_Init(void);
void SCI_Init(void);
void SCI_Send1(unsigned int o);


/*定义motec报文结构*/
struct Motec_Data
{
	UINT8 Header1;
	UINT8 Header2;
	UINT8 Header3;
	UINT8 Data_Length;
	UINT16 RPM;
	UINT16 Throttle_Position;
	UINT16 Mainfold_Pressure;
	UINT16 Air_Temperature;
	UINT16 Engine_Temperature;
	UINT16 Lambda1;
	UINT16 Lambda2;
	UINT16 Exhaust_Mainfold_Pressure;
	UINT16 Mass_Air_Flow;
	UINT16 Fuel_Temperature;
	UINT16 Fuel_Pressure;
	UINT16 Oil_Temperature;
	UINT16 Oil_Pressure;
	UINT16 Gear_Voltage;
	UINT16 Knock_Voltage;
	UINT16 Gear_Shift_Force;
	UINT16 Exhaust_Gas_Temp_1;
	UINT16 Exhaust_Gas_Temp_2;
	UINT16 User_Channel_1;
	UINT16 User_Channel_2;
	UINT16 User_Channel_3;
	UINT16 User_Channel_4;
	UINT16 Battery_Volt;
	UINT16 ECU_Temperature;
	UINT16 Speed_1;
	UINT16 Speed_2;
	UINT16 Speed_3;
	UINT16 Speed_4;
	UINT16 Drive_Speed;
	UINT16 Ground_Speed;
	UINT16 Slip;
	UINT16 Aim_Slip;
	UINT16 Launch_RPM;
	UINT16 Lambda_1_Short_term_trim;
	UINT16 Lambda_2_Short_term_trim;
	UINT16 Lambda_1_Long_term_trim;
	UINT16 Lambda_2_Long_term_trim;
	UINT16 Aim_Lambda_1;
	UINT16 Aim_Lambda_2;
	UINT16 Fuel_Cut_Level;
	UINT16 Ignition_Cut_Level;
	UINT16 Ignition_Advance;
	UINT16 Load_Point;
	UINT16 Efficiency_Point;
	UINT16 Fuel_Used;
	UINT16 AUX_1;
	UINT16 AUX_2;
	UINT16 AUX_3;
	UINT16 AUX_4;
	UINT16 AUX_5;
	UINT16 AUX_6;
	UINT16 AUX_7;
	UINT16 AUX_8;
	UINT16 Fuel_Actual_Pulse_Width;
	UINT16 Fuel_Effective_Pulse_Width;
	UINT16 Fuel_Injector_Duty_Cycle;
	UINT16 Gear;
	UINT16 Sync_Position;
	UINT16 Fuel_Comp_1;
	UINT16 Fuel_Comp_2;
	UINT16 DEG_1;
	UINT16 DEG_2;
	UINT16 DEG_3;
	UINT16 DEG_4;
	UINT16 DEG_5;
	UINT16 DEG_6;
	UINT16 DEG_7;
	UINT16 DEG_8;
	UINT16 DEG_9;
	UINT16 DEG_10;
	UINT16 DEG_11;
	UINT16 DEG_12;
	UINT16 DEG_13;
	UINT16 DEG_14;
	UINT16 DEG_15;
	UINT16 DEG_16;
	UINT16 SFG_1;
	UINT16 SFG_2;
	UINT16 SFG_3;
	UINT16 SFG_4;
	UINT16 SFG_5;
	UINT16 SFG_6;
	UINT16 SFG_7;
	UINT16 SFG_8;
	UINT8 CRC_1;
	UINT8 CRC_2;
	UINT8 CRC_3;
	UINT8 CRC_4;
};
union 
{
	struct Motec_Data M84_Data;//MOTEC报文结构体
	UINT8 Frames[22][8]; //CAN帧缓存，每帧8字节，共22帧	
}ECU_Data;

void main() 
{
   unsigned char w,q,b,s,g,nq,nb,ns,ng;//分别为转速（5）,扭矩（4位）
   unsigned long int Nm;
   unsigned long int ADresult,ADresult2; //AD2： 采集电压
   asm("sei");          //关中断
   MCU_Init();
   AD_Init();
   SCI_Init();
   

   //CAN0_Init(BT_1M,FOUR_Filte,ON_IT,0x520,0xE8,0x520,0x520);//BT=1M,4个16位过滤器，过滤ID为0x52,开接收中断
  // CAN4_Init(BT_1M,FOUR_Filte,ON_IT,0xE8,0xE8,0xE8,0xE8);//BT=1M,4个16位过滤器，过滤ID为0xE8,开接收中断 
   CAN0_Init(BT_1M,FOUR_Filte,ON_IT,0xE8,0xE8,0xE8,0xE8);//BT=1M,4个16位过滤器，过滤ID为0xE8,开接收中断
   asm("cli");           //开中断
   for(;;) 
   {
    ATD0CTL5 = 0b00000010;     //.7=0,左对齐 .6=0转换结果无符号；AN2通道
    while(ATD0STAT0_SCF == 0)            //
    {
      if(ATD0STAT0_SCF == 1) 
      {
        
        ADresult=ATD0DR0;            //从AD数据寄存器中读10位数据
        ADresult=(ADresult>>6);     //调整数据，使低10位有效
        break;
      }
    }
    

        ADresult2=(ADresult*4900)/1023;//发送到pc数据为1023（2^10-1）,当前采集端对地电压为4.9V（最大电压，实际电压4.9V，因为钳位，）
                                    //ADresult2为转换后实际电压值，精确到千分位(即毫伏)
        Nm=((ADresult2-2427)*411)/225; //Nm为扭矩，测得20kg对应电压2880mv,0kg对应电压2430mv,扭矩=20kg*9.8m/s^2*0.42m=82.32Nm,        
                                    //1kg=22.5mv=4.116Nm,精确到十分位
    
   
     w=ASCII[ECU_Data.M84_Data.RPM/10000];
     q=ASCII[ECU_Data.M84_Data.RPM/1000%10];
     b=ASCII[ECU_Data.M84_Data.RPM/100%10];              //转化成ASCLL码 转速
     s=ASCII[ECU_Data.M84_Data.RPM/10%10];
     g=ASCII[ECU_Data.M84_Data.RPM%10]; 
       
     nq=ASCII[Nm/1000%10];
     nb=ASCII[Nm/100%10];              //转化成ASCLL码 扭矩
     ns=ASCII[Nm/10%10];
     ng=ASCII[Nm%10]; 
     wtime1++;
     if(wtime1>2000) 
     {
       wtime2++;
     }
     if(wtime2>1500) 
     { 
       wtime1=0;
       wtime2=0;

       SCI_Send1(32);               //空格
       SCI_Send1(nq);
       SCI_Send1(nb);               //打印扭矩
       SCI_Send1(ns);
       SCI_Send1(ng);
       
       SCI_Send1(32);               //空格
       
       SCI_Send1(w);
       SCI_Send1(q);
       SCI_Send1(b);
       SCI_Send1(s);               //打印转速
       SCI_Send1(g);
       SCI_Send1(47);              //打印‘/’
       SCI_Send1(47);
       SCI_Send1(10);              //换行

     }
   }
   
}

 //[MCU_Init.c]DG128芯片初始化函数-------------------------------------------*
//功  能:                                                                  *
//    芯片初始化设置:通过设置CLKSEL寄存器，确定内部总线的时钟源;通过PLL编  *
//    程,设置PLLCLK;通过设置INTCR和COPCTL寄存器,分别决定是否允许IRQ中断和  *
//    看门狗。                                                             *
//    (1)外部晶振=16Mhz BusClock=24Mhz                                     *
//    (2)禁止IRQ中断                                                       *
//    (3)禁止看门狗                                                        *
//参  数:无                                                                *
//返  回:无                                                                *
//-------------------------------------------------------------------------* 
void MCU_Init(void)
{
    //(2)CLKSEL的第7位置0，选择系统时钟源为OSCCLK
      //在PLL程序执行前,内部总线频率=OSCCLK/2
      //clksel.7(PLL选择位)决定内部总线时钟来源
      //=O,BusClock=OSCCLK/2; =1,BusClock=PLLCLK/2
    CLKSEL &= 0x7f;      //此时:BusClock=OSCCLK/2
    //(3)禁止PLL
    PLLCTL &= 0xbf;//PLLCTL.6(pllon)设为0;先关闭PLL
    //(4)根据需要的时钟频率设置SYNR和REFDV寄存器
    //计算公式:PLLCLK=2*OSCCLK*((SYNR+1)/(REFDV+1))
    SYNR = 2;         //对PLLCLK增频的因子
    REFDV = 1;        //对PLLCLK分频的因子      锁相环时钟fBUSCLK=PLLCLK/2=OSCCLK*(SYNR+1)/(REFDV+1)=24MHz
        
    //(5)打开PLL
    PLLCTL |= (1<<6);    //PLLCTL.6(pllon)设为1;开PLL
    //(6)通过判断CRGFLG寄存器的LOCK位，确定PLL是否稳定 
    while ((CRGFLG&0x08) == 0x00);
    //(7)时钟频率稳定后，允许锁相环时钟源作为系统时钟源；
    CLKSEL |= (1<<7);    //本句执行后:BusClock=PLLCLK/2
    //(8)设置是否允许IRQ中断、是否允许看门狗
    INTCR &= 0xbf;       //IRQCR.6(IRQEN) =0禁止IRQ中断(默认开)
    COPCTL = 0x00;       //COPCTL.2-0(cr2:cr0) =000禁止看门狗
}  

/*AD初始化*/
/*返回值:无*/
/*2019.7.20*/
void AD_Init(void)
{    
    ATD0CTL2=0b11000000; //打开ad模块 打开自动清零
    ATD0CTL3=0b00001011; //非先进先出 序列长度1 立即冻结 只对一个通道进行一次转换 
    //ATD0CTL4=0b00000011; //10位精度 采样时间2xA/D clock ，PRS=3,分频系数8,允许fbus范围（4~16）而fbus=24M,也能采集的到,why?  
                         // ATDclock=(fbus/(PRS+1)*0.5) ad转换时钟=3MHz  minfbus=(PRS+1);maxfbus=4(PRS+1)
    ATD0CTL4=0b0000101; //10位精度 采样时间2xA/D clock ，PRS=5,分频系数8,允许fbus范围（6~24）  
                         // ATDclock=(fbus/(PRS+1)*0.5) ad转换时钟=2MHz  minfbus=(PRS+1);maxfbus=4(PRS+1)                       
}
/* SCI_Init:SCI初始化*/
/*返回值:无*/
/*功能:禁止接收中断，波特率为9600*/
/*参数:无*/
/*2019.7.20
*/
void SCI_Init(void)
{
	unsigned char t;
	SCI0BDL=156;      //BT=24M/(16*156)=9600 
	SCI0BDH=0x00;		 //禁止LIN断点检测中断,禁止RXD输入活动边中断
	SCI0CR1=0x00;		 //正常启动，无奇偶校验
	t=SCI0DRL;			 //读数据寄存器(清0)
	t=SCI0SR1;			 //读状态寄存器1(清0)
	SCI0CR2=0x0C;		 //使能发送器接收器	
}
/*SCISend1:发送一个字节*/
/*参数：o=要发送的数据*/
/*参数:无*/
/*2019.7.20
*/
void SCI_Send1(unsigned int o) 
{
  if (o== '\n')  
  	{
      	while(!(SCI0SR1&0x80));     
      	SCI0DRL= 0x0A;       				 //换行
	    return;
   	}  
	while(!(SCI0SR1&0x80)) ;//发送数据寄存器为空
	SCI0DRL= o;//1字节存入发送寄存器            
}
/*CAN接收中断服务程序*/
//void interrupt VectorNumber_Vcan0rx CANRx_ISR(void)        isrCAN0Rec
#pragma CODE_SEG __NEAR_SEG NON_BANKED
__interrupt VectorNumber_Vcan0rx void CANR0_ISR(void)
{
  UINT8 Msg_Length;  //接受数据长度
  UINT8 Index;       //接受数据索引
  UINT8 RxData[8];   //缓冲区
  UINT8 static Counter = 0;//帧计数器
  UINT8 i;
  
  hw_RecID=CAN0RXIDR0; //取报文ID段寄存器
  hw_RecID=(hw_RecID<<3)|(CAN0RXIDR1>>5);    
  Msg_Length=(CAN0RXDLR & 0x0F);//获得收到的帧长度
  
  if(hw_RecID==0xE8)
  {  
    for(Index=0;Index<Msg_Length;Index++)//存储收到的帧
    RxData[Index]=*(&CAN0RXDSR0+Index);
    if(Counter>=22)     //帧计数器达到22自动清零，避免越界
    {
      Counter = 0;
    }
    //判断是否为报文头部，若是，这计数器清零
    if(RxData[0] == 0x82 && RxData[1] == 0x81 && RxData[2] == 0x80)
    Counter = 0;
     for(i=0;i<8;i++)  //将收到的帧存入缓冲区
     {
      ECU_Data.Frames[Counter][i] = RxData[i];
     }
     Counter++;       //计数器自增
     
  }
  CAN0RFLG_RXF = 1;//清除中断标志 
}
/*#pragma CODE_SEG __NEAR_SEG NON_BANKED
__interrupt VectorNumber_Vcan4rx void isrCAN4Rec(void) 
{
	
	CAN4RFLG_RXF = 1; //清除中断标志  
} */
