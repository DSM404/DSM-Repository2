#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include  "MSCAN.h" 

unsigned char ASCII[10]="0123456789"; //ASCII���


long int wtime1=0,wtime2=0;            //�Ǵ�����ӡ
unsigned char CAN0SendData[8]={0,0,0,0,0,0,0,0};    //  CAN0�������ݻ�����                    
unsigned char CAN0RecdData[8]={0,0,0,0,0,0,0,0};	  //  CAN0�������ݻ�����
//unsigned short int hw_SendID=(0x520)<<1;  //CAN0����ID
unsigned short int hw_RecID;                //CAN0����ID������
/*��غ�������*/
void MCU_Init(void);
void AD_Init(void);
void SCI_Init(void);
void SCI_Send1(unsigned int o);


/*����motec���Ľṹ*/
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
	struct Motec_Data M84_Data;//MOTEC���Ľṹ��
	UINT8 Frames[22][8]; //CAN֡���棬ÿ֡8�ֽڣ���22֡	
}ECU_Data;

void main() 
{
   unsigned char w,q,b,s,g,nq,nb,ns,ng;//�ֱ�Ϊת�٣�5��,Ť�أ�4λ��
   unsigned long int Nm;
   unsigned long int ADresult,ADresult2; //AD2�� �ɼ���ѹ
   asm("sei");          //���ж�
   MCU_Init();
   AD_Init();
   SCI_Init();
   

   //CAN0_Init(BT_1M,FOUR_Filte,ON_IT,0x520,0xE8,0x520,0x520);//BT=1M,4��16λ������������IDΪ0x52,�������ж�
  // CAN4_Init(BT_1M,FOUR_Filte,ON_IT,0xE8,0xE8,0xE8,0xE8);//BT=1M,4��16λ������������IDΪ0xE8,�������ж� 
   CAN0_Init(BT_1M,FOUR_Filte,ON_IT,0xE8,0xE8,0xE8,0xE8);//BT=1M,4��16λ������������IDΪ0xE8,�������ж�
   asm("cli");           //���ж�
   for(;;) 
   {
    ATD0CTL5 = 0b00000010;     //.7=0,����� .6=0ת������޷��ţ�AN2ͨ��
    while(ATD0STAT0_SCF == 0)            //
    {
      if(ATD0STAT0_SCF == 1) 
      {
        
        ADresult=ATD0DR0;            //��AD���ݼĴ����ж�10λ����
        ADresult=(ADresult>>6);     //�������ݣ�ʹ��10λ��Ч
        break;
      }
    }
    

        ADresult2=(ADresult*4900)/1023;//���͵�pc����Ϊ1023��2^10-1��,��ǰ�ɼ��˶Եص�ѹΪ4.9V������ѹ��ʵ�ʵ�ѹ4.9V����Ϊǯλ����
                                    //ADresult2Ϊת����ʵ�ʵ�ѹֵ����ȷ��ǧ��λ(������)
        Nm=((ADresult2-2427)*411)/225; //NmΪŤ�أ����20kg��Ӧ��ѹ2880mv,0kg��Ӧ��ѹ2430mv,Ť��=20kg*9.8m/s^2*0.42m=82.32Nm,        
                                    //1kg=22.5mv=4.116Nm,��ȷ��ʮ��λ
    
   
     w=ASCII[ECU_Data.M84_Data.RPM/10000];
     q=ASCII[ECU_Data.M84_Data.RPM/1000%10];
     b=ASCII[ECU_Data.M84_Data.RPM/100%10];              //ת����ASCLL�� ת��
     s=ASCII[ECU_Data.M84_Data.RPM/10%10];
     g=ASCII[ECU_Data.M84_Data.RPM%10]; 
       
     nq=ASCII[Nm/1000%10];
     nb=ASCII[Nm/100%10];              //ת����ASCLL�� Ť��
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

       SCI_Send1(32);               //�ո�
       SCI_Send1(nq);
       SCI_Send1(nb);               //��ӡŤ��
       SCI_Send1(ns);
       SCI_Send1(ng);
       
       SCI_Send1(32);               //�ո�
       
       SCI_Send1(w);
       SCI_Send1(q);
       SCI_Send1(b);
       SCI_Send1(s);               //��ӡת��
       SCI_Send1(g);
       SCI_Send1(47);              //��ӡ��/��
       SCI_Send1(47);
       SCI_Send1(10);              //����

     }
   }
   
}

 //[MCU_Init.c]DG128оƬ��ʼ������-------------------------------------------*
//��  ��:                                                                  *
//    оƬ��ʼ������:ͨ������CLKSEL�Ĵ�����ȷ���ڲ����ߵ�ʱ��Դ;ͨ��PLL��  *
//    ��,����PLLCLK;ͨ������INTCR��COPCTL�Ĵ���,�ֱ�����Ƿ�����IRQ�жϺ�  *
//    ���Ź���                                                             *
//    (1)�ⲿ����=16Mhz BusClock=24Mhz                                     *
//    (2)��ֹIRQ�ж�                                                       *
//    (3)��ֹ���Ź�                                                        *
//��  ��:��                                                                *
//��  ��:��                                                                *
//-------------------------------------------------------------------------* 
void MCU_Init(void)
{
    //(2)CLKSEL�ĵ�7λ��0��ѡ��ϵͳʱ��ԴΪOSCCLK
      //��PLL����ִ��ǰ,�ڲ�����Ƶ��=OSCCLK/2
      //clksel.7(PLLѡ��λ)�����ڲ�����ʱ����Դ
      //=O,BusClock=OSCCLK/2; =1,BusClock=PLLCLK/2
    CLKSEL &= 0x7f;      //��ʱ:BusClock=OSCCLK/2
    //(3)��ֹPLL
    PLLCTL &= 0xbf;//PLLCTL.6(pllon)��Ϊ0;�ȹر�PLL
    //(4)������Ҫ��ʱ��Ƶ������SYNR��REFDV�Ĵ���
    //���㹫ʽ:PLLCLK=2*OSCCLK*((SYNR+1)/(REFDV+1))
    SYNR = 2;         //��PLLCLK��Ƶ������
    REFDV = 1;        //��PLLCLK��Ƶ������      ���໷ʱ��fBUSCLK=PLLCLK/2=OSCCLK*(SYNR+1)/(REFDV+1)=24MHz
        
    //(5)��PLL
    PLLCTL |= (1<<6);    //PLLCTL.6(pllon)��Ϊ1;��PLL
    //(6)ͨ���ж�CRGFLG�Ĵ�����LOCKλ��ȷ��PLL�Ƿ��ȶ� 
    while ((CRGFLG&0x08) == 0x00);
    //(7)ʱ��Ƶ���ȶ����������໷ʱ��Դ��Ϊϵͳʱ��Դ��
    CLKSEL |= (1<<7);    //����ִ�к�:BusClock=PLLCLK/2
    //(8)�����Ƿ�����IRQ�жϡ��Ƿ������Ź�
    INTCR &= 0xbf;       //IRQCR.6(IRQEN) =0��ֹIRQ�ж�(Ĭ�Ͽ�)
    COPCTL = 0x00;       //COPCTL.2-0(cr2:cr0) =000��ֹ���Ź�
}  

/*AD��ʼ��*/
/*����ֵ:��*/
/*2019.7.20*/
void AD_Init(void)
{    
    ATD0CTL2=0b11000000; //��adģ�� ���Զ�����
    ATD0CTL3=0b00001011; //���Ƚ��ȳ� ���г���1 �������� ֻ��һ��ͨ������һ��ת�� 
    //ATD0CTL4=0b00000011; //10λ���� ����ʱ��2xA/D clock ��PRS=3,��Ƶϵ��8,����fbus��Χ��4~16����fbus=24M,Ҳ�ܲɼ��ĵ�,why?  
                         // ATDclock=(fbus/(PRS+1)*0.5) adת��ʱ��=3MHz  minfbus=(PRS+1);maxfbus=4(PRS+1)
    ATD0CTL4=0b0000101; //10λ���� ����ʱ��2xA/D clock ��PRS=5,��Ƶϵ��8,����fbus��Χ��6~24��  
                         // ATDclock=(fbus/(PRS+1)*0.5) adת��ʱ��=2MHz  minfbus=(PRS+1);maxfbus=4(PRS+1)                       
}
/* SCI_Init:SCI��ʼ��*/
/*����ֵ:��*/
/*����:��ֹ�����жϣ�������Ϊ9600*/
/*����:��*/
/*2019.7.20
*/
void SCI_Init(void)
{
	unsigned char t;
	SCI0BDL=156;      //BT=24M/(16*156)=9600 
	SCI0BDH=0x00;		 //��ֹLIN�ϵ����ж�,��ֹRXD�������ж�
	SCI0CR1=0x00;		 //��������������żУ��
	t=SCI0DRL;			 //�����ݼĴ���(��0)
	t=SCI0SR1;			 //��״̬�Ĵ���1(��0)
	SCI0CR2=0x0C;		 //ʹ�ܷ�����������	
}
/*SCISend1:����һ���ֽ�*/
/*������o=Ҫ���͵�����*/
/*����:��*/
/*2019.7.20
*/
void SCI_Send1(unsigned int o) 
{
  if (o== '\n')  
  	{
      	while(!(SCI0SR1&0x80));     
      	SCI0DRL= 0x0A;       				 //����
	    return;
   	}  
	while(!(SCI0SR1&0x80)) ;//�������ݼĴ���Ϊ��
	SCI0DRL= o;//1�ֽڴ��뷢�ͼĴ���            
}
/*CAN�����жϷ������*/
//void interrupt VectorNumber_Vcan0rx CANRx_ISR(void)        isrCAN0Rec
#pragma CODE_SEG __NEAR_SEG NON_BANKED
__interrupt VectorNumber_Vcan0rx void CANR0_ISR(void)
{
  UINT8 Msg_Length;  //�������ݳ���
  UINT8 Index;       //������������
  UINT8 RxData[8];   //������
  UINT8 static Counter = 0;//֡������
  UINT8 i;
  
  hw_RecID=CAN0RXIDR0; //ȡ����ID�μĴ���
  hw_RecID=(hw_RecID<<3)|(CAN0RXIDR1>>5);    
  Msg_Length=(CAN0RXDLR & 0x0F);//����յ���֡����
  
  if(hw_RecID==0xE8)
  {  
    for(Index=0;Index<Msg_Length;Index++)//�洢�յ���֡
    RxData[Index]=*(&CAN0RXDSR0+Index);
    if(Counter>=22)     //֡�������ﵽ22�Զ����㣬����Խ��
    {
      Counter = 0;
    }
    //�ж��Ƿ�Ϊ����ͷ�������ǣ������������
    if(RxData[0] == 0x82 && RxData[1] == 0x81 && RxData[2] == 0x80)
    Counter = 0;
     for(i=0;i<8;i++)  //���յ���֡���뻺����
     {
      ECU_Data.Frames[Counter][i] = RxData[i];
     }
     Counter++;       //����������
     
  }
  CAN0RFLG_RXF = 1;//����жϱ�־ 
}
/*#pragma CODE_SEG __NEAR_SEG NON_BANKED
__interrupt VectorNumber_Vcan4rx void isrCAN4Rec(void) 
{
	
	CAN4RFLG_RXF = 1; //����жϱ�־  
} */
