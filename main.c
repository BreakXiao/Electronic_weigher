/* ΢С�Ŵ���
   PA4 ��AD��ȡ��
   PA5 ��DA����ڣ�������
LCD1000:   
PB 15 ��SDIN
PB12 ��SYNC
PB10 ��RCON
PB13�� SCLK
*/
#include "MyFONT.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_spi.h"
#include <stdio.h>
#include "LCD.h"
#include "math.h"
#include "WAVEDAT.h" 
#include "ldc1000.h"

#define KEY_RAM        (*((volatile unsigned short *) 0x60070000))     //�ⲿ��ַ�Ǵ�0x60000000��ʼ��
#define SPI_RWBIT 		0x80 
#define uchar unsigned char
#define ulong unsigned long
#define uint unsigned int
#define SRAM            (*((volatile unsigned short *) 0x60000000))  
#define PAGE_RAM        (*((volatile unsigned short *) 0x60001000))  
#define DAC_RAM         (*((volatile unsigned short *) 0x60002000))  
#define ADC_RAM         (*((volatile unsigned short *) 0x60003000))


#define SDA_IN() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;} //PB9����
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9���
#define IIC_SCL0 GPIO_ResetBits(GPIOB, GPIO_Pin_8)  
#define IIC_SCL1 GPIO_SetBits(GPIOB, GPIO_Pin_8)    
#define IIC_SDA0 GPIO_ResetBits(GPIOB, GPIO_Pin_9)  
#define IIC_SDA1 GPIO_SetBits(GPIOB, GPIO_Pin_9)
#define READ_SDA GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_9)


#define HOME_PAGE 0
#define SET_PRICE 1
#define SET_K 2
#define I2C1_SLAVE_ADDRESS7    0x90
#define pass asm("nop")
#define True 1
#define False 0
#define not !
#define or ||
#define and &&
#define elif else if
#define Queue_N 15


GPIO_InitTypeDef           GPIO_InitStructure;
DAC_InitTypeDef            DAC_InitStructure;
ADC_InitTypeDef            ADC_InitStructure;
TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
NVIC_InitTypeDef           NVIC_InitStructure;
ADC_CommonInitTypeDef      ADC_CommonInitStructure;

RCC_ClocksTypeDef          RCC_ClockFreq;
ErrorStatus                HSEStartUpStatus;

u8  posbit=0x00;
u8  posbit1=0x00;
u8  posbit2=0x00;
u8 posbit3=0x00;
u16 DACDAT;
u8 keysign=0;
u8 keycode;
u32 count;
u16 DA_count;
u8  sindat[256];
char temp[3];
u8 k;
u16 timer,tdata;
unsigned char num;
unsigned char amp;
u32 ans1;
u32 K;//ת��ϵ��


struct float_num
{
   uint data; //��ֵ
   u8 point_n; //С������
   int data_n; //λ��
};
uint   t_count;
uint queue[Queue_N];
u8 price_num[10],price_n,price_point;
u8 Mode,kflag;
u16 kcount;
int data;
u16 adc_num,ten,q_last;
u8 q_front,q_flag,q_num;
ulong q_sum;
  
  

struct float_num weight,price,cost,sum_cost,qupi,show_weight;
void RCC_Configuration(void);/*ϵͳʱ�ӳ�ʼ��*/
void GPIO_Configuration(void);
void EXTI_init(void);
void TIM1INT_init(void);
void TIM2INT_init(void);
void TIM1_init(void);
void TIM2_init(void);
void DAC_init(void);
void ADC_init(void);
void delay_ms(volatile u16 time);
void SysClk_Init(u32 clock);
u16 Get_Adc1(u8 ch);
uint ten_pow(u8 t);
void show_view();//��ʾ��ʼ����
void all_init();//�ܳ�ʼ������
void delay(int x);
void delay_us(volatile u16 time);
uint Read_AD();
void Write_AD();
uint get_ad_result(int time);
void I2C_Configuration(void);
void show_float(u16 x,u16 y,struct float_num f);
void asm_delay(uint time)
{
  uint i;
  for(i=0;i<time/2;i++)
    asm("nop");
}
u8 get_data_n(ulong data);
uint change(uint num,u16 flag);
void date_init();
u16 kn;
uint change_2(uint num,u16 flag);
uint adjust()
{
    ulong q_s=0;
    uint i,j;
    for(j=0;j<10;j++)
    {
       q_sum=0;
       i=50;
       while(i--)
       {
         adc_num=Read_AD();
         q_sum+=adc_num;
       }    
       q_sum/=50;
       q_s+=q_sum;
    }
    q_s/=10;
     return ((uint)(q_s));
    
}
uint adjust_num;
uint add_num,add_number;
u8 qupi_flag;
uint m[6]={5,10,20,50,100,200};
float km[8];
u16 set_k_flag;

//change();
int main(void)
{
  int i=0;
  u16 cha;
  u16 flag;
  uint add_jiange;
  date_init();
  all_init(); //��ʼ���ܺ���
  qupi_flag=0;
  LCD_ShowChineseStringBig(100,150,5,CYAN,str_jiaozhun); 
  adjust_num=adjust();
   weight.data=0;
    t_count=0;
  q_last=0;
  add_num=0;
  add_number=0;
  weight.point_n=show_weight.point_n=1;//С����λ��
  add_jiange=25;   
  show_view();//��ʾ��ҳ
  kn=0;
  set_k_flag=0;
  
  while(1)
  {   
    
    
     if(count>=900&&Mode==SET_K)
     {
          q_sum=0;
          adc_num=Read_AD();
          queue[q_front]=adc_num;
          q_front=(q_front+1)%Queue_N;
          
          for(i=0;i<Queue_N;i++)
            q_sum+=queue[i];
          q_sum/=Queue_N;
          
          adc_num=(uint)(q_sum);
          
          if(adc_num<adjust_num)
              adc_num=0;
          else
             adc_num-=adjust_num;
          
         LCD_ShowStringBig(30,100,"           ",BLACK);
          LCD_ShowNumBig(30,100,adc_num,WHITE);
         LCD_ShowNumBig(30,50,m[kn],WHITE);
         count=0;
    }    
    
    
    if(count>=2500&&Mode==HOME_PAGE)
    {
        count=0;        
        adc_num=Read_AD();//��ȡAD 
        //��ȥУ׼ֵ����С��0��Ϊ0
        if(adc_num<adjust_num)
          adc_num=0;
        else
          adc_num-=adjust_num;
         
        //ȷ����ֵ�����Ǹ�
        if(adc_num>q_last)
        {
            cha=adc_num-q_last;
            flag=1;
        }
        else 
        {
            cha=q_last-adc_num;
             flag=0;
        }
        
        //����ֵ����ֵ�ڣ�����Ϊ�Ƕ����������г���
        if(cha<26)
        {
          q_last=adc_num;
          continue;
        }
       
        //����ʼ����
        //��ʾ������......
       // LCD_ShowChineseStringBig(20+3*32,80,4,CYAN,str_chenliang); 
        delay_ms(4000);
      //  LCD_ShowStringBig(20,80,"               ",BLACK);
       
       
        //�ٲ�һ�Σ�ȡƽ��ֵ
        q_sum=0;
        for(i=0;i<Queue_N;i++)
        {
            adc_num=Read_AD();
            if(adc_num<adjust_num)
               adc_num=0;
            else
               adc_num-=adjust_num;
            q_sum+=adc_num;
            delay_ms(50);
        }
        adc_num=(u16)(q_sum/Queue_N);
         
        //�ٴμ���ֵ��С
        if(adc_num>q_last)
        {
            cha=adc_num-q_last;  
            flag=1;
        }
        else
        {
          cha=q_last-adc_num;  
          flag=0;
        }
        
        
        //����С��˵���Ƕ���������
        if(cha<26)
        {
          q_last=adc_num;
          continue;
        }
         
        //��ʽ��ʼ������

        //�����ϴ�ֵ
        q_last=adc_num;

         
        //����ǰ�����Ϣ���ı�������ֵ
        if(flag==1)
        {     
          //3s�в�����Ư����ֵ��С�����е���         
          if(adc_num<1000)
          cha=cha;
          else if(adc_num<4000)
            cha+=1;
          else if(adc_num<10000)
            cha+1;
          else cha+=1;
          weight.data+=change(cha,flag);
        }
        else
        {
         //3s�в�����Ư����ֵ��󣬽��е���
          if(adc_num>6000)
            cha-=1;
          if(weight.data>change(cha,flag))
            weight.data-=change(cha,flag);
          else weight.data=0;
        }
        
        //����С����Ϊ����ģ�������У׼
        if(weight.data<80&&flag==0)
        {
             LCD_ShowChineseStringBig(40,50,5,CYAN,str_jiaozhun); 
             adjust_num=adjust();
             weight.data=0;
             show_view();//��ʾ��ҳ
             add_num=0;
        }
        
        //��ʾ����
        weight.data_n=get_data_n(weight.data);
        if(weight.data_n<=weight.point_n)
          weight.data_n+=weight.point_n-weight.data_n+1; 
      
        LCD_ShowStringBig(20+3*32,250,"           ",BLACK);
        if(qupi_flag==0)
        {
           show_float(20+3*32,250,weight);
           LCD_ShowChineseStringBig(20+3*32+(weight.data_n+2)*16,250,4,CYAN,str_ke);
           show_weight=weight;
        }
        else
        {
           show_weight=weight;
           if(weight.data>qupi.data)
              show_weight.data=weight.data-qupi.data;
           else show_weight.data=0;
           
           show_weight.data_n=get_data_n(show_weight.data);
           if(show_weight.data_n<=show_weight.point_n)
              show_weight.data_n+=show_weight.point_n-show_weight.data_n+1;            
           show_float(20+3*32,250,show_weight);
           LCD_ShowChineseStringBig(20+3*32+(show_weight.data_n+2)*16,250,4,CYAN,str_ke);         
        }
        
        
        LCD_ShowStringBig(20+3*32,150,"           ",BLACK);
        data=price.data*show_weight.data;
        cost.data=data/ten_pow(price.point_n+show_weight.point_n-2);
        cost.data_n=get_data_n(cost.data);
        cost.point_n=2;  //��ʾ2λС��
        if(cost.data_n<=cost.point_n)
          cost.data_n+=cost.point_n-cost.data_n+1;    
        show_float(20+3*32,150,cost);
        LCD_ShowChineseStringBig(20+3*32+(cost.data_n+2)*16,150,1,CYAN,str_yuan);             
    }
    
    if(keysign==1)
    {
        keysign=0;
      	switch(keycode)
	{
          case 0x00:         					//'1'��
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=1;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'1',WHITE);
                 price_n++;
              }            
              break;	
          }
          case 0x01:         					//'2'��
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=2;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'2',WHITE);
                 price_n++;
              }            
              break;	            	
          }
          case 0x02:         					//'3'��
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=3;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'3',WHITE);
                 price_n++;
              }            
              break;	
          }
          case 0x03:         					//'����'��
          {
              if(Mode==HOME_PAGE)
              {
                Mode=SET_PRICE;
                LCD_Clear1(BLACK);
                LCD_ShowChineseStringBig(20,200,3,YELLOW,str_danjia);
                price_n=0;
                price_point=100;
               
              }
     
              break;	
          }
          case 0x04:         					//'4'��
          {    
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=4;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'4',WHITE);
                 price_n++;
              }             
              break;	
          }
          case 0x05:         					//'5'��
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=5;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'5',WHITE);
                 price_n++;
              }             
              break;	
          }
          case 0x06:         					//'6'��
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=6;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'6',WHITE);
                 price_n++;
              }             
              break;	
          }
          case 0x07:         					//'ȥƤ'��
          {
             if(Mode==HOME_PAGE)
             {
               if(qupi_flag==0)
               {
                  qupi=weight;
                  if(qupi.data>1000)
                  {
                     LCD_ShowChineseStringBig(40,50,10,BRED,str_tishi);
                     delay_ms(1000);
                     LCD_ShowStringBig(40,50,"                 ",BLACK);
                     qupi.data=0;
                     continue;
                  }
                  qupi_flag=1;
                  show_weight.data=0;
                 
               }
               else 
               {
                 qupi.data=0;
                 qupi_flag=0;
                 show_weight=weight;
                }
                 
                 show_weight.data_n=get_data_n(show_weight.data);
                 if(show_weight.data_n<=show_weight.point_n)
                   show_weight.data_n+=show_weight.point_n-show_weight.data_n+1;
                 LCD_ShowStringBig(20+3*32,250,"           ",BLACK);
                 show_float(20+3*32,250,show_weight);
                 LCD_ShowChineseStringBig(20+3*32+(show_weight.data_n+2)*16,250,4,CYAN,str_ke); 
                 LCD_ShowStringBig(20+3*32,150,"           ",BLACK);
                 data=price.data*show_weight.data;
                 cost.data=data/ten_pow(price.point_n+show_weight.point_n-2);
                 cost.data_n=get_data_n(cost.data);
                 cost.point_n=2;  //��ʾ2λС��
                 if(cost.data_n<=cost.point_n)
                    cost.data_n+=cost.point_n-cost.data_n+1;    
                show_float(20+3*32,150,cost);
                LCD_ShowChineseStringBig(20+3*32+(cost.data_n+2)*16,150,1,CYAN,str_yuan); 
               /***ȥƤ��ʾ***/
               LCD_ShowStringBig(20+3*32,10,"            ",BLACK);
               LCD_ShowChineseStringBig(20,10,4,YELLOW,str_qupi);    
               show_float(20+4*32,10,qupi);
                LCD_ShowChineseStringBig(20+4*32+(qupi.data_n+2)*16,10,1,CYAN,str_ke);
                break;                
             }
            
          }
          case 0x08:         					//'7'��
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=7;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'7',WHITE);
                 price_n++;
              }    
              break;	
          }
          case 0x09:         					//'8'��
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=8;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'8',WHITE);
                 price_n++;
              }    
              break;	
          }
          case 0x0a:         					//'9'��
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=9;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'9',WHITE);
                 price_n++;
              }    
              break;	
          }
          case 0x0b:                                     //'�ۼӼ�'
          {
             if(Mode==HOME_PAGE)
             {
                sum_cost.data+=cost.data;
                sum_cost.data_n=get_data_n(sum_cost.data);
                if(sum_cost.data_n<=sum_cost.point_n)
                  sum_cost.data_n+=sum_cost.point_n-sum_cost.data_n+1;    
                show_view();
             }
             break;	
          }
          case 0x0c:         					//'0'��
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=0;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'0',WHITE);
                 price_n++;
              }    
              break;	
          }
          case 0x0d:         					//'.'��
          {
              if(Mode==SET_PRICE)
             {
                 price_point=price_n;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'.',WHITE);
                 price_n++;
              }
              else if(Mode==HOME_PAGE)
              {
                 kn=0;
                 Mode=SET_K;
                 count=0;
                 LCD_Clear1(BLACK);
              }
              break;	
          }
          case 0x0e:         					//'ȷ��'��
          {
             if(Mode==SET_PRICE)
             {
                price.data=0;
                ten=1;
                if(price_point!=100)
                price.point_n=price_n-1-price_point;
                else price.point_n=0;
                
                for(i=price_n-1;i>=0;i--)
                {
                   if(i==price_point)
                   {
                     price_n--;
                     continue;
                   }
                   price.data+=price_num[i]*ten;
                   ten*=10;
                }
                price.data_n=price_n;
                
                show_view();
                Mode=HOME_PAGE;
                
              } 
              else if(Mode==HOME_PAGE)
              {
                  LCD_ShowChineseStringBig(40,50,5,CYAN,str_jiaozhun); 
                  adjust_num=adjust();
                  weight.data=0;
                  show_view();//��ʾ��ҳ
                  add_num=0;
              }
              else if(Mode==SET_K)
              {
                  km[kn]=((float)(m[kn]))/((float)(adc_num));
                  if(kn<3)
                    km[kn]+=0.000075;
                  else if(kn<4)
                    km[kn]+=0.000062;
                  else km[kn]+=0.000045;
                  kn++;
                  LCD_ShowNumBig(30,200,(u32)(km[kn-1]*10000),WHITE);
                  delay_ms(3000);
                  LCD_ShowStringBig(20,80,"          ",BLACK);           
                  if(kn==6)
                   {
                      km[kn]=km[5]-0.000005;
                      Mode=HOME_PAGE;
                      show_view();
                      set_k_flag=1;
                   }
                    continue;               
              }
              break;	
          }
          case 0x0f:         					//'���'��
          {
             if(Mode==SET_PRICE)
             {
                LCD_Clear1(BLACK);
                LCD_ShowChineseStringBig(20,200,3,YELLOW,str_danjia);
                price_n=0;
              }
              else if(Mode==HOME_PAGE)
              {
                sum_cost.data=0;
                sum_cost.data_n=3;
                show_view();
              }
              break;	
          }
        }
    }
  }
}
void date_init()
{
  u16 i;
  kflag=kcount=0;
  posbit=0;
  Mode=0;
  for(i=0;i<Queue_N;i++)
  queue[i]=0;
  q_front=0;
  q_flag=0;
  data=205;
  weight.data=100;
  weight.data_n=3;
  weight.point_n=1;
  
  price.data=100;
  price.data_n=get_data_n(data);
  price.point_n=2;
  
  sum_cost.data=0;
  sum_cost.data_n=3;
  sum_cost.point_n=2;
  
  qupi.data=0;
  qupi.data_n=3;
  qupi.point_n=2;
}
/*********************************************
               �ܳ�ʼ������
**********************************************/
void all_init()
{
  LCD_Init();                                                   //Һ����ʼ��
  EXTI_init();                                                  //�ⲿ�жϳ�ʼ��
  EXTI_ClearITPendingBit(EXTI_Line1);
  GPIO_Configuration();  //I/O��ʼ�� 
  I2C_Configuration();
  Write_AD();
  TIM2_init();
  TIM2INT_init(); 
  count=0;
  num=0;
  keysign=0; 
}


//----------------------------------------------------------------------------
//ADֵ�����֮���ת��
//----------------------------------------------------------------------------

uint change_2(uint num,u16 flag)
{
    float f_num;
  f_num=((float)(num));
    if(f_num<350)  //5g
       f_num*=km[0]-(1-flag)*0.000004;
     else if(f_num<700)   // 10g
       f_num*=km[1]-(1-flag)*0.000004;
     else if(f_num<1500)  //20g
       f_num*=km[2]-(1-flag)*0.000004;
     else if(f_num<2800)  //50g
       f_num*=km[3]-(1-flag)*0.000004;
     else if(f_num<5700)  //100g
       f_num*=km[4]-(1-flag)*0.000004;  
     else if (f_num<11500) //200g
       f_num*=km[5]-(1-flag)*0.000004;
     else f_num*=km[6]-(1-flag)*0.000004;  //500g  
     
     
     f_num*=10;
     return ((int)(f_num));       
}

uint change(uint num,u16 flag)
{
  //Km=;
  float f_num;
  if(set_k_flag==1)
    return change_2(num,flag);
  
  
  f_num=((float)(num));
   if(flag==1)  //����
   {
     
     if(f_num<350)  //5g
       f_num*=0.01985;
     else if(f_num<700)   // 10g
       f_num*=0.01965;
     else if(f_num<1500)  //20g
       f_num*=0.019605;
     else if(f_num<2800)  //50g
       f_num*=0.01953;
     else if(f_num<5700)  //100g
       f_num*=0.019499;  
     else if(f_num<7800) //150g 
       f_num*=0.01948;
     else if (f_num<11500) //200g
       f_num*=0.019525; 
     else f_num*=0.01958;  //500g
   }
   else 
   {
     if(f_num<70)
       f_num*=0.01950;
     else if(f_num<350)   //5g 
     {
       f_num*=0.01975;
     }
     else if (f_num < 700)  //10g
       f_num*=0.01960;
     else if(f_num<1500)  //20g
       f_num*=0.01960;
     else if(f_num<2800)  //50g
       f_num*=0.01960;
     else if(f_num<5700)  //100g
       f_num*=0.01960;  
     else if(f_num<7800) //150g 
       f_num*=0.01960;
     else if(f_num<11500)  //200g
       f_num*=0.01960; 
     else f_num*=0.01960;  //500g
   }
  f_num*=10;
  return ((int)(f_num));
}



//----------------------------------------------------------------------------
//�������ֵĳ���
//----------------------------------------------------------------------------
u8 get_data_n(ulong data)
{
  u8 i=0;
  while(data!=0)
  {
    data/=10;
    i++;
  }
  return i;
}


//----------------------------------------------------------------------------
//����10��t�η�
//----------------------------------------------------------------------------
uint ten_pow(u8 t)
{
  u8 i;
  uint ans=1;
  for(i=0;i<t;i++)
    ans*=10;
  return ans;
}

//----------------------------------------------------------------------------
//��ʾ�׽���
//----------------------------------------------------------------------------
void show_view()
{

    weight.data_n=get_data_n(weight.data);

    
    /***��������ʾ���ο�***/
    LCD_Clear1(BLACK);
    LCD_Show_Rect(10,90,350,200,GREEN);    
    LCD_Show_Rect(10,5,350,40,GREEN);
    LCD_Show_Rect(410,5,50,285,WHITE);
    LCD_Show_Rect(405,5,60,285,WHITE);
    
    
    LCD_ShowChineseStringBig(420,12+60*4,1,RED,str_jian);
    LCD_ShowChineseStringBig(420,12+60*3,1,BRRED,str_yi);
    LCD_ShowChineseStringBig(420,12+60*2,1,GBLUE,str_dian);        
    LCD_ShowChineseStringBig(420,12+60,1,BRED,str_zi);    
    LCD_ShowChineseStringBig(420,12,1,GRED,str_chen);
    
    /***������ʾ***/
     LCD_ShowChineseStringBig(20,250,3,YELLOW,str_weigh);
   // show_float(20+3*32,250,weight);
    //LCD_ShowChineseStringBig(20+3*32+(weight.data_n+2)*16,250,4,CYAN,str_ke);
        
       weight.data_n=get_data_n(weight.data);
       if(weight.data_n<=weight.point_n)
          weight.data_n+=weight.point_n-weight.data_n+1; 
      
        LCD_ShowStringBig(20+3*32,250,"           ",BLACK);
        if(qupi_flag==0)
        {
           show_float(20+3*32,250,weight);
           LCD_ShowChineseStringBig(20+3*32+(weight.data_n+2)*16,250,4,CYAN,str_ke);
           show_weight=weight;
        }
        else
        {
           show_weight=weight;
           if(weight.data>qupi.data)
              show_weight.data=weight.data-qupi.data;
           else show_weight.data=0;
           
           show_weight.data_n=get_data_n(show_weight.data);
           if(show_weight.data_n<=show_weight.point_n)
              show_weight.data_n+=show_weight.point_n-show_weight.data_n+1;            
           show_float(20+3*32,250,show_weight);
           LCD_ShowChineseStringBig(20+3*32+(show_weight.data_n+2)*16,250,4,CYAN,str_ke);         
        }
        
        
        
    /***������ʾ***/
    LCD_ShowChineseStringBig(20,200,3,YELLOW,str_danjia);
    show_float(20+3*32,200,price);
    LCD_ShowChineseStringBig(20+3*32+(price.data_n+2)*16,200,3,CYAN,str_yuanke);  
    
    /***�ܼ���ʾ***/
    LCD_ShowChineseStringBig(20,150,3,YELLOW,str_zongjia);
    data=price.data*show_weight.data;
    cost.data=data/ten_pow(price.point_n+show_weight.point_n-2);
    cost.data_n=get_data_n(cost.data);
    cost.point_n=2;  //��ʾ2λС��
    if(cost.data_n<=cost.point_n)
      cost.data_n+=cost.point_n-cost.data_n+1;    
    show_float(20+3*32,150,cost);
    LCD_ShowChineseStringBig(20+3*32+(cost.data_n+2)*16,150,1,CYAN,str_yuan);
    

    /***�ۼӼ���ʾ***/
    LCD_ShowChineseStringBig(20,100,4,YELLOW,str_leijia);    
    show_float(20+4*32,100,sum_cost);
    LCD_ShowChineseStringBig(20+4*32+(sum_cost.data_n+2)*16,100,1,CYAN,str_yuan);
    
    
    /***ȥƤ��ʾ***/
    LCD_ShowChineseStringBig(20,10,4,YELLOW,str_qupi);    
    show_float(20+4*32,10,qupi);
    LCD_ShowChineseStringBig(20+4*32+(qupi.data_n+2)*16,10,1,CYAN,str_ke);
}

//----------------------------------------------------------------------------
//��ʾ������
//----------------------------------------------------------------------------
void show_float(u16 x,u16 y,struct float_num f)
{
    uint num;
    int i;

    if(f.data_n==0)
       LCD_ShowCharBig(x,y,'0',WHITE);
    
    for(i=f.data_n-1;i>=0;i--)
    {
         num=f.data%10;
         f.data/=10;
         if(i>=f.data_n-f.point_n)
            LCD_ShowCharBig(x+(i+1)*16,y,'0'+num,WHITE);
         else LCD_ShowCharBig(x+i*16,y,'0'+num,WHITE);
         
         if(i==f.data_n-f.point_n)
           LCD_ShowCharBig(x+i*16,y,'.',WHITE);
    }

}

//----------------------------------------------------------------------------
//���ADCֵ
//ch��ͨ��0~16
//����ת�����
//----------------------------------------------------------------------------
u16 Get_Adc1(u8 ch)
{
  ADC_RegularChannelConfig(ADC1,ch,1,ADC_SampleTime_480Cycles);
  ADC_SoftwareStartConv(ADC1);
  while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
  return ADC_GetConversionValue(ADC1);
}

void delay(int x)
{
  int i,j;
  for (i=0;i<x;i++)
    for(j=0;j<100;j++)
      ;
}


//----------------------------------------------------------------------------
//��ʱ��2�жϷ������
//----------------------------------------------------------------------------
void TIM2_IRQHandler(void)    //TIM2�ж�
{
   uint adc_num; 
   if(TIM_GetITStatus (TIM2,TIM_IT_Update)!=RESET)
   {
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
    if(Mode==HOME_PAGE||Mode==SET_K)
      count++;
    if(kflag==1)
    {
      kcount++;
      if(kcount>=4000)
      {
        kflag=0;
        kcount=0;
      }
    }
  }
}  


void EXTI0_IRQHandler(void)	//�����ж�
{
     if(kcount==0)
     {
       kflag=1;
       keycode = KEY_RAM;
       keycode &= 0x0f;
       keysign = 1;
     }
     EXTI_ClearITPendingBit(EXTI_Line0);
}




void GPIO_Configuration(void)
{
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);      //ʹ��GPIOAʱ��
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);      //ʹ��GPIOBʱ��
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);      //ʹ��GPIOBʱ��
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);      //ʹ��GPIOEʱ��
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);      //ʹ��GPIODʱ��
      
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;                  //PA8���ù������������MCO)
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
      GPIO_Init(GPIOA, &GPIO_InitStructure);    
      
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;     //PC4��PC5����������
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
      GPIO_Init(GPIOC, &GPIO_InitStructure);    
      
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;                  //PE0����������
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
      GPIO_Init(GPIOE, &GPIO_InitStructure);
        
      
}


void EXTI_init(void)
{
	
  
      EXTI_InitTypeDef EXTI_InitStructure;
      NVIC_InitTypeDef NVIC_InitStructure;
      GPIO_InitTypeDef GPIO_InitStructure;
     
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //Enable GPIOb clock 
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);// Enable SYSCFG clock 

      
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
    
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
      GPIO_Init(GPIOB, &GPIO_InitStructure);     
      
      
      
      SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);/* Connect EXTI Line0 to PB pin */    
      EXTI_InitStructure.EXTI_Line = EXTI_Line0;/* Configure EXTI Line0 */
      EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
      EXTI_InitStructure.EXTI_LineCmd = ENABLE;
      EXTI_Init(&EXTI_InitStructure);
      EXTI_ClearFlag(EXTI_Line0);
      
      
      /* Enable and set EXTI Line0 Interrupt to the lowest priority */
      NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
      
      keycode = 0;
      keysign = 0;
}


void TIM1INT_init(void)
 /* Enable the TIM1 global Interrupt */
{ 
      NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
      TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);                      //��������ж�
}




void TIM2INT_init(void)
 /* Enable the TIM2 global Interrupt */
{ 
      NVIC_InitStructure.NVIC_IRQChannel =  TIM2_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
      TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);                      //��������ж�
}

void TIM1_init(void)
{
/* TIM1 Configuration */ 
  /*��ʱʱ��TOUT=(124+1)*(167+1)/168=125us*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);                  //ʹ��TIM1
      TIM_DeInit(TIM1);                                             //��λ��ʱ��1 
      TIM_TimeBaseStructure.TIM_Period=124;                         //�����Զ���װ�ؼĴ������ڵ�ֵ
      TIM_TimeBaseStructure.TIM_Prescaler=167*3/10;                       //����ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
      TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;         //����ʱ�ӷָ�TIM_CKD_DIV1=0x0000��
      TIM_TimeBaseStructure.TIM_RepetitionCounter=0x00;             //����RCR�Ĵ���ֵ
      TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;     //TIIM���ϼ���
      TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);                //��ʼ��TIM3
      TIM_Cmd(TIM1,ENABLE);                                         //����TIM1����
      TIM_ClearFlag(TIM1, TIM_FLAG_Update);                         //�������־ 
 }

void TIM2_init(void)
{
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);       //ʹ��TIM2 
      TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);               //TIM2 Configuration
      TIM_TimeBaseStructure.TIM_Period = 124;                       // ��ʱʱ��TOUT=(124+1)*(167+1)/168=125us  
      TIM_TimeBaseStructure.TIM_Prescaler = 167;       
      TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
      TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
      TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

      /* TIM2 TRGO selection */
      TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);      //�����¼���ѡΪ��������
  
      /* TIM2 enable counter */
      TIM_Cmd(TIM2, ENABLE); 
}


//----------------------------------------------------------------------------
//�Դ�I2C����
//----------------------------------------------------------------------------
void I2C_Configuration(void)
{
  I2C_InitTypeDef I2C_InitStructure;
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);      //ʹ��GPIOBʱ��
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);        //ʹ��I2C1    
      /* Configure I2C1 pins: SCL and SDA */
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//��©���
      GPIO_InitStructure. GPIO_PuPd=GPIO_PuPd_UP;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      
      GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_I2C1);
      GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_I2C1);
      
      
      //RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1 ,ENABLE);
     // RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1,DISABLE);
      
      /* I2C configuration */
      I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;//���� I2CΪ I2Cģʽ
      I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;//I2C����ģʽ Tlow / Thigh = 2
        
      I2C_InitStructure.I2C_OwnAddress1 = I2C1_SLAVE_ADDRESS7 ;//���õ�һ���豸��ַ

      I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;//ʹ��Ӧ��
      I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;//Ӧ�� 7λ��ַ
      I2C_InitStructure.I2C_ClockSpeed =1000;//����ʱ��Ƶ��

      /* I2C Peripheral Enable */
          /* Apply I2C configuration after enabling it */
      I2C_Init(I2C1, &I2C_InitStructure);
      I2C_Cmd(I2C1, ENABLE);//ʹ��I2C����
  
}
//----------------------------------------------------------------------------
//��ADS1100��д����ƼĴ���
//----------------------------------------------------------------------------
void Write_AD() 
{
    //  while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY)); 
     // asm_delay(1000);
      I2C_GenerateSTART(I2C1, ENABLE);                                          //����START����
      while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));               //�ȴ�ACK
     // asm_delay(1000);
      I2C_Send7bitAddress(I2C1, 0x90, I2C_Direction_Transmitter);                 //��ָ���Ĵ� I2C�豸���͵�ַ��,ѡ���ͷ���
      while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//�ȴ�ACK 
      //      asm_delay(1000);
      I2C_SendData(I2C1,0x8f);                                                 //����ADS1100�������15SPS��PGA=8
      while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));        //�������
      
      I2C_GenerateSTOP(I2C1, ENABLE);                                           //���� I2Cx���� STOP����
}

//----------------------------------------------------------------------------
//��ADS�ж�ȡ����
//----------------------------------------------------------------------------
uint Read_AD()
{
      u8 adcdath,adcdatl,adcdats;
      uint ADCdata;
    
      
      while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));                             //�ȴ�I2C
      I2C_GenerateSTART(I2C1, ENABLE);                                          //����START����
      while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));               //�ȴ�ACK
      
      I2C_Send7bitAddress(I2C1, 0x91, I2C_Direction_Receiver);                 //��ָ���Ĵ� I2C�豸���͵�ַ��,ѡ���ͷ���
      while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));    //�ȴ�ACK 
   
      I2C_AcknowledgeConfig(I2C1, ENABLE);                                     //ʹ�ܻ���ʧ��ָ�� I2C��Ӧ����
      while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)));
      adcdath= I2C_ReceiveData(I2C1);                                           //����ͨ�� I2Cx������յ�����
     // I2C_GenerateSTOP(I2C1, ENABLE);                                           //���� I2Cx���� STOP����
      
                                               
      while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)));
      adcdatl= I2C_ReceiveData(I2C1);                                           //����ͨ�� I2Cx������յ�����
      
      while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)));
      adcdats= I2C_ReceiveData(I2C1);                                           //����ͨ�� I2Cx������յ�����
      
      I2C_AcknowledgeConfig(I2C1, DISABLE);                                      //ʹ�ܻ���ʧ��ָ�� I2C��Ӧ����
      I2C_GenerateSTOP(I2C1, ENABLE);
      ADCdata=adcdath*256+adcdatl;                                      //ʹ�ܻ���ʧ��ָ�� I2C��Ӧ����

      
      return ADCdata;
}



void ADC_init()
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE); //ADC1��λ
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;//A4 is adc1 4 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;  
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA,&GPIO_InitStructure); 
  


  RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);
  
  
  ADC_CommonInitStructure.ADC_Mode=ADC_Mode_Independent;//����ģʽ
  ADC_CommonInitStructure.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_5Cycles;
                                                //���������׶�֮���ӳ�5��ʱ��
  ADC_CommonInitStructure.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled ;
                                                //DMA��ֹ
  ADC_CommonInitStructure.ADC_Prescaler=ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ
                                  //ADCCLK=PCLK2/4=84/4=21Mhz,���ɳ���36Mhz
  ADC_CommonInit(&ADC_CommonInitStructure); //ͨ�ó�ʼ��
  
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;  //12λ�ֱ���
  ADC_InitStructure.ADC_ScanConvMode =DISABLE;  //��ɨ��ģʽ
  ADC_InitStructure.ADC_ContinuousConvMode =DISABLE ;  //������ģʽ
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigInjecConvEdge_None ;
                                                  //��ֹ������⣬ʹ���������
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���
  ADC_InitStructure.ADC_NbrOfConversion =1 ;//1��ת���ڹ���������
  ADC_Init(ADC1, &ADC_InitStructure);  //ADC��ʼ��
  ADC_Cmd(ADC1,ENABLE);  //����ADת��

}


void DAC_init(void)
{
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);        //ʹ��DACʱ��
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;     //PA.4 DAC1���;PA.5DAC2���
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
      GPIO_Init(GPIOA,&GPIO_InitStructure);
      
      

      /* DAC channel2 Configuration */
     
      
      DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;                       //��ʹ�ô�������TEN1=0
      DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;         //��ʹ�ò��η���
      DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
      DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;          //DAC1�������ر�
      DAC_Init(DAC_Channel_2, &DAC_InitStructure);		                //��ʼ��DACͨ��2


      /* Enable DAC Channel2: Once the DAC channel2 is enabled, PA.05 is 
         automatically connected to the DAC converter. */
      DAC_Cmd(DAC_Channel_2, ENABLE);

      /* Set DAC dual channel DHR12RD register */
      DAC_SetDualChannelData(DAC_Align_12b_R, 0x100, 0x100);    
}


void delay_ms(volatile u16 time)
{
    volatile u16 i = 0;
    while (time--)
    {
      i = 12000;  //�Լ�����
    while (i--);
    }
}

void delay_us(volatile u16 time)
{
    volatile u16 i = 0;
    while (time--)
    {
      i = 12;  //�Լ�����
    while (i--);
    }
}



void SysClk_Init(u32 clock)
{
  SysTick_Config(clock);
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}






