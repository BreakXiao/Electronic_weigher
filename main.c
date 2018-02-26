/* 微小放大器
   PA4 是AD读取口
   PA5 是DA输出口（方波）
LCD1000:   
PB 15 接SDIN
PB12 接SYNC
PB10 接RCON
PB13接 SCLK
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

#define KEY_RAM        (*((volatile unsigned short *) 0x60070000))     //外部地址是从0x60000000开始的
#define SPI_RWBIT 		0x80 
#define uchar unsigned char
#define ulong unsigned long
#define uint unsigned int
#define SRAM            (*((volatile unsigned short *) 0x60000000))  
#define PAGE_RAM        (*((volatile unsigned short *) 0x60001000))  
#define DAC_RAM         (*((volatile unsigned short *) 0x60002000))  
#define ADC_RAM         (*((volatile unsigned short *) 0x60003000))


#define SDA_IN() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;} //PB9输入
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9输出
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
u32 K;//转化系数


struct float_num
{
   uint data; //数值
   u8 point_n; //小数点数
   int data_n; //位数
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
void RCC_Configuration(void);/*系统时钟初始化*/
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
void show_view();//显示初始界面
void all_init();//总初始化函数
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
  all_init(); //初始化总函数
  qupi_flag=0;
  LCD_ShowChineseStringBig(100,150,5,CYAN,str_jiaozhun); 
  adjust_num=adjust();
   weight.data=0;
    t_count=0;
  q_last=0;
  add_num=0;
  add_number=0;
  weight.point_n=show_weight.point_n=1;//小数点位数
  add_jiange=25;   
  show_view();//显示首页
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
        adc_num=Read_AD();//读取AD 
        //减去校准值，若小于0则为0
        if(adc_num<adjust_num)
          adc_num=0;
        else
          adc_num-=adjust_num;
         
        //确定差值是正是负
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
        
        //若差值在阈值内，则认为是抖动，不进行称量
        if(cha<26)
        {
          q_last=adc_num;
          continue;
        }
       
        //否则开始称量
        //显示称量中......
       // LCD_ShowChineseStringBig(20+3*32,80,4,CYAN,str_chenliang); 
        delay_ms(4000);
      //  LCD_ShowStringBig(20,80,"               ",BLACK);
       
       
        //再测一次，取平均值
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
         
        //再次检测差值大小
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
        
        
        //若很小，说明是抖动，跳过
        if(cha<26)
        {
          q_last=adc_num;
          continue;
        }
         
        //正式开始称量！

        //更改上次值
        q_last=adc_num;

         
        //根据前面的信息，改变重量的值
        if(flag==1)
        {     
          //3s中产生温漂，差值变小，进行调整         
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
         //3s中产生温漂，差值变大，进行调整
          if(adc_num>6000)
            cha-=1;
          if(weight.data>change(cha,flag))
            weight.data-=change(cha,flag);
          else weight.data=0;
        }
        
        //若很小，且为减后的，则重新校准
        if(weight.data<80&&flag==0)
        {
             LCD_ShowChineseStringBig(40,50,5,CYAN,str_jiaozhun); 
             adjust_num=adjust();
             weight.data=0;
             show_view();//显示首页
             add_num=0;
        }
        
        //显示部分
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
        cost.point_n=2;  //显示2位小数
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
          case 0x00:         					//'1'键
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=1;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'1',WHITE);
                 price_n++;
              }            
              break;	
          }
          case 0x01:         					//'2'键
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=2;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'2',WHITE);
                 price_n++;
              }            
              break;	            	
          }
          case 0x02:         					//'3'键
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=3;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'3',WHITE);
                 price_n++;
              }            
              break;	
          }
          case 0x03:         					//'单价'键
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
          case 0x04:         					//'4'键
          {    
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=4;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'4',WHITE);
                 price_n++;
              }             
              break;	
          }
          case 0x05:         					//'5'键
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=5;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'5',WHITE);
                 price_n++;
              }             
              break;	
          }
          case 0x06:         					//'6'键
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=6;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'6',WHITE);
                 price_n++;
              }             
              break;	
          }
          case 0x07:         					//'去皮'键
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
                 cost.point_n=2;  //显示2位小数
                 if(cost.data_n<=cost.point_n)
                    cost.data_n+=cost.point_n-cost.data_n+1;    
                show_float(20+3*32,150,cost);
                LCD_ShowChineseStringBig(20+3*32+(cost.data_n+2)*16,150,1,CYAN,str_yuan); 
               /***去皮显示***/
               LCD_ShowStringBig(20+3*32,10,"            ",BLACK);
               LCD_ShowChineseStringBig(20,10,4,YELLOW,str_qupi);    
               show_float(20+4*32,10,qupi);
                LCD_ShowChineseStringBig(20+4*32+(qupi.data_n+2)*16,10,1,CYAN,str_ke);
                break;                
             }
            
          }
          case 0x08:         					//'7'键
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=7;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'7',WHITE);
                 price_n++;
              }    
              break;	
          }
          case 0x09:         					//'8'键
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=8;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'8',WHITE);
                 price_n++;
              }    
              break;	
          }
          case 0x0a:         					//'9'键
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=9;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'9',WHITE);
                 price_n++;
              }    
              break;	
          }
          case 0x0b:                                     //'累加键'
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
          case 0x0c:         					//'0'键
          {
             if(Mode==SET_PRICE)
             {
                 price_num[price_n]=0;
                 LCD_ShowCharBig(20+3*32+price_n*16,200,'0',WHITE);
                 price_n++;
              }    
              break;	
          }
          case 0x0d:         					//'.'键
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
          case 0x0e:         					//'确定'键
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
                  show_view();//显示首页
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
          case 0x0f:         					//'清除'键
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
               总初始化函数
**********************************************/
void all_init()
{
  LCD_Init();                                                   //液晶初始化
  EXTI_init();                                                  //外部中断初始化
  EXTI_ClearITPendingBit(EXTI_Line1);
  GPIO_Configuration();  //I/O初始化 
  I2C_Configuration();
  Write_AD();
  TIM2_init();
  TIM2INT_init(); 
  count=0;
  num=0;
  keysign=0; 
}


//----------------------------------------------------------------------------
//AD值与克数之间的转换
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
   if(flag==1)  //正向
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
//返回数字的长度
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
//返回10的t次方
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
//显示首界面
//----------------------------------------------------------------------------
void show_view()
{

    weight.data_n=get_data_n(weight.data);

    
    /***清屏，显示矩形框***/
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
    
    /***重量显示***/
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
        
        
        
    /***单价显示***/
    LCD_ShowChineseStringBig(20,200,3,YELLOW,str_danjia);
    show_float(20+3*32,200,price);
    LCD_ShowChineseStringBig(20+3*32+(price.data_n+2)*16,200,3,CYAN,str_yuanke);  
    
    /***总价显示***/
    LCD_ShowChineseStringBig(20,150,3,YELLOW,str_zongjia);
    data=price.data*show_weight.data;
    cost.data=data/ten_pow(price.point_n+show_weight.point_n-2);
    cost.data_n=get_data_n(cost.data);
    cost.point_n=2;  //显示2位小数
    if(cost.data_n<=cost.point_n)
      cost.data_n+=cost.point_n-cost.data_n+1;    
    show_float(20+3*32,150,cost);
    LCD_ShowChineseStringBig(20+3*32+(cost.data_n+2)*16,150,1,CYAN,str_yuan);
    

    /***累加价显示***/
    LCD_ShowChineseStringBig(20,100,4,YELLOW,str_leijia);    
    show_float(20+4*32,100,sum_cost);
    LCD_ShowChineseStringBig(20+4*32+(sum_cost.data_n+2)*16,100,1,CYAN,str_yuan);
    
    
    /***去皮显示***/
    LCD_ShowChineseStringBig(20,10,4,YELLOW,str_qupi);    
    show_float(20+4*32,10,qupi);
    LCD_ShowChineseStringBig(20+4*32+(qupi.data_n+2)*16,10,1,CYAN,str_ke);
}

//----------------------------------------------------------------------------
//显示浮点数
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
//获得ADC值
//ch：通道0~16
//返回转换结果
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
//定时器2中断服务程序
//----------------------------------------------------------------------------
void TIM2_IRQHandler(void)    //TIM2中断
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


void EXTI0_IRQHandler(void)	//键盘中断
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
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);      //使能GPIOA时钟
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);      //使能GPIOB时钟
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);      //使能GPIOB时钟
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);      //使能GPIOE时钟
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);      //使能GPIOD时钟
      
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;                  //PA8复用功能推挽输出（MCO)
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
      GPIO_Init(GPIOA, &GPIO_InitStructure);    
      
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;     //PC4和PC5推拉输出输出
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
      GPIO_Init(GPIOC, &GPIO_InitStructure);    
      
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;                  //PE0推拉输出输出
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
      TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);                      //允许溢出中断
}




void TIM2INT_init(void)
 /* Enable the TIM2 global Interrupt */
{ 
      NVIC_InitStructure.NVIC_IRQChannel =  TIM2_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
      TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);                      //允许溢出中断
}

void TIM1_init(void)
{
/* TIM1 Configuration */ 
  /*定时时间TOUT=(124+1)*(167+1)/168=125us*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);                  //使能TIM1
      TIM_DeInit(TIM1);                                             //复位定时器1 
      TIM_TimeBaseStructure.TIM_Period=124;                         //设置自动重装载寄存器周期的值
      TIM_TimeBaseStructure.TIM_Prescaler=167*3/10;                       //设置时钟频率除数的预分频值
      TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;         //设置时钟分割TIM_CKD_DIV1=0x0000，
      TIM_TimeBaseStructure.TIM_RepetitionCounter=0x00;             //设置RCR寄存器值
      TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;     //TIIM向上计数
      TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);                //初始化TIM3
      TIM_Cmd(TIM1,ENABLE);                                         //开启TIM1计数
      TIM_ClearFlag(TIM1, TIM_FLAG_Update);                         //清溢出标志 
 }

void TIM2_init(void)
{
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);       //使能TIM2 
      TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);               //TIM2 Configuration
      TIM_TimeBaseStructure.TIM_Period = 124;                       // 定时时间TOUT=(124+1)*(167+1)/168=125us  
      TIM_TimeBaseStructure.TIM_Prescaler = 167;       
      TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
      TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
      TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

      /* TIM2 TRGO selection */
      TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);      //更新事件被选为触发输入
  
      /* TIM2 enable counter */
      TIM_Cmd(TIM2, ENABLE); 
}


//----------------------------------------------------------------------------
//自带I2C配置
//----------------------------------------------------------------------------
void I2C_Configuration(void)
{
  I2C_InitTypeDef I2C_InitStructure;
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);      //使能GPIOB时钟
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);        //使能I2C1    
      /* Configure I2C1 pins: SCL and SDA */
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//开漏输出
      GPIO_InitStructure. GPIO_PuPd=GPIO_PuPd_UP;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      
      GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_I2C1);
      GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_I2C1);
      
      
      //RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1 ,ENABLE);
     // RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1,DISABLE);
      
      /* I2C configuration */
      I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;//设置 I2C为 I2C模式
      I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;//I2C快速模式 Tlow / Thigh = 2
        
      I2C_InitStructure.I2C_OwnAddress1 = I2C1_SLAVE_ADDRESS7 ;//设置第一个设备地址

      I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;//使能应答
      I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;//应答 7位地址
      I2C_InitStructure.I2C_ClockSpeed =1000;//设置时钟频率

      /* I2C Peripheral Enable */
          /* Apply I2C configuration after enabling it */
      I2C_Init(I2C1, &I2C_InitStructure);
      I2C_Cmd(I2C1, ENABLE);//使能I2C外设
  
}
//----------------------------------------------------------------------------
//往ADS1100中写入控制寄存器
//----------------------------------------------------------------------------
void Write_AD() 
{
    //  while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY)); 
     // asm_delay(1000);
      I2C_GenerateSTART(I2C1, ENABLE);                                          //产生START条件
      while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));               //等待ACK
     // asm_delay(1000);
      I2C_Send7bitAddress(I2C1, 0x90, I2C_Direction_Transmitter);                 //向指定的从 I2C设备传送地址字,选择发送方向
      while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//等待ACK 
      //      asm_delay(1000);
      I2C_SendData(I2C1,0x8f);                                                 //发送ADS1100控制命令，15SPS，PGA=8
      while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));        //发送完成
      
      I2C_GenerateSTOP(I2C1, ENABLE);                                           //产生 I2Cx传输 STOP条件
}

//----------------------------------------------------------------------------
//从ADS中读取数据
//----------------------------------------------------------------------------
uint Read_AD()
{
      u8 adcdath,adcdatl,adcdats;
      uint ADCdata;
    
      
      while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));                             //等待I2C
      I2C_GenerateSTART(I2C1, ENABLE);                                          //产生START条件
      while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));               //等待ACK
      
      I2C_Send7bitAddress(I2C1, 0x91, I2C_Direction_Receiver);                 //向指定的从 I2C设备传送地址字,选择发送方向
      while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));    //等待ACK 
   
      I2C_AcknowledgeConfig(I2C1, ENABLE);                                     //使能或者失能指定 I2C的应答功能
      while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)));
      adcdath= I2C_ReceiveData(I2C1);                                           //返回通过 I2Cx最近接收的数据
     // I2C_GenerateSTOP(I2C1, ENABLE);                                           //产生 I2Cx传输 STOP条件
      
                                               
      while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)));
      adcdatl= I2C_ReceiveData(I2C1);                                           //返回通过 I2Cx最近接收的数据
      
      while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)));
      adcdats= I2C_ReceiveData(I2C1);                                           //返回通过 I2Cx最近接收的数据
      
      I2C_AcknowledgeConfig(I2C1, DISABLE);                                      //使能或者失能指定 I2C的应答功能
      I2C_GenerateSTOP(I2C1, ENABLE);
      ADCdata=adcdath*256+adcdatl;                                      //使能或者失能指定 I2C的应答功能

      
      return ADCdata;
}



void ADC_init()
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE); //ADC1复位
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;//A4 is adc1 4 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;  
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA,&GPIO_InitStructure); 
  


  RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);
  
  
  ADC_CommonInitStructure.ADC_Mode=ADC_Mode_Independent;//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_5Cycles;
                                                //两个采样阶段之间延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled ;
                                                //DMA禁止
  ADC_CommonInitStructure.ADC_Prescaler=ADC_Prescaler_Div4;//预分频4分频
                                  //ADCCLK=PCLK2/4=84/4=21Mhz,不可超过36Mhz
  ADC_CommonInit(&ADC_CommonInitStructure); //通用初始化
  
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;  //12位分辨率
  ADC_InitStructure.ADC_ScanConvMode =DISABLE;  //非扫描模式
  ADC_InitStructure.ADC_ContinuousConvMode =DISABLE ;  //非连续模式
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigInjecConvEdge_None ;
                                                  //禁止触发检测，使用软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐
  ADC_InitStructure.ADC_NbrOfConversion =1 ;//1个转换在规则序列中
  ADC_Init(ADC1, &ADC_InitStructure);  //ADC初始化
  ADC_Cmd(ADC1,ENABLE);  //开启AD转换

}


void DAC_init(void)
{
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);        //使能DAC时钟
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;     //PA.4 DAC1输出;PA.5DAC2输出
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
      GPIO_Init(GPIOA,&GPIO_InitStructure);
      
      

      /* DAC channel2 Configuration */
     
      
      DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;                       //不使用触发功能TEN1=0
      DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;         //不使用波形发生
      DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
      DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;          //DAC1输出缓存关闭
      DAC_Init(DAC_Channel_2, &DAC_InitStructure);		                //初始化DAC通道2


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
      i = 12000;  //自己定义
    while (i--);
    }
}

void delay_us(volatile u16 time)
{
    volatile u16 i = 0;
    while (time--)
    {
      i = 12;  //自己定义
    while (i--);
    }
}



void SysClk_Init(u32 clock)
{
  SysTick_Config(clock);
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}






