#include "stm32f0xx.h"        // Device header
#include "SSD1306.h"
#include "SE8R01.h"
//#include "main.h"

#define RX_Mode	
//#define TX_Mode	

//#define I2C_100				1	//		100 kHz i2C Freq
#define I2C_400				1		//		400 kHz i2C Freq
	#define I2C_GPIOB			1	//		i2C PB6,PB7
//#define I2C_GPIOF			1	//		i2C PF0,PF1

uint8_t i,k,test=0;
uint8_t IRQ_Flag,sec_tic;
uint16_t TimingDelay,led_count,ms1000;
uint8_t tx_buf[TX_PLOAD_WIDTH]={'T','e','s','t',' ',' '};
struct se8r01_str se8r01;


void TimingDelayDec(void) 																													{
 if (TimingDelay			!=0x00) TimingDelay--;
 if (!led_count) {led_count=500; GPIOB->ODR ^=1;}
 if (!ms1000) {ms1000=1000;sec_tic=1;}
 led_count--;ms1000--;
 }

void TIM17_IRQHandler(void)																													{
		if (TIM17->SR & TIM_SR_UIF) {
					TimingDelayDec();
  				TIM17->SR &=(~TIM_SR_UIF);
		}
}	
void delay_ms (uint16_t DelTime) 																										{
    TimingDelay=DelTime;
  while(TimingDelay!= 0x00);
}
void EXTI0_1_IRQHandler(void)																												{
  if(EXTI->PR & EXTI_PR_PIF0)	{
			EXTI->PR |= EXTI_PR_PIF0;
			IRQ_Flag=1;
	}
}

void initial (void)																																{
//---------------TIM17------------------
  RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;    																			//HSI 8 MHz - 1 msek
  TIM17->PSC = 8000-1;
  TIM17->ARR = 1;
  TIM17->CR1 |= TIM_CR1_ARPE | TIM_CR1_DIR | TIM_CR1_CEN; 											// 
	TIM17->DIER |=TIM_DIER_UIE;
	NVIC_EnableIRQ (TIM17_IRQn);
	NVIC_SetPriority(TIM17_IRQn,0x05);	

//-------------------GPIOB-Blinking Led		
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN; 																					//
	GPIOB->MODER |= GPIO_MODER_MODER0_0;																					//Pb0-Out 
//-------------------GPIOA-CONFIG RX MODE----------
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN; 																					//
	GPIOA->MODER &=(~GPIO_MODER_MODER2);																					//switch	
	GPIOA->PUPDR |=GPIO_PUPDR_PUPDR2_0;																						//Pull-Up
//------------I2C1---------------------	
#ifdef I2C_GPIOB
//------------I2C1 GPIOB_SETTING ---------------------	
	RCC->AHBENR 		|=RCC_AHBENR_GPIOBEN;
	GPIOB->MODER 		|=GPIO_MODER_MODER6_1 		| GPIO_MODER_MODER7_1; 							// Alt -mode /Pb6 -SCL , Pb7- SDA
	GPIOB->OSPEEDR 	|=GPIO_OSPEEDER_OSPEEDR6 	| GPIO_OSPEEDER_OSPEEDR7;
	GPIOB->OTYPER		|=GPIO_OTYPER_OT_6 				| GPIO_OTYPER_OT_7;
	GPIOB->AFR[0] 	|=(1<<GPIO_AFRL_AFRL6_Pos) |(1<<GPIO_AFRL_AFRL7_Pos);  				// I2C - Alternative PB7, PB6
#endif	
#ifdef I2C_GPIOF	
	//------------I2C1 GPIOF_SETTING---------------------	
	RCC->AHBENR 		|=RCC_AHBENR_GPIOFEN;
	GPIOF->MODER 		|=GPIO_MODER_MODER0_1 		| GPIO_MODER_MODER1_1; 							// Alt -mode /Pf0 - SDA, Pf1- SCL
	GPIOF->OSPEEDR 	|=GPIO_OSPEEDER_OSPEEDR0 	| GPIO_OSPEEDER_OSPEEDR1;
	GPIOF->OTYPER		|=GPIO_OTYPER_OT_0 				| GPIO_OTYPER_OT_1;
	GPIOF->AFR[0] 	|=(1<<GPIO_AFRL_AFRL0_Pos) |(1<<GPIO_AFRL_AFRL1_Pos);  				// I2C - Alternative
#endif	

	GPIOA->MODER |= GPIO_MODER_MODER1_0;																					//LCD_Res_pin
RCC->APB1ENR |=RCC_APB1ENR_I2C1EN;
#ifdef I2C_100	
	I2C1->TIMINGR |=(0x1	<<I2C_TIMINGR_PRESC_Pos); 	//100 kHz - I2C bus speed
	I2C1->TIMINGR |=(0x13	<<I2C_TIMINGR_SCLL_Pos);
	I2C1->TIMINGR |=(0xF	<<I2C_TIMINGR_SCLH_Pos);
	I2C1->TIMINGR |=(0x2	<<I2C_TIMINGR_SDADEL_Pos);
	I2C1->TIMINGR |=(0x4	<<I2C_TIMINGR_SCLDEL_Pos);
#endif
#ifdef I2C_400	
	I2C1->TIMINGR |=(0x0	<<I2C_TIMINGR_PRESC_Pos); 	//400 kHz - I2C bus speed
	I2C1->TIMINGR |=(0x9	<<I2C_TIMINGR_SCLL_Pos);
	I2C1->TIMINGR |=(0x3	<<I2C_TIMINGR_SCLH_Pos);
	I2C1->TIMINGR |=(0x1	<<I2C_TIMINGR_SDADEL_Pos);
	I2C1->TIMINGR |=(0x3	<<I2C_TIMINGR_SCLDEL_Pos);
#endif	
I2C1->CR2 &=(~I2C_CR2_HEAD10R) & (~I2C_CR2_ADD10);
I2C1->CR1 |=I2C_CR1_PE;
	
	
//------------------------SPI-----------------------------------
	RCC->AHBENR 		|=RCC_AHBENR_GPIOAEN;
	GPIOA->OSPEEDR 	|= GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR4 
										|GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
	GPIOA->MODER 		|=GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; 	//Pa2, Pa4 - out,Pa5..7 - Alt_mode 
	GPIOA->AFR[0] 	|=(0<<GPIO_AFRL_AFRL7_Pos) |(0<<GPIO_AFRL_AFRL6_Pos) | (0<<GPIO_AFRL_AFRL5_Pos);  // SPI - Alternative
	GPIOA->MODER 		|=GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0; //Pa3, Pa4 - out, Pa0 Input inerrupt
	
	RCC->APB2ENR |=RCC_APB2ENR_SPI1EN;
	SPI1->CR1 |=SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | (0<<SPI_CR1_BR_Pos);  // if HSI8 - SpiSpeed (BR=2) - 1MHz
	SPI1->CR2 |=SPI_CR2_FRXTH;
	SPI1->CR1 |=SPI_CR1_SPE;
	CS_HI();
	
//----------------------EXTI-----------------------------------
	RCC->APB2ENR |=RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI0_PA;
	EXTI->FTSR |= EXTI_FTSR_TR0; //Falling
	
  NVIC_SetPriority(EXTI0_1_IRQn, 2); 
  NVIC_EnableIRQ(EXTI0_1_IRQn); 
	//EXTI->IMR |= EXTI_IMR_MR0;				//Cannot activate immediately, because need switch GD0 to Rx\Tx buf Interrupt mode
	__enable_irq ();	
} 

int main(void)
{
initial();
delay_ms (100);	
LCD_Init();LCD_Clear();
LCD_Gotoxy (10,0);LCD_PrintStr(" TEST SE8R01 ",1);
LCD_Gotoxy (1,1); LCD_PrintStr("Status=",0);LCD_PrintHex(init_io(),0);	

CEq_LO();
se8r01_powerup();
se8r01_calibration();
se8r01_setup();
radio_settings();
	
#ifdef RX_Mode 
 SPI_RW_Reg(WRITE_SE_REG|iRF_BANK0_CONFIG, 0x3f); //RX 
 RXX(&se8r01);	
#endif
#ifdef TX_Mode 
	SPI_RW_Reg(WRITE_SE_REG|iRF_BANK0_CONFIG, 0x3E); //TX
	for (uint8_t t=0;t<TX_PLOAD_WIDTH;t++){ se8r01.txbuf[t]=tx_buf[t];}
#endif

CEq_HI();
EXTI->IMR |= EXTI_IMR_MR0;

 
//-----------------------------initial data----------------------------------

while (1)  /* Main loop */
{
	
	if (sec_tic) {	sec_tic=0;
	LCD_Gotoxy (1,7); LCD_PrintStr("LCD = ",0);LCD_PrintDec(test++,0);LCD_PrintStr("   ",0);

#ifdef TX_Mode 
	se8r01.txbuf[5]=k++;
	TXX(&se8r01);
	LCD_Gotoxy (1,4); LCD_PrintStr("STAT = ",0);LCD_PrintBin(se8r01.status,0);LCD_PrintStr("   ",0);	
#endif	
}



if (IRQ_Flag && (SPI_Read(STATUS) & STA_MARK_RX))	{	//RX_Mode

  RXX(&se8r01);
	IRQ_Flag=0;
	LCD_Gotoxy (1,3); LCD_PrintStr("Rx = ",0);LCD_PrintDec(se8r01.rpd,0);LCD_PrintStr(" dBm.   ",0);
	LCD_Gotoxy (1,4); LCD_PrintStr("STAT = ",0);LCD_PrintBin(se8r01.status,0);LCD_PrintStr("   ",0);
	LCD_Gotoxy (1,5); LCD_PrintStr(se8r01.rxbuf,0);LCD_Gotoxy (31,5);LCD_PrintDec(se8r01.rxbuf[5],0);LCD_PrintStr("   ",0);  
 }


} // end - main loop 
} // end - Main  
	
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ while (1)  {  } }
#endif
