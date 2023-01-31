/*Based registers included*/
#include "stm32l1xx.h"


/*Base LL driver included*/
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_bus.h"

/*Base LL driver for LCD included*/
#include "stm32l1xx_ll_lcd.h"
#include "stm32l152_glass_lcd.h"

/*Base LL driver for TIM included*/
#include "stm32l1xx_ll_tim.h"
#include "dwt_delay.h"

/*for sprinf function access*/
#include <stdio.h> 

/*for boolean type*/
#include <stdbool.h>

/*clock and TIM*/
void SystemClock_Config(void);
void TIM2_Config(void);
uint32_t start_time, current_time; 			/* Timer variables */ 


/* DHT11 sensor pin */
#define DHT11_PIN    LL_GPIO_PIN_2
#define DHT11_PORT   GPIOB
uint8_t DHT11_check_pulse(void);
void DHT11_readbyte(void);

int state = 0;
uint8_t humidity = 0, temperature = 0;  /* Variables to store sensor data */

/*Button*/
void EXTI0_Init(void);   /*EXTI*/
bool click = 0;

/*LCD*/
char disp_str[7] = "0";

//GPIO
void GPIO_Init(void);

int main(){
	
    SystemClock_Config();		/*max-performance config*/
    LCD_GLASS_Init();				/*Initialize LCD*/
    GPIO_Init();						/*Configure GPIO*/
    EXTI0_Init();						/*Configure EXTI*/
//    TIM2_Config();          /*Configure TIM2*/
		DWT_Init();							/*Configure DWT*/
	
    while(1){
			/* Read data from DHT11 sensor */
			int x = DHT11_check_pulse();
//		DHT11_readbyte();
			
			LL_mDelay(100);

        /*check T is over limit?*/
			if(temperature > 20){
				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11);
				LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);
				DAC->DHR12R2 = 0x0FFF;
			}
			else{ 
				LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11);
				LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
				DAC->DHR12R2 = 0;
			}
        
      /*show data to LCD*/
			if(click){
				sprintf(disp_str,"H %d", humidity);
			}
			else{ 
				sprintf(disp_str,"T %d", temperature);
			}
			LCD_GLASS_DisplayString((uint8_t*)disp_str);			//diplay on LCD
			}
}

void SystemClock_Config(void)
{
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
	
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
 }
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz                             */
  /* This frequency can be calculated through LL RCC macro                          */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}

void GPIO_Init(void)
{
		LL_GPIO_InitTypeDef GPIO_InitStruct;    /*declare struct for GPIO config*/
	
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);	/* Enable GPIOA port */
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);	/* Enable GPIOB port */
		
		/*PB2 DHT11 (input)*/
		GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;					/*set port to output*/
		GPIO_InitStruct.Pin = DHT11_PIN;							/*set on pin 2*/
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;		/*set output to push-pull type*/
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;		        		/*set output to push-pull type*/
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;				/*set port to low output speed*/
		LL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);					/*config to GPIOB register*/
		
		/*PA0 User Button (input)*/
		GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;					/*set port to input*/
		GPIO_InitStruct.Pin = LL_GPIO_PIN_0;						/*set on pin 0*/
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;		/*set output to push-pull type*/
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;						/*set push-pull to no pull*/
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;			/*set port to fast output speed*/
		LL_GPIO_Init(GPIOA, &GPIO_InitStruct);						/*config to GPIOA register*/
		
		/*PA5 Buzzer (output)*/
		RCC->APB1ENR |= (1<<29);		/*open clock for DAC*/
		GPIOA->MODER |= (3<<10); 		/*alternate function*/
		DAC->CR |= (1<<16);				/*enable DAC channel 2*/
		
		/*PB11 LED (output)*/
		GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;					/*set port to input*/
		GPIO_InitStruct.Pin = LL_GPIO_PIN_11;						/*set on pin 11*/
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;		/*set output to push-pull type*/
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;						/*set push-pull to no pull*/
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;			/*set port to fast output speed*/
		LL_GPIO_Init(GPIOA, &GPIO_InitStruct);						/*config to GPIOA register*/
				
		/*PB6 LED on board (output)*/
		GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;					/*set port to input*/
		GPIO_InitStruct.Pin = LL_GPIO_PIN_6;						/*set on pin 6*/
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;		/*set output to push-pull type*/
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;						/*set push-pull to no pull*/
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;			/*set port to fast output speed*/
		LL_GPIO_Init(GPIOB, &GPIO_InitStruct);						/*config to GPIOB register*/
}

/* Function to initialize EXTI0 */
void EXTI0_Init(void)
{
		RCC->APB2ENR |= (1<<0);		        /*open clock for SYSCFG*/
	
		SYSCFG->EXTICR[0] &= ~(15<<0);		/* EXTI Line 0 (EXTI0) for PA0*/
		EXTI->RTSR |= (1<<0);				/*Rising trigger slection register for PA0*/
		EXTI->IMR |= (1<<0);				/*Interrupt mark for PA0*/
	
		/*NVIC conf*/
		NVIC_EnableIRQ((IRQn_Type)6);		/*use AF 6 (EXTI0)*/	
		NVIC_SetPriority((IRQn_Type)6,1);	/*set priority for PA0 (1st)*/
}

/* Funtion to EXTI interrupt (pressed user button) */
void EXTI0_IRQHandler(void) 
{
		if((EXTI->PR & (1<<0)) == 1){
			LL_LCD_Clear();				/*clear display*/
			click = !click;				/*Toggle s from display data on LCD*/
			EXTI->PR |= (1<<0);		    /*reset EXTI0 for next interrupt*/
		}
}

/* Function to initialize TIM2 */
void TIM2_Config(void)
{
		/*declare struct for GPIO config*/
		LL_TIM_InitTypeDef timbase_initstructure;
	
		/* Enable TIM2 clock */
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
		
		timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;									/*DTS=tCK_INT*/
		timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;        								/*Counter used as upcounter*/
		timbase_initstructure.Prescaler = 32;				/* Set the pre-scaler value to have TIM2 counter clock equal to 1 MHz */
		timbase_initstructure.Autoreload = 1; 																				/* Set the auto-reload value to have a counter frequency of 1 microsecond */
		LL_TIM_Init(TIM2, &timbase_initstructure);
	
		/*config NVIC*/
		NVIC_SetPriority(TIM2_IRQn, 0);		/* Enable the TIM2 IRQ in the NVIC */
		NVIC_EnableIRQ(TIM2_IRQn);				/* Set priority for TIM2 interrupt */
	
	  /* Enable TIM2 interrupt */
    LL_TIM_EnableIT_UPDATE(TIM2);
	
		/* Enable TIM2 */
		LL_TIM_EnableCounter(TIM2);
		
}

void TIM2_IRQHandler(void)
{
    /* Check whether update interrupt is pending */
    if(LL_TIM_IsActiveFlag_UPDATE(TIM2) == SET)
    {
				start_time--;
        /* Clear the update interrupt flag*/
        LL_TIM_ClearFlag_UPDATE(TIM2);

    }
}

/* Function to read data from DHT11 sensor using TIM2 */
uint8_t DHT11_check_pulse(void)
{
/*START SECTION*/
    /* Set DHT11 pin as output */
    LL_GPIO_SetPinMode(DHT11_PORT, DHT11_PIN, LL_GPIO_MODE_OUTPUT);
	
		/* Send start signal to DHT11 sensor */
    LL_GPIO_ResetOutputPin(DHT11_PORT, DHT11_PIN);		
    DWT_Delay(18000);
	
//    LL_GPIO_SetOutputPin(DHT11_PORT, DHT11_PIN);
//		DWT_Delay(30);
	
    /* Set DHT11 pin as input */
    LL_GPIO_SetPinMode(DHT11_PORT, DHT11_PIN, LL_GPIO_MODE_INPUT);
		DWT_Delay(60);
	
		if(LL_GPIO_IsInputPinSet(DHT11_PORT, DHT11_PIN) == 0){
			DWT_Delay(80);
			if(LL_GPIO_IsInputPinSet(DHT11_PORT, DHT11_PIN) == 0)
				return 1;
		}
		
		while(LL_GPIO_IsInputPinSet(DHT11_PORT, DHT11_PIN) == 1);

	uint16_t data[5];
	
	for(int j = 0; j < 5; j++){
		for(int i = 7; i <= 0; i--){
			while(LL_GPIO_IsInputPinSet(DHT11_PORT, DHT11_PIN)==0);
			DWT_Delay(40);
			if(LL_GPIO_IsInputPinSet(DHT11_PORT, DHT11_PIN)==0)
			{
				data[j] |= ( 0 << i);		//Read low first
				continue;
			}
			else
			{
				data[j] |= ( 1 << i);
				while(LL_GPIO_IsInputPinSet(DHT11_PORT, DHT11_PIN)==1);
			}
		}
	}	
	temperature = data[0];
	humidity = data[2];
					return 2;
}

void DHT11_readbyte(void){
/*READ SECTION*/
	uint16_t data[5];
	
	for(int j = 0; j < 5; j++){
		for(int i = 7; i <= 0; i--){
			while(LL_GPIO_IsInputPinSet(DHT11_PORT, DHT11_PIN)==0);
			DWT_Delay(40);
			if(LL_GPIO_IsInputPinSet(DHT11_PORT, DHT11_PIN)==0)
			{
				data[j] |= ( 0 << i);		//Read low first
				continue;
			}
			else
			{
				data[j] |= ( 1 << i);
				while(LL_GPIO_IsInputPinSet(DHT11_PORT, DHT11_PIN)==1);
			}
		}
	}	
	temperature = data[0];
	humidity = data[2];
}
