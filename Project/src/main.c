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

void SystemClock_Config(void);

/* DHT11 sensor pin */
#define DHT11_PIN    LL_GPIO_PIN_2
#define DHT11_PORT   GPIOB

int DHT11_read(void);
int state = 0;
uint8_t humidity = 0, temperature = 0;  /* Variables to store sensor data */

/* Motor */
#define MOTOR_ENABLE_PIN LL_GPIO_PIN_4
#define MOTOR_ENABLE_PORT GPIOA

#define MOTOR_H_PIN LL_GPIO_PIN_7
#define MOTOR_H_PORT GPIOB

#define MOTOR_L_PIN LL_GPIO_PIN_11
#define MOTOR_L_PORT GPIOA

void TIM_OC_Config(void);
void TIM_OC_GPIO_Config(void);
void TIM4_Base_Config(void);
void _1293d_config();

/* Button */
#define BUTTON_PIN LL_GPIO_PIN_0
#define BUTTON_PORT GPIOA

void EXTI0_Init(void);
bool click = 0;

/* LED */
#define LED_PIN LL_GPIO_PIN_6
#define LED_PORT GPIOB

/* LCD */
char disp_str[7] = "0";

/* GPIO setup */
void GPIO_Init(void);

int duty = 0;

int main(){	
	SystemClock_Config();		/* max-performance config */
	LCD_GLASS_Init();		/* Initialize LCD */
	GPIO_Init();			/* Configure GPIO */
	EXTI0_Init();			/* Configure EXTI */
	DWT_Init();			/* Configure DWT */
	TIM_OC_Config();		/* Configure TIM for motor */
	_1293d_config();		/* Configure IC for motor */
	
	/* Motor turn Right */
	LL_GPIO_SetOutputPin(MOTOR_H_PORT, MOTOR_H_PIN);			/* set PB7 HIGH */
	LL_GPIO_ResetOutputPin(MOTOR_L_PORT, MOTOR_L_PIN);			/* set PA11 LOW */
	
	while(1){
	/* Read data from DHT11 sensor */
	state = DHT11_read();
	LL_mDelay(100);

	/* check temparature is over limit? */
	if(temperature > 28){
		duty = 35 + ((int)temperature - 28) * 5;
		LL_TIM_OC_SetCompareCH2(TIM4, duty);
		LL_GPIO_SetOutputPin(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);		/* Enable Motor */
		LL_GPIO_SetOutputPin(LED_PORT, LED_PIN);				/* Turn ON LED */
	}
	else{ 
		LL_GPIO_ResetOutputPin(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);		/* Disable Motor */
		LL_GPIO_ResetOutputPin(LED_PORT, LED_PIN);				/* Turn OFF LED */
	}
	    
	  /* show data on LCD */
	if(click){
		sprintf(disp_str,"H %d", humidity);		/* humidity */
	}
	else{ 
		sprintf(disp_str,"T %d", temperature);		/* temperature */
	}					
	LCD_GLASS_DisplayString((uint8_t*)disp_str);		/* diplay on LCD */
	}
}

/* Function to initializ GPIO*/
void GPIO_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct;   			/* declare struct for GPIO config */	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);	/* Enable GPIOA port */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);	/* Enable GPIOB port */
	
	/*PB2 DHT11 (input)*/
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;			/* set port to outpu t*/
	GPIO_InitStruct.Pin = DHT11_PIN;				/* set on pin 2 */
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;		/* set output to push-pull type */
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;		        	/* set output to push-pull type */
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;			/* set port to low output speed */
	LL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);			/* config to GPIOB register */
	
	/*PA0 User Button (input)*/
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;			/* set port to input */
	GPIO_InitStruct.Pin = BUTTON_PIN;				/* set on pin 0 */
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;		/* set output to push-pull type */
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;				/* set push-pull to no pull */
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;		/* set port to fast output speed */
	LL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);			/* config to GPIOA register */	
			
	/*PB6 LED on board (output)*/
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;			/* set port to output */
	GPIO_InitStruct.Pin = LED_PIN;					/* set on pin 6 */
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;		/* set output to push-pull type */
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;				/* set push-pull to no pull */
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;		/* set port to fast output speed */
	LL_GPIO_Init(LED_PORT, &GPIO_InitStruct);			/* config to GPIOB register */
}

/* Function to initialize EXTI0 */
void EXTI0_Init(void)
{
	RCC->APB2ENR |= (1<<0);		        /* open clock for SYSCFG */
																			 
	SYSCFG->EXTICR[0] &= ~(15<<0);		/* EXTI Line 0 (EXTI0) for PA0 */
	EXTI->RTSR |= (1<<0);			/* Rising trigger slection register for PA0 */
	EXTI->IMR |= (1<<0);			/* Interrupt mark for PA0 */	
	
	/*NVIC conf*/
	NVIC_EnableIRQ((IRQn_Type)6);		/* use AF 6 (EXTI0) */	
	NVIC_SetPriority((IRQn_Type)6, 0);	/* set priority for PA0 (1st) */
}

/* Funtion to EXTI interrupt (pressed user button) */
void EXTI0_IRQHandler(void) 
{
	if((EXTI->PR & (1<<0)) == 1){
		LL_LCD_Clear();			/* clear display */
		click = !click;			/* Toggle s from display data on LCD */
		EXTI->PR |= (1<<0);		/* reset EXTI0 for next interrupt */
	}
}
/* Function to read data from DHT11 sensor using TIM2 */
int DHT11_read(void)
{
	int i, j, data[5] = {0};
	
	/************** START SECTION **************/
	/* Set DHT11 pin as output and pulls down */
	LL_GPIO_SetPinMode(DHT11_PORT, DHT11_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_ResetOutputPin(DHT11_PORT, DHT11_PIN);  
	LL_mDelay(18);

	/* Set DHT11 pin as input and wait for start data transmission*/
	LL_GPIO_SetPinMode(DHT11_PORT, DHT11_PIN, LL_GPIO_MODE_INPUT);
	
	i = 3;
	do
	{
	   DWT_Delay(20);
	   if(i-- == 0) return 1; // Error : Time Out;
	}
	while(LL_GPIO_IsInputPinSet(DHT11_PORT, DHT11_PIN) == 1);    /* wait for Low */
	 
	while(LL_GPIO_IsInputPinSet(DHT11_PORT, DHT11_PIN) == 0);   /* wait for High */

	/************** DATA SECTION **************/
	for(j = 0; j < 5; j++)
	{
	   for(i = 7; i >= 0; i--)
	   {
	      while(LL_GPIO_IsInputPinSet(DHT11_PORT, DHT11_PIN) == 1);   /* wait for Low */
	      while(LL_GPIO_IsInputPinSet(DHT11_PORT, DHT11_PIN) == 0);   /* wait for High */
	      DWT_Delay(40);
	
	      if(LL_GPIO_IsInputPinSet(DHT11_PORT, DHT11_PIN) == 0)
	      {
	         data[j] |= (0 << i);	/* set 0 */
	      }
	      else
	      {
	         data[j] |= (1 << i);	/* set 1 */
	         while(LL_GPIO_IsInputPinSet(DHT11_PORT, DHT11_PIN) == 1);   /* wait for Low */
	      }
	   }
   }

   temperature = data[2];
   humidity = data[0];
   return 2;
}

/* Function configures the GPIO pins for the 1293d IC */
void _1293d_config()
{
	LL_GPIO_InitTypeDef d_initstruct;				/*declare struct for GPIO config*/															/*declare struct for IC config */
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);		/* Enable GPIOA port */
	
	d_initstruct.Mode = LL_GPIO_MODE_OUTPUT;			/* select output mode */
	d_initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;		/* select to push-pull as output type */
	d_initstruct.Pull = LL_GPIO_PULL_NO;				/* select I/O no pull */
	d_initstruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;		/* select I/O fast output speed */
	d_initstruct.Pin = MOTOR_ENABLE_PIN;				/* select pin 4 */
	LL_GPIO_Init(MOTOR_ENABLE_PORT, &d_initstruct);			/* Initialize GPIO register */
																											
	d_initstruct.Pin = MOTOR_L_PIN;					/* select pin 11 */
	LL_GPIO_Init(MOTOR_L_PORT, &d_initstruct);			/* Initialize GPIO register */
}

/* Function configures TIM4 */
void TIM4_Base_Config(void)
{
	LL_TIM_InitTypeDef timbase_initstructure;				/*declare struct for IC config */
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);			/* Enable TIM4 */
	
	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV2; 	/*16MHz*/
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;		/* Counter used as upcounter */
	timbase_initstructure.Autoreload = 100;					/* set auto reload */
	timbase_initstructure.Prescaler = 32000;				/* set prescaler */
	LL_TIM_Init(TIM4 , &timbase_initstructure);				/* Initialize TIMER */
	
	LL_TIM_EnableCounter(TIM4);						/* Enable TIMER4 counter*/
}


/* Function configures GPIO for TIM4 */
void TIM_OC_GPIO_Config(void)
{
	LL_GPIO_InitTypeDef gpio_initstructure;					/*declare struct for GPIO config*/
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);		
	
	gpio_initstructure.Mode = LL_GPIO_MODE_ALTERNATE;			/* select alternate mode */
	gpio_initstructure.Pin = MOTOR_H_PIN;					/* set on pin 7 */
	gpio_initstructure.Alternate = LL_GPIO_AF_2;				/* use AF2 */
	gpio_initstructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;		/* select to push-pull as output type */
	gpio_initstructure.Pull = LL_GPIO_PULL_NO;				/* select I/O no pull */
	gpio_initstructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;		/* set port to very fast output speed */
	LL_GPIO_Init(MOTOR_H_PORT, &gpio_initstructure);			/* config to GPIO register */	
	
}

/* Function configures OC TIM4 for motor*/
void TIM_OC_Config(void)
{
	LL_TIM_OC_InitTypeDef tim_oc_initstructure;
	
	TIM_OC_GPIO_Config();		/* config GPIO for TIMER */
	TIM4_Base_Config();		/* config TIMER */
	
	tim_oc_initstructure.OCState = LL_TIM_OCSTATE_DISABLE;				/* OCx is not active */
	tim_oc_initstructure.OCMode = LL_TIM_OCMODE_PWM1;				/* select PWE mode */
	tim_oc_initstructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;			/* OCxactive high */
	tim_oc_initstructure.CompareValue = 100;					/* set Compare value*/			
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH2, &tim_oc_initstructure);		/*config to TIMER4_CH2*/	
	
	/*Start Output Compare in PWM Mode*/
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);				/* Enable capture/compare channels */
}

/* Function to max-performance config */
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
