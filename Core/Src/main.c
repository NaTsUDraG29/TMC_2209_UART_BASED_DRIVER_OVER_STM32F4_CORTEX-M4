/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * Name : Keshava D Hegde
  * Date : Sept 7, 2025
  * Topic : Stepper motor config and driver
**/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "stepper_motion.h"
#include "stepper_config.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void GPIO_init(void);
void UART_init(void);
void DWT_init(void);
void GPIO_set(GPIO_TypeDef *GPIO, uint16_t pin);
void GPIO_reset(GPIO_TypeDef *GPIO, uint16_t pin);
void USART2_init(void);
uint8_t hardware_uart2_read();
void software_UART_TX(uint8_t data);
uint8_t software_UART_RX(uint32_t data);
void UART_software_config_outputmode(void);
void UART_software_config_inputmode(void);
void generic_gpio_init(GPIO_TypeDef *GPIO,uint8_t set_mode,uint8_t pin);
void timer_2_init();
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{


  HAL_Init();


  SystemClock_Config();


  while (1)
  {
	  //test code for uart loopback

	  uint8_t a = 0xF,b= 0xF,c= 0xF;
	  software_UART_TX(0x1);
	  a = hardware_uart2_read();
	  software_UART_TX(0x2);
	  b = hardware_uart2_read();
	  software_UART_TX(0x3);
	  c = hardware_uart2_read();
//	  software_UART_TX(0x4);
//	  software_UART_TX(0x5);
  }

  return 0;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

//Test the blink for hardware faults
void GPIO_init(void)
{

//	GPIO_TypeDef GPIOA;

	//enable the GPIO PORT clock in RCC
	RCC->AHB1ENR |= GPIOA_EN; // setting up GPIO A

	//Set the GPIO pin to output mode

	//clear the Moder bits first
	GPIOA->MODER &= ~(3 << MODER(8));
	GPIOA->MODER |= (0x1 << MODER(8));

	//set the output type of the pin (setting it to push pull
	GPIOA->OTYPER &= ~(0x1 << GPIO8);

	//setting the pin to fast speed
	GPIOA->OSPEEDR &= ~(3 << MODER(8));
	GPIOA->OSPEEDR |= (3 << MODER(8));

	//setting the pin to pull down initially
	GPIOA->PUPDR &= ~(3 << MODER(8));
	GPIOA->PUPDR |= (2 << MODER(8));


}

//UART config
void UART_software_config_outputmode(void)
{
	//Enable the clock for the GPIO port used for uart
	RCC->AHB1ENR |= GPIOC_EN;
	//Enable the clock for UART
	RCC->APB1ENR |= USART2_EN;
	//set the pin to output mode for software UART
	GPIOC->MODER &= ~(3 << MODER(4));
	GPIOC->MODER |= (1 << MODER(4));

	GPIOC->OTYPER &= ~(1 << UART_GPIO);

	GPIOC->OSPEEDR &= ~(3 << MODER(4));
	GPIOC->OSPEEDR |= (3 << MODER(4));

	GPIOC->PUPDR &= ~(3 << MODER(4));
	GPIOC->PUPDR |= (1 << MODER(4));

	GPIOC->BSRR = (1 << UART_GPIO);//pulled to high state

}

void UART_software_config_inputmode(void)
{

	//set the pin to output mode for software UART
	GPIOC->MODER &= ~(3 << MODER(4)); //clear the bits to 0
//	GPIOC->MODER &= (1 << MODER(4));

	GPIOC->OTYPER &= ~(1 << UART_GPIO);

//	GPIOC->OSPEEDR &= ~(3 << MODER(4));
//	GPIOC->OSPEEDR |= (3 << MODER(4));

	GPIOC->PUPDR &= ~(3 << MODER(4));
	GPIOC->PUPDR |= (1 << MODER(4));

	GPIOC->BSRR = (1 << UART_GPIO);

}

//Data watch point for count cycles and delay
void DWT_init(void)
{
	//enable the trace/ debug support
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	//reset and enable the cycle count
	DWT->CYCCNT = 0;

	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

//software uart init
void UART_init(void)
{
	//Set up the pins used for TX and RX
	UART_software_config_inputmode();

	DWT_init();
}


void GPIO_reset(GPIO_TypeDef *GPIO, uint16_t pin)
{
	GPIO->ODR = (1 << (pin + 16));
}

void GPIO_set(GPIO_TypeDef *GPIO, uint16_t pin)
{
	 GPIO->BSRR = (1 << pin);
}
// Delay in CPU cycles
static inline void delay_cycles(uint32_t cycles)
{
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles)
    {
        __asm__("NOP");//asking the MCU to burn a cycle doing nothing
    }
}

// Delay in microseconds
static inline void delay_us(uint32_t us)
{
    uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000L) * us;
    delay_cycles(cycles);
}

// Delay in milliseconds
static inline void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        delay_us(1000);
    }
}


void software_UART_TX(uint8_t data)
{
	//initially the uart will be high , pull it low to start the data transmission
	GPIO_reset(GPIOC,UART_GPIO);
	GPIO_set(GPIOA,GPIO8);
	delay_us(DELAY_PER_BIT);

	for(int i = 0; i < 8; i++)
	{
		if(data & (1 << i))
			GPIO_set(GPIOC,UART_GPIO);
		else
			GPIO_reset(GPIOC,UART_GPIO);
		delay_us(DELAY_PER_BIT);
	}

	GPIO_set(GPIOC,UART_GPIO);
	GPIO_reset(GPIOA,GPIO8);
	delay_us(DELAY_PER_BIT);
}

uint8_t software_UART_RX(uint32_t timeout_us)
{
	//now the GPIO is set to input mode
	    UART_Byte_t result;
	    result.status = UART_OK;
	    result.data = 0;

	    uint32_t waited = 0;

	    // 1. Wait for start bit (LOW)
	    while ((GPIOC->IDR & (1 << 4)))
	    {
	        delay_us(1);
	        if (++waited >= timeout_us)
	        {
	            result.status = UART_TIMEOUT;
	            return 0;
	        }
	    }

	    // 2. Wait half bit time
	    delay_us(DELAY_PER_BIT/2);

	    // 3. Read 8 bits LSB first
	    for (int i = 0; i < 8; i++)
	    {
	        delay_us(DELAY_PER_BIT);
	        if (GPIOC->IDR & (1 << 4))
	        {
	            result.data |= (1 << i);
	        }
	    }

	    // 4. Stop bit check
	    delay_us(DELAY_PER_BIT);
	    if (!(GPIOC->IDR & (1 << 4)))
	    {
	        result.status = UART_FRAME_ERROR;
	    }

	    return result.data;
}



void USART2_init(void)
{
	RCC->APB1ENR |= USART2_EN;
	RCC->AHB1ENR |= GPIOA_EN;

	GPIOA->MODER &= ~(3 << MODER(3));
	GPIOA->MODER |= 2 << MODER(3);

	//AF7 for PA3

	GPIOA->AFR[0] &= ~AF_UART_RESET;
	GPIOA->AFR[0] |= AF_UART_SET; //setting AF7 for usart2 rx

	USART2->BRR |= HAL_RCC_GetPCLK1Freq()/19200;
	USART2->CR1 = USART_CR1_RE | USART_CR1_UE;
}

uint8_t hardware_uart2_read(void)
{
	uint16_t runtime = 1000;
    while (!(USART2->SR & USART_SR_RXNE) && --runtime);  // wait until data ready
    return ((uint8_t)USART2->DR);
}

void generic_gpio_init(GPIO_TypeDef *GPIO,uint8_t set_mode,uint8_t pin)
{
	//this function assumes that the clock for the gpio is already set
	if(set_mode == 1)//output mode
	{
			GPIO->MODER &= ~(3 << MODER(pin)); //clear the bits to 0
			GPIOC->MODER &= (1 << MODER(pin));

			GPIO->OTYPER &= ~(1 << pin);//push pull

			GPIOC->OSPEEDR &= ~(3 << MODER(pin));
			GPIOC->OSPEEDR |= (3 << MODER(pin));

			GPIO->PUPDR &= ~(3 << MODER(pin));
			GPIO->PUPDR |= (2 << MODER(pin));

			GPIO->BSRR = (1 << (pin+16));//set to low
	}
	else//input mode
	{
					GPIO->MODER &= ~(3 << MODER(pin)); //clear the bits to 0
//					GPIOC->MODER &= (1 << MODER(4));

					GPIO->OTYPER &= ~(1 << pin);//push pull

					GPIOC->OSPEEDR &= ~(3 << MODER(pin));
					GPIOC->OSPEEDR |= (3 << MODER(pin));

					GPIO->PUPDR &= ~(3 << MODER(pin));
					GPIO->PUPDR |= (2 << MODER(pin));

//					GPIO->BSRR = (1 << pin+16);
	}
}

void timer_2_init()
{
	//Enable tim2 clock
	RCC->APB1ENR |= TIM2_EN;

	//Enable the F clock
	RCC->AHB1ENR |= GPIOF_EN;

	//Set the GPIO F pin 13 to step and 12 to dir
	//lets create a generic init fucntion which sets the pins to input or output mode
	generic_gpio_init(GPIOF, GPIO_OUPUT_M, GPIO12);//dir
	generic_gpio_init(GPIOF, GPIO_OUPUT_M, GPIO13);//step



	//now configure the tim2
	TIM2->DIER |= (1 << 0);//update intrp enanble
	TIM2->CR1 |=(1 << 0);//counter enable

	NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler()
{

	    if (TIM2->SR & TIM_SR_UIF) // Check update flag
	    {
	        TIM2->SR &= ~TIM_SR_UIF; // Clear flag
	    }
	        // Toggle STEP pin
	    if(stepper.remaining_steps > 0)
	    {
	    	GPIOF->ODR ^= (1 << GPIO13);
	    	stepper.remaining_steps--;
	    }

}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
