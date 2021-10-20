#include "earth_moisture.h"

double get_moisture_percentage(){
	// Returns a double representing the percentage of humidity in the moisture
	// 0% would indicate the same humidity as in the air
	// 100% would indicate the same humidity as in water
	// Sensor output has to be connected to A0
	int result = 0;
	for(int i = 0; i < 20; i++){
		ADC1->SQR5=0;				//conversion sequence starts at ch0
		ADC1->CR2|=1;				//bit 0, ADC on/off (1=on, 0=off)
		ADC1->CR2|=0x40000000;		//start conversion
		while(!(ADC1->SR & 2)){}	//wait for conversion complete
		result+=ADC1->DR;			//read conversion result
	}
	result = result / 20;

	// Convert the result to a usable percentage of humidity
	double based_to_0 = (result - 1770>0 ? result - 1770:0);
	double percentage = ((1860 - based_to_0) / 1860.00) * 100.00;
	double converted_result = (percentage>0 ? percentage:0);

	ADC1->CR2&=~1;				//bit 0, ADC on/off (1=on, 0=off)
	return converted_result;
}

// Example of usage:
//double percentage;
//char buf[100];
//int i;
//while (1)
//{
//  percentage = get_moisture_percentage();
//  sprintf(buf, "%d.%d", (int)percentage, (int)((percentage - (double)(int)percentage) * 100));
//  i = 0;
//  while(buf[i] != '\0'){
//	  USART2_write(buf[i]);
//	  i++;
//  }
//  USART2_write('\n');
//  USART2_write('\r');
//  delay_Ms(100);
//}


// void MX_ADC_Init(void)
// {

//   ADC_InitTypeDef ADC_InitStruct = {0};

//   /* GPIO Ports Clock Enable */
//   __HAL_RCC_ADC1_CLK_ENABLE();

//   /*Configure GPIO pin Output Level */
//   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

//   /*Configure GPIO pin Output Level */
//   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

//   /*Configure GPIO pin : PtPin */
//   GPIO_InitStruct.Pin = B1_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

//   /*Configure GPIO pins : PA5 PA10 */
//   GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_10;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

// }