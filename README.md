# STM32F407G-DISC1

A working directory for learning how to programme an ARM stm32f4 processor via the STM32F407G-DISC1 dev board from STMicroelectronics and the http://embedded.fm/ blog series [Embedded Wednesdays: Getting Started In Embedded Systems](http://embedded.fm/blog/embedded-wednesdays).

Note that when you're setting up the HAL UART function, `&huart2` refers to the number UART you chose and setup in CubeMX.

`HAL_UART_Receive(&huart2, (uint8_t *) result, len, HAL_MAX_DELAY);`

i.e. HAL UART 2 (huart2), HAL UART 3 (huart3), etc.

![USART 2 Setup](/images/STM32F4_UART2_setup.png)
![20x4 LED Panel over i2c](/images/stm32f4_LED_Display.jpg)
