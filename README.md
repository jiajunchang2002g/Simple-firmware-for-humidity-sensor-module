Simple firmware for the DHT-11 Temperature and Relative Humidity Sensor Module. 
Drivers : STM32F4xx_HAL_Driver, CMSIS
Clock Config : Enable TIM2 (APB1) and scale it to 25MHZ, SysClock timebase is 1ms by default. 
GPIO config : GPIOB, GPIO_PIN_9

Recognizes timeout errors. 
