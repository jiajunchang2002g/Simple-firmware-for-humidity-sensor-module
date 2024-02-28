Simple firmware for the DHT-11 Temperature and Relative Humidity Sensor Module on the STM32 Blackpill (F411ceu6)

Inspired by https://github.com/dhrubasaha08/DHT11.git 

Drivers : STM32F4xx_HAL_Driver, CMSIS
Clock Config : Enable TIM2 (APB1) and scale it to 25MHZ, SysClock timebase is 1ms by default. 
GPIO config : GPIOB, GPIO_PIN_9

- Recognizes timeout and checksum errors, returns temperature and humidity values

- Use debug mode live-expression to obtain readings since serial print is not available without extensive UART code

Datasheet for DHT-11 : 

https://www.mouser.com/datasheet/2/758/DHT11-Technical-Data-Sheet-Translated-Version-1143054.pdf
