# AsyncDataStreamer


#### Description
* Implements real-time data acquisition from a potentiometer and a DHT11 temperature sensor, and uses FreeRTOS to configure task priority and concurrency on an STM32 microcontroller (STM32 Nucleo).
* Uses ADC channels and GPIO interrupts to ensure timely and accurate sensor readings.
* Uses UART serial communication to write and display data, and SPI communication to interface with the SD card module, for storing sensor data.


#### Resources

* [Write to SD in RTOS](https://controllerstech.com/how-to-store-sensors-data-into-sd-card/)
