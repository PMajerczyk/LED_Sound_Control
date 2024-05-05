# LED_Sound_Control
It is a sound LED control on bluepill STM32F103C8T6. It doesn't use HAL library but works on registers. To speed up the work I used CMSIS (Cortex Microcontroller Software Interface Standard) <br> 

## Components:
- STM32F103C8T6 <br>
- ST-LINK V2 <br>
- LED RGB (in my case, a common anode) <br>
- Sound sensor ky-037 <br>
- Resistors 150, 100, 100 Ohm <br>

## Connections:

**Sound sensor:** <br>
- AO `→` A0 <br>
- G `→` GND <br>
- \+ `→` 5V <br>
- DO `→` A5 <br>

**LED RGB:** <br>
- R `→` 150Ω `→` A6 <br>
- \+ `→` 5V <br>
- G `→` 100Ω `→` A7 <br>
- B `→` 100Ω `→` B0 <br>

## Main [code](https://github.com/PMajerczyk/LED_Sound_Control/blob/main/blink/Src/main.c):
Includes all necessary configurations and LED control algorithm

