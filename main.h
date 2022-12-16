#ifndef MAIN_H_
#define MAIN_H_

//>>>>>>>>>>> НАСТРОЙКИ МК <<<<<<<<<<<//

#define F_CPU 8000000UL // частота работы МК
#define BAUDRATE 9600L // скорость передачи данных по usart

//>>>>>>>>>>> БИБЛИОТЕКИ <<<<<<<<<<<//

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//>>>>>>>>>>> DEFINE <<<<<<<<<<<//

#define B_DDR DDRA
#define B_PORT PORTA
#define B_PIN PINA
#define BTN1 4
#define BTN2 5
#define BTN3 6
#define BTN4 7

///////////////////////////////////////////

#define DEV_1_1_ON PORTB |= (1 << PINB0)
#define DEV_1_2_ON PORTB |= (1 << PINB1)
#define DEV_1_3_ON PORTB |= (1 << PINB2)
#define BREAK_1_ON PORTB |= (1 << PINB6)

#define DEV_1_1_OFF PORTB &= ~(1 << PINB0)
#define DEV_1_2_OFF PORTB &= ~(1 << PINB1)
#define DEV_1_3_OFF PORTB &= ~(1 << PINB2)
#define BREAK_1_OFF PORTB &= ~(1 << PINB6)

///////////////////////////////////////////

#define DEV_2_1_ON PORTB |= (1 << PINB3)
#define DEV_2_2_ON PORTB |= (1 << PINB4)
#define DEV_2_3_ON PORTB |= (1 << PINB5)
#define BREAK_2_ON PORTB |= (1 << PINB7)

#define DEV_2_1_OFF PORTB &= ~(1 << PINB3)
#define DEV_2_2_OFF PORTB &= ~(1 << PINB4)
#define DEV_2_3_OFF PORTB &= ~(1 << PINB5)
#define BREAK_2_OFF PORTB &= ~(1 << PINB7)

///////////////////////////////////////////

#define DEV_3_1_ON PORTC |= (1 << PINC0)
#define DEV_3_2_ON PORTC |= (1 << PINC1)
#define DEV_3_3_ON PORTC |= (1 << PINC2)
#define BREAK_3_ON PORTC |= (1 << PINC6)

#define DEV_3_1_OFF PORTC &= ~(1 << PINC0)
#define DEV_3_2_OFF PORTC &= ~(1 << PINC1)
#define DEV_3_3_OFF PORTC &= ~(1 << PINC2)
#define BREAK_3_OFF PORTC &= ~(1 << PINC6)

///////////////////////////////////////////

#define DEV_4_1_ON PORTC |= (1 << PINC3)
#define DEV_4_2_ON PORTC |= (1 << PINC4)
#define DEV_4_3_ON PORTC |= (1 << PINC5)
#define BREAK_4_ON PORTC |= (1 << PINC7)

#define DEV_4_1_OFF PORTC &= ~(1 << PINC3)
#define DEV_4_2_OFF PORTC &= ~(1 << PINC4)
#define DEV_4_3_OFF PORTC &= ~(1 << PINC5)
#define BREAK_4_OFF PORTC &= ~(1 << PINC7)

///////////////////////////////////////////

#define LED_PORT PORTD

#define LED_1_ON PORTD |= (1 << PINC4)
#define LED_2_ON PORTD |= (1 << PINC5)
#define LED_3_ON PORTD |= (1 << PINC6)
#define LED_4_ON PORTD |= (1 << PINC7)

///////////////////////////////////////////

#define LED_1_OFF PORTD &= ~(1 << PINC4)
#define LED_2_OFF PORTD &= ~(1 << PINC5)
#define LED_3_OFF PORTD &= ~(1 << PINC6)
#define LED_4_OFF PORTD &= ~(1 << PINC7)

///////////////////////////////////////////

//>>>>>>>>>>> ПЕРЕМЕННЫЕ <<<<<<<<<<<//

char data[64]; // массив символов для usart
const char command_time[] = "time\r";
const char command_delay[] = "delay\r";
const char command_speed[] = "speed\r";
const char command_get[] = "get\r";
static int UsartPutChar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(UsartPutChar, NULL, _FDEV_SETUP_WRITE);
// const char command[] = "1\r";

float radius = 0.1;                            // радиус в метрах
float current_speed[4] = {0.8, 0.8, 0.8, 0.8}; // скорость, м/с
float angular_speed[4]; // угловая скорость, об/мин
float adc[4]; // массив для хранения показаний с датчиков скорости

uint16_t timetable_device[4][3] = {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}};
uint8_t timetable_delay[4] = {1, 1, 1, 1};
float max_speed[4] = {1, 1, 1, 1};

static volatile uint32_t time_ms = 0; // переменная для отчета системного
                                      // времени

uint16_t modifier =
    1000; // коэф умножения времени работы конвейеров, переменная для отладки

uint8_t con_state[4] = {0}; // флаги состояний линий конвейеров

uint16_t timer_low_speed[4] = {0};
uint8_t flag_low_speed[4] = {0};
uint8_t speed_state[4] = {0};

uint8_t conveyor_state[4] = {0}; // Переменная состояния конечного автомата
uint32_t Delay[4]; // Переменная программного таймера

uint8_t led_state[4] = {0}; // Переменная состояния конечного автомата
uint32_t led_delay[4]; // Переменная программного таймера

#endif /* MAIN_H_ */
