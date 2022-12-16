#include "main.h"

//>>>>>>>>>>>>>>>>>>>>>>>> функции для работы с USART <<<<<<<<<<<<<<<<<<<<<<<<//

// инициализация usart
void UsartInit() {
  stdout = &mystdout;

  UBRRL = 51;                        // 8 000 000 / 9600 / 16 - 1 = 51
  UCSRB = (1 << TXEN) | (1 << RXEN); // разрешаем прием-передау
  UCSRC = (1 << URSEL) | (3 << UCSZ0); // 8 бит
  UCSRB |= (1 << RXCIE); // Разрешаем прерывание при передаче
}

// отправка символа по usart
static int UsartPutChar(char c, FILE *stream) {
  if (c == '\n')
    UsartPutChar('\r', stream);
  while (!(UCSRA & (1 << UDRE))) {
  }
  UDR = c;
  return 0;
}

// отправка байта по USART
void SendByteUsart(unsigned char c) {
  while (!(UCSRA & (1 << UDRE))) {
  }
  UDR = c;
}

// прием данных по усарт
void ReceivingUsart() {
  memset(data, 0, sizeof data);

  int i = 0;
  do {
    while (!(UCSRA & (1 << RXC))) {
    }
    data[i] = UDR;
    i++;
  } while (data[i - 1] != '\r');
}

// настройка времени работы устройств по usart
void SetTime() {
  printf("\n>>>> Enter timetable (time) <<<<\n");

  for (int conveyor = 0; conveyor < 4; conveyor++) {
    for (int device = 0; device < 3; device++) {
      printf("Enter conveyor #%d, device #%d: ", conveyor + 1, device + 1);
      ReceivingUsart();
      timetable_device[conveyor][device] = ((data[0] & 0b00001111) * 100) +
                                           ((data[1] & 0b00001111) * 10) +
                                           (data[2] & 0b00001111);
    }
    SendByteUsart(13);
  }
}

// настройка длительности паузы по usart
void SetDelay() {
  printf("\n>>>> Enter timetable (delay) <<<<\n");
  for (int conveyor = 0; conveyor < 4; conveyor++) {
    printf("Enter delay for conveyor #%d: ", conveyor + 1);
    ReceivingUsart();
    timetable_delay[conveyor] =
        ((data[0] & 0b00001111) * 10) + (data[1] & 0b00001111);
  }
}

// настройка скорости по usart
void SetSpeed() {
  printf("\n>>>> Enter max speed <<<<\n");

  for (int conveyor = 0; conveyor < 4; conveyor++) {
    printf("Enter speed for conveyor #%d: ", conveyor + 1);
    ReceivingUsart();
    max_speed[conveyor] = (data[0] & 0b00001111) +
                          ((data[2] & 0b00001111) * 0.1) +
                          ((data[3] & 0b00001111) * 0.01);
    current_speed[conveyor] = max_speed[conveyor] * 0.8;
  }
}

// получение всех настроек по usart
void GetTimetable() {
  printf("\n>>>> TIMETABLE & SPEED <<<<\n");
  for (int conveyor = 0; conveyor < 4; conveyor++) {
    for (int device = 0; device < 3; device++) {
      printf("Time conveyor #%d, device #%d: %d\n", conveyor + 1, device + 1,
             timetable_device[conveyor][device]);
    }
    printf("Delay for conveyor #%d: %d\n", conveyor + 1,
           timetable_delay[conveyor]);
    printf("Max speed for conveyor #%d: %.2f\n", conveyor + 1,
           max_speed[conveyor]);
  }
}

// прерывание usart
ISR(USART_RXC_vect) {
  ReceivingUsart();

  if (strcmp(data, command_time) == 0)
    SetTime();
  else if (strcmp(data, command_delay) == 0)
    SetDelay();
  else if (strcmp(data, command_speed) == 0)
    SetSpeed();
  else if (strcmp(data, command_get) == 0)
    GetTimetable();
}

//>>>>>>>>>>>>>>>>>>>>>>>>> функции для работы с ADC <<<<<<<<<<<<<<<<<<<<<<<<<//

// инициализация ADC
void AdcInit(void) {
  ADCSRA |= (1 << ADEN) // Разрешение использования АЦП
            | (1 << ADPS2) | (1 << ADPS1) |
            (1 << ADPS0); // Делитель 128 = 64 кГц
  ADMUX |=
      (0 << REFS1) | (0 << REFS0); // Внутренний Источник ОН 2,56в, вход ADC0
}

// преобразование ADC
unsigned int AdcConvert(void) {
  ADCSRA |= (1 << ADSC); // Начинаем преобразование
  while ((ADCSRA & (1 << ADSC)))
    ; // проверим закончилось ли аналого-цифровое преобразование
  return (unsigned int)ADC;
}

// подсчет силы тока, угловой скорости, скорости
void FindAdc(unsigned char adc_input) {
  ADMUX = adc_input;
  adc[adc_input] = (float)AdcConvert() / 205;
  angular_speed[adc_input] = (adc[adc_input] - 1.23) / 0.0185;
  current_speed[adc_input] = 2 * 3.14 * radius * angular_speed[adc_input] / 60;
  printf("ADC%d: I=%.2fV, ang.speed=%.2f RPM, speed=%.2f m/s\n", adc_input + 1,
         adc[adc_input], angular_speed[adc_input], current_speed[adc_input]);
}

//>>>>>>>>>>>>>>>>>>>>> функции для работы с кнопками <<<<<<<<<<<<<<<<<<<<<<<//

// опрос кнопок
void AskButtons() {
  for (int con = 0; con < 4; con++) {
    if (!(B_PIN & (0x01 << (con + 4)))) {
      while (!(B_PIN & (0x01 << (con + 4)))) {
      }
      if (con_state[con] == 0) {
        flag_low_speed[con] = 0;
        con_state[con] = 1;
        LED_PORT |= (1 << (con + 4));
        printf("Conveyor #1 is running...\n");
      } else {
        con_state[con] = 0;
        LED_PORT &= ~(1 << (con + 4));
        printf("Conveyor #%d is stopped.\n", con + 1);
      }
    }
  }
}

//>>>>>>>>>>>>>>>>>>>>> функции для работы со временем <<<<<<<<<<<<<<<<<<<<<<<//

// прерывание таймера
ISR(TIMER0_OVF_vect) {
  TCNT0 = 248; // 8 тиков до переполнения
  time_ms++;   // системное время увеличили на 1
}

// инициализируем таймер, чтобы переполнялся 1000 раз в секунду
void T0Init() {
  TCCR0 = 0;
  TCCR0 = (0 << WGM01) | (0 << WGM00);
  TCNT0 = 248;
  TIFR |= (1 << TOV0);
  TIMSK |= (1 << TOIE0);
  TCCR0 |= (1 << CS02) | (0 << CS01) | (1 << CS00); // предделитель 1024
}

// инициализация системного времени
void TimerInit() {
  time_ms = 0; // обнуляем переменную времени
  T0Init();    // запускаем аппаратный таймер
}

// функция установки таймера на системном времени
uint32_t TimerSet(const uint32_t AddTimeMs) {
  return time_ms + AddTimeMs; // возвращает системное время + время задержки
}

// проверка установленного таймера, если время не прошло, то возращаем фолз
bool TimerIsExpired(const uint32_t Timer) {
  if ((time_ms - Timer) < (1UL << 31))
    return (Timer <= time_ms);
  return false;
}

//>>>>>>>>>>>>>>>> функции для работы с тормозом конвейера <<<<<<<<<<<<<<<<<<<//

void TimerSpeed(uint8_t conveyor) {
  if (speed_state[conveyor] == 1)
    timer_low_speed[conveyor]++;

  if (timer_low_speed[conveyor] > 9) {
    timer_low_speed[conveyor] = 0;
    flag_low_speed[conveyor] = 1;
  }
}

// прерывание таймера
ISR(TIMER1_COMPA_vect) {
  TimerSpeed(0);
  TimerSpeed(1);
  TimerSpeed(2);
  TimerSpeed(3);
}

// инициализируем таймер, чтобы переполнялся 1 раз в секунду
void T1Init() {
  TCCR1B |= (1 << WGM12); // установка режима СТС (сброс по совпадению)
  TIMSK |= (1 << OCIE1A); // установка бита разрешения прерывания 1-ого счетчика
                          // по совпадению с OCR1A

  OCR1AH = 0b01111010; // запись в регистр числа (31250) для сравнения
  OCR1AL = 0b00010010;

  TCCR1B |= (1 << CS12); // установка делителя на 256

  // частота прерывания таймера 0:
  // 8_000_000 / 256 = 31250
  // 31250 / 31250 = 1 Гц
}

void CheckBreak(void) {
  if (current_speed[0] / max_speed[0] >= 0.95)
    BREAK_1_ON;
  if (current_speed[0] / max_speed[0] <= 0.9)
    BREAK_1_OFF;

  if (current_speed[1] / max_speed[1] >= 0.95)
    BREAK_2_ON;
  if (current_speed[1] / max_speed[1] <= 0.9)
    BREAK_2_OFF;

  if (current_speed[2] / max_speed[2] >= 0.95)
    BREAK_3_ON;
  if (current_speed[2] / max_speed[2] <= 0.9)
    BREAK_3_OFF;

  if (current_speed[3] / max_speed[3] >= 0.95)
    BREAK_4_ON;
  if (current_speed[3] / max_speed[3] <= 0.9)
    BREAK_4_OFF;

  if (current_speed[0] / max_speed[0] <= 0.75) {
    speed_state[0] = 1;
  } else {
    speed_state[0] = 0;
    timer_low_speed[0] = 0;
  }

  if (current_speed[1] / max_speed[1] <= 0.75) {
    speed_state[1] = 1;
  } else {
    speed_state[1] = 0;
    timer_low_speed[1] = 0;
  }

  if (current_speed[2] / max_speed[2] <= 0.75) {
    speed_state[2] = 1;
  } else {
    speed_state[2] = 0;
    timer_low_speed[2] = 0;
  }

  if (current_speed[3] / max_speed[3] <= 0.75) {
    speed_state[3] = 1;
  } else {
    speed_state[3] = 0;
    timer_low_speed[3] = 0;
  }
}

void LedAlarm(uint8_t led) {
  switch (led_state[led]) {
  case 0: {
    LED_PORT |= (1 << (led + 4));
    led_delay[led] = TimerSet(300); // Ставим задержку
    led_state[led] = 1; // переходим в следующее состояние автомата
    break;
  }

  case 1: {
    if (!TimerIsExpired(led_delay[led]))
      break; // Если задержка не прошла - сразу выходим
    LED_PORT &= ~(1 << (led + 4));
    led_delay[led] = TimerSet(300);
    led_state[led] = 2;
    break;
  }

  case 2: {
    if (!TimerIsExpired(led_delay[led]))
      break;
    led_state[led] = 0;
    break;
  }

  default:
    break;
  }
}

void turn_on(uint8_t con, uint8_t device) {
  switch (con) {
  case 0: {
    if (device == 0)
      DEV_1_1_ON;
    else if (device == 1)
      DEV_1_2_ON;
    else if (device == 2)
      DEV_1_3_ON;
    break;
  }
  case 1: {
    if (device == 0)
      DEV_2_1_ON;
    else if (device == 1)
      DEV_2_2_ON;
    else if (device == 2)
      DEV_2_3_ON;
    break;
  }
  case 2: {
    if (device == 0)
      DEV_3_1_ON;
    else if (device == 1)
      DEV_3_2_ON;
    else if (device == 2)
      DEV_3_3_ON;
    break;
  }
  case 3: {
    if (device == 0)
      DEV_4_1_ON;
    else if (device == 1)
      DEV_4_2_ON;
    else if (device == 2)
      DEV_4_3_ON;
    break;
  }
  }
}

void turn_off(uint8_t con, uint8_t device) {
  switch (con) {
  case 0: {
    if (device == 0)
      DEV_1_1_OFF;
    else if (device == 1)
      DEV_1_2_OFF;
    else if (device == 2)
      DEV_1_3_OFF;
    break;
  }
  case 1: {
    if (device == 0)
      DEV_2_1_OFF;
    else if (device == 1)
      DEV_2_2_OFF;
    else if (device == 2)
      DEV_2_3_OFF;
    break;
  }
  case 2: {
    if (device == 0)
      DEV_3_1_OFF;
    else if (device == 1)
      DEV_3_2_OFF;
    else if (device == 2)
      DEV_3_3_OFF;
    break;
  }
  case 3: {
    if (device == 0)
      DEV_4_1_OFF;
    else if (device == 1)
      DEV_4_2_OFF;
    else if (device == 2)
      DEV_4_3_OFF;
    break;
  }
  }
}

void Alarm() {
  for (int led = 0; led < 4; led++) {
    if (flag_low_speed[led] == 1) {
      con_state[led] = 0;
      current_speed[led] = max_speed[led] * 0.8;
      turn_off(led, 0);
      turn_off(led, 1);
      turn_off(led, 2);
      LedAlarm(led);
    }
  }
}

//>>>>>>>>>>>>>>>>>>>> функции для работы с конвейером <<<<<<<<<<<<<<<<<<<<<<//

void SwitchConveyor(uint8_t con, char state) {
  if (state == 1) {
    switch (conveyor_state[con]) {
    case 0: {
      Delay[con] =
          TimerSet(timetable_device[con][0] * modifier); // Ставим задержку
      turn_on(con, 0);
      conveyor_state[con] = 1; // переходим в следующее состояние автомата
      break;
    }

    case 1: {
      if (!TimerIsExpired(Delay[con]))
        break; // Если задержка не прошла - сразу выходим
      turn_off(con, 0);
      Delay[con] = TimerSet(timetable_delay[con] * modifier);
      conveyor_state[con] = 2;
      break;
    }

    case 2: {
      if (!TimerIsExpired(Delay[con]))
        break;
      turn_on(con, 1);
      Delay[con] = TimerSet(timetable_device[con][1] * modifier);
      conveyor_state[con] = 3;
      break;
    }
    case 3: {
      if (!TimerIsExpired(Delay[con]))
        break;
      turn_off(con, 1);
      Delay[con] = TimerSet(timetable_delay[con] * modifier);
      conveyor_state[con] = 4;
      break;
    }
    case 4: {
      if (!TimerIsExpired(Delay[con]))
        break;
      turn_on(con, 2);
      Delay[con] = TimerSet(timetable_device[con][2] * modifier);
      conveyor_state[con] = 5;
      break;
    }
    case 5: {
      if (!TimerIsExpired(Delay[con]))
        break;
      turn_off(con, 2);
      Delay[con] = TimerSet(timetable_delay[con] * modifier);
      conveyor_state[con] = 6;
      break;
    }
    case 6: {
      if (!TimerIsExpired(Delay[con]))
        break;
      conveyor_state[con] = 0;
      break;
    }

    default:
      break;
    }
  } else {
    turn_off(con, 0);
    turn_off(con, 1);
    turn_off(con, 2);
    conveyor_state[con] = 0;
  }
}

// проверка скорости конвейеров
void CheckAdc(void) {
  static uint8_t state = 0; // Переменная состояния конечного автомата
  static uint32_t Delay; // Переменная программного таймера

  switch (state) {
  case 0: {
    Delay = TimerSet(1000); // Ставим задержку
    state = 1; // переходим в следующее состояние автомата
    break;
  }

  case 1: {
    if (!TimerIsExpired(Delay))
      break; // Если задержка не прошла - сразу выходим

    if (con_state[0] == 1)
      FindAdc(0);
    if (con_state[1] == 1)
      FindAdc(1);
    if (con_state[2] == 1)
      FindAdc(2);
    if (con_state[3] == 1)
      FindAdc(3);

    state = 0;
    break;
  }

  default:
    break;
  }
}

//>>>>>>>>>>>>>>> функции для инициализации всех устройств <<<<<<<<<<<<<<<<<//

void Init(void) {
  B_DDR = 0x00;
  B_PORT = 0xff;

  DDRB = 0xff;
  PORTB = 0x00;

  DDRC = 0xff;
  PORTC = 0x00;

  DDRD = 0xff;
  PORTD = 0x00;

  UsartInit(); // инициализация усарт
  AdcInit();   // инициализация АЦП
  TimerInit(); // инициализация системного времени
  T1Init();

  sei(); // разрешение прерываний
}

//>>>>>>>>>>>>>>>>>>>>>>>>> MAIN <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

int main(void) {
  Init();

  while (1) {
    AskButtons();
    SwitchConveyor(0, con_state[0]);
    SwitchConveyor(1, con_state[1]);
    SwitchConveyor(2, con_state[2]);
    SwitchConveyor(3, con_state[3]);
    CheckAdc();
    CheckBreak();
    Alarm();
  }
}
