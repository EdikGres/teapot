#include "GyverRelay.h"

//---------------------Thermistor---------------------
//pin for Thermistor
#define THERMISTORPIN A0
// сопротивление при 25 градусах по Цельсию
#define THERMISTORNOMINAL 85000
// temp. для номинального сопротивления (практически всегда равна 25 C)
#define TEMPERATURENOMINAL 24
// сколько показаний используем для определения среднего значения
#define NUMSAMPLES 5
// бета коэффициент термистора (обычно 3000-4000)
#define BCOEFFICIENT 3750
// сопротивление второго резистора
#define SERIESRESISTOR 10000
//---------------------Thermistor---------------------
//---------------------Relay---------------------
#define RELAY 7
//---------------------Relay---------------------
GyverRelay regulator(REVERSE);

int samples[NUMSAMPLES];
float set_temp = 70;
float hyster = 3;

//---------------------RUCHKA---------------------
#define RUCHKA A0
//---------------------RUCHKA---------------------


void setup(void) {
Serial.begin(9600);
analogReference(EXTERNAL);
pinMode(RELAY, OUTPUT);

regulator.k = 4;
regulator.setpoint = set_temp;
regulator.hysteresis = hyster;
}

void loop(void) {
float temp = getTemp();
Serial.print("Temperature ");
Serial.print(temp);
Serial.println(" *C");
regulator.input = temp;
static uint32_t tmr;
  if (millis() - tmr > 500) {
    tmr = millis();
    regulator.input = getTemp(); // сообщаем регулятору текущую температуру
    digitalWrite(RELAY, regulator.getResult());   // отправляем на реле (ОС работает по своему таймеру)
  }
//digitalWrite(RELAY, regulator.getResult());
analogRead(
  
delay(250);

}

float getTemp() {
  uint8_t i;
  float average;
  // сводим показания в вектор с небольшой задержкой между снятием показаний
  for (i=0; i< NUMSAMPLES; i++) {
    samples[i] = analogRead(THERMISTORPIN);
    delay(10);
  }
  // рассчитываем среднее значение
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
    average += samples[i];
  }
  average /= NUMSAMPLES;
  
  // конвертируем значение в сопротивление
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
  
  float steinhart;
  steinhart = average / THERMISTORNOMINAL; // (R/Ro)
  steinhart = log(steinhart); // ln(R/Ro)
  steinhart /= BCOEFFICIENT; // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart; // инвертируем
  steinhart -= 273.15; // конвертируем в градусы по Цельсию
  return steinhart;
}
