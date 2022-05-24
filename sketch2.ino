#include "GyverRelay.h"
#include <LiquidCrystal.h>
//---------------------Thermistor---------------------
//pin for Thermistor
#define THERMISTORPIN A0
// сопротивление при 25 градусах по Цельсию
#define THERMISTORNOMINAL 85000
// temp. для номинального сопротивления (практически всегда равна 25 C)
#define TEMPERATURENOMINAL 24
// сколько показаний используем для определения среднего значения
#define NUMSAMPLES 8
// бета коэффициент термистора (обычно 3000-4000)
#define BCOEFFICIENT 3750
// сопротивление второго резистора
#define SERIESRESISTOR 10000
//---------------------Thermistor---------------------
//---------------------Relay---------------------
#define RELAY 7
//---------------------Relay---------------------
GyverRelay regulator(REVERSE);


//---------------------RUCHKA---------------------
#define RUCHKA A5
//---------------------RUCHKA---------------------
//---------------------DISPLAY---------------------
constexpr uint8_t PIN_RS = 12;
constexpr uint8_t PIN_EN = 11;
constexpr uint8_t PIN_DB4 = 5;
constexpr uint8_t PIN_DB5 = 4;
constexpr uint8_t PIN_DB6 = 3;
constexpr uint8_t PIN_DB7 = 2;
LiquidCrystal lcd(PIN_RS, PIN_EN, PIN_DB4, PIN_DB5, PIN_DB6, PIN_DB7);
//---------------------DISPLAY---------------------

int samples[NUMSAMPLES];
float set_temp = 70;
float hyster = 2;
int ruchka_resist = 0;
int counter = 0;

void setup(void) {
Serial.begin(9600);
analogReference(EXTERNAL);
pinMode(RELAY, OUTPUT);

regulator.k = 3;
regulator.hysteresis = hyster;

lcd.begin(16, 2);
lcd.setCursor(0, 0);

}

void loop(void) {
  counter++;
  float temp = getTemp();
  Serial.print("Temperature ");
  Serial.print(temp);
  Serial.println(" *C");
  regulator.input = temp;
  static uint32_t tmr;
    if (millis() - tmr > 500) {
      tmr = millis();
      regulator.input = getTemp()+3; // сообщаем регулятору текущую температуру
      if(set_temp == 99) regulator.input = getTemp();
      digitalWrite(RELAY, regulator.getResult());   // отправляем на реле (ОС работает по своему таймеру)
    }
  //digitalWrite(RELAY, regulator.getResult());
  ruchka_resist = analogRead(RUCHKA);
  set_temp = map(ruchka_resist, 0,1024,0,100);
  regulator.setpoint = set_temp;
  Serial.print("Ruchka: ");
  Serial.println(set_temp);
  
  lcd.setCursor(0,0);
  lcd.print("Cur_Temp:");
  lcd.print(temp);
  lcd.setCursor(0,1);
  lcd.print("Set_Temp:");
  lcd.print(set_temp);
  delay(250);
  //lcd.clear();
  if(counter > 4){
    lcd.clear();
    lcd.begin(16, 2);
    lcd.setCursor(0, 0);
    counter = 0;
}
}

float getTemp() {
  uint8_t i;
  float average;
  // сводим показания в вектор с небольшой задержкой между снятием показаний
  for (i=0; i< NUMSAMPLES; i++) {
    samples[i] = analogRead(THERMISTORPIN);
    delay(5);
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
