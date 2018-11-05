#include <EEPROM.h>
//#include <TimerOne.h> 
 #define SERIAL_SPEED 9600

//РАСПИНОВКА
#define START_STOP_BUTTON 7//КНОПКА СТАРТА/ОСТАНОВА

#define TEN 3//УПРАВЛЕНИЕ ТЕНОМ
#define NULL_DETECT 2//ДЕТЕКТОР 0, Прерывание INT0
#define KRAN1 13//УПРАВЛЕНИЕ КРАНОМ
#define KRAN2 12//УПРАВЛЕНИЕ КРАНОМ
#define KRAN3 11//УПРАВЛЕНИЕ КРАНОМ
#define SINGLE_KRAN1 10//УПРАВЛЕНИЕ НЕЗАВИСИМЫМ КРАНОМ
#define SINGLE_KRAN2 9//УПРАВЛЕНИЕ НЕЗАВИСИМЫМ КРАНОМ
#define MODE_LED1 4//ИНДИКАЦИЯ
#define MODE_LED2 5//ИНДИКАЦИЯ
#define MODE_LED3 6//ИНДИКАЦИЯ


//Настройка датчиков температуры
#define B 3950 // B-коэффициент
#define TERMO_R1 4700 // подтяжка последовательного резистора, 4k7
#define THERMISTOR_R 100000 // номинальное сопротивления термистора, 100 кОм
#define NOMINAL_T 25 // номинальная температура (при которой TR = 100 кОм)

//АДРЕСА В ПАМЯТИ ЕЕPROM
#define EE_MAX_HEAT 0//Максимальная мощность ТЕНа %
#define EE_WORK_Temperature_0 4//Рабочая температура браги
#define EE_MAX_Temperature_1 8//Максимальная температура пара(при достижении - выключить)
#define EE_KRAN1_MIN_T 12//Диапазон вкл-выкл крана
#define EE_KRAN1_MAX_T 16//Диапазон вкл-выкл крана
#define EE_KRAN2_MIN_T 20//Диапазон вкл-выкл крана
#define EE_KRAN2_MAX_T 24//Диапазон вкл-выкл крана
#define EE_KRAN3_MIN_T 28//Диапазон вкл-выкл крана
#define EE_KRAN3_MAX_T 32//Диапазон вкл-выкл крана
#define EE_ADD2PHASE 36//Настройка смещения фазы для фазного включения ТЭНА
#define EE_STABLE_TIME 40//Время стабилизации(минуты)(время работы на себя после достижения указанноой температуры браги)
#define EE_SINGLE_KRAN1_MIN_T 44//Диапазон вкл-выкл НЕЗАВИСИМОГО крана
#define EE_SINGLE_KRAN1_MAX_T 48//Диапазон вкл-выкл НЕЗАВИСИМОГО крана
#define EE_SINGLE_KRAN2_MIN_T 52//Диапазон вкл-выкл НЕЗАВИСИМОГО крана
#define EE_SINGLE_KRAN2_MAX_T 56//Диапазон вкл-выкл НЕЗАВИСИМОГО крана


#define SERIAL_BUFFER_SIZE 64 //Размер буфера приема
String RX_buffer;
//Переменные окружения(глобальные)
float Temperature_0;//Температура бака
float Temperature_1;//Температура пара в баке
float Temperature_2;//Контрольный датчик(управляет кранами)
float Temperature_3;//Дополнительный датчик
float Temperature_4;//Дополнительный датчик
float Temperature_5;//Независимый датчик(Диапазон температур работает всегда)
byte MODE=0;//Текущий режим работы
unsigned int CURRENT_HEAT=0;//Мощность нагрева в %
byte WAVE_COUNTER=0;//Счетчик полупериодов (100 = 1с)
byte SECONDS=0;//прошедших секунд
unsigned int STABLE_TIME_COUNTER=0;//Счетчик минут состояния равновесия
byte CHECK_HEAT_STATE=1;//НЕОБХОДИМОСТЬ ПРОВЕРИТЬ МОЩНОСТЬ НАГРЕВА
int show_values=1;//НЕОБХОДИМОСТЬ ВЫДАТЬ В ПОРТ ЗНАЧЕНИЯ АЦП И ПРОЧИХ
byte show_parameters=1;//НЕОБХОДИМОСТЬ ВЫДАТЬ В ПОРТ НАСТРОЙКИ
byte NULL_DETECTED_STATE=0;//ДЛЯ ФАЗОВОГО УПРАВЛЕНИЯ
/*
* РЕЖИМЫ
* 0 - Выключено все
* 1 - Разогрев куба
* 2 - Ожидание стабилизации
* 3 - Рабочий режим(Включается работа кранов)
*/
//Данные хранимые в EEPROM
unsigned int MAX_HEAT;//Максимальная мощность ТЕНа %
float WORK_Temperature_0;//Рабочая температура браги
float MAX_Temperature_1;//Максимальная температура пара(при достижении - выключить)
float KRAN1_MIN_T;//Диапазон вкл-выкл крана
float KRAN1_MAX_T;//Диапазон вкл-выкл крана
float KRAN2_MIN_T;//Диапазон вкл-выкл крана
float KRAN2_MAX_T;//Диапазон вкл-выкл крана
float KRAN3_MIN_T;//Диапазон вкл-выкл крана
float KRAN3_MAX_T;//Диапазон вкл-выкл крана
float SINGLE_KRAN1_MIN_T;//Диапазон вкл-выкл НЕЗАВИСИМОГО крана
float SINGLE_KRAN1_MAX_T;//Диапазон вкл-выкл НЕЗАВИСИМОГО крана
float SINGLE_KRAN2_MIN_T;//Диапазон вкл-выкл НЕЗАВИСИМОГО крана
float SINGLE_KRAN2_MAX_T;//Диапазон вкл-выкл НЕЗАВИСИМОГО крана
unsigned int ADD2PHASE;//Настройка смещения фазы для фазного включения ТЭНА
unsigned int STABLE_TIME;//Время стабилизации(минуты)(время работы на себя после достижения указанноой температуры браги)




void setup() {
  TCCR1A=0;//ШИМ ОТКЛЮЧЕН
   TIMSK1|=(1<<OCF1B)|(1<<OCF1A);//РАЗРЕШИМ ПРЕРЫВАНИЯ

  pinMode(START_STOP_BUTTON, INPUT_PULLUP);//Кнопка с подтяжкой
  
  pinMode(NULL_DETECT, INPUT_PULLUP);//ДЕТЕКТОР НОЛЯ - ВХОД с подтяжкой
  attachInterrupt(0, NULL_DETECTED, FALLING); //ПРЕРЫВАНИЕ НА ПЕРЕХОД ЧЕРЕЗ 0  
/*
LOW – выполняется по низкому уровню сигнала, когда на контакте нулевое значение. Прерывание может циклично повторяться — например, при нажатой кнопке.
CHANGE – по фронту, прерывание происходит при изменении сигнала с высокого на низкий или наоборот. Выполняется один раз при любой смене сигнала.
RISING – выполнение прерывания один раз при изменении сигнала от LOW к HIGH.
FALLING – выполнение прерывания один раз при изменении сигнала от HIGH к LOW.
 */
  pinMode(TEN, OUTPUT);
  digitalWrite(TEN, 0);
  pinMode(KRAN1, OUTPUT);  digitalWrite(KRAN1, 0);
  pinMode(KRAN2, OUTPUT);  digitalWrite(KRAN2, 0);
  pinMode(KRAN3, OUTPUT);  digitalWrite(KRAN3, 0);
  pinMode(SINGLE_KRAN1, OUTPUT);  digitalWrite(SINGLE_KRAN1, 0);
  pinMode(SINGLE_KRAN2, OUTPUT);  digitalWrite(SINGLE_KRAN2, 0);
  pinMode(MODE_LED1, OUTPUT);  digitalWrite(MODE_LED1, 0);
  pinMode(MODE_LED2, OUTPUT);  digitalWrite(MODE_LED2, 0);
  pinMode(MODE_LED3, OUTPUT);  digitalWrite(MODE_LED3, 0);  
  
MAX_HEAT = EEPROM.read(EE_MAX_HEAT);//Максимальная мощность ТЕНа %
WORK_Temperature_0 = EEPROM_read_float(EE_WORK_Temperature_0);//Рабочая температура браги
MAX_Temperature_1 = EEPROM_read_float(EE_MAX_Temperature_1);//Максимальная температура пара(при достижении - выключить)
KRAN1_MIN_T = EEPROM_read_float(EE_KRAN1_MIN_T);//Диапазон вкл-выкл крана
KRAN1_MAX_T = EEPROM_read_float(EE_KRAN1_MAX_T);//Диапазон вкл-выкл крана
KRAN2_MIN_T = EEPROM_read_float(EE_KRAN2_MIN_T);//Диапазон вкл-выкл крана
KRAN2_MAX_T = EEPROM_read_float(EE_KRAN2_MAX_T);//Диапазон вкл-выкл крана
KRAN3_MIN_T = EEPROM_read_float(EE_KRAN3_MIN_T);//Диапазон вкл-выкл крана
KRAN3_MAX_T = EEPROM_read_float(EE_KRAN3_MAX_T);//Диапазон вкл-выкл крана
SINGLE_KRAN1_MIN_T = EEPROM_read_float(EE_SINGLE_KRAN1_MIN_T);//Диапазон вкл-выкл НЕЗАВИСИМОГО крана
SINGLE_KRAN1_MAX_T = EEPROM_read_float(EE_SINGLE_KRAN1_MAX_T);//Диапазон вкл-выкл НЕЗАВИСИМОГО крана
SINGLE_KRAN2_MIN_T = EEPROM_read_float(EE_SINGLE_KRAN2_MIN_T);//Диапазон вкл-выкл НЕЗАВИСИМОГО крана
SINGLE_KRAN2_MAX_T = EEPROM_read_float(EE_SINGLE_KRAN2_MAX_T);//Диапазон вкл-выкл НЕЗАВИСИМОГО крана
ADD2PHASE = EEPROM_read_int(EE_ADD2PHASE);//Настройка смещения фазы для фазного включения ТЭНА
STABLE_TIME = EEPROM.read(EE_STABLE_TIME);//Время стабилизации(минуты)(время работы на себя после достижения указанноой температуры браги)
    Serial.begin(SERIAL_SPEED);

//  Timer1.initialize(ADD2PHASE);//Инициалицация таймера
// Timer1.disablePwm(9);
// Timer1.disablePwm(10);
//  Timer1.attachInterrupt(timer1Int); // Обработчик прерывания
}



void loop() {
 Temperature_0 = get_temperature_from_adc(A0);
 Temperature_1 = get_temperature_from_adc(A1);
 Temperature_2 = get_temperature_from_adc(A2);
 Temperature_3 = get_temperature_from_adc(A3);
 Temperature_4 = get_temperature_from_adc(A4);
 Temperature_5 = get_temperature_from_adc(A5);
read_uart_buffer();

if (MODE==0){//ЕСЛИ СИСТЕМА ВЫКЛЮЧЕНА
  CURRENT_HEAT=0;
  digitalWrite(KRAN1, 0);
  digitalWrite(KRAN2, 0);
  digitalWrite(KRAN3, 0);
  setLed(0);
 }
else { //ЕСЛИ СИСТЕМА ВКЛЮЧЕНА
  if(CHECK_HEAT_STATE==1){//НЕОБХОДИМОСТЬ ПРОВЕРИТЬ МОЩНОСТЬ НАГРЕВА
  CHECK_HEAT_STATE=0; CURRENT_HEAT=CURRENT_HEAT+WORK_Temperature_0-Temperature_0; if(CURRENT_HEAT>100)CURRENT_HEAT=100; if(CURRENT_HEAT<0)CURRENT_HEAT=0; if(CURRENT_HEAT>MAX_HEAT)CURRENT_HEAT=MAX_HEAT;}}

 if(MODE==1){
if((Temperature_0+1)>=WORK_Temperature_0)MODE=2;//Переключение на стабилизацию
digitalWrite(KRAN1, 0);
digitalWrite(KRAN2, 0);
digitalWrite(KRAN3, 0);
setLed(1);}

   if(MODE==2){//ЕСЛИ РЕЖИМ СТАБИЛИЗАЦИИ(РАБОТА НА СЕБЯ)
if(STABLE_TIME_COUNTER>=STABLE_TIME)MODE=3;//ПЕРЕКЛЮЧЕНИЕ РЕЖИМА НА ОТБОР
digitalWrite(KRAN1, 0);
digitalWrite(KRAN2, 0);
digitalWrite(KRAN3, 0);
setLed(2);}
else {STABLE_TIME_COUNTER=0; SECONDS=0;}//НЕ СЧИТАЕМ ВРЕМЯ СТАБИЛИЗАЦИИ ВНЕ РЕЖИМА

   if(MODE==3){//ЕСЛИ РЕЖИМ ОТБОРА
if(Temperature_1>MAX_Temperature_1)MODE=0;//ВЫКЛЮЧАЕМ ЕСЛИ ПАР РАЗОГРЕТ ЧЕРЕЗМЕРНО(СПИРТА НЕТ)
if(Temperature_2>KRAN1_MIN_T&&Temperature_2<KRAN1_MAX_T)digitalWrite(KRAN1, 1);else digitalWrite(KRAN1, 0);//КРУТИМ КРАНЫ
if(Temperature_2>KRAN2_MIN_T&&Temperature_2<KRAN2_MAX_T)digitalWrite(KRAN2, 1);else digitalWrite(KRAN2, 0);//КРУТИМ КРАНЫ
if(Temperature_2>KRAN3_MIN_T&&Temperature_2<KRAN3_MAX_T)digitalWrite(KRAN3, 1);else digitalWrite(KRAN3, 0);//КРУТИМ КРАНЫ
   setLed(3);}

if(Temperature_5>SINGLE_KRAN1_MIN_T&&Temperature_2<SINGLE_KRAN1_MAX_T)digitalWrite(SINGLE_KRAN1, 1);else digitalWrite(SINGLE_KRAN1, 0);//КРУТИМ НЕЗАВИСИМЫЕ КРАНЫ
if(Temperature_5>SINGLE_KRAN2_MIN_T&&Temperature_2<SINGLE_KRAN2_MAX_T)digitalWrite(SINGLE_KRAN2, 1);else digitalWrite(SINGLE_KRAN2, 0);//КРУТИМ НЕЗАВИСИМЫЕ КРАНЫ

if (digitalRead(START_STOP_BUTTON)==0){//ОБРАБОТКА КНОПКИ
  while(digitalRead(START_STOP_BUTTON)==0){}//Ждем отпускания кнопки
  if (MODE==0)MODE=1;else MODE=0;//ЗАПУСК ИЛИ СТОП
  }


if( show_parameters==1){//НЕОБХОДИМОСТЬ ВЫДАТЬ В ПОРТ НАСТРОЙКИ
show_parameters=0;//ПОКАЗАЛИ
    Serial.println("--------------------------");
    Serial.print("MAX_HEAT="); Serial.println(MAX_HEAT);
    Serial.print("WORK_Temperature_0="); Serial.println(WORK_Temperature_0);
    Serial.print("MAX_Temperature_1="); Serial.println(MAX_Temperature_1);
    Serial.print("SINGLE_KRAN1_MIN_T="); Serial.println(SINGLE_KRAN1_MIN_T);
    Serial.print("SINGLE_KRAN1_MAX_T="); Serial.println(SINGLE_KRAN1_MAX_T);
    Serial.print("SINGLE_KRAN2_MIN_T="); Serial.println(SINGLE_KRAN2_MIN_T);
    Serial.print("SINGLE_KRAN2_MAX_T="); Serial.println(SINGLE_KRAN2_MAX_T);
    Serial.print("KRAN1_MIN_T="); Serial.println(KRAN1_MIN_T);
    Serial.print("KRAN1_MAX_T="); Serial.println(KRAN1_MAX_T);
    Serial.print("KRAN2_MIN_T="); Serial.println(KRAN2_MIN_T);
    Serial.print("KRAN2_MAX_T="); Serial.println(KRAN2_MAX_T);
    Serial.print("KRAN3_MIN_T="); Serial.println(KRAN3_MIN_T);
    Serial.print("KRAN3_MAX_T="); Serial.println(KRAN3_MAX_T);
    Serial.print("ADD2PHASE="); Serial.println(ADD2PHASE);
    Serial.print("STABLE_TIME="); Serial.println(STABLE_TIME);
    Serial.println("");
}
show_values++;
if( show_values==600)CHECK_HEAT_STATE=1;//ПОДРУЛИВАЕМ НАГРЕВ
if( show_values==600){;//НЕОБХОДИМОСТЬ ВЫДАТЬ В ПОРТ ЗНАЧЕНИЯ АЦП И ПРОЧИХ
show_values=0;//ПОКАЗАЛИ
    Serial.println("--------------------------");
    Serial.print("Temperature_0="); Serial.println(Temperature_0);
    Serial.print("Temperature_1="); Serial.println(Temperature_1);
    Serial.print("Temperature_2="); Serial.println(Temperature_2);
    Serial.print("Temperature_3="); Serial.println(Temperature_3);
    Serial.print("Temperature_4="); Serial.println(Temperature_4);
    Serial.print("Temperature_5="); Serial.println(Temperature_5);
    Serial.print("CURRENT_HEAT="); Serial.println(CURRENT_HEAT);
    Serial.print("MODE="); Serial.println(MODE);
    String SELF_TIME;
    if (MODE==2){
      SELF_TIME=STABLE_TIME_COUNTER;
       SELF_TIME=  SELF_TIME + "min " + SECONDS +"sec";}
    else{SELF_TIME="0";}
    Serial.print("SELF_TIME="); Serial.println(SELF_TIME);
    Serial.println("");
}
}

  //===================================
  //Циклический опрос буфера
  //===================================
void read_uart_buffer(){
  char next_read;
  while(Serial.available())//пока есть данные
    { next_read=Serial.read(); 
      if(next_read==0x0A||next_read==0x0D){uart_get_line();return;}
      RX_buffer=RX_buffer+next_read;
      }}
//===================================      
//   ОБРАБОТКА ПОЛУЧЕННОЙ СТРОКИ
//===================================
  void uart_get_line(){
int    Separator=RX_buffer.indexOf("=");
    
    if(Separator>0){//Если есть знак равенства
    String ParamName=RX_buffer.substring(0,Separator);//Строка до равно
    String ParamValue=RX_buffer.substring(Separator+1);//Строка после
    int intVal=ParamValue.toInt();
    float floatVal=ParamValue.toFloat();
if(ParamName=="MAX_HEAT"){MAX_HEAT=intVal;EEPROM_write_int(EE_MAX_HEAT,MAX_HEAT);}//Максимальная мощность ТЕНа %
if(ParamName=="WORK_Temperature_0"){WORK_Temperature_0=floatVal;EEPROM_write_float(EE_WORK_Temperature_0,WORK_Temperature_0);}//Рабочая температура браги
if(ParamName=="MAX_Temperature_1"){MAX_Temperature_1=floatVal;EEPROM_write_float(EE_MAX_Temperature_1,MAX_Temperature_1);}//Максимальная температура пара(при достижении - выключить)
if(ParamName=="KRAN1_MIN_T"){KRAN1_MIN_T=floatVal;EEPROM_write_float(EE_KRAN1_MIN_T,KRAN1_MIN_T);}//Диапазон вкл-выкл крана
if(ParamName=="KRAN1_MAX_T"){KRAN1_MAX_T=floatVal;EEPROM_write_float(EE_KRAN1_MAX_T,KRAN1_MAX_T);}//Диапазон вкл-выкл крана
if(ParamName=="KRAN2_MIN_T"){KRAN2_MIN_T=floatVal;EEPROM_write_float(EE_KRAN2_MIN_T,KRAN2_MIN_T);}//Диапазон вкл-выкл крана
if(ParamName=="KRAN2_MAX_T"){KRAN2_MAX_T=floatVal;EEPROM_write_float(EE_KRAN2_MAX_T,KRAN2_MAX_T);}//Диапазон вкл-выкл крана
if(ParamName=="KRAN3_MIN_T"){KRAN3_MIN_T=floatVal;EEPROM_write_float(EE_KRAN3_MIN_T,KRAN3_MIN_T);}//Диапазон вкл-выкл крана
if(ParamName=="KRAN3_MAX_T"){KRAN3_MAX_T=floatVal;EEPROM_write_float(EE_KRAN3_MAX_T,KRAN3_MAX_T);}//Диапазон вкл-выкл крана

if(ParamName=="SINGLE_KRAN1_MIN_T"){SINGLE_KRAN1_MIN_T=floatVal;EEPROM_write_float(EE_SINGLE_KRAN1_MIN_T,SINGLE_KRAN1_MIN_T);}//Диапазон вкл-выкл крана
if(ParamName=="SINGLE_KRAN1_MAX_T"){SINGLE_KRAN1_MAX_T=floatVal;EEPROM_write_float(EE_SINGLE_KRAN1_MAX_T,SINGLE_KRAN1_MAX_T);}//Диапазон вкл-выкл крана
if(ParamName=="SINGLE_KRAN2_MIN_T"){SINGLE_KRAN2_MIN_T=floatVal;EEPROM_write_float(EE_SINGLE_KRAN2_MIN_T,SINGLE_KRAN2_MIN_T);}//Диапазон вкл-выкл крана
if(ParamName=="SINGLE_KRAN2_MAX_T"){SINGLE_KRAN2_MAX_T=floatVal;EEPROM_write_float(EE_SINGLE_KRAN2_MAX_T,SINGLE_KRAN2_MAX_T);}//Диапазон вкл-выкл крана

if(ParamName=="ADD2PHASE"){ADD2PHASE=intVal; while(ADD2PHASE>10000){ADD2PHASE-=10000;}EEPROM_write_int(EE_ADD2PHASE,ADD2PHASE);}//Настройка смещения фазы для фазного включения ТЭНА
if(ParamName=="STABLE_TIME"){STABLE_TIME=intVal;EEPROM_write_int(EE_STABLE_TIME,STABLE_TIME);}//Время стабилизации(минуты)
if(ParamName=="MODE"){MODE=intVal;}//ПЕРЕКЛЮЧЕНИЕ РЕЖИМА
}    
show_parameters=1;//Выведем все параметры.
RX_buffer="";}//ОЧИСТКА БУФЕРА ПЕРЕД ВЫХОДОМ
//===================================
//Получаем температуру из АЦП
//===================================
float get_temperature_from_adc(int adc_num){//A0 A1 и тд
      pinMode( adc_num, INPUT );
   int t = analogRead( adc_num );
    float tr = TERMO_R1/(1023.0 /t - 1);//сопротивление считаем
    float steinhart;
    steinhart = tr / THERMISTOR_R; // (R/Ro)
    steinhart = log(steinhart); // ln(R/Ro)
    steinhart /= B; // 1/B * ln(R/Ro)
    steinhart += 1.0 / (NOMINAL_T + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart; // Invert
    steinhart -= 273.15; 
    return(steinhart);}

//===================================
//Запись в EEPROM;
//===================================
void EEPROM_write_float(int addr, float val) // запись в ЕЕПРОМ
{byte *x = (byte *)&val;  for(byte i = 0; i < 4; i++) EEPROM.write(i+addr, x[i]);}

float EEPROM_read_float(int addr) // чтение из ЕЕПРОМ
{byte x[4]; for(byte i = 0; i < 4; i++) x[i] = EEPROM.read(i+addr);float *y = (float *)&x;  return y[0];}

void EEPROM_write_int(int addr, int val) // запись в ЕЕПРОМ
{byte *x = (byte *)&val;  for(byte i = 0; i < 2; i++) EEPROM.write(i+addr, x[i]);}

int EEPROM_read_int(int addr) // чтение из ЕЕПРОМ
{byte x[2]; for(byte i = 0; i < 2; i++) x[i] = EEPROM.read(i+addr);int *y = (int *)&x;  return y[0];}

//   ВАРИАНТ С ПРОПУСКОМ ПЕРИОДОВ
//===================================
//  ПРЕРЫВАНИЕ INT0
//===================================
void NULL_DETECTED(){
  if(CURRENT_HEAT==100)digitalWrite(TEN, 1);
 // if(NULL_DETECTED_STATE==0){//ТОЛЬКО ЕСЛИ ЖДВЛИ ПРЕРЫВАНИЕ
 byte off_delay=5;//Выкл чуть раньше
  TCCR1B=0;//СТОП ТАЙМЕРА С ДЕЛИТЕЛЕМ 8 (цикл полупериода = 20000 тиков)
  unsigned int PERIOD=TCNT1;
 PERIOD -= off_delay;//ЧУТЬ ОБРЕЖЕМ ПЕРИОД
  TCNT1=0;
float COLDpercent=(100.0 - CURRENT_HEAT)/100.0;
int setHeat=PERIOD *COLDpercent;//ЗАДЕРЖКА ВКЛЮЧЕНИЯ НАГРЕВА, тиков таймера
setHeat += ADD2PHASE * 2 ;//ЗАДЕРЖКА ВКЛЮЧЕНИЯ НАГРЕВА
// unsigned long  setHEATtozero=ADD2PHASE * 2 - off_delay;//Всегда выключаемся в конце полупериода
 
if   (setHeat>PERIOD){setHeat=setHeat-PERIOD;}//ЕСЛИ ВЫХОДИМ ЗА ПЕРИОД - РАБОТАЕМ В ЭТОМ
//int offimpulse=;//Время дать выкл
//(ОБРАБАТЫВАЕМ ОКОЛОНУЛЕВЫЕ ЗАПРОСЫ)
int delta=setHeat+off_delay-(ADD2PHASE * 2);
if   (delta>=0 && delta<off_delay){setHeat=setHeat-delta-1;}
 OCR1A=setHeat;//1мкс=2 тика
OCR1B=ADD2PHASE * 2;//Установим
//OCR1B=setHeat+off_delay;//Установим
  TCCR1B=2;//СТАРТ ТАЙМЕРА С ДЕЛИТЕЛЕМ 8 (цикл полупериода = 20000 тиков)

  WAVE_COUNTER++;
if (WAVE_COUNTER==100){WAVE_COUNTER=0;SECONDS++;}
if (SECONDS==60){SECONDS=0;STABLE_TIME_COUNTER++;}
NULL_DETECTED_STATE=1;
  //}
  }

ISR(TIMER1_COMPA_vect)
{if(CURRENT_HEAT>0){digitalWrite(TEN, 1);}NULL_DETECTED_STATE=0;} 

ISR(TIMER1_COMPB_vect)
{digitalWrite(TEN, 0);} 
//===================================
//  УСТАНОВКА ДИОДОВ
//===================================
void setLed(byte mode_num){
if(mode_num==1)digitalWrite(MODE_LED1, 1);else digitalWrite(MODE_LED1, 0);
if(mode_num==2)digitalWrite(MODE_LED2, 2);else digitalWrite(MODE_LED2, 0);
if(mode_num==3)digitalWrite(MODE_LED3, 3);else digitalWrite(MODE_LED3, 0);
}
