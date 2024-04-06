//Релиз от 25,06,2020
//Параметры в структуре от Watashi
//Рампа для низа
//Преднагрев 5 секунд 3% мощности
//управление включением/выключением пайки и переключением профилей с РС
//сохранение и загрузка профилей с/на диск PC
//ПИД по измерению
//По нажатию Cancel в течение 5 сек перезаписывается текущий профиль на предустановленный
//15 профилей
//Возможность "добавить" температуру НИ во время пайки клавишами вверх/вниз

#include <EEPROM.h>
#include <UTFT.h>

#define resist_keyboard 1 //Раскоментировать, если клавиатура резистивная
//#define params_from_pc 1 //Раскоментировать, если нужно передавать параметры в РС
//#define no_pc 1;       //Раскоментировать, если вообще не надо связь с РС и закоментировать params_from_pc


//Секция экрана-----------------------------------------
UTFT myGLCD(CTE40, 38, 39, 40, 41);
//UTFT myGLCD(CTE32HR, 38, 39, 40, 41);
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t BigFontRus[]; //Кирилица
extern uint8_t SevenSegNumFont[];
//-------------------------------------------------------

//Секция ПИД----------------------------------------------
float integra, integra2;  //интегральные составляющие ВИ и НИ
float e1, p1, d1, e2, p2, d2; //ошибка регулирования, П-составляющая, Д-составляющая для ВИ и НИ соответственно
#define i_min 0.0         //минимум И составляющей
#define i_max 100.0       //максимум И составляющей

unsigned int Setpoint1;
double Input1;
unsigned int  Output1;
unsigned int  Output2;
double Input2;
#define SENSOR_SAMPLING_TIME 250 //время чтения температуры и пересчёта ПИД(милисекунды)
//-----------------------------------------------------------

//Секция графиков--------------------------------------------
int Xgr, XTgr = 2; // x координата графика
byte Cgr, CTgr = 0;
float Input_f1, Input_f2;
//-----------------------------------------------------------

//секция алгоритма Брезенхема--------------------------------
int er1 = 1;
int er2 = 1;
int reg1;
int reg2;
boolean out1;
boolean out2;
//-----------------------------------------------------------

//Секция кнопок, пинов подключеня-----------------------------------------
//RelayPin "1"-ВЕРХНИЙ нагреватель
//RelayPin "2"-НИЖНИЙ нагреватель
#define RelayPin1 7  //назначаем пин "ВЕРХНЕГО" нагревателя
#define RelayPin2 6  //назначаем пин "НИЖНЕГО" нагревателя
// Выходы реле
#define P1_PIN 9    //назначаем пин реле 1
#define P2_PIN 10   //назначаем пин реле 2
#define P3_PIN 11   //назначаем пин реле 3
#define P4_PIN 12   //назначаем пин реле 4
byte buzzerPin = 8;

#ifdef resist_keyboard
//#define A_PINS_BASE 100 // номер с которого начинается нумерация наших "псевдо-кнопок".
#define PIN_RIGHT 100
#define PIN_UP 101
#define PIN_DOWN 102
#define PIN_LEFT 103
#define PIN_SELECT 104

struct A_PIN_DESC { // определяем  структуру которой будем описывать какое значение мы ожидаем для каждого псевдо-пина
  byte pinNo; // номер пина
  int expectedValue;// ожидаемое значение
};
A_PIN_DESC expected_values[] = { // ожидаемые значения для псевдо-кнопок
  { PIN_RIGHT, 0},
  { PIN_UP, 145},
  { PIN_DOWN, 351},
  { PIN_LEFT, 513},
  { PIN_SELECT, 750}
};
#define A_PINS_COUNT sizeof(expected_values)/sizeof(A_PIN_DESC) // вычисляем сколько у нас всего "псевдо-кнопок" заданно.
#define A_POSSIBLE_ABERRATION 70 // допустимое отклонение analogRead от ожидаемого значения, при котором псевдо кнопка считается нажатой
bool digitalReadA(byte pinNo) {

  for (byte i = 0; i < A_PINS_COUNT; i++) { // ищем описание нашего всевдо-пина
    A_PIN_DESC pinDesc = expected_values[i]; // берем очередное описание
    if (pinDesc.pinNo == pinNo) { // нашли описание пина?
      int value = analogRead(A0); // производим чтение аналогово входа
      return (abs(value - pinDesc.expectedValue) < A_POSSIBLE_ABERRATION); // возвращаем HIGH если отклонение от ожидаемого не больше чем на A_POSSIBLE_ABERRATION
    }
  }
  return LOW; // если не нашли описания - считаем что пин у нас LOW
}
//Назначаем пины кнопок управления
int upSwitchPin = PIN_UP;
int downSwitchPin = PIN_DOWN;
int cancelSwitchPin = PIN_LEFT;
int okSwitchPin = PIN_SELECT;
#endif

#ifndef resist_keyboard
byte upSwitchPin = 21;     //пин кнопки вверх
byte downSwitchPin = 20;   //пин кнопки вниз
byte cancelSwitchPin = 19; //пин кнопки отмена или назад
byte okSwitchPin = 18 ;    //пин кнопки ОК или подтверждения
#endif

//назначаем пины усилителя термопары MAX6675 "ВЕРХНЕГО" нагревателя   clk=sck cs=cs do=so
byte thermoCLK = 14;  //=sck
byte thermoCS = 15;   //=cs
byte thermoDO = 16;   //=so
//назначаем пины усилителя термопары MAX6675 "НИЖНЕГО" нагревателя clk=sck cs=cs do=so
byte thermoCLK2 = 14;  //=sck
byte thermoCS2 = 17;   //=cs
byte thermoDO2 = 16;   //=so

//состояние кнопок по умолчанию
int upSwitchState = 0;
int downSwitchState = 0;
int cancelSwitchState = 0;
int okSwitchState = 0;

//переменные для кнопок
byte Hselect = 0;             //выбор параметра для изменения во время пайки
long ms_button = 0;           //время при котором была нажата кнопка
boolean  button_state = false;//признак нажатой кнопки
boolean  button_long_state = false;//признак длинного нажатия кнопки
boolean button_state1 = false;

//------------------------------------------------------------------------

//Секция профиля----------------------------------------------------------
struct pr {                        //основные поля профиля
  byte profileSteps;               //количество шагов профиля
  unsigned int Setpoint2;          //заданная температура нижнего нагревателя
  byte rampRateStep[3];            //скорость роста температуры
  byte dwellTimerStep[3];          //установленное время перехода на следующий шаг
  unsigned int temperatureStep[3]; //заданные температуры для шагов профиля ВИ
  byte min_pwr_TOPStep[3];         //минимальная мощность верхнего нагревателя
  byte max_pwr_TOPStep[3];         //максимальная мощность верхнего нагревателя
  byte BottomRampRateStep;         //скорость нагрева нижним нагревателем
  byte kp1;                        //пропорциональный коэффициент ВИ
  byte ki1;                        //интегральный коэффициент ВИ
  byte kd1;                        //дифференциальный коэффициент ВИ
  byte kp2;                        //пропорциональный коэффициент НИ
  byte ki2;                        //дифференциальный коэффициент НИ
  byte kd2;                        //дифференциальный коэффициент НИ
  byte tableSize;                  //размер стола
  byte min_pwr_BOTTOM;             //минимальная мощность нижнего нагревателя
  byte max_pwr_BOTTOM;             //максимальная мощность нижнего нагревателя
};

unsigned int SizeProfile = sizeof(pr);// длинна поля данных
pr profile;                   //структура для параметров
byte currentProfile = 1;      //текущий профиль
byte currentStep = 1;         //текущий шаг профиля
byte profileName;             //имя профиля
byte editStep = 0;            //текущий шаг, при редактировании

String profile_names[15] = {
  "СНЯТИЕ БЕССВИНЕЦ ",
  " СНЯТИЕ СВИНЕЦ   ",
  "УС-ВКА БЕССВИНЕЦ ",
  " УС-ВКА СВИНЕЦ   ",
  " НИЖНИЙ ПОДОГРЕВ ",
  "ПРОФИЛЬ          ",
  "ПРОФИЛЬ          ",
  "ПРОФИЛЬ          ",
  "ПРОФИЛЬ          ",
  "ПРОФИЛЬ          ",
  "ПРОФИЛЬ          ",
  "ПРОФИЛЬ          ",
  "ПРОФИЛЬ          ",
  "ПРОФИЛЬ          ",
  "ПРОФИЛЬ          "
};
//------------------------------------------------------------------------

//Секция флагов-----------------------------------------------------------
boolean TopStart = false;     //признак запуска ВИ
byte flag = 0;                //флаг для фиксации стартовой температуры
byte x = 1;                   //переменная для перехода на нужный шаг при горячей плате
boolean alarmOn = false;      //признак ошибки
boolean updateScreen = true;  //признак обновления экрана
boolean is_bottomRamp = false;        //признак работы рампы низа
//------------------------------------------------------------------------

//Секция переменных общего назначения-------------------------------------
unsigned int SP2;
unsigned int bottomTemp;
unsigned int startTemp;
unsigned int setpointRamp;
int counter;
long previousMillis; //это для счетчиков
unsigned long nextRead1; //переменная для обновления текущей температуры
unsigned int tc1;
unsigned int tc2;
byte Secs = 0;
unsigned long prev_millis = 0;
int i = 0;
float Input1_spd;
float Input2_spd;

int Input1_fraction;
int Input2_fraction;
//------------------------------------------------------------------------

//секция ввода/вывода для ПЭВМ-----------------------------------------------
#ifndef no_pc
char buf[20];   //буфер вывода сообщений через сом порт
#endif
#ifdef params_from_pc
int b1, b2 = 0;
char buf1[70];  //буфер вывода праметров через сом порт
String ttydata; //принятая строка параметров с сом порта от ПЭВМ
#endif
//---------------------------------------------------------------------------

//these are the different states of the sketch. We call different ones depending on conditions
// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE : byte
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_MENU_STEPS,
  REFLOW_STATE_MENU_TABLE_SIZE,
  REFLOW_STATE_MENU_BOTTOM_HEAT,
  REFLOW_STATE_MENU_BOTTOM_PWR_MIN,
  REFLOW_STATE_MENU_BOTTOM_PWR_MAX,
  REFLOW_STATE_MENU_BOTTOM_RAMP,
  REFLOW_STATE_MENU_STEP_RAMP,
  REFLOW_STATE_MENU_STEP_TARGET,
  REFLOW_STATE_MENU_TOP_PWR_MIN,
  REFLOW_STATE_MENU_TOP_PWR_MAX,
  REFLOW_STATE_MENU_STEP_DWELL,
  REFLOW_STATE_MENU_BOTTOM_P,
  REFLOW_STATE_MENU_BOTTOM_I,
  REFLOW_STATE_MENU_BOTTOM_D,
  REFLOW_STATE_MENU_TOP_P,
  REFLOW_STATE_MENU_TOP_I,
  REFLOW_STATE_MENU_TOP_D,
  REFLOW_STATE_PRE_HEATER,
  REFLOW_STATE_BOTTOM_STEP_RAMP,
  REFLOW_STATE_STEP_RAMP,
  REFLOW_STATE_TABLE_SIZE,
  REFLOW_STATE_STEP,
  REFLOW_STATE_STEP_DWELL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_ERROR,
  REFLOW_STATE_PROFILE_INIT
}
reflowState_t;

typedef enum REFLOW_STATUS : byte //this is simply to check if reflow should be running or not
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
}
reflowStatus_t;
reflowStatus_t reflowStatus;
reflowState_t reflowState;

void loadProfile()//this function loads whichever profile currentProfile variable is set to
{
  for (byte j = 0; j <= SizeProfile - 1; j++) {
    EEPROM.get((currentProfile - 1)*SizeProfile, profile);
  }
#ifdef params_from_pc
  sprintf (buf1, "PR%01d%01d%03d%03d%03d%02d%02d%02d%03d%03d%03d%03d%03d%03d%03d%03d%03d%03d%03d%03d%02d%02d%02d%02d%02d%02d%02d\r\n",
           profile.profileSteps % 10, profile.tableSize % 10, int(profile.Setpoint2), profile.min_pwr_BOTTOM, profile.max_pwr_BOTTOM,
           (profile.rampRateStep[0]) % 100, (profile.rampRateStep[1]) % 100, (profile.rampRateStep[2]) % 100,
           profile.temperatureStep[0], profile.temperatureStep[1], profile.temperatureStep[2],
           profile.min_pwr_TOPStep[0], profile.min_pwr_TOPStep[1], profile.min_pwr_TOPStep[2],
           profile.max_pwr_TOPStep[0], profile.max_pwr_TOPStep[1], profile.max_pwr_TOPStep[2],
           profile.dwellTimerStep[0], profile.dwellTimerStep[1], profile.dwellTimerStep[2],
           profile.kp2 % 100, profile.kp1 % 100, profile.ki2 % 100, profile.ki1 % 100, profile.kd2 % 100, profile.kd1 % 100, profile.BottomRampRateStep % 100);
  Serial.print(buf1);
  delay(100);
#endif
  SP2 = profile.Setpoint2;
  return;
}

void SaveProfile () {  //  сохранение текущего профиля
  for (byte j = 0; j <= SizeProfile - 1; j++) {
    EEPROM.put((currentProfile - 1)*SizeProfile, profile);
  }
}

#ifdef params_from_pc
void ParseParameters ()//парсер параметров
{
  String st;
  sprintf (buf, "TXRecive Params%02d\r\n", ttydata.length());
  Serial.print(buf);
  st = ttydata.substring(0, 1);
  profile.profileSteps = st.toInt();
  st = ttydata.substring(1, 2);
  profile.tableSize = st.toInt();
  st = ttydata.substring(2, 5);
  profile.Setpoint2 = st.toInt();
  st = ttydata.substring(5, 8);
  profile.min_pwr_BOTTOM = st.toInt();
  st = ttydata.substring(8, 11);
  profile.max_pwr_BOTTOM = st.toInt();
  for (byte i = 0; i < 3; i++) {
    st = ttydata.substring(11 + i * 2, 13 + i * 2);
    profile.rampRateStep[i] = st.toInt();
    st = ttydata.substring(17 + i * 3, 20 + i * 3);
    profile.temperatureStep[i] = st.toInt();
    st = ttydata.substring(26 + i * 3, 29 + i * 3);
    profile.min_pwr_TOPStep[i] = st.toInt();
    st = ttydata.substring(35 + i * 3, 38 + i * 3);
    profile.max_pwr_TOPStep[i] = st.toInt();
    st = ttydata.substring(44 + i * 3, 47 + i * 3);
    profile.dwellTimerStep[i] = st.toInt();
  }
  st = ttydata.substring(53, 55);
  profile.kp2 = st.toInt();
  st = ttydata.substring(55, 57);
  profile.kp1 = st.toInt();
  st = ttydata.substring(57, 59);
  profile.ki2 = st.toInt();
  st = ttydata.substring(59, 61);
  profile.ki1 = st.toInt();
  st = ttydata.substring(61, 63);
  profile.kd2 = st.toInt();
  st = ttydata.substring(63, 65);
  profile.kd1 = st.toInt();
  st = ttydata.substring(65, 67);
  profile.BottomRampRateStep = st.toInt();
  ttydata = "";
  SaveProfile();
  updateScreen = true;
  return;
}
#endif

void setup()
{
#ifndef no_pc
  Serial.begin(9600);
#endif
  //setup reley pins as outputs
  pinMode(P1_PIN, OUTPUT);
  pinMode(P2_PIN, OUTPUT);
  pinMode(P3_PIN, OUTPUT);
  pinMode(P4_PIN, OUTPUT);
  pinMode(thermoCLK, OUTPUT);
  pinMode(thermoCLK2, OUTPUT);
  pinMode(thermoDO, INPUT);
  pinMode(thermoDO2, INPUT);
  pinMode(thermoCS, OUTPUT);
  pinMode(thermoCS2, OUTPUT);
#ifndef resist_keyboard
  pinMode (upSwitchPin, INPUT_PULLUP); //подключен подтягивающий резистор
  pinMode (downSwitchPin, INPUT_PULLUP); //подключен подтягивающий резистор
  pinMode (cancelSwitchPin, INPUT_PULLUP); //подключен подтягивающий резистор
  pinMode (okSwitchPin, INPUT_PULLUP); //подключен подтягивающий резистор
#endif
#ifdef resist_keyboard
  pinMode(A0, INPUT);
#endif
  pinMode(RelayPin1, OUTPUT);
  pinMode(RelayPin2, OUTPUT);
  myGLCD.InitLCD();
  myGLCD.clrScr();
  myGLCD.setFont(BigFont);
  myGLCD.setColor(VGA_GREEN);
  myGLCD.print(F("ARDUINO MEGA2560"), CENTER, 100);
  myGLCD.setColor(VGA_RED);
  myGLCD.print(F("REWORK STATION"), CENTER, 130);
  myGLCD.setColor(250, 180, 000);
  myGLCD.print(F("TFT-LCD 480*320 MAX6675_Dimmer"), CENTER, 160);
  myGLCD.setColor(VGA_SILVER);
  myGLCD.print(F("RevoloveR_Dmitrysh_V4.1.1"), CENTER, 190);
  //Мелодия приветствия Марио
  tone(buzzerPin, 1318, 150);
  delay(150);
  tone(buzzerPin, 1318, 300);
  delay(300);
  tone(buzzerPin, 1318, 150);
  delay(300);
  tone(buzzerPin, 1046, 150);
  delay(150);
  tone(buzzerPin, 1318, 300);
  delay(300);
  tone(buzzerPin, 1568, 600);
  delay(600);
  tone(buzzerPin, 784, 600);
  delay(600);
  noTone(buzzerPin);
  //
  loadProfile();//вызов функции loadProfile для загрузки данных профиля из eeprom
  myGLCD.clrScr();
  nextRead1 = millis();
  attachInterrupt(0, Dimming, RISING); // настроить порт прерывания(0 или 1) 2й или 3й цифровой пин
}


void loop()
{
#ifdef resist_keyboard
  //Считываем состояние кнопок управления для резистивной клавиатуры
  upSwitchState = digitalReadA(upSwitchPin);
  downSwitchState = digitalReadA(downSwitchPin);
  cancelSwitchState = digitalReadA(cancelSwitchPin);
  okSwitchState = digitalReadA(okSwitchPin);
#endif

#ifndef resist_keyboard
  //Считываем состояние кнопок управления для обычных кнопок
  upSwitchState = !digitalRead(upSwitchPin);
  downSwitchState = !digitalRead(downSwitchPin);
  cancelSwitchState = !digitalRead(cancelSwitchPin);
  okSwitchState = !digitalRead(okSwitchPin);
#endif

  unsigned long currentMillis = millis();


  //отключил бузер при нажатии на кнопки (нервирует), кому надо раскоментируйте ниже 6 строк
  //if (upSwitchState == HIGH || downSwitchState == HIGH || cancelSwitchState == HIGH || okSwitchState == HIGH)
  //{
  //tone(buzzerPin, 1045);
  //delay(100);
  //noTone(buzzerPin);
  //}
  if (reflowState == REFLOW_STATE_COMPLETE || alarmOn) {
    if (i < 2 && cancelSwitchState == LOW) {
      alarmOn = true;
      tone(buzzerPin, 1046);
      delay(100);
      noTone(buzzerPin);
      delay(100);
      tone(buzzerPin, 1046);
      delay(100);
      noTone(buzzerPin);
      delay(100);
      tone(buzzerPin, 1046);
      delay(100);
      noTone(buzzerPin);
      delay(100);
      tone(buzzerPin, 1046);
      delay(100);
      noTone(buzzerPin);
      delay(100);
      i++;
    }
    else {
      i = 0;
      alarmOn = false;
    }
  }

  switch (reflowState)
  {
    case REFLOW_STATE_IDLE:
      TopStart = false;
      currentStep = 1;
      counter = 0;
      setpointRamp = 0;
      x = 1;             //устанавливаем переменную в исходное состояние
      flag = 0;         //после остановки профиля сбрасываем флаг
      Output1 = 0;
      Output2 = 0;

      //Настройка экрана Рабочий режим
      if (updateScreen) {
        myGLCD.clrScr();
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("`", 175, 115);
        myGLCD.textRus("`/с", 175, 133);
        myGLCD.textRus("`/с", 175, 245);
        myGLCD.textRus("`", 175, 227);
        myGLCD.textRus("`", 420, 120);
        myGLCD.textRus("`", 420, 232);
        myGLCD.setColor(100, 100, 100);  // "вкладка"
        myGLCD.drawLine(195, 108, 210, 78); //   для
        myGLCD.drawLine(210, 78, 475, 78); // мощности
        myGLCD.drawLine(475, 78, 475, 108); //  верха
        myGLCD.drawLine(195, 106, 475, 106); //
        myGLCD.setColor(100, 100, 100);  // "вкладка"
        myGLCD.drawLine(195, 221, 210, 191); //   для
        myGLCD.drawLine(210, 191, 475, 191); // мощности
        myGLCD.drawLine(475, 191, 475, 221); //   низа
        myGLCD.drawLine(197, 219, 475, 219); //
        //myGLCD.setColor(30,30,30);
        myGLCD.textRus("График Температуры", 90, 20);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("Мощность    %", 230, 84);
        myGLCD.textRus("Мощность    %", 230, 198);
        myGLCD.drawRoundRect(3, 108, 478, 179);
        myGLCD.drawRoundRect(3, 221, 478, 291);
        myGLCD.setColor(40, 40, 40);    // верхний график
        myGLCD.drawRect(1, 1, 479, 72); //температуры
        //myGLCD.drawRect(224,221,478,291);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.drawLine(224, 108, 224, 179); //разделитель
        myGLCD.drawLine(224, 221, 224, 291); //
        myGLCD.textRus("ВЕРХ->", 5, 115);
        myGLCD.textRus("НИЗ ->", 5, 227);
        myGLCD.textRus("Рост", 5, 133);
        myGLCD.textRus("Рост", 5, 245);
        updateScreen = false;
      }

      if (millis() > nextRead1)
      {
        // Read thermocouples next sampling period
        nextRead1 = millis() + SENSOR_SAMPLING_TIME;
        Input1 = Input1 * 0.8 + 0.2 * (max6675_read_temp (thermoCLK, thermoCS, thermoDO));
        Input2 = Input2 * 0.8 + 0.2 * (max6675_read_temp (thermoCLK2, thermoCS2, thermoDO2));
        tc1 = Input1;
        tc2 = Input2;

#ifdef params_from_pc
        //блок обработки данных с ПЭВМ--------------------------------------------------------------------

        b2 = Serial.available();
        if (b2 > 0) {
          b1 = Serial.read();
          if ((b1 == 'P') ) {
            sprintf (buf, "TXSend Params\r\n");
            Serial.println(buf);
            delay(50);
            loadProfile();
          }
          if (b1 == 'N') {
            ttydata = ttydata + Serial.readString();
            sprintf (buf, "TXRecive Params %02d\r\n", b2);
            Serial.println(buf);
            Serial.println("TX" + ttydata);
            if (ttydata.length() == 67)
            {
              Serial.println("TX+Parsing...\r\n");
              ParseParameters();
            }
            if (ttydata.length() > 67)
            {
              Serial.println("TXERROR on reciever\r\n");
              ttydata = "";
            }
          }
          if (b1 == 'S') {
            Serial.println("TXPC_Start");
            delay(100);
            goto START;
          }
          if (b1 == 'U') {
            currentProfile = currentProfile + 1;
            if (currentProfile >= 6) currentProfile = 1;
            loadProfile();//вызов функции loadProfile для загрузки данных профиля из eeprom
          }
          if (b1 == 'D') {
            currentProfile = currentProfile - 1;
            if (currentProfile <= 0) currentProfile = 5;
            loadProfile();//вызов функции loadProfile для загрузки данных профиля из eeprom
          }
        }

        //--------------------------------------------------------------------------------------------------
#endif
#ifndef no_pc
        sprintf (buf, "OK%03d%03d%03d%03d%03d\r\n", (Output1), (Output2), tc1, tc2, (profileName));
        Serial.print(buf);
#endif
        if (Input1 <= -0) {
          myGLCD.setColor(VGA_BLACK);
          myGLCD.drawRoundRect(340, 100, 460, 180);
          myGLCD.setFont(BigFont);
          myGLCD.setColor(VGA_RED);
          myGLCD.print("ERROR", 250, 140);
        } else {
          myGLCD.setFont(SevenSegNumFont);
          myGLCD.setColor(VGA_SILVER);
          myGLCD.printNumI(tc1, 235, 120, 4, '0');
        }
        if (Input2 <= -0) {
          myGLCD.setFont(BigFont);
          myGLCD.setColor(VGA_RED);
          myGLCD.print("ERROR", 250, 250);
        } else {
          myGLCD.setFont(SevenSegNumFont);
          myGLCD.setColor(VGA_SILVER);
          myGLCD.printNumI(tc2, 235, 232, 4, '0');
        }
      }

      myGLCD.setFont(BigFont);
      myGLCD.setColor(250, 180, 000);
      myGLCD.printNumI(currentProfile, 440, 300, 2);
      myGLCD.setColor(VGA_SILVER);
      myGLCD.printNumI(profile.temperatureStep[editStep], 100, 115, 4, ' ');
      myGLCD.printNumI(SP2, 100, 227, 4, ' ');

      profileName = currentProfile;
      myGLCD.setFont(BigFontRus);
      myGLCD.setColor(VGA_LIME);
      myGLCD.textRus(profile_names[profileName - 1], 180, 300);


      if (upSwitchState == HIGH && ( millis() - ms_button) > 500) //if up switch is pressed go to next profile
      {
        ms_button =  millis();
        currentProfile = currentProfile + 1;
        if (currentProfile >= 16)//if currentProfile = 5 and up is pressed go back to profile 1
        {
          currentProfile = 1;
        }
        loadProfile();//вызов функции loadProfile для загрузки данных профиля из eeprom

      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 500) //same as above just go down one profile
      {
        ms_button =  millis();
        currentProfile = currentProfile - 1;
        if (currentProfile <= 0)
        {
          currentProfile = 15;
        }
        loadProfile();//вызов функции loadProfile для загрузки данных профиля из eeprom
      }

      if (cancelSwitchState == HIGH && !button_state1 && ( millis() - ms_button) > 100)
      {
        ms_button = millis();
        button_state1 = true;
        button_long_state = false;
      }
      //держим "cancel" 5+ секунды и заходим в инициализацию профиля
      if (cancelSwitchState == HIGH && !button_long_state && ( millis() - ms_button) > 5000)
      {
        button_long_state = false;
        reflowState = REFLOW_STATE_PROFILE_INIT;
      }

      //фиксируем момент нажатия кнопки "ОК" + защита от дребезга
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
        button_long_state = false;
      }
      //держим "ОК" 3+ секунды и заходим в меню настроек
      if (okSwitchState == HIGH && !button_long_state && ( millis() - ms_button) > 1500)
      {
        button_long_state = true;
        button_state = false;
        reflowState = REFLOW_STATE_MENU_STEPS;
        //update next screen
        updateScreen = true;

      }


      //включаем пайку, кнопка сработает после отпускания "ОК"
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
START: ms_button =  millis();
        button_state = false;
        tone(buzzerPin, 1045, 500);  //звуковой сигнал при старте профиля
#ifndef no_pc
        sprintf (buf, "SYNC\r\n");
        Serial.print(buf);
#endif
        bottomTemp = 0;
        integra = 0;
        integra2 = 0;
        p1 = 0;
        p2 = 0;
        d1 = 0;
        d2 = 0;
        updateScreen = true;
        Xgr = 2;
        XTgr = 2;
        reflowStatus = REFLOW_STATUS_ON;
        reflowState = REFLOW_STATE_TABLE_SIZE;
      }

      if ((cancelSwitchState) == LOW && (okSwitchState == LOW) && (button_long_state || button_state || button_state1)) {
        button_long_state = false;
        button_state = false;
        button_state1 = false;
      }

      break;

    //инициализация стандартного профиля
    case REFLOW_STATE_PROFILE_INIT:
      profile = {3, 150, 5, 6, 8, 10, 10, 10, 160, 200, 225, 0, 0, 0, 100, 100, 100, 5, 8, 20, 40, 10, 15, 40, 3, 0, 100};
      SaveProfile();
      tone(buzzerPin, 800, 200);  //звуковой сигнал при сбросе профиля
      myGLCD.clrScr();
      myGLCD.textRus("ПРОФИЛЬ ПЕРЕЗАПИСАН", 100, 200);
      delay(500);
      button_long_state = false;
      updateScreen = true;
      loadProfile();
      reflowState = REFLOW_STATE_IDLE;
      break;


    //устанавливаем количество шагов профиля
    case REFLOW_STATE_MENU_STEPS:
      if (updateScreen) {
        myGLCD.clrScr();
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_RED);
        myGLCD.textRus("ПР-ЛЬ", 0, 20);
        myGLCD.textRus("НИЖНИЙ НАГРЕВАТЕЛЬ", 90, 60);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("К-ВО ШАГОВ", 10, 110);
        myGLCD.textRus(profile_names[profileName - 1], 120, 20);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("РАЗМЕР НИЗА", 260, 110);
        myGLCD.textRus("ТЕМПЕРАТУРА НИЗ", 5, 180);
        myGLCD.textRus("МОЩНОСТЬ НИЗ", 280, 180);
        myGLCD.textRus("МИН", 320, 220);
        myGLCD.textRus("МАХ", 320, 260);
        myGLCD.textRus("`", 180, 223);
        myGLCD.textRus("%", 430, 220);
        myGLCD.textRus("%", 430, 260);
        myGLCD.textRus("СКОРОСТЬ НАГРЕВА НИЗОМ", 5, 300);
        myGLCD.textRus("`/с", 425, 300);
        myGLCD.printNumF(profile.BottomRampRateStep * 0.1, 0, 365, 300);
        myGLCD.printNumI(profile.tableSize, 450, 110);
        myGLCD.printNumI(profile.min_pwr_BOTTOM, 380, 220, 3, '0');
        myGLCD.printNumI(profile.max_pwr_BOTTOM, 380, 260, 3, '0');
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(SP2, 75, 220, 4, '0');
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(5, 103, 200, 131);
        myGLCD.setFont(BigFontRus);
        myGLCD.printNumI(currentProfile, 88, 20);
        myGLCD.printNumI(profile.profileSteps, 180, 110);
        updateScreen = false;
      }
      //    myGLCD.setColor(250, 180, 000);
      myGLCD.printNumI(profile.profileSteps, 180, 110);

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.profileSteps = profile.profileSteps + 1;
        if (profile.profileSteps >= 4)
        {
          profile.profileSteps = 1;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.profileSteps = profile.profileSteps - 1;
        if (profile.profileSteps <= 0)
        {
          profile.profileSteps = 3;
        }
      }

      if (okSwitchState == LOW && button_long_state) button_long_state = false; //чтобы после входа в меню не кнопка "ОК" не считалась нажатой

      if (okSwitchState == HIGH && !button_state && !button_long_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        SaveProfile();
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_TABLE_SIZE;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
    //устанавливаем размер стола
    case REFLOW_STATE_MENU_TABLE_SIZE:
      if (updateScreen) {
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(5, 103, 200, 131);
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("РАЗМЕР НИЗА", 260, 110);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("К-ВО ШАГОВ", 10, 110);
        myGLCD.printNumI(profile.profileSteps, 180, 110);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(250, 103, 475, 131);
        myGLCD.setFont(BigFontRus);
        myGLCD.printNumI(profile.tableSize, 450, 110);
        updateScreen = false;
      }
      myGLCD.printNumI(profile.tableSize, 450, 110);

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.tableSize = profile.tableSize + 1;
        if (profile.tableSize >= 4)
        {
          profile.tableSize = 1;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.tableSize = profile.tableSize - 1;
        if (profile.tableSize <= 0)
        {
          profile.tableSize = 3;
        }
      }
      if (okSwitchState == HIGH && !button_state && !button_long_state && ( millis() - ms_button) > 120)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        SaveProfile();
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_HEAT;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
    //устанавливаем температуру "Нижнего Нагревателя"
    case REFLOW_STATE_MENU_BOTTOM_HEAT:
      if (updateScreen) {
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(250, 103, 475, 131);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("РАЗМЕР НИЗА", 260, 110);
        myGLCD.printNumI(profile.tableSize, 450, 110);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("ТЕМПЕРАТУРА НИЗ", 5, 180);
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(2, 175, 258, 203);
        myGLCD.setFont(SevenSegNumFont);
        updateScreen = false;
      }

      myGLCD.printNumI(profile.Setpoint2, 75, 220, 4, '0');

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.Setpoint2 = profile.Setpoint2 + 1;
        if (profile.Setpoint2 >= 1200)
        {
          profile.Setpoint2 = 20;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.Setpoint2 = profile.Setpoint2 - 1;
        if (profile.Setpoint2 <= 20)
        {
          profile.Setpoint2 = 1200;
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        SaveProfile();
        updateScreen = true;
        SP2 = profile.Setpoint2;
        reflowState = REFLOW_STATE_MENU_BOTTOM_PWR_MIN;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
    //устанавливаем минимальную мощность "НИЖНЕГО НАГРЕВАТЕЛЯ"
    case REFLOW_STATE_MENU_BOTTOM_PWR_MIN:
      if (updateScreen) {
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(2, 175, 258, 203);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("МОЩНОСТЬ НИЗ", 280, 180);
        myGLCD.textRus("МИН", 320, 220);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("ТЕМПЕРАТУРА НИЗ", 5, 180);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(SP2, 75, 220, 4, '0');
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(275, 175, 475, 203);
        myGLCD.setFont(BigFontRus);
        updateScreen = false;
      }
      myGLCD.printNumI(profile.min_pwr_BOTTOM, 380, 220, 3, '0');

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.min_pwr_BOTTOM = profile.min_pwr_BOTTOM + 1;
        if (profile.min_pwr_BOTTOM >= 100)
        {
          profile.min_pwr_BOTTOM = 100;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();

        if (profile.min_pwr_BOTTOM < 1)
        {
          profile.min_pwr_BOTTOM = 0;
        }
        else profile.min_pwr_BOTTOM = profile.min_pwr_BOTTOM - 1;
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        SaveProfile();
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_PWR_MAX;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 50)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    //устанавливаем максимальную мощность "НИЖНЕГО НАГРЕВАТЕЛЯ"
    case REFLOW_STATE_MENU_BOTTOM_PWR_MAX:
      if (updateScreen) {
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(2, 175, 258, 203);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("МОЩНОСТЬ НИЗ", 280, 180);
        myGLCD.textRus("МАХ", 320, 260);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("МИН", 320, 220);
        myGLCD.printNumI(profile.min_pwr_BOTTOM, 380, 220, 3, '0');
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(275, 175, 475, 203);
        updateScreen = false;
      }
      myGLCD.printNumI(profile.max_pwr_BOTTOM, 380, 260, 3, '0');

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.max_pwr_BOTTOM = profile.max_pwr_BOTTOM + 1;
        if (profile.max_pwr_BOTTOM >= 100)
        {
          profile.max_pwr_BOTTOM = 100;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        if (profile.max_pwr_BOTTOM < 1)
        {
          profile.max_pwr_BOTTOM = 0;
        }
        else profile.max_pwr_BOTTOM = profile.max_pwr_BOTTOM - 1;
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        SaveProfile();
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_RAMP;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 50)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    //устанавливаем скорость нагрева "Нижним Нагревателем"
    case REFLOW_STATE_MENU_BOTTOM_RAMP:
      if (updateScreen) {
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(275, 175, 475, 203);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("МОЩНОСТЬ НИЗ", 280, 180);
        myGLCD.textRus("МАХ", 320, 260);
        myGLCD.printNumI(profile.max_pwr_BOTTOM, 380, 260, 3, '0');
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("СКОРОСТЬ НАГРЕВА НИЗОМ", 5, 300);
        // myGLCD.textRus("`/с",425, 300);
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(0, 295, 360, 319);
        updateScreen = false;
      }

      myGLCD.printNumF(profile.BottomRampRateStep * 0.1, 0, 365, 300);

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.BottomRampRateStep = profile.BottomRampRateStep + 1;
        if (profile.BottomRampRateStep >= 99)
        {
          profile.BottomRampRateStep = 1;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.BottomRampRateStep = profile.BottomRampRateStep - 1;
        if (profile.BottomRampRateStep <= 1)
        {
          profile.BottomRampRateStep = 99;
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        SaveProfile();
        updateScreen = true;
        myGLCD.clrScr();
        reflowState = REFLOW_STATE_MENU_STEP_RAMP;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 50)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }

      break;

    //устанавливаем скорость нагрева "Верхним Нагревателем"
    case REFLOW_STATE_MENU_STEP_RAMP:
      if (updateScreen) {
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_RED);
        myGLCD.textRus("ПР-ЛЬ", 0, 20);
        myGLCD.textRus("ВЕРХНИЙ НАГРЕВАТЕЛЬ", 120, 55);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("СКОРОСТЬ НАГРЕВА ВЕРХОМ", 5, 90);
        myGLCD.textRus(profile_names[profileName - 1], 120, 20);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("ШАГ:", 0, 55);
        myGLCD.textRus("ТЕМП-РА ВЕРХ", 5, 140);
        myGLCD.textRus("МОЩНОСТЬ ВЕРХ", 260, 140);
        myGLCD.textRus("МИН", 310, 180);
        myGLCD.textRus("МАХ", 310, 220);
        myGLCD.printNumI(profile.min_pwr_TOPStep[editStep], 370, 180, 3, '0');
        myGLCD.printNumI(profile.max_pwr_TOPStep[editStep], 370, 220, 3, '0');
        myGLCD.textRus("ВРЕМЯ ПЕРЕХОДА НА СЛЕД ШАГ", 5, 270);
        myGLCD.textRus("`/с", 428, 90);
        myGLCD.textRus("с", 462, 270);
        myGLCD.textRus("`", 160, 183);
        myGLCD.textRus("%", 420, 180);
        myGLCD.textRus("%", 420, 220);
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(0, 85, 378, 112);
        myGLCD.printNumI(currentProfile, 88, 20);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(profile.temperatureStep[editStep], 55, 180, 4, '0');
        myGLCD.setFont(BigFont);
        myGLCD.printNumI(profile.dwellTimerStep[editStep], 427, 270, 2, '0');
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(editStep + 1, 70, 55);
        updateScreen = false;
      }
      myGLCD.printNumF(profile.rampRateStep[editStep] * 0.1, 0, 380, 90);

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.rampRateStep[editStep] = profile.rampRateStep[editStep] + 1;
        if (profile.rampRateStep[editStep] >= 99)
        {
          profile.rampRateStep[editStep] = 1;  //максимальная скорость роста температуры от 0.1с. до 3с.
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.rampRateStep[editStep] = profile.rampRateStep[editStep] - 1;
        if (profile.rampRateStep[editStep] <= 1)
        {
          profile.rampRateStep[editStep] = 99;  //минимальная скорость роста температуры от 3с. до 0.1с.
        }
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        updateScreen = true;
        if (editStep + 1 == profile.profileSteps) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_STEP_TARGET;
        }
        else editStep++;
        SaveProfile();
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        updateScreen = true;
        editStep = 0;
        reflowState = REFLOW_STATE_IDLE;
      }

      break;
    //устанавливаем температуру "Верхнего Нагревателя"
    case REFLOW_STATE_MENU_STEP_TARGET:
      if (updateScreen) {
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(0, 85, 378, 112);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("СКОРОСТЬ НАГРЕВА ВЕРХОМ", 5, 90);
        myGLCD.textRus("`/с", 428, 90);
        myGLCD.printNumF(profile.rampRateStep[editStep] * 0.1, 0, 380, 90);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("ТЕМП-РА ВЕРХ", 5, 140);
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(2, 131, 200, 160);
        myGLCD.printNumI(editStep + 1, 70, 55);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(profile.temperatureStep[editStep], 55, 180, 4, '0');
        updateScreen = false;
      }

      myGLCD.printNumI(profile.temperatureStep[editStep], 55, 180, 4, '0');

      if (profile.temperatureStep[editStep] <= profile.Setpoint2)
      {
        profile.temperatureStep[editStep] = profile.Setpoint2;
      }
      if (editStep > 0)
      {
        if (profile.temperatureStep[editStep] <= profile.temperatureStep[editStep - 1])
          profile.temperatureStep[editStep] = profile.temperatureStep[editStep - 1];
      }

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.temperatureStep[editStep] = profile.temperatureStep[editStep] + 1;
        if (profile.temperatureStep[editStep] >= 1200)
        {
          profile.temperatureStep[editStep] = 0;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.temperatureStep[editStep] = profile.temperatureStep[editStep] - 1;
        if (profile.temperatureStep[editStep] <= 0)
        {
          profile.temperatureStep[editStep] = 1200;
        }

      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        updateScreen = true;
        if (editStep + 1 == profile.profileSteps) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_TOP_PWR_MIN;
        }
        else editStep++;
        SaveProfile();
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        updateScreen = true;
        editStep = 0;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
    //устанавливаем минимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
    case REFLOW_STATE_MENU_TOP_PWR_MIN:
      if (updateScreen)
      {
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(2, 131, 200, 160);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("ТЕМП-РА ВЕРХ", 5, 140);
        myGLCD.printNumF(profile.rampRateStep[editStep] * 0.1, 0, 380, 90);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(profile.temperatureStep[editStep], 55, 180, 4, '0');
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("МОЩНОСТЬ ВЕРХ", 260, 140);
        myGLCD.textRus("МИН", 310, 180);
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(editStep + 1, 70, 55);
        myGLCD.drawRoundRect(255, 131, 470, 160);
        myGLCD.setFont(BigFontRus);
        myGLCD.printNumI(profile.min_pwr_TOPStep[editStep], 370, 180, 3, '0');
        updateScreen = false;
      }
      myGLCD.printNumI(profile.min_pwr_TOPStep[editStep], 370, 180, 3, '0');

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.min_pwr_TOPStep[editStep] = profile.min_pwr_TOPStep[editStep] + 1;
        if (profile.min_pwr_TOPStep[editStep] >= 100)
        {
          profile.min_pwr_TOPStep[editStep] = 100;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        if (profile.min_pwr_TOPStep[editStep] < 1)
        {
          profile.min_pwr_TOPStep[editStep] = 0;
        }
        else profile.min_pwr_TOPStep[editStep] = profile.min_pwr_TOPStep[editStep] - 1;
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        updateScreen = true;
        if (editStep + 1 == profile.profileSteps) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_TOP_PWR_MAX;
        }
        else editStep++;
        SaveProfile();
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        updateScreen = true;
        editStep = 0;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    //устанавливаем максимальную мощность "ВЕРХНЕГО НАГРЕВАТЕЛЯ"
    case REFLOW_STATE_MENU_TOP_PWR_MAX:
      if (updateScreen)
      {
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(2, 131, 200, 160);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("МИН", 310, 180);
        myGLCD.printNumI(profile.min_pwr_TOPStep[editStep], 370, 180, 3, '0');
        myGLCD.printNumF(profile.rampRateStep[editStep] * 0.1, 0, 380, 90);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(profile.temperatureStep[editStep], 55, 180, 4, '0');
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("МАХ", 310, 220);
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(editStep + 1, 70, 55);
        myGLCD.drawRoundRect(255, 131, 470, 160);
        myGLCD.setFont(BigFontRus);
        myGLCD.printNumI(profile.max_pwr_TOPStep[editStep], 370, 220, 3, '0');
        updateScreen = false;
      }
      myGLCD.printNumI(profile.max_pwr_TOPStep[editStep], 370, 220, 3, '0');

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.max_pwr_TOPStep[editStep] = profile.max_pwr_TOPStep[editStep] + 1;
        if (profile.max_pwr_TOPStep[editStep] >= 100)
        {
          profile.max_pwr_TOPStep[editStep] = 100;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        if (profile.max_pwr_TOPStep[editStep] <= 1)
        {
          profile.max_pwr_TOPStep[editStep] = 0;
        }
        else profile.max_pwr_TOPStep[editStep] = profile.max_pwr_TOPStep[editStep] - 1;
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        updateScreen = true;
        if (editStep + 1 == profile.profileSteps) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_STEP_DWELL;
        }
        else editStep++;
        SaveProfile();
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        updateScreen = true;
        editStep = 0;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    //устанавливаем время перехода на следующий шаг
    case REFLOW_STATE_MENU_STEP_DWELL:
      if (updateScreen) {
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(255, 131, 470, 160);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("МОЩНОСТЬ ВЕРХ", 260, 140);
        myGLCD.textRus("МАХ", 310, 220);
        myGLCD.printNumI(profile.max_pwr_TOPStep[editStep], 370, 220, 3, '0');
        myGLCD.printNumF(profile.rampRateStep[editStep] * 0.1, 0, 380, 90);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(profile.temperatureStep[editStep], 55, 180, 4, '0');
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("ВРЕМЯ ПЕРЕХОДА НА СЛЕД ШАГ", 5, 270);
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(2, 263, 420, 290);
        myGLCD.printNumI(editStep + 1 , 70, 55);
        myGLCD.printNumI(profile.dwellTimerStep[editStep], 427, 270, 2, '0');
        updateScreen = false;
      }
      myGLCD.printNumI(profile.dwellTimerStep[editStep], 427, 270, 2, '0');

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        profile.dwellTimerStep[editStep] = profile.dwellTimerStep[editStep] + 1;
        if (profile.dwellTimerStep[editStep] >= 90)
        {
          profile.dwellTimerStep[editStep] = 90;
        }
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        if (profile.dwellTimerStep[editStep] < 1)
        {
          profile.dwellTimerStep[editStep] = 0;
        }
        else profile.dwellTimerStep[editStep] = profile.dwellTimerStep[editStep] - 1;
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        updateScreen = true;
        if (editStep + 1 == profile.profileSteps) {
          editStep = 0;
          reflowState = REFLOW_STATE_MENU_BOTTOM_P;
        }
        else editStep++;
        SaveProfile();
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        updateScreen = true;
        editStep = 0;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
    //настройка "ПИД" нижнего нагревателя
    case REFLOW_STATE_MENU_BOTTOM_P:
      if (updateScreen) {
        myGLCD.clrScr();
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_RED);
        myGLCD.textRus("НАСТРОЙКА ПИД", 130, 5);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("НИЖНИЙ НАГРЕВАТЕЛЬ", 90, 35);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("ВЕРХНИЙ НАГРЕВАТЕЛЬ", 85, 170);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(profile.ki2, 200, 100, 3, '0');
        myGLCD.printNumI(profile.kd2, 375, 100, 3, '0');
        myGLCD.setFont(BigFont);
        myGLCD.print("I=", 230, 70);
        myGLCD.print("D=", 405, 70);
        myGLCD.print("P=", 55, 205);
        myGLCD.print("I=", 230, 205);
        myGLCD.print("D=", 405, 205);
        myGLCD.setColor(250, 180, 000);
        myGLCD.print(String("P="), 55, 70);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.drawRoundRect(80, 28, 390, 55);
        myGLCD.printNumI(profile.kp2, 25, 100, 3, '0');
        updateScreen = false;
      }
      myGLCD.printNumI(profile.kp2, 25, 100, 3, '0');

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        if (profile.kp2 >= 254)
        {
          profile.kp2 = 255;
        }
        else profile.kp2 = profile.kp2 + 1;
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        if (profile.kp2 < 1)
        {
          profile.kp2 = 0;
        }
        else profile.kp2 = profile.kp2 - 1;
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        SaveProfile();
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_I;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
    case REFLOW_STATE_MENU_BOTTOM_I:
      if (updateScreen) {
        myGLCD.setFont(BigFont);
        myGLCD.setColor(250, 180, 000);
        myGLCD.print("I=", 230, 70);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.print("P=", 55, 70);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(profile.kp2, 25, 100, 3, '0');
        myGLCD.setColor(250, 180, 000);
        updateScreen = false;
      }
      myGLCD.printNumI(profile.ki2, 200, 100, 3, '0');

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        if (profile.ki2 >= 254)
        {
          profile.ki2 = 255;
        }
        else profile.ki2 = profile.ki2 + 1;
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        if (profile.ki2 < 1)
        {
          profile.ki2 = 0;
        }
        else profile.ki2 = profile.ki2 - 1;
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        SaveProfile();
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_BOTTOM_D;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
    case REFLOW_STATE_MENU_BOTTOM_D:
      if (updateScreen) {
        myGLCD.setFont(BigFont);
        myGLCD.setColor(250, 180, 000);
        myGLCD.print("D=", 405, 70);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.print("I=", 230, 70);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(profile.ki2, 200, 100, 3, '0');
        myGLCD.setColor(250, 180, 000);
        updateScreen = false;
      }
      myGLCD.printNumI(profile.kd2, 375, 100, 3, '0');

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        if (profile.kd2 >= 254)
        {
          profile.kd2 = 255;
        }
        else profile.kd2 = profile.kd2 + 1;
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        if (profile.kd2 < 1)
        {
          profile.kd2 = 0;
        }
        else profile.kd2 = profile.kd2 - 1;
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        SaveProfile();
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_TOP_P;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
    //настройка "ПИД" верхнего нагревателя
    case REFLOW_STATE_MENU_TOP_P:
      if (updateScreen) {
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(250, 180, 000);
        myGLCD.print("P=", 55, 205);
        myGLCD.setColor(VGA_LIME);
        myGLCD.textRus("ВЕРХНИЙ НАГРЕВАТЕЛЬ", 85, 170);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.textRus("НИЖНИЙ НАГРЕВАТЕЛЬ", 90, 35);
        myGLCD.setFont(BigFont);
        myGLCD.print("D=", 405, 70);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(profile.kd2, 375, 100, 3, '0');
        myGLCD.printNumI(profile.ki1, 200, 240, 3, '0');
        myGLCD.printNumI(profile.kd1, 375, 240, 3, '0');
        myGLCD.setColor(VGA_BLACK);
        myGLCD.drawRoundRect(80, 28, 390, 55);
        myGLCD.setColor(250, 180, 000);
        myGLCD.drawRoundRect(80, 163, 394, 190);
        myGLCD.printNumI(profile.kp1, 25, 240, 3, '0');
        updateScreen = false;
      }
      myGLCD.printNumI(profile.kp1, 25, 240, 3, '0');

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        if (profile.kp1 >= 254)
        {
          profile.kp1 = 255;
        }
        else profile.kp1 = profile.kp1 + 1;
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        if (profile.kp1 < 1)
        {
          profile.kp1 = 0;
        }
        else profile.kp1 = profile.kp1 - 1;
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        SaveProfile();
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_TOP_I;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
    case REFLOW_STATE_MENU_TOP_I:
      if (updateScreen) {
        myGLCD.setFont(BigFont);
        myGLCD.setColor(250, 180, 000);
        myGLCD.print("I=", 230, 205);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.print("P=", 55, 205);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(profile.kp1, 25, 240, 3, '0');
        myGLCD.setColor(250, 180, 000);
        updateScreen = false;
      }

      myGLCD.printNumI(profile.ki1, 200, 240, 3, '0');

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        if (profile.ki1 >= 254)
        {
          profile.ki1 = 255;
        }
        else profile.ki1 = profile.ki1 + 1;
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        if (profile.ki1 < 1)
        {
          profile.ki1 = 0;
        }
        else profile.ki1 = profile.ki1 - 1;
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        SaveProfile();
        updateScreen = true;
        reflowState = REFLOW_STATE_MENU_TOP_D;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
    case REFLOW_STATE_MENU_TOP_D:
      if (updateScreen) {
        myGLCD.setFont(BigFont);
        myGLCD.setColor(250, 180, 000);
        myGLCD.print("D=", 405, 205);
        myGLCD.setColor(VGA_SILVER);
        myGLCD.print("I=", 230, 205);
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.printNumI(profile.ki1, 200, 240, 3, '0');
        myGLCD.setColor(250, 180, 000);
        updateScreen = false;
      }
      myGLCD.printNumI(profile.kd1, 375, 240, 3, '0');

      if (upSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        if (profile.kd1 >= 254)
        {
          profile.kd1 = 255;
        }
        else profile.kd1 = profile.kd1 + 1;
      }
      if (downSwitchState == HIGH && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        if (profile.kd1 < 1)
        {
          profile.kd1 = 0;
        }
        else profile.kd1 = profile.kd1 - 1;
      }
      if (okSwitchState == HIGH && !button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = true;
      }
      if (okSwitchState == LOW && button_state && ( millis() - ms_button) > 100)
      {
        ms_button =  millis();
        button_state = false;
        SaveProfile();
        myGLCD.clrScr();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        //myGLCD.clrScr();
        updateScreen = true;
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    //фиксируем размер стола
    case REFLOW_STATE_TABLE_SIZE:
      if (profile.tableSize == 1) {
        digitalWrite(P1_PIN, HIGH);
      }
      if (profile.tableSize == 2) {
        digitalWrite(P1_PIN, HIGH);
        digitalWrite(P2_PIN, HIGH);
      }
      if (profile.tableSize == 3) {
        digitalWrite(P1_PIN, HIGH);
        digitalWrite(P2_PIN, HIGH);
        digitalWrite(P3_PIN, HIGH);
      }
      previousMillis = millis();
      reflowState = REFLOW_STATE_PRE_HEATER;

      break;

    case REFLOW_STATE_PRE_HEATER:
      myGLCD.setFont(BigFontRus);
      myGLCD.setColor(VGA_RED);
      myGLCD.textRus("ПРЕДНАГРЕВ", 2, 300);
      Output2 = 3;
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        Output2 = 0;
        reflowState = REFLOW_STATE_COMPLETE;
      }
      if (millis() - previousMillis > 5000)
      {
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_BLACK);
        myGLCD.textRus("ПРЕДНАГРЕВ", 2, 300);
        Output2 = 0;
        //bottomTemp=tc2;
        reflowState = REFLOW_STATE_STEP_RAMP;
      }

      break;

    //рампа для нижнего нагревателя
    case REFLOW_STATE_BOTTOM_STEP_RAMP:
      if ((currentMillis - previousMillis) > 1000 / (profile.BottomRampRateStep * 0.1)) //скорость роста температуры от 0.1с. до 3с.
      {
        previousMillis = currentMillis;

        if (setpointRamp < profile.Setpoint2)
        {
          counter = counter + 1;
          setpointRamp = counter + startTemp;
          bottomTemp = setpointRamp;
        }
        else bottomTemp = profile.Setpoint2;
        myGLCD.setFont(BigFont);
        myGLCD.setColor(250, 180, 000);
        if (bottomTemp < 9999) {
          myGLCD.printNumI(bottomTemp, 100, 227, 4, '0');
        }
      }
      if (Input2 >= profile.Setpoint2 )
      {
        counter = 0;
        updateScreen = true;
        reflowState = REFLOW_STATE_STEP_RAMP;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        digitalWrite(P1_PIN, LOW);
        digitalWrite(P2_PIN, LOW);
        digitalWrite(P3_PIN, LOW);
        digitalWrite(P4_PIN, LOW);
        updateScreen = true;
        reflowStatus = REFLOW_STATUS_OFF;
        reflowState = REFLOW_STATE_COMPLETE;
      }
      break;

    //"Старт и процесс пайки", рост температуры с заданной скоростью
    case REFLOW_STATE_STEP_RAMP:
      if (updateScreen) {
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(VGA_GREEN);
        myGLCD.textRus("ШАГ", 5, 300);
        myGLCD.textRus("П-", 90, 300);
        myGLCD.textRus(profile_names[profileName - 1], 180, 300);
        myGLCD.setFont(BigFont);
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(profile.temperatureStep[editStep], 100, 115, 4, '0');
        //myGLCD.printNumI(SP2,100, 227,4,'0');
        myGLCD.setFont(BigFontRus);
        myGLCD.setColor(0, 255, 50);
        updateScreen = false;
      }
      if (tc1 >= SP2 - 10 && !TopStart) {
        TopStart = true;  //если температура низа вышла на уставку включаем верхний нагреватель
        bottomTemp = profile.Setpoint2;
      }
      if (TopStart == true) {  // включен верхний нагреватель
        if (flag == 0)           //фиксируем стартовую температуру
        {
          startTemp = tc1;
          flag = 1;
        }
        //устанавливаем нужный шаг, до которой нагрета плата
        if (startTemp > profile.temperatureStep[currentStep - 1]) {
          for (x = 1; startTemp > profile.temperatureStep[currentStep - 1]; currentStep++) {
            x++;
          }
        }
        if (currentStep > x && flag == 1) {
          flag = 0;
          startTemp = profile.temperatureStep[currentStep - 2];
          flag = 1;
        }
        myGLCD.setFont(BigFont);
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(currentStep, 60, 300);
        //myGLCD.setFont(SevenSegNumFont);
        // myGLCD.printNumI(temperatureStep[currentStep - 1],90, 120);

        //счётчик скорости роста температуры
        if ((currentMillis - previousMillis) > 1000 / (profile.rampRateStep[currentStep - 1] * 0.1)) //скорость роста температуры от 0.1с. до 3с.
        {
          previousMillis = currentMillis;
          counter = counter + 1;
          setpointRamp = counter + startTemp;
          //myGLCD.setFont(BigFont);
          myGLCD.setColor(250, 180, 000);
          //myGLCD.printNumI(setpointRamp,420, 300,4,'0');
          myGLCD.printNumI(setpointRamp, 100, 115, 4, '0');
          Setpoint1 = setpointRamp;

        }
      } //закрывам скобку

      else  if (!is_bottomRamp) {
        is_bottomRamp = true;
        startTemp = tc2;
        reflowState = REFLOW_STATE_BOTTOM_STEP_RAMP;
      }

      if (setpointRamp >= profile.temperatureStep[currentStep - 1]) //если достигли нужной температуры
      {
        // myGLCD.setColor(VGA_RED);
        // myGLCD.printNumI(temperatureStep[currentStep - 1],420, 300);
        reflowState = REFLOW_STATE_STEP;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 50)
      {
        ms_button =  millis();
        digitalWrite(P1_PIN, LOW);
        digitalWrite(P2_PIN, LOW);
        digitalWrite(P3_PIN, LOW);
        digitalWrite(P4_PIN, LOW);

        reflowStatus = REFLOW_STATUS_OFF;
        reflowState = REFLOW_STATE_COMPLETE;
        updateScreen = true;
      }
      break;

    case REFLOW_STATE_STEP:
      Setpoint1 = profile.temperatureStep[currentStep - 1];

      if (Input1 >= profile.temperatureStep[currentStep - 1] )
      {
        counter = 0;
        reflowState = REFLOW_STATE_STEP_DWELL;
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        digitalWrite(P1_PIN, LOW);
        digitalWrite(P2_PIN, LOW);
        digitalWrite(P3_PIN, LOW);
        digitalWrite(P4_PIN, LOW);
        updateScreen = true;

        reflowStatus = REFLOW_STATUS_OFF;
        reflowState = REFLOW_STATE_COMPLETE;
      }
      break;

    //считаем время перехода на следующий шаг
    case REFLOW_STATE_STEP_DWELL:
      if (currentMillis - previousMillis > 1000)
      {
        previousMillis = currentMillis;
        counter = counter + 1;
        myGLCD.setFont(BigFont);
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(counter, 120, 300, 2, '0');
      }
      if (counter >= profile.dwellTimerStep[currentStep - 1]) //если счётчик равен установленному времени
      {
        myGLCD.setFont(BigFont);
        myGLCD.setColor(VGA_RED);
        //myGLCD.printNumI(counter, 120, 300, 2, '0');
        tone(buzzerPin, 1045, 500);  //звуковой сигнал
        counter = 0;
        setpointRamp = 0;

        if (profile.profileSteps == currentStep) //если достигли последнего шага
        {
          currentStep = 1;
          x = 1;      //устанавливаем переменную в исходное состояние
          flag = 0;   //после завершения профиля сбрасываем флаг
          reflowState = REFLOW_STATE_COMPLETE;
        }
        else //если шаг не последний
        {
          currentStep++; //переходим на следующий шаг

          reflowState = REFLOW_STATE_STEP_RAMP;
        }
      }
      if (cancelSwitchState == HIGH && ( millis() - ms_button) > 60)
      {
        ms_button =  millis();
        digitalWrite(P1_PIN, LOW);
        digitalWrite(P2_PIN, LOW);
        digitalWrite(P3_PIN, LOW);
        digitalWrite(P4_PIN, LOW);
        updateScreen = true;

        reflowStatus = REFLOW_STATUS_OFF;
        reflowState = REFLOW_STATE_COMPLETE;
      }
      break;
    //завершение пайки
    case REFLOW_STATE_COMPLETE:
      digitalWrite(P1_PIN, LOW);
      digitalWrite(P2_PIN, LOW);
      digitalWrite(P3_PIN, LOW);
      digitalWrite(P4_PIN, LOW);
#ifdef params_from_pc
      if (b1 == 'E') {
        Serial.println("TXStop_PC");
        b1 = ' ';
      } else Serial.println("TXStop");
#endif
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
      updateScreen = true;
      TopStart = false;
      Xgr = 2;
      XTgr = 2;
      is_bottomRamp = false;
      break;
  }
  //включение нагревателей
  if (reflowStatus == REFLOW_STATUS_ON)
  {
#ifdef params_from_pc
    b2 = Serial.available();
    if (b2 > 0)
    {
      b1 = Serial.read();
      if (b1 == 'E') reflowState = REFLOW_STATE_COMPLETE;
    }
#endif

    if (millis() > nextRead1)
    { nextRead1 = millis() + SENSOR_SAMPLING_TIME;
      Input1 = Input1 * 0.6 + 0.4 * (max6675_read_temp (thermoCLK, thermoCS, thermoDO));
      Input2 = Input2 * 0.6 + 0.4 * (max6675_read_temp (thermoCLK2, thermoCS2, thermoDO2));
      if (reflowState != REFLOW_STATE_PRE_HEATER) Output2 = Pid2(Input2, bottomTemp, profile.kp2, profile.ki2, profile.kd2);
      if (TopStart) Output1 = Pid1(Input1, Setpoint1, profile.kp1, profile.ki1, profile.kd1);
      tc1 = Input1;
      tc2 = Input2;
      myGLCD.setColor(VGA_YELLOW);
      myGLCD.setFont(BigFontRus);
      Input1_spd = (Input1 - Input_f1) * 1000 / (millis() - prev_millis);
      Input2_spd = (Input2 - Input_f2) * 1000 / (millis() - prev_millis);
      if (abs(Input1_spd) < 10) {
        myGLCD.printNumF(Input1_spd, 2, 84, 133, ',', 5);
      }

      if (abs(Input2_spd) < 10) {
        myGLCD.printNumF(Input2_spd, 2, 84, 245, ',', 5);
      }

      prev_millis = millis();
      Input_f1 = Input1;
      Input_f2 = Input2;
      myGLCD.setColor(255, 80, 000);
      myGLCD.setFont(BigFontRus);

      Input2_fraction = (Input2 - int(Input2)) * 100;
      Input1_fraction = (Input1 - int(Input1)) * 100;

      if ((Input2_fraction < 99) and (Input2_fraction >= 0)) {
        myGLCD.printNumI(Input2_fraction, 365, 232, 2, '0');
      }

      if ((Input1_fraction < 99) and (Input1_fraction >= 0)) {
        myGLCD.printNumI(Input1_fraction, 365, 120, 2, '0');
      }

      //--------------------------------
      myGLCD.setColor(VGA_YELLOW);
      myGLCD.textRus("НАГРЕВ", 365, 266);
      if (TopStart) myGLCD.textRus("НАГРЕВ", 365, 153);
      if ( reflowState != REFLOW_STATE_BOTTOM_STEP_RAMP)
      { Icontrol();
        myGLCD.setColor(250, 180, 000);
        myGLCD.printNumI(bottomTemp, 100, 227, 4, '0');
      }
#ifndef no_pc
      sprintf (buf, "OK%03d%03d%03d%03d%03d\r\n", int(Output1), int(Output2), tc1, tc2, int(profileName)); // график ПК
#endif
      // график---------------------------------------
      if (CTgr == 4) //скорость графика в условии
      {
        myGLCD.setColor(VGA_YELLOW);
        myGLCD.printNumI(Output2, 365, 198, 3, ' ');
        myGLCD.printNumI(Output1, 365, 84, 3, ' ');
        myGLCD.setColor(0, 0, 0);
        myGLCD.drawLine(XTgr, 2, XTgr, 71);
        myGLCD.setColor(30, 30, 30);
        myGLCD.drawPixel(XTgr, 12);
        myGLCD.drawPixel(XTgr, 22);
        myGLCD.drawPixel(XTgr, 32);
        myGLCD.drawPixel(XTgr, 42);
        myGLCD.drawPixel(XTgr, 52);
        myGLCD.drawPixel(XTgr, 62);
        myGLCD.setColor(40, 150, 0);
        myGLCD.drawPixel(XTgr, 71 - round((tc2 - 15) / 3.4));
        myGLCD.setColor(150, 0, 40);
        myGLCD.drawPixel(XTgr, 71 - round((tc1 - 15) / 3.4));
        myGLCD.setColor(40, 50, 0);
        myGLCD.drawLine(XTgr + 1, 2, XTgr + 1, 71);
        XTgr++;
        if (XTgr >= 477)
        {
          XTgr = 2;
          myGLCD.setColor(0, 0, 0);
          myGLCD.drawLine(477, 2, 477, 71);
        }
        CTgr = 0;
      } CTgr++;
      //============================================================

      if (Input1 <= -0) {
        myGLCD.setFont(BigFont);
        myGLCD.setColor(VGA_RED);
        myGLCD.print("ERROR", 250, 140);
      } else {
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.setColor(VGA_RED);
        myGLCD.printNumI(tc1, 235, 120, 4, '0');
      }
      if (Input2 <= -0) {
        myGLCD.setFont(BigFont);
        myGLCD.setColor(VGA_RED);
        myGLCD.print("ERROR", 250, 250);
      } else {
        myGLCD.setFont(SevenSegNumFont);
        myGLCD.setColor(VGA_RED);
        myGLCD.printNumI(tc2, 235, 232, 4, '0');
      }
    }

    if (tc1 == SP2 - 5) {
      digitalWrite(P4_PIN, HIGH);
      tone(buzzerPin, 1045, 500);  //звуковой сигнал
    }
    if (tc1 >= 1100)
    {
      digitalWrite(P1_PIN, LOW);
      digitalWrite(P2_PIN, LOW);
      digitalWrite(P3_PIN, LOW);
      digitalWrite(P4_PIN, LOW);
      tone(buzzerPin, 1045, 400);  //звуковой сигнал

      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_IDLE;
      updateScreen = true;
    }
    if (!TopStart) Output1 = 0;
  }
  else
  {
    digitalWrite(RelayPin1, LOW);
    digitalWrite(RelayPin2, LOW);
  }
}

void Dimming()
{
  OutPWR_TOP();
  OutPWR_BOTTOM();
#ifndef no_pc
  if (Secs >= 99)
  {
    Serial.println(buf);
    Secs = 0;
  } else Secs++;
#endif
}

void Icontrol()
{
  if (upSwitchState == HIGH && ( millis() - ms_button) > 200)
  {
    ms_button =  millis();
    bottomTemp++;
    if (bottomTemp >= 900) bottomTemp = 900;
  }

  if (downSwitchState == HIGH && ( millis() - ms_button) > 200)
  {
    ms_button =  millis();
    if (bottomTemp > 1) bottomTemp--; else bottomTemp = 0;
  }
}

void OutPWR_TOP() {
  reg1 = Output1 + er1; //pwr- задание выходной мощности в %,в текущем шаге профиля, er- ошибка округления
  if (reg1 < 50) {
    out1 = LOW;
    er1 = reg1; // reg- переменная для расчетов
  }
  else {
    out1 = HIGH;
    er1 = reg1 - 100;
  }
  digitalWrite(RelayPin1, out1); //пин через который осуществляется дискретное управление
}

void OutPWR_BOTTOM() {
  reg2 = Output2 + er2; //pwr- задание выходной мощности в %, er- ошибка округления
  if (reg2 < 50) {
    out2 = LOW;
    er2 = reg2; // reg- переменная для расчетов
  }
  else {
    out2 = HIGH;
    er2 = reg2 - 100;
  }
  digitalWrite(RelayPin2, out2); //пин через который осуществляется дискретное управление
}

unsigned int Pid1(double temp, unsigned int ust, byte kP, byte kI, byte kd)
{
  unsigned int out = 0;
  static float ed = 0;
  e1 = (ust - temp); //ошибка регулирования
  p1 =  (kP * e1); //П составляющая
  integra = (integra < i_min) ? i_min : (integra > i_max) ? i_max : integra + (kI * e1) / 1000.0; //И составляющая
  d1 = kd * (temp - ed); //Д составляющая
  ed = temp;
  out = (p1 + integra - d1 < profile.min_pwr_TOPStep[currentStep - 1]) ? profile.min_pwr_TOPStep[currentStep - 1] : (p1 + integra - d1 > profile.max_pwr_TOPStep[currentStep - 1]) ? profile.max_pwr_TOPStep[currentStep - 1] : p1 + integra - d1;
  return out;
}

unsigned int Pid2(double temp, unsigned int ust, byte kP, byte kI, byte kd)
{
  unsigned int out = 0;
  static float ed = 0;
  e2 = (ust - temp); //ошибка регулирования
  p2 =  (kP * e2); //П составляющая
  integra2 = (integra2 < i_min) ? i_min : (integra2 > i_max) ? i_max : integra2 + (kI * e2) / 1000.0; //И составляющая
  d2 = kd * (temp - ed); //Д составляющая
  ed = temp;
  out = (p2 + integra2 - d2 < profile.min_pwr_BOTTOM) ? profile.min_pwr_BOTTOM : (p2 + integra2 - d2 > profile.max_pwr_BOTTOM) ? profile.max_pwr_BOTTOM : p2 + integra2 - d2;
  return out;
}

double max6675_read_temp (int ck, int cs, int so)
{ char i;
  int tmp = 0;
  digitalWrite(cs, LOW);//cs = 0;                            // Stop a conversion in progress
  asm volatile (" nop"  "\n\t");
  for (i = 15; i >= 0; i--)
  { digitalWrite(ck, HIGH);
    asm volatile (" nop"  "\n\t");
    if ( digitalRead(so))
      tmp |= (1 << i);
    digitalWrite(ck, LOW);
    asm volatile (" nop"  "\n\t");
  }
  digitalWrite(cs, HIGH);
  if (tmp & 0x4) {
    return -100;
  } else
    return ((tmp >> 3)) * 0.25;
}
