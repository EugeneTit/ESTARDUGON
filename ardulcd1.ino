//ook_est__Decoder__11_tmp_narmon_Oregon_est_10_75_chnls_19lcd_newIP (новый ip адрес)
//   РАБОЧАЯ !!!!!!!     11.10.75.9 версия скетча   ook_est__Decoder__11_tmp_narmon_Oregon_est_10_75_channels_12    вход данных с приёмника   D3  pin   !!!  EST !!!

//ДОБАВЛЕНО АТМОСФЕРНОЕ ДАВЛЕНИЕ  добавлен 2 канал первое соединение через 75 сек  
// добавляется третий канал!    добавлено 15.02.2015 // 08.03.2015    ПРОВЕРЕНО !!!
// 08.03.2015 улучшен вывод в прот
// 08.03.2015  mac - адрес изменён на  0xDA, 0xFF, 0x00, 0x11, 0x22, 0x33          (DAFF00112233)   Опалихинская
// 08.03.2015 табличный вывод в порт при помощи отдельной функции
//08.03.2015 Датчики нормально названы
//09.03.2015 Добавил температуру c BMP
//18.03.2013 добавляю Watchdog
//20.03.2013 добавил задержку при ошибке соединения для срабатывания Watchdog и перезагрузки устройства
// версия дебаг с разделённым буфером
//21.07.2015  прикручиваю 2004 дисплей на квадратной шине...
//14.08.2016 изменился ip адрес сервера, новый ip адрес народного мониторинга 94.142.140.101


// Oregon V2 decoder modfied - Olivier Lebrun
// Oregon V2 decoder added - Dominique Pierre
// New code to decode OOK signals from weather sensors, etc.
// 2010-04-11 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: ookDecoder.pde 5331 2010-04-17 10:45:17Z jcw $
  ///**************************************************************************************************************narmon_Oregon_est_8

 //
  


//     
// PURPOSE: DHT library test sketch for Arduino. Works with web service narodmon.ru
//

///#include <dht.h>
#include <avr/wdt.h>
#include <Dhcp.h>
#include <Dns.h>
#include <Ethernet.h>
#include <EthernetClient.h>
#include <EthernetServer.h>
#include <EthernetUdp.h>
#include <util.h>
#include <SPI.h>
#include <Wire.h> // библиотеке I2C 
#include <BMP085.h> // иблиотека барометра
#include <LiquidCrystal_I2C.h>  // библия ЖКИ на шине I2C  добавлено 21.07.2015

BMP085 dps = BMP085();  //тип барометра

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display

///dht DHT;

///#define DHT22_PIN 7
const unsigned long postingInterval = 400000;  // интервал между отправками данных в миллисекундах (5 минут)
unsigned long lastConnectionTime = 0;           // время последней передачи данных
boolean lastConnected = false;                  // состояние подключения
int HighByte, LowByte, TReading, SignBit,chanel, Tc_100;
char replyBuffer[131];
char CharTemp1[6],CharHum1[6], CharTemp2[6],CharHum2[6], CharTemp3[6], CharHum3[6], CharPresmm[6], CharBMPTemp[6]; //  переменные  текущёй температуры и влажность для ТРЁХ каналов + давление + температура BMP  (строковые)
///int CurTemp1, CurHum1;   // переменные текущёй температуры и влажности
float CurTemp1 = 0, CurHum1 = 0, CurTemp2 = 0, CurHum2 = 0, CurTemp3 = 0, CurHum3 = 0, Presmm = 0, BMPTemp= 0;   // переменные текущёй температуры и влажности для ТРЁХ каналов + давление в мм + температура датчика BMP  (числа с плавающей точкой)
long Pres = 0, Tempera= 0; //переменная давления в Паскалях целые длинные и Температура датчика барометра
unsigned long WDTime = millis(), PrevWDTime=0;  //время Watchdog 
//byte channel;

// ========================Задаем данные сети======================
byte mac[] = {
  0xDA, 0xFF, 0x00, 0x11, 0x22, 0x33 }; //mac - адрес ethernet shielda
EthernetClient client; //
int ledPin = 9; // указываем что светодиод будет управляться через 9 Pin
String readString = String(30); //string for fetching data from address
boolean LEDON = false; //изначальный статус светодиода - выключен
//IPAddress server(94,19,113,221); // IP сервера narodmon.ru 
IPAddress server(94,142,140,101); // новый IP сервера narodmon.ru 94.142.140.101
// ===============================================================
 ///************************************************************************************************************** 
class DecodeOOK {
protected:
    byte total_bits, bits, flip, state, pos, data[25];
 
    virtual char decode (word width) =0;
 
public:
 
    enum { UNKNOWN, T0, T1, T2, T3, OK, DONE };
 
    DecodeOOK () { resetDecoder(); }
 
    bool nextPulse (word width) {
        if (state != DONE)
 
            switch (decode(width)) {
                case -1: resetDecoder(); break;
                case 1:  done(); break;
            }
        return isDone();
    }
 
    bool isDone () const { return state == DONE; }
 
    const byte* getData (byte& count) const {
        count = pos;
        return data; 
    }
 
    void resetDecoder () {
        total_bits = bits = pos = flip = 0;
        state = UNKNOWN;
    }
 
    // add one bit to the packet data buffer
 
    virtual void gotBit (char value) {
        total_bits++;
        byte *ptr = data + pos;
        *ptr = (*ptr >> 1) | (value << 7);
 
        if (++bits >= 8) {
            bits = 0;
            if (++pos >= sizeof data) {
                resetDecoder();
                return;
            }
        }
        state = OK;
    }
 
    // store a bit using Manchester encoding
    void manchester (char value) {
        flip ^= value; // manchester code, long pulse flips the bit
        gotBit(flip);
    }
 
    // move bits to the front so that all the bits are aligned to the end
    void alignTail (byte max =0) {
        // align bits
        if (bits != 0) {
            data[pos] >>= 8 - bits;
            for (byte i = 0; i < pos; ++i)
                data[i] = (data[i] >> bits) | (data[i+1] << (8 - bits));
            bits = 0;
        }
        // optionally shift bytes down if there are too many of 'em
        if (max > 0 && pos > max) {
            byte n = pos - max;
            pos = max;
            for (byte i = 0; i < pos; ++i)
                data[i] = data[i+n];
        }
    }
 
    void reverseBits () {
        for (byte i = 0; i < pos; ++i) {
            byte b = data[i];
            for (byte j = 0; j < 8; ++j) {
                data[i] = (data[i] << 1) | (b & 1);
                b >>= 1;
            }
        }
    }
 
    void reverseNibbles () {
        for (byte i = 0; i < pos; ++i)
            data[i] = (data[i] << 4) | (data[i] >> 4);
    }
 
    void done () {
        while (bits)
            gotBit(0); // padding
        state = DONE;
    }
};
 
class OregonDecoderV2 : public DecodeOOK {
  public:   
 
    OregonDecoderV2() {}
 
    // add one bit to the packet data buffer
    virtual void gotBit (char value) {
        if(!(total_bits & 0x01))
        {
            data[pos] = (data[pos] >> 1) | (value ? 0x80 : 00);
        }
        total_bits++;
        pos = total_bits >> 4;
        if (pos >= sizeof data) {
            Serial.println("sizeof data");
            resetDecoder();
            return;
        }
        state = OK;
    }
 
    virtual char decode (word width) {
       if (200 <= width && width < 1200) {
            //Serial.println(width);
            byte w = width >= 700;
 
            switch (state) {
                case UNKNOWN:
                    if (w != 0) {
                        // Long pulse
                        ++flip;
                    } else if (w == 0 && 24 <= flip) {
                        // Short pulse, start bit
                        flip = 0;
                        state = T0;
                    } else {
                        // Reset decoder
                        return -1;
                    }
                    break;
                case OK:
                    if (w == 0) {
                        // Short pulse
                        state = T0;
                    } else {
                        // Long pulse
                        manchester(1);
                    }
                    break;
                case T0:
                    if (w == 0) {
                      // Second short pulse
                        manchester(0);
                    } else {
                        // Reset decoder
                        return -1;
                    }
                    break;
              }
        } else if (width >= 2500  && pos >= 8) {
            return 1;
        } else {
            return -1;
        }
        return 0;
    }
};
 
OregonDecoderV2 orscV2;
 
volatile word pulse;
 
void ext_int_1(void)
{
    static word last;
    // determine the pulse length in microseconds, for either polarity
    pulse = micros() - last;
    last += pulse;
}
 
float temperature(const byte* data)
{
    int sign = (data[6]&0x8) ? -1 : 1;
    float temp = ((data[5]&0xF0) >> 4)*10 + (data[5]&0xF) + (float)(((data[4]&0xF0) >> 4) / 10.0);
    return sign * temp;
}
 
byte humidity(const byte* data)
{
    return (data[7]&0xF) * 10 + ((data[6]&0xF0) >> 4);
}
 
// Ne retourne qu'un apercu de l'etat de la baterie : 10 = faible
byte battery(const byte* data)
{
    return (data[4] & 0x4) ? 10 : 90;
}
 
 
 
byte channel(const byte* data)    //подпрограмма определения канала !
{
    byte channel;
    switch (data[2])
    {
        case 0x10:
            channel = 1;
            chanel = 1;
            break;
        case 0x20:
            channel = 2;
            chanel = 2;
            break;
        case 0x40:
            channel = 3;
            chanel = 3;
            break;
     }
 
     return channel;
     chanel = channel;
}
 
void reportSerial (const char* s, class DecodeOOK& decoder)     
{
    ///float CurTemp1, CurHum1;   // переменные текущёй температуры и влажности
    byte pos;
    const byte* data = decoder.getData(pos);
    Serial.print(s);
    Serial.print(' ');
    for (byte i = 0; i < pos; ++i) {
        Serial.print(data[i] >> 4, HEX);
        Serial.println(data[i] & 0x0F, HEX);
    }
 
      
///////////////////////////////////////////////////////////////////// пробный алгоритм определения канала (начало)
    if(data[2] == 0x10)  // если канал 1
    {
    //  Serial.println("Find data on Channel:");
       Serial.print("[THN132N,...] Id:");
       Serial.print(data[3], HEX);
       Serial.print(" ,Channel:");
       Serial.print(channel(data));
       Serial.print(" ,temp:");
       Serial.print(temperature(data));
       CurTemp1=temperature(data);   /// присваеваем текущие значения температуры  первому каналу
       Serial.print(" ,hum:");
       Serial.print(humidity(data));
       CurHum1=humidity(data);        /// присваеваем текущие значения  влажности  первому каналу
       Serial.print(" ,bat:");
       Serial.println(battery(data)); 
       SerPrintTable ();      ///  отображаем текущие переменные
       
     }
    
     else if(data[2] == 0x20)     //  если канал = 2
               {   
       Serial.print("[THGR228N,...] Id:");
       Serial.print(data[3], HEX);
       Serial.print(" ,Channel:");
       Serial.print(channel(data));
       Serial.print(" ,temp:");
       Serial.print(temperature(data));
       CurTemp2=temperature(data);   /// присваеваем текущие значения температуры второго канала
       Serial.print(" ,hum:");
       Serial.print(humidity(data));
       CurHum2=humidity(data);         /// присваеваем текущие значения  влажности второго канала
       Serial.print(" ,bat:");
       Serial.println(battery(data)); 
       SerPrintTable ();   ///  отображаем текущие переменные
      /* Serial.print("CurTemp1="); 
       Serial.println(CurTemp1); 
       Serial.print("CurHum1=");
       Serial.println(CurHum1);  
       Serial.print("CurTemp2="); 
       Serial.println(CurTemp2); 
       Serial.print("CurHum2=");
       Serial.println(CurHum2); 
       //Serial.println(CharHum2); 
       Serial.print("CurTemp3="); 
       Serial.println(CurTemp3); 
       Serial.print("CurHum3=");
       Serial.println(CurHum3); 
       //Serial.println(CharHum3);  
       //   Serial.println(chanel); ///  отображаем текущие переменные
       //Serial.println();*/
    }
    
      
     else if(data[2] == 0x40)     //  если канал = 3 (ТРИ)   добавлено 15.02.2015    0x40 hex == 64 dec
               {
       Serial.print("[THGR228N,...] Id:");
       Serial.print(data[3], HEX);
       Serial.print(" ,Channel:");
       Serial.print(channel(data));
       Serial.print(" ,temp:");
       Serial.print(temperature(data));
       CurTemp3=temperature(data);   /// присваеваем текущие значения температуры третего канала
       Serial.print(" ,hum:");
       Serial.print(humidity(data));
       CurHum3=humidity(data);         /// присваеваем текущие значения  влажности третего канала  
       Serial.print(" ,bat:");
       Serial.println(battery(data)); 
       SerPrintTable ();  /// отладочный вывод в порт текущих значений переменных....
     /*  Serial.print("CurTemp1="); 
       Serial.println(CurTemp1); 
       Serial.print("CurHum1=");
       Serial.println(CurHum1);  
       Serial.print("CurTemp2="); 
       Serial.println(CurTemp2); 
       Serial.print("CurHum2=");
       Serial.println(CurHum2); 
       //Serial.println(CharHum2); 
       Serial.print("CurTemp3="); 
       Serial.println(CurTemp3); 
       Serial.print("CurHum3=");
       Serial.println(CurHum3); 
       //Serial.println(CharHum3); 
      // Serial.println(chanel); ///  отображаем текущие переменные
       //Serial.println();  */
    }
    
   
 
 else {
   Serial.println("NOT Find data of Channel:");
 }
   /*
     dps.getPressure(&Pres);  //получение давление в паскалях
     dps.getTemperature(&Tempera); 
    // Presmm = Pres/133.3224;  // преобразование в мм рт. ст
     //dtostrf(Presmm, 4, 1, CharPresmm);  //преобразование текущего давления из числа в текст
     BMPTemp= Tempera*0.1,1;   //преобразование температуры датчика BMP в нормальный вид  ????
     //dtostrf(BMPTemp, 4, 1, CharBMPTemp);  //преобразование температуры датчика BMP из числа в текст  
     Presmm = Pres/133.3224;  // преобразование в мм рт. ст
     Serial.print("Presmm=");
     Serial.print(Presmm);
     Serial.print("       ");
     Serial.print("BMPTemp=");
     Serial.print(BMPTemp);
     Serial.println();   */
     
     decoder.resetDecoder();
}

//**********************************************************************************************************************************************************************************************************************************************************
void setup ()         //      setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup  setup      
{
  wdt_disable(); // бесполезная строка до которой не доходит выполнение при bootloop, отключаем WDT
  lcd.init();                      // инициализация ЖКИ   экрана 
  lcd.backlight();   // включаем подсветку экрана
  lcd.print("Start-UP  BOOT ....");    ///  выводим сообщение о загрузке
  Serial.begin(115200);
   Serial.println("ook_est__Decoder__11_tmp_narmon_Oregon_est_10_75_chnls_19lcd_newIP ");
    Serial.println("configuring Ethernet connection . . . ");
   ////delay(4000);
  // Ethernet connection:
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    lcd.setCursor(1,1);    /// выводим сообщение о загрузке
    lcd.print("Ethernet ERROR!!!");    /// выводим сообщение
    // ничего не делаем. doing nothing
    for(;;);
  }
  // пара секунд для инициализации Ethernet. Couple of seconds just to warm up Ethernet.
  delay(2000);
  lcd.setCursor(1,1);    /// выводим сообщение о загрузке
    lcd.print("Ethernet OK !");    /// выводим сообщение
  
  dps.init();   // какаято инициализация барометра
  
  lastConnectionTime = millis()-postingInterval+75000; //первое соединение через 75 секунд после запуска. First connection after 75 sec timeout.
  //устанавливаем pin 9 на выход
  pinMode(ledPin, OUTPUT); 
  ///digitalWrite(ledPin, LOW); 
  
  digitalWrite(ledPin, HIGH); 
 
  ///Serial.print("LIBRARY VERSION: ");
  ///Serial.println(DHT_LIB_VERSION);
  Serial.println();
  //*********************************************************************************************************************************************************************************************************************************************************
  Serial.println("*********** narodmon.ru Temperature/Humidity/Pressure ARDUINO from Eugene Titov ***********");
  ///digitalWrite(ledPin, LOW);
        //Serial.println("\n[ookDecoder] working...");
        wdt_enable (WDTO_8S); // watchdog включен и установлен на  8 сек.
        //Serial.println("WatchDog is Enabled"); //debug сообщение про watchdog
    attachInterrupt(1, ext_int_1, CHANGE);
    float CurTemp1, CurHum1;   // переменные текущёй температуры и влажности
 
    //DDRE  &= ~_BV(PE5); //input with pull-up 
    //PORTE &= ~_BV(PE5);
}
 
 //********************************************************************************************************************************************************************************************************************************************************* 
void loop () {            //      loop       loop       loop       loop       loop       loop       loop       loop       loop       loop       loop       loop       loop       loop       loop       loop       loop       loop      
   
   WDTime = millis();
   if(WDTime - PrevWDTime >= 5000) {                 // величина задержки перед сбросом  WDT
    // save the last time reset wdc
    PrevWDTime = WDTime;
    wdt_reset();  // sbros watchdog
    //Serial.println("WatchDog reset"); //debug сообщение про watchdog
   }
   
    static int i = 0;
    cli();
    word p = pulse;
 
    pulse = 0;
    sei();
 
    if (p != 0)
    {
        if (orscV2.nextPulse(p))
            reportSerial("OSV2", orscV2);  
    }
    
    
    //********************************************************************************************************
  ////  delay (1000);
  ///digitalWrite(ledPin, HIGH);
  // TempDig = DHT.temperature;
   dtostrf(CurTemp1, 4, 1, CharTemp1);  //преобразование текущей температуры из числа в текст
   dtostrf(CurTemp2, 4, 1, CharTemp2);  //преобразование текущей температуры из числа в текст  2 канал
   dtostrf(CurTemp3, 4, 1, CharTemp3);  //преобразование текущей температуры из числа в текст  3 (ТРЕТИЙ) канал
 //// delay (1000);
  dtostrf(CurHum1, 4, 1, CharHum1);  //преобразование текущей влажности из числа в текст
   dtostrf(CurHum2, 4, 1, CharHum2);  //преобразование текущей влажности из числа в текст  2 канал
    dtostrf(CurHum3, 4, 1, CharHum3);  //преобразование текущей влажности из числа в текст  3 (ТРЕТИЙ) канал
  
   
  ////Serial.print (CharTemp1);
 //// Serial.print (" C");
 //// Serial.print(",\t");
 //// Serial.print (CharHum1);
  ////Serial.println ('%');
  ///digitalWrite(ledPin, LOW);

  ////delay(50000);                                 // задержка
  
  
  
  
  
  
  
  // -----------------------------------------------------------------------------------------------

  if (client.available()) {                    // я так понимаю, сначала идёт чтение и игнор данных подключенного клиента
    char c = client.read();

  }

  if (client.connected() && lastConnected)
  {

    client.stop();

  }


  if(!client.connected() && (millis() - lastConnectionTime > postingInterval))

  {

    
     dps.getPressure(&Pres);  //получение давление в паскалях
     dps.getTemperature(&Tempera); 
     Presmm = Pres/133.3224;  // преобразование в мм рт. ст
     dtostrf(Presmm, 4, 1, CharPresmm);  //преобразование текущего давления из числа в текст
     BMPTemp= Tempera*0.1,1;   //преобразование температуры датчика BMP в нормальный вид  ????
     dtostrf(BMPTemp, 4, 1, CharBMPTemp);  //преобразование температуры датчика BMP из числа в текст
     
    //формирование HTTP-запроса. Preparing http request.
   Serial.println("Bilding reply Buffer...");        /// Дебаг меседж...
    memset(replyBuffer, 0, sizeof(replyBuffer));      // отчистка буфера      
    //memset(replyBuffer2, 0, sizeof(replyBuffer2));      /// отчистка буфера 2  !!!       
    strcpy(replyBuffer,"ID=");
    //Конвертируем MAC-адрес. Converting mac address.
    for (int k=0; k<6; k++)
    {
      int b1=mac[k]/16;
      int b2=mac[k]%16;
      char c1[2],c2[2];

      if (b1>9) c1[0]=(char)(b1-10)+'A';
      else c1[0] = (char)(b1) + '0';
      if (b2>9) c2[0]=(char)(b2-10)+'A';
      else c2[0] = (char)(b2) + '0';

      c1[1]='\0';
      c2[1]='\0';

      strcat(replyBuffer,c1);
      strcat(replyBuffer,c2);
    }
    //--------------------------------------------------------------------------------------------------------------------------------------------
    
    if  (CurHum3 !=0)  {  // проверка наличия данных влажности с 3 канала     111111111111111111111111111111111111111111111111111111111111111111111
  
    //конвертируем адрес термодатчика с 3 ТРЕТЕГО канала. Converting first sensor address.---------------------------------------------------------
    

    strcat(replyBuffer,"&");
   /* for (int k=0; k<6; k++)
    {            
      int b1=mac[k]/16;
      int b2=mac[k]%16;
      char c1[2],c2[2];

      if (b1>9) c1[0]=(char)(b1-10)+'A';
      else c1[0] = (char)(b1) + '0';
      if (b2>9) c2[0]=(char)(b2-10)+'A';
      else c2[0] = (char)(b2) + '0';

      c1[1]='\0';
      c2[1]='\0';

      strcat(replyBuffer,c1);
      strcat(replyBuffer,c2);
    }*/
    strcat(replyBuffer,"Temp3");          // раньше было просто 01

    
    strcat(replyBuffer,"=");                         ///   определяем  отрицательные значения
    if (SignBit)
    {
      strcat(replyBuffer,"-");
    }
    strcat(replyBuffer,CharTemp3);                     // добавляем температуру 2 канала  CharTemp2 
    // //конвертируем адрес датчика влажности с 3 ТРЕТЕГО канала. Converting secong sensor address.-------------------------------------------------------------------------------------------------------------------------
    strcat(replyBuffer,"&");
    /*for (int k=0; k<6; k++)                                         // / снова Конвертируем MAC-адрес.
    {
      int b1=mac[k]/16;
      int b2=mac[k]%16;
      char c1[2],c2[2];

      if (b1>9) c1[0]=(char)(b1-10)+'A';
      else c1[0] = (char)(b1) + '0';
      if (b2>9) c2[0]=(char)(b2-10)+'A';
      else c2[0] = (char)(b2) + '0';

      c1[1]='\0';
      c2[1]='\0';

      strcat(replyBuffer,c1);
      strcat(replyBuffer,c2);
    }*/
    strcat(replyBuffer,"Hum3");              // раньше было просто 02

    strcat(replyBuffer,"=");

    strcat(replyBuffer,CharHum3);       // добавляем влажность 3 ТРЕТЕГО канала
  }
    // добавляем  CharHum3
    
    
   //--------------------------------------------------------------------------------------------------------------------------------------------- 
  if  (CurHum2 !=0)  {  // проверка наличия данных влажности с 2 канала     111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
  
    //конвертируем адрес термодатчика с 2 канала. Converting first sensor address.---------------------------------------------------------
    

    strcat(replyBuffer,"&");
   /* for (int k=0; k<6; k++)
    {
      int b1=mac[k]/16;
      int b2=mac[k]%16;
      char c1[2],c2[2];

      if (b1>9) c1[0]=(char)(b1-10)+'A';
      else c1[0] = (char)(b1) + '0';
      if (b2>9) c2[0]=(char)(b2-10)+'A';
      else c2[0] = (char)(b2) + '0';

      c1[1]='\0';
      c2[1]='\0';

      strcat(replyBuffer,c1);
      strcat(replyBuffer,c2);
    }*/
    strcat(replyBuffer,"Temp2");     // раньше было просто 03

    
    strcat(replyBuffer,"=");                         ///   определяем  отрицательные значения
    if (SignBit)
    {
      strcat(replyBuffer,"-");
    }
    strcat(replyBuffer,CharTemp2);                     // добавляем температуру 2 канала  CharTemp2 
    // //конвертируем адрес датчика влажности с 2 канала. Converting secong sensor address.-------------------------------------------------------------------------------------------------------------------------
    strcat(replyBuffer,"&");
   /* for (int k=0; k<6; k++)                                         // / снова Конвертируем MAC-адрес.
    {
      int b1=mac[k]/16;
      int b2=mac[k]%16;
      char c1[2],c2[2];

      if (b1>9) c1[0]=(char)(b1-10)+'A';
      else c1[0] = (char)(b1) + '0';
      if (b2>9) c2[0]=(char)(b2-10)+'A';
      else c2[0] = (char)(b2) + '0';

      c1[1]='\0';
      c2[1]='\0';

      strcat(replyBuffer,c1);
      strcat(replyBuffer,c2);
    }*/
    strcat(replyBuffer,"Hum2"); // раньше было просто 04

    strcat(replyBuffer,"=");

    strcat(replyBuffer,CharHum2);       // добавляем влажность 2 второго канала
  }
    // добавляем  CharHum2
    
   //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
    
      // //конвертируем адрес датчика давления. Converting secong sensor address.-------------------------------------------------------------------------------------------------------------------------
    strcat(replyBuffer,"&");
    /*for (int k=0; k<6; k++)                                         // / снова Конвертируем MAC-адрес.
    {
      int b1=mac[k]/16;
      int b2=mac[k]%16;
      char c1[2],c2[2];

      if (b1>9) c1[0]=(char)(b1-10)+'A';
      else c1[0] = (char)(b1) + '0';
      if (b2>9) c2[0]=(char)(b2-10)+'A';
      else c2[0] = (char)(b2) + '0';

      c1[1]='\0';
      c2[1]='\0';

      strcat(replyBuffer,c1);
      strcat(replyBuffer,c2);
    }*/
    strcat(replyBuffer,"pressure");  //+++++     // раньше было просто 05

    strcat(replyBuffer,"=");

    strcat(replyBuffer,CharPresmm);                              // добавляем  давление
    // __________________________________________________________________________________________________________________________________________________________________________________________
    
    // //конвертируем адрес температуры датчика BMP. Converting secong sensor address.-------------------------------------------------------------------------------------------------------------------------
    strcat(replyBuffer,"&");
   /* for (int k=0; k<6; k++)                                         // / снова Конвертируем MAC-адрес.
    {
      int b1=mac[k]/16;
      int b2=mac[k]%16;
      char c1[2],c2[2];

      if (b1>9) c1[0]=(char)(b1-10)+'A';
      else c1[0] = (char)(b1) + '0';
      if (b2>9) c2[0]=(char)(b2-10)+'A';
      else c2[0] = (char)(b2) + '0';

      c1[1]='\0';
      c2[1]='\0';

      strcat(replyBuffer,c1);
      strcat(replyBuffer,c2);
    }*/
    strcat(replyBuffer,"BMPTemp");  //+++++     

    strcat(replyBuffer,"=");

    strcat(replyBuffer,CharBMPTemp);                              // добавляем  температуру датчика BMP.
    //_________________________________________________________________________________________________________________________________________________________________________________
    
    
    if  (CurHum1 !=0)
    // добавляем  CharHum1-------------------------------------------------                Если есть данные с 1 канала
    {
     strcat(replyBuffer,"&");
    /*for (int k=0; k<6; k++)
    {
      int b1=mac[k]/16;
      int b2=mac[k]%16;
      char c1[2],c2[2];

      if (b1>9) c1[0]=(char)(b1-10)+'A';
      else c1[0] = (char)(b1) + '0';
      if (b2>9) c2[0]=(char)(b2-10)+'A';
      else c2[0] = (char)(b2) + '0';

      c1[1]='\0';
      c2[1]='\0';

      strcat(replyBuffer,c1);
      strcat(replyBuffer,c2);
    }*/
    strcat(replyBuffer,"Temp1");  //+++++         // раньше было просто 06

    
    strcat(replyBuffer,"=");                         ///   определяем  отрицательные значения
    if (SignBit)
    {
      strcat(replyBuffer,"-");
    }
    strcat(replyBuffer,CharTemp1);                     // добавляем  CharTemp1
    // //конвертируем адрес датчика влажности с 1 канала. -------------------------------------------------------------------------------------------------------------------------
    strcat(replyBuffer,"&");
    /*for (int k=0; k<6; k++)                                         // / снова Конвертируем MAC-адрес.
    {
      int b1=mac[k]/16;
      int b2=mac[k]%16;
      char c1[2],c2[2];

      if (b1>9) c1[0]=(char)(b1-10)+'A';
      else c1[0] = (char)(b1) + '0';
      if (b2>9) c2[0]=(char)(b2-10)+'A';
      else c2[0] = (char)(b2) + '0';

      c1[1]='\0';
      c2[1]='\0';

      strcat(replyBuffer,c1);
      strcat(replyBuffer,c2);
    }*/
    strcat(replyBuffer,"Hum1");    // раньше было просто 05

    strcat(replyBuffer,"=");

    strcat(replyBuffer,CharHum1);   
    
  }
    
    
    
    
     httpRequest();                                  // функция отправки запроса  ?
  }

  lastConnected = client.connected();

}



void SerPrintTable () {                            // функция  выдачи в порт таблицы резульатов  
       
       
       //Serial.println();                            // начальный перевод строки
       Serial.print("CurTemp1="); 
       Serial.print(CurTemp1); 
       Serial.print("      ");
       Serial.print("CurTemp2="); 
       Serial.print(CurTemp2); 
       Serial.print("      ");
       Serial.print("CurTemp3="); 
       Serial.println(CurTemp3); 
       Serial.print("CurHum1=");
       Serial.print(CurHum1);  
       Serial.print("       ");
       Serial.print("CurHum2=");
       Serial.print(CurHum2);
       Serial.print("       ");
       //Serial.println(CharHum2); 
       Serial.print("CurHum3=");
       Serial.println(CurHum3); 
       //Serial.println(CharHum3); 
     dps.getPressure(&Pres);  //получение давление в паскалях
     dps.getTemperature(&Tempera); 
     Presmm = Pres/133.3224;  // преобразование в мм рт. ст
     dtostrf(Presmm, 4, 1, CharPresmm);  //преобразование текущего давления из числа в текст
     BMPTemp= Tempera*0.1,1;   //преобразование температуры датчика BMP в нормальный вид  ????
     dtostrf(BMPTemp, 4, 1, CharBMPTemp);  //преобразование температуры датчика BMP из числа в текст
     Serial.print("Presmm=");
     Serial.print(Presmm);
     Serial.print("      ");
     Serial.print("BMPTemp=");
     Serial.print(BMPTemp);
     //Serial.println(); 
    // wdt_reset();  // sbros watchdog
    LCDPrintTable();
     Serial.println();  
        
        
    }
    
void LCDPrintTable () {                            // функция  отображения  резульатов  на LCD   (добавлено 21.07.2015)
lcd.clear();   // отчистка дисплея
lcd.backlight();  // включение подсветки
lcd.setCursor(0,0);

lcd.print("T1= ");
lcd.print(CurTemp1,1);
lcd.setCursor(9,0);
lcd.print(" H1= ");
lcd.print(CurHum1,0);
lcd.print(" %");


lcd.setCursor(0,1);
lcd.print("T2= ");
lcd.print(CurTemp2,1);
lcd.setCursor(9,1);
lcd.print(" H2= ");
lcd.print(CurHum2,0);
lcd.print(" %");


lcd.setCursor(0,2);
lcd.print("T3= ");
lcd.print(CurTemp3,1);
lcd.setCursor(9,2);
lcd.print(" H3= ");
lcd.print(CurHum3,0);
lcd.print(" %");

lcd.setCursor(0,3);
lcd.print("P= ");
lcd.print(Presmm);
lcd.print(" Tmp= ");
lcd.print(BMPTemp);

}


//   вроде функция     отправка запроса


void httpRequest() {
   // Serial.println("*********************");
  lcd.noBacklight();  // вЫключение подсветки
  if (client.connect(server, 80)) {

    // send the HTTP POST request:
    //Serial.println("Start Sending DATA to SERVER....");    //debug messege
    client.println("POST http://narodmon.ru/post.php HTTP/1.0");
    Serial.println("POST http://narodmon.ru/post.php HTTP/1.0");
    client.println("Host: narodmon.ru");
    Serial.println("Host: narodmon.ru");
    //client.println("User-Agent: arduino-ethernet");
    //client.println("Connection: close");
    client.println("Content-Type: application/x-www-form-urlencoded");
    Serial.println("Content-Type: application/x-www-form-urlencoded");
    client.print("Content-Length: ");
    client.println(strlen(replyBuffer));
    Serial.print("Content-Length: ");
    Serial.println(strlen(replyBuffer));
    client.println();
    Serial.println();
    client.println(replyBuffer);
    Serial.println(replyBuffer);
    // дебаг// дебаг// дебаг// дебаг// дебаг// дебаг// дебаг// дебаг// дебаг
    SerPrintTable ();  //  тестовый табличный вывод в порт
    /*Serial.print("CurTemp1=");
    Serial.println(CurTemp1);
    Serial.print("CurHum1=");
    Serial.println(CurHum1);
    Serial.print("Presmm=");
    Serial.println(Presmm);
    Serial.print("CurTemp2=");
    Serial.println(CurTemp2);
    Serial.print("CurHum2=");
    Serial.println(CurHum2);
    
    Serial.print("CurTemp3=");
    Serial.println(CurTemp3);
    Serial.print("CurHum3=");
    Serial.println(CurHum3);*/


    
    client.println();
    lastConnectionTime = millis();
    delay(1000);
  }
  else {
    Serial.print("http request failed - client not connected");   // ОШИБКА СОЕДИНЕНИЯ ??
    
    lcd.clear();   // отчистка дисплея
    lcd.backlight();  // включение подсветки
    lcd.setCursor(1,1);
    lcd.print("Connection Error!!!");

    client.stop();
    CurHum3 =0;
    CurHum2 =0; 
    CurHum1 =0;   //  сброс параметров для определения каналов
    delay(9000);  //   задержка для срабатывания WatchDog
  }
}

//
// END OF FILE
//******************************************************************************************************************************
///}
