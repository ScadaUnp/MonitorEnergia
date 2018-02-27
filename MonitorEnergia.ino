/**
 * Monitoramento de Energia usando ESP32
 * Autor: João Campos
 * Sensores Utilizados
 * 1 Transformador de Corrente sct013
 *
 * Tutoriais Utilizados
 * https://www.filipeflop.com/blog/medidor-de-corrente-sct013-com-arduino/
 * http://esp-idf.readthedocs.io/en/latest/api-reference/peripherals/adc.html
 * https://github.com/openenergymonitor/EmonLib/blob/master/EmonLib.cpp
 * https://github.com/capella-ben/emon_esp32/blob/master/Power_Monitor.ino
 * https://github.com/espressif/esp-idf/blob/6acb38af4c273077f6cfd54d6f84af6d5f1e20f9/examples/peripherals/adc/main/adc1_example_main.c
 */
#include <Modbus.h>
#include <ModbusIP_ESP32.h>
#include <sys/time.h>
/*#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF    1100        //Valor de Tensão de referência em MV
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;
*/

// from EmonLib
#define ADC_BITS    12
#define ADC_COUNTS  (1<<ADC_BITS)
// Calibrar aqui a Corrente
// Pino, calibracao - Cur Const= Ratio/BurdenR. 2000/33 = 60 --> https://www.filipeflop.com/blog/medidor-de-corrente-sct013-com-arduino/
float ICAL = 60;
float offsetI = ADC_COUNTS>>1;
float filteredI = 0;
float last_filtered_value;
double  sqI, sumI, Irms;
int sample = 0;


void floatToWordArray(float , word * );
float calcIrms(const int, unsigned int);


/** From --> http://esp-idf.readthedocs.io/en/latest/api-reference/peripherals/adc.html

The ADC driver API supports ADC1 (9 channels, attached to GPIOs 32 - 39), and ADC2 (10 channels,
attached to GPIOs 0, 2, 4, 12 - 15 and 25 - 27). However, there’re some restrictions for the application to use ADC2:

 ESP32 Core Board V2 / ESP32 DevKitC: GPIO 0 cannot be used due to external auto program circuits.
*/
const int porta_TC = GPIO_NUM_34;

//Constantes para o sensor de corrente



//ModbusIP object
ModbusIP mb;



//Declaração de INPUT registers
const int SENSOR_CORRENTE_0 = 0;
const int SENSOR_CORRENTE_1 = 1;



hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  portEXIT_CRITICAL_ISR(&timerMux);

  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);

  // It is safe to use digitalRead/Write here if you want to toggle an output
}


void setup() {
  Serial.begin(115200);
    delay(10);
 // Criar semáforo para checar no loop
  timerSemaphore = xSemaphoreCreateBinary();
  //Config Modbus IP
  mb.config("Sua SSID", "Sua Password");


     while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    // Declarar tipos de portas
  pinMode(porta_TC, INPUT);
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Colocar alarme para a função onTimer ser chamada a cada 1 s (valor em microsegundos).
  // Repetir alarme (=true)
  timerAlarmWrite(timer, 1000000, true);

  // Adicionar input registers

  mb.addIreg(SENSOR_CORRENTE_0);
  mb.addIreg(SENSOR_CORRENTE_1);


  // Iniciar alarme
  timerAlarmEnable(timer);

}

void loop() {
  //Call once inside loop() - all magic here
   mb.task();


  // semáforo foi disparado pelo Hardware timer
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){

      float I = calcIrms(porta_TC, 1000);
      word registro_corrente[2];


      floatToWordArray(I, registro_corrente);
      mb.Ireg(SENSOR_CORRENTE_0, registro_corrente[1]);
      mb.Ireg(SENSOR_CORRENTE_1, registro_corrente[0]);


  }

  // put your main code here, to run repeatedly:

}

// Adaptad from EmonLib
// and https://github.com/capella-ben/emon_esp32/blob/master/Power_Monitor.ino
float calcIrms(const int pin, unsigned int Number_of_Samples){

  int SupplyVoltage=3300;

  for (unsigned int n = 0; n < Number_of_Samples; n++)
    {
      sample = analogRead(pin);

      // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
      //  then subtract this - signal is now centered on 0 counts.
      offsetI = (offsetI + (sample-offsetI)/1024);
      filteredI = sample - offsetI;

      // Root-mean-square method current
      // 1) square current values
      sqI = filteredI * filteredI;
      // 2) sum
      sumI += sqI;

      #ifdef DEBUG      //debug generates the table of output so you can plot a graph
        Serial.print(n);
        Serial.print("\t");
        Serial.print(sample);
        Serial.print("\t");
        Serial.print(filteredI);
        Serial.print("\t");
        Serial.println(offsetI);
      #endif


    }

    double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
    Irms = I_RATIO * sqrt(sumI / Number_of_Samples);

    #ifdef DEBUG      // summary var output
      Serial.println();
      Serial.print("I_RATIO ");
      Serial.println(I_RATIO);
      Serial.print("Irms:   ");
      Serial.println(Irms);
      Serial.print("Power:  ");
      Serial.println(Irms*240);
    #endif

    sumI = 0;

    return Irms;


}




void floatToWordArray(float number, word *reg){
 byte*  ArrayOfFourBytes;
 ArrayOfFourBytes = (byte*) &number;
 reg[0] = (ArrayOfFourBytes[1]<<8)| ArrayOfFourBytes[0];
 reg[1] = (ArrayOfFourBytes[3]<<8)| ArrayOfFourBytes[2];
}
