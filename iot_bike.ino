#include <stdio.h>
#include <driverlib.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <stdlib.h>
#include <LCD_screen.h>
#include <LCD_screen_font.h>
#include <LCD_utilities.h>
#include <Screen_HX8353E.h>
#include <Terminal12e.h>
#include <Terminal6e.h>
#include <Terminal8e.h>
//#include <ESP8266WiFi.h>
Screen_HX8353E myScreen;
void drawAccelData(void);
volatile uint16_t resultsBuffer[7];
const int SAMPLE_FREQ = 5;

// your network name also called SSID
const char *ssid = "mousse"; // Enter your WiFi name
const char *password = "qweqweqwe";  // Enter WiFi password// your network password
// MQTTServer to use
char server[] = "192.168.1.125";
volatile uint8_t hundredths = 0;
volatile uint32_t seconds = 0;
volatile uint8_t initialized = 0;
volatile uint8_t windowActive = 0;
volatile uint8_t timeWindow = 0;
uint8_t tau = 10000;
uint8_t countr = 0;
const char *mqtt_broker = "broker.emqx.io";
const char *topic = "esp8266/test";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;


WiFiClient wifiClient;
PubSubClient client(server, 1883, callback, wifiClient);
void callback(char* topic, byte* payload, unsigned int length) {

  // Type Cast Input Bytes to Char
  char* str = (char*)payload;

  /* Check the Second Character of the char* pointer
      str[1] == 'N' ---> IO_button = ON
      str[1] == 'F' ---> IO_button = OFF
  */
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message:");
    for (int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
    }
    Serial.println();
    Serial.println("-----------------------");
  /*  feed data */
  /* Implement logic to decide whether Xacc or Yacc is published here */

}


void SysTick_Handler() {
  if (hundredths == 100) {
    hundredths = 0;
    seconds++;
    myScreen.gText(40, 115,  "G2: " + String(seconds));
  }
  hundredths++;
}


void LCD_init() {

  myScreen.begin();

  /* Let's make a title*/
  myScreen.gText(8, 5, "Acceleration Data");
  //myScreen.gText(8, 35, "Velocity Data");
  //myScreen.gText(8, 65, "Displacement Data");
  //myScreen.gText(8, 95, "Gesture Detected");

}

void ADC_init() {
  /* Configures Pin 4.0, 4.2, and 6.1 ad ADC inputs */
  // ACC Z = P4.2
  // ACC Y = P4.0
  // ACC X = P6.1
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION);
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

  ADC14_registerInterrupt(ADC14_IRQHandler);

  /* Initializing ADC (ADCOSC/64/8) */
  ADC14_enableModule();
  ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

  /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM2 (A11, A13, A14)  with no repeat)
     with internal 2.5v reference */
  ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, true);
  ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, ADC_NONDIFFERENTIAL_INPUTS);

  ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);

  ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A11, ADC_NONDIFFERENTIAL_INPUTS);


  ADC14_setResolution(ADC_10BIT); //IMPORTANT -> This seemed to give me the least noisey values (8 bit res was too small though)

  /* Enabling the interrupt when a conversion on channel 2 (end of sequence)
      is complete and enabling conversions */
  ADC14_enableInterrupt(ADC_INT2);

  /* Enabling Interrupts */
  Interrupt_enableInterrupt(INT_ADC14);
  Interrupt_enableMaster();

  /* Setting up the sample timer to automatically step through the sequence
     convert.*/
  ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

  /* Triggering the start of the sample */
  ADC14_enableConversion();
  ADC14_toggleConversionTrigger();

  SysTick_enableInterrupt();


}

void drawAccelData(void) {
  myScreen.gText(40, 15,  "X: " + String(resultsBuffer[0]));
  myScreen.gText(40, 25,  "Y: " + String(resultsBuffer[1]));
  //myScreen.gText(40, 45,  "X: " + String(resultsBuffer[2]));
  //myScreen.gText(40, 55,  "Y: " + String(resultsBuffer[3]));
  //myScreen.gText(40, 75,  "X: " + String(resultsBuffer[4]));
  //myScreen.gText(40, 85,  "Y: " + String(resultsBuffer[5]));
  //myScreen.gText(40, 105, String(resultsBuffer[6]));
  myScreen.gText(8, 75, "Vibration: " + String(countr));
  //  myScreen.gText(40, 105,  "G: " + String(/*Gesture Detected*/));
  //  myScreen.gText(40, 115,  "S: " + String(/*Current State - Nice For Debugging*/));

}

void ADC14_IRQHandler(void)
{
  uint64_t status;

  status = MAP_ADC14_getEnabledInterruptStatus();
  MAP_ADC14_clearInterruptFlag(status);

  /* ADC_MEM2 conversion completed */
  if (status & ADC_INT2)
  {

    // if ([DATA IS NOT BEING PUBLISHED TO SERVER])


    /* Store ADC14 conversion results */
    //    resultsBuffer[0] = ADC14_getResult(ADC_MEM0); // X Accelerometer
    //    resultsBuffer[1] = ADC14_getResult(ADC_MEM1); // Y Accelerometer

    volatile uint16_t x_readings[10];
    volatile uint16_t y_readings[10];


    /* Below implement logic for a smoothing filter for Acc readings, numReadings = 10 */
    int count = 9;

    while (count >= 0) {
      x_readings[count] = ADC14_getResult(ADC_MEM0);
      y_readings[count] = ADC14_getResult(ADC_MEM1);
      count--;
    }

    volatile uint16_t x_sum = 0;
    volatile uint16_t y_sum = 0;

    for (int i = 0; i < 10; i++) {
      x_sum += x_readings[i];
      y_sum += y_readings[i];
    }

    volatile uint16_t curr_x_acc = x_sum / 10;
    volatile uint16_t curr_y_acc = y_sum / 10;

    /************************************************************************************/

    // Calculate Velocity Readings
    volatile uint16_t curr_x_vel = ((resultsBuffer[0] + curr_x_acc) / (2 * SAMPLE_FREQ));
    volatile uint16_t curr_y_vel = ((resultsBuffer[1] + curr_y_acc) / (2 * SAMPLE_FREQ));


    // Calculate Displacement Readings
    volatile uint16_t curr_x_dis = ((resultsBuffer[2] + curr_x_vel) / (2 * SAMPLE_FREQ)) * 10;
    volatile uint16_t curr_y_dis = ((resultsBuffer[3] + curr_y_vel) / (2 * SAMPLE_FREQ)) * 10;
    if (windowActive == 1) {
      timeWindow = timeWindow + 1;
    }
    if (windowActive == 1 && timeWindow > tau) {
      windowActive = 0;
      timeWindow = 0;
    }
    if (curr_x_dis == 200 && curr_y_dis == 200) {
      initialized = 1;
    }

    if (initialized == 1) {

      if (curr_y_dis < 200 || curr_x_dis > 200) {
        resultsBuffer[6] = 1;
        windowActive = 1;
        countr++;
      }
      else {
        resultsBuffer[6] = 0;
      }
    }
    //Set values back to normal
    resultsBuffer[0] = curr_x_acc;
    resultsBuffer[1] = curr_y_acc;
    resultsBuffer[2] = curr_x_vel;
    resultsBuffer[3] = curr_y_vel;
    resultsBuffer[4] = curr_x_dis;
    resultsBuffer[5] = curr_y_dis;


    /* IMPLEMENT GESTURE DETECTION STATE MACHINE */



    // Draw accelerometer data on display
    drawAccelData();
  }

}


void setup() {

  WDT_A_hold(WDT_A_BASE);

  Serial.begin(115200);
  //
  /*  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
      String client_id = "esp8266-client-";
      client_id += String(WiFi.macAddress());
      Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      } else {
          Serial.print("failed with state ");
          Serial.print(client.state());
          delay(2000);
      }
  }*/

  //   Start Ethernet with the build in MAC Address
  //   attempt to connect to Wifi network:
  Serial.begin(115200);
  // connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("Connecting to WiFi..");
  }


  Serial.println("\nYou're connected to the network");
  //
  //

  int count = 0; 
  while(count < 10000) {
    Serial.println(client.connect("aedes", "username", "password")); 
    count++; 
  }
  
//  if (!client.connect("aedes", "username", "password")) {
//    Serial.println("Connection failed");
//  } else {
//    Serial.println("Connection success");
//  }


  /* Initialize all variables to 0 here  */


  /***************************************/

  /* Initialize LCD and make some titles */
  LCD_init();

  SysTick_registerInterrupt(SysTick_Handler);
  SysTick_setPeriod(480000);
  SysTick->VAL = 0;
  SysTick_enableModule();

  /* Initialize ADC *SEE THIS FUNCTION IMPORTANT LINES ARE HIGHLIGHTED* */
  ADC_init();



}

void loop() {


  /* Everything for interacting with the MQTT server is in this loop */
  /* When publishing to MQTT, you should not be interfacing with the ADC */



}
