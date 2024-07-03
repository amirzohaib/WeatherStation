/***************************************************************************
This Program Takes in Analog values from LDR and converts them to a value 0-100% and displays it on OLED 
 ***************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>


/***************************************************************************
OLED 
 ***************************************************************************/

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)

/***************************************************************************
BMP280 Sensor 
 ***************************************************************************/

Adafruit_BMP280 bmp; // I2C address is 0x76 or 0x77
float TempCalibrated; // Variable declared to Calibrate the Sensor Readings

/***************************************************************************
AHT20 Sensor 
 ***************************************************************************/
Adafruit_AHTX0 aht;


/***************************************************************************
LDR
 ***************************************************************************/
#define ldr  33 


/***************************************************************************
Button and Debounce
 ***************************************************************************/
#define buttonPin  18    

int buttonState;              // current reading from the input pin
int lastButtonState = LOW;    // previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

/***************************************************************************
Screen Menus
 ***************************************************************************/
int displayScreenNum = 0;
int displayScreenNumMax = 3;
unsigned long lastTimer = 0;
unsigned long timerDelay = 15000;
/***************************************************************************
Symbols
 ***************************************************************************/
unsigned char temperature_icon[] ={
  0b00000001, 0b11000000, //        ###      
  0b00000011, 0b11100000, //       #####     
  0b00000111, 0b00100000, //      ###  #     
  0b00000111, 0b11100000, //      ######     
  0b00000111, 0b00100000, //      ###  #     
  0b00000111, 0b11100000, //      ######     
  0b00000111, 0b00100000, //      ###  #     
  0b00000111, 0b11100000, //      ######     
  0b00000111, 0b00100000, //      ###  #     
  0b00001111, 0b11110000, //     ########    
  0b00011111, 0b11111000, //    ##########   
  0b00011111, 0b11111000, //    ##########   
  0b00011111, 0b11111000, //    ##########   
  0b00011111, 0b11111000, //    ##########   
  0b00001111, 0b11110000, //     ########    
  0b00000111, 0b11100000, //      ######     
};

unsigned char humidity_icon[] ={
  0b00000000, 0b00000000, //                 
  0b00000001, 0b10000000, //        ##       
  0b00000011, 0b11000000, //       ####      
  0b00000111, 0b11100000, //      ######     
  0b00001111, 0b11110000, //     ########    
  0b00001111, 0b11110000, //     ########    
  0b00011111, 0b11111000, //    ##########   
  0b00011111, 0b11011000, //    ####### ##   
  0b00111111, 0b10011100, //   #######  ###  
  0b00111111, 0b10011100, //   #######  ###  
  0b00111111, 0b00011100, //   ######   ###  
  0b00011110, 0b00111000, //    ####   ###   
  0b00011111, 0b11111000, //    ##########   
  0b00001111, 0b11110000, //     ########    
  0b00000011, 0b11000000, //       ####      
  0b00000000, 0b00000000, //                 
};

unsigned char arrow_down_icon[] ={
  0b00001111, 0b11110000, //     ########    
  0b00011111, 0b11111000, //    ##########   
  0b00011111, 0b11111000, //    ##########   
  0b00011100, 0b00111000, //    ###    ###   
  0b00011100, 0b00111000, //    ###    ###   
  0b00011100, 0b00111000, //    ###    ###   
  0b01111100, 0b00111110, //  #####    ##### 
  0b11111100, 0b00111111, // ######    ######
  0b11111100, 0b00111111, // ######    ######
  0b01111000, 0b00011110, //  ####      #### 
  0b00111100, 0b00111100, //   ####    ####  
  0b00011110, 0b01111000, //    ####  ####   
  0b00001111, 0b11110000, //     ########    
  0b00000111, 0b11100000, //      ######     
  0b00000011, 0b11000000, //       ####      
  0b00000001, 0b10000000, //        ##       
};

unsigned char sun_icon[] ={
  0b00000000, 0b00000000, //        Icon for Sun         
  0b00100000, 0b10000010, //   #     #     # 
  0b00010000, 0b10000100, //    #    #    #  
  0b00001000, 0b00001000, //     #       #   
  0b00000001, 0b11000000, //        ###      
  0b00000111, 0b11110000, //      #######    
  0b00000111, 0b11110000, //      #######    
  0b00001111, 0b11111000, //     #########   
  0b01101111, 0b11111011, //  ## ######### ##
  0b00001111, 0b11111000, //     #########   
  0b00000111, 0b11110000, //      #######    
  0b00000111, 0b11110000, //      #######    
  0b00010001, 0b11000100, //    #   ###   #  
  0b00100000, 0b00000010, //   #           # 
  0b01000000, 0b10000001, //  #      #      #
  0b00000000, 0b10000000, //         #       
};

/***************************************************************************
Screen menu Functions
 ***************************************************************************/
// SCREEN NUMBER 0: TEMPERATURE
void displayTemperature(){
  sensors_event_t humidity, temp; //Get Data from AHT20
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

  display.clearDisplay();
  display.setTextSize(2);
  display.drawBitmap(15, 5, temperature_icon, 16, 16 ,1);
  display.setCursor(35, 5);
  float temperature = bmp.readTemperature();
  display.print(temperature);
  display.cp437(true);
  display.setTextSize(1);
  display.print(" ");
  display.write(167);
  display.print("C");
  display.setCursor(0, 34);
  display.setTextSize(1);
  display.print("Humidity: ");
  display.print(humidity.relative_humidity);
  display.print(" %");
  display.setCursor(0, 44);
  display.setTextSize(1);
  display.print("Pressure: ");
  display.print(bmp.readPressure()/100.0F);
  display.print(" hpa");
  //displayIndicator(displayScreenNum);
  display.display();
 
}

// SCREEN NUMBER 1: HUMIDITY
void displayHumidity(){
  sensors_event_t humidity, temp; //Get Data from AHT20
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

  display.clearDisplay();
  display.setTextSize(2);
  display.drawBitmap(15, 5, humidity_icon, 16, 16 ,1);
  display.setCursor(35, 5);
  //float humidity = humidity.relative_humidity;
  display.print(humidity.relative_humidity);
  display.print(" %");
  display.setCursor(0, 34);
  display.setTextSize(1);
  display.print("Temperature: ");
  display.print(bmp.readTemperature());
  display.cp437(true);
  display.print(" ");
  display.write(167);
  display.print("C");
  display.setCursor(0, 44);
  display.setTextSize(1);
  display.print("Pressure: ");
  display.print(bmp.readPressure()/100.0F);
  display.print(" hpa");
  //displayIndicator(displayScreenNum);
  display.display();
 
}

// SCREEN NUMBER 2: PRESSURE
void displayPressure(){
  sensors_event_t humidity, temp; //Get Data from AHT20
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data


  display.clearDisplay();
  display.setTextSize(2);
  display.drawBitmap(0, 5, arrow_down_icon, 16, 16 ,1);
  display.setCursor(20, 5);
  display.print(bmp.readPressure()/100.0F);
  display.setTextSize(1);
  display.print(" hpa");
  display.setCursor(0, 34);
  display.setTextSize(1);
  display.print("Temperature: ");
  display.print(bmp.readTemperature());
  display.cp437(true);
  display.print(" ");
  display.write(167);
  display.print("C");
  display.setCursor(0, 44);
  display.setTextSize(1);
  display.print("Humidity: ");
  display.print(humidity.relative_humidity);
  display.print(" hpa");
  //displayIndicator(displayScreenNum);
  display.display();
 
}

// SCREEN NUMBER 3: LUMINOSITY
void displayLDR(){
  sensors_event_t humidity, temp; //Get Data from AHT20
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data


  display.clearDisplay();
  display.setTextSize(2);
  display.drawBitmap(33, 5, sun_icon, 16, 16 ,1);
  display.setCursor(53, 5);
  int ldrReading = map(analogRead(ldr), 0, 4095, 100, 0);
  display.print(ldrReading);
  display.print(" %");
  display.setTextSize(1);
  display.setCursor(0, 34);
  display.print("Temperature: ");
  display.print(bmp.readTemperature());
  display.print(" ");
  display.cp437(true);
  display.write(167);
  display.print("C");
  display.setCursor(0, 44);
  display.setTextSize(1);
  display.print("Humidity: ");
  display.print(humidity.relative_humidity);
  display.print(" %");
  display.setCursor(0, 44);
  //displayIndicator(displayScreenNum);
  display.display();

}

// Display the right screen accordingly to the displayScreenNum
void updateScreen() {
  
  if (displayScreenNum == 0) {
    displayTemperature();
  }
  else if (displayScreenNum ==1){
    displayHumidity();
  }
  else if (displayScreenNum==2){
    displayPressure();
  }
  else {
    displayLDR();
  }
}

void setup() {

/***************************************************************************
Serial Setup 
 ***************************************************************************/
  Serial.begin(9600);
  while ( !Serial ) delay(100);   // wait for native usb

   // Initialize the pushbutton pin as an input
  pinMode(buttonPin, INPUT);

 /***************************************************************************
OLED Setup 
 ***************************************************************************/
 // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.setTextColor(WHITE);
  delay(1000); // Pause for 1 Sec

/***************************************************************************
AHT20 Setup 
 ***************************************************************************/
    if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
      // display.clearDisplay();// Clear the buffer.
      // display.setCursor(0, 0);
      // display.setTextSize(1);
      // display.print("Could not find AHT? Check wiring");
      // display.display(); //Sends Output to Display
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");
      // display.clearDisplay();// Clear the buffer.
      // display.setCursor(0, 0);
      // display.setTextSize(1);
      // display.print("AHT10 or AHT20 found");
      // display.display(); //Sends Output to Display
      // delay(2000); // Pause for 1 Sec


 /***************************************************************************
 Start BMP280
 ***************************************************************************/
unsigned status;
  status = bmp.begin(0x77); //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
          // display.clearDisplay();// Clear the buffer.
          // display.setTextSize(1);
          // display.setCursor(0, 20);
          // display.print("Could not find a valid BMP280 sensor");
          // display.setCursor(0, 40);
          // display.print("SensorID was: 0x");
          // display.setCursor(97, 40);
          // display.print(bmp.sensorID(),16);
          // display.display(); //Sends Output to Display

    while (1) delay(10);
  }
  Serial.println("BMP280 found");
      // display.setCursor(0, 20);
      // display.setTextSize(1);
      // display.print("BMP280 found");
      // display.display(); //Sends Output to Display
      // delay(2000); // Pause for 1 Sec
      
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */




}

void loop() {

 // read the state of the switch into a local variable
  int reading = digitalRead(buttonPin);


// Change screen when the pushbutton is pressed
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) {
        updateScreen();
        Serial.println(displayScreenNum);
        if(displayScreenNum < displayScreenNumMax) {
          displayScreenNum++;
        }
        else {
          displayScreenNum = 0;
        }
        lastTimer = millis();
      }
    }
  }
  lastButtonState = reading;
  


}
