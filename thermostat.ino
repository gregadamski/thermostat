#include <OneWire.h>

byte type_s;
byte addr[8];
OneWire  ds(D5);  // on pin 10 (a 4.7K resistor is necessary)

#define H1PIN D6
#define H2PIN D7

// display initialization
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

float targetTemp = 70.0;
float histeresis = 1.0;
float heating = false;

void setup() 
{
  Serial.begin(115200);

  // initialize the temp sensor
  pinMode(D5, INPUT_PULLUP);     // Initialize the LED_BUILTIN pin as an output
  delay(250);
  initTempSensor();

  // heater relay init
  //initHeaterPins();
  initDisplay();
}

void loop() {
  // put your main code here, to run repeatedly:
  float temp;
  temp = measureTemp();
  Serial.println(temp);
  
  boolean heater1 = false;
  boolean heater2 = false;

  String heater1StatusStr = String("H1: ") + String(heater1) + String("; H2: ") + String(heater2);
  String tempStatusStr = String("Temp: ") +  String(temp);
  printStatus(heater1StatusStr, tempStatusStr, F(""));

  delay(1000);
  
}

void initHeaterPins()
{
    pinMode(H1PIN, OUTPUT);
    pinMode(H2PIN, OUTPUT);
}


void setHeaterState(int heater, boolean enabled)
{

  int heaterState = enabled ? HIGH : LOW;
  int heaterPin;
  if (heater == 1) {
    heaterPin = H1PIN;    
  } else
  if (heater == 2) {
    heaterPin = H1PIN;    
  } else {
    return;
  }

  digitalWrite(heaterPin, heaterState);
}


void initDisplay()
{
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();

}

void printStatus(String line1, String line2, String line3) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner


  display.setCursor(0,0);             // Start at top-left corner
  display.println(line1);
  display.println(line2);
  display.println(line3);

  display.display();
}


void initTempSensor()
{
  byte i;
  byte present = 0;
  byte data[12];
  float celsius, fahrenheit;
  
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
}


float measureTemp()
{

  byte i;
  byte present = 0;
  byte data[12];
  float celsius;

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  return celsius;
}
