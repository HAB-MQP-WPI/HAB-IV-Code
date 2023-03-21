// Libraries
#include <Wire.h> // Needed for many of the included sensors
#include <Adafruit_BMP085.h> // Library used with Adafruit BMP180 Altitude and Pressure sensor
#include "DFRobot_OzoneSensor.h" // Library for the DFRobot Ozone Sensor
#include <OneWire.h> // Library used with One-Wire Temp Sensor
#include <SoftwareSerial.h> // Library used with One-Wire Temp Sensor
#include <SparkFun_MS5803_I2C.h> // Library used with Red Pressure Sensor
#include <SparkFun_VEML6075_Arduino_Library.h> // Library used with SparkFun UV Sensor

/////////////////////////////////////////////////////////////////////////////////////////////

double base_altitude = 165; // !!!VERY IMPORTANT!!! - Altitude of where payload will be turned on (will change depending on launch location) 

/////////////////////////////////////////////////////////////////////////////////////////////

// Sensor Pins
#define ETS_IN (A2) // ETS Input from Raspberry Pi (A2)
#define ETS_OUT 3 // ETS Output to Power MOSFET (Level Shifted From Input) (D0)
#define UV_PIN (A0) // Data pin of the UV sensor (A0)
#define CH4_PIN (A1) // Data pin of the CH4 sensor (A1)
#define CO2_PIN (A3) // Data pin of the CO2 sensor (A3)
#define DS18S20_Pin 2 // Data pin of the One-Wire Temperature Sensor (D2)

/////////////////////////////////////////////////////////////////////////////////////////////

// Globals

// CO2 Sensor Globals
#define BOOL_PIN 2 // Used in CO2 initialization
#define DC_GAIN 8.5 // Defines the DC gain of amplifier
#define READ_SAMPLE_INTERVAL 50 // CO2 sample amount
#define READ_SAMPLE_TIMES 5 // Defines the time interval(in milisecond) between each samples
// These two values differ from sensor to sensor. user should determine this value. - !!!!!!NOT CALIBRATED YET!!!!!!!
#define ZERO_POINT_VOLTAGE 0.220 // Defines the output of the sensor in volts when the concentration of CO2 is 400PPM
#define REACTION_VOLTAGE 0.030 // Defines the voltage drop of the sensor when move the sensor from air into 1000ppm CO2
int percentage;
float CO2Curve[3] = {2.602, ZERO_POINT_VOLTAGE, (REACTION_VOLTAGE / (2.602 - 3))}; // Used in converting CO2 sensor output to ppm value
float volts;

// O3 Sensor Globals
#define COLLECT_NUMBER   20 // Collect number for the ozone sensor, the collection range is 1-100
#define Ozone_IICAddress OZONE_ADDRESS_3 // I2C address of ozone sensor - 0x73

// Red Pressure Sensor Globals
float temperature_c, temperature_f; // Stores last measured temperature values
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline; // Stores last measured pressure values

/////////////////////////////////////////////////////////////////////////////////////////////

// Sensor I/O Setup
Adafruit_BMP085 bmp; // Set up BMP object for BMP180 sensor
DFRobot_OzoneSensor Ozone; // Set up object for O3 sensor
OneWire OneWireTemp(DS18S20_Pin);  // on digital pin 2
MS5803 RedPressure(ADDRESS_HIGH); // Set up object for Red Pressure Sensor ADDRESS_HIGH = 0x76
VEML6075 uv; // Create a VEML6075 object

/////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(9600); // Initialize serial monitor at 9600 baud rate

  // Initialize ETS I/O
  pinMode(ETS_IN, INPUT); // Set ETS_OUT as Output
  pinMode(ETS_OUT, OUTPUT); // Set ETS_OUT as Output
  digitalWrite(ETS_OUT, LOW); // Initialize ETS_OUT low

  //  Intialize the BMP sensor - from HAB_Alt_Temp_Test
  while (!bmp.begin()) { // Program will not start in the event that the BMP180 is not found
    Serial.println("I2C BMP device number error!");
    delay(1000);
  }
  Serial.println("I2C BMP connect success !");

  // Intialize the CO2 Sensor - from HAB_CO2_Test
//  pinMode(BOOL_PIN, INPUT); // Set pin to input
//  digitalWrite(BOOL_PIN, HIGH); // Turn on pullup resistors

  // Intialize the O3 Sensor - from HAB_O3_Test
  while (!Ozone.begin(Ozone_IICAddress)) { // Program will not start in the event that the O3 sensor is not found
    Serial.println("I2C O3 device number error!");
    delay(1000);
  }
  Serial.println("I2C O3 connect success !");
  Ozone.setModes(MEASURE_MODE_PASSIVE); // Set ozone sensor to passive mode

  // the VEML6075's begin function can take no parameters
  // It will return true on success or false on failure to communicate
  while (!uv.begin())
  {
    Serial.println("I2C UV device number error!");
    delay(1000);
  }
    Serial.println("I2C UV connect success !");

  // Initialize the Red Pressure Sensor
  Wire.begin(); // Start your preferred I2C object
  RedPressure.reset(); //Retrieve calibration constants for conversion math.
  RedPressure.begin();

  pressure_baseline = RedPressure.getPressure(ADC_4096); // Take a base measurement to compare future measurements to

  Serial.println(); // Spacer line before first packet starts
}

/////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  checkETS();
  Serial.println("ARDUINO SENSOR PACKET"); // Extra spacer line between measurements
  measureBMPData();
  measureCO2Data();
  measureO3Data();
  measureCH4Data();
  measureUVData();
  measureOneWireTempData();
  measureRedPressureData();

  Serial.println(); // Extra spacer line between measurements
  delay(1000); // Delay measurements by 1s
}
