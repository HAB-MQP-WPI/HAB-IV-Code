// ETS Functions

void checkETS() {
  int value = analogRead(ETS_IN);
  float V_ETS_IN = value * 5.0/1024.0; // Take ADC reading and convert to actual voltage value
  if(V_ETS_IN > 3.0){ // Pi will be sending 3.3v logic
    digitalWrite(ETS_OUT, HIGH); // Set ETS_OUT high - same logic as Pi but shifted up to 5v
  }else{
    digitalWrite(ETS_OUT, LOW); // Set ETS_OUT low
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////

// BMP180 Sensor Functions

void measureBMPData() {
  // Read the BMP Sensor Values - from HAB_ALT_Temp_Test
  Serial.print("BMP180 Temp (*C): ");
  Serial.println(bmp.readTemperature());

  Serial.print("BMP180 Pressure (Pa): ");
  Serial.println(bmp.readPressure());

  Serial.print("BMP180 Alt (m): ");
  Serial.println(bmp.readAltitude());
}

/////////////////////////////////////////////////////////////////////////////////////////////

// CO2 Sensor Functions

void measureCO2Data() { // Read the CO2 sensor values - from HAB_CO2_Test

  volts = MGRead(CO2_PIN); // Reads the output of CO2 sensor
  percentage = MGGetPercentage(volts, CO2Curve); // Converts output of CO2 sensor to a ppm value
  Serial.print("CO2 (ppm): ");
  if (percentage == -1) {
    Serial.println( "400" ); // Default value of 400ppm if less is read
  } else {
    Serial.println(percentage);
  }
}

float MGRead(int mg_pin) // Reads the output of CO2 sensor
{
  int i;
  float v = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    v += analogRead(mg_pin);
    delay(READ_SAMPLE_INTERVAL);
  }
  v = (v / READ_SAMPLE_TIMES) * 5 / 1024 ;
  return v;
}

int  MGGetPercentage(float volts, float *pcurve) //Converts output of CO2 sensor to a ppm value
{
  if ((volts / DC_GAIN ) >= ZERO_POINT_VOLTAGE) {
    return -1;
  } else {
    return pow(10, ((volts / DC_GAIN) - pcurve[1]) / pcurve[2] + pcurve[0]);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////

// O3 Sensor Functions

void measureO3Data() { // Read the Ozone Sensor Values - from HAB_O3_Test

  int16_t ozoneConcentration = Ozone.readOzoneData(COLLECT_NUMBER);
  Serial.print("OZ (ppb): ");
  Serial.println(ozoneConcentration);
}

/////////////////////////////////////////////////////////////////////////////////////////////

// CH4 Sensor Functions

void measureCH4Data() { // Read the Methane Sensor Values - from HAB_CH4_Test

  double V_RL = analogRead(CH4_PIN) * 5/1024; // Read analog input pin 1 and convert to voltage (5v / 1024 ADC levels)
  double ppm = 10.938*exp(1.7742*V_RL); // Converts voltage read across 4.7k load to PPM
  Serial.print("CH4 (ppm): ");
  Serial.println(ppm);  // Prints the value read in PPM
}

/////////////////////////////////////////////////////////////////////////////////////////////

// UV Sensor Functions

void measureUVData() { // Read the UV Sensor Values - from HAB_UV_Test
  Serial.print("UV Index: ");
  Serial.println(String(uv.index()));  
}

/////////////////////////////////////////////////////////////////////////////////////////////

// One-Wire Temperature Sensor Functions

void measureOneWireTempData() { // Read the One-Wire Temp values - from HAB_OneWireTemp_Test

  float temperature = getTemp();

  Serial.print("One-Wire Temp (*F): ");
  Serial.println(temperature);
}

float getTemp() { // returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !OneWireTemp.search(addr)) {
    //no more sensors on chain, reset search
    OneWireTemp.reset_search();
    return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }

  OneWireTemp.reset();
  OneWireTemp.select(addr);
  OneWireTemp.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = OneWireTemp.reset();
  OneWireTemp.select(addr);
  OneWireTemp.write(0xBE); // Read Scratchpad

  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = OneWireTemp.read();
  }

  OneWireTemp.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); // using two's compliment
  float TemperatureSum = tempRead / 16;

  return (TemperatureSum * 18 + 5) / 10 + 32;
}

/////////////////////////////////////////////////////////////////////////////////////////////

// Red Pressure Sensor Functions

void measureRedPressureData() { // Read the Red Pressure Sensor - from HAB_RedPressure_Test

  temperature_c = RedPressure.getTemperature(CELSIUS, ADC_512); // Read temperature from the sensor in deg C.
  temperature_f = RedPressure.getTemperature(FAHRENHEIT, ADC_512); // Read temperature from the sensor in deg F.
  // Converting to Fahrenheit is not internal to the sensor. Additional math is done to convert a Celsius reading.

  double pressure_abs = RedPressure.getPressure(ADC_4096);
  double currAlt = altitude(pressure_abs, sealevel(pressure_baseline, base_altitude)); // 1013.25 mBar is sea level pressure
  altitude_delta = altitude(pressure_abs , pressure_baseline);
  // ^Taking our baseline pressure at the beginning we can find an approximate change in altitude based on the differences in pressure.

  // Report values via I2C
  Serial.print("Red Pressure Temp (*C): ");
  Serial.println(temperature_c);

  Serial.print("Red Pressure Temp (*F): ");
  Serial.println(temperature_f);

  Serial.print("Red Pressure (mb): ");
  Serial.println(pressure_abs);

  Serial.print("Red Alt (m): ");
  Serial.println(currAlt);

  Serial.print("Red Alt Change from Startup (m): ");
  Serial.println(altitude_delta);
}

double sealevel(double P, double A) // Used with Red Pressure Sensor
// Given a pressure P (mbar) taken at a specific altitude (meters),
// return the equivalent pressure (mbar) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
  return (P / pow(1 - (A / 44330.0), 5.255));
}


double altitude(double P, double P0)// Used with Red Pressure Sensor
// Given a pressure measurement P (mbar) and the pressure at a baseline P0 (mbar),
// return altitude (meters) above baseline.
{
  return (44330.0 * (1 - pow(P / P0, 1 / 5.255)));
}

////////////////////////////////////////////////////////////////////////////////////////////
