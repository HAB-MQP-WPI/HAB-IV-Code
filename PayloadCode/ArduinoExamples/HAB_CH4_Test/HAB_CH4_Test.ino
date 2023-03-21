// Source: https://learn.sparkfun.com/tutorials/hazardous-gas-monitor

void setup()
{
  Serial.begin(9600);      // Sets the serial port to 9600
}

void loop()
{
  double V_RL = analogRead(1) * 5/1024; // Read analog input pin 1 and convert to voltage (5v / 1024 ADC levels)
  double ppm = 10.938*exp(1.7742*V_RL); //Converts voltage read across 4.7k load to PPM
  Serial.print("CH4: ");
  Serial.print(ppm);  // Prints the value read in PPM
  Serial.println(" ppm");
  delay(100);                        // wait 100ms for next reading
}
