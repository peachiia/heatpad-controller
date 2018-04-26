#define THERMISTER_PIN A0
#define THERMISTER_REF_RESISTER 9960.0
#define ADC_MAX 1023

#define THERMISTER_ROOM_RESISTANCE 10000    // Resistance @ Norminal  
#define THERMISTER_ROOM_TEMP 25				// Temp @ Norminal
#define THERMISTER_COEF_B 3950				// Beta Coefficient of thermistor (usually 3000-4000)

void setup()
{
	Serial.begin(115200);
	pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
	blink_blocking(LED_BUILTIN, 100);
	blink_blocking(LED_BUILTIN, 100);
	blink_blocking(LED_BUILTIN, 100);

	double resistance = getResistance(THERMISTER_REF_RESISTER, analogRead(THERMISTER_PIN));
	double celcius = toCelcius(resistance);

	Serial.print("Resistance: ");
	Serial.print(resistance);
	Serial.print(" Ohms");

	Serial.print(" (");
	Serial.print(analogRead(THERMISTER_PIN));
	Serial.print(") -> ");

	Serial.print(celcius);
	Serial.println(" C");

	blink_blocking(LED_BUILTIN, 400);
}

void blink_blocking(int pin, int duration)
{
	digitalWrite(pin, HIGH);
	delay(duration);
	digitalWrite(pin, LOW);
	delay(duration);
}

double getResistance(double Rref, int ADCval)
{
	// For Normal Measuring Circuit
	// return Rref / ((ADC_MAX/ (double)ADCval) - 1);

	// For Reverse Measuring Circuit
	return Rref * ((ADC_MAX / ((double)ADCval)) - 1); 
}

double toCelcius(double resistance)
{
	// Steinhart–Hart eq. (NTC - Beta method)
	double value = 0;
	value = resistance / THERMISTER_ROOM_RESISTANCE;    // (R/Ro)
	value = log(value);									// ln(R/Ro)
	value /= THERMISTER_COEF_B;							// 1/B * ln(R/Ro)
	value += 1.0 / (THERMISTER_ROOM_TEMP + 273.15);		// + (1/To)
	value = 1.0 / value;								// Invert
	value -= 273.15;									// Convert to C
	return value;
}