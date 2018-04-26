#define THERMISTER_PIN A0
#define THERMISTER_REF_RESISTER 9960.0
#define ADC_MAX 1023


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

	Serial.print("Resistance: ");
	Serial.print(getResistance(THERMISTER_REF_RESISTER, analogRead(THERMISTER_PIN)));
	Serial.print(" Ohms");

	Serial.print(" (");
	Serial.print(analogRead(THERMISTER_PIN));
	Serial.println(")");
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