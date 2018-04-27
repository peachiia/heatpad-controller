#include <Arduino.h>

#define THERMISTER_PIN A0
#define THERMISTER_REF_RESISTER 9960.0
#define ADC_MAX 1023

#define THERMISTER_ROOM_RESISTANCE 10000    // Resistance @ Norminal  
#define THERMISTER_ROOM_TEMP 25				// Temp @ Norminal
#define THERMISTER_COEF_B 2988.64			// Beta Coefficient of thermistor (usually 3000-4000)
											// 2988.64 == getBetaCoef(62, 3324.33, 5.5, 20274.66)

#define TEMP_KELVIN_DIFF 273.15				// Constant Difference between Kelvin to Celcius

#define CONTROLLER_PWM_MAX 255
#define CONTROLLER_PWM_PIN 6
#define CONTROLLER_INA_PIN 7
#define CONTROLLER_INB_PIN 8


void blink_blocking(int pin, int duration);
double getResistance(double Rref, int ADCval);
double toCelcius(double resistance);
double getBetaCoef(double T1, double R1, double T2, double R2);
void initController();
bool setController(double percentage);

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(115200);
	
	initController();
	setController(100);
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
	// Steinhart-Hart eq. (NTC - Beta method)
	double value = 0;
	value = resistance / THERMISTER_ROOM_RESISTANCE;    // (R/Ro)
	value = log(value);									// ln(R/Ro)
	value /= THERMISTER_COEF_B;							// 1/B * ln(R/Ro)
	value += 1.0 / (THERMISTER_ROOM_TEMP + 273.15);		// + (1/To)
	value = 1.0 / value;								// Invert
	value -= TEMP_KELVIN_DIFF;							// Convert to C
	return value;
}

double getBetaCoef(double T1, double R1, double T2, double R2)
{
	// For NTC Thermistor. See "Steinhart-Hart" for more informations.
	
	// Convert T1 and T2 to Kelvin
	T1 += TEMP_KELVIN_DIFF;
	T2 += TEMP_KELVIN_DIFF;

	return log(R1/R2) / ((1/T1) - (1/T2));
}

void initController()
{
	Serial.println("Init Controller");
	pinMode(CONTROLLER_PWM_PIN, OUTPUT);
	pinMode(CONTROLLER_INA_PIN, OUTPUT);
	pinMode(CONTROLLER_INB_PIN, OUTPUT);

	digitalWrite(CONTROLLER_INA_PIN, HIGH);
	digitalWrite(CONTROLLER_INB_PIN, LOW);
	analogWrite(CONTROLLER_PWM_PIN, 0);
}

bool setController(double percentage)
{
	int pwm = (int)(CONTROLLER_PWM_MAX * (percentage/100));
	Serial.print("setController: ");
	Serial.print(percentage);
	Serial.print("% -> PWM : ");
	Serial.println(pwm);
	if (pwm >= 0 && pwm <= CONTROLLER_PWM_MAX)
	{
		analogWrite(CONTROLLER_PWM_PIN, pwm);
		return true;
	}
	else {
		return false;
	}
}

