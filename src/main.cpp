#include <Arduino.h>
//#define VERBOSE

#define STATIC_TIMER_UPDATE timer = millis() + duration
#define STATIC_TIMER_INIT static unsigned long STATIC_TIMER_UPDATE
#define STATIC_TIMER_CHECK (millis() >= timer)

#define THERMISTER_PIN A0
#define THERMISTER_REF_RESISTER 9960.0
#define ADC_MAX 1023

#define TEMP_KELVIN_DIFF 273.15				// Constant Difference between Kelvin to Celcius
#define THERMISTER_ROOM_RESISTANCE 10000    // Resistance @ Norminal  
#define THERMISTER_ROOM_TEMP 25				// Temp @ Norminal
#define THERMISTER_COEF_B 2988.64			// Beta Coefficient of thermistor (usually 3000-4000)
											// 2988.64 == getBetaCoef(62, 3324.33, 5.5, 20274.66)

double resistance = 10000.0;
double celcius = 25.0;

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

void task_Temperature(int duration);

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(115200);
	
	initController();
	setController(100);
}

void loop()
{
	task_Temperature(500);
	task_Temperature(500);
/*
	Serial.print(celcius);
	Serial.print(" ");
	Serial.print(38);
	Serial.println();
	blink_blocking(LED_BUILTIN, 5);
	*/
}

void task_Temperature(int duration)
{
	STATIC_TIMER_INIT;
	if (STATIC_TIMER_CHECK) {
		resistance = getResistance(THERMISTER_REF_RESISTER, analogRead(THERMISTER_PIN));
		celcius = toCelcius(resistance);
		Serial.println("Task");
		STATIC_TIMER_UPDATE;
	}
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
	value += 1.0 / (THERMISTER_ROOM_TEMP + TEMP_KELVIN_DIFF);		// + (1/To)
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
	#ifdef VERBOSE
		Serial.println("Init Controller");
	#endif
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
	#ifdef VERBOSE
		Serial.print("setController: ");
		Serial.print(percentage);
		Serial.print("% -> PWM : ");
		Serial.println(pwm);
	#endif
	if (pwm >= 0 && pwm <= CONTROLLER_PWM_MAX)
	{
		analogWrite(CONTROLLER_PWM_PIN, pwm);
		return true;
	}
	else {
		return false;
	}
}

