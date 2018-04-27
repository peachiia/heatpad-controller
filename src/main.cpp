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

double rawdata = 10000.0;
double resistance = 10000.0;
double celcius = 25.0;

#define CONTROLLER_PWM_MAX 255
#define CONTROLLER_PWM_PIN 6
#define CONTROLLER_INA_PIN 7
#define CONTROLLER_INB_PIN 8

#define THERMISTER_DENOISE_ORDER 10

void blink_blocking(int pin, int duration);
double getResistance(double Rref, int ADCval);
double getDenoisedResistance(double resistance);
double toCelcius(double resistance);
double getBetaCoef(double T1, double R1, double T2, double R2);
void printTemp();

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
	task_Temperature(10);
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
		rawdata = getResistance(THERMISTER_REF_RESISTER, analogRead(THERMISTER_PIN));
		resistance = getDenoisedResistance(rawdata);
		celcius = toCelcius(resistance);
		printTemp();
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

double getDenoisedResistance(double resistance)
{
	static double buffer[THERMISTER_DENOISE_ORDER];
	static int i,n;

	buffer[n] = resistance;
	n = (n + 1) % THERMISTER_DENOISE_ORDER;

	resistance = 0;
	for (i = 0; i < THERMISTER_DENOISE_ORDER; i++) {
		resistance += buffer[i];
	}
	return resistance / THERMISTER_DENOISE_ORDER;
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

void printTemp()
{
	Serial.print(toCelcius(rawdata));
	Serial.print(" ");
	Serial.print(celcius);
	Serial.print(" ");
	Serial.println(38);
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

