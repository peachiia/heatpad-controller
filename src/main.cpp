#include <Arduino.h>
//#define VERBOSE

#pragma region TASK_TIMER
	#define STATIC_TIMER_UPDATE timer = millis() + duration					// Reschedule for next calling
	#define STATIC_TIMER_INIT static unsigned long STATIC_TIMER_UPDATE		// Schedule Variable Declaration
	#define STATIC_TIMER_CHECK (millis() >= timer)							// Schedule Checking
#pragma endregion

#pragma region THERMISTER
	#define THERMISTER_PIN A0
	#define THERMISTER_REF_RESISTER 9960.0
	#define ADC_MAX 1023

	#define TEMP_KELVIN_DIFF 273.15				// Constant Difference between Kelvin to Celcius
	#define THERMISTER_ROOM_RESISTANCE 10000    // Resistance @ Norminal  
	#define THERMISTER_ROOM_TEMP 25				// Temp @ Norminal
	#define THERMISTER_COEF_B 2988.64			// Beta Coefficient of thermistor (usually 3000-4000)
												// 2988.64 == getBetaCoef(62, 3324.33, 5.5, 20274.66)

	double resistance = 10000.0;
	double noisyTemp = 25.0;
	double denoisedTemp = 25.0;

	#define DENOISE_ORDER 1
	#define DENOISE_DIFF_LIMIT 0.02

	double getResistance(double Rref, int ADCval);
	double toCelcius(double resistance);
	double getBetaCoef(double T1, double R1, double T2, double R2);
	double getDenoisedData(double data);
#pragma endregion

#pragma region CONTROLLER
	#define CONTROLLER_PWM_MAX 255
	#define CONTROLLER_PWM_PIN 6
	#define CONTROLLER_INA_PIN 7
	#define CONTROLLER_INB_PIN 8

	void initController();
	bool setController(double percentage);
#pragma endregion

#pragma region PID
	double kP = 500; // 200
	double kI = 0; // 2
	double kD = 0; //5
	double valP, valI, valD, valPID;

	double currentTemp = 25.0;
	double previousTemp = 25.0;
	double setpointTemp = 40.0;
	double errorTemp = 0;

	double deltaTime = 0;
	double controlPWM = 0;
#pragma endregion

void task_Temperature(long duration);
void task_PID(long duration);
void task_Controller(long duration);
void task_Plot(long duration);
void task_Terminal(long duration);

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(115200);
	
	initController();
	Serial.print(">> ");
}

void loop()
{
	//task_Temperature(10);
	//task_PID(50);
	//task_Controller(50);
	//task_Plot(20);

	task_Terminal(50);
}


void task_Temperature(long duration)
{
	STATIC_TIMER_INIT;
	if (STATIC_TIMER_CHECK) {
		resistance = getResistance(THERMISTER_REF_RESISTER, analogRead(THERMISTER_PIN));
		noisyTemp = toCelcius(resistance);
		denoisedTemp = getDenoisedData(noisyTemp);
		
		STATIC_TIMER_UPDATE;
	}
}


void task_PID(long duration)
{
	STATIC_TIMER_INIT;
	if (STATIC_TIMER_CHECK) {
		double previousTime = millis();
		currentTemp = denoisedTemp;
		errorTemp = setpointTemp - currentTemp;
		deltaTime = (millis() - previousTime) / 1000;
		previousTime = millis();

		valP = errorTemp * kP;
		valI = valI + ((errorTemp * kI) * deltaTime);
		valD = (previousTemp - currentTemp) * kD;
		valPID = valP + valI + valD;

		STATIC_TIMER_UPDATE;
	}
}


void task_Controller(long duration)
{
	static double percentage;
	STATIC_TIMER_INIT;
	if (STATIC_TIMER_CHECK) {
		percentage = valPID;
		if (percentage < 0) {
			percentage = 0;
		}
		else if (percentage > 100) {
			percentage = 100;
		}
		setController(percentage);

		STATIC_TIMER_UPDATE;
	}
}


void task_Plot(long duration)
{
	STATIC_TIMER_INIT;
	if (STATIC_TIMER_CHECK) {
		// Serial.print(noisyTemp);
		// Serial.print(" ");
		Serial.print(denoisedTemp);
		Serial.print(" ");
		Serial.print(setpointTemp);
		// Serial.print(" // ");
		// Serial.print(valPID);

		Serial.println();
		STATIC_TIMER_UPDATE;
	}

	if (abs(currentTemp-setpointTemp) < 0.5) {
		digitalWrite(LED_BUILTIN, HIGH);
	}
	else {
		digitalWrite(LED_BUILTIN, LOW);
	}
}


double getResistance(double Rref, int ADCval)
{
	// For Normal Measuring Circuit
	// return Rref / ((ADC_MAX/ (double)ADCval) - 1);

	// For Reverse Measuring Circuit
	return Rref * ((ADC_MAX / ((double)ADCval)) - 1); 
}


double getDenoisedData(double data)
{
	static double buffer = data;
	static double prev = data;

	if((data - prev) > DENOISE_DIFF_LIMIT) {
		prev += DENOISE_DIFF_LIMIT;
		data = prev ;
	}
	else if ((prev - data) > DENOISE_DIFF_LIMIT) {
		prev -= DENOISE_DIFF_LIMIT;
		data = prev;
	} 
	else {
		prev = data;
	}
	buffer = (data + (buffer * (DENOISE_ORDER - 1))) / DENOISE_ORDER;
	return buffer;
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
	if (pwm >= 0 && pwm <= CONTROLLER_PWM_MAX) {
		analogWrite(CONTROLLER_PWM_PIN, pwm);
		return true;
	}
	else {
		return false;
	}
}

#define COMMAND_MAX_LENGTH 50
char bufferString[COMMAND_MAX_LENGTH];
int bufferLength = 0;
char bufferChar;

void task_Terminal(long duration)
{
	STATIC_TIMER_INIT;
	if (STATIC_TIMER_CHECK) {
		while (Serial.available() > 0) // TODO: Watchdog Timer Required.
		{
			bufferChar = (char)Serial.read();

			if (bufferChar == '\n') {
				if (bufferLength > 0) {
					// Exec();
					bufferLength = 0;
				} 
				Serial.println();
				Serial.print(">> ");
			}
			else if (bufferChar == '\b' || bufferChar == 127) {  // backspace for windows, putty
				if (bufferLength >= 1) {
					bufferLength--;
					Serial.print(bufferChar);
				}
				else {
					bufferLength = 0;
				}
			}
			else if (bufferChar== '\r') { // ignore

			}
			else {
				Serial.print(bufferChar);
				bufferString[bufferLength++] = bufferChar;
			}
		}
		STATIC_TIMER_UPDATE;
	}
}