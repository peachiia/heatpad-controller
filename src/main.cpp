#include <Arduino.h>
#include <EEPROM.h>
#include <stdio.h>
//#define VERBOSE

struct profile_storage
{
	uint8_t THERMISTER_PIN = 0;
	double THERMISTER_REF_RESISTER = 9960.0;
	int ADC_MAX = 1023;

	double THERMISTER_ROOM_RESISTANCE = 10000;    	// Resistance @ Norminal  
	double THERMISTER_ROOM_TEMP = 25;				// Temp @ Norminal
	double THERMISTER_COEF_B = 2988.64;				// Beta Coefficient of thermistor
													// 2988.64 == getBetaCoef(62, 3324.33, 5.5, 20274.66)

	int DENOISE_ORDER = 1;
	double DENOISE_DIFF_LIMIT = 0.02;

	int CONTROLLER_PWM_MAX = 255;
	uint8_t CONTROLLER_PWM_PIN = 6;
	uint8_t CONTROLLER_INA_PIN = 7;
	uint8_t CONTROLLER_INB_PIN = 8;

	double kP = 500.0; 
	double kI = 0;
	double kD = 0;

	double setpointTemp = 40.0;
} profile;

#pragma region TASK
	#define STATIC_TIMER_UPDATE timer = millis() + duration					// Reschedule for next calling
	#define STATIC_TIMER_INIT static unsigned long STATIC_TIMER_UPDATE		// Schedule Variable Declaration
	#define STATIC_TIMER_CHECK (millis() >= timer)							// Schedule Checking

	void task_run();
	void task_stop();
	bool task_isRunning();

	bool isControlTaskEnabled = false;
#pragma endregion

#pragma region THERMISTER
	#define TEMP_KELVIN_DIFF 273.15				// Constant Difference between Kelvin to Celcius

	double resistance = 10000.0;
	double noisyTemp = 25.0;
	double denoisedTemp = 25.0;
	double currentTemp = 25.0;

	double getResistance(double Rref, int ADCval);
	double toCelcius(double resistance);
	double getBetaCoef(double T1, double R1, double T2, double R2);
	double getDenoisedData(double data);
#pragma endregion

#pragma region CONTROLLER
	void initController();
	bool setController(double percentage);
#pragma endregion

#pragma region PID
	double valP, valI, valD, valPID;

	double previousTemp = 25.0;
	double errorTemp = 0;

	double deltaTime = 0;
	double controlPWM = 0;
#pragma endregion

#pragma region TERMINAL
	#define COMMAND_MAX_LENGTH 50
	char bufferString[COMMAND_MAX_LENGTH];
	int bufferLength = 0;
	char bufferChar;

	char command[COMMAND_MAX_LENGTH];
	int commandLength = 0;
	bool flagExecRequest = false;
	bool flagTerminalResume = true;

	void init_Terminal();
#pragma endregion


void task_Temperature(long duration);
void task_PID(long duration);
void task_Controller(long duration);
void task_Plot(long duration);
void task_Terminal(long duration);
void task_Exec();

bool isMatch(char *p,  char *keyword);
void command_help();
void command_info();
void command_run();
void command_stop();
void command_default();
void command_show();
void command_load();
void command_save();
void command_set();

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);

	init_Terminal();
	task_run();
}

void loop()
{
	if (task_isRunning())
	{
		task_Temperature(10);
		task_PID(50);
		task_Controller(50);
		task_Plot(20);
	}
	task_Terminal(50);
	task_Exec();
}


void init_Temperature()
{
	resistance = 10000.0;
	noisyTemp = 25.0;
	denoisedTemp = 25.0;
	currentTemp = 25.0;
}


void task_Temperature(long duration)
{
	STATIC_TIMER_INIT;
	if (STATIC_TIMER_CHECK) {
		resistance = getResistance(profile.THERMISTER_REF_RESISTER, analogRead(profile.THERMISTER_PIN));
		noisyTemp = toCelcius(resistance);
		denoisedTemp = getDenoisedData(noisyTemp);
		currentTemp = denoisedTemp;
		STATIC_TIMER_UPDATE;
	}
}


void init_PID()
{
	valPID = 0;

	previousTemp = 25.0;
	errorTemp = 0;

	deltaTime = 0;
	controlPWM = 0;
}


void task_PID(long duration)
{
	STATIC_TIMER_INIT;
	if (STATIC_TIMER_CHECK) {
		double previousTime = millis();
		errorTemp = profile.setpointTemp - currentTemp;
		deltaTime = (millis() - previousTime) / 1000;
		previousTime = millis();

		valP = errorTemp * profile.kP;
		valI = valI + ((errorTemp * profile.kI) * deltaTime);
		valD = (previousTemp - currentTemp) * profile.kD;
		valPID = valP + valI + valD;

		STATIC_TIMER_UPDATE;
	}
}


void init_Controller()
{
	#ifdef VERBOSE
		Serial.println("Init Controller");
	#endif
	pinMode(profile.CONTROLLER_PWM_PIN, OUTPUT);
	pinMode(profile.CONTROLLER_INA_PIN, OUTPUT);
	pinMode(profile.CONTROLLER_INB_PIN, OUTPUT);

	digitalWrite(profile.CONTROLLER_INA_PIN, HIGH);
	digitalWrite(profile.CONTROLLER_INB_PIN, LOW);
	analogWrite(profile.CONTROLLER_PWM_PIN, 0);

	setController(0);
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


void init_Plot()
{

}


void task_Plot(long duration)
{
	STATIC_TIMER_INIT;
	if (STATIC_TIMER_CHECK) {
		// Serial.print(noisyTemp);
		// Serial.print(" ");
		Serial.print(denoisedTemp);
		Serial.print(" ");
		Serial.print(profile.setpointTemp);
		// Serial.print(" // ");
		// Serial.print(valPID);

		Serial.println();
		STATIC_TIMER_UPDATE;
	}

	if (abs(currentTemp-profile.setpointTemp) < 0.5) {
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
	return Rref * ((profile.ADC_MAX / ((double)ADCval)) - 1); 
}


double getDenoisedData(double data)
{
	static double buffer = data;
	static double prev = data;

	if((data - prev) > profile.DENOISE_DIFF_LIMIT) {
		prev += profile.DENOISE_DIFF_LIMIT;
		data = prev ;
	}
	else if ((prev - data) > profile.DENOISE_DIFF_LIMIT) {
		prev -= profile.DENOISE_DIFF_LIMIT;
		data = prev;
	} 
	else {
		prev = data;
	}
	buffer = (data + (buffer * (profile.DENOISE_ORDER - 1))) / profile.DENOISE_ORDER;
	return buffer;
}


double toCelcius(double resistance)
{
	// Steinhart-Hart eq. (NTC - Beta method)
	double value = 0;
	value = resistance / profile.THERMISTER_ROOM_RESISTANCE;    // (R/Ro)
	value = log(value);									// ln(R/Ro)
	value /= profile.THERMISTER_COEF_B;							// 1/B * ln(R/Ro)
	value += 1.0 / (profile.THERMISTER_ROOM_TEMP + TEMP_KELVIN_DIFF);		// + (1/To)
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


bool setController(double percentage)
{
	int pwm = (int)(profile.CONTROLLER_PWM_MAX * (percentage/100));
	#ifdef VERBOSE
		Serial.print("setController: ");
		Serial.print(percentage);
		Serial.print("% -> PWM : ");
		Serial.println(pwm);
	#endif
	if (pwm >= 0 && pwm <= profile.CONTROLLER_PWM_MAX) {
		analogWrite(profile.CONTROLLER_PWM_PIN, pwm);
		return true;
	}
	else {
		return false;
	}
}


void init_Terminal()
{
	Serial.begin(115200);
}


void task_Terminal(long duration)
{
	STATIC_TIMER_INIT;
	if (STATIC_TIMER_CHECK) {
		if (flagTerminalResume) {
			Serial.print(F(">> "));
			flagTerminalResume = false;
		}					
		while (Serial.available() > 0) // TODO: Watchdog Timer Required.
		{
			if (!flagExecRequest) {
				bufferChar = (char)Serial.read();
				if (bufferChar == '\n' || bufferChar== '\r') {
					if (bufferLength > 0) {
						bufferString[bufferLength] = '\0';
						memcpy(command, bufferString, bufferLength + 1);
						commandLength = bufferLength;
						bufferLength = 0;
						flagExecRequest = true;
						Serial.println();
						break;
					} 
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
				else if (bufferChar == ' ') {
					if ((bufferLength > 0) && (bufferString[bufferLength-1] != ' ')) {
						Serial.print(bufferChar);
						bufferString[bufferLength++] = bufferChar;
					}
				}
				else {
					Serial.print(bufferChar);
					bufferString[bufferLength++] = bufferChar;
				}
			}
		}
		STATIC_TIMER_UPDATE;
	}
}


void task_Exec()
{
	if (flagExecRequest) {
		#ifdef VERBOSE
			Serial.print("EXEC: ");
			Serial.println(command);
		#endif

		if (isMatch(command, "help")) {
			command_help();
		}
		else if (isMatch(command, "info")) {
			command_info();
		}
		else if (isMatch(command, "run")) {
			command_run();
		}
		else if (isMatch(command, "stop")) {
			command_stop();
		}
		else if (isMatch(command, "default")) {
			command_default();
		}
		else if (isMatch(command, "show")) {
			command_show();
		}
		else if (isMatch(command, "load")) {
			command_load();
		}
		else if (isMatch(command, "save")) {
			command_save();
		}
		else if (isMatch(command, "set")) {
			command_set();
		}
		else {
			Serial.println("Unknown Command!. Try 'help'.");
		}
		Serial.println();
		flagExecRequest = false;
		flagTerminalResume = true;
	}
}


bool isMatch(char *p, char *keyword)
{
	int i, len = strlen(keyword);
	for (i = 0; i < len; i++)
	{
		if (tolower(p[i]) != tolower(keyword[i])) {
			break;
		}
	}
	return (i == len);
}


void command_help()
{
	Serial.print( F(  "\n  Open Heat-Pad Controller Terminal\n" \
						"    help        : show help message\n" \
						"    info        : show info of system\n" \
						"    run         : start control loop\n" \
						"    stop        : stop control loop\n" \
						"    showprofile : show current profile\n" \
						"    loadprofile : load profile from Storage\n" \
						"    saveprofile : save profile to Storage\n" \
						"    set <k> <v> : set value to specific profile key\n" \
						"    default     : restore profile on RAM to DEFAULT\n" \
					));
}


void command_info()
{
	Serial.print(F("\n  System Infomation\n"));
	Serial.print(F("   - Status        : "));
	Serial.println(task_isRunning()? "Running":"STOP");

	Serial.print(F("   - Setpoint Temp.: "));
	Serial.println(profile.setpointTemp);

	if(task_isRunning()) {
		Serial.print(F("   - Current Temp. : "));
		Serial.println(currentTemp);
	}
}


void command_run()
{
	Serial.print(F("Task starting... "));
	if (!task_isRunning()) {
		task_run();
		Serial.print(F("DONE\n"));
	}
	else{
		Serial.print(F("FAIL: already running\n"));
	}
	command_info();
}


void command_stop()
{
	Serial.print(F("Task stopping... "));
	task_stop();
	Serial.print(F("DONE\n"));
	command_info();
}


void task_run()
{
	isControlTaskEnabled = true;
	init_Temperature();
	init_PID();
	init_Controller();
	init_Plot();
	init_Terminal();
}


void task_stop()
{
	isControlTaskEnabled = false;
	setController(0);
}


bool task_isRunning()
{
	return isControlTaskEnabled;
}


void command_default()
{
	if (task_isRunning()) {
		Serial.print(F("FAIL: Use 'stop' to stop control task first."));
	}
	else {
		Serial.print(F("Set Default to Current Profile... "));
		struct profile_storage default_profile;
		profile = default_profile;
		Serial.print(F("DONE\n"));

		command_show();
	}
}


void command_show()
{

}


void command_load()
{
	Serial.print(F("Loading Profile from Storage... "));
	EEPROM.get(0, profile);
	Serial.print(F("DONE\n"));
}


void command_save()
{
	Serial.print(F("Saving Profile to storage... "));
	EEPROM.put(0, profile);
	Serial.print(F("DONE\n"));
}


void command_set()
{

}
