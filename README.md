# Open Heat-Pad Controller
Heat-Pad Controller using Arduino and Thermistor. 

This project is an example of multi-tasking on Arduino using timer.  I'm try NOT to include OOP concept at this time, to keep it as simple as possible for a student. However, Please feel free to improve it if you like.

## Setup

**For Platform.IO**
clone this project to your local machine and you're good to go.

**For Arduino**
Create your own new sketch then copy all the source code in main.cpp to it. 

## Terminal

Connect the board to any Serial Port Terminal (i.e. Putty, Realterm) with **Baudrate at 115200.**

You should see something like this on your terminal.

```bash
>>
```

**If it's have a lot of number flooding on your screen type 'stop' and enter to stop all tasks first.**

For first time usage, following step is recommend.

1. type `default` to load default profile.

   ```bash
   >> default
   
   Set Default to Current Profile... DONE
   
   Current Profile
   --------------------
   11 - setpointTemp : 40.00
   12 - isAutorunEnabled : 0
   13 - isPlottingTaskEnabled : 0
   
   21 - ADC_MAX : 1023
   22 - THERMISTER_PIN : 0
   23 - THERMISTER_REF_RESISTER : 9960.00
   24 - THERMISTER_ROOM_RESISTANCE : 10000.00
   25 - THERMISTER_ROOM_TEMP : 25.00
   26 - THERMISTER_COEF_B : 2988.64
   
   31 - DENOISE_ORDER: 1
   32 - DENOISE_DIFF_LIMIT : 0.02
   41 - CONTROLLER_PWM_MAX : 255
   42 - CONTROLLER_PWM_PIN : 6
   43 - CONTROLLER_INA_PIN : 7
   44 - CONTROLLER_INB_PIN : 8
   
   51 - kP : 500.00
   52 - kI : 0.00
   53 - kD : 0.00
   
   >> 
   ```

2. type `save` to save default profile to the storage.

   ```bash
   >> save
   Saving Profile to storage... DONE
   
   Current Profile
   --------------------
   11 - setpointTemp : 40.00
   12 - isAutorunEnabled : 0
   13 - isPlottingTaskEnabled : 0
   
   21 - ADC_MAX : 1023
   22 - THERMISTER_PIN : 0
   23 - THERMISTER_REF_RESISTER : 9960.00
   24 - THERMISTER_ROOM_RESISTANCE : 10000.00
   25 - THERMISTER_ROOM_TEMP : 25.00
   26 - THERMISTER_COEF_B : 2988.64
   
   31 - DENOISE_ORDER: 1
   32 - DENOISE_DIFF_LIMIT : 0.02
   41 - CONTROLLER_PWM_MAX : 255
   42 - CONTROLLER_PWM_PIN : 6
   43 - CONTROLLER_INA_PIN : 7
   44 - CONTROLLER_INB_PIN : 8
   
   51 - kP : 500.00
   52 - kI : 0.00
   53 - kD : 0.00
   
   >>
   ```

For more information, type `help`



## Tested Hardware

- MCU: Arduino NANO v.3

- Thermistor: 10k @ 25 &deg;C NTC Type

- Heat-Pad: Custom PCB Heatbed

- Power Control: 24V 3A Motor Driver Board

- Power Supply: 24V 3A or Battery with Regulator

- Power Regulator: 5V 1A (or over) for MCU (For standalone usage)

  

1. Thermistor is serially connected with 10k Ohms Resister between 5V and Ground Pin on MCU.

   ```
   |---------| -- 5V --------------||
   |         |                     ||  ...... Thermister
   | Aruino  | -- Analog 0 -----||-||
   |         |                  || .......... 10K Resister
   |---------| -- GND ----------||
   
   * if Thermnistor and R 10k are swapped, 
     use invert measuering code on 'getResistance()' instread.
   ```

   

2. Motor Driver connected with MCU on Digital Pin 6, 7, 8 For PWM, Direction A, Direction B respectively.

   ```
   |---------| -- Digital 6 -------- PWM --- |---------------| 
   |         |                               |               |---------------- V Supply +
   |         | -- Digital 7 -------- IN A -- |               |---------------- V Supply -
   | Arduino |                               |  Motor Driver |
   |         | -- Digital 8 -------- IN B -- |               |----------------- HeatBed +
   |         |                               |               |----------------- HeatBed -
   |---------| -- GND -------------- Gnd --- |---------------|
                                                              
   ** For 'Power Side', twisted wire is recommended.        |-------- Power Side -------|
   ```



## Plotting

if  `13 - isPlottingTaskEnabled` is enabled ( Value = 1) then value of Current_Temperature and Setpoint Temperature will be periodically shown like this.

```
26.41 40.00
26.41 40.00
26.37 40.00
26.33 40.00
26.29 40.00
26.33 40.00
26.35 40.00
26.35 40.00
...
```

you can use **Serial Plotter** on Arduino IDE ( Toos > Serial Plotter ) to plot these value in real time.