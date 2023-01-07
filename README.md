<h1>Flight Computer Firmware</h1>

<p>PCB: Flight Computer A0002</p>
<p>MCU: STM32H750VBT6 </p>
<p>MPU Architecture: ARM Cortex-M7</p>

<p>Description: The flight computer is Sun Devil Rocketry's first ever in-house designed and manufactured high-power rocketry altimeter. The flight computer project aims to serve as a drop-in replacement for the commercially produced altimeters currently used in Sun Devil Rocketry rockets, while also supporting more advanced rocketry projects by including extra sensors and peripheral interfaces. The board supports dual-deployment parachute ejection through the use of ematch screw terminals, a barometric pressure sensor, a buzzer, an ARM Cortex-M7 microcontroller, and 4 Mb of external flash for logging flight data. The computer also includes a USB port and a micro SD card for easy access to flight data, so a seperate data collection kit is not required. To support more advanced projects, the computer contains a LoRa wireless module, a GPS module, a 9-axis IMU, and four 3-connector PWM servo drivers. Design of the flight computer was optimized for a minimum form factor to allow the computer to be used in a variety of differently sized rockets. </p>

<h2>Working Directory Structure</h2>

<p>
app: application code for the flight computer containing source directories

auto: auto-generated code from STM32CubeMX (not compiled into application)

init: Microcontroller initialization and configuration code

lib: third-party libraries for device drivers and middleware, microcontroller pin and peripheral configurations 

mod: Hardware modules containing hardware specific code for SDR boards

test: Test code
</p>

<h2>Source Directories:</h2>
<p>
blink:blinks status LED to test programmer and board setup 

flight/data-logger: firmware to collect data during test flights 

flight/dual-deploy: firmware for dual-deploy parachute recovery 

terminal: firmware to allow terminal access to all PCB hardware 
</p>
