<!-- next traceable tag: RQ.FC-SYS.00039 -->
<!-- This req doc is written and updated manually and checked in along with source code -->
<!-- Desired behaviors are expected to be specified by human engineers. No AI assistance allowed for requirement specification -->
# SDR Flight Computer System Requirements Document

### QA level: Mission Critical
### Part Number: A0002-2XX

## 1. Overview

The All Purpose Primary Avionics (a.k.a. “APPA”) project seeks to improve the reliability and maintainability of Sun Devil Rocketry’s Avionics firmware. In its current form, the codebase contains a large amount of repeated code between different applications. This causes a number of problems:

1. Any new features in one application typically require a port to another.
2. Any existing features that require updates require similar updates to any clones.
3. Any existing features that are modified across applications require each implementation to be tested separately, increasing the club’s final verification burden before every launch.
4. Implementations of the same feature may vary between applications, making it more difficult to see when just one implementation has issues.
5. Interfaces with common modules need to be able to accommodate every implementation. This has, for example, caused SDEC’s complexity to increase significantly as more firmware projects have unique needs.

The first primary goal of APPA is to mitigate these problems at the source; rather than attacking increasingly common bugs individually, the intention is to fundamentally alter the architecture of the flight firmware to make these bugs common by ceasing support for multiple applications with cloned functions in favor of a common firmware application. 

The second primary goal of APPA is to decentralize the application. In much of the flight firmware, the bulk of the application sits in the same file as the main() function. This was acceptable when the firmware was a far smaller project. But today, SDR flight computers are directly responsible for flight dynamics/control, serve as the primary data loggers for some flights, and are otherwise significantly more complex than they were previously. This complexity has revealed the need for more exhaustive testing, both at the unit level and above. In its current form, the avionics team is unable to perform unit testing on any function in the main.c file. By moving the flight and terminal portions of the application to their own functions and files, the maximum cyclomatic complexity of the application has been significantly reduced, allowing the team to tackle unit testing on the app layer rather than just the mod layer like before. It also makes the code significantly more readable, as the highest level operation of the application is able to be abstracted significantly more than in previous iterations.

## 2. Hardware Requirements

RQ.FC-SYS.00001 - The system shall be designed for operation on an STM32-based microcontroller.

    - Test Plan: Structural Requirement

RQ.FC-SYS.00002 - The system shall have a method to report its angular and linear acceleration.

    - Test Plan: Flash Extract System Test (manual)

RQ.FC-SYS.00003 - The system shall have a method to report its GPS coordinates.

    - Test Plan: Flash Extract System Test (manual)

RQ.FC-SYS.00004 - The system shall have a method to report barometric readings.

    - Test Plan: Flash Extract System Test (manual)

RQ.FC-SYS.00005 - The system shall have a method to connect to an outside computer.

    - Test Plan: Flash Extract System Test (manual)

RQ.FC-SYS.00006 - The system shall have a method to record data to non-volatile memory.

    - Test Plan: Flash Extract System Test (manual)

RQ.FC-SYS.00007 - The system shall have an external switch to facilitate arming the system.

    - Test Plan: Flash Extract System Test (manual)

The system shall have a method to provide basic operational information to the user, including:

	- RQ.FC-SYS.00008 - A visual method.

	- RQ.FC-SYS.00009 - An auditory method.

    - Test Plan: Flash Extract System Test (manual)

RQ[POSTPONED].FC-SYS.00010 - The system shall have a method to control four servos.

RQ[POSTPONED].FC-SYS.00011 - The system shall have a method to activate ejection charges.

RQ.FC-SYS.00012 - The system shall have a method to accept power over USB.

    - Test Plan: Flash Extract System Test (manual)

RQ.FC-SYS.00013 - The system shall have a method to accept power from a 2S LiPo.

    - Test Plan: Launch Detect System Test (manual)

RQ.FC-SYS.00014 - The system shall have a method to send wireless radio signals.

    - Test Plan: Telemetry System Test (manual)

RQ.FC-SYS.00015 - The system shall have a method to restart the application.

    - Test Plan: Flash Extract System Test (manual)

RQ.FC-SYS.00016 - The system shall have a method to upload new firmware.

    - Test Plan: Flash Extract System Test (manual)

## 3. Firmware Requirements

RQ.FC-SYS.00017 - The system shall utilize the Sun Devil Rocketry “lib”, “mod”, and “driver” libraries to interface with flight computer hardware.

    - Test Plan: Structural Requirement

RQ.FC-SYS.00018 - The system shall have an option to log flight data.

    - RQ.FC-SYS.00019 - The data that is logged shall be configurable.

    - RQ.FC-SYS.00020 - The data logging shall begin when launch has been detected and end when flash is full.

    - RQ.FC-SYS.00021 - The data logging shall have a method to reduce the sampling rate and extend the period that data is logged.

    - Test Plan: Flash Extract System Test (manual)

RQ.FC-SYS.00022 - The system shall have a methods to detect if launch has occurred, including:

    - RQ.FC-SYS.00023 - A method based on barometric pressure

    - RQ.FC-SYS.00024 - A method based on acceleration

    - Test Plan: Launch Detect System Test (manual)

RQ.FC-SYS.00025 - The system shall calibrate the IMU and barometer before launch.

    - Test Plan: Flash Extract System Test (manual)

RQ.FC-SYS.00026 - The system shall have an interface to communicate with SDEC.

    - Test Plan: Flash Extract System Test (manual)

RQ.FC-SYS.00027 - The system shall change modes from the SDEC interface to the pre-launch mode when the external switch is activated.

    - Test Plan: Flash Extract System Test (manual)

RQ.FC-SYS.00028 - The system shall have a number of configurable parameters that can be changed without recompiling the application.

    - Test Plan: Flash Extract System Test (manual)

RQ.FC-SYS.00029 - The system shall be able to wirelessly broadcast vehicle data to observers on the ground.

    - RQ.FC-SYS.00030 - The wireless system shall be configurable at runtime.

    - RQ.FC-SYS.00031 - The wireless system shall run without blocking the main flight loop.

    - The wireless system shall broadcast messages with the following data:

        - RQ.FC-SYS.00032 - Messages containing vehicle state data, including position and orientation.

        - RQ.FC-SYS.00033 - Messages containing vehicle identification data, such as the firmware and hardware IDs.

        - RQ.FC-SYS.00034 - Messages containing software calibration data, such as the IMU offsets and barometric calibration offsets.

    - Test Plan: Telemetry System Test (manual)

RQ.FC-SYS.00035 - The system shall calculate an orientation estimate based on sensor data.

    - RQ.FC-SYS.00036 - The orientation shall be represented by a Hamilton unit quaternion.

    - RQ.FC-SYS.00037 - The system shall fuse data from the accelerometer and gyroscope to determine orientation.

    - Test Plan: Telemetry System Test (manual)

RQ.FC-SYS.00038 - The system shall have a method to recover from software triggered fail-fast errors.

    - Test Plan: Error Recovery Integration Test