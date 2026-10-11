<!-- next traceable tag: RQ.FC-SW.00010 -->
<!-- This req doc is written and updated manually and checked in along with source code -->
<!-- Desired behaviors are expected to be specified by human engineers. No AI assistance allowed for requirement specification -->
# SDR Flight Computer Software Design Document

### QA level: Mission Critical
### Part Number: A0002-2XX

## 1. Overview

<tbd>

The software for the Flight Computer is broken down into the following subsystems:

    1. Hardware/Software Initialization (NFQ)
    2. Terminal Interface (NFQ)
    3. Core Flight Logic & State Transitions (FQ)
    4. Data Logging (FQ)
    5. [POSTPONED] Parachute Deployment (FQ)
    6. [POSTPONED] Flight Controls (FQ)
    7. Telemetry (FQ)
    8. Error Handling & Recovery (FQ)

## 2. Structural Requirements

## 3. Functional Requirements

### 3.1. Hardware/Software Initialization

Hardware/Software initialization does not have any low-level software requirements.

### 3.2. Terminal Interface

The terminal interface does not have any low-level software requirements.

### 3.3. Core Flight Logic

The flight computer's status LED color, finite state machine states & state transitions shall be defined by the table below:

| Requirement                                        | State                  | Description                                                                | LED Color  | Relevant Function    | Transition Function                           |
|----------------------------------------------------|------------------------|----------------------------------------------------------------------------|------------|----------------------|-----------------------------------------------|
| RQ.FC-SW.00010 (Derived)                           | FC_STATE_INIT          | Initializes the flight computer's peripherals on power up.                 | LED off    | main()               | Automatically moves to FC_STATE_IDLE          |
| RQ.FC-SW.00011 (Trace: RQ.FC-SYS.00026)            | FC_STATE_IDLE          | Provides a terminal interface for USB communication and waits until armed. | LED_GREEN  | prelaunch_terminal() | ign_switch_armed()                            |
| RQ.FC-SW.00010 (Trace: RQ.FC-SYS.00025)            | FC_STATE_CALIB         | Runs one-time setup operations that need to happen after arming.           | LED_YELLOW | flight_calib()       | Automatically moves to FC_STATE_LAUNCH_DETECT |
| RQ.FC-SW.00011 (Trace: RQ.FC-SYS.00022)            | FC_STATE_LAUNCH_DETECT | Waits for launch to be detected and erases flash before it fills too fast  | LED_CYAN   | flight_loop()        | launch_detection()                            |
| RQ.FC-SW.00012 (Trace: RQ.FC-SYS.00018)            | FC_STATE_ASCENT        | Logs data in flight. Powered ascent.                                       | See XXXXX  | flight_loop()        | apogee_detect() and coast_detect()            |
| RQ.FC-SW.00013 (Trace: RQ.FC-SYS.00018)            | FC_STATE_COAST         | Logs data in flight. Unpowered ascent.                                     | See XXXXX  | flight_loop()        | apogee_detect()                               |
| RQ[POSTPONED].FC-SW.00014 (Trace: RQ.FC-SYS.00039) | FC_STATE_APOGEE        | Triggers parachute deployment.                                             | See XXXXX  | flight_deploy()      | Automatically moves to FC_STATE_DESCENT       |
| RQ.FC-SW.00015 (Trace: RQ.FC-SYS.00018)            | FC_STATE_DESCENT       | Logs data in flight. Unpowered descent.                                    | See XXXXX  | flight_loop()        | Final state                                   |
| RQ[POSTPONED].FC-SW.00016 (Trace: RQ.FC-SYS.00018) | FC_STATE_LANDED        | Flight computer is at rest on the ground.                                  | TBD        | TBD                  | Final state                                   |

### 3.4. Data Logging

The terminal interface does not have any low-level software requirements.

### 3.5. Parachute Deployment

Parachute deployment does not have any low-level software requirements.

### 3.6. Flight Controls

Flight controls do not have any low-level software requirements.

### 3.7. Telemetry

Telemetry does not have any low level software requirements.

### 3.8. Error Handling & Recovery

#### 3.8.1. Error Detection

RQ.FC-SW.00004 (Trace: RQ.FC-SYS.00038) - The error recovery system shall intercept fail-fast error calls to trigger a software reset.

RQ.FC-SW.00001 (Trace: RQ.FC-SYS.00038) - The error recovery system shall store the following parameters across the reset and reconstruct the rest of the application state to avoid prior issues polluting the system post-reset:
    
    RQ.FC-SW.00002 (Trace: RQ.FC-SYS.00038) - The current flight computer state machine state

    RQ.FC-SW.00003 (Trace: RQ.FC-SYS.00038) - A flag indicating that a recovery should be performed upon power-up

#### 3.8.2. Error Recovery

The following requirements will be triggered if the error recovery flag is found on startup:

RQ.FC-SW.00005 (Trace: RQ.FC-SYS.00038) - The system shall restore the previous flight computer state

    - RQ.FC-SW.00006 (Trace: RQ.FC-SYS.00038) - If the restored state is ascent or later, the error recovery system shall search flash to find the first empty frame and set the address to match it

    - RQ.FC-SW.00008 (Derived) - If the restored state is launch detect or earlier, the system shall erase the contents of flash while preserving the presets.

    - Rationale: The boost phase of flight is the most critical for data logging, and we don't restore timers when we reset, so we might write further into flash than expected. Erasing during launch detect would allow us to capture this phase properly, and erasing during/after ascent would wipe out the boost data.
    
    - RQ.FC-SW.00009 (Derived) - If a flash error is detected, the system shall indicate that the flash subsystem is not usable.

    - Rationale: Robustness

RQ.FC-SW.00007 (Derived) - If the flight computer state has passed calibration and telemetry is configured, the system shall re-initialize the telemetry system.

    - Rationale: The requirement says we need to reconstruct the FC's state, which means we need to redo setup steps that were skipped on recovery.

