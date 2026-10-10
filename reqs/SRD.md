<!-- next traceable tag: RQ.FC-SW.00010 -->
<!-- This req doc is written and updated manually and checked in along with source code -->
<!-- Desired behaviors are expected to be specified by human engineers. No AI assistance allowed for requirement specification -->
# SDR Flight Computer Software Requirements Document

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

The core flight logic does not have any low-level software requirements.

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

RQ.FC-SW.00006 (Trace: RQ.FC-SYS.00038) - The error recovery system shall search flash to find the first empty frame and set the address to match it

    - RQ.FC-SW.00008 (Derived) - If the next empty frame cannot be found and the restored state is launch detect or earlier, the system shall erase the contents of flash while preserving the presets.

    - Rationale: The boost phase of flight is the most critical for data logging. Erasing during launch detect would allow us to capture this phase properly, and erasing during/after ascent would wipe out the boost data.

    - RQ.FC-SW.00009 (Derived) - Otherwise, the system shall indicate that the flash subsystem is not usable.

    - Rationale: Robustness

RQ.FC-SW.00007 (Trace: RQ.FC-SYS.00038) - If the flight computer state has passed calibration, the system shall re-initialize the telemetry system.

