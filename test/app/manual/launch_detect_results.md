## Structural Requirement Analysis

### Conductor: Nicholas Armistead | 10/1/2026

## Requirements

RQ.FC-SYS.00013
RQ.FC-SYS.00022
RQ.FC-SYS.00023
RQ.FC-SYS.00024

## Procedure:

Case 1 (accel):

Upload a configuration with accel threshold at 2g and 3 samples.
Arm and shake FC
Observe state transition
Success criteria:

Observe sensor data and ensure that the threshold and counts are reached at the point where the state transitions.

Case 2 (baro):

Power Flight Computer using the shop's digital power supply set to 7.4V (equivalent to 2S LiPo power)
Upload a configuration with baro threshold at 300 Pa and 5 samples.
Arm and place FC in vacuum chamber and partially depressurize (use a test-only unit for safety)
Observe state transition
Success criteria:

Observe sensor data and ensure that the threshold and counts are reached at the point where the state transitions.

## Results

Case 1 (accel): PASS
Case 2 (baro): PASS