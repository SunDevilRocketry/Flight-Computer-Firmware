## Structural Requirement Analysis

### Conductor: Nicholas Armistead | 10/3/2026

## Requirements

RQ.FC-SYS.00002
RQ.FC-SYS.00003
RQ.FC-SYS.00004
RQ.FC-SYS.00005
RQ.FC-SYS.00006
RQ.FC-SYS.00007
RQ.FC-SYS.00008
RQ.FC-SYS.00009
RQ.FC-SYS.00012
RQ.FC-SYS.00013
RQ.FC-SYS.00014
RQ.FC-SYS.00015
RQ.FC-SYS.00016
RQ.FC-SYS.00018
RQ.FC-SYS.00019
RQ.FC-SYS.00020
RQ.FC-SYS.00021
RQ.FC-SYS.00025
RQ.FC-SYS.00026
RQ.FC-SYS.00027
RQ.FC-SYS.00028

## Procedure:

-------Test 1-------

Upload a preset
Download a preset
Run flash extract
Observe continuity between all configs, upload here

-------Test 2-------

Upload base config
Arm FC
Wait 25 seconds
Run flash extract and upload data

-------Test 3-------

Upload base config
Arm FC, shake to trigger launch detect
Wait for flash to fill (blue LED)
Run flash extract and upload data

-------Test 4-------

Upload base config with rate limit set to 100
Arm FC, shake to trigger launch detect
Run for a few seconds, then reset
Run flash extract and upload data

-------Test 5-------

Upload base config with only pre-converted data enabled
Arm FC, shake to trigger launch detect
Run for a few seconds, then reset
Run flash extract and upload data

## Results

Test 1: PASS

Test 2: PASS

Test 3: PASS

Test 4: PASS
Note that the actual logging rate is closer to 90 Hz, but this is acceptable since the rate limiter sets a maximum rate.

Test 5: PASS