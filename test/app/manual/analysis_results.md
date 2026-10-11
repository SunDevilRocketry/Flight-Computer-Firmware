## Structural Requirement Analysis

### Conductor: Eli Sells | 10/10/2026

Some requirements cannot be verified through automated methods. These have been manually verified below:

| Requirement tag | Requirement text | Remarks | Test status |
| --- | --- | --- | --- |
| `RQ.FC-SYS.00001` | The system shall be designed for operation on an STM32-based microcontroller. | The flight computer is based on the STM32H750. | <span style="color: #15803d"><strong>PASS</strong></span> |
| `RQ.FC-SYS.00017` | The system shall utilize the Sun Devil Rocketry “lib”, “mod”, and “driver” libraries to interface with flight computer hardware. | The "mod", "lib", and "driver" libraries are compiled into the application. | <span style="color: #15803d"><strong>PASS</strong></span> |