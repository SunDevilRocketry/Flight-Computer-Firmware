---
name: Verification
about: Test a piece of functionality
title: Verify app/[...]/foo.c
labels: verification
assignees: ''

---

### Verification Task Type
Low-level (Unit Test), High-level (Integration test), or System-level.

### File or Functionality Under Test
For a low-level test, this should be _one_ file. For a high-level test, this is a broader functionality, e.g. flash data logging.

### Suggested Test Procedure
1. Power on
2. Short the switch terminal
3. Power off
4. Collect data from SDEC.

### Subject Matter Experts
Who should be consulted for guidance? Don't just say @nick or @eli for everything, try to find out if the person who implemented the file originally is still at SDR. "git blame" is your friend here.
