## Sun Devil Rocketry Firmware Testing

### Getting Started

To get started with firmware testing, you'll need to follow these steps

1. Complete the SDR firmware setup and emulator setup
2. Install GCOVR

The steps above are sufficient for our unit tests. To run integration tests, you
need to follow these steps:

1. Install SDECv2 as editable (from this directory, run "pip install -e ./SDECv2")
2. Install the test utilities as editable (from this directory, run "pip install -e ./utilities")
3. Add your virtual comport pair as environment variables "SDEC_COMPORT" and "EMULATOR_COMPORT"