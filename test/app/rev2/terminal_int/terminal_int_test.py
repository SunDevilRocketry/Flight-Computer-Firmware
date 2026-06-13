import os
import time
import traceback
from pathlib import Path
import json
import csv
import shutil

from SDECv2.BaseController import Firmware, BaseController
from SDECv2.BaseController import create_controllers
from SDECv2.Parser import Parser, PresetConfig, DataBitmask, FeatureBitmask, Telemetry, create_configs
from SDECv2.SerialController import SerialSentry, SerialObj, Comport
from SDECv2.Sensor import SensorSentry, Sensor, create_sensors

from sdr_emulator_utils import Emulator, Tester

# Helper functions
def check_sensor(tester, sensor_name, readout):
    # These sensors will not trigger a fail if the readout is invalid
    postponed_list = ["pos", "altg", "speedg", "utc_time", "long", "lat", "ns", "ew", "gll_s", "rmc_s"]

    if readout:
        min = float('-inf')
        max = float('inf')
        match sensor_name:
            # RAW VALUES
            case "accXconv" | "accYconv" | "accZconv":
                min = -12
                max = 12
            case "gyroXconv" | "gyroYconv" | "gyroZconv":
                min = -25
                max = 25
            case "magXconv" | "magYconv":
                min = -20
                max = 20
            case "magZconv":
                min = -125
                max = 125
            case "pres":
                min = 90000
                max = 101300
            case "temp":
                min = 20
                max = 40
            # EMULATOR-SUPPORTED DERIVED VALUES
            case "alt":
                min = 300
                max = 400
            # OTHERS
            case _:
                pass # If unsupported by the emulator, all we care about is that the reading isn't broken. 
                        # This is true since the else branch's assert wasn't reached
        
        tester.assert_float_in_range(readout, min, max, f"Check that {sensor_name} is in the expected range.")
    else:
        # GPS is expected to fail if there are no valid readings, and GPS is only enabled after arming.
        # This is known behavior that you may want to consider a bug. If you would like this fixed,
        # please open a ticket.
        #
        # Position is known to be defective as of v2.6.0. The issue was noticed during mod#102
        # and we chose not to address it. The reading will instead be removed at a later date
        # when space is needed in the sensor data struct.
        #
        # Follow mod#101 for updates on this.
        if sensor_name not in postponed_list:
            # tester.assert_neq(0, 0, f"{sensor_name} has no available reading, so autofail.")
            print(f"{sensor_name} died lul")

sdec_comport = os.environ.get("SDEC_COMPORT")
emulator_comport = os.environ.get("EMULATOR_COMPORT")

# set up results asserter
tester = Tester()

# Set up emulator
emulator = Emulator("../../../../")
# Set serial object
serial_connection = SerialObj()
# Set up temporary directory
tmp_dir = Path("tmp")
tmp_dir.mkdir(exist_ok=True)

# connect
try:
    ########################################################
    ####################### SETUP ##########################
    ########################################################
    print("[setup] Building Emulator")
    emulator.build()

    print("[setup] Starting emulator")
    emulator.start()
    time.sleep(5)

    ########################################################
    ####################### CONNECT ########################
    ########################################################
    
    print("[connect] Connecting")
    serial_connection.init_comport(sdec_comport, 921600, 3)
    serial_connection.open_comport()
    serial_connection.connect()

    # assert proper connection
    print("[connect] Verifying connection")
    tester.assert_eq(serial_connection.target.controller.id, b'\x05', "Check that connect completed successfully (HW Opcode).")
    tester.assert_eq(serial_connection.target.firmware.id, b'\x06', "Check that connect completed successfully (FW Opcode).")

    print("[connect] Pinging FC")
    print("NOTE: The ping command does not have official SDEC support. We will manually send the opcode.")
    serial_connection.send(b"\x01")
    pong = serial_connection.read()
    tester.assert_eq(pong, serial_connection.target.controller.id, 'Check that ping returns the hardware opcode.')

    serial_connection.reset_input_buffer()
    serial_connection.reset_output_buffer()

    ########################################################
    ####################### SENSOR #########################
    ########################################################
    print("[sensor] Running dump subcommand")
    # Much of this was ripped from SDEC. I honestly don't love how they do
    # sensor dump, but oh well.
    flight_computer_rev2_sensors = create_sensors.flight_computer_rev2_sensors()
    sensor_sentry = SensorSentry(sensors=flight_computer_rev2_sensors)
    sensor_dump = sensor_sentry.dump(serial_connection)

    print("[sensor] Verifying dump results")
    accel_readings = 0
    baro_readings  = 0
    for sensor, readout in sensor_dump.items():
        check_sensor(tester, sensor.short_name, readout)

    serial_connection.reset_input_buffer()
    serial_connection.reset_output_buffer()

    ########################################################
    ######################## FIN ###########################
    ########################################################
    # Fin operations are unsupported on SDECv2
    # and are thus postponed from FCF until re-implemented.

    ########################################################
    ####################### FLASH ##########################
    ########################################################
    # Flash operations (except extract, which is tested
    # elsewhere) are unsupported on SDECv2 and are thus 
    # postponed from FCF until re-implemented.

    ########################################################
    ####################### IGNITE #########################
    ########################################################
    # Ignite operations are unsupported on SDECv2
    # and are thus postponed from FCF until re-implemented.

    ########################################################
    ####################### PRESET #########################
    ########################################################

    print("[preset] Most preset functionality is verified in the flight integration test. We will test the verify command though")
    print("[preset] Uploading presets")
    parser = Parser.upload_preset(serial_connection, path="support/test_presets.json")
    tester.assert_eq(type(parser), Parser, "Check that the parser object was created successfully.")

    print("[preset] Restart Emulator")
    serial_connection.close_comport()
    emulator.stop()
    time.sleep(7)
    emulator.start()
    time.sleep(10)

    print("[preset] Reconnect")
    serial_connection.init_comport(sdec_comport, 921600, 3)
    serial_connection.open_comport()
    serial_connection.reset_input_buffer()
    serial_connection.reset_output_buffer() # Flush before reconnect
    time.sleep(1)
    serial_connection.connect()
    tester.assert_eq(serial_connection.target.controller.id, b'\x05', "Check that connect completed successfully (HW Opcode).")
    tester.assert_eq(serial_connection.target.firmware.id, b'\x06', "Check that connect completed successfully (FW Opcode).")
    time.sleep(1)

    # POSTPONED: preset verify doesn't work on Linux, for some reason. This should be debugged.
    #print("[preset] Verify checksum")
    #tester.assert_eq(Parser.verify_preset(serial_connection), True, "Verify that the checksum matches what was sent on upload.")
    #serial_connection.reset_input_buffer()
    #serial_connection.reset_output_buffer()

    ########################################################
    ####################### SERVO ##########################
    ########################################################
    # Servo operations are unsupported on SDECv2
    # and are thus postponed from FCF until re-implemented.

    ########################################################
    ##################### DASHBOARD ########################
    ########################################################
    print("[dashboard] Get dashboard dump")
    telem_obj = Telemetry()

    telem_obj.dashboard_dump(serial_connection)
    dashboard_dump = telem_obj.get_latest_dashboard_dump()
    
    print("[dashboard] Verify sensors")
    for sensor, readout in dashboard_dump.items():
        check_sensor(tester, sensor, readout)

    serial_connection.reset_input_buffer()
    serial_connection.reset_output_buffer()

    ########################################################
    ######################## LORA ##########################
    ########################################################
    # ETS todo: LoRa commands are unsupported because
    # LoRa init is unsupported.
    #
    # This subset should follow sdr-emulator#56
    # or be implemented in a separate test

    ########################################################
    ###################### FINALIZE ########################
    ########################################################
    print("[finalize] Tearing down emulator")
    emulator.stop()
    time.sleep(5)

except Exception as e:
    print("A fatal error occurred during the test.")
    print(e)
    traceback.print_exc()
    tester.assert_eq(False, True, "A fatal error occurred during execution. See the log for more details.")
    emulator.stop()
finally:
    try:
        serial_connection.close_comport()
    except Exception:
        pass
    script_dir = Path(__file__).parent.resolve()
    shutil.rmtree(tmp_dir, ignore_errors=True)
    tester.write_results(str(script_dir) + "/results.txt", "flight_int")
    emulator.generate_coverage()
    emulator.copy_coverage(script_dir)#