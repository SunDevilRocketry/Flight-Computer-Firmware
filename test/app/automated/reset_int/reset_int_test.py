import os
import time
import traceback
from pathlib import Path
import json
import csv
import shutil
import threading

from SDECv2.BaseController import Firmware, BaseController
from SDECv2.BaseController import create_controllers
from SDECv2.Parser import Parser, PresetConfig, DataBitmask, FeatureBitmask, Telemetry, create_configs
from SDECv2.SerialController import SerialSentry, SerialObj, Comport

from sdr_emulator_utils import Emulator, Tester

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

# Set up threading/sync objects
stop_event = threading.Event()
telemetry_obj = Telemetry()

POLL_INTERVAL = 1/60

# Inspired by the API dashboard thread
def dashboard_update_thread(serial_connection: SerialObj):
    next_time = time.perf_counter()
    while not stop_event.is_set():
        next_time += POLL_INTERVAL

        try:
            serial_connection.reset_input_buffer()
            telemetry_obj.dashboard_dump(serial_connection)
        except ValueError as e: # Serial returned bad values
            print("Warning: The telemetry thread received an error. Attempting to recover, but this may not work.")
            serial_connection.reset_input_buffer()
            tester.assert_eq(False, True, "The dashboard thread received an error: {e}.")

        sleep_time = next_time - time.perf_counter()
        if sleep_time > 0: 
            time.sleep(sleep_time)
        else:
            next_time = time.perf_counter()

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
    
    print("[setup] Connecting")
    serial_connection.init_comport(sdec_comport, 921600, 3)
    serial_connection.open_comport()
    serial_connection.connect()

    # assert proper connection
    print("[setup] Verifying connection")
    tester.assert_eq(serial_connection.target.controller.id, b'\x05', "Check that connect completed successfully (HW Opcode).")
    tester.assert_eq(serial_connection.target.firmware.id, b'\x06', "Check that connect completed successfully (FW Opcode).")

    print("[setup] Uploading presets")
    parser = Parser.upload_preset(serial_connection, path="support/test_presets.json")
    tester.assert_eq(type(parser), Parser, "The parser object was created successfully.")

    print("[setup] Uploading LoRa presets")
    parser.upload_lora_preset(serial_connection, "support/test_presets.json")
    
    # give enough time to complete the serial transaction
    print("[setup] Waiting for completion")
    time.sleep(5)

    print("[setup] Tearing down setup phase")
    serial_connection.close_comport()
    emulator.stop()
    time.sleep(5)

    ########################################################
    ###################### EXECUTE #########################
    ########################################################
    print("[execute] Starting emulator")
    emulator.start(fast_arm=True, connect_gs=True)
    time.sleep(5)

    print("[execute] Waiting 10 seconds for calib and launch detect to run")
    time.sleep(10) # Allow enough time for calib to complete

    print("[execute] Tearing down initial run")
    emulator.stop()
    time.sleep(10)

    print("[execute] Starting emulator")
    emulator.start(recovery_reg=0x80000004) # Will break if ascent != 4
    time.sleep(5)

    print("[execute] Waiting 20 seconds for flash to fill")
    time.sleep(20) # Allow enough time for calib to complete

    ########################################################
    ###################### VERIFY ##########################
    ########################################################
    print("[verify] Starting emulator")
    emulator.start()
    time.sleep(5)

    print("[verify] Connecting")
    serial_connection.reset_input_buffer() # Flush before reconnect
    serial_connection.connect()

    # assert proper connection
    print("[verify] Asserting Connection Status")
    tester.assert_eq(serial_connection.target.controller.id, b'\x05', "Check that connect completed successfully (HW Opcode).")
    tester.assert_eq(serial_connection.target.firmware.id, b'\x06', "Check that connect completed successfully (FW Opcode).")
    
    # Flash Extract
    print("[verify] Flash Extract")
    extract_results = tmp_dir / "extract_results.csv"
    extract_preset = tmp_dir / "extract_preset.json"
    appa_parser = Parser(
            preset_config=create_configs.appa_preset_config(),
            preset_data=None
        )
    appa_parser.flash_extract(serial_connection, preset_path=extract_preset.resolve(), data_path=extract_results.resolve())

    flash_data = []
    with open(extract_results.resolve(), "r") as f:
        reader = csv.reader(f)
        for row in reader:
            flash_data.append(row)
    flash_preset = {}
    with open(extract_preset.resolve(), "r") as f:
        flash_preset = json.load(f)

    # Verify extracted preset matches downloaded
    print("[verify] Verifying extract")

    # Verify that after launch detect there are ascent frames
    launch_detect = False
    ascent = False
    for row in flash_data:
        if row[1] == '3':
            launch_detect = True
        elif row[1] == '4':
            ascent = True
    tester.assert_eq(launch_detect, True, "Launch detect frames present in extracted data.")
    tester.assert_eq(flash_data[2][0], True, "Ascent frames present in extracted data.")

    # Finish test
    print("[verify] Tearing down verify phase")
    emulator.stop()
    time.sleep(5)

except Exception as e:
    print("A fatal error occurred during the test.")
    print(e)
    traceback.print_exc()
    tester.assert_eq(False, True, "A fatal error occurred during execution. See the log for more details.")
finally:
    try:
        serial_connection.close_comport()
    except Exception:
        pass
    script_dir = Path(__file__).parent.resolve()
    shutil.rmtree(tmp_dir, ignore_errors=True)
    tester.write_results(str(script_dir) + "/results.txt", "flight_int")
    emulator.generate_coverage()
    emulator.copy_coverage(script_dir)