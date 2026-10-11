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

    print("[Setup] Uploading presets")
    parser = Parser.upload_preset(serial_connection, path="support/test_presets.json")
    tester.assert_eq(type(parser), Parser, "The parser object was created successfully.")

    print("[Setup] Uploading LoRa presets")
    parser.upload_lora_preset(serial_connection, "support/test_presets.json")
    
    # give enough time to complete the serial transaction
    print("[Setup] Waiting for completion")
    time.sleep(5)

    print("[Setup] Tearing down setup phase")
    serial_connection.close_comport()
    emulator.stop()
    time.sleep(5)

    ########################################################
    ################## Case 1: EXECUTE #####################
    ########################################################
    
    # Tests the ascent phase restoration

    print("[Case 1: Execute] Set up dashboard drain")
    dashboard_dump_thread = threading.Thread(
        target=dashboard_update_thread,
        args=[serial_connection],
        daemon=True
    )
    
    print("[Case 1: Execute] Starting emulator")
    emulator.start(fast_arm=True, connect_gs=True)
    time.sleep(5)

    print("[Case 1: Execute] Waiting 10 seconds for calib and launch detect to run")
    time.sleep(10) # Allow enough time for calib to complete

    print("[Case 1: Execute] Tearing down initial run")
    emulator.stop()
    time.sleep(10)

    print("[Case 1: Execute] Starting emulator with the recovery register set to 0x80000004")
    emulator.start(connect_gs=True, recovery_reg=0x80000004) # Will break if ascent != 4
    time.sleep(5)

    print("[Case 1: Execute] Start dashboard drain")
    serial_connection.open_comport()
    serial_connection.reset_input_buffer() # Flush before reconnect
    serial_connection.connect()
    dashboard_dump_thread.start()

    print("[Case 1: Execute] Waiting 20 seconds for flash to fill")
    time.sleep(20) # Allow enough time for calib to complete

    print("[Case 1: Execute] Tearing down recovered run")
    stop_event.set()
    dashboard_dump_thread.join()
    serial_connection.close_comport()
    emulator.stop()
    time.sleep(10)

    ########################################################
    ################## Case 1: VERIFY ######################
    ########################################################
    print("[Case 1: Verify] Starting emulator")
    emulator.start()
    time.sleep(5)

    print("[Case 1: Verify] Connecting")
    serial_connection.open_comport()
    serial_connection.reset_input_buffer() # Flush before reconnect
    serial_connection.connect()

    # assert proper connection
    print("[Case 1: Verify] Asserting Connection Status")
    tester.assert_eq(serial_connection.target.controller.id, b'\x05', "Check that connect completed successfully (HW Opcode).")
    tester.assert_eq(serial_connection.target.firmware.id, b'\x06', "Check that connect completed successfully (FW Opcode).")
    
    # Flash Extract
    print("[Case 1: Verify] Flash Extract")
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
    print("[Case 1: Verify] Verifying extract")

    # Verify that after launch detect there are ascent frames
    launch_detect = True
    for row in flash_data[1:]: # skip header row
        if row[1] == '3':
            launch_detect = True
        elif row[1] != '3':
            tester.assert_eq(row[1], '4', "Verify that the next row after launch detect is in the state the recovery register holds", reqs="RQ.FC-SW.00005, RQ.FC-SW.00006, RQ.FC-SYS.00038")
            break
        else:
            tester.assert_eq(True, False, "Verify that the next row after launch detect is in the state the recovery register holds")
            break

    print("[Case 1: Verify] Check that a dashboard_dump message was received i.e. telemetry was initialized.")
    dump_msg = telemetry_obj.get_latest_dashboard_dump()
    tester.assert_eq(dump_msg is not None, True, "Check that a dashboard_dump message was received i.e. telemetry was initialized.", reqs="RQ.FC-SW.00007")

    # Finish case
    print("[Case 1: Verify] Tearing down verify phase")
    emulator.stop()
    time.sleep(5)

    ########################################################
    ################## Case 2: EXECUTE #####################
    ########################################################

    # Tests the calib phase restoration

    # Replace the global telemetry object to clear globals
    telemetry_obj = Telemetry()
    print("[Case 2: Execute] Set up dashboard drain")
    dashboard_dump_thread = threading.Thread(
        target=dashboard_update_thread,
        args=[serial_connection],
        daemon=True
    )

    print("[Case 2: Execute] Starting emulator")
    emulator.start(fast_arm=True, connect_gs=True)
    time.sleep(5)

    print("[Case 2: Execute] Waiting 10 seconds for calib and launch detect to run")
    time.sleep(10) # Allow enough time for calib to complete

    print("[Case 2: Execute] Tearing down initial run")
    emulator.stop()
    time.sleep(10)

    print("[Case 2: Execute] Starting emulator with the recovery register set to 0x80000002")
    emulator.start(connect_gs=True, recovery_reg=0x80000002) # Will break if calib != 2
    time.sleep(5)

    print("[Case 2: Execute] Start dashboard drain")
    #serial_connection.open_comport()
    serial_connection.reset_input_buffer() # Flush before reconnect
    serial_connection.connect()
    dashboard_dump_thread.start()

    print("[Case 2: Execute] Waiting 20 seconds for calib to finish")
    time.sleep(20) # Allow enough time for calib to complete

    print("[Case 2: Execute] Tearing down recovered run")
    stop_event.set()
    dashboard_dump_thread.join()
    emulator.stop()
    serial_connection.close_comport()
    time.sleep(10)

    ########################################################
    ################## Case 2: VERIFY ######################
    ########################################################
    print("[Case 2: Verify] Starting emulator")
    emulator.start()
    time.sleep(5)

    print("[Case 2: Verify] Connecting")
    serial_connection.open_comport()
    serial_connection.reset_input_buffer() # Flush before reconnect
    serial_connection.connect()

    # assert proper connection
    print("[Case 2: Verify] Asserting Connection Status")
    tester.assert_eq(serial_connection.target.controller.id, b'\x05', "Check that connect completed successfully (HW Opcode).")
    tester.assert_eq(serial_connection.target.firmware.id, b'\x06', "Check that connect completed successfully (FW Opcode).")
    
    # Flash Extract
    print("[Case 2: Verify] Flash Extract")
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
    print("[Case 2: Verify] Verifying extract")

    # Verify that after launch detect there are ascent frames
    launch_detect = True
    for row in flash_data[1:]: # skip header row
        if row[1] == '3':
            launch_detect = True
        elif row[1] == '-1':
            break
        else:
            launch_detect = False
            tester.assert_eq(True, False, "Verify that the data from the ascent phase was cleared out")
            break

    tester.assert_eq(launch_detect, True, "Verify that the data from the ascent phase was cleared out", reqs="RQ.FC-SW.00005, RQ.FC-SW.00008, RQ.FC-SYS.00038")

    print("[Case 2: Verify] Check that a dashboard_dump message was received i.e. telemetry was initialized.")
    dump_msg = telemetry_obj.get_latest_dashboard_dump()
    tester.assert_eq(dump_msg is not None, True, "Check that a dashboard_dump message was received i.e. telemetry was initialized.", reqs="RQ.FC-SW.00007")
    
    # Finish case
    print("[Case 2: Verify] Tearing down verify phase")
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
    try:
        emulator.stop()
    except Exception:
        pass
    script_dir = Path(__file__).parent.resolve()
    shutil.rmtree(tmp_dir, ignore_errors=True)
    tester.write_results(str(script_dir) + "/results.txt", "flight_int")
    emulator.generate_coverage()
    emulator.copy_coverage(script_dir)