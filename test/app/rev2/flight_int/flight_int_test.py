import os
import time
import traceback
from pathlib import Path
import json
import csv

from SDECv2.BaseController import Firmware, BaseController
from SDECv2.BaseController import create_controllers
from SDECv2.Parser import Parser, PresetConfig, DataBitmask, FeatureBitmask, create_configs
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
    
    # give enough time to complete the serial transaction
    print("[setup] Waiting for completion")
    time.sleep(2)

    print("[setup] Tearing down setup phase")
    emulator.stop()
    time.sleep(5)

    ########################################################
    ###################### EXECUTE #########################
    ########################################################
    print("[execute] Starting emulator")
    emulator.start(fast_arm=True)
    time.sleep(5)

    print("[execute] Waiting 10 seconds for calibration")
    time.sleep(10) # Allow enough time for calib to complete

    print("[execute] Tearing down execute phase")
    emulator.stop()
    time.sleep(5)

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

    # download the preset
    print("[verify] Preset Download")
    tmp_dir = Path("tmp")
    tmp_dir.mkdir(exist_ok=True)
    tmp_preset = tmp_dir / "tmp_preset.json"
    appa_parser = Parser(
        preset_config=create_configs.appa_preset_config(),
        preset_data=None
    )
    appa_parser.download_preset(serial_connection, path=tmp_preset.resolve())
    downloaded = {}
    with open(tmp_preset.resolve(), 'r') as file:
        downloaded = json.load(file)
    oracle = {}
    with open("support/test_presets.json", 'r') as file:
        oracle = json.load(file)

    # Check presets
    print("[verify] Checking Presets")
    tester.assert_eq(downloaded.get("Feature Bitmask"), oracle.get("Feature Bitmask"), "Feature Bitmasks Equivalent")
    tester.assert_eq(downloaded.get("Data Bitmask"), oracle.get("Data Bitmask"), "Feature Bitmasks Equivalent")
    tester.assert_eq(downloaded.get("Config Data"), oracle.get("Config Data"), "Config Data Equivalent")
    
    # Flash Extract
    print("[verify] Flash Extract")
    extract_results = tmp_dir / "extract_results.json"
    extract_preset = tmp_dir / "extract_preset.json"
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
    tester.assert_eq(flash_preset, downloaded, "Flash extract data matches downloaded preset.")

    # Verify frame sizes match
    # (row 0 is header row)
    tester.assert_eq(flash_data[1][0], '1', "First save bit present in extracted data.")
    tester.assert_eq(flash_data[2][0], '1', "Second save bit present in extracted data.")

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
    tester.write_results(str(script_dir) + "/results.txt", "flight_int")
    emulator.generate_coverage()
    emulator.copy_coverage(script_dir)