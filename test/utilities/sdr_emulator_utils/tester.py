# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025 Sun Devil Rocketry
#
# Test utility for FW/SDEC integration tests.

from datetime import datetime

class Result:
    def __init__(self, success, actual: str, expected: str, msg: str):
        self.success = success
        self.actual = actual
        self.expected = expected
        self.msg = msg

        if not self.success:
            print(str(self))

    def is_pass(self):
        return self.success

    def __str__(self):
        if not self.success:
            return f'FAIL: "{self.msg}"\nActual: "{self.actual}" | Expected: "{self.expected}"'
        else:
            return f'"{self.msg}"\nActual: "{self.actual}" | Expected: "{self.expected}"'

class Tester:
    def __init__(self):
        self.__results = []

    def assert_eq(self, arg1, arg2, msg: str):
        self.__results.append(Result(arg1 == arg2, str(arg1), str(arg2), msg))

    def assert_neq(self, arg1, arg2, msg: str):
        self.__results.append(Result(arg1 != arg2, str(arg1), str(arg2), msg))

    # Writes out results in the integration test style
    def write_results(self, results_file: str, test_name: str):
        # Get current date and time
        now = datetime.now()

        # Format as Month Day, Year
        formatted_date = now.strftime("%B %d, %Y")
        formatted_time = now.strftime("%H:%M:%S")
        header_str = (
             "----------------------------------------\n"
             "-----------Sun Devil Rocketry-----------\n"
             "-----------Auto Test Results------------\n"
             "----------------------------------------\n\n"
             "--General Build Information--\n"
            f"Test Name:        {test_name}\n"
             "Test Type:        SW Integration\n"
            f"Run Date:         {formatted_date}\n"
            f"Run Time:         {formatted_time}\n\n"
             "----------------------------------------\n"
             "----------------Results-----------------\n"
             "----------------------------------------\n\n"
        )
        # get results
        if len(self.__results) == 0:
            raise KeyError("No results exist!")
        passes = 0
        total = 0
        with open(results_file, "w") as f:
            f.write(header_str)
            for result in self.__results:
                total += 1
                if result.is_pass():
                    passes += 1
                f.write(str(result) + "\n\n")
            # Construct footer
            fails = str(total - passes)
            passes = str(passes)
            final_result = "PASS"
            if fails != "0":
                final_result = "FAIL"
            footer_str = (
            "----------------------------------------\n"
            "-------------Test Complete--------------\n"
            "----------------------------------------\n"
            f"Passes: {passes}\n"
            f"Fails:  {fails}\n"
            f"Result: {final_result}\n"
            "----------------------------------------")
            f.write("\n" + footer_str)