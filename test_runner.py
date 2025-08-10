#!/usr/bin/env python3
"""
ESP32 Unity Test Runner for CI/CD
Flashes firmware, captures test output, and generates JUnit XML
"""

import argparse
import serial
import subprocess
import time
import re
import sys
from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom

class ESP32TestRunner:
    def __init__(self, port, baud_rate=115200, timeout=120, unity_cmd='*', auto_trigger=True):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.test_results = []
        self.unity_cmd = unity_cmd
        self.auto_trigger = auto_trigger
        self.test_summary = None  # Store the final test summary
        self.timed_out = False    # Track if test run timed out

    def flash_firmware(self, firmware_path):
        """Flash the test firmware to ESP32"""
        print(f"Flashing {firmware_path} to {self.port}")
        
        cmd = [
            "idf.py", "-C", "unity-app",
            "-DTESTS_ALL=1", "flash"
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Flash failed: {result.stderr}")
            return False
            
        print("Flash successful")
        time.sleep(2)  # Wait for ESP32 to restart
        return True
        
    def run_tests(self):
        """Connect to ESP32 and capture test output"""
        print(f"Connecting to {self.port} at {self.baud_rate} baud")
        
        try:
            with serial.Serial(self.port, self.baud_rate, timeout=2) as ser:
                print(f"Serial connection established. DTR: {ser.dtr}, RTS: {ser.rts}")
                
                # Try different reset sequences
                print("Attempting ESP32 reset...")
                
                # Method 1: Standard reset
                ser.dtr = False
                ser.rts = True
                time.sleep(0.1)
                ser.rts = False
                time.sleep(1)
                
                # Check for any immediate output
                ser.timeout = 0.1
                initial_data = ser.read(100)
                if initial_data:
                    print(f"Initial data received: {initial_data}")
                
                # Method 2: Try EN pin reset if no response
                if not initial_data:
                    print("No initial response, trying alternative reset...")
                    ser.dtr = True
                    time.sleep(0.1)
                    ser.dtr = False
                    time.sleep(1)
                
                # Set longer timeout for reading
                ser.timeout = 2
                ser.flushInput()  # Clear any remaining data
                
                # Send Unity test trigger commands
                if self.auto_trigger:
                    print(f"Sending Unity test commands: '{self.unity_cmd}' + ENTER")
                    time.sleep(0.5)  # Wait for ESP32 to be ready
                    ser.write(self.unity_cmd.encode())   # Send Unity command
                    time.sleep(0.1)
                    ser.write(b'\r\n')  # Send ENTER
                    time.sleep(0.1)
                else:
                    print("Auto-trigger disabled, waiting for manual test start...")
                
                # Capture output
                output_lines = []
                start_time = time.time()
                tests_started = False
                last_output_time = start_time
                no_output_warning_shown = False
                unity_menu_seen = False
                
                print("Waiting for test output...")
                
                while time.time() - start_time < self.timeout:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        
                        if line:
                            print(f"[{time.time() - start_time:.1f}s] {line}")
                            output_lines.append(line)
                            last_output_time = time.time()
                            no_output_warning_shown = False
                            
                            # Check for Unity menu/prompt
                            if "Enter test for running." in line:
                                unity_menu_seen = True
                                if not tests_started and self.auto_trigger:
                                    print("Unity prompt detected: 'Enter test for running.' - sending commands...")
                                    time.sleep(0.2)  # Small delay to ensure ESP32 is ready
                                    ser.write(self.unity_cmd.encode())   # Run tests
                                    time.sleep(0.1)
                                    ser.write(b'\r\n')  # ENTER
                                    print(f"Sent: '{self.unity_cmd}' + ENTER")
                            
                            # Also check for other Unity menu patterns (fallback)
                            elif any(pattern in line.lower() for pattern in [
                                "press", "enter", "menu", "select", "*", "run all"
                            ]):
                                unity_menu_seen = True
                                if not tests_started and self.auto_trigger:
                                    print("Unity menu detected, sending commands...")
                                    ser.write(self.unity_cmd.encode())   # Run tests
                                    time.sleep(0.1)
                                    ser.write(b'\r\n')  # ENTER
                            
                            # Look for Unity test patterns (more flexible)
                            if any(pattern in line.lower() for pattern in [
                                "unity", "test", "pass", "fail", "ignore", 
                                "run_test", "setup()", "teardown()", "running"
                            ]):
                                tests_started = True
                                
                            # Check for test completion
                            if tests_started and any(pattern in line for pattern in [
                                "FINAL", "Tests ", "Failures ", "Ignored", 
                                "-----------------------", "UNITY END"
                            ]):
                                if self._is_test_complete(output_lines):
                                    print("Tests completed successfully")
                                    break
                                    
                            # Emergency exit on certain patterns
                            if any(pattern in line for pattern in [
                                "Guru Meditation Error", "abort()", "CORRUPTED", 
                                "Fatal exception", "panic"
                            ]):
                                print("ESP32 crashed during testing")
                                break
                        
                        else:
                            # No data received
                            current_time = time.time()
                            if current_time - last_output_time > 5 and not no_output_warning_shown:
                                elapsed = current_time - start_time
                                print(f"Warning: No output for 5 seconds (total time: {elapsed:.1f}s, lines: {len(output_lines)})")
                                
                                # If we haven't seen Unity menu and no tests started, try sending commands
                                if not unity_menu_seen and not tests_started and elapsed > 3 and self.auto_trigger:
                                    print(f"Trying to trigger Unity tests with '{self.unity_cmd}'...")
                                    ser.write(self.unity_cmd.encode())
                                    time.sleep(0.1)
                                    ser.write(b'\r\n')
                                    time.sleep(0.1)
                                    ser.write(b'\n')  # Try just newline too
                                
                                if current_time - last_output_time > 15:
                                    print("This might indicate:")
                                    print("  - ESP32 is not running test firmware")
                                    print("  - Wrong serial port or baud rate")
                                    print("  - ESP32 is in bootloader mode")
                                    print("  - Hardware connection issue")
                                    no_output_warning_shown = True
                                
                    except serial.SerialTimeoutException:
                        continue
                    except KeyboardInterrupt:
                        print("\nInterrupted by user")
                        break
                
                # Check if we timed out
                if time.time() - start_time >= self.timeout:
                    print(f"\nTIMEOUT: Tests exceeded maximum runtime of {self.timeout} seconds")
                    self.timed_out = True
                        
                print(f"\nTest run completed. Captured {len(output_lines)} lines of output.")
                
                # If we got no output, show debugging info
                if not output_lines:
                    print("\nDEBUGGING INFO:")
                    print(f"Port: {self.port}")
                    print(f"Baud rate: {self.baud_rate}")
                    print(f"Is port open: {ser.is_open}")
                    print("Try manually connecting with: screen /dev/ttyUSB1 115200")
                    print("Or: python3 -m serial.tools.miniterm /dev/ttyUSB1 115200")
                    return None
                    
                # Parse results
                self._parse_unity_output(output_lines)
                return output_lines
                
        except serial.SerialException as e:
            print(f"Serial connection failed: {e}")
            return None
            
    def _is_test_complete(self, lines):
        """Check if all tests have completed"""
        # Look for Unity summary line
        for line in reversed(lines[-10:]):  # Check last 10 lines
            if re.search(r'\d+ Tests \d+ Failures \d+ Ignored', line):
                return True
        return False
        
    def _parse_unity_output(self, lines):
        """Parse Unity test output and extract results"""
        current_test = None
        
        for line in lines:
            # Look for test summary line: "n Tests m Failures k Ignored"
            summary_match = re.search(r'(\d+)\s+Tests?\s+(\d+)\s+Failures?\s+(\d+)\s+Ignored', line)
            if summary_match:
                tests_count = int(summary_match.group(1))
                failures_count = int(summary_match.group(2))
                ignored_count = int(summary_match.group(3))
                self.test_summary = {
                    'tests': tests_count,
                    'failures': failures_count,
                    'ignored': ignored_count
                }
                print(f"Found test summary: {tests_count} Tests {failures_count} Failures {ignored_count} Ignored")
            
            # Test start: TEST_FILE:LINE:test_function_name:PASS/FAIL
            match = re.match(r'(.+?):(\d+):(.+?):(PASS|FAIL|IGNORE)', line)
            if match:
                file_name, line_no, test_name, result = match.groups()
                
                test_case = {
                    'name': test_name,
                    'file': file_name,
                    'line': int(line_no),
                    'result': result,
                    'message': '',
                    'time': 0.0  # Unity doesn't provide timing by default
                }
                
                # Look for failure message in next few lines
                if result == 'FAIL':
                    # Find the assertion message
                    for next_line in lines[lines.index(line):lines.index(line)+5]:
                        if 'Expected' in next_line or 'Actual' in next_line:
                            test_case['message'] = next_line.strip()
                            break
                
                self.test_results.append(test_case)
                
        print(f"Parsed {len(self.test_results)} test results")

    def get_test_status(self):
        """
        Return pass/fail status based on test results.
        Returns True if tests passed, False if failed or timed out.
        """
        # Return False if timed out
        if self.timed_out:
            return False
            
        # If we have a test summary, use that
        if self.test_summary:
            return self.test_summary['failures'] == 0
            
        # Fallback: check individual test results
        if self.test_results:
            failures = sum(1 for t in self.test_results if t['result'] == 'FAIL')
            return failures == 0
            
        # No test results found - consider this a failure
        return False
        
    def generate_junit_xml(self, output_file):
        """Generate JUnit XML from test results"""
        root = Element('testsuites')
        testsuite = SubElement(root, 'testsuite')
        
        total_tests = len(self.test_results)
        failures = sum(1 for t in self.test_results if t['result'] == 'FAIL')
        skipped = sum(1 for t in self.test_results if t['result'] == 'IGNORE')
        
        testsuite.set('name', 'ESP32_Unity_Tests')
        testsuite.set('tests', str(total_tests))
        testsuite.set('failures', str(failures))
        testsuite.set('skipped', str(skipped))
        testsuite.set('time', '0')
        
        for test in self.test_results:
            testcase = SubElement(testsuite, 'testcase')
            testcase.set('classname', test['file'])
            testcase.set('name', test['name'])
            testcase.set('time', str(test['time']))
            
            if test['result'] == 'FAIL':
                failure = SubElement(testcase, 'failure')
                failure.set('message', test['message'])
                failure.text = f"Test failed at {test['file']}:{test['line']}"
                
            elif test['result'] == 'IGNORE':
                skipped_elem = SubElement(testcase, 'skipped')
                skipped_elem.set('message', 'Test ignored')
        
        # Pretty print XML
        rough_string = tostring(root, 'utf-8')
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent='  ')
        
        with open(output_file, 'w') as f:
            f.write(pretty_xml)
            
        print(f"JUnit XML written to {output_file}")


def test_basic_connection(port, baud_rate):
    """Quick connection test"""
    try:
        with serial.Serial(port, baud_rate, timeout=1) as ser:
            print(f"✓ Successfully opened {port}")
            ser.dtr = False
            ser.rts = True
            time.sleep(0.1)
            ser.rts = False
            time.sleep(1)
            
            data = ser.read(100)
            if data:
                print(f"✓ ESP32 responding ({len(data)} bytes)")
                return True
            else:
                print("✗ No response from ESP32")
                return False
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description='ESP32 Unity Test Runner')
    parser.add_argument('--port', required=True, help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--firmware', required=True, help='Firmware binary path')
    parser.add_argument('--output', default='results.xml', help='Output JUnit XML file')
    parser.add_argument('--timeout', type=int, default=120, help='Test timeout in seconds')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--skip-flash', action='store_true', help='Skip firmware flashing')
    parser.add_argument('--diagnostic', action='store_true', help='Run connection diagnostic first')
    parser.add_argument('--unity-cmd', default='*', help='Unity command to run tests (default: *)')
    parser.add_argument('--no-auto-trigger', action='store_true', help='Do not send Unity commands automatically')

    args = parser.parse_args()

    # Run diagnostic if requested
    if args.diagnostic:
        print("Running connection diagnostic...")
        test_basic_connection(args.port, args.baud)
        print("-" * 50)

    runner = ESP32TestRunner(
        args.port, 
        args.baud, 
        args.timeout, 
        args.unity_cmd, 
        not args.no_auto_trigger
    )

    # Flash firmware (unless skipped)
    if not args.skip_flash:
        if not runner.flash_firmware(args.firmware):
            print("HINT: Use --skip-flash if firmware is already loaded")
            print("RESULT: FAIL")
            sys.exit(1)
    else:
        print("Skipping firmware flash (using existing firmware)")

    # Run tests
    try:
        output = runner.run_tests()
        if output is None:
            print("\nFailed to get test output. Try:")
            print(f"  python3 esp32_diagnostic.py --port {args.port}")
            print("RESULT: FAIL")
            sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nTest execution interrupted by user")
        print("RESULT: FAIL")
        sys.exit(1)

    # Save raw output
    with open('test_output.log', 'w') as f:
        f.write('\n'.join(output))

    # Generate XML
    runner.generate_junit_xml(args.output)

    # Print summary
    if runner.test_results:
        passed = sum(1 for t in runner.test_results if t['result'] == 'PASS')
        failed = sum(1 for t in runner.test_results if t['result'] == 'FAIL')
        ignored = sum(1 for t in runner.test_results if t['result'] == 'IGNORE')
        print(f"\n=== TEST SUMMARY ===")
        print(f"Passed: {passed}")
        print(f"Failed: {failed}") 
        print(f"Ignored: {ignored}")
        print(f"Total: {len(runner.test_results)}")

    # Determine pass/fail and exit with appropriate code
    test_passed = runner.get_test_status()

    if test_passed:
        print("RESULT: PASS")
        sys.exit(0)
    else:
        print("RESULT: FAIL")
        if runner.timed_out:
            print("Reason: Test execution timed out")
        elif runner.test_summary and runner.test_summary['failures'] > 0:
            print(f"Reason: {runner.test_summary['failures']} test failures")
        else:
            print("Reason: No valid test results found or test failures")
        sys.exit(1)


if __name__ == '__main__':
    main()