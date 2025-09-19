import time
import datetime
import pytz

# --- RPi specific imports ---
try:
    import RPi.GPIO as GPIO
    import smbus2
    import board
    import busio
    import adafruit_mcp4725
    RPI_MODE = True
    print("Running in Raspberry Pi mode.")
except ImportError:
    print("WARNING: RPi or Adafruit libraries not found. Running in simulation mode.")
    RPI_MODE = False

# --- Hardware Configuration ---
RELAY_PIN = 6
BOOST_PIN = 13
I2C_BUS = 1
DS3231_ADDRESS = 0x68
MCP4725_ADDRESS = 0x62 # Default I2C address for the MCP4725

# --- Cycle Configuration ---
CYCLE_MINUTES = 5
PUMP_DURATION_SEC = 270 # Pump for 4 minutes and 30 seconds
SPARKS_PER_CYCLE = 5

# --- RTC Helper Functions ---
def bcd_to_dec(bcd):
    return (bcd // 16 * 10) + (bcd % 16)

def get_rtc_datetime():
    """Reads time from a DS3231 RTC module. Falls back to system time on error or simulation."""
    if not RPI_MODE:
        return datetime.datetime.now()
    try:
        bus = smbus2.SMBus(I2C_BUS)
        time_data = bus.read_i2c_block_data(DS3231_ADDRESS, 0, 7)
        bus.close()
        sec = bcd_to_dec(time_data[0] & 0x7F)
        minute = bcd_to_dec(time_data[1])
        hour = bcd_to_dec(time_data[2] & 0x3F)
        date = bcd_to_dec(time_data[4])
        month = bcd_to_dec(time_data[5] & 0x1F)
        year = bcd_to_dec(time_data[6]) + 2000
        return datetime.datetime(year, month, date, hour, minute, sec)
    except Exception as e:
        print(f"Warning: Could not read from RTC ({e}). Falling back to system time.")
        return datetime.datetime.now()

# --- RPi Controller ---
class RPIController:
    def __init__(self):
        self.dac = None
        if RPI_MODE:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(RELAY_PIN, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(BOOST_PIN, GPIO.OUT, initial=GPIO.LOW)
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.dac = adafruit_mcp4725.MCP4725(i2c, address=MCP4725_ADDRESS)
                self.dac.normalized_value = 0.0
                print("GPIO and MCP4725 DAC initialized.")
            except Exception as e:
                print(f"FATAL: Could not initialize MCP4725 DAC. Error: {e}")
                self.dac = None

    def set_pump(self, state):
        if RPI_MODE and self.dac:
            self.dac.normalized_value = 1.0 if state else 0.0
        print(f"  - Pump DAC set to {'ON' if state else 'OFF'}")

    def spark(self, count):
        print(f"  - Sparking {count} times...")
        for _ in range(count):
            if RPI_MODE:
                GPIO.output(BOOST_PIN, GPIO.HIGH)
                time.sleep(0.1)
                GPIO.output(RELAY_PIN, GPIO.HIGH)
                time.sleep(0.05)
                GPIO.output(RELAY_PIN, GPIO.LOW)
                GPIO.output(BOOST_PIN, GPIO.LOW)
                time.sleep(1) # Wait between sparks
            else:
                time.sleep(1.15) # Simulate spark time
        print("  - Sparking complete.")

    def cleanup(self):
        if RPI_MODE:
            self.set_pump(False)
            GPIO.cleanup()
        print("\nHardware cleaned up. Exiting.")


def run_monitoring_loop(controller):
    """Main loop for the 5-minute monitoring cycle."""
    pst = pytz.timezone('America/Los_Angeles')
    
    while True:
        # 1. Calculate next cycle time
        now_naive = get_rtc_datetime()
        now = pst.localize(now_naive)
        
        minutes_past_hour = now.minute
        minute_offset = minutes_past_hour % CYCLE_MINUTES
        
        next_cycle_time = now - datetime.timedelta(minutes=minute_offset, seconds=now.second, microseconds=now.microsecond) \
                          + datetime.timedelta(minutes=CYCLE_MINUTES)
        
        spark_start_time = next_cycle_time - datetime.timedelta(seconds=(CYCLE_MINUTES * 60 - PUMP_DURATION_SEC))
        
        print(f"\n[{now.strftime('%H:%M:%S')}] New cycle started. Next cycle at {next_cycle_time.strftime('%H:%M:%S')}.")

        # 2. Pumping phase
        controller.set_pump(True)
        print(f"  - Pumping until {spark_start_time.strftime('%H:%M:%S')}...")
        while pst.localize(get_rtc_datetime()) < spark_start_time:
            time.sleep(1)

        # 3. Sparking phase
        controller.set_pump(False)
        controller.spark(SPARKS_PER_CYCLE)

        # 4. Wait for next cycle
        now = pst.localize(get_rtc_datetime())
        print(f"  - Cycle complete. Waiting for next start at {next_cycle_time.strftime('%H:%M:%S')}...")
        while pst.localize(get_rtc_datetime()) < next_cycle_time:
            time.sleep(1)


if __name__ == '__main__':
    controller = RPIController()
    try:
        run_monitoring_loop(controller)
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down...")
    finally:
        controller.cleanup()
