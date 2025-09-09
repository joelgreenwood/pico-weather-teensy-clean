import machine
import utime
from machine import Pin, I2C
import struct

from display import EPD_2in13_V3_Landscape
from images import show_image, IMAGE_DIM
from render import DisplayController
from utils import read_config, sentence_join, wrap_text, truncate_lines
from weather import get_img_for_title, Weather, load_cached_weather, fetch_weather, cache_weather

# ---------------- Teensy I2C ----------------
TEENSY_I2C_ADDRESS = 0x08
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)

def c_to_f(c):
    return c * 9 / 5 + 32

def wait_for_teensy():
    """Wait until Teensy appears on the I2C bus."""
    print("Scanning for Teensy...")
    while True:
        devices = i2c.scan()
        if TEENSY_I2C_ADDRESS in devices:
            print(f"Teensy detected at 0x{TEENSY_I2C_ADDRESS:02X}")
            return
        print("Teensy not found, retrying in 1 second...")
        utime.sleep(1)

def send_temp_to_teensy(celsius_temp):
    """Send temperature to Teensy with exponential backoff on failure."""
    temp_f = c_to_f(celsius_temp)
    temp_int = int(temp_f * 10)
    data = struct.pack(">H", temp_int)

    delay = 0.02
    max_attempts = 3

    while True:
        for attempt in range(max_attempts):
            try:
                i2c.writeto(TEENSY_I2C_ADDRESS, data)
                print(f"Sent {temp_f:.1f} F as {temp_int} (0x{temp_int:04X})")
                return True
            except OSError as e:
                if e.args[0] in (5, 110):  # EIO or ETIMEDOUT
                    print(f"I2C attempt {attempt+1} failed: {e}, retrying in {delay:.2f}s")
                    utime.sleep(delay)
                else:
                    print(f"I2C fatal error: {e}")
                    return False

        print(f"Teensy not responding after {max_attempts} attempts, backing off for {delay:.1f}s")
        utime.sleep(delay)
        delay = min(delay * 2, 10)  # exponential backoff capped at 10s

# ---------------- Weather render helpers ----------------
def render_weather(display: DisplayController, weather: Weather, show_min_max: bool = False):
    image_x = 0
    image_y = display.get_last_text_y() + 7

    img_paths = [get_img_for_title(title) for title in weather.titles if get_img_for_title(title)]
    for img_path in set(img_paths):
        show_image(display, img_path, image_x, image_y)
        image_x += IMAGE_DIM + 4

    temp_f = c_to_f(weather.temp.main)
    temp = f"{temp_f:.1f} F"
    title = sentence_join(weather.titles)
    desc = wrap_text(weather.description, DisplayController.MAX_TEXT_WIDTH)

    display.display_text_at_coordinates(DisplayController.RENDER_FLAG_APPEND_ONLY, image_x, temp)
    if show_min_max:
        temp_min_f = c_to_f(weather.temp.temp_min)
        temp_max_f = c_to_f(weather.temp.temp_max)
        display.display_right(DisplayController.RENDER_FLAG_APPEND_ONLY, f"L/H: {temp_min_f:.1f}-{temp_max_f:.1f} F")

    display.display_text_at_coordinates(DisplayController.RENDER_FLAG_APPEND_ONLY, image_x, title, *desc)

def render(display: DisplayController, current: Weather, daily: Weather):
    display.display_text(DisplayController.RENDER_FLAG_CLEAR | DisplayController.RENDER_FLAG_BLANK, "NOW")
    render_weather(display, current)
    display.render_horizontal_separator()
    display.display_text(DisplayController.RENDER_FLAG_APPEND_ONLY, "TODAY")
    display.add_vertical_space(2)
    display.display_text(DisplayController.RENDER_FLAG_APPEND_ONLY, *truncate_lines(daily.day_summary, 3))
    display.add_vertical_space(4)
    render_weather(display, daily, show_min_max=True)
    display.flush_display()

# ---------------- Weather fetch ----------------
def fetch(config, display):
    current = load_cached_weather('current', config.cache_mins)
    daily = load_cached_weather('daily', config.cache_mins)

    if all([current, daily]):
        print("Using cached weather")
        return current, daily

    display.display_text(DisplayController.RENDER_FLAG_BLANK | DisplayController.RENDER_FLAG_FLUSH,
                         f"Connecting to {config.ssid}...")

    # network connect
    from net import connect_to_network, disconnect
    wlan, ip = connect_to_network(config.ssid, config.password)
    display.display_text(DisplayController.RENDER_FLAG_FLUSH, "Connected", f"IP: {ip}")

    try:
        current, daily = fetch_weather(config.lat, config.lon, config.openweathermap_key)
    except Exception as e:
        print(f"Error fetching weather: {e}")
        display.display_text(DisplayController.RENDER_FLAG_FLUSH, "Failed to fetch weather", f"Cause: {e}")
        display.deep_sleep()
    finally:
        disconnect(wlan)
        if not all([current, daily]):
            utime.sleep(300)
            machine.reset()

    cache_weather(current, 'current')
    cache_weather(daily, 'daily')
    return current, daily

# ---------------- Main ----------------
def main():
    config = read_config()
    epd = EPD_2in13_V3_Landscape()
    display = DisplayController(epd)

    # Wait for Teensy before entering main loop
    wait_for_teensy()

    TEENSY_SEND_INTERVAL = 5  # seconds
    DISPLAY_REFRESH_INTERVAL = config.refresh_mins * 60  # seconds

    last_teensy_send = 0
    last_display_refresh = 0

    current = None
    daily = None

    while True:
        now = utime.time()

        # Send temp to Teensy
        if current and now - last_teensy_send >= TEENSY_SEND_INTERVAL:
            send_temp_to_teensy(current.temp.main)
            last_teensy_send = now

        # Fetch weather & update display
        if now - last_display_refresh >= DISPLAY_REFRESH_INTERVAL or current is None:
            display.init()
            current, daily = fetch(config, display)
            render(display, current, daily)
            display.deep_sleep()
            last_display_refresh = now

        utime.sleep(0.05)  # small sleep to yield CPU

if __name__ == "__main__":
    main()
