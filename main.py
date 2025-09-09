import machine
import utime
from machine import Pin, I2C, UART
import struct

from display import EPD_2in13_V3_Landscape
from images import show_image, IMAGE_DIM
from net import connect_to_network, disconnect
from render import DisplayController
from utils import format_date, read_config, wrap_text, sentence_join, Config, truncate_lines
from weather import get_img_for_title, Weather, load_cached_weather, fetch_weather, cache_weather


# --- UART setup for Teensy ---
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))  # GP0=TX, GP1=RX

def c_to_f(c):
    return c * 9 / 5 + 32

def send_temp_to_teensy(celsius_temp):
    """
    Convert Celsius temp to Fahrenheit, scale by 10, and send over UART.
    Format: "T=###\n"
    """
    temp_f = c_to_f(celsius_temp)
    tenths = int(temp_f * 10)
    msg = "T={}\n".format(tenths)
    try:
        uart.write(msg)
        print(f"Sent over UART: {msg.strip()}")
    except Exception as e:
        print(f"UART send failed: {e}")


# --- Weather fetch ---
def fetch(config: Config, display: DisplayController) -> tuple[Weather, Weather]:
    current: Weather = load_cached_weather('current', config.cache_mins)
    daily: Weather = load_cached_weather('daily', config.cache_mins)

    if all([current, daily]):
        print(f"using cached weather")
        return current, daily

    display.display_text(
        DisplayController.RENDER_FLAG_BLANK | DisplayController.RENDER_FLAG_FLUSH,
        f"Connecting to {config.ssid}..."
    )

    try:
        wlan, ip = connect_to_network(config.ssid, config.password)
    except KeyboardInterrupt:
        print('received keyboard interrupt when connecting to network')
        machine.reset()

    display.display_text(
        DisplayController.RENDER_FLAG_FLUSH,
        "Connected",
        f"IP: {ip}"
    )

    try:
        current, daily = fetch_weather(config.lat, config.lon, config.openweathermap_key)
    except Exception as e:
        print(f"error fetching weather: {e}")
        display.display_text(
            DisplayController.RENDER_FLAG_FLUSH,
            "Failed to fetch weather",
            f"Cause: {e}"
        )
        display.deep_sleep()
    finally:
        disconnect(wlan)
        if not all([current, daily]):
            print(f"Sleeping for 5 minutes then resetting the device")
            utime.sleep(300)
            machine.reset()

    cache_weather(current, 'current')
    cache_weather(daily, 'daily')

    return current, daily


# --- Render helpers ---
def render_weather(display: DisplayController, weather: Weather, show_min_max: bool = False):
    image_x = 0
    image_y = display.get_last_text_y() + 7

    img_paths = []
    for title in weather.titles:
        img_path = get_img_for_title(title)
        if img_path:
            img_paths.append(img_path)

    for img_path in set(img_paths):
        show_image(display, img_path, image_x, image_y)
        image_x += IMAGE_DIM + 4

    temp_f = c_to_f(weather.temp.main)
    temp = f"{temp_f:.1f} F"
    title = sentence_join(weather.titles)
    desc = wrap_text(weather.description, DisplayController.MAX_TEXT_WIDTH)

    display.display_text_at_coordinates(
        DisplayController.RENDER_FLAG_APPEND_ONLY,
        image_x,
        temp,
    )

    if show_min_max:
        temp_min_f = c_to_f(weather.temp.temp_min)
        temp_max_f = c_to_f(weather.temp.temp_max)
        display.display_right(
            DisplayController.RENDER_FLAG_APPEND_ONLY,
            f"L/H: {temp_min_f:.1f}-{temp_max_f:.1f} F"
        )

    display.display_text_at_coordinates(
        DisplayController.RENDER_FLAG_APPEND_ONLY,
        image_x,
        title,
        *desc,
    )


def render(display: DisplayController, current: Weather, daily: Weather):
    weather_date = format_date(current.dt)

    display.display_text(
        DisplayController.RENDER_FLAG_CLEAR | DisplayController.RENDER_FLAG_BLANK | DisplayController.RENDER_FLAG_THIN_PADDING,
        "NOW"
    )
    display.display_right(
        DisplayController.RENDER_FLAG_APPEND_ONLY | DisplayController.RENDER_FLAG_NO_V_CURSOR,
        weather_date
    )
    render_weather(display, current)

    display.render_horizontal_separator()
    display.display_text(DisplayController.RENDER_FLAG_APPEND_ONLY, "TODAY")
    display.add_vertical_space(2)

    today_summary = truncate_lines(daily.day_summary, 3)
    display.display_text(
        DisplayController.RENDER_FLAG_APPEND_ONLY,
        *today_summary
    )
    display.add_vertical_space(4)
    render_weather(display, daily, show_min_max=True)

    display.flush_display()


# --- Main loop ---
def main():
    config = read_config()
    epd = EPD_2in13_V3_Landscape()
    display = DisplayController(epd)

    TEENSY_SEND_INTERVAL = 5        # seconds
    DISPLAY_REFRESH_INTERVAL = config.refresh_mins * 60
    last_teensy_send = 0
    last_display_refresh = 0

    current = None
    daily = None

    while True:
        now = utime.time()

        # --- Teensy update every 5 sec ---
        if current and (now - last_teensy_send >= TEENSY_SEND_INTERVAL):
            send_temp_to_teensy(current.temp.main)
            last_teensy_send = now

        # --- Display update on refresh interval ---
        if now - last_display_refresh >= DISPLAY_REFRESH_INTERVAL or current is None:
            display.init()
            current, daily = fetch(config, display)
            render(display, current, daily)
            display.deep_sleep()
            last_display_refresh = now

        # --- tiny sleep to yield CPU ---
        utime.sleep(0.05)


if __name__ == '__main__':
    main()
