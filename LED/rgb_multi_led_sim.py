
from IPython.display import display, HTML, clear_output
import time

# Dictionary to store current LED states
led_states = {}

# Set a single RGB LED color
def set_color(led_id, r, g, b):
    led_states[led_id] = (int(r), int(g), int(b))
    render_leds()

# Blink a single RGB LED
def blink(led_id, r, g, b, repeat=3, delay=0.5):
    for _ in range(repeat):
        set_rgb_color(led_id, r, g, b)
        time.sleep(delay)
        set_rgb_color(led_id, 0, 0, 0)
        time.sleep(delay)

# Display all LEDs as colored circles
def render_leds():
    clear_output(wait=True)
    html = ""
    for i in sorted(led_states):
        color = f"rgb{led_states[i]}"
        html += f"<div style='display:inline-block;margin:10px;'>"
        html += f"<div style='width:80px;height:80px;border-radius:50%;background:{color};box-shadow:0 0 10px {color};'></div>"
        html += f"<div style='text-align:center;font-family:sans-serif;'>LED {i}</div></div>"
    display(HTML(html))
