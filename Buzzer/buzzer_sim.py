
import numpy as np
from IPython.display import Audio, display

def generate_buzzer_wave(freq=440, duration=1, pause=0.5, repeat=3, rate=44100):
    tone_samples = int(rate * duration)
    pause_samples = int(rate * pause)
    
    # Generate tone and silence
    t = np.linspace(0, duration, tone_samples, False)
    tone = 0.5 * np.sin(2 * np.pi * freq * t)
    silence = np.zeros(pause_samples)
    
    # Combine tone + pause for repeat count
    waveform = np.concatenate([(np.concatenate([tone, silence])) for _ in range(repeat)])
    return Audio(waveform, rate=rate, autoplay=True)

def play_buzzer_sim(freq, duration, pause, repeat):
    display(generate_buzzer_wave(freq, duration, pause, repeat))
