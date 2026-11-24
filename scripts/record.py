import sounddevice as sd
import numpy as np
from scipy.io.wavfile import write

# Simple program to record audio files for the TONOR G11 microphone
sd.default.device = 0
fs = 44100

duration = 6  # seconds
# myrecording = sd.rec(int(duration * fs), samplerate=fs, channels=1)

sd.default.samplerate = fs
sd.default.channels = 1

myrecording = sd.rec(int(duration * fs))
sd.wait()

recordings = [
    "mono_1.wav",
    "mono_2.wav",
    "vary_1.wav",
    "vary_2.wav",
    "noisy_1.wav",
    "noisy_2.wav",
    "loud_1.wav",
    "loud_2.wav",
    "quiet_1.wav",
    "quiet_2.wav"
]

reco = [
    "run_1.wav",
    "run_2.wav",
    "run_3.wav",
    "run_4.wav",
    "run_5.wav",
    "run_6.wav",
    "run_7.wav",
    "run_8.wav",
    "run_9.wav",
    "run_10.wav",
    "run_11.wav",
    "run_12.wav",
    "run_13.wav",
    "run_14.wav",
    "run_15.wav",
]

write(reco[14], fs, myrecording)



