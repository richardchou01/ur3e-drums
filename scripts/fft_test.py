import numpy as np
from scipy.io import wavfile
from scipy.signal import find_peaks, stft
import time

from pathlib import Path

# Path to data files
audio_dir = Path("data")

wav_files = sorted(audio_dir.glob("*.wav"))

for index, file_path in enumerate(wav_files):
    print(f"{index}: {file_path.name} (Full path: {file_path})")
    # Load your audio file
    start = time.perf_counter()
    # sample_rate, data = wavfile.read("data/run_1.wav")
    sample_rate, data = wavfile.read(file_path)

    # Compute the Short-Time Fourier Transform (STFT)
    # nperseg controls the trade-off between time resolution and frequency resolution
    frequencies, times, Zxx = stft(data, fs=sample_rate, nperseg=1024)
    magnitudes = np.abs(Zxx)

    # Isolate your target frequency band (e.g., 950 Hz to 1050 Hz)
    target_low, target_high = 950, 1050
    freq_mask = (frequencies >= target_low) & (frequencies <= target_high)

    # Sum or average the energy in that band for each time window
    band_energy = magnitudes[freq_mask, :].mean(axis=0)

    # Find peaks in the energy envelope to locate event timestamps
    # Adjust 'height' (amplitude threshold) and 'distance' (minimum samples between events)
    peaks, _ = find_peaks(band_energy, height=np.max(band_energy) * 0.5, distance=10)

    # Convert peak indices directly to seconds
    event_timestamps = times[peaks]

    elapsed = time.perf_counter() - start
    print(f"Elapsed: {elapsed:.4f} seconds")

    for i, t in enumerate(event_timestamps):
        print(f"Hit {i+1} detected at {t:.3f} seconds")
