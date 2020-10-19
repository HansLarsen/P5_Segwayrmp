
import numpy as np
import simpleaudio as sa
import wave


freq = 1000

sample_rate = 44100
T = 0.1
t = np.linspace(0, T, T*sample_rate, False)

note = np.sin(freq * t * 2 * np.pi)

silence = np.zeros(2000)

audio = np.hstack(note)
audio *= 32767 / np.max(np.abs(audio))
audio = audio.astype(np.int16)

audio2 = np.hstack((note, silence, note))
audio2 *= 32767 / np.max(np.abs(audio2))
audio2 = audio2.astype(np.int16)

beep2 = sa.WaveObject(audio2, 1, 2, sample_rate)
beep = sa.WaveObject(audio, 1, 2, sample_rate)

with wave.open("beep.wav", "w") as f:
    f.setnchannels(1)
    f.setsampwidth(2)
    f.setframerate(sample_rate)
    f.writeframes(audio.tobytes())


with wave.open("beep2.wav", "w") as f:
    f.setnchannels(1)
    f.setsampwidth(2)
    f.setframerate(sample_rate)
    f.writeframes(audio2.tobytes())

