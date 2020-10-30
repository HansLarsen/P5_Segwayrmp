# two classes wrapping calls to Watson for text-to-speech (TTS) and speech-to-text (STT)

# STT places heard inputs in a queue, from which they can be retrieved
# Note, due to mic_callback, STT spawns new threads for each request to watson
# so execution is not linear!


#TTS:
from tts_wrapper import WatsonTTS
import simpleaudio as sa

#STT:
import json 
from os.path import join, dirname 
from ibm_watson import SpeechToTextV1 
from ibm_watson.websocket import RecognizeCallback, AudioSource 
from ibm_cloud_sdk_core.authenticators import IAMAuthenticator 

import speech_recognition as sr
from queue import Queue


class TTS():
    def __init__(self, api_key, api_url):
        self.tts = WatsonTTS(api_key=api_key, api_url=api_url, lang="en-US")

    def synth_to_file(self, stringToSynth, outputFileName):
        self.tts.synth(stringToSynth, outputFileName)

    def synth_to_play(self, stringToPlay, tmpFileName='tmp.wav', block=False):
        self.tts.synth(stringToPlay, tmpFileName)
        wave_obj = sa.WaveObject.from_wave_file(tmpFileName)
        self.play_obj = wave_obj.play()
        if block:
            self.play_obj.wait_done()

    def is_playing_sound(self):
        return self.play_obj.is_playing()


class recognizeCallback(RecognizeCallback):
    def __init__(self, parent):
        RecognizeCallback.__init__(self)
        self.parent = parent

    def on_data(self, data):
        dic = json.loads( 
                json.dumps( data, indent=2)) 

        string = "" 
        # pull transcript from the Json file:
        while bool(dic.get('results')): 
            string = dic.get('results').pop().get('alternatives').pop().get('transcript')+string[:] 

        self.parent.add_string_to_queue(string)

    def on_error(self, error):
        print('recognizeCallback Error received: {}'.format(error))

    def on_inactivity_timeout(self, error):
        print('recognizeCallback Inactivity timeout: {}'.format(error))



class STT():


    def __init__(self, api_key, api_url, noise_adjust_time = 0, echo=False, phrase_time_limit=None):
        # Watson cloud authentication:
        authenticator = IAMAuthenticator(api_key)  
        self.service = SpeechToTextV1(authenticator = authenticator) 
        self.service.set_service_url(api_url)

        #callback class:
        self.audio_callback = recognizeCallback(self)

        #setup speech_recognition to capture audio from mic:
        self.recognizer = sr.Recognizer()
        self.mic = sr.Microphone()
        self.string_queue = Queue()
        if noise_adjust_time > 0:
            with self.mic as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=noise_adjust_time)

        self.echo = echo
        self.phrase_time_limit = phrase_time_limit
        self.background_listening = False

        self.beep = sa.WaveObject.from_wave_file("beep.wav")
        self.beep2 = sa.WaveObject.from_wave_file("beep2.wav")


    def adjust_for_ambient_noise(self, noise_adjust_time = 1):
        with self.mic as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=noise_adjust_time)


    def audioFile_to_queue(self, fileToRecognize):

        #open audio file and STT it:
        with open(join(dirname('__file__'), fileToRecognize),  
                'rb') as audio_file: 

                self.service.recognize_using_websocket(AudioSource(audio_file), 'audio/wav', self.audio_callback,   
                                model='en-US_NarrowbandModel', continuous=True)


    def mic_callback(self, recognizer, audioData):

        audioFile = "tmp.wav"
        self.stop_background_listening()

        with open(audioFile, "wb") as f:
            f.write(audioData.get_wav_data())

        if self.echo:
            wave_obj = sa.WaveObject.from_wave_file(audioFile)
            wave_obj.play() #use wave_obj.play().wait_done() if a blocking call is wanted, only useful for debug

        self.audioFile_to_queue(audioFile)


    def start_background_listening(self):
        self.stop_listening = self.recognizer.listen_in_background(self.mic, self.mic_callback, phrase_time_limit=self.phrase_time_limit)
        self.background_listening = True


    def stop_background_listening(self, wait_for_stop=False):
        try:
            self.stop_listening
        except NameError:
            print("Can't stop listening, as I am not listening!")
        else:
            self.stop_listening(wait_for_stop=wait_for_stop)

        self.background_listening = False


    def has_string_in_queue(self):
        return not self.string_queue.empty()


    def get_string_from_queue(self):
        if not self.string_queue.empty():
            return self.string_queue.get()
        else:
            return ""

    def add_string_to_queue(self, string):
        self.string_queue.put(string)


    def listen(self, seconds_to_listen):
        if self.background_listening:
            self.stop_background_listening()

        
        self.beep.play().wait_done()

        with self.mic as source:
            audio = self.recognizer.listen(source, timeout=seconds_to_listen, phrase_time_limit=seconds_to_listen)

        self.beep2.play()

        audioFile = "input.wav"
        with open(audioFile, "wb") as f:
            f.write(audio.get_wav_data())

        self.audioFile_to_queue(audioFile)

    

    