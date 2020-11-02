import tts_stt
import time

#TTS keys:
api_key_tts = "xYp7XRVQ5BMDmd0iUT9jU1ajLFQYggd535_S0wTuundt"
api_url_tts = "https://api.eu-de.text-to-speech.watson.cloud.ibm.com/instances/1577753b-d45e-4545-977d-d7f6b6a17295"

#STT key:
api_key_stt = "Mw7LV3RBAXxIdeJZ3NjKjWlzwBG4Q4ifgwMMHosuUipb"
api_url_stt = "https://api.eu-de.speech-to-text.watson.cloud.ibm.com/instances/c916a869-a2c0-4c87-8acc-95e92f93f53c"



def main():
    tts = tts_stt.TTS(api_key_tts, api_url_tts)
    stt = tts_stt.STT(api_key_stt, api_url_stt, 3, echo=False, phrase_time_limit=20)    

    #tts.synth_to_play("Please say something, and I will repeat it ", block=True)

    stt.start_background_listening()

    print("Say something")

    done = False
    while(not done):
        time.sleep(0.1)
        if(stt.has_string_in_queue()):
            string = stt.get_string_from_queue()
            print("Heard something: " + string)
            #use block=True to ensure program does not stop before audio is done playing
            tts.synth_to_play(string, block=True)
            done = True


if( __name__ == "__main__"):
    main()


