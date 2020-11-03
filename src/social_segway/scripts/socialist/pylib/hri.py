import talker as hri

#TTS keys:
api_key_tts = "xYp7XRVQ5BMDmd0iUT9jU1ajLFQYggd535_S0wTuundt"
api_url_tts = "https://api.eu-de.text-to-speech.watson.cloud.ibm.com/instances/1577753b-d45e-4545-977d-d7f6b6a17295"

#STT key:
api_key_stt = "Mw7LV3RBAXxIdeJZ3NjKjWlzwBG4Q4ifgwMMHosuUipb"
api_url_stt = "https://api.eu-de.speech-to-text.watson.cloud.ibm.com/instances/c916a869-a2c0-4c87-8acc-95e92f93f53c"


def main():
    talker = hri.Talker(api_key_tts, api_url_tts, api_key_stt, api_url_stt)

    output = talker.get_room_with_change()
    output = talker.place_object_on_robot("cup")
    output = talker.place_object_on_furniture("cup", "table")

    print("")
    print(output)



if __name__ == "__main__":
    main()