import tts_stt

class Talker:

    def __init__(self, api_key_tts, api_url_tts, api_key_stt, api_url_stt, debug=False):
        self.tts = tts_stt.TTS(api_key_tts, api_url_tts)
        self.stt = tts_stt.STT(api_key_stt, api_url_stt, noise_adjust_time = 0, phrase_time_limit=None)

        if not debug:
            self.tts.synth_to_play("Please be quiet for 3 seconds", block=True)
            self.stt.adjust_for_ambient_noise(3)
        
        self.rooms = list()
        self.rooms.append("kitchen")
        self.rooms.append("dining room")
        self.rooms.append("bedroom")
        self.rooms.append("living room")
        self.rooms.append("hallway")

        self.okay = list()
        self.okay.append("okay")
        self.okay.append("yes")
        self.okay.append("done")
        self.okay.append("yep")


    def listen(self, seconds_to_listen = 10):

        #ensure queue is empty:
        while self.stt.has_string_in_queue():
            self.stt.get_string_from_queue()

        #listen for new input:
        self.stt.listen(seconds_to_listen)
        if self.stt.has_string_in_queue():
            output = self.stt.get_string_from_queue()
        output = output.lower()
        print("heard: " + output)
        return output.lower()


    def get_room_with_change(self, debug_string=""):
        if debug_string == "":
            self.tts.synth_to_play("In which room is there a change?", block=True)
            input_string = self.listen()
        else:
            input_string = debug_string

        for room in self.rooms:
            if room in input_string:
                return room

        return ""


    def place_object_on_robot(self, object_name, debug_string=""):

        if debug_string == "":
            string_to_speak = "Please place the " + object_name + " on my holder and say okay when you are finished." 
            self.tts.synth_to_play(string_to_speak, block=True)

        failures = 0
        done = False
        while not done:

            if failures >= 2:
                return False

            if debug_string == "":
                answer = self.listen()
            else:
                answer = debug_string

            if answer == "":
                failures = failures + 1
                self.tts.synth_to_play("Please say okay, if you have placed the " + object_name + " on my holder")
            else:
                for okay in self.okay:
                    if okay in answer:
                        return True
                failures = failures +1
                self.tts.synth_to_play("Please say okay, if you have placed the " + object_name + " on my holder", block=True)


    def place_object_on_furniture(self, object_name, where_to_place, debug_string=""):
        string_to_speak = "Please take the " + object_name + " from my holder and place it on the " + where_to_place + " and say okay when you are finished."
        
        if debug_string == "":
            self.tts.synth_to_play(string_to_speak, block=True)

        done = False
        failures = 0
        while not done:

            if failures >= 2:
                return False

            if debug_string == "":
                answer = self.listen()
            else:
                answer = debug_string

            if answer == "":
                self.tts.synth_to_play("Please say okay, if you have taken the " + object_name + " from my holder")
                failures = failures + 1
            else:
                for okay in self.okay:
                    if okay in answer:
                        return True
                self.tts.synth_to_play("Please say okay, if you have taken the " + object_name + " from my holder")
                failures = failures + 1
