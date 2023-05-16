#!/usr/bin/python3

import roslib
import time
import rospy

import speech_recognition as sr

# for talking user interface
import tkinter as tk
import os

from rospy.timer import sleep
import re

from sound_play.libsoundplay import SoundClient

class ConversationManager:
    def __init__(self):
        # The documentation is here: https://github.com/Uberi/speech_recognition


        # The main interface to the speech recognition engines
        self.sr = sr.Recognizer()
        
        # These are the methods that are available to us for recognition.
        # Please note that most of them use an internet connection and currently they are using
        # a default API user/pass, so there are restrictions on the number of requests we can make.
        # recognize_bing(): Microsoft Bing Speech
        # recognize_google(): Google Web Speech API
        # recognize_google_cloud(): Google Cloud Speech - requires installation of the google-cloud-speech package
        # recognize_houndify(): Houndify by SoundHound
        # recognize_ibm(): IBM Speech to Text
        # recognize_sphinx(): CMU Sphinx - requires installing PocketSphinx
        # recognize_wit(): Wit.ai
        
        # An interface to the default microphone
        self.mic = sr.Microphone()


        #self.create_ui()
        

        # true if the robot asked the user for input, false otherwise
        self.ready_for_input = False

        self.STOP_NOW = False
        print("MICROPHONE LIST", sr.Microphone.list_microphone_names())
        # You can get the list of available devices: sr.Microphone.list_microphone_names()
        # You can set the fault microphone like this: self. mic = sr.Microphone(device_index=3)
        # where the device_index is the position in the list from the first command.

    # callbacks for buttons
    def speak_btn_callback(self):
        if self.ready_for_input:
            self.recognize_speech()

    def destroy_ui(self):
        self.root.destroy()

    def quit_btn_callback(self):
        # destroy UI
        self.root.destroy()
        self.STOP_NOW = True
        self._recognized_text = "nothing_valid"
        self.got_data_var.set(1)

    def manual_entry_cb(self):
        self._recognized_text = self.answer_input.get()
        self.got_data_var.set(1)


    def create_ui(self):
        """
        Create UI with buttons for managing conversation.
        """
        root = tk.Tk()

        self.root = root
        root.title("Manage speech")
        root.geometry("500x300")

        label = tk.Label(root, fg="dark blue")
        label.pack()
        self.label = label
        label.config(text="Press for speaking.")
        # set label color to red -> no speaking
        #self.label.config(bg="#FF0000")

        #self.label_signal = tk.Label(root, bg="red")
        #self.label_signal.pack()

        button = tk.Button(root, text='Speak', width=25, command=self.speak_btn_callback)
        button.pack()
        self.speak_button = button

        self.answer_input = tk.Entry(root)
        self.answer_input.pack()
        button_manual = tk.Button(root, text='Enter manually', width=25, command=self.manual_entry_cb)
        button_manual.pack()

        button_quit = tk.Button(root, text='Quit', width=25, command=self.quit_btn_callback)
        button_quit.pack()

    def recognize_speech(self):
        # for backup
        audio = None
        with self.mic as source:
            print('Adjusting mic for ambient noise...')
            self.sr.adjust_for_ambient_noise(source)
            print('SPEAK NOW!')
            #self.root.config(bg="green")
            #audio = self.sr.listen(source)
            audio = self.sr.listen(source, timeout=5, phrase_time_limit=30)
        
        print('STOPPED LISTENING!')
        #self.root.config(bg="red")
        print('I am now processing the sounds you made.')
        recognized_text = ''
        try:
            recognized_text = self.sr.recognize_google(audio)
            # if we wanted to get all possible transcriptions (not just the most probbable)
            #recognized_text = self.sr.recognize_google(audio, show_all=True)
            #print(recognized_text)
        except sr.RequestError as e:
            print('API is probably unavailable', e)
        except sr.UnknownValueError:
            print('Did not manage to recognize anything.')
            
        print("You said: " + recognized_text)

        self._recognized_text = recognized_text

        # message got data
        self.got_data_var.set(1)

        # make button visible again
        #os.system("kate&")
        return recognized_text
    
    def check_for_colors(self,string):
        color_regex = re.compile(r"\b(red|blue|green|yellow|black)\b", re.IGNORECASE)
        matches = re.findall(color_regex, string)
        
        if matches:
            print("Colors found in the string:", matches)
        else:
            print("No colors found in the string.")

        return matches

    """
    Plays appropriate sound in string prameter.
    Do not forget to use: rosrun  sound_play  soundplay_node.py
    """
    def say_something(self, message_cotent):
        # display message
        self.label.config(text=message_cotent)

        voice = 'voice_kal_diphone'
        volume = 1.0
        s = message_cotent
        rospy.loginfo("Robot just said: %s" % str(message_cotent))
        if self.soundhandle == None:
            return
        self.soundhandle.say(s, voice, volume)

    def get_cylinder_colors(self):
        """
        Returns two colors of the cylinders where robber  might be hiding, or none if person doesent know.
        """
        self.got_data_var = tk.IntVar()
        self.say_something("Do you know where the robber is hiding?")
        self.speak_button.wait_variable(self.got_data_var)
        answer = self._recognized_text
        print("Response we got: "  + answer)

        matches = self.check_for_colors(answer)

        if len(matches) == 2:
            print("Person knew something, got two potencial cylinders")
            return matches
        
        print("person knew nothing.")
        return None

    def ask_question(self, question_func):
        """
        Asks a question multiple times until it gets the answer
        or max number of attempts is surpassed.
        """
        self.ready_for_input = True

        max_retries = 5
        for i in range(max_retries):

            if self.STOP_NOW:
                #return "stop_now"
                exit(0)

            res = question_func()
            if res != None:
                return res

            # otherwise ask the question again
            self.say_something("i did not understand")

        self.ready_for_input = False

    def talkToPerson(self, sound_handle):
        """
        Talks to person and returns the data from conversation
        """
        self.create_ui()

        self.soundhandle = sound_handle
        # receive answers and process them

        # prepare vars on None
        cylinders_to_approach = None
        
        cylinders_to_approach = self.ask_question(self.get_cylinder_colors)

        if cylinders_to_approach is not None:
            print("We got the information needed")
        else:
            print("Still need to speek to other people")

        self.destroy_ui()

        return cylinders_to_approach


if __name__ == '__main__':
    rospy.init_node('speech_transcriber', anonymous=True)
    
    soundhandle = SoundClient()
    # for testing
    st = ConversationManager()
    st.talkToPerson(soundhandle)
    #rospy.sleep(3)
    #st.talkToPerson(None)
    """
    while not rospy.is_shutdown():
        text = st.recognize_speech()
        print('I recognized this sentence:', text)
        time.sleep(4)
    """

