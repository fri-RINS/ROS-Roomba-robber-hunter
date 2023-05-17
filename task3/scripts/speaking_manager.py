import rospy
from exercise2.srv import PlaySound,PlaySoundRequest
from sound_play.libsoundplay import SoundClient

class SpeakingManager:
    def __init__(self):
        self.sound_handler = SoundClient()
        self.sound_client = rospy.ServiceProxy('play_sound', PlaySound)

    def say_ring_color(self,color):
        
        soundhandle = self.sound_handler
        rospy.sleep(1) # wait for the sound client to initialize
        # say the phrase "green ring detected"
        phrase = f"{color} ring detected"
        rospy.loginfo(f"Saying {color} ring detected")
        soundhandle.say(phrase)
        rospy.sleep(2) # wait for the sound to finish playing

    def say_cylinder_color(self,color):
        soundhandle = self.sound_handler
        rospy.sleep(1) # wait for the sound client to initialize
        # say the phrase "green ring detected"
        phrase = f"{color} cylinder detected"
        rospy.loginfo(f"Saying {color} cylinder detected")
        soundhandle.say(phrase)
        rospy.sleep(2) # wait for the sound to finish playing 

    def say_goodbye(self):
        soundhandle = self.sound_handler
        rospy.sleep(1) # wait for the sound client to initialize
        # say the phrase "green ring detected"
        phrase = "I am done"
        rospy.loginfo("Saying Goodbye")
        soundhandle.say(phrase)
        rospy.sleep(5) # wait for the sound to finish playing 

    def greet(self):
        srv = PlaySoundRequest()
        srv.message = "Hello there"
        # call the service
        rospy.loginfo("Saying hello")
        response = self.sound_client(srv)
        rospy.sleep(1)

    def say_started_parking(self):
        soundhandle = self.sound_handler
        rospy.sleep(1) # wait for the sound client to initialize
        # say the phrase "green ring detected"
        phrase = "Started parking!"
        rospy.loginfo("Saying started parking!")
        soundhandle.say(phrase)
        rospy.sleep(2) # wait for the sound to finish playing 

    def say_approaching_green(self):
        soundhandle = self.sound_handler
        rospy.sleep(1) # wait for the sound client to initialize
        # say the phrase "green ring detected"
        phrase = "Approaching green!"
        rospy.loginfo("Saying approaching green!")
        soundhandle.say(phrase)
        rospy.sleep(2) # wait for the sound to finish playing 

    def say_arrest_robber(self):
        soundhandle = self.sound_handler
        rospy.sleep(1) # wait for the sound client to initialize
        # say the phrase "green ring detected"
        phrase = "You are arrested. Enter the robot car immediately!"
        rospy.loginfo("You are arrested. Enter the robot car immediately!")
        soundhandle.say(phrase)
        rospy.sleep(2) # wait for the sound to finish playing 