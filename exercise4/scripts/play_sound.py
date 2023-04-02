import sys

# Import after printing usage for speed.
import roslib; roslib.load_manifest('sound_play')
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from exercise2.srv import PlaySound, PlaySoundResponse 


def handle_play_sound(req):
  rospy.loginfo("Saying : %s", req.message)
  
  
  soundhandle = SoundClient()
  rospy.sleep(1)
  
  soundhandle.say(req.message)
  rospy.sleep(1)
  return PlaySoundResponse()
	
def play_sound():

  rospy.init_node('play_sound_node')
  rospy.Service('play_sound', PlaySound, handle_play_sound)
  rospy.loginfo("Ready to play sound")
  rospy.spin()

if __name__ == '__main__':
    play_sound()
