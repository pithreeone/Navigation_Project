# Import the Gtts module for text  
# to speech conversion 
from gtts import gTTS 
  
# import Os module to start the audio file
import os 
  
mytext = 'hey google hey google hey google hey google hey google '
# mytext = 'su3'

# Language we want to use 
language = 'en'
# language = 'zh'
myobj = gTTS(text=mytext, lang=language, slow=False) 
myobj.save("output.mp3") 

from pydub import AudioSegment
sound = AudioSegment.from_mp3("output.mp3")
sound.export("output.wav", format="wav")
# Play the converted file 
os.system("mpg123 output.mp3")