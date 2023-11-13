import speech_recognition as sr
from gtts import gTTS
import os 

# with sr.AudioFile("output.wav") as source:
#      audio = r.record(source)

# text = r.recognize_google(audio,language='zh-tw')
# print(text)

def speak(text):
    # language = 'zh'
    myobj = gTTS(text=text, lang='en', slow=False)
    filename = "output.mp3"
    myobj.save(filename)
    os.system("mpg123 " + filename)

def get_audio():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        audio = r.listen(source)
        said = ""
        try:
            said = r.recognize_google(audio)
            print(said)
        except Exception as e:
            print("Exception: " + str(e))
    return said

# speak('hello')
text = get_audio()

if "hello" in text:
    speak("hello, how are you")

