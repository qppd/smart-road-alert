from gtts import gTTS
import os

text = "STOP KAMOTE" 
tts = gTTS(text=text, lang='tl')

tts.save("output.mp3")
os.system("mpg123 output.mp3")        