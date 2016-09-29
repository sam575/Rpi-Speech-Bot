import audioop
import socket
import time
import pyaudio
import wave
import struct
import math
import array
import os
import speech_recognition as sr
from xbee import XBee
import serial

#Giving access to USB port for Xbee transfer
sudoPassword = '******'
command = 'chmod 666 /dev/ttyUSB4'
p = os.system('echo %s|sudo -S %s' % (sudoPassword,command))
PORT = '/dev/ttyUSB4'
BAUD = 9600

ser = serial.Serial(PORT, BAUD)		

xbee = XBee(ser)
CHUNK = 1024 
FORMAT = pyaudio.paInt16 #paInt8
CHANNELS = 2 
RATE = 44100 #sample rate
RECORD_SECONDS = 2
WAVE_OUTPUT_FILENAME = "output.wav"

#Commands and their shortform
a= ["forward" , "backward" , "turn right" , "turn left" , "faster" , "normal" , "turn back" ,"stop", "look right" , "look left" , "little left" , "little right" , "hello" , "click" , "find red" , "find Blue"]
b=["f" , "b" , "tr" , "tl" , "fas" , "nor" , "tb" ,"stop", "lor" , "lol" , "lil" , "lir" , "hello" , "click" , "find red" , "find blue"]
no=1 
i1=0
sen = []

def get_samples(file):
     
	waveFile = wave.open(file, 'r')
    	samples = []
	samples1 = []
 
    # Gets total number of frames
    	length = waveFile.getnframes()
     
    # Read them into the frames array
    	for i in range(0,length):
        	waveData = waveFile.readframes(1)
        	data = struct.unpack("%ih"%2, waveData)
         
        # After unpacking, each data array here is actually an array of ints
        # The length of the array depends on the number of channels you have
         
        # Drop to mono channel
        	samples.append(int(data[0]))
     	s=0   	
   	for i in samples:
		s=s+(i**2)
   	return s

#******************************************************#
#Records for every 3-4 seconds. Writes the recorded data into a file.
#Recognises the speech in the recorded file.
#If text recognised as one of the command sends the command to Rpi and starts recording again 
#and overwrites the recorded file
#If unable to recognise or no input, starts recording again
while no==1:
	ni=1
	while ni==1:
		time.sleep(1)
		p = pyaudio.PyAudio()

		stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK) #buffer

		print("* recording")

		frames = []

		for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    			data = stream.read(CHUNK)
    			frames.append(data) # 2 bytes(16 bits) per channel

		print("* done recording")

		stream.stop_stream()
		stream.close()
		p.terminate()

		wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
		wf.setnchannels(CHANNELS)
		wf.setsampwidth(p.get_sample_size(FORMAT))
		wf.setframerate(RATE)
		wf.writeframes(b''.join(frames))
		wf.close()
		N = get_samples(WAVE_OUTPUT_FILENAME)
		ni=0
# obtain path to "output.wav" in the same folder as this script
	from os import path
	WAV_FILE = path.join(path.dirname(path.realpath(__file__)), "output.wav")
# use "english.wav" as the audio source
	r = sr.Recognizer()
 
	with sr.WavFile(WAV_FILE) as source:
		audio = r.record(source) # read the entire WAV file
	try:
		c=r.recognize_google(audio)
		print("You said: " + c)    # recognize speech using Google Speech Recognition
		for i in range(-1,17):
			if c == a[i] :
				om=b[i]
				
				print("sending " + c)
				# Send the string 'Hello World' to the module with MY set to 1
				xbee.tx(dest_addr='\x00\x02', data='%s\n' % om)
				#print("2")
				# Wait for and get the response
				#print(xbee.wait_read_frame())
				#print("3")
				#ser.close()
				print 'sent'	
	except sr.UnknownValueError :
		print("Google Speech Recognition could not understand audio")
	except sr.RequestError as e :
		print("Could not request results from google".format(e))
	except LookupError:                            # speech is unintelligible
    		print("Could not understand audio")
	 
ser.close() 	