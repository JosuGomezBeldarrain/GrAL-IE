import paho.mqtt.client as mqtt

import time
from pygame import mixer


    
def on_connect(client, userdata,flags,rc):
    print("Connected with result code"+str(rc))
    client.subscribe("arduino/HR")
    client.subscribe("arduino/alarma/acc")
    client.subscribe("arduino/alarma/ecg")
    client.subscribe("arduino/ECG")
    client.subscribe("arduino/ardatzak")
    global sentsibilitatea
    sentsibilitatea = input("Sentsibilitate baxua ala altua nahi duzu? Baxua nahi baduzu sakatu 1, eta altua nahi baduzu sakatu 2.")
    
def on_message(client,userdata,msg):
    global sentsibilitatea
    if msg.topic != "arduino/ECG":
        print(msg.topic+" "+str(msg.payload))
    if msg.topic == "arduino/HR":
        file1 = open("Bihotz-taupadak.txt","a")
        file1.write(str(msg.payload))
        file1.write("\n")
        file1.close()
    elif msg.topic == "arduino/ECG":
        file2 = open("ECG.txt","a")
        file2.write(str(msg.payload))
        file2.write("\n")
        file2.close()
    elif msg.topic == "arduino/ardatzak":
        file3=open("ardatzak2.txt","a")
        file3.write(str(msg.payload))
        file3.write("\n")
        file3.close()
    elif msg.topic == "arduino/alarma/ecg":
        print("Epilepsia krisi bat izaten ari da!")
        mixer.init()
        mixer.music.load("alarma.mp3")
        mixer.music.play()
    elif msg.topic == "arduino/alarma/acc" and sentsibilitatea == "2":
        print("Epilepsia krisi bat izaten ari da!")
        mixer.init()
        mixer.music.load("alarma.mp3")
        mixer.music.play()
    
    


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

    
broker = 'test.mosquitto.org'
port = 1883

client.connect(broker,port,60)

client.loop_start()
while True:
    time.sleep(0.02)
client.loop_stop()

