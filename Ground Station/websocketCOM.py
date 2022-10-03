from asyncio import constants
from ssl import OP_NO_COMPRESSION
import websocket
import threading
import json 
import time
import serial

PORT = "COM14"

def on_message(ws, message):
    message = json.loads(message)
    if message["type"] == 0:
        serialPort.write(str.encode(str(message["text"])))
    if message["type"] == 3 and message["target"] == 2:
        ws.send(json.dumps({
            "type": 4,
            "from": 2
        }))
    if message["type"] == 3 and message["target"] == 3:
        serialPort.write(b'#')

def on_open(ws):
    ws.send(json.dumps({
        "type": 2,
        "clientType": 1
    }))
    print("Connected to websocket")

if __name__ == "__main__":
    ws = websocket.WebSocketApp("ws://jkostecki.ddns.net:1111", on_message = on_message, on_open = on_open)
    wst = threading.Thread(target=ws.run_forever)
    wst.daemon = True
    wst.start()

    serialPort = serial.Serial(port=PORT, baudrate=38400, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
    time.sleep(3)

    while True:
        try:
            serialString = serialPort.readline().decode("utf-8").replace("\n", "").replace("\r", "")
            if serialString != "":  
                if serialString[len(serialString) - 1] == '%':
                    ws.send(json.dumps({
                        "type": 4,
                        "from": 3
                    }))
                elif serialString[0] == "$":
                    ws.send(json.dumps({
                        "type": 5,
                        "text": serialString.replace("$", "")
                    }))
                else:
                    ws.send(json.dumps({
                        "type": 1,
                        "text": serialString
                    }))
                
        except Exception:
            pass