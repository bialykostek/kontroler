from ssl import OP_NO_COMPRESSION
import websocket
import threading
import json 
import time
import serial

def on_message(ws, message):
    message = json.loads(message)
    if message["type"] == 0:
        serialPort.write(str.encode(str(message["text"])))

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

    serialPort = serial.Serial(port='COM14', baudrate=38400, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
    time.sleep(3)

    while True:
        try:
            serialString = serialPort.readline().decode("utf-8").replace("\n", "").replace("\r", "")
            if serialString != "":
                ws.send(json.dumps({
                    "type": 1,
                    "text": serialString
                }))
                
        except Exception:
            pass