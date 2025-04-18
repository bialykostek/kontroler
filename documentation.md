Orders [@]:
0 - arm
1 - disarm
2 - send one info frame
3 - reset / 1st step
4 - reset / 2sd step

Information request [?]:
0 - is alive?
1 - is armed?
2 - show waypoints
3 - show current left column
4 - show current right column
5 - show current scale NS
6 - show current scale EW
7 - show current angle

Change value [!]:
0 - sending data (0 - false, 1 true)
1 - change waypoints sending index (int)
2 - change waypoint value (int)
3 - set current position as left column
4 - set current position as right column
5 - set scale NS
6 - set scale EW
7 - set angle
8 - set current waypoint
9 - calibrate compas
10 - change sending data mode (0 - normal, 1 - AI style)
11 - GPS frequency

Responses [#]:
0 - heartbeat (1 - ok)
1 - start mode (0 - normal, 1 - emergency)
2 - arm info (0 - disarmed, 1 - armed)
3 - ICM initialization result (0 - error, 1 - success)
4 - DMP initialization result (0 - error, 1 - success)
5 - GPS initialization result (0 - error, 1 - success)
6 - LPS initialization result (0 - error, 1 - success)
7 - emergency mode (1 - emergency)
8 - change value confirmation (1 - ok)
9 - current left column
10 - current right column
11 - current scale NS
12 - current scale EW
13 - current angle
14 - system ready

Setting chanel value:
0-50 - manual
15-30 - stop sending messages
75-105 - auto mode

Websocket message type:
0 - Web App -> Ground Station
1 - Ground Station -> Web App
2 - Connection device type
3 - Echo request
4 - Echo response
5 - Telemetry Data
6 - Web App -> Server (settings)
7 - Server -> WebApp (settings)

Server settings actions (type 6):
0 - turn on saving
1 - turn of saving
2 - ask if saving
3 - ask for logs

Server settings responses (type 7):
0 - server on/off
1 - log list

Controler EEPROM adresses:
0 - arm info
1 - 113 - waypoints
114-117 - left column long
118-121 - left column lat
122-125 - right column long
126-129 - right column lat
130-133 - scale NS
134-137 - scale EW
138-141 - angle 

Neural Network inputs:
0 - roll
1 - pitch
2 - roll previous
3 - pitch previous
4 - ground speed
5 - altitute
6 - vPitot
7 - angles[0]
8 - angles[1]
9 - angles[2]
10 - angles[3]
11 - angles[4]
12 - angles[5]
13 - angles[6]
14 - angles[7]
15 - distance to current
16 - input1 previous
17 - input2 previous
18 - input3 previous
19 - input4 previous
20 - input5 previous
21 - output1
22 - output2
23 - output3
24 - output4
25 - output5