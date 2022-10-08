Orders [@]:
0 - arm
1 - disarm

Information request [?]:
0 - is alive?
1 - is armed?
2 - show waypoints

Change value [!]:
0 - sending data (0 - false, 1 true)
1 - change waypoints sending index
2 - change waypoint value

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