var websocket;
var connected = false;
var changeValueCallback = null;
var sendingWaypointsStatus = -1;
var changingValueInterval;
var sendingWaypointsIndex = 0;

function sendToPlane(text){
    if(websocket.readyState == 1){
        websocket.send(JSON.stringify({
            type: 0,
            text: text
        }));
        addConsoleOutput("now", "<", text, "order");
    }else{
        addConsoleOutput("now", "<", text, "notdelivered");
    }
}

function planeExecute(order){
    sendToPlane("@"+order+";");
}

function planeQuery(order){
    sendToPlane("?"+order+";");
}

function planeChangeValue(target, value, callback=null){
    sendToPlane("!"+target+";"+value+";");
    changeValueCallback = callback;
}

function sendObject(object){
    if(websocket.readyState == 1){
        websocket.send(JSON.stringify(object));
    }
}

function disconnectFromServer(){
    websocket.close();
}

function confirmChangeValue(){
    if(changeValueCallback != null){
        window[changeValueCallback]();
    }
}

function sendWaypointsConfirm(){
    if(sendingWaypointsStatus == -1){
        sendingWaypointsStatus = 0;
        sendingWaypointsIndex = 0;
    }else if(sendingWaypointsStatus == 0){
        sendingWaypointsStatus++;
        clog("Updating waypoints index ok");
    }else if(sendingWaypointsStatus == 1){
        sendingWaypointsStatus--;
        sendingWaypointsIndex++;
        if(sendingWaypointsIndex >= waypointsTransformed.length * 2){
            sendingWaypointsStatus = 2;
        }
        clog("Updating waypoint " + sendingWaypointsIndex  + " value ok");
    }
    clearInterval(changingValueInterval);
    sendWaypoints();
}

function sendWaypoints(){
    if(sendingWaypointsStatus == -1){
        var tmpFunc = _ => {
            planeChangeValue(0, 0, callback = "sendWaypointsConfirm");
        };
        changingValueInterval = setInterval(tmpFunc, 200);
        tmpFunc();
    }
    if(sendingWaypointsStatus == 0){
        var tmpFunc = _ => {
            planeChangeValue(1, sendingWaypointsIndex, callback = "sendWaypointsConfirm");
        };
        changingValueInterval = setInterval(tmpFunc, 200);
        tmpFunc();
    }
    if(sendingWaypointsStatus == 1){
        var tmpFunc = _ => {
            planeChangeValue(2, waypointsTransformed[parseInt(sendingWaypointsIndex/2)][sendingWaypointsIndex%2], callback = "sendWaypointsConfirm");
        };
        changingValueInterval = setInterval(tmpFunc, 200);
        tmpFunc();
    }
    if(sendingWaypointsStatus == 2){
        clog("Updating waypoints data complete!", "positive");
        sendingWaypointsStatus = -1;
    }
}

function connectToServer(){
    websocket = new WebSocket('ws://jkostecki.ddns.net:1111');
    websocket.onopen = _ => {
        websocket.send(JSON.stringify({
            type: 2,
            clientType: 2   
        }));
        connected = true;
        clog("Connected to server", "positive");
        echoStart();
        $("#connectButton").html("Disconnect");
        sendObject({
            type: 6,
            action: 2
        });
        sendObject({
            type: 6,
            action: 3
        });
    };
    websocket.onmessage = (data) => {
        data = JSON.parse(data.data);
        if(data.type == 1){
            if(data.text[0] == '#'){
                var messageType = parseInt(data.text.split('|')[0].split('#')[1]);
                var response = data.text.split('|')[1];

                switch (messageType) {
                    case 0:
                        echoPlane(response);
                        break;
                    case 1:
                        startInfo(response);
                        break;
                    case 2:
                        responseArmed(response);
                        break;
                    case 3:
                        ICMinit(response);
                        break;
                    case 4:
                        DMPinit(response);
                        break;
                    case 5:
                        GPSinit(response);
                        break;
                    case 6:
                        LPSinit(response);
                        break;
                    case 7:
                        emergency(response);
                        break;
                    case 8:
                        confirmChangeValue(response);
                        break;
                    default:
                      clog("Unknow message type " + messageType + ": " + response, "negative");
                  }  
            }else{
                planeLog(data.text);
            }
        }

        if(data.type == 4){
            echoResponse(data.from);
        }

        if(data.type == 5){
            updateLiveData(data.text);
        }

        if(data.type == 7){
            if(data.message == 0){
                toogleSaving(data.value);
            }
            if(data.message == 1){
                loadLogs(JSON.parse(data.value));
            }
        }
    }
    websocket.onclose = _ => {
        connected = false;
        echoStart();
        clog("Disconnected from server", "negative");
        $("#connectButton").html("Connect");
    }
}
function connectionInit(){

    $("#connectButton").click(_ => {
        if(!connected){
            connectToServer();
        }else{
            disconnectFromServer();
        }
    });

    connectToServer();
    clog("Connected module initialized", "info");
}