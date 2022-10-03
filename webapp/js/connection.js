var websocket;
var connected = false;

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

function sendObject(object){
    if(websocket.readyState == 1){
        websocket.send(JSON.stringify(object));
    }
}

function disconnectFromServer(){
    websocket.close();
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
                    default:
                      clog("Unknow message type " + messageType + ": " + response, "negative");
                  }  
            }
        }
        if(data.type == 4){
            echoResponse(data.from);
        }
        if(data.type == 5){
            updateLiveData(data.text);
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