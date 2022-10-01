var websocket;

function sendToPlane(text){
    if(websocket.readyState == 1){
        websocket.send(JSON.stringify({
            type: 0,
            text: text
        }));
    }
}

function sendObject(object){
    if(websocket.readyState == 1){
        websocket.send(JSON.stringify(object));
    }
}

function connectionInit(){
    websocket = new WebSocket('ws://jkostecki.ddns.net:1111');
    websocket.onopen = _ => {
        websocket.send(JSON.stringify({
            type: 2,
            clientType: 2
        }));
    };
    websocket.onmessage = (data) => {
        data = JSON.parse(data.data);
        if(data.type == 1){
            addConsoleOutput("now", ">", data.text);
        }
        if(data.type == 4){
            echoResponse(data.from);
        }
    }
}