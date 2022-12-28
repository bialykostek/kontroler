#!/usr/bin/env node

const logsLocation = "./logs/"
const WebSocket = require('ws')
const wss = new WebSocket.Server({
    port: 1111
})
const fs = require('fs');

var saving = false;
var clients = [];
var currentFile;

function sendToWeb(object){
    clients.forEach((wsx) => {
        if(wsx.clientType == 2){
            wsx.send(JSON.stringify(object));
        }
     });
}

function sendToGroundStation(object){
    clients.forEach((wsx) => {
        if(wsx.clientType == 1){
            wsx.send(JSON.stringify(object));
        }
     });
}

wss.on('connection', (ws) => {

    ws.on('message', (data) => {
        data = JSON.parse(data);

        if(data.type == 0){
            sendToGroundStation(data);
        }

        if(data.type == 1){
            sendToWeb(data);
        }

        if(data.type == 2){
            ws.clientType = data.clientType;
        }

        if(data.type == 3){
            if(data.target == 1){
                sendToWeb({
                    type: 4,
                    from: 1
                });
            }
            if(data.target == 2){
                sendToGroundStation({
                    type: 3,
                    target: 2
                });
            }
            if(data.target == 3){
                sendToGroundStation({
                    type: 3,
                    target: 3
                });
            }
        }

        if(data.type == 4){
            sendToWeb(data);
        }

        if(data.type == 5){
            sendToWeb(data);
            if(saving){
                fs.appendFile(logsLocation + currentFile, data.text + "|" + Dat$
                    if (err) throw err;
                });
            }
        }

        if(data.type == 6){
            if(data.action == 0){
                saving = true;
                currentFile = "";
                let date = new Date();
                var h = date.getHours();
                var m = date.getMinutes();
                var s = date.getSeconds();
 var day = date.getDate();
                var month = date.getMonth() + 1;
                if(h < 10){
                    h = '0' + h;
                }
                if(m < 10){
                    m = '0' + m;
                }
                if(s < 10){
                    s = '0' + s;
                }
                if(day < 10){
                    day = '0' + day;
                }
                if(month < 10){
                    month = '0' + month;
                }
                currentFile = "log_" + day + '_' + month + "_" +  h + '_' + m +$
                console.log("Started saving to " + currentFile);
            }
            if(data.action == 1){
                saving = false;
                console.log("Stopped saving");
            }
            if(data.action == 2){
                sendToWeb({
                    type: 7,
                    message: 0,
                    value: saving.toString()
                });
            }
            if(data.action == 3){
                fs.readdir(logsLocation, (err, files) => {
                    sendToWeb({
                        type: 7,
                        message: 1,
                        value: JSON.stringify(files)
                    });
                });
            }
        }

    });

    ws.on('close', _ => {
        var index = clients.indexOf(ws);
        if(index != -1){
            clients.splice(index, 1);
        }
    });

    ws.clientType = 0;
    clients.push(ws);

});

