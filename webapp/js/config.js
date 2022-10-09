var armed = false;
var saving = false;

function toogleSaving(response){
    if(response == "true"){
        saving = true;
        $('#savingButton').html("Stop saving");
    }else{
        saving = false;
        $('#savingButton').html("Start saving");
    }
}

function startInfo(response){
    if(response == "0"){
        planeLog("System started normally", "positive");
        downloadConfig();
    }
    if(response == "1"){
        planeLog("Emergency mode start", "negative");
    }
}

function responseArmed(response){
    if(response == "1"){
        armed = true;
        $("#armButton").html("Disarm");
    }
    if(response == "0"){
        armed = false;
        $("#armButton").html("Arm");
    }
}

function ICMinit(response){
    if(response == "1"){
        planeLog("ICM successfully initialized", "positive");
    }else{
        planeLog("ICM initialization error, trying again...", "negative");
    }
}

function LPSinit(response){
    if(response == "1"){
        planeLog("LPS successfully initialized", "positive");
    }else{
        planeLog("LPS initialization error, trying again...", "negative");
    }
}

function GPSinit(response){
    if(response == "1"){
        planeLog("GPS successfully initialized", "positive");
    }else{
        planeLog("GPS initialization error, trying again...", "negative");
    }
}

function DMPinit(response){
    if(response == "1"){
        planeLog("DMP successfully initialized", "positive");
    }else{
        planeLog("DMP initialization error, restart device", "negative");
    }
}

function emergency(response){
    if(response == "1"){
        planeLog("Entered emergency mode", "negative");
    }
}

function downloadConfig(){
    planeQuery(1);
}

function toogleArm(){
    if(armed){
        planeExecute(1);
    }else{
        planeExecute(0);
    }
    planeQuery(1);
}

function updateLeftColumnDone(){
    clog("Right column longitude updated");
}

function updateLeftColumnLat(){
    clog("Left column longitude updated");
    var tmpFunc = _ => {
        planeChangeValue(4, sendingWaypointsIndex, callback = "updateLeftColumnDone");
    };
    changingValueInterval = setInterval(tmpFunc, 200);
    tmpFunc();
}

function updateLeftColumnLon(){
    var tmpFunc = _ => {
        planeChangeValue(3, sendingWaypointsIndex, callback = "updateLeftColumnLat");
    };
    changingValueInterval = setInterval(tmpFunc, 200);
    tmpFunc();
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

function initConfig(){
    $('#armButton').click(_ => {
        toogleArm();
    });

    $('#savingButton').click(_ => {
        if(!saving){
            sendObject({
                type: 6,
                action: 0
            });
        }else{
            sendObject({
                type: 6,
                action: 1
            });
        }
        sendObject({
            type: 6,
            action: 2
        });
        sendObject({
            type: 6,
            action: 3
        });
    });

    $('#updateWaypointsBtn').click(_ => {
        sendWaypoints();
    });

    $('#updateLeftColumnBtn').click(_ => {
        planeChangeValue(3, 0);
    });

    $('#updateRightColumnBtn').click(_ => {
        planeChangeValue(4, 0);
    });

    $('#calibrateCoordinateBtn').click(_ => {
        calibrateCoordinateSystems();
    });

    sendObject({
        type: 6,
        action: 2
    });
    
    clog("Config initialization done", "info");
}