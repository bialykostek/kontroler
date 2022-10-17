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

function updateScaleNS(){
    var tmpFunc = _ => {
        planeChangeValue(5, scaleNS, callback = "updateScaleEW");
    };
    changingValueInterval = setInterval(tmpFunc, 200);
    tmpFunc();
}

function updateScaleEW(){
    clearInterval(changingValueInterval);
    var tmpFunc = _ => {
        planeChangeValue(6, scaleEW, callback = "updateAngle");
    };
    changingValueInterval = setInterval(tmpFunc, 200);
    tmpFunc();
}

function updateAngle(){
    clearInterval(changingValueInterval);
    var tmpFunc = _ => {
        planeChangeValue(7, angle, callback = "updateLocalParamsDone");
    };
    changingValueInterval = setInterval(tmpFunc, 200);
    tmpFunc();
}

function updateLocalParamsDone(){
    planeLog("Update of local coordinate params complete", "positive");
    clearInterval(changingValueInterval);
    changeValueCallback = null;
}

function getLeftColumn(data){
    planeQuery(3);
}

function getRightColumn(data){
    planeQuery(4);
}

function gotLeftColumn(data){
    clog("Plane left column: " + data);
    data = data.split(";");
    leftColumn[0] = parseInt(data[0])/10000000;
    leftColumn[1] = parseInt(data[1])/10000000;
}

function gotRightColumn(data){
    clog("Plane right column: " + data);
    data = data.split(";");
    rightColumn[0] = parseInt(data[0])/10000000;
    rightColumn[1] = parseInt(data[1])/10000000;
}

function calibrateCoordinateSystems(){
    calibration();
    updateScaleNS();
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
        changeValueCallback = null;
        clog("Updating waypoints data complete!", "positive");
        sendingWaypointsStatus = -1;
    }
}

function stopSending(){
    clearInterval(changingValueInterval);
    planeLog("Sending data from plane stopped");
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

    $('#sendingOnBtn').click(_ => {
        planeChangeValue(0, 1);
    });

    $('#stopSendingBtn').click(_ => {
        clog("Stop sending value change request");
        clearInterval(changingValueInterval);
    });

    $('#getOneFrameBtn').click(_ => {
        planeExecute(2);
    });

    $('#getLeftColumnBtn').click(_ => {
        getLeftColumn();
    });

    $('#getRightColumnBtn').click(_ => {
        getRightColumn();
    });

    $('#calibrateYawBtn').click(_ => {
        planeChangeValue(9, 0);
    });

    $('#sendingOffBtn').click(_ => {
        var tmpFunc = _ => {
            planeChangeValue(0, 0, callback = "stopSending");
        };
        changingValueInterval = setInterval(tmpFunc, 200);
        tmpFunc();
    });    

    sendObject({
        type: 6,
        action: 2
    });
    
    clog("Config initialization done", "info");
}