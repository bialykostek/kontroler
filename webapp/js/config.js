var armed = false;

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

function initConfig(){
    $('#armButton').click(_ => {
        toogleArm();
    });

    clog("Config initialization done", "info");
}