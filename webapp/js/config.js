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