function echoStart(){
    $('#echoTestTable .default').removeClass('active');
    setTimeout(_ => { 
        $('#echoWebApp').addClass('active');
    }, 50);
    sendObject({
        type: 3,
        target: 1
    });
    sendObject({
        type: 3,
        target: 2
    });
    sendObject({
        type: 3,
        target: 3
    });
    planeQuery(0);
}

function echoPlane(response){
    if(response == "1"){
        $('#echoControler').addClass('active');
        downloadConfig();
    }
}

function echoResponse(from){
    if(from == 1){
        $('#echoServer').addClass('active');
    }
    if(from == 2){
        $('#echoRelayApp').addClass('active');
    }
    if(from == 3){
        $('#echoGroundStation').addClass('active');
    }
}

function settingsInit(){
    $('#echoStart').click(echoStart);

    clog("Settings initialized", "info");
}