function echoStart(){
    $('#echoTestTable .default').removeClass('active');
    $('#echoWebApp').addClass('active');
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
}