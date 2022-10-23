var liveData = true;
var currentLogFile = "";
var currentLogData = [];
var playerFrameNumber = 0;

var playing = false;
var playingSpeed = 1;

var nextFrameTimeout;

function nextFrame(){
    playerFrameNumber++;
    if(playerFrameNumber >= currentLogData.length - 1){
        playerFrameNumber = currentLogData.length - 1;
        playing = false;
        $('#playPauseButton img').attr('src', 'static/play.png');
    }
    $('#playingTimeline').val(playerFrameNumber/currentLogData.length*100);
    updateInfoSpan();
    updateLiveData(currentLogData[playerFrameNumber][0]);
    if(playing){
        nextFrameTimeout = setTimeout(nextFrame, (currentLogData[playerFrameNumber+1][1] - currentLogData[playerFrameNumber][1])/playingSpeed);   
    }
}

function updateInfoSpan(){
    var date = new Date(currentLogData[playerFrameNumber][1]);
    var h = date.getHours();
    var m = date.getMinutes();
    var s = date.getSeconds();
    if(h < 10){
        h = '0' + h;
    }
    if(m < 10){
        m = '0' + m;
    }
    if(s < 10){
        s = '0' + s;
    }
    $('#playerInfoSpan').html("Frame " + playerFrameNumber + " of " + currentLogData.length + ", " + parseInt(playerFrameNumber/currentLogData.length*100) + "%, " + h + ":" + m + ":" + s);
}

function playerInit(){

    $('#prevFrameBtn').click(_=>{
        playerFrameNumber--;
        if(playerFrameNumber < 0){
            playerFrameNumber = 0;
        }
        $('#playingTimeline').val(playerFrameNumber/currentLogData.length*100);
        updateInfoSpan();
        updateLiveData(currentLogData[playerFrameNumber][0]);
    });

    $('#nextFrameBtn').click(_=>{
        playerFrameNumber++;
        if(playerFrameNumber >= currentLogData.length){
            playerFrameNumber = currentLogData.length -1;
        }
        $('#playingTimeline').val(playerFrameNumber/currentLogData.length*100);
        updateInfoSpan();
        updateLiveData(currentLogData[playerFrameNumber][0]);
    });

    $('#tooglePlayer').checkbox({
        onChecked: function(){
            liveData = false;
            currentLogData = []
            $.get("http://jkostecki.ddns.net:8080/log_21_10_00_05_16.txt?xdxd", (data) => {
                currentLogData = data.split('\n');
                for(var i=0; i < currentLogData.length; i++){
                    currentLogData[i] = [currentLogData[i].split('|')[0], parseInt(currentLogData[i].split('|')[1])];
                }
                clog("Log file downloaded", "info");
            });

        },
        onUnchecked: function(){
            liveData = true;
        }
    });

    $('#playingSpeedSlider').on('input', _ => {
        playingSpeed = $('#playingSpeedSlider').val();
        if(playingSpeed <= 50){
            playingSpeed = playingSpeed * 0.018367 + 0.081633;
        }else{
            playingSpeed = playingSpeed * playingSpeed * 0.0036 - playingSpeed * 0.36 + 10
        }
        var text = 'x' + Math.round(playingSpeed*10)/10;
        if(text == 'x10'){
            text = 'x9.9'
        }
        if(text.length == 2){
            text += '.0';
        }
        $('#playerSpeedValue').html(text);
    });

    $('#playingTimeline').on('input', _ => {
        playerFrameNumber = parseInt($('#playingTimeline').val()/100*currentLogData.length);
        playing = false;
        $('#playPauseButton img').attr('src', 'static/play.png');
        clearTimeout(nextFrameTimeout);
        updateLiveData(currentLogData[playerFrameNumber][0]);
    });

    $('#resetPlayingSpeed').click(_ => {
        playingSpeed = 1;
        $('#playingSpeedSlider').val(50);
        $('#playerSpeedValue').html('x1.0');
    });

    $('#playPauseButton').click(_ => {
        if(playing){
            playing = false;
            $('#playPauseButton img').attr('src', 'static/play.png');
            clearTimeout(nextFrameTimeout);
        }else{
            if(playerFrameNumber < currentLogData.length-1){
                playing = true;
                $('#playPauseButton img').attr('src', 'static/pause.png');     
                nextFrame();
            }
        }
    });

    clog("Player initialized", "info");
}
