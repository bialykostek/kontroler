var planeVis = {
    x: 500,
    y: 500,
    angle: 30,
    img: null,
    size: 50
}
var testAnim = false;
var animation;

var playing = false;
var playingSpeed = 1;

function visualizationFrame(){
    var canvas = document.getElementById("visualizationCanvas");
    var ctx = canvas.getContext("2d");

    //background
    ctx.fillStyle = "#2f482d";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    //field
    ctx.fillStyle = '#3d6f37';
    ctx.beginPath();
    ctx.moveTo(100, 100);
    ctx.lineTo(1100, 100);
    ctx.lineTo(1100, 600);
    ctx.lineTo(100, 600);
    ctx.closePath();
    ctx.fill();

    //plane
    ctx.save();
    ctx.translate(planeVis.x, planeVis.y);
    ctx.rotate(planeVis.angle);
    ctx.translate(-planeVis.size/2, -planeVis.size/2);
    ctx.drawImage(planeVis.img, 0, 0, planeVis.size, planeVis.size);
    ctx.restore();

    //small randomization
    planeVis.x += Math.random()*10-5;
    planeVis.y += Math.random()*10-5;
    planeVis.angle += (Math.random()*2-1)*Math.PI/180*10;
}

function visualizationInit(){
    var planeImg = new Image();

    planeImg.src = 'static/plane.png';
    planeImg.onload = _ => {
        visualizationFrame();
    }
    planeVis.img = planeImg;
    
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

    $('#resetPlayingSpeed').click(_ => {
        playingSpeed = 50;
        $('#playingSpeedSlider').val(playingSpeed);
        $('#playerSpeedValue').html('x1.0');
    });

    $('#playPauseButton').click(_ => {
        if(playing){
            playing = false;
            $('#playPauseButton img').attr('src', 'static/play.png');
        }else{
            playing = true;
            $('#playPauseButton img').attr('src', 'static/pause.png');
        }
    });

    clog("Visualization ready", "info");
}

