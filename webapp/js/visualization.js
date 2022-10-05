var leftColumn = [19.8747786801465, 50.05427240641037];
var rightColumn = [19.87556705467462, 50.05478046358079];

var yRatio = 1;
var angle = 0;
var distanceRatio = 1;
var visualizationScale = 10;

var visLeftColumn = [550, 700];
var visRightColumn = [1350, 700];

var versorX;
var versorY;

var scaleH = 11;
var scaleV = 9;

var canvas;
var ctx; 

var testWp = 0;

setInterval(_ => {
    testWp++;
    visualizationFrame();
}, 1000);

var waypoints = [
    [0, 0],
    [7, 6.5],
    [15,11],
    [23,15],
    [31, 17],
    [39, 14],
    [45, 8],
    [47, 0],
    [45, -8],
    [39, -14],
    [31, -17],
    [23, -15],
    [15, -11],
    [7, -6.5],
    [0, 0],
    [-7, 6.5],
    [-15,11],
    [-23,15],
    [-31, 17],
    [-39, 14],
    [-45, 8],
    [-47, 0],
    [-45, -8],
    [-39, -14],
    [-31, -17],
    [-23, -15],
    [-15, -11],
    [-7, -6.5]
];

var waypointsTransformed = [];
var waypointsGlobal = [];

var planeVis = {
    x: 500,
    y: 500,
    angle: 30,
    img: null,
    size: 70
}

var testAnim = false;
var animation;

var playing = false;
var playingSpeed = 1;

function rotate(point, angle){
    point = math.matrix([[point[0]], [point[1]]]);
    var rot = math.matrix([[Math.cos(angle), -Math.sin(angle)], [Math.sin(angle), Math.cos(angle)]]);
    var res = math.multiply(rot, point);
    return [res._data[0][0], res._data[1][0]];
}

function localToGlobal(point){
    point = point.slice();
    console.log(point);
    point[0] -= visLeftColumn[0];
    point[1] -= visLeftColumn[1];
    console.log(point);
    var point2 = rotate(point, angle);
    console.log(angle);
    console.log(point2);
    point2[0] /= distanceRatio * visualizationScale;
    point2[1] /= distanceRatio * visualizationScale;
    console.log(point2);
    point2[1] /= yRatio;
    console.log(point2);
    point2[0] += leftColumn[0];
    point2[1] += leftColumn[1];
    console.log(point2);
    return point2;
}

function translateToLocal(point){
    point[0] -= leftColumn[0];
    point[1] -= leftColumn[1];
    point[1] *= yRatio;
    point[0] *= distanceRatio * visualizationScale;
    point[1] *= distanceRatio * visualizationScale;
    var point2 = rotate(point, -angle);
    point2[0] += visLeftColumn[0];
    point2[1] += visLeftColumn[1];
    return point2;
}

function distance(lat1, lon1, lat2, lon2) {
    if ((lat1 == lat2) && (lon1 == lon2)) {
        return 0;
    }else{
        var radlat1 = Math.PI * lat1/180;
        var radlat2 = Math.PI * lat2/180;
        var theta = lon1-lon2;
        var radtheta = Math.PI * theta/180;
        var dist = Math.sin(radlat1) * Math.sin(radlat2) + Math.cos(radlat1) * Math.cos(radlat2) * Math.cos(radtheta);
        if (dist > 1) {
            dist = 1;
        }
        dist = Math.acos(dist);
        dist = dist * 180/Math.PI;
        dist = dist * 60 * 1.1515;
        dist = dist * 1.609344 * 1000;
        return dist;
    }
}

function scaleWp(wp){
    return [wp[0]*scaleH, wp[1]*scaleV];
}

function centerWp(wp){
    return [wp[0]+canvas.width/2, wp[1]+canvas.height/2];
}


function updatePlanePosition(position){
    position = translateToLocal(position);
    planeVis.x = position[0];
    planeVis.y = position[1];
}

function visualizationFrame(){

    //background
    ctx.fillStyle = "#2f482d";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    //field
    ctx.fillStyle = '#3d6f37';
    ctx.beginPath();
    ctx.moveTo(100, 100);
    ctx.lineTo(1800, 100);
    ctx.lineTo(1800, 1300);
    ctx.lineTo(100, 1300);
    ctx.closePath();
    ctx.fill();
    waypointsTransformed.forEach((wp, ind) => {
        ctx.beginPath();
        ctx.arc(wp[0], wp[1], 10, 0, 2 * Math.PI, false);
        ctx.fillStyle = '#94004a';
        if(ind < testWp){
            ctx.fillStyle = "#1b4b7d";
        }
        ctx.fill();
    });

    //column left
    ctx.beginPath();
    ctx.arc(visLeftColumn[0], visLeftColumn[1], 30, 0, 2 * Math.PI, false);
    ctx.fillStyle = '#00add4';
    ctx.fill();

    //column right
    ctx.beginPath();
    ctx.arc(visRightColumn[0], visRightColumn[1], 30, 0, 2 * Math.PI, false);
    ctx.fillStyle = '#00add4';
    ctx.fill();

    //plane
    ctx.save();
    ctx.translate(planeVis.x, planeVis.y);
    ctx.rotate(planeVis.angle);
    ctx.translate(-planeVis.size/2, -planeVis.size/2);
    ctx.drawImage(planeVis.img, 0, 0, planeVis.size, planeVis.size);
    ctx.restore();
}

function visualizationInit(){
    var planeImg = new Image();
    
    canvas = document.getElementById("visualizationCanvas");
    ctx = canvas.getContext("2d");
    ctx.transform(1, 0, 0, -1, 0, canvas.height);

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

    $('#scaleHslider').on('input', _ => {
        waypointsTransformed = [];
        waypoints.forEach((wp, ind) => {
            waypointsTransformed.push(centerWp(scaleWp(wp)));
        });
        scaleH = ($('#scaleHslider').val() * 1.2 + 30)/10;
        visualizationFrame();
    });

    $('#scaleVslider').on('input', _ => {
        waypointsTransformed = [];
        waypoints.forEach((wp, ind) => {
            waypointsTransformed.push(centerWp(scaleWp(wp)));
        });
        scaleV = ($('#scaleVslider').val() * 1.9 + 10)/10;
        visualizationFrame();
    });
    
    yRatio = distance(leftColumn[1], leftColumn[0], rightColumn[1], leftColumn[0]) / distance(leftColumn[1], leftColumn[0], leftColumn[1], leftColumn[0] + Math.abs(rightColumn[0] - leftColumn[0]))

    angle = Math.atan(distance(rightColumn[1], leftColumn[0], rightColumn[1], rightColumn[0])/distance(leftColumn[1], leftColumn[0], rightColumn[1], leftColumn[0]));
    if(rightColumn[1] < leftColumn[1]){
        angle += Math.PI/2;
        if(rightColumn[0] < leftColumn[0]){
            angle += Math.PI/2;
        }
    }else{
        if(rightColumn[0] < leftColumn[0]){
            angle += Math.PI/2*3;
        }
    }

    distanceRatio = distance(leftColumn[1], leftColumn[0], rightColumn[1], leftColumn[0])/Math.abs(leftColumn[1] - rightColumn[1]);

    versorX = [rightColumn[0] - leftColumn[0], rightColumn[1] - leftColumn[1]];
    versorY = [-(rightColumn[1] - leftColumn[1]), rightColumn[0] - leftColumn[0]];

    versorXlen = Math.sqrt(versorX[0]*versorX[0] + versorX[1]*versorX[1]);
    versorX[0] /= versorXlen;

    var tempPlane = [19.875438578825595,  50.05479358675747];

    waypointsTransformed = [];
    waypoints.forEach((wp) => {
        waypointsTransformed.push(centerWp(scaleWp(wp)));
    });
    waypointsGlobal = [];
    waypointsTransformed.forEach((wp) => {
       waypointsGlobal.push(localToGlobal(wp));
    });

    markersToMap();

    updatePlanePosition(tempPlane);

    clog("Visualization ready", "info");
}

