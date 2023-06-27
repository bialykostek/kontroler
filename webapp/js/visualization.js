var leftColumn = [19.888026, 50.0252842];
var rightColumn = [19.8889826, 50.0250206];

var angle = 5.878002559431705;
var visualizationScale = 10;

var visLeftColumn = [550, 700];
var visRightColumn = [1350, 700];

var scaleEW = 1;
var scaleNS = 1;

var scaleH = 11;
var scaleV = 9;

var canvas;
var ctx; 

var lineA = 1;
var lineB = 1;
var lineC = 1;

var currentWaypoint = 0;

var drawLine = false;
var drawing = false;
var trackHis = [];

var nextPathAngles = [0, 0, 0, 0, 0, 0, 0];

var betweenWaypoints = 90;

var distanceToCurrent = 0;

var waypoints = [
    [0, 0],
    [7, 6.5],
    [15,11],
    [23,15],
    [31, 17],
    [39, 14],
    [44, 8.5],
    [47, 0],
    [44, -8.5],
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
    [-44, 8.5],
    [-47, 0],
    [-44, -8.5],
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
    long: 0,
    lat: 0,
    angle: 30,
    img: null,
    size: 70,
    heading: 0
}

function rotate(point, angle){
    point = math.matrix([[point[0]], [point[1]]]);
    var rot = math.matrix([[Math.cos(angle), -Math.sin(angle)], [Math.sin(angle), Math.cos(angle)]]);
    var res = math.multiply(rot, point);
    return [res._data[0][0], res._data[1][0]];
}

function localToGlobal(point){
    point = point.slice();
    point[0] -= visLeftColumn[0];
    point[1] -= visLeftColumn[1];
    point = rotate(point.slice(), angle);
    point[0] /= scaleEW * visualizationScale;
    point[1] /= scaleNS * visualizationScale;
    point[0] += leftColumn[0];
    point[1] += leftColumn[1];
    return point;
}

function globalToLocal(point){
    point = point.slice();
    point[0] -= leftColumn[0];
    point[1] -= leftColumn[1];
    point[0] *= scaleEW * visualizationScale;
    point[1] *= scaleNS * visualizationScale;
    point = rotate(point.slice(), -angle);
    point[0] += visLeftColumn[0];
    point[1] += visLeftColumn[1];
    return point;
}

function distance(lon1, lat1, lon2, lat2) {
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
    planeVis.long = position[0];
    planeVis.lat = position[1];
    position = globalToLocal(position);
    planeVis.x = position[0];
    planeVis.y = position[1];
    markersToMap();
}

function calibration(){
    angle = Math.atan(distance(leftColumn[0], rightColumn[1], rightColumn[0], rightColumn[1])/distance(leftColumn[0], leftColumn[1], leftColumn[0], rightColumn[1]));
    if(rightColumn[0] < leftColumn[0]){
        if(rightColumn[1] < leftColumn[1]){
            //III
            angle = Math.PI*3/2 - angle;
        }else{
            //II
            angle = Math.PI/2 + angle;
        }
    }else{
        if(rightColumn[1] < leftColumn[1]){
            //IV
            angle = Math.PI*3/2 + angle;
        }else{
            //I
            angle = Math.PI/2 - angle;
        }
    }

    scaleEW = distance(leftColumn[0], leftColumn[1], leftColumn[0] + 0.003, leftColumn[1])/0.003;
    scaleNS = distance(leftColumn[0], leftColumn[1], leftColumn[0], leftColumn[1] + 0.002)/0.002;
    waypointsTransformed = [];
    waypoints.forEach((wp) => {
        waypointsTransformed.push(centerWp(scaleWp(wp)));
    });
    waypointsGlobal = [];
    waypointsTransformed.forEach((wp) => {
       waypointsGlobal.push(localToGlobal(wp));
    });

    markersToMap();
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
        if(ind < currentWaypoint){
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

    //next waypoint line
    ctx.beginPath();
    ctx.lineWidth = 1;
    ctx.moveTo(0, -lineC/lineB);
    ctx.lineTo(canvas.width, -lineA/lineB*canvas.width - lineC/lineB);
    ctx.strokeStyle = "#000000";
    ctx.stroke();
    
    //next path trace
    ctx.beginPath();
    ctx.lineWidth = 3;
    ctx.save();
    ctx.translate(planeVis.x, planeVis.y);
    ctx.rotate(planeVis.angle);
    ctx.moveTo(0, 0);
    nextPathAngles.forEach((val, ind) => {
        ctx.rotate(val);
        if(ind == 0){
            ctx.translate(distanceToCurrent, 0);
        }else{
            ctx.translate(betweenWaypoints, 0);
        }
        ctx.lineTo(0, 0);       
    });
    ctx.restore();
    ctx.strokeStyle = "#eb34a5";
    ctx.stroke();

    //yaw line
    ctx.beginPath();
    ctx.lineWidth = 2;
    ctx.moveTo(planeVis.x, planeVis.y);
    if(planeVis.angle < Math.PI/2 || planeVis.angle > Math.PI*3/2){
        ctx.lineTo(canvas.width, canvas.width*Math.tan(planeVis.angle) + planeVis.y - Math.tan(planeVis.angle)*planeVis.x);
        
   }else{
        ctx.lineTo(0, planeVis.y - Math.tan(planeVis.angle)*planeVis.x);
    }
    ctx.strokeStyle = "#5feb34";
    ctx.stroke();

    //heading line
    ctx.beginPath();
    ctx.lineWidth = 2;
    ctx.moveTo(planeVis.x, planeVis.y);
    if(planeVis.heading < Math.PI/2 || planeVis.heading > Math.PI*3/2){
        ctx.lineTo(canvas.width, canvas.width*Math.tan(planeVis.heading) + planeVis.y - Math.tan(planeVis.heading)*planeVis.x);
   }else{
        ctx.lineTo(0, planeVis.y - Math.tan(planeVis.heading)*planeVis.x);
    }
    ctx.strokeStyle = "#0008ff";
    ctx.stroke();

    //plane
    ctx.save();
    ctx.translate(planeVis.x, planeVis.y);
    ctx.rotate(planeVis.angle);
    ctx.translate(-planeVis.size/2, -planeVis.size/2);
    ctx.drawImage(planeVis.img, 0, 0, planeVis.size, planeVis.size);
    if(drawing){
        if(trackHis.length == 0 || ((planeVis.x-trackHis[trackHis.length-1][0])*(planeVis.x-trackHis[trackHis.length-1][0]) + (planeVis.y-trackHis[trackHis.length-1][1])*(planeVis.y-trackHis[trackHis.length-1][1]) < 5000)){
          
            trackHis.push([planeVis.x, planeVis.y]);  
        }
    }
    ctx.restore();
    
    //plane track
    if(drawLine){
        ctx.beginPath();
        ctx.lineWidth = 3;
        ctx.moveTo(trackHis[0], trackHis[1]);
        trackHis.forEach((val) => {
            ctx.lineTo(val[0], val[1]);
        });
        ctx.strokeStyle = "#FFFF00";
        ctx.stroke();
    }
    
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

    $('#drawLineBtn').on('click', _ => {
        if(drawLine){
            drawLine = false;
            trackHis = [];
        }else{
            drawLine = true;
        }
    });
    
    calibration();

    clog("Visualization ready", "info");
}

