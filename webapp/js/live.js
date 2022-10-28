function updateLiveData(data){
    data = data.split(',');
    data.forEach((val, ind) => {
        $('#liveData' + ind).html(val);
    });
    updateChart(parseFloat(data[3]));

    planeVis.x = parseInt(data[20]);
    planeVis.y = parseInt(data[21]);
    planeVis.long = parseInt(data[10])/10000000;
    planeVis.lat = parseInt(data[9])/10000000;
    planeVis.angle = parseFloat(data[8]);

    lineA = parseFloat(data[17]);
    lineB = parseFloat(data[18]);
    lineC = parseFloat(data[19]);

    nextPathAngles[0] = parseFloat(data[23]);
    nextPathAngles[1] = parseFloat(data[24]);
    nextPathAngles[2] = parseFloat(data[26]);
    nextPathAngles[3] = parseFloat(data[27]);
    nextPathAngles[4] = parseFloat(data[28]);
    nextPathAngles[5] = parseFloat(data[29]);
    nextPathAngles[6] = parseFloat(data[30]);

    distanceToCurrent = parseFloat(data[31]);
    currentWaypoint = parseInt(data[22]);

    planeVis.heading = parseFloat(data[14]);
    
    visualizationFrame();
    var lonLat = new OpenLayers.LonLat(planeVis.long, planeVis.lat).transform(new OpenLayers.Projection("EPSG:4326"), map.getProjectionObject());
    markers.markers[6].moveTo(lonLat);
    map.setCenter(lonLat);

}

function initLiveData(){
    clog("Live data initialized", "info");
}