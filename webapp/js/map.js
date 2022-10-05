var map;
var markers;

function addMarker(point, icon){
    var lonLat = new OpenLayers.LonLat(point[1], point[0]).transform(new OpenLayers.Projection("EPSG:4326"), map.getProjectionObject());
    var marker = new OpenLayers.Marker(lonLat);
    markers.addMarker(marker);
    marker.setUrl(icon);
    map.setCenter(lonLat);
}

function markersToMap(){
    markers.destroy();
    markers = new OpenLayers.Layer.Markers("Markers");
    map.addLayer(markers);
    
    waypointsGlobal.forEach((wp) => {
        addMarker(wp, 'static/waypoint.png');
    });

    addMarker(localToGlobal([100, 100]), 'static/border.png');
    addMarker(localToGlobal([1800, 100]), 'static/border.png');
    addMarker(localToGlobal([1800, 1300]), 'static/border.png');
    addMarker(localToGlobal([100, 1300]), 'static/border.png');

    addMarker(localToGlobal(visLeftColumn), 'static/column.png');
    addMarker(localToGlobal(visRightColumn), 'static/column.png');

}

function mapInit(){
    map = new OpenLayers.Map("mapDiv");
    var mapnik         = new OpenLayers.Layer.OSM();
    var fromProjection = new OpenLayers.Projection("EPSG:4326");   // Transform from WGS 1984
    var toProjection   = new OpenLayers.Projection("EPSG:900913"); // to Spherical Mercator Projection
    var position       = new OpenLayers.LonLat(13.41,52.52).transform(fromProjection, toProjection);
    var zoom           = 15; 

    map.addLayer(mapnik);
    map.setCenter(position, zoom);

    markers = new OpenLayers.Layer.Markers("Markers");
    map.addLayer(markers);
     

    clog("Map ready", "info");
}