function mapInit(){
    map = new OpenLayers.Map("mapDiv");
    var mapnik         = new OpenLayers.Layer.OSM();
    var fromProjection = new OpenLayers.Projection("EPSG:4326");   // Transform from WGS 1984
    var toProjection   = new OpenLayers.Projection("EPSG:900913"); // to Spherical Mercator Projection
    var position       = new OpenLayers.LonLat(13.41,52.52).transform( fromProjection, toProjection);
    var zoom           = 15; 

    map.addLayer(mapnik);
    map.setCenter(position, zoom);

    clog("Map ready", "info");
}