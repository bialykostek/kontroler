function chartInit(){
    
    var dps = []; // dataPoints
    var chart = new CanvasJS.Chart("chartDiv", {
        data: [{
            type: "line",
            dataPoints: dps
        }],
        theme: "dark1"
    });

    var xVal = 0;
    var yVal = 100; 
    var updateInterval = 50;
    var dataLength = 300; // number of dataPoints visible at any point

    var updateChart = function (count) {

        count = count || 1;

        for (var j = 0; j < count; j++) {
            yVal = yVal +  Math.round(5 + Math.random() *(-5-5));
            dps.push({
                x: xVal,
                y: yVal
            });
            xVal++;
        }

        if (dps.length > dataLength) {
            dps.shift();
        }

        chart.render();
    };

    updateChart(dataLength);
    setInterval(function(){updateChart()}, updateInterval);


    clog("Chart ready", "info");
}