var chart;

var chartData = [];
var chartIndex = 0;
var chartLength = 300;

function updateChart(val){
    chartData.push({
        x: chartIndex,
        y: val
    });
    chartIndex++;
    if(chartData.length > chartLength){
        chartData.shift();
    }
    chart.render();
}

function chartInit(){
    
    chart = new CanvasJS.Chart("chartDiv", {
        data: [{
            type: "line",
            dataPoints: chartData
        }],
        theme: "dark1"
    });

    clog("Chart ready", "info");
}