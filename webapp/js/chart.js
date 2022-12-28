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

function showChart(data){
    chartData.length = 0;
    data.forEach((el, val) => {
        chartData.push({
            x: val,
            y: el
        });
    });
    chart.render();
}

function chartInit(){
    
    chart = new CanvasJS.Chart("chartDiv", {
        data: [{
            type: "line",
            dataPoints: chartData
        }],
        //theme: "dark1"
		theme: "light1"
    });

    clog("Chart ready", "info");
}