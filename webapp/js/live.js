function updateLiveData(data){
    data = data.split(',');
    data.forEach((val, ind) => {
        $('#liveData' + ind).html(val);
    });
    updateChart(parseFloat(data[3]));
}

function initLiveData(){
    clog("Live data initialized", "info");
}