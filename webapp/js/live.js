function updateLiveData(data){
    data = data.split(',');
    data.forEach((val, ind) => {
        $('#liveData' + ind).html(val);
    });
}

function initLiveData(){
    clog("Live data initialized", "info");
}