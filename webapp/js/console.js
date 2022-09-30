var consoleAutoscroll = true;
var websocket;
var inputHistory = [];
var historyIndex = 0;

function addConsoleOutput(time, direction, text, type="default"){
    let date = new Date();
    var h = date.getHours();
    var m = date.getMinutes();
    var s = date.getSeconds();
    if(h < 10){
        h = '0' + h;
    }
    if(m < 10){
        m = '0' + m;
    }
    if(s < 10){
        s = '0' + s;
    }
    if(time === "now"){
        time = h + ':' + m + ':' + s;
    }

    $('#consoleOutput tbody').append('<tr class="' + type + '"><td class="consoleTime">' + time + ' ' + direction + '</td><td class="consoleMessage">' + text + '</td></tr>');
    if(consoleAutoscroll){
        $('#consoleOutput')[0].scrollTop = $('#consoleOutput')[0].scrollHeight;
    }
    if(direction == "<"){
        historyIndex = inputHistory.length + 1;
        inputHistory.push(text);
    }
}

function sendToRemote(text){
    if(websocket.readyState == 1){
        websocket.send(JSON.stringify({
            type: 0,
            text: text
        }));
    }
}

function consoleInit(){
    $('#consoleOutput').on('mousedown', (e) => {
        var focus = true;
        $(document).on('mousemove', (ev) => {
            focus = false;
            $(document).off('mousemove');
        });
        $(document).on('mouseup', (ev) => {
            if(focus){
                $('#consoleInputText').focus();
            }
            $(document).off('mouseup');
            $(document).off('mousemove');
        });
    });

    $('#consoleInputText').keypress(function(event){
        event = event || window.event;
        var keycode = (event.keyCode ? event.keyCode : event.which);
        if(keycode == '13' && $('#consoleInputText').val() != ''){
            consoleAutoscroll = true;
            addConsoleOutput("now", '<', $('#consoleInputText').val());
            sendToRemote($('#consoleInputText').val());
            $('#consoleInputText').val('');
        }
    });

    $('#consoleInputText').on('keydown', function(event){
        event = event || window.event;
        var keycode = (event.keyCode ? event.keyCode : event.which);
        if(keycode == '38'){
            if(historyIndex > 0){
                historyIndex--;
                $('#consoleInputText').val(inputHistory[historyIndex]);
            }
        }
        if(keycode == '40'){
            if(historyIndex < inputHistory.length-1){
                historyIndex++;
                $('#consoleInputText').val(inputHistory[historyIndex]);
            }else{
                historyIndex = inputHistory.length;
                $('#consoleInputText').val('');
            }
            
        }
    });

    $('#consoleOutput').on('scroll', _ => {
        consoleAutoscroll = false;
        if($('#consoleOutput')[0].scrollTop + $('#consoleOutput').height() + 2 >= $('#consoleOutput')[0].scrollHeight){
            consoleAutoscroll = true;
        }
    });

    websocket = new WebSocket('ws://jkostecki.ddns.net:1111');
    websocket.onopen = _ => {
        websocket.send(JSON.stringify({
            type: 2,
            clientType: 2
        }));
    };
    websocket.onmessage = (data) => {
        data = JSON.parse(data.data);
        if(data.type == 1){
            addConsoleOutput("now", ">", data.text);
        }
    }
}