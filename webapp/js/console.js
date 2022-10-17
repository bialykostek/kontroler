var consoleAutoscroll = true;
var inputHistory = [];
var historyIndex = 0;

var maxRows = 500;

function clog(text, style){
    addConsoleOutput("now", "~", text, style);
}

function planeLog(text, style){
    addConsoleOutput("now", ">", text, style);
}

function addConsoleOutput(time, direction, text, style=""){
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

    $('#consoleOutput tbody').append('<tr class="' + style + '"><td class="consoleTime">' + time + ' ' + direction + '</td><td class="consoleMessage">' + text + '</td></tr>');
    if(consoleAutoscroll){
        $('#consoleOutput')[0].scrollTop = $('#consoleOutput')[0].scrollHeight;
    }
    if(direction == "<"){
        historyIndex = inputHistory.length + 1;
        inputHistory.push(text);
    }
    if($('#consoleOutput tbody tr').length > maxRows){
        $($('#consoleOutput tbody tr')[0]).remove();
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
            if($('#consoleInputText').val() != "clear"){
                sendToPlane($('#consoleInputText').val());
                
            }else{
                $('#consoleOutput tbody tr').remove();
            }
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
                $('#consoleInputText').focus();
            }
        }
        if(keycode == '40'){
            if(historyIndex < inputHistory.length-1){
                historyIndex++;
                $('#consoleInputText').val(inputHistory[historyIndex]);
                $('#consoleInputText').focus();
            }else{
                historyIndex = inputHistory.length;
                $('#consoleInputText').val('');
                $('#consoleInputText').focus();
            }
        }
    });

    $('#consoleOutput').on('scroll', _ => {
        consoleAutoscroll = false;
        if($('#consoleOutput')[0].scrollTop + $('#consoleOutput').height() + 2 >= $('#consoleOutput')[0].scrollHeight){
            consoleAutoscroll = true;
        }
    });

    clog("Console ready", "info");
}