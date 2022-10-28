function loadLogs(logs){
    $('#logsList tbody tr').remove();
    var logId = logs.length-1;
    logs.reverse().forEach(log => {
        $('#logsList tbody').append("<tr><td>" + logId + "</td><td>" + log + "</td></tr>");
        logId--;
    });
    currentLogFile = logs[0];
    $('#fileNameSpan').html(currentLogFile);

    $('#logsList tbody tr').click((e)=>{
        currentLogFile = $($(e.target).parent().children()[1]).html();
        $('#fileNameSpan').html(currentLogFile);
    });

}