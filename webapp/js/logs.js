function loadLogs(logs){
    $('#logsList tbody tr').remove();
    var logId = logs.length-1;
    logs.reverse().forEach(log => {
        $('#logsList tbody').append("<tr><td>" + logId + "</td><td>" + log + "</td></tr>");
        logId--;
    });
}