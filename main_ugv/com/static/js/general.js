$(function() {
    startRefresh();
});

function startRefresh() {
    setTimeout(startRefresh,1000);
    $.get('pagelink.php', function(data) {
        $('#objectDetect').html(data);    
    });
}