var verticalRatioL = 0.6;
var verticalRatioR = 0.5;
var horizontalRatio = 0.6;

function resizeHandler(){
    $('#leftColumn').css('width', 'calc(' + Math.round(horizontalRatio*100) + '% - 0.5em)');
    $('#rightColumn').css('width', 'calc(' + (100 - Math.round(horizontalRatio*100)) + '% - 0.5em)');
    $('#quaterTL').css('height', 'calc(' + Math.round(verticalRatioL*100) + '% - 0.5em)');
    $('#quaterBL').css('height', 'calc(' + (100 - Math.round(verticalRatioL*100)) + '% - 0.5em)');
    $('#quaterTR').css('height', 'calc(' + Math.round(verticalRatioR*100) + '% - 0.5em)');
    $('#quaterBR').css('height', 'calc(' + (100 - Math.round(verticalRatioR*100)) + '% - 0.5em)');
}

function relocatePage(id, quater){
    if($('#'+id).closest('.quater').attr('id') == 'quater'+quater) {
        return;
    }

    if($('.pageTitle[data-page="'+id+'"]').hasClass('active')){
        setActive($('.pageTitle[data-page="'+id+'"]').closest('.quater').find('.pageTitle').not('.active').data('page'));
    }
    
    $('.pageTitle[data-page="'+id+'"]').remove();

    $('#'+id).appendTo("#quater" + quater + " .quaterContent");
    $("#quater" + quater + " .quaterHeader").append("<div class='pageTitle' data-page='" + $('#'+id).attr('id') + "'>" + $('#'+id).data('title') + "</div>");
    if($('#'+id).closest('.quater').find('.pageTitle').length == 1){
        setActive(id);
    }else{
        $('#'+id).hide();
    }
}

function setActive(id){
    $('#' + id).closest('.quater').find('.pageTitle.active').removeClass('active');
    $('#' + id).closest('.quater').find('.page').hide();
    $('#' + id).show();
    $('.pageTitle[data-page=' + id + ']').addClass('active');
}

function titleDragHandler(e){
    var isDrag = false;
    e.preventDefault();

    $(document).on('mousemove', _ => {
        isDrag = true;
    });

    $(document).on('mouseup', (ev) => {
        if(!isDrag){
            setActive($(e.target).data('page'));
        }
        if(ev.clientX > $('#quaterTL').offset().left && ev.clientX < $('#quaterTL').offset().left + $('#quaterTL').width() && ev.clientY > $('#quaterTL').offset().top && ev.clientY < $('#quaterTL').offset().top + $('#quaterTL').height()){
            relocatePage($(e.target).data('page'), 'TL');    
        }
        else if(ev.clientX > $('#quaterTR').offset().left && ev.clientX < $('#quaterTR').offset().left + $('#quaterTR').width() && ev.clientY > $('#quaterTR').offset().top && ev.clientY < $('#quaterTR').offset().top + $('#quaterTR').height()){
            relocatePage($(e.target).data('page'), 'TR');    
        }
        else if(ev.clientX > $('#quaterBL').offset().left && ev.clientX < $('#quaterBL').offset().left + $('#quaterBL').width() && ev.clientY > $('#quaterBL').offset().top && ev.clientY < $('#quaterBL').offset().top + $('#quaterBL').height()){
            relocatePage($(e.target).data('page'), 'BL');    
        }
        else if(ev.clientX > $('#quaterBR').offset().left && ev.clientX < $('#quaterBR').offset().left + $('#quaterBR').width() && ev.clientY > $('#quaterBR').offset().top && ev.clientY < $('#quaterBR').offset().top + $('#quaterBR').height()){
            relocatePage($(e.target).data('page'), 'BR');    
        }
        $(document).off('mouseup');
        $(document).off('mousemove');
        $('.pageTitle').off('mousedown');
        $('.pageTitle').on('mousedown', (e) => {
            titleDragHandler(e);
        });
    });
}

function panelsInit(){

    $(window).on('resize', resizeHandler);

    $('#columnDivider').on('mousedown', (e) => {
        e.preventDefault();
        $(document).on('mousemove', (e) => {
            var trueInner = $('#bigContainer').innerWidth() - parseFloat($('#bigContainer').css('padding-left')) - parseFloat($('#bigContainer').css('padding-right'));
            var newWidthL = e.clientX - $('#bigContainer').offset().left - parseFloat($('#bigContainer').css('padding-left')) - $('#columnDivider').width()/2;
            if(newWidthL < trueInner/10){
                newWidthL = trueInner/10;
            }
            var newWidthR = Math.round(trueInner - $('#columnDivider').width() - newWidthL - 0.5);
            if(newWidthR < trueInner/10){
                newWidthL = 9*trueInner/10 - $('#columnDivider').width() - 0.5;
                newWidthR = trueInner/10;
            }
            $('#leftColumn').css('width', newWidthL);
            $('#rightColumn').css('width', newWidthR);
            horizontalRatio = newWidthL/trueInner;
        });
        $(document).on('mouseup', (e) => {
            $(document).off('mousemove');
            $(document).off('mouseup');
        });
    });

    $('#cellsDividerL').on('mousedown', (e) => {
        e.preventDefault();
        $(document).on('mousemove', (e) => {
            var trueInner = $('#leftColumn').innerHeight() - parseFloat($('#leftColumn').css('padding-top')) - parseFloat($('#leftColumn').css('padding-bottom'));
            var newHeightT = e.clientY - $('#leftColumn').offset().top - parseFloat($('#leftColumn').css('padding-top')) - $('.cellsDivider').height()/2;
            if(newHeightT < trueInner/10){
                newHeightT = trueInner/10;
            }
            var newHeightB = Math.round(trueInner - $('.cellsDivider').height() - newHeightT);
            if(newHeightB < trueInner/10){
                newHeightB = trueInner/10;
                newHeightT = trueInner/10 * 9 - $('.cellsDivider').height();
            }
            $('#quaterTL').css('height', newHeightT);
            $('#quaterBL').css('height', newHeightB);
            verticalRatioL = newHeightT/trueInner;
        });
        $(document).on('mouseup', (e) => {
            $(document).off('mousemove');
            $(document).off('mouseup');
        });
    });

    $('#cellsDividerR').on('mousedown', (e) => {
        e.preventDefault();
        $(document).on('mousemove', (e) => {
            var trueInner = $('#rightColumn').innerHeight() - parseFloat($('#rightColumn').css('padding-top')) - parseFloat($('#rightColumn').css('padding-bottom'));
            var newHeightT = e.clientY - $('#rightColumn').offset().top - parseFloat($('#rightColumn').css('padding-top')) - $('.cellsDivider').height()/2;
            if(newHeightT < trueInner/10){
                newHeightT = trueInner/10;
            }
            var newHeightB = Math.round(trueInner - $('.cellsDivider').height() - newHeightT);
            if(newHeightB < trueInner/10){
                newHeightB = trueInner/10;
                newHeightT = trueInner/10 * 9 - $('.cellsDivider').height();
            }
            $('#quaterTR').css('height', newHeightT);
            $('#quaterBR').css('height', newHeightB);
            verticalRatioR = newHeightT/trueInner;
        });
        $(document).on('mouseup', (e) => {
            $(document).off('mousemove');
            $(document).off('mouseup');
        });
    });

    $('.page').each((ind, page) => {
        relocatePage($(page).attr('id'), $(page).data('default'));
    });

    $('.quater').each((ind, quater) => {
        setActive($(quater).find('.page').attr('id'));
    });
    $('.pageTitle').on('mousedown', (e) => {
        titleDragHandler(e);
    });

    resizeHandler();

    $('.quaterHeader').sortable();

    clog("Panel layout ready", "info");
}