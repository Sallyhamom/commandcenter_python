const btn = document.querySelector(".action-btn");
if(btn != null){
    btn.onclick = function (e) {
        // Create span element
        let ripple = document.createElement("span");

        // Add ripple class to span
        ripple.classList.add("ripple");

        // Add span to the button
        this.appendChild(ripple);

        // Get position of X
        let x = e.clientX - e.currentTarget.offsetLeft;

        // Get position of Y
        let y = e.clientY - e.currentTarget.offsetTop;

        // Position the span element
        ripple.style.left = `${x}px`;
        ripple.style.top = `${y}px`;

        // Remove span after 0.3s
        setTimeout(() => {
            ripple.remove();
        }, 300);

    };
}

function test(){
	var tabsNewAnim = $('#navbarSupportedContent');
	var selectorNewAnim = $('#navbarSupportedContent').find('li').length;
	var activeItemNewAnim = tabsNewAnim.find('.active');
	if(activeItemNewAnim.length){
        var activeWidthNewAnimHeight = activeItemNewAnim.innerHeight();
        var activeWidthNewAnimWidth = activeItemNewAnim.innerWidth();
        var itemPosNewAnimTop = activeItemNewAnim.position();
        var itemPosNewAnimLeft = activeItemNewAnim.position();
        $(".hori-selector").css({
            "top":itemPosNewAnimTop.top + "px",
            "left":itemPosNewAnimLeft.left + "px",
            "height": activeWidthNewAnimHeight + "px",
            "width": activeWidthNewAnimWidth + "px"
        });
        $("#navbarSupportedContent").on("click","li",function(e){
            $('#navbarSupportedContent ul li').removeClass("active");
            $(this).addClass('active');
            var activeWidthNewAnimHeight = $(this).innerHeight();
            var activeWidthNewAnimWidth = $(this).innerWidth();
            var itemPosNewAnimTop = $(this).position();
            var itemPosNewAnimLeft = $(this).position();
            $(".hori-selector").css({
                "top":itemPosNewAnimTop.top + "px",
                "left":itemPosNewAnimLeft.left + "px",
                "height": activeWidthNewAnimHeight + "px",
                "width": activeWidthNewAnimWidth + "px"
            });
        });
    }
}
$(document).ready(function(){
	setTimeout(function(){ test(); });
});
$(window).on('resize', function(){
	setTimeout(function(){ test(); }, 500);
});
$(".navbar-toggler").click(function(){
	$(".navbar-collapse").slideToggle(300);
	setTimeout(function(){ test(); });
});

'use-script';
let batteryLevelDisplay = document.querySelector('.battery--level');
let batteryPercent = document.querySelector('.battery--percent');

function battery() {
    navigator.getBattery().then(function (battery) {
        let batteryLevel = battery.level * 100;

        batteryLevelDisplay.style.width = batteryLevel + '%';
        batteryPercent.textContent = batteryLevel + '%';

        if (batteryLevel > 80) {
            batteryLevelDisplay.style.backgroundColor = `var(--green)`;
            batteryPercent.style.color = `var(--green)`;
        } else if (batteryLevel > 60 && batteryLevel <= 80) {
            batteryLevelDisplay.style.backgroundColor = `var(--light-green)`;
            batteryPercent.style.color = `var(--light-green)`;
        } else if (batteryLevel > 40 && batteryLevel <= 60) {
            batteryLevelDisplay.style.backgroundColor = `var(--light-red)`;
            batteryPercent.style.color = `var(--light-red)`;
        } else {
            batteryLevelDisplay.style.backgroundColor = `var(--red)`;
            batteryPercent.style.color = `var(--red)`;
        }
    });
}

if(batteryLevelDisplay != null){
    battery();
    setInterval(battery,1000);
}

$("#view-map").click(function(){
    if(!$(".map_space input").length){
        $(".map_space").append('<input type="image" src="static/images/perspective_grid_pattern.jpg" style="width:100%;height:80%;">');
    }
});

//stop button.
$("#stop-button").click(function(){
    if($("select:disabled").length){
        $("select:disabled").removeAttr("disabled");
    }
})


$(".accept").click(function(){
    var accessId = [{"accessId": $(this).attr('id').split("_")[1]},
                    {"role": $(this).parents('tr').first().find('select').val()}];
    $.ajax({
      type: "POST",
      url: "/add",
      data: JSON.stringify(accessId),
      contentType: "application/json",
      dataType: 'json'
    });
});

$(".reject").click(function(){
    console.log($(this).attr('id'));
});