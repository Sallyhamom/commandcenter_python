var ipaddress = sessionStorage.getItem('ipaddress');
//var localIP = sessionStorage.getItem('localIP');
const mqtt = require('mqtt');
var mqttClient = null;
/*var client = null;
if(localIP != null){
    const options = {
        keepalive: 60,
        schema: 'mqtts://',
        protocol: 'mqtt',
        protocolId: "MQTT",
        protocolVersion: 5,
        clean: true,
      };
    client = mqtt.connect("mqtt://"+localIP+":9883", options);
    client.on("connect", () => {
        console.log("Subscribing client connected:" + client);

      });
}*/
if(ipaddress != null){
    const options = {
        keepalive: 60,
        schema: 'mqtts://',
        protocol: 'mqtt',
        protocolId: "MQTT",
        protocolVersion: 5,
        clean: true,
      };
    mqttClient = mqtt.connect("mqtt://"+ipaddress+":9883", options);
    mqttClient.on("connect", () => {
        console.log("Client connected:" + mqttClient);

        //alert("Connected");
        $("#connected-image").removeClass("d-none");
        $("#ip-address").attr("disabled",true);;
      });

    $("#start-button").click(function(){
        if(!$(".destination").length){
            alert("Destination not added");
        }else{
            $("select").attr("disabled", "true");
        }
        var speed={};
        mqttClient.publish("/api/start_navigation", JSON.stringify(speed), { qos: 0, retain: false }, (error) => {
            if (error) {
              console.error(error)
            }
        });
    });

    $("#stop-button").click(function(){
        $("select").removeAttr("disabled");
        var speed={};
        mqttClient.publish("/api/stop_navigation", JSON.stringify(speed), { qos: 0, retain: false }, (error) => {
            if (error) {
              console.error(error)
            }
        });
    });

    $("#stop-all").click(function(){
        speed = {
            'state': 1
        }
        mqtt_publish("/api/estop", speed);
        stop();
    });
}

$("#via-station").click(function(){

        $("ul.timeline .from").after("<li class='timeline-item via'><div class='timeline-body'>"+
            "<div class='timeline-meta'><span style='font-size:10px;font-weight: bold;font-style: oblique;'>Via: </span></div><div class='timeline-content timeline-indicator'>"+
            "<select class='select' id='from-station' name='from-station' style='font-size:0.9em; margin-left:-10px;'>"+
            "<option value='-1'>Select Station</option><option value='Station 1'>Station 1</option><option value='Station 2'>Station 2</option>"+
            "<option value='Station 3'>Station 3</option></select></h6></div></li>");

});

$("#to-station").click(function(){
    if(!$(".destination").length){
        $("ul.timeline .timeline-item:last").after("<li class='timeline-item destination'><div class='timeline-body'>"+
                "<div class='timeline-meta'><span style='font-size:10px;font-weight: bold;font-style: oblique;'>To Station: </span></div><div class='timeline-content timeline-indicator'>"+
                "<select class='select' id='from-station' name='from-station' style='font-size:0.9em; margin-left:-10px;'>"+
                 "<option value='-1'>Select Station</option><option value='Station 1'>Station 1</option><option value='Station 2'>Station 2</option>"+
                 "<option value='Station 3'>Station 3</option></select></h6></div></li>");
    }
});

// --------------add active class-on another-page move----------
jQuery(document).ready(function($){
	// Get current path and find target link
	var path = window.location.pathname.split("/").pop();

	// Account for home page with empty path
	if ( path == '' ) {
		path = 'index.html';
	}

	var target = $('#navbarSupportedContent ul li a[href="'+path+'"]');
	// Add active class to target link
	target.parent().addClass('active');
});

$("#station-btn").click(function(){
    $("#pointModal .modal-title").text("Add Station")
    $("#pointModal").modal("show");
});
$(".modal-close").click(function(){
    $("#pointModal").modal("hide");
});

$("#dock-btn").click(function(){
    $("#pointModal .modal-title").text("Add Dock")
    $("#pointModal").modal("show");
});

$("#charging-btn").click(function(){
    $("#pointModal .modal-title").text("Add Charging Point")
    $("#pointModal").modal("show");
});

$("#savePoint").click(function(){
    $("#pointModal").modal("hide");
    document.querySelectorAll(".modal-body form")[0].reset();
})

document.querySelectorAll(`[effect="ripple"]`).forEach(el => {
    el.addEventListener('click', e => {
        e = e.touches ? e.touches[0] : e;
        const r = el.getBoundingClientRect(),
              d = Math.sqrt(Math.pow(r.width, 2) + Math.pow(r.height, 2)) * 2;
        el.style.cssText = `--s: 0; --o: 1;`;
        el.offsetTop;
        el.style.cssText = `--t: 1; --o: 0; --d: ${d}; --x:${e.clientX - r.left}; --y:${e.clientY - r.top};`;
    });
});
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

let current = this;
var viewer = new ROS2D.Viewer({
        divID : 'map',
        width : $(".map_space").width(),
        height : $(".map_space").height()
    });
current.rootObject = viewer.scene || new createjs.Container();
current.currentGrid = new createjs.Shape();
current.rootObject.addChild(this.currentGrid);
current.rootObject.addChild(new ROS2D.Grid({size:1}));
current.currentRobot = new ROS2D.NavigationImage({
    size : 3,
    image : './static/images/cat_assmbly.png',
    pulse: true,
    alpha : 1
});
current.rootObject.addChild(currentRobot);
/*const { open } = require('rosbag');
*/
/*async function logMessagesFromFooBar() {

    current.rootObject = viewer.scene || new createjs.Container();
    current.currentGrid = new createjs.Shape();
    current.rootObject.addChild(this.currentGrid);
    current.rootObject.addChild(new ROS2D.Grid({size:1}));

    const bag = await open('C:/Users/Sally Hamom/Downloads/2024-07-09-16-00-33/2024-07-09-16-00-33.bag');
    await bag.readMessages({ topics: ['/map'] }, (result) => {
        mqttClient.publish('/api/map', JSON.stringify(result.message), { qos: 0, retain: false }, (error) => {
            if (error) {
              console.error(error)
            }
        });

    });

    current.currentRobot = new ROS2D.NavigationImage({
        size : 2,
        image : './static/images/cat_assmbly.png',
        pulse: true,
        alpha : 1
    });
    console.log(currentRobot)
    current.rootObject.addChild(currentRobot);
    await bag.readMessages({ topics: ['/odometry/filtered'] }, (result) => {

        sleep(8000).then(() => {
            mqttClient.publish('/api/odom', JSON.stringify(result.message), { qos: 0, retain: false }, (error) => {
                if (error) {
                  console.error(error)
                }
            });
        });
    });

}

logMessagesFromFooBar();*/

function sleep (time) {
  return new Promise((resolve) => setTimeout(resolve, time));
}
if(mqttClient != null){
        mqttClient.subscribe('/api/map', formMap());
        mqttClient.subscribe('/api/odom', formMap());
    }

function formMap(){
    mqttClient.on('message', function (topic, message) {
        var msg = JSON.parse(message);
        var that = current;
        var index = null;
        if( topic == '/api/map'){
            //msg.info.resolution = 0.8
            if (that.currentGrid) {
              index = that.rootObject.getChildIndex(that.currentGrid);
              that.rootObject.removeChild(that.currentGrid);
            }
            that.currentGrid = new ROS2D.OccupancyGrid({
                message : msg
            });
            if (index !== null) {
              that.rootObject.addChildAt(that.currentGrid, index);
            }
            else {
              that.rootObject.addChild(that.currentGrid);
            }
            viewer.scaleToDimensions(currentGrid.width, currentGrid.height);
            viewer.shift(currentGrid.pose.position.x, currentGrid.pose.position.y);
        }else{
            var initScaleSet = false;
            var pose = msg.pose.pose;
            if (that.currentRobot) {
              index = that.rootObject.getChildIndex(that.currentRobot);
              //that.rootObject.removeChild(that.currentRobot);
            }

            that.currentRobot.x = pose.position.x;
            that.currentRobot.y = -pose.position.y;
            if (!initScaleSet) {
              that.currentRobot.scaleX = 1.0 / that.rootObject.scaleX;
              that.currentRobot.scaleY = 1.0 / that.rootObject.scaleY;
              initScaleSet = true;
            }

            that.currentRobot.rotation = that.rootObject.rosQuaternionToGlobalTheta(pose.orientation);

            that.currentRobot.visible = true;

        }
    });
}
/*
const yaml = require('js-yaml');
const fs   = require('fs');

try {
  const doc = yaml.load(fs.readFileSync('C:/Users/Sally Hamom/Downloads/map.yaml', 'utf8'));
  console.log(doc);
} catch (e) {
  console.log(e);
}
*/


