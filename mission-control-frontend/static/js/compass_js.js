var mqttClient;
//const allElements = document.querySelectorAll('.disable-btn');
var connected = false;
var joystick_disabled = false;
var stage = new createjs.Stage('joystick');
var psp = new createjs.Shape();
var xCenter = 0;
var yCenter = 0;
var joystick_speed={};
let intervalId = null;
$(".clamp-line").animate({
    width: $(".clamp-line").width() + 10
});
(function () {
  const base = document.querySelector("#base-container");
  const stick = document.querySelector("#stick-area");
  const display = document.querySelector("#controller-values-display");
  const displayX = display.children[0].children[0];
  const displayZ = display.children[1].children[0];
  let held = false;
  var coords = {};
  const getCenter = () => {
    const { x, y, width, height } = base.getBoundingClientRect();
    return {
      x: x + width / 2,
      y: y + height / 2
    };
  };

  const updateDisplay = ({ x, y }) => {
    if(isNaN(x) && isNaN(y)){
        x = 0;
        y = 0;
    }

    displayX.innerHTML = -1 * (y).toFixed(1);
    displayZ.innerHTML = 1 * (x).toFixed(1);

  };

  const getNormalizedPosition = (dX, dY, limiter = 0.5 /* default limter strength. 0 for free, higher values for tighter */) => {
    const radius = base.getBoundingClientRect().width / 2 / (1 + limiter);
    const direction = Math.atan2(dY, dX);
    const magnitude = Math.min(radius, Math.sqrt(dY ** 2 + dX ** 2));
    const pos = {
      dX: magnitude * Math.cos(direction),
      dY: magnitude * Math.sin(direction)
    };
    coords.x=isNaN(pos.dX / radius) ? 0 : pos.dX / radius;
    coords.y=isNaN(pos.dY / radius) ? 0 : pos.dY / radius;
    var x = pos.dX / radius;
    var y = pos.dY / radius
    updateDisplay({
      x: x,
      y: y
    });

    if(mqttClient != undefined){

        joystick_speed = {
            "linear": {
               'x': -1 * (y).toFixed(1)
            },
            "angular" : {
                "z":  (y).toFixed(1) > 0 ? 1 * (x).toFixed(1) : -1 * (x).toFixed(1)
            }
        }
        if(x==0 && y == 0){
            stop();
            if(!joystick_disabled){
                repeat(() => {
                    if(!joystick_disabled){
                        mqtt_publish("/api/cmd_vel", joystick_speed);
                    }
                 }, 20);
            }else{
                stop()
                //clearInterval(timer);
            }

        }else{
            if(!joystick_disabled){
                mqtt_publish("/api/cmd_vel", joystick_speed);
                delay(function(){
                    mqtt_publish("/api/cmd_vel", joystick_speed);
                }, 20 );
            }/*else{
                clearTimeout(intervalId);
            }*/
        }
    }
    return pos;
  };

  const setStickPosition = (_dX, _dY) => {
    const { dX, dY } = getNormalizedPosition(_dX, _dY);
    stick.style.left = dX + "px";
    stick.style.top = dY + "px";
  };

  const setStickPositionViaInput = ({ clientX, clientY }) => {
    const { x, y } = getCenter();
    const dX = clientX - x;
    const dY = clientY - y;
    setStickPosition(dX, dY);
  };

  [
    "mousedown", "touchstart"
  ].forEach(type => base.addEventListener(type, ev => {
    held = true;
    setStickPositionViaInput(ev.type === "touchstart" ? ev.touches[0] : ev);
  }));

  [
    "mousemove", "touchmove"
  ].forEach(type => document.addEventListener(type, ev => {
    if (held) setStickPositionViaInput(ev.type === "touchmove" ? ev.touches[0] : ev);
  }));

  [
    "mouseup", "mouseleave", "touchend"
  ].forEach(type => document.addEventListener(type, ev => {
    held = false;
    setStickPosition(0, 0);
  }));
})();
var lt = 0;
function init(){

    var vertical = new createjs.Shape();
    var horizontal = new createjs.Shape();
    /*vertical.graphics.beginFill('#334857').drawRect(80, 0, 6, 140);
    horizontal.graphics.beginFill('#334857').drawRect(80, 85, 60, 6);*/
    vertical.graphics.beginFill('#334857').drawRect(100, 0, 6, 120);
    horizontal.graphics.beginFill('#334857').drawRect(20, 0, 80, 6);
    stage.addChild(psp);
    stage.addChild(vertical);
    stage.addChild(horizontal);
    stage.update();
    var myElement = $('#joystick')[0];
    var mc = new Hammer(myElement);
}
(createjs.Graphics.Dash = function(instr) {
    if (instr == null) { instr = [0]; }
    this.instr = instr;
}).prototype.exec = function(ctx) {
    ctx.setLineDash(this.instr);
};
createjs.Graphics.prototype.dash = function(instr) {
    return this.append(new createjs.Graphics.Dash(instr));
}

const mqtt = require('mqtt');

function connectToBroker() {

  const options = {
    keepalive: 60,
    schema: 'mqtts://',
    protocol: 'ws',
    protocolId: "MQTT",
    protocolVersion: 5,
    clean: true,
  };
  mqttClient = mqtt.connect("ws://"+$("#ip-address").val()+":9883", options);
  mqttClient.on("connect", () => {
    console.log("Client connected.");
    //alert("Connected");
    $("#connected-image").removeClass("d-none");
    $("#ip-address").attr("disabled",true);;
  });

}

function mqtt_publish(topic, payload){
    if(mqttClient != undefined && mqttClient != null){
        mqttClient.publish(topic, JSON.stringify(payload), { qos: 0, retain: false }, (error) => {
            if (error) {
              console.error(error)
            }
          });
    }
}

let timer = null;
function repeat(what, time) {
    //if(!joystick_disabled){
      timer = setInterval(what, time); // Schedule
      //timer = window.setTimeout(what, 2000);
      //what;
   // }
}

var delay = ( function() {
    var timer = 0;
    return function(callback, ms) {
        clearTimeout (timer);
        timer = setTimeout(callback, ms);
    };
})();

function stop() {
  clearInterval(timer);
  //clearTimeout(intervalId);
}

$("#disconnect-btn").click(function(){
    mqttClient.end();
    $("#direction-value").val("");
    $("#ip-address").removeAttr("disabled");
    $("#connected-image").addClass("d-none");
    clearTimeout(intervalId);
    stop();
    $('.button#fog-lamp').parent('.plate').removeClass('on');
    $('.button#head-lamp').parent('.plate').removeClass('on');
    $('.button#white-lamp').parent('.plate').removeClass('on');
});
var lamp_val = {
    'front_white': 0,
    'yellow_fog': 0,
    'white_fog': 0
}

$('.button#white-lamp').click(function(){
    if(!$('.button#white-lamp').parent('.plate').hasClass('on')){
        lamp_val.white_fog = 1;
    }else{
        lamp_val.white_fog = 0;
    }
    if(mqttClient != undefined && mqttClient != null){
        mqtt_publish("/api/input_states", lamp_val);
    }
    $('.button#white-lamp').parent('.plate').toggleClass('on');
});

$('.button#fog-lamp').click(function(){

    if(!$('.button#fog-lamp').parent('.plate').hasClass('on')){
        lamp_val.yellow_fog = 1;
    }else{
        lamp_val.yellow_fog = 0;
    }
    if(mqttClient != undefined && mqttClient != null){
        mqtt_publish("/api/input_states", lamp_val);
    }
    $('.button#fog-lamp').parent('.plate').toggleClass('on');
});

$('.button#head-lamp').click(function(){

    if(!$('.button#head-lamp').parent('.plate').hasClass('on')){
        lamp_val.front_white = 1;
    }else{
        lamp_val.front_white = 0;
    }
    if(mqttClient != undefined && mqttClient != null){
        mqtt_publish("/api/input_states", lamp_val);
    }
    $('.button#head-lamp').parent('.plate').toggleClass('on');
});

$("#right-fwd").click(function(){
    if(mqttClient != undefined && mqttClient != null){
        mqtt_publish("/api/r_fwd", {'state':1});
    }
})

$("#right-rev").click(function(){
    if(mqttClient != undefined && mqttClient != null){
        mqtt_publish("/api/r_rev", {'state':1});
    }
})

$("#left-fwd").click(function(){
    if(mqttClient != undefined && mqttClient != null){
        mqtt_publish("/api/l_fwd", {'state':1});
    }
})

$("#left-rev").click(function(){
    if(mqttClient != undefined && mqttClient != null){
        mqtt_publish("/api/l_rev", {'state':1});
    }
})