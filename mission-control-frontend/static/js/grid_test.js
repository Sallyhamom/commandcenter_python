
//var canvas_actual = document.getElementById('canvas_actual');
var canvas_occ = document.getElementById('map-canvas');
//var ctx_actual = canvas_actual.getContext('2d');
var ctx_occ = canvas_occ.getContext('2d');

var map_scale = 10; // cm per grid square side
var beam_dist_grid = 400; // 400 grid squares long
var beam_dist_actual = beam_dist_grid * map_scale; // ... in cm
var base_point = {x:300, y:300}

/* The specified accuracy for the LidarLite-2v3 is +/- 2.5cm-- I'm taking that
 * as the 2-sigma value, so sigma is 2.5/2.
 */
var sensor_real_stddev = (10 / 2);

/* The absolute encoder jumps a bit sometimes, randomly increasing or decreasing by 1.
 * So the 2-sigma range is ((1 / 1024) * 2PI) radians; the 2's cancel out,
 * so sigma is just 1 * (PI/1024).
 * In reality, it's probably slightly lower, but this is being conservative.
 */
var angle_real_stddev = (Math.PI / 1024);

var sensor_stddev = sensor_real_stddev;
var angle_stddev = angle_real_stddev;

const width = 600, height = 600;

function random_normal() {
    var u = 0, v = 0;
    while(u === 0) u = Math.random(); //Converting [0,1) to (0,1)
    while(v === 0) v = Math.random();
    return Math.sqrt( -2.0 * Math.log( u ) ) * Math.cos( 2.0 * Math.PI * v );
}

var map_grid = new OccupancyGrid(width, height, map_scale);
var actual_grid = new BinaryGrid(width, height);

/* Setup initial walls with corners at:
 * (50, 50) -- (550, 50)
 * |            |
 * (50, 550) -- (550, 550)
 */
var c0 = {x:50, y:50}, c1 = {x: 550, y: 50}, c2 = {x: 50, y: 550}, c3 = {x:550, y:550};
actual_grid.set_line(c0, c1, true);
actual_grid.set_line(c0, c2, true);
actual_grid.set_line(c3, c1, true);
actual_grid.set_line(c3, c2, true);

var angle_elem = document.getElementById('angle');

function do_map_update(angle_rad) {
    var noisy_angle = (random_normal() * angle_stddev) + angle_rad;
    var raycast_cells = actual_grid.get_raycast_dist(base_point.x, base_point.y, noisy_angle, beam_dist_grid);
    var raycast_actual = raycast_cells * map_scale;
    var raycast_noisy = (random_normal() * sensor_stddev) + raycast_actual;
    //console.log(`Raycast distance: ${raycast_cells} cells, ${raycast_actual} cm`);
    //console.log(`Noisy value: ${raycast_noisy}`);

    map_grid.sensorUpdate(base_point.x, base_point.y, angle_rad, raycast_noisy, beam_dist_actual, sensor_stddev);
}

function render_lines(angle_rad) {
    var dx = Math.cos(angle_rad) * beam_dist_grid;
    var dy = Math.sin(angle_rad) * beam_dist_grid;

    var raycast_cells = actual_grid.get_raycast_dist(base_point.x, base_point.y, angle_rad, beam_dist_grid);
    var rx = Math.cos(angle_rad) * raycast_cells;
    var ry = Math.sin(angle_rad) * raycast_cells;

    var beam_points = cells_on_line(base_point.x, base_point.y, dx, dy, actual_grid.size);
    var raycast_points = cells_on_line(base_point.x, base_point.y, rx, ry, actual_grid.size);

    /*var actual_img_data = ctx_actual.createImageData(width, height);
    actual_img_data = actual_grid.render(actual_img_data);*/

    var occ_img_data = ctx_occ.createImageData(width, height);
    occ_img_data.data.set(getBagData.data)
    occ_img_data = map_grid.render(occ_img_data);

    /*beam_points.forEach(
        (p) => {
            //render_point(actual_img_data, p, actual_grid.size, {r:0, g: 0, b: 255});
        }
    );*/

    raycast_points.forEach(
        (p) => {
            //render_point(actual_img_data, p, actual_grid.size, {r:255, g: 0, b: 0});
            render_point(occ_img_data, p, map_grid.size, {r:255, g: 0, b: 0});
        }
    );

    //ctx_actual.putImageData(actual_img_data, 0, 0);
    ctx_occ.putImageData(occ_img_data, 0, 0);
}

function make_block(x,y, radius, state) {
    for(let ix=-radius;ix<=radius;ix++) {
        for(let iy=-radius;iy<=radius;iy++) {
            actual_grid.set_grid_point(x+ix, y+iy, state);
        }
    }
}

var editState = false;
canvas_actual.onmousedown = (event) => {
    var x = event.pageX - canvas_actual.offsetLeft;
    var y = event.pageY - canvas_actual.offsetTop;

    editState = !actual_grid.get_grid_point(x,y);
    if(editState) {
        make_block(x, y, 3, editState);
    } else {
        make_block(x, y, 5, editState);
    }
}

canvas_actual.onmousemove = (event) => {
    var x = event.pageX - canvas_actual.offsetLeft;
    var y = event.pageY - canvas_actual.offsetTop;

    var lmb = (event.buttons & 1);
    if(lmb > 0) {
        if(editState) {
            make_block(x, y, 3, editState);
        } else {
            make_block(x, y, 5, editState);
        }
    }
}

var toggleSweepButton = document.getElementById('toggle-sweep');
var sweepState = false;
toggleSweepButton.textContent = (sweepState ? 'Stop Sweep' : 'Start Sweep');

toggleSweepButton.onclick = (event) => {
    sweepState = !sweepState;
    toggleSweepButton.textContent = (sweepState ? 'Stop Sweep' : 'Start Sweep');
}

/* sensor sweep */
var angle_deg = 0;
var keys = { left: false, right: false, up: false, down: false };

window.onkeydown = (event) => {
    switch(event.key) {
        case 'ArrowDown':
            keys.down = true;
            break;
        case 'ArrowUp':
            keys.up = true;
            break;
        case 'ArrowLeft':
            keys.left = true;
            break;
        case 'ArrowRight':
            keys.right = true;
            break;
    }
}

window.onkeyup = (event) => {
    switch(event.key) {
        case 'ArrowDown':
            keys.down = false;
            break;
        case 'ArrowUp':
            keys.up = false;
            break;
        case 'ArrowLeft':
            keys.left = false;
            break;
        case 'ArrowRight':
            keys.right = false;
            break;
    }
}


window.setInterval(() => {
    var angle_rad = angle_deg * (Math.PI / 180.0);

    if(keys.left) {
        base_point.x = Math.max(base_point.x-5, 0);
    }

    if(keys.right) {
        base_point.x = Math.min(base_point.x+5, actual_grid.size.x);
    }

    if(keys.up) {
        base_point.y = Math.max(base_point.y-5, 0);
    }

    if(keys.down) {
        base_point.y = Math.min(base_point.y+5, actual_grid.size.y);
    }

    if(sweepState) {
        var old_angle = angle_deg;
        angle_deg += 360 / 10;

        for(let i=old_angle; Math.abs(i-angle_deg) >= 0.5; i += 0.25) {
            angle_rad = i * (Math.PI / 180.0);
            do_map_update(angle_rad);
        }

        angle_rad = angle_deg * (Math.PI / 180.0);
    }

    render_lines(angle_rad);
}, 10);


render_lines(0);