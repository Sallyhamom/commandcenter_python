function euclidean_dist(x0, y0, x1, y1) {
    return Math.sqrt(Math.pow(x0 - x1, 2) + Math.pow(y0 - y1, 2));
}

function point_to_index(point, size) {
    return point.x + (point.y * size.x);
}

function cells_on_line(x0, y0, dx, dy, size) {
    var nx = Math.abs(dx), ny = Math.abs(dy);
    var sx = (dx > 0 ? 1 : -1), sy = (dy > 0 ? 1 : -1);

    var points = [{x: x0, y: y0}];
    var cur_x = x0, cur_y = y0;
    for(let ix=0, iy=0; (ix <= nx) && (iy <= ny);) {
        if(Math.abs( ((0.5 + ix) / nx) - ((0.5 + iy) / ny)) < 0.0000001) {
            cur_x += sx;
            cur_y += sy;
            ix++;
            iy++;
        } else if((0.5 + ix) / nx < (0.5 + iy) / ny) {
            cur_x += sx;
            ix++;
        } else {
            cur_y += sy;
            iy++;
        }

        if(size !== undefined) {
            if(cur_x < 0 || cur_x > size.x) {
                break;
            }

            if(cur_y < 0 || cur_y > size.y) {
                break;
            }
        }

        points.push({ x: cur_x, y: cur_y });
    }

    return points;
}

class BinaryGrid {
    constructor(width, height) {
        this.size = {x: width, y: height};
        this.grid = new Uint8Array(width * height);
    }

    set_grid_point(x, y, state) {
        var idx = point_to_index({x,y}, this.size);
        this.grid[idx] = (state ? 255 : 0);
    }

    get_grid_point(x, y) {
        var idx = point_to_index({x,y}, this.size);
        return this.grid[idx] > 0;
    }

    set_line(p0, p1, state) {
        var points = cells_on_line(p0.x, p0.y, p1.x-p0.x, p1.y-p0.y);

        points.forEach(
            (p) => {
                var idx = point_to_index(p, this.size);
                this.grid[idx] = (state ? 255 : 0);
            }
        );
    }

    get_raycast_dist(x, y, theta, maxDist) {
        var dx = (Math.cos(theta)*maxDist);
        var dy = (Math.sin(theta)*maxDist);

        var points = cells_on_line(x, y, dx, dy, this.size);

        return points.reduce(
            (acc, val) => {
                var idx = point_to_index(val, this.size);
                if(this.grid[idx] === 0) {
                    return acc; // maintain old value
                }

                var dist = euclidean_dist(x, y, val.x, val.y);
                if(dist < acc) {
                    return dist;
                } else {
                    return acc;
                }
            }, maxDist
        );
    }

    render(imageData) {
        for(let y=0;y<this.size.y;y++) {
            for(let x=0;x<this.size.x;x++) {
                var idx = x + (this.size.x * y);
                if(this.grid[idx] > 0) {
                    render_point(imageData, {x,y}, this.size, {r:0, g:0, b:0});
                } else {
                    if(x % 10 === 0 || y % 10 === 0) {
                        render_point(imageData, {x,y}, this.size, {r:255-32, g:255-32, b:255-32});
                    } else {
                        render_point(imageData, {x,y}, this.size, {r:255, g:255, b:255});
                    }
                }
            }
        }

        return imageData;
    }
}


function render_point(imgData, p, size, color) {
    var idx = (p.x * 4) + (size.x * p.y * 4);
    imgData.data[idx] = color.r;
    imgData.data[idx+1] = color.g;
    imgData.data[idx+2] = color.b;
    imgData.data[idx+3] = 255;
}

class OccupancyGrid {
    /* grid scale = width/height of 1 grid square in cm */
    constructor(width, height, gridScale, min_occ, max_occ) {
        this.size = {x: width, y: height};
        this.grid = new Float64Array(width * height);
        this.scale = gridScale;
        this.min_occ = min_occ || Math.log(0.01 / 0.99);
        this.max_occ = max_occ || Math.log(0.99 / 0.01);
    }

    /*
     * negative modifier = lower probability occupied
     * positive modifier = higher probability occupied
     */
    sensorModifier(dist, dist_exp, dist_max, std_dev) {
        var variance = Math.pow(std_dev, 2);

        var max_range_size = 6*std_dev; // twice the three-sigma range

        // log(p_occ)
        var p_occ = -1 * Math.pow(dist - dist_exp, 2) / (2*variance);
        p_occ -= Math.log(Math.sqrt(2*Math.PI*variance));

        var p_free = (1 / dist_max);
        if(Math.abs(dist - dist_max) <= 3*std_dev) {
            // add max-range probability
            p_free += (1/max_range_size);
            p_free -= (1/(max_range_size * dist_max));
        }
        p_free = Math.log(p_free);

        //console.log(`log(p_occ) = ${p_occ}\nlog(p_free) = ${p_free}`);

        return p_occ - p_free;
        /*
        var rand = Math.log(dist_max / Math.sqrt(2*Math.PI*variance));
        var hit = Math.pow(dist - dist_exp, 2) / (2*variance);
        return rand - hit;
        */
    }

    diagonal_dist(theta, dist) {
        return Math.max(Math.abs(Math.cos(theta)), Math.abs(Math.sin(theta)))*dist;
    }

    /* x,y = position updating from
     * theta = direction of sensor beam (0 = +X axis, increases CCW)
     * dist
     * max_dist
     * std_dev = standard deviation of
     */
    sensorUpdate(x, y, theta, dist, max_dist, std_dev) {
        /* dx and dy are in units of grid squares */
        var dx = (Math.cos(theta)*dist) / this.scale;
        var dy = (Math.sin(theta)*dist) / this.scale;

        var points = cells_on_line(x, y, dx, dy, this.size);

        for(let i=0;i<points.length;i++) {
            let cur = points[i];
            let grid_dist = euclidean_dist(cur.x, cur.y, x, y);

            let modifier = this.sensorModifier(
                dist,
                grid_dist * this.scale, /* Go from grid scale to actual scale */
                max_dist,
                std_dev
            );

            let gridIdx = cur.x + (this.size.x * cur.y);
            this.grid[gridIdx] += modifier;

            this.grid[gridIdx] = Math.max(this.grid[gridIdx], this.min_occ);
            this.grid[gridIdx] = Math.min(this.grid[gridIdx], this.max_occ);
        }
    }

    render(imageData) {
        for(let y=0;y<this.size.y;y++) {
            for(let x=0;x<this.size.x;x++) {
                var idx = x + (this.size.x * y);
                if(this.grid[idx] > 0) {
                    render_point(imageData, {x,y}, this.size, {r:0, g:0, b:0});
                } else {
                    if(x % 10 === 0 || y % 10 === 0) {
                        render_point(imageData, {x,y}, this.size, {r:255-32, g:255-32, b:255-32});
                    } else {
                        render_point(imageData, {x,y}, this.size, {r:255, g:255, b:255});
                    }
                }
            }
        }

        return imageData;
    }
}