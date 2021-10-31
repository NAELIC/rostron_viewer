const canvas = document.querySelector("#field")
const ctx = document.querySelector("#field").getContext("2d");

// Canvas variable
var zoom_mult = 1.5;

var canvas_size = {
    width: window.innerWidth,
    height: window.innerHeight
}

ctx.canvas.width = canvas_size.width;
ctx.canvas.height = canvas_size.height;


var canvas_center = {
    x: canvas_size.width / 2 / 100 / zoom_mult,
    y: canvas_size.height / 2 / 100 / zoom_mult
}


// var canvas_drift = {
//     x: 0,
//     y: 0
// }

function update_canvas() {
    canvas_size = {
        width: window.innerWidth,
        height: window.innerHeight
    }

    ctx.canvas.width = canvas_size.width;
    ctx.canvas.height = canvas_size.height;

    canvas_center = {
        x: canvas_size.width / 2 / 100 / zoom_mult,
        y: canvas_size.height / 2 / 100 / zoom_mult
    }

    ctx.clearRect(0, 0, canvas_size.width, canvas_size.height);
    ctx.scale(zoom_mult * 100, -zoom_mult * 100);
    ctx.translate(canvas_center.x, -canvas_center.y);
}


function convert_list_to_field(field_list) {
    return {
        width: field_list[0],
        length: field_list[1],
        goal_depth: field_list[2],
        goal_width: field_list[3],
        penalty_width: field_list[4],
        penalty_depth: field_list[5],
        boundary_width: field_list[6]
    }
}

function draw_field(field) {
    ctx.strokeRect(-field.length / 2, -field.width / 2, field.length, field.width)
}

function draw_line_vertical(field) {
    ctx.beginPath();
    ctx.moveTo(0, field.width / 2);
    ctx.lineTo(0, - field.width / 2);
    ctx.stroke();
    ctx.closePath();
}

function draw_penalty(field) {
    ctx.strokeRect((field.length / 2) - field.penalty_depth, -field.penalty_width / 2, field.penalty_depth, field.penalty_width);

    // Right
    ctx.strokeRect(- (field.length / 2), - field.penalty_width / 2, field.penalty_depth, field.penalty_width);

}

function draw_goal(field) {
    ctx.strokeRect(field.length / 2, -field.goal_width / 2, field.goal_depth, field.goal_width);
    ctx.strokeRect(-(field.length / 2) - field.goal_depth, - field.goal_width / 2, field.goal_depth, field.goal_width);
}

function draw_ball(ball) {
    ctx.beginPath();
    ctx.strokeStyle = 'orange';
    ctx.fillStyle = 'orange';
    ctx.arc(ball.x, ball.y, 0.02, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.fill();
    ctx.closePath();
}


function draw_shape(robot) {
    ctx.beginPath();
    ctx.arc(
        robot.x,
        -robot.y,
        0.085,
        -robot.orientation + 0.75,
        -robot.orientation + Math.PI * 2 - 0.75
    );
    ctx.fill();
    ctx.closePath();
}

function draw_text(robot) {
    ctx.save();
    ctx.translate(
        robot.x,
        -robot.y
    );

    ctx.fillStyle = 'black';
    ctx.font = 'normal normal 0.009rem arial';
    ctx.scale(1, -1);
    ctx.fillText(robot.id, -0.085 / 2, 0.085 / 2);

    ctx.restore();

}

function draw_robots(robots, yellow) {
    ctx.fillStyle = yellow ? "#dbd81d" : "#249ed6"
    robots.allies.forEach((robot) => {
        draw_shape(robot);
        draw_text(robot);
    })

    ctx.fillStyle = yellow ? "#249ed6" : "#dbd81d"
    robots.opponents.forEach((robot) => {
        draw_shape(robot);
        draw_text(robot);
    })
}

function add_annotation_point(point) {
    let x = point.x
    let y = point.y
    let box_point = 0.05
    
    ctx.strokeStyle = point.color;
    ctx.fillStyle = point.color;
    ctx.lineWidth = 0.02;

    ctx.beginPath();
    ctx.moveTo(x - box_point, y - box_point);
    ctx.lineTo(x + box_point, y + box_point);
    ctx.moveTo(x - box_point, y + box_point);
    ctx.lineTo(x + box_point, y - box_point);
    ctx.stroke();
}

function add_annotations(type, params) {
    switch (type) {
        case 'point':
            point = JSON.parse(params);
            add_annotation_point(point);
            break;
        default:
            console.warn('Error')
    }
}


window.addEventListener("load", () => {
    var backend;

    new QWebChannel(qt.webChannelTransport, (channel) => {
        backend = channel.objects.backend

        setInterval(async () => {
            field = await backend.get_field();
            ball = await backend.get_ball();
            yellow = await backend.is_yellow();
            robots = await backend.get_robots();
            annotations = await backend.get_annotations();

            update_canvas();

            ctx.strokeStyle = "#fff";
            ctx.lineWidth = field.boundary_width / 10;

            draw_field(field);
            draw_line_vertical(field);
            draw_penalty(field);
            draw_goal(field);

            draw_ball(ball);

            draw_robots(robots, yellow);

            // console.warn(annotations);
            for (id in annotations) {
                add_annotations(annotations[id].type, annotations[id].params)
            }
        }, 60)
    })
})

