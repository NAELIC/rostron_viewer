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
    ctx.scale(zoom_mult * 100, zoom_mult * 100);
    ctx.translate(canvas_center.x, canvas_center.y);
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
    // ctx.strokeRect(0, 0, 150,150);
}

window.addEventListener("load", () => {
    var backend;

    new QWebChannel(qt.webChannelTransport, (channel) => {
        backend = channel.objects.backend

        setInterval(() => {
            backend.get_field().then((field_list) => {
                const field = convert_list_to_field(field_list);

                update_canvas();

                // ctx.fillStyle = "#26a349";
                // ctx.fillRect(-canvas.width, -canvas.height, canvas.width, canvas.height);

                ctx.strokeStyle = "#fff";
                ctx.lineWidth = field.boundary_width / 10;
                console.warn(field.length);

                draw_field(field);
            });
        }, 60)
    })
})


