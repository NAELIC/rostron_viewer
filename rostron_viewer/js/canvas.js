const canvas = document.querySelector("#field")
const ctx = document.querySelector("#field").getContext("2d");

// Canvas variable
var zoom_mult = 1;

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


    ctx.scale(zoom_mult / 100, -zoom_mult / 100);
    ctx.clearRect(0, 0, canvas_size.width, canvas_size.height);
    ctx.translate(canvas_center.x, -canvas_center.y);
}


function convert_list_to_field(field_list) {
    return {
        width : field_list[0], 
        length : field_list[1], 
        goal_depth : field_list[2], 
        goal_width : field_list[3], 
        penalty_width : field_list[4],  
        penalty_depth: field_list[5],
        boundary_width : field_list[6]
    }
}

function draw_field(field) {
    ctx.strokeRect(-field.length / 2, field.width / 2, field.length, field.width)
}

window.addEventListener("load", () => {
    const backend;

    new QWebChannel(qt.webChannelTransport, (channel) => {
        backend = channel.objects.backend

        setInterval(() => {
            backend.get_field().then((field_list) => {
                const field = convert_list_to_field(field_list);
                
                update_canvas();
                ctx.strokeStyle = "#FFF";
                ctx.lineWidth = field.boundary_width;
                
                draw_field(field);


            });
        }, 60)
    })
})



// ctx.fillStyle = "blue";
// ctx.fillRect(0, 0, canvas.width, canvas.height);