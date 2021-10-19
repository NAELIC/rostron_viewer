window.addEventListener("load", () => {
    var backend;

    new QWebChannel(qt.webChannelTransport, (channel) => {
        backend = channel.objects.backend

        setInterval(() => {
            backend.get_field().then((field) => {
                console.warn(field)
            });
        }, 60)
    })
})

let canvas = document.querySelector("#field")

let ctx = canvas.getContext("2d");
ctx.canvas.width = window.innerWidth;
ctx.canvas.height = window.innerHeight;

ctx.fillStyle = "blue";
ctx.fillRect(0, 0, canvas.width, canvas.height);