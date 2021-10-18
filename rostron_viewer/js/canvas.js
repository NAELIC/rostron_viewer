let canvas = document.querySelector("#field")

let ctx = canvas.getContext("2d");
ctx.canvas.width  = window.innerWidth;
ctx.canvas.height = window.innerHeight;

ctx.fillStyle = "blue";
ctx.fillRect(0, 0, canvas.width, canvas.height);