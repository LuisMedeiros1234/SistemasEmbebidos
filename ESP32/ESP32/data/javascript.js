// javascript.js
var canvas = document.getElementById('plotCanvas');
var ctx = canvas.getContext('2d');

var xData = [];
var yData = [];

function updatePlot(x, y, theta, angularVelocity) {
  x = x * 25;
  y = y * 25;
  xData.push(x);
  yData.push(y);

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // Calculate center of the plot
  var centerX = canvas.width / 2;
  var centerY = canvas.height / 2;

  // Plot border
  ctx.strokeStyle = '#000';
  ctx.strokeRect(0, 0, canvas.width, canvas.height);

  // Plot axes
  ctx.beginPath();
  ctx.moveTo(0, centerY);
  ctx.lineTo(canvas.width, centerY);
  ctx.moveTo(centerX, 0);
  ctx.lineTo(centerX, canvas.height);
  ctx.stroke();

  // Plot line segments
  ctx.beginPath();
  ctx.moveTo(centerX + xData[0], centerY - yData[0]);
  for (var i = 1; i < xData.length; i++) {
    ctx.lineTo(centerX + xData[i], centerY - yData[i]);
  }
  ctx.strokeStyle = 'lightgray';
  ctx.stroke();

  // Plot blue dots at past positions
  ctx.fillStyle = 'blue';
  for (var i = 0; i < xData.length - 1; i++) {
    ctx.beginPath();
    ctx.arc(centerX + xData[i], centerY - yData[i], 2, 0, Math.PI * 2);
    ctx.fill();
  }

  // Plot black dot at current position
  ctx.fillStyle = 'black';
  ctx.beginPath();
  ctx.arc(centerX + xData[xData.length - 1], centerY - yData[yData.length - 1], 4, 0, Math.PI * 2);
  ctx.fill();

  // Plot green dot at (0,0)
  ctx.fillStyle = 'green';
  ctx.beginPath();
  ctx.arc(centerX, centerY, 5, 0, Math.PI * 2);
  ctx.fill();

  // Update values
  document.getElementById('xValue').innerText = (x/25).toFixed(2);
  document.getElementById('yValue').innerText = (y/25).toFixed(2);
  document.getElementById('thetaValue').innerText = theta.toFixed(2);
  document.getElementById('angularVelocityValue').innerText = angularVelocity.toFixed(2);
}

function fetchData() {
  fetch('/data')
    .then(response => response.json())
    .then(data => {
      updatePlot(data.x, data.y, data.theta, data.angularVelocity);
      setTimeout(fetchData, 500); // Fetch data every 0.5 seconds
    });
}

fetchData(); // Start fetching data immediately

var state = 0;

function toggleState() {
    var button = document.getElementById('controlButton');
    if (state === 0) {
        state = 1;
        button.innerText = 'Stop';
        sendState(1); // Send state 1 (Stop)
    } else {
        state = 0;
        button.innerText = 'Start';
        sendState(0); // Send state 0 (Start)
    }
}

function setSpeed(speedValue) {
  sendSpeed(speedValue);
}

function promptCoordinates() {
  var Px = prompt("Enter Px:");
  var Py = prompt("Enter Py:");
  var Pt = prompt("Enter Pt:");
  if (Px !== null && Py !== null) {
    sendCoordinates(Px, Py, Pt);
  }
}

function sendState(stateValue) {
    fetch('/control?state=' + stateValue) // Send state to server
        .then(response => {
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }
            return response.json();
        })
        .then(data => {
            console.log('State change successful');
        })
        .catch(error => {
            console.error('There was a problem with the state change:', error);
        });
}

function sendSpeed(speedValue) {
  fetch('/speed?speed=' + speedValue)
    .then(response => {
      if (!response.ok) {
        throw new Error('Network response was not ok');
      }
      return response.json();
    })
    .then(data => {
      console.log('Speed set successfully');
    })
    .catch(error => {
      console.error('There was a problem setting speed:', error);
    });
}

function sendCoordinates(Px, Py, Pt) {
  fetch('/coordinates?Px=' + Px + '&Py=' + Py + '&Pt=' + Pt)
    .then(response => {
      if (!response.ok) {
        throw new Error('Network response was not ok');
      }
      return response.json();
    })
    .then(data => {
      console.log('Coordinates set successfully');
    })
    .catch(error => {
      console.error('There was a problem setting coordinates:', error);
    });
}
