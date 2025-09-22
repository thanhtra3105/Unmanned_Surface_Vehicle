let map = L.map('map').setView([-35.363, 149.165], 15);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 19,
}).addTo(map);

let waypoints = [];
map.on('click', function(e) {
  let marker = L.marker(e.latlng).addTo(map);
  waypoints.push({
    lat: e.latlng.lat,
    lng: e.latlng.lng,
    speed: 1.0,
    hold_time: 2
  });
  document.getElementById("log").textContent = JSON.stringify(waypoints, null, 2);
});

function uploadMission() {
  fetch('/upload-mission', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ mission: waypoints })
  })
  .then(res => res.json())
  .then(data => {
    document.getElementById("log").textContent = JSON.stringify(data, null, 2);
  })
  .catch(err => alert("Error: " + err));
}
