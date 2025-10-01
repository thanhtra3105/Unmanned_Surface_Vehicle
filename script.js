let map;
let waypoints = [];
let markers = [];
let polyline;

function initMap() {
  map = new google.maps.Map(document.getElementById("map"), {
    center: { lat: 16.0686935, lng: 108.1567368 }, // 16.0686935,108.1567368
    zoom: 15,
  });

  map.addListener("click", (e) => {
    addWaypoint(e.latLng);
  });
}

function addWaypoint(location) {
    const marker = new google.maps.Marker({
    position: location,
    map: map,
    label: (waypoints.length + 1).toString(),
    draggable: true
    });

    marker.addListener("dragend", () => {
    updateWaypoints();
    });

    markers.push(marker);
    waypoints.push({ lat: location.lat(), lng: location.lng(), speed: 1.0, hold_time: 2 });
    drawPolyline();
}


function updateWaypoints() {
  waypoints = markers.map((m, idx) => ({
    lat: m.getPosition().lat(),
    lng: m.getPosition().lng(),
    speed: 1.0,
    hold_time: 2
  }));
  drawPolyline();
}

function drawPolyline() {
  if (polyline) polyline.setMap(null);
  const path = markers.map((m) => m.getPosition());
  polyline = new google.maps.Polyline({
    path: path,
    map: map,
    strokeColor: "#FF0000",
    strokeOpacity: 1.0,
    strokeWeight: 2,
  });
}

function exportMission() {
  const json = JSON.stringify({ mission: waypoints }, null, 2);
  const blob = new Blob([json], { type: "application/json" });
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = "mission.json";
  link.click();
}

function clearMission() {
  markers.forEach(m => m.setMap(null));
  markers = [];
  waypoints = [];
  if (polyline) polyline.setMap(null);
}

function importMission() {
  document.getElementById("fileInput").click();
}

document.getElementById("fileInput")?.addEventListener("change", function(e) {
  const file = e.target.files[0];
  if (!file) return;

  const reader = new FileReader();
  reader.onload = function(event) {
    const data = JSON.parse(event.target.result);
    loadMission(data.mission);
  };
  reader.readAsText(file);
});

function loadMission(mission) {
  clearMission();
  mission.forEach((wp) => {
    const location = new google.maps.LatLng(wp.lat, wp.lng);
    addWaypoint(location);
  });
}

window.onload = initMap;
