let map;
let waypoints = [];
let markers = [];
let polyline = null;

function initMap() {
    map = new google.maps.Map(document.getElementById("map"), {
        center: { lat: 16.0659092, lng: 108.1609844 },
        zoom: 16,
    });

    map.addListener("click", (event) => {
        addWaypoint(event.latLng);
    });
}
function addWaypoint(location) {
    let idx = markers.length;

    let marker = new google.maps.Marker({
        position: location,
        map: map,
        label: `${idx + 1}`,
        draggable: true   // 🔥 Cho phép kéo marker
    });

    markers.push(marker);
    waypoints.push({
        lat: location.lat(),
        lng: location.lng(),
        speed: 1.0,
        hold_time: 2
    });

    // Khi kéo xong thì cập nhật lại waypoint
    marker.addListener("dragend", function(event) {
        let newPos = event.latLng;
        waypoints[idx].lat = newPos.lat();
        waypoints[idx].lng = newPos.lng();
        drawPolyline(); // vẽ lại polyline
    });

    drawPolyline();
}

function drawPolyline() {
    if (polyline) {
        polyline.setMap(null); // xóa đường cũ
    }

    let path = waypoints.map(wp => ({ lat: wp.lat, lng: wp.lng }));

    polyline = new google.maps.Polyline({
        path: path,
        geodesic: true,
        strokeColor: "#FF0000",
        strokeOpacity: 1.0,
        strokeWeight: 2,
    });

    polyline.setMap(map);
}

function clearWaypoints() {
    markers.forEach(m => m.setMap(null));
    markers = [];
    waypoints = [];
    if (polyline) {
        polyline.setMap(null);
    }
}

function exportMission() {
    console.log(JSON.stringify({ mission: waypoints }, null, 2));
}

function updateProgress(percent, success=false) {
  const bar = document.getElementById("progress-bar");
  bar.style.width = percent + "%";
  if (success) {
    bar.style.background = "green"; // ✅ thành công thì đổi sang xanh
  } else {
    bar.style.background = "red";   // ❌ mặc định hoặc thất bại là đỏ
  }
}

async function uploadMission() {
  const mission = waypoints.map(wp => ({
    lat: wp.lat,
    lng: wp.lng,
    speed: 1.0,
    hold_time: 2
  }));

  updateProgress(10); // bắt đầu
  try {
    const res = await fetch("/upload-mission", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mission })
    });
    const data = await res.json();

    if (data.success) {
      updateProgress(100, true); // 🎉 full xanh khi thành công
    } else {
      updateProgress(100, false); // full đỏ nếu thất bại
      alert("❌ Failed: " + data.message);
    }
  } catch (err) {
    updateProgress(100, false);
    alert("⚠️ Error: " + err);
  }
}


// =============================
// Gọi API điều khiển vehicle
// =============================
function armVehicle() {
  fetch("/arm", { method: "POST" })
    .then(res => res.json())
    .then(data => alert("🚀 ARM: " + JSON.stringify(data)))
    .catch(err => alert("❌ ARM error: " + err));
}

function disarmVehicle() {
  fetch("/disarm", { method: "POST" })
    .then(res => res.json())
    .then(data => alert("🛑 DISARM: " + JSON.stringify(data)))
    .catch(err => alert("❌ DISARM error: " + err));
}

function startMission() {
  fetch("/start-mission", { method: "POST" })
    .then(res => res.json())
    .then(data => alert("▶️ Start Mission: " + JSON.stringify(data)))
    .catch(err => alert("❌ Start error: " + err));
}

vehicleMarker = null;

let boatIcon = {
  url: "/static/boat.png",   // icon thuyền bạn để trong thư mục static
  scaledSize: new google.maps.Size(30, 30), // kích thước
  anchor: new google.maps.Point(20, 20)     // tâm icon nằm ở giữa
};

function updateVehiclePosition() {
  fetch("/vehicle-position")
    .then(res => res.json())
    .then(data => {
      if (data.success) {
        const pos = { lat: data.lat, lng: data.lon };

        if (!vehicleMarker) {
          vehicleMarker = new google.maps.Marker({
            position: pos,
            map: map,
            icon: boatIcon,
            title: "USV Vehicle"
          });
        } else {
          vehicleMarker.setPosition(pos);
        }
      }
    })
    .catch(err => console.error("Position update failed:", err));
}

// gọi update liên tục mỗi 2 giây
setInterval(updateVehiclePosition, 2000);

// Khởi tạo map khi load
window.onload = initMap;


