let map;
let botMarker;

async function fetchLocation() {
    const response = await fetch("php/get_location.php");
    const data = await response.json();

    if (data.error) {
        console.error(data.error);
        return null;
    }

    return data;
}

async function initMap() {
    const location = await fetchLocation();
    if (!location) return;

    const lat = parseFloat(location.lat);
    const long = parseFloat(location.longi);

    map = L.map('map').setView([lat, long], 15);

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19,
    }).addTo(map);

    const botIcon = L.icon({
        iconUrl: 'assets/vehicle.png',
        iconSize: [50, 40]
    });

    botMarker = L.marker([lat, long], { icon: botIcon }).addTo(map);

    map.on('click', function(e) {
        L.Routing.control({
            waypoints: [
                L.latLng(lat, long),
                L.latLng(e.latlng.lat, e.latlng.lng)
            ],
            draggableWaypoints: false
        }).addTo(map);
    });
}

initMap();