<?php
header('Content-Type: application/json');

$server = "localhost";
$username = "root";
$password = "";
$database = "botmaps";

$conn = mysqli_connect($server, $username, $password, $database);

if (!$conn) {
    die(json_encode(["error" => "Database connection failed"]));
}

$sql = "SELECT * FROM geolocations ORDER BY id DESC LIMIT 1";
$result = mysqli_query($conn, $sql);

if ($row = mysqli_fetch_assoc($result)) {
    echo json_encode([
        "lat" => $row['lat'],
        "longi" => $row['longi']
    ]);
} else {
    echo json_encode(["error" => "No location found"]);
}
?>