#include <arpa/inet.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <mutex>
#include <netinet/in.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "zadanie2_interfaces/msg/game_state.hpp"

struct WebRobotState // stav robota pre webovu vizualizaciu
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    bool has_odom = false;
};

struct WebLidarState // stav lidaru pre webovu vizualizaciu
{
    std::vector<float> ranges;
    double angle_min = 0.0;
    double angle_increment = 0.0;
    double range_min = 0.0;
    double range_max = 0.0;
    bool has_scan = false;
};

class WebNode : public rclcpp::Node
{
public:
    WebNode()
        : Node("web_node") // vytvorenie ROS node pre webovu aplikaciu
    {
        this->declare_parameter<int>("port"); // port web servera, napr 8080
        this->declare_parameter<double>("linear_speed"); // linearna rychlost pre ovladanie cez web
        this->declare_parameter<double>("angular_speed"); // uhlova rychlost pre ovladanie cez web
        this->declare_parameter<double>("map_width"); // sirka mapy v metroch
        this->declare_parameter<double>("map_height"); // vyska mapy v metroch
        this->declare_parameter<std::string>("map_path"); // cesta k png mape

        port_ = getRequiredParameter<int>("port");
        linear_speed_ = getRequiredParameter<double>("linear_speed");
        angular_speed_ = getRequiredParameter<double>("angular_speed");
        map_width_ = getRequiredParameter<double>("map_width");
        map_height_ = getRequiredParameter<double>("map_height");
        map_path_ = getRequiredParameter<std::string>("map_path");

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/player1/cmd_vel", // web ovlada player1 cez cmd_vel
            10
        );

        player1_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/player1/odom", // citame polohu player1
            10,
            std::bind(&WebNode::player1OdomCallback, this, std::placeholders::_1)
        );

        player2_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/player2/odom", // citame polohu player2/bota
            10,
            std::bind(&WebNode::player2OdomCallback, this, std::placeholders::_1)
        );

        player1_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/player1/scan", // citame lidar player1
            10,
            std::bind(&WebNode::player1ScanCallback, this, std::placeholders::_1)
        );

        player2_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/player2/scan", // citame lidar player2
            10,
            std::bind(&WebNode::player2ScanCallback, this, std::placeholders::_1)
        );

        game_state_sub_ = this->create_subscription<zadanie2_interfaces::msg::GameState>(
            "/game_state", // citame stav hry, skore, odpadky, stanicu, prekazky, drahy
            10,
            std::bind(&WebNode::gameStateCallback, this, std::placeholders::_1)
        );

        running_ = true;
        server_thread_ = std::thread(&WebNode::serverLoop, this); // web server bezi vo vlastnom vlakne

        RCLCPP_INFO(
            this->get_logger(),
            "web_node started on http://localhost:%d",
            port_
        );
    }

    ~WebNode()
    {
        running_ = false;

        if (server_fd_ >= 0) {
            shutdown(server_fd_, SHUT_RDWR); // prerusi blokujuci accept
            close(server_fd_);
            server_fd_ = -1;
        }

        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }

private:
    template<typename T>
    T getRequiredParameter(const std::string& name) // povinne citanie parametrov z yaml
    {
        T value;

        if (!this->get_parameter(name, value)) {
            throw std::runtime_error("Missing required ROS parameter: " + name);
        }

        return value;
    }

    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) const // prepocet quaternionu na theta/yaw
    {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    void player1OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) // callback pre /player1/odom
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        player1_.x = msg->pose.pose.position.x;
        player1_.y = msg->pose.pose.position.y;
        player1_.theta = quaternionToYaw(msg->pose.pose.orientation);
        player1_.has_odom = true;
    }

    void player2OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) // callback pre /player2/odom
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        player2_.x = msg->pose.pose.position.x;
        player2_.y = msg->pose.pose.position.y;
        player2_.theta = quaternionToYaw(msg->pose.pose.orientation);
        player2_.has_odom = true;
    }

    void updateLidarState(
        WebLidarState& state,
        const sensor_msgs::msg::LaserScan::SharedPtr msg) // ulozenie lidar dat
    {
        state.ranges = msg->ranges;
        state.angle_min = msg->angle_min;
        state.angle_increment = msg->angle_increment;
        state.range_min = msg->range_min;
        state.range_max = msg->range_max;
        state.has_scan = true;
    }

    void player1ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) // callback pre /player1/scan
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        updateLidarState(player1_lidar_, msg);
    }

    void player2ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) // callback pre /player2/scan
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        updateLidarState(player2_lidar_, msg);
    }

    void gameStateCallback(const zadanie2_interfaces::msg::GameState::SharedPtr msg) // callback pre /game_state
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        last_game_state_ = *msg;
        has_game_state_ = true;
    }

    std::string escapeJsonString(const std::string& text) const // uprava stringu aby bol validny JSON
    {
        std::ostringstream out;

        for (char c : text) {
            if (c == '"') {
                out << "\\\"";
            } else if (c == '\\') {
                out << "\\\\";
            } else if (c == '\n') {
                out << "\\n";
            } else {
                out << c;
            }
        }

        return out.str();
    }

    void appendDoubleArray(
        std::ostringstream& out,
        const std::vector<double>& values) const // zapis pola double do JSON
    {
        out << "[";

        for (size_t i = 0; i < values.size(); ++i) {
            if (i > 0) {
                out << ",";
            }

            out << values[i];
        }

        out << "]";
    }

    void appendLidarPointsJson(
        std::ostringstream& out,
        const WebRobotState& robot,
        const WebLidarState& lidar) const // prepocet lidar ranges na body vo svete a zapis do JSON
    {
        out << "[";

        if (!robot.has_odom || !lidar.has_scan) {
            out << "]";
            return;
        }

        bool first = true;

        for (size_t i = 0; i < lidar.ranges.size(); ++i) {
            double range = static_cast<double>(lidar.ranges[i]);

            if (!std::isfinite(range)) {
                continue;
            }

            if (range < lidar.range_min || range > lidar.range_max) {
                continue;
            }

            double local_angle = lidar.angle_min + static_cast<double>(i) * lidar.angle_increment;
            double world_angle = robot.theta + local_angle;

            double x = robot.x + range * std::cos(world_angle);
            double y = robot.y + range * std::sin(world_angle);

            if (!first) {
                out << ",";
            }

            out << "{";
            out << "\"x\":" << x << ",";
            out << "\"y\":" << y;
            out << "}";

            first = false;
        }

        out << "]";
    }

    std::string makeStateJson() // vytvori JSON so stavom hry pre web
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        std::ostringstream out;

        if (!has_game_state_) { // ak este neprisiel game_state, vratime prazdny stav
            out << "{";
            out << "\"status\":\"WAITING\",";
            out << "\"remaining_trash\":0,";
            out << "\"player1_score\":0,";
            out << "\"player2_score\":0,";
            out << "\"player1_capacity\":0,";
            out << "\"player2_capacity\":0,";
            out << "\"player1\":{\"x\":" << player1_.x
                << ",\"y\":" << player1_.y
                << ",\"theta\":" << player1_.theta << "},";
            out << "\"player2\":{\"x\":" << player2_.x
                << ",\"y\":" << player2_.y
                << ",\"theta\":" << player2_.theta << "},";
            out << "\"trash\":[],";
            out << "\"station\":{\"x\":0,\"y\":0,\"radius\":0},";
            out << "\"circle_obstacles\":[],";
            out << "\"rectangle_obstacles\":[],";
            out << "\"player1_lidar_points\":[],";
            out << "\"player2_lidar_points\":[],";
            out << "\"player1_path_x\":[],";
            out << "\"player1_path_y\":[],";
            out << "\"player2_path_x\":[],";
            out << "\"player2_path_y\":[],";
            out << "\"map_width\":" << map_width_ << ",";
            out << "\"map_height\":" << map_height_;
            out << "}";

            return out.str();
        }

        out << "{";

        out << "\"status\":\"" << escapeJsonString(last_game_state_.status) << "\",";
        out << "\"remaining_trash\":" << last_game_state_.remaining_trash << ",";
        out << "\"player1_score\":" << last_game_state_.player1_score << ",";
        out << "\"player2_score\":" << last_game_state_.player2_score << ",";
        out << "\"player1_capacity\":" << last_game_state_.player1_capacity << ",";
        out << "\"player2_capacity\":" << last_game_state_.player2_capacity << ",";

        out << "\"player1\":{\"x\":" << player1_.x
            << ",\"y\":" << player1_.y
            << ",\"theta\":" << player1_.theta << "},";

        out << "\"player2\":{\"x\":" << player2_.x
            << ",\"y\":" << player2_.y
            << ",\"theta\":" << player2_.theta << "},";

        out << "\"trash\":[";

        size_t trash_count = std::min({
            last_game_state_.trash_id.size(),
            last_game_state_.trash_type.size(),
            last_game_state_.trash_x.size(),
            last_game_state_.trash_y.size(),
            last_game_state_.trash_radius.size(),
            last_game_state_.trash_collected.size()
        });

        for (size_t i = 0; i < trash_count; ++i) {
            if (i > 0) {
                out << ",";
            }

            out << "{";
            out << "\"id\":" << last_game_state_.trash_id[i] << ",";
            out << "\"type\":\"" << escapeJsonString(last_game_state_.trash_type[i]) << "\",";
            out << "\"x\":" << last_game_state_.trash_x[i] << ",";
            out << "\"y\":" << last_game_state_.trash_y[i] << ",";
            out << "\"radius\":" << last_game_state_.trash_radius[i] << ",";
            out << "\"collected\":" << (last_game_state_.trash_collected[i] ? "true" : "false");
            out << "}";
        }

        out << "],";

        out << "\"station\":{";
        out << "\"x\":" << last_game_state_.station_x << ",";
        out << "\"y\":" << last_game_state_.station_y << ",";
        out << "\"radius\":" << last_game_state_.station_radius;
        out << "},";

        out << "\"circle_obstacles\":[";

        size_t circle_count = std::min({
            last_game_state_.circle_obstacles_x.size(),
            last_game_state_.circle_obstacles_y.size(),
            last_game_state_.circle_obstacles_radius.size()
        });

        for (size_t i = 0; i < circle_count; ++i) {
            if (i > 0) {
                out << ",";
            }

            out << "{";
            out << "\"x\":" << last_game_state_.circle_obstacles_x[i] << ",";
            out << "\"y\":" << last_game_state_.circle_obstacles_y[i] << ",";
            out << "\"radius\":" << last_game_state_.circle_obstacles_radius[i];
            out << "}";
        }

        out << "],";

        out << "\"rectangle_obstacles\":[";

        size_t rectangle_count = std::min({
            last_game_state_.rectangle_obstacles_x.size(),
            last_game_state_.rectangle_obstacles_y.size(),
            last_game_state_.rectangle_obstacles_width.size(),
            last_game_state_.rectangle_obstacles_height.size()
        });

        for (size_t i = 0; i < rectangle_count; ++i) {
            if (i > 0) {
                out << ",";
            }

            out << "{";
            out << "\"x\":" << last_game_state_.rectangle_obstacles_x[i] << ",";
            out << "\"y\":" << last_game_state_.rectangle_obstacles_y[i] << ",";
            out << "\"width\":" << last_game_state_.rectangle_obstacles_width[i] << ",";
            out << "\"height\":" << last_game_state_.rectangle_obstacles_height[i];
            out << "}";
        }

        out << "],";

        out << "\"player1_lidar_points\":";
        appendLidarPointsJson(out, player1_, player1_lidar_);
        out << ",";

        out << "\"player2_lidar_points\":";
        appendLidarPointsJson(out, player2_, player2_lidar_);
        out << ",";

        out << "\"player1_path_x\":";
        appendDoubleArray(out, last_game_state_.player1_path_x);
        out << ",";

        out << "\"player1_path_y\":";
        appendDoubleArray(out, last_game_state_.player1_path_y);
        out << ",";

        out << "\"player2_path_x\":";
        appendDoubleArray(out, last_game_state_.player2_path_x);
        out << ",";

        out << "\"player2_path_y\":";
        appendDoubleArray(out, last_game_state_.player2_path_y);
        out << ",";

        out << "\"map_width\":" << map_width_ << ",";
        out << "\"map_height\":" << map_height_;

        out << "}";

        return out.str();
    }

    void publishCommand(const std::string& key) // prevedie klavesu z webu na Twist prikaz
    {
        geometry_msgs::msg::Twist cmd;

        if (key == "w") {
            cmd.linear.x = linear_speed_;
            cmd.angular.z = 0.0;
        } else if (key == "s") {
            cmd.linear.x = -linear_speed_;
            cmd.angular.z = 0.0;
        } else if (key == "d") {
            cmd.linear.x = 0.0;
            cmd.angular.z = -angular_speed_; // rovnako ako keyboard teleop
        } else if (key == "a") {
            cmd.linear.x = 0.0;
            cmd.angular.z = angular_speed_; // rovnako ako keyboard teleop
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }

        cmd_pub_->publish(cmd);
    }

    std::string readBinaryFile(const std::string& path) const // nacita png mapu ako binarny subor
    {
        std::ifstream file(path, std::ios::binary);

        if (!file.is_open()) {
            return "";
        }

        std::ostringstream buffer;
        buffer << file.rdbuf();

        return buffer.str();
    }

    std::string getHtmlPage() const // HTML + JavaScript stranka pre web ovladanie
    {
        return R"rawliteral(
<!DOCTYPE html>
<html lang="sk">
<head>
    <meta charset="UTF-8">
    <title>OPK Robot Game</title>
    <style>
        body {
            margin: 0;
            background: #202124;
            color: white;
            font-family: Arial, sans-serif;
            text-align: center;
        }

        h1 {
            margin: 14px;
        }

        #info {
            width: 1000px;
            margin: 0 auto 10px auto;
            padding: 10px;
            background: #303134;
            border-radius: 10px;
            display: flex;
            justify-content: space-around;
            gap: 10px;
            flex-wrap: wrap;
        }

        .box {
            min-width: 180px;
            text-align: left;
        }

        .label {
            color: #aaa;
        }

        canvas {
            background: #111;
            border: 2px solid #555;
            border-radius: 8px;
        }

        #hint {
            margin: 10px;
            color: #ccc;
        }
    </style>
</head>
<body>
    <h1>OPK Robot Game - Web Control</h1>

    <div id="info">
        <div class="box">
            <b>Game</b>
            <div><span class="label">Status:</span> <span id="status">---</span></div>
            <div><span class="label">Remaining:</span> <span id="remaining">---</span></div>
        </div>

        <div class="box">
            <b>Player 1</b>
            <div><span class="label">Score:</span> <span id="p1score">0</span></div>
            <div><span class="label">Capacity:</span> <span id="p1cap">0</span></div>
        </div>

        <div class="box">
            <b>Player 2 / Bot</b>
            <div><span class="label">Score:</span> <span id="p2score">0</span></div>
            <div><span class="label">Capacity:</span> <span id="p2cap">0</span></div>
        </div>

        <div class="box">
            <b>Controls</b>
            <div>W/S - forward/backward</div>
            <div>A - turn right</div>
            <div>D - turn left</div>
            <div>Release key - stop</div>
        </div>
    </div>

    <canvas id="canvas" width="1000" height="650"></canvas>
    <div id="hint">Click the page once, then use WASD.</div>

<script>
const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");

let state = null;
let pressed = {};
let activeKey = null;

const mapImage = new Image();
let mapLoaded = false;

mapImage.onload = () => {
    mapLoaded = true;
};

mapImage.src = "/map_image";

function worldToCanvas(x, y) {
    const margin = 40;
    const width = canvas.width - 2 * margin;
    const height = canvas.height - 2 * margin;

    const mapWidth = state ? state.map_width : 30.72;
    const mapHeight = state ? state.map_height : 20.48;

    return {
        x: margin + width - (x / mapWidth) * width,
        y: margin + (y / mapHeight) * height
    };
}

function drawCircleWorld(x, y, radius, color) {
    if (!state) {
        return;
    }

    const p = worldToCanvas(x, y);
    const scale = (canvas.width - 80) / state.map_width;

    ctx.beginPath();
    ctx.arc(p.x, p.y, radius * scale, 0, 2 * Math.PI);
    ctx.fillStyle = color;
    ctx.fill();
}

function drawRectWorld(x, y, width, height, color) {
    if (!state) {
        return;
    }

    const center = worldToCanvas(x, y);
    const scaleX = (canvas.width - 80) / state.map_width;
    const scaleY = (canvas.height - 80) / state.map_height;

    ctx.fillStyle = color;
    ctx.fillRect(
        center.x - (width * scaleX) / 2,
        center.y - (height * scaleY) / 2,
        width * scaleX,
        height * scaleY
    );
}

function drawPath(pathX, pathY, color) {
    if (!pathX || !pathY || pathX.length < 2) {
        return;
    }

    const count = Math.min(pathX.length, pathY.length);
    const first = worldToCanvas(pathX[0], pathY[0]);

    ctx.beginPath();
    ctx.moveTo(first.x, first.y);

    for (let i = 1; i < count; i++) {
        const p = worldToCanvas(pathX[i], pathY[i]);
        ctx.lineTo(p.x, p.y);
    }

    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.stroke();
}

function drawRobot(robot, color, label) {
    const p = worldToCanvas(robot.x, robot.y);

    ctx.save();
    ctx.translate(p.x, p.y);

    // Mapa vo webe je zrkadlena podla osi X, preto sa uhol meni na pi - theta.
    ctx.rotate(Math.PI - robot.theta);

    ctx.beginPath();
    ctx.arc(0, 0, 12, 0, 2 * Math.PI);
    ctx.fillStyle = color;
    ctx.fill();

    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(24, 0);
    ctx.strokeStyle = "white";
    ctx.lineWidth = 3;
    ctx.stroke();

    ctx.restore();

    ctx.fillStyle = "white";
    ctx.font = "14px Arial";
    ctx.fillText(label, p.x + 15, p.y - 12);
}

function drawLidarPoints(points, color) {
    if (!points) {
        return;
    }

    ctx.fillStyle = color;

    for (const point of points) {
        const p = worldToCanvas(point.x, point.y);

        ctx.beginPath();
        ctx.arc(p.x, p.y, 2.5, 0, 2 * Math.PI);
        ctx.fill();
    }
}

function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "#151515";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    const margin = 40;
    const drawWidth = canvas.width - 2 * margin;
    const drawHeight = canvas.height - 2 * margin;

    if (mapLoaded) {
        ctx.save();
        ctx.translate(margin + drawWidth, margin);
        ctx.scale(-1, 1);
        ctx.drawImage(
            mapImage,
            0,
            0,
            drawWidth,
            drawHeight
        );
        ctx.restore();
    } else {
        ctx.fillStyle = "#111";
        ctx.fillRect(margin, margin, drawWidth, drawHeight);
    }

    ctx.strokeStyle = "rgba(80,80,80,0.6)";
    ctx.lineWidth = 1;

    for (let i = 0; i <= 10; i++) {
        let x = margin + i * drawWidth / 10;
        ctx.beginPath();
        ctx.moveTo(x, margin);
        ctx.lineTo(x, canvas.height - margin);
        ctx.stroke();

        let y = margin + i * drawHeight / 10;
        ctx.beginPath();
        ctx.moveTo(margin, y);
        ctx.lineTo(canvas.width - margin, y);
        ctx.stroke();
    }

    if (!state) {
        ctx.fillStyle = "white";
        ctx.font = "24px Arial";
        ctx.fillText("Waiting for ROS data...", 380, 320);
        return;
    }

    drawPath(state.player1_path_x, state.player1_path_y, "#0066ff");
    drawPath(state.player2_path_x, state.player2_path_y, "#bb44ff");

    drawLidarPoints(state.player1_lidar_points, "red");
    drawLidarPoints(state.player2_lidar_points, "cyan");

    drawCircleWorld(state.station.x, state.station.y, state.station.radius, "rgba(0,255,0,0.65)");

    if (state.circle_obstacles) {
        for (const obstacle of state.circle_obstacles) {
            drawCircleWorld(
                obstacle.x,
                obstacle.y,
                obstacle.radius,
                "rgba(255,0,0,0.75)"
            );
        }
    }

    if (state.rectangle_obstacles) {
        for (const obstacle of state.rectangle_obstacles) {
            drawRectWorld(
                obstacle.x,
                obstacle.y,
                obstacle.width,
                obstacle.height,
                "rgba(255,0,0,0.75)"
            );
        }
    }

    for (const trash of state.trash) {
        if (trash.collected) {
            continue;
        }

        let color = "#33ff66";

        if (trash.type === "paper") {
            color = "#ffff33";
        } else if (trash.type === "plastic") {
            color = "#ffaa00";
        } else if (trash.type === "glass") {
            color = "#33ff66";
        }

        drawCircleWorld(trash.x, trash.y, trash.radius, color);
    }

    drawRobot(state.player1, "#0066ff", "P1");
    drawRobot(state.player2, "#bb44ff", "BOT");
}

async function fetchState() {
    try {
        const response = await fetch("/state");
        state = await response.json();

        document.getElementById("status").innerText = state.status;
        document.getElementById("remaining").innerText = state.remaining_trash;
        document.getElementById("p1score").innerText = state.player1_score;
        document.getElementById("p1cap").innerText = state.player1_capacity;
        document.getElementById("p2score").innerText = state.player2_score;
        document.getElementById("p2cap").innerText = state.player2_capacity;

        draw();
    } catch (e) {
        console.log(e);
    }
}

async function sendCommand(key) {
    try {
        await fetch("/cmd?key=" + encodeURIComponent(key));
    } catch (e) {
        console.log(e);
    }
}

document.addEventListener("keydown", (event) => {
    const key = event.key.toLowerCase();

    if (["w", "a", "s", "d"].includes(key)) {
        event.preventDefault();

        activeKey = key;
        pressed[key] = true;
        sendCommand(key);
    }
});

document.addEventListener("keyup", (event) => {
    const key = event.key.toLowerCase();

    if (["w", "a", "s", "d"].includes(key)) {
        event.preventDefault();

        pressed[key] = false;

        if (activeKey === key) {
            activeKey = null;
            sendCommand("stop");
        }
    }
});

setInterval(() => {
    if (activeKey !== null) {
        sendCommand(activeKey);
    }
}, 100);

setInterval(fetchState, 100);
setInterval(draw, 100);
</script>
</body>
</html>
)rawliteral";
    }

    std::string getQueryValue(const std::string& path, const std::string& key) const // vytiahne hodnotu z URL napr /cmd?key=w
    {
        std::string pattern = key + "=";
        size_t pos = path.find(pattern);

        if (pos == std::string::npos) {
            return "";
        }

        pos += pattern.size();
        size_t end = path.find('&', pos);

        if (end == std::string::npos) {
            end = path.size();
        }

        return path.substr(pos, end - pos);
    }

    void sendHttpResponse(
        int client_fd,
        const std::string& body,
        const std::string& content_type,
        int status_code = 200,
        const std::string& status_text = "OK")
    {
        std::ostringstream response;

        response << "HTTP/1.1 " << status_code << " " << status_text << "\r\n";
        response << "Content-Type: " << content_type << "\r\n";
        response << "Content-Length: " << body.size() << "\r\n";
        response << "Connection: close\r\n";
        response << "Access-Control-Allow-Origin: *\r\n";
        response << "\r\n";
        response << body;

        std::string response_text = response.str();
        send(client_fd, response_text.c_str(), response_text.size(), 0);
    }

    void handleClient(int client_fd) // obsluha jedneho HTTP klienta
    {
        char buffer[4096];
        std::memset(buffer, 0, sizeof(buffer));

        int received = recv(client_fd, buffer, sizeof(buffer) - 1, 0);

        if (received <= 0) {
            close(client_fd);
            return;
        }

        std::string request(buffer);
        std::istringstream request_stream(request);

        std::string method;
        std::string path;
        std::string version;

        request_stream >> method >> path >> version;

        if (method != "GET") {
            sendHttpResponse(
                client_fd,
                "Only GET is supported",
                "text/plain",
                405,
                "Method Not Allowed"
            );
            close(client_fd);
            return;
        }

        if (path == "/") {
            sendHttpResponse(
                client_fd,
                getHtmlPage(),
                "text/html; charset=utf-8"
            );
        } else if (path == "/map_image") {
            std::string image_data = readBinaryFile(map_path_);

            if (image_data.empty()) {
                sendHttpResponse(
                    client_fd,
                    "Map image not found",
                    "text/plain",
                    404,
                    "Not Found"
                );
            } else {
                sendHttpResponse(
                    client_fd,
                    image_data,
                    "image/png"
                );
            }
        } else if (path == "/state") {
            sendHttpResponse(
                client_fd,
                makeStateJson(),
                "application/json"
            );
        } else if (path.rfind("/cmd", 0) == 0) {
            std::string key = getQueryValue(path, "key");
            publishCommand(key);

            sendHttpResponse(
                client_fd,
                "{\"ok\":true}",
                "application/json"
            );
        } else {
            sendHttpResponse(
                client_fd,
                "Not found",
                "text/plain",
                404,
                "Not Found"
            );
        }

        close(client_fd);
    }

    void serverLoop() // jednoduche HTTP server vlakno cez sockety
    {
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);

        if (server_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not create socket.");
            return;
        }

        int option = 1;
        setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

        sockaddr_in address;
        std::memset(&address, 0, sizeof(address));

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = inet_addr("127.0.0.1"); // server bezi len lokalne na localhost
        address.sin_port = htons(static_cast<uint16_t>(port_));

        if (bind(server_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not bind web server port.");
            close(server_fd_);
            server_fd_ = -1;
            return;
        }

        if (listen(server_fd_, 10) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not listen on web server socket.");
            close(server_fd_);
            server_fd_ = -1;
            return;
        }

        while (running_) {
            sockaddr_in client_address;
            socklen_t client_length = sizeof(client_address);

            int client_fd = accept(
                server_fd_,
                reinterpret_cast<sockaddr*>(&client_address),
                &client_length
            );

            if (client_fd < 0) {
                if (running_) {
                    RCLCPP_WARN(this->get_logger(), "Web server accept failed.");
                }

                continue;
            }

            std::thread(&WebNode::handleClient, this, client_fd).detach();
        }
    }

    int port_ = 8080;
    double linear_speed_ = 0.0;
    double angular_speed_ = 0.0;
    double map_width_ = 30.72;
    double map_height_ = 20.48;
    std::string map_path_;

    WebRobotState player1_;
    WebRobotState player2_;

    WebLidarState player1_lidar_;
    WebLidarState player2_lidar_;

    zadanie2_interfaces::msg::GameState last_game_state_;
    bool has_game_state_ = false;

    std::mutex data_mutex_;

    bool running_ = false;
    int server_fd_ = -1;
    std::thread server_thread_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr player1_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr player2_odom_sub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr player1_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr player2_scan_sub_;

    rclcpp::Subscription<zadanie2_interfaces::msg::GameState>::SharedPtr game_state_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WebNode>());
    rclcpp::shutdown();
    return 0;
}