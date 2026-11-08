/*
 * drone_3d_viz.cpp
 * ----------------
 * Membaca file CSV hasil capture drone dan menghasilkan file HTML
 * dengan simulasi 3D drone menggunakan HTML5 Canvas + Three.js (via CDN).
 *
 * Kompilasi:
 *   g++ -std=c++11 -O2 drone_3d_viz.cpp -o drone_3d_viz
 *   ./drone_3d_viz drone_data.csv output.html
 *
 * Output: file HTML yang bisa dibuka di browser untuk visualisasi 3D drone.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>
#include <algorithm>

struct DroneSample {
    double timestamp_ms;
    float roll, pitch, yaw;
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float temp_c;
    float heading;
};

bool parseCSVLine(const std::string& line, DroneSample& s) {
    std::stringstream ss(line);
    std::string token;
    auto getNext = [&]() -> bool {
        if (!std::getline(ss, token, ',')) return false;
        return true;
    };
    if (!getNext()) return false; s.timestamp_ms = std::stod(token);
    if (!getNext()) return false; s.roll = std::stof(token);
    if (!getNext()) return false; s.pitch = std::stof(token);
    if (!getNext()) return false; s.yaw = std::stof(token);
    if (!getNext()) return false; s.ax = std::stof(token);
    if (!getNext()) return false; s.ay = std::stof(token);
    if (!getNext()) return false; s.az = std::stof(token);
    if (!getNext()) return false; s.gx = std::stof(token);
    if (!getNext()) return false; s.gy = std::stof(token);
    if (!getNext()) return false; s.gz = std::stof(token);
    if (!getNext()) return false; s.mx = std::stof(token);
    if (!getNext()) return false; s.my = std::stof(token);
    if (!getNext()) return false; s.mz = std::stof(token);
    if (!getNext()) return false; s.temp_c = std::stof(token);
    if (!getNext()) return false; s.heading = std::stof(token);
    return true;
}

std::vector<DroneSample> loadCSV(const std::string& filename) {
    std::vector<DroneSample> data;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: tidak bisa membuka " << filename << "\n";
        return data;
    }
    std::string line;
    bool headerSkipped = false;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        if (!headerSkipped) {
            std::stringstream check(line);
            std::string first;
            std::getline(check, first, ',');
            try {
                std::stod(first);
                headerSkipped = true;
            } catch (...) {
                headerSkipped = true;
                continue;
            }
        }
        DroneSample s;
        if (parseCSVLine(line, s)) data.push_back(s);
    }
    return data;
}

// Escape string untuk JavaScript
std::string jsEscape(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '"') out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else if (c == '\n') out += "\\n";
        else if (c == '\r') out += "\\r";
        else out += c;
    }
    return out;
}

int main(int argc, char* argv[]) {
    std::string inputFile = "drone_data.csv";
    std::string outputFile = "drone_3d.html";

    if (argc > 1) inputFile = argv[1];
    if (argc > 2) outputFile = argv[2];

    std::vector<DroneSample> samples = loadCSV(inputFile);
    if (samples.empty()) {
        std::cerr << "Tidak ada data.\n";
        return 1;
    }

    std::cout << "Memuat " << samples.size() << " sampel dari " << inputFile << "\n";

    // Susun data JavaScript
    std::stringstream jsData;
    jsData << "const droneData = [\n";
    for (size_t i = 0; i < samples.size(); i++) {
        const auto& s = samples[i];
        jsData << "  {\n";
        jsData << "    t: " << s.timestamp_ms << ",\n";
        jsData << "    roll: " << s.roll << ",\n";
        jsData << "    pitch: " << s.pitch << ",\n";
        jsData << "    yaw: " << s.yaw << ",\n";
        jsData << "    ax: " << s.ax << ", ay: " << s.ay << ", az: " << s.az << ",\n";
        jsData << "    gx: " << s.gx << ", gy: " << s.gy << ", gz: " << s.gz << ",\n";
        jsData << "    mx: " << s.mx << ", my: " << s.my << ", mz: " << s.mz << ",\n";
        jsData << "    heading: " << s.heading << "\n";
        jsData << "  }";
        if (i + 1 < samples.size()) jsData << ",";
        jsData << "\n";
    }
    jsData << "];\n";

    // Buat file HTML lengkap dengan Three.js
    std::ofstream html(outputFile);
    if (!html.is_open()) {
        std::cerr << "Gagal membuat " << outputFile << "\n";
        return 1;
    }

    html << "<!DOCTYPE html>\n";
    html << "<html lang=\"id\">\n";
    html << "<head>\n";
    html << "    <meta charset=\"UTF-8\">\n";
    html << "    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n";
    html << "    <title>Drone 3D Simulation - MPU6050 + Magnetometer</title>\n";
    html << "    <style>\n";
    html << "        body { margin: 0; background: #111; color: #fff; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; overflow: hidden; }\n";
    html << "        #canvas-container { width: 100vw; height: 100vh; position: relative; }\n";
    html << "        #info-panel {\n";
    html << "            position: absolute; top: 20px; left: 20px;\n";
    html << "            background: rgba(0,0,0,0.7); padding: 20px; border-radius: 10px;\n";
    html << "            pointer-events: none; max-width: 300px;\n";
    html << "            border: 1px solid #444; backdrop-filter: blur(5px);\n";
    html << "        }\n";
    html << "        h1 { margin: 0 0 10px 0; font-size: 18px; color: #00d4ff; }\n";
    html << "        .telemetry { font-size: 14px; margin: 5px 0; font-family: 'Consolas', monospace; }\n";
    html << "        .label { color: #888; }\n";
    html << "        .value { color: #00ff88; }\n";
    html << "        #controls {\n";
    html << "            position: absolute; bottom: 20px; left: 20px;\n";
    html << "            background: rgba(0,0,0,0.7); padding: 15px; border-radius: 8px;\n";
    html << "            border: 1px solid #444;\n";
    html << "        }\n";
    html << "        button {\n";
    html << "            background: #00d4ff; border: none; color: #000; padding: 8px 16px;\n";
    html << "            border-radius: 4px; cursor: pointer; font-weight: bold; margin-right: 10px;\n";
    html << "        }\n";
    html << "        button:hover { background: #00b8e6; }\n";
    html << "        button.secondary { background: #666; color: #fff; }\n";
    html << "        #status { color: #ff6b6b; font-size: 12px; margin-top: 10px; }\n";
    html << "    </style>\n";
    html << "</head>\n";
    html << "<body>\n";
    html << "    <div id=\"canvas-container\"></div>\n";
    html << "    <div id=\"info-panel\">\n";
    html << "        <h1>🚁 Drone Attitude Monitor</h1>\n";
    html << "        <div class=\"telemetry\"><span class=\"label\">Waktu:</span> <span class=\"value\" id=\"val-time\">0</span> ms</div>\n";
    html << "        <div class=\"telemetry\"><span class=\"label\">Roll:</span> <span class=\"value\" id=\"val-roll\">0</span>°</div>\n";
    html << "        <div class=\"telemetry\"><span class=\"label\">Pitch:</span> <span class=\"value\" id=\"val-pitch\">0</span>°</div>\n";
    html << "        <div class=\"telemetry\"><span class=\"label\">Yaw:</span> <span class=\"value\" id=\"val-yaw\">0</span>°</div>\n";
    html << "        <div class=\"telemetry\"><span class=\"label\">Heading (Mag):</span> <span class=\"value\" id=\"val-heading\">0</span>°</div>\n";
    html << "        <div class=\"telemetry\"><span class=\"label\">Sampel:</span> <span class=\"value\" id=\"val-sample\">0</span> / " << samples.size() << "</div>\n";
    html << "    </div>\n";
    html << "    <div id=\"controls\">\n";
    html << "        <button id=\"btn-play\">▶ Play</button>\n";
    html << "        <button class=\"secondary\" id=\"btn-pause\">⏸ Pause</button>\n";
    html << "        <button class=\"secondary\" id=\"btn-reset\">🔄 Reset</button>\n";
    html << "        <div id=\"status\">Siap</div>\n";
    html << "    </div>\n";

    html << "    <script src=\"https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js\"></script>\n";
    html << "    <script>\n";
    html << jsData.str();
    html << "\n";
    html << "    // Setup Three.js\n";
    html << "    const container = document.getElementById('canvas-container');\n";
    html << "    const scene = new THREE.Scene();\n";
    html << "    scene.background = new THREE.Color(0x1a1a2e);\n";
    html << "    scene.fog = new THREE.Fog(0x1a1a2e, 10, 50);\n";
    html << "\n";
    html << "    const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 100);\n";
    html << "    camera.position.set(0, 5, 8);\n";
    html << "    camera.lookAt(0, 0, 0);\n";
    html << "\n";
    html << "    const renderer = new THREE.WebGLRenderer({ antialias: true });\n";
    html << "    renderer.setSize(window.innerWidth, window.innerHeight);\n";
    html << "    renderer.shadowMap.enabled = true;\n";
    html << "    container.appendChild(renderer.domElement);\n";
    html << "\n";
    html << "    // Lighting\n";
    html << "    const ambientLight = new THREE.AmbientLight(0x404040, 1);\n";
    html << "    scene.add(ambientLight);\n";
    html << "    const dirLight = new THREE.DirectionalLight(0xffffff, 1);\n";
    html << "    dirLight.position.set(5, 10, 5);\n";
    html << "    dirLight.castShadow = true;\n";
    html << "    scene.add(dirLight);\n";
    html << "\n";
    html << "    // Grid floor\n";
    html << "    const grid = new THREE.GridHelper(40, 40, 0x333333, 0x1a1a2e);\n";
    html << "    scene.add(grid);\n";
    html << "\n";
    html << "    // Drone model\n";
    html << "    const droneGroup = new THREE.Group();\n";
    html << "    scene.add(droneGroup);\n";
    html << "\n";
    html << "    // Body drone (pusat)\n";
    html << "    const bodyGeo = new THREE.BoxGeometry(1.2, 0.2, 0.6);\n";
    html << "    const bodyMat = new THREE.MeshStandardMaterial({ color: 0x222222, metalness: 0.8, roughness: 0.2 });\n";
    html << "    const body = new THREE.Mesh(bodyGeo, bodyMat);\n";
    html << "    body.castShadow = true;\n";
    html << "    droneGroup.add(body);\n";
    html << "\n";
    html << "    // Arms (4 rotor arms)\n";
    html << "    const armGeo = new THREE.BoxGeometry(1.6, 0.05, 0.05);\n";
    html << "    const armMat = new THREE.MeshStandardMaterial({ color: 0x111111 });\n";
    html << "\n";
    html << "    function createArm(x, z, rotY) {\n";
    html << "        const arm = new THREE.Mesh(armGeo, armMat);\n";
    html << "        arm.position.set(x, 0, z);\n";
    html << "        arm.rotation.y = rotY;\n";
    html << "        arm.castShadow = true;\n";
    html << "        droneGroup.add(arm);\n";
    html << "        \n";
    html << "        // Motor\n";
    html << "        const motorGeo = new THREE.CylinderGeometry(0.08, 0.08, 0.15, 8);\n";
    html << "        const motorMat = new THREE.MeshStandardMaterial({ color: 0xff4444 });\n";
    html << "        const motor = new THREE.Mesh(motorGeo, motorMat);\n";
    html << "        motor.position.set(x, 0, z);\n";
    html << "        motor.rotation.x = Math.PI / 2;\n";
    html << "        motor.castShadow = true;\n";
    html << "        droneGroup.add(motor);\n";
    html << "        \n";
    html << "        // Propeller\n";
    html << "        const propGeo = new THREE.BoxGeometry(0.5, 0.02, 0.5);\n";
    html << "        const propMat = new THREE.MeshStandardMaterial({ color: 0x00d4ff, transparent: true, opacity: 0.6 });\n";
    html << "        const prop = new THREE.Mesh(propGeo, propMat);\n";
    html << "        prop.position.set(x, 0.12, z);\n";
    html << "        prop.castShadow = true;\n";
    html << "        droneGroup.add(prop);\n";
    html << "        return { arm, motor, prop };\n";
    html << "    }\n";
    html << "\n";
    html << "    createArm(0.8, 0, 0);\n";
    html << "    createArm(-0.8, 0, Math.PI);\n";
    html << "    createArm(0, 0.8, Math.PI / 2);\n";
    html << "    createArm(0, -0.8, -Math.PI / 2);\n";
    html << "\n";
    html << "    // Center LED / camera\n";
    html << "    const camGeo = new THREE.SphereGeometry(0.06, 8, 8);\n";
    html << "    const camMat = new THREE.MeshStandardMaterial({ color: 0x00ff88, emissive: 0x00ff88 });\n";
    html << "    const centerCam = new THREE.Mesh(camGeo, camMat);\n";
    html << "    centerCam.position.set(0, -0.12, 0.3);\n";
    html << "    droneGroup.add(centerCam);\n";
    html << "\n";
    html << "    // Coordinate axes helper\n";
    html << "    const axes = new THREE.AxesHelper(3);\n";
    html << "    scene.add(axes);\n";
    html << "\n";
    html << "    // State\n";
    html << "    let currentIndex = 0;\n";
    html << "    let isPlaying = false;\n";
    html << "    let animationId = null;\n";
    html << "    let lastTime = 0;\n";
    html << "    let playSpeed = 1.0;\n";
    html << "\n";
    html << "    function updateDrone(index) {\n";
    html << "        if (index < 0 || index >= droneData.length) return;\n";
    html << "        const d = droneData[index];\n";
    html << "\n";
    html << "        // Update UI\n";
    html << "        document.getElementById('val-time').textContent = d.t.toFixed(0);\n";
    html << "        document.getElementById('val-roll').textContent = d.roll.toFixed(2);\n";
    html << "        document.getElementById('val-pitch').textContent = d.pitch.toFixed(2);\n";
    html << "        document.getElementById('val-yaw').textContent = d.yaw.toFixed(2);\n";
    html << "        document.getElementById('val-heading').textContent = d.heading.toFixed(2);\n";
    html << "        document.getElementById('val-sample').textContent = (index + 1);\n";
    html << "\n";
    html << "        // Update drone rotation (Three.js uses radians, internal order: YXZ)\n";
    html << "        // Convert degrees to radians\n";
    html << "        const rollRad = THREE.MathUtils.degToRad(d.roll);\n";
    html << "        const pitchRad = THREE.MathUtils.degToRad(d.pitch);\n";
    html << "        const yawRad = THREE.MathUtils.degToRad(d.yaw);\n";
    html << "\n";
    html << "        droneGroup.rotation.set(pitchRad, yawRad, rollRad); // YXZ order\n";
    html << "\n";
    html << "        // Spin propellers\n";
    html << "        const spin = (Date.now() * 0.02) % (Math.PI * 2);\n";
    html << "        droneGroup.children.forEach(child => {\n";
    html << "            if (child.geometry && child.geometry.type === 'BoxGeometry' && child.scale.x > 1) {\n";
    html << "                // propeller meshes\n";
    html << "            }\n";
    html << "        });\n";
    html << "    }\n";
    html << "\n";
    html << "    function animate() {\n";
    html << "        animationId = requestAnimationFrame(animate);\n";
    html << "\n";
    html << "        const now = Date.now();\n";
    html << "        if (isPlaying && lastTime !== 0) {\n";
    html << "            const dt = (now - lastTime) * playSpeed;\n";
    html << "            const dtMs = droneData[currentIndex + 1] ? droneData[currentIndex + 1].t - droneData[currentIndex].t : 100;\n";
    html << "            if (dt >= dtMs) {\n";
    html << "                currentIndex = Math.min(currentIndex + 1, droneData.length - 1);\n";
    html << "                lastTime = now;\n";
    html << "                updateDrone(currentIndex);\n";
    html << "            }\n";
    html << "        }\n";
    html << "        lastTime = now;\n";
    html << "\n";
    html << "        // Orbit camera slowly around drone\n";
    html << "        const time = Date.now() * 0.001;\n";
    html << "        camera.position.x = 8 * Math.sin(time * 0.3);\n";
    html << "        camera.position.z = 8 * Math.cos(time * 0.3);\n";
    html << "        camera.lookAt(0, 0, 0);\n";
    html << "\n";
    html << "        renderer.render(scene, camera);\n";
    html << "    }\n";
    html << "\n";
    html << "    // Controls\n";
    html << "    document.getElementById('btn-play').onclick = () => { isPlaying = true; document.getElementById('status').textContent = 'Playing...'; };\n";
    html << "    document.getElementById('btn-pause').onclick = () => { isPlaying = false; document.getElementById('status').textContent = 'Paused'; };\n";
    html << "    document.getElementById('btn-reset').onclick = () => { currentIndex = 0; lastTime = 0; updateDrone(0); document.getElementById('status').textContent = 'Reset'; };\n";
    html << "\n";
    html << "    window.addEventListener('resize', () => {\n";
    html << "        camera.aspect = window.innerWidth / window.innerHeight;\n";
    html << "        camera.updateProjectionMatrix();\n";
    html << "        renderer.setSize(window.innerWidth, window.innerHeight);\n";
    html << "    });\n";
    html << "\n";
    html << "    // Init\n";
    html << "    updateDrone(0);\n";
    html << "    animate();\n";
    html << "    </script>\n";
    html << "</body>\n";
    html << "</html>\n";

    html.close();
    std::cout << "Visualisasi 3D disimpan ke: " << outputFile << "\n";
    std::cout << "Buka file tersebut di browser untuk melihat simulasi drone 3D.\n";

    return 0;
}