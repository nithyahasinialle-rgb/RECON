/**
 * RECON: 3D Multi-Sensor ToF Array & RF SLAM Web Dashboard Engine (Three.js + WebGL)
 * Accumulates permanent, persistent 3D red hit dots mapping out surrounding furniture and walls.
 */

let scene, camera, renderer, controls;
let roverMesh, rayLinesGroup, pointCloudMesh, redDotsGroup, ransacPlaneMesh;
let mappedPoints = [];
let pathPoints = [];
let trajectoryLine;

let isAutoRunning = false;
let autoWaypointIdx = 0;

const WAYPOINTS = [
    {x: 1.5, y: 1.5, name: "Origin Home Base"},
    {x: 3.5, y: 2.3, name: "Oak Table 1 Inspection"},
    {x: 4.5, y: 3.5, name: "Table 1 East Scan"},
    {x: 3.5, y: 4.7, name: "Table 1 North Scan"},
    {x: 1.2, y: 7.5, name: "Storage Cabinet Zone"},
    {x: 6.5, y: 5.5, name: "Walnut Table 2 Inspection"},
    {x: 7.5, y: 6.5, name: "Table 2 East Scan"},
    {x: 8.8, y: 2.5, name: "Bookshelf Station"},
    {x: 1.5, y: 1.5, name: "Return Home Base"}
];

let robotPose = { x: 1.5, y: 1.5, yaw: 0.0 };

function init3D() {
    const container = document.getElementById('canvas-container');
    const width = container.clientWidth;
    const height = container.clientHeight;

    // Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0f1217);

    // Camera
    camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
    camera.position.set(5.0, -7.0, 8.0);
    camera.up.set(0, 0, 1);

    // Renderer
    renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('webgl-canvas'), antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;

    // Orbit Controls
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.target.set(5.0, 5.0, 0.5);
    controls.update();

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(10, -10, 15);
    scene.add(dirLight);

    // Grid (10x10m Room)
    const gridHelper = new THREE.GridHelper(10, 20, 0x00f2fe, 0x223344);
    gridHelper.rotation.x = Math.PI / 2;
    gridHelper.position.set(5, 5, 0);
    scene.add(gridHelper);

    // Build Environment Furniture & Beacons
    buildEnvironment();

    // Build Rover Model
    buildRover();

    // Ray Lines Group
    rayLinesGroup = new THREE.Group();
    scene.add(rayLinesGroup);

    // Permanent Red Hit Dots Group
    redDotsGroup = new THREE.Group();
    scene.add(redDotsGroup);

    // Trajectory Line
    const pathMat = new THREE.LineBasicMaterial({ color: 0x00f2fe, linewidth: 3 });
    const pathGeo = new THREE.BufferGeometry();
    trajectoryLine = new THREE.Line(pathGeo, pathMat);
    scene.add(trajectoryLine);

    // RANSAC Surface Plane
    const planeGeo = new THREE.PlaneGeometry(0.8, 0.8);
    const planeMat = new THREE.MeshStandardMaterial({
        color: 0xffb703,
        transparent: true,
        opacity: 0.35,
        side: THREE.DoubleSide
    });
    ransacPlaneMesh = new THREE.Mesh(planeGeo, planeMat);
    ransacPlaneMesh.visible = false;
    scene.add(ransacPlaneMesh);

    // Event Listeners
    window.addEventListener('resize', onWindowResize);
    setupUIControls();

    // Animation Loop
    animate();
}

function buildEnvironment() {
    // 1. Conference Table 1 (Oak Wood) at (3.5, 3.5)
    createTable(3.5, 3.5, 1.6, 0.9, 0.75, 0x8b5a2b);

    // 2. Conference Table 2 (Dark Walnut) at (6.5, 6.5)
    createTable(6.5, 6.5, 1.6, 0.9, 0.75, 0x3d2817);

    // 3. Storage Cabinet at (1.2, 7.5)
    createBox(1.2, 7.5, 0.7, 0.6, 1.4, 1.4, 0x4a5568);

    // 4. Bookshelf Unit at (8.8, 2.5)
    createBox(8.8, 2.5, 0.8, 0.5, 1.2, 1.6, 0x6b4226);

    // 5. 3 Wi-Fi Access Point Telemetry Beacons
    createBeacon(0.5, 0.5);
    createBeacon(9.5, 0.5);
    createBeacon(5.0, 9.5);
}

function createTable(x, y, sx, sy, sz, colorHex) {
    const group = new THREE.Group();
    const topGeo = new THREE.BoxGeometry(sx, sy, 0.05);
    const topMat = new THREE.MeshStandardMaterial({ color: colorHex, roughness: 0.4 });
    const topMesh = new THREE.Mesh(topGeo, topMat);
    topMesh.position.set(0, 0, sz - 0.025);
    group.add(topMesh);

    const legGeo = new THREE.BoxGeometry(0.08, 0.08, sz - 0.05);
    const legMat = new THREE.MeshStandardMaterial({ color: 0x1a202c, roughness: 0.3 });

    const dx = sx / 2 - 0.08;
    const dy = sy / 2 - 0.08;
    const legPos = [[dx, dy], [dx, -dy], [-dx, dy], [-dx, -dy]];

    legPos.forEach(p => {
        const leg = new THREE.Mesh(legGeo, legMat);
        leg.position.set(p[0], p[1], (sz - 0.05) / 2);
        group.add(leg);
    });

    group.position.set(x, y, 0);
    scene.add(group);
}

function createBox(x, y, z, sx, sy, sz, colorHex) {
    const geo = new THREE.BoxGeometry(sx, sy, sz);
    const mat = new THREE.MeshStandardMaterial({ color: colorHex, roughness: 0.5 });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(x, y, z);
    scene.add(mesh);
}

function createBeacon(x, y) {
    const geo = new THREE.CylinderGeometry(0.12, 0.12, 1.5, 16);
    const mat = new THREE.MeshStandardMaterial({ color: 0xff6b00, roughness: 0.3 });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.rotation.x = Math.PI / 2;
    mesh.position.set(x, y, 0.75);
    scene.add(mesh);
}

function buildRover() {
    roverMesh = new THREE.Group();

    const chassisGeo = new THREE.BoxGeometry(0.30, 0.22, 0.08);
    const chassisMat = new THREE.MeshStandardMaterial({ color: 0x00d2ff, roughness: 0.2, metalness: 0.8 });
    const chassis = new THREE.Mesh(chassisGeo, chassisMat);
    chassis.position.z = 0.05;
    roverMesh.add(chassis);

    const wheelGeo = new THREE.CylinderGeometry(0.04, 0.04, 0.03, 16);
    const wheelMat = new THREE.MeshStandardMaterial({ color: 0x111111 });

    const wPos = [[0.08, 0.12], [0.08, -0.12], [-0.08, 0.12], [-0.08, -0.12]];
    wPos.forEach(p => {
        const w = new THREE.Mesh(wheelGeo, wheelMat);
        w.rotation.x = Math.PI / 2;
        w.position.set(p[0], p[1], 0.04);
        roverMesh.add(w);
    });

    roverMesh.position.set(robotPose.x, robotPose.y, 0);
    scene.add(roverMesh);
}

function updateToFRaysAndPoints() {
    rayLinesGroup.clear();

    const showRays = document.getElementById('chk-show-rays').checked;
    const showRedDots = document.getElementById('chk-show-red-dots').checked;
    const fitPlane = document.getElementById('chk-fit-plane').checked;
    const pointSize = parseFloat(document.getElementById('slider-point-size').value);

    document.getElementById('val-point-size').innerText = pointSize.toFixed(2) + "m";

    const angles = [0, Math.PI / 2, -Math.PI / 2, Math.PI];
    const currentScanPoints = [];

    angles.forEach(offset => {
        const rayAngle = robotPose.yaw + offset;
        const sensorX = robotPose.x + 0.12 * Math.cos(rayAngle);
        const sensorY = robotPose.y + 0.12 * Math.sin(rayAngle);
        const sensorZ = 0.10;

        const grid = 6;
        const fov = 0.45;

        for (let r = 0; r < grid; r++) {
            for (let c = 0; c < grid; c++) {
                const az = (c / (grid - 1) - 0.5) * fov;
                const el = (r / (grid - 1) - 0.5) * fov;
                const heading = rayAngle + az;

                const dist = rayCastDistance(robotPose.x, robotPose.y, heading, el);

                if (dist < 2.0) {
                    const hx = robotPose.x + dist * Math.cos(heading) * Math.cos(el);
                    const hy = robotPose.y + dist * Math.sin(heading) * Math.cos(el);
                    const hz = 0.10 + dist * Math.sin(el);

                    currentScanPoints.push({x: hx, y: hy, z: hz});
                    addAccumulatedPoint(hx, hy, hz);

                    if (showRays) {
                        const lineMat = new THREE.LineBasicMaterial({ color: 0x00f2fe, transparent: true, opacity: 0.65 });
                        const lineGeo = new THREE.BufferGeometry().setFromPoints([
                            new THREE.Vector3(sensorX, sensorY, sensorZ),
                            new THREE.Vector3(hx, hy, hz)
                        ]);
                        const rayLine = new THREE.Line(lineGeo, lineMat);
                        rayLinesGroup.add(rayLine);
                    }
                }
            }
        }
    });

    // Render ALL accumulated 3D red hit dots permanently
    renderPermanentRedHitDots(showRedDots, pointSize);

    if (fitPlane && currentScanPoints.length > 4) {
        fitRANSACPlane(currentScanPoints);
    } else {
        ransacPlaneMesh.visible = false;
    }
}

function rayCastDistance(rx, ry, heading, el) {
    const dx = Math.cos(heading) * Math.cos(el);
    const dy = Math.sin(heading) * Math.cos(el);
    let minD = 2.0;

    if (Math.abs(dx) > 1e-4) {
        let t1 = (10.0 - rx) / dx; if (t1 > 0 && t1 < minD) minD = t1;
        let t2 = -rx / dx;        if (t2 > 0 && t2 < minD) minD = t2;
    }
    if (Math.abs(dy) > 1e-4) {
        let t1 = (10.0 - ry) / dy; if (t1 > 0 && t1 < minD) minD = t1;
        let t2 = -ry / dy;        if (t2 > 0 && t2 < minD) minD = t2;
    }

    const boxes = [
        {x: 3.5, y: 3.5, sx: 1.6, sy: 0.9},
        {x: 6.5, y: 6.5, sx: 1.6, sy: 0.9},
        {x: 1.2, y: 7.5, sx: 0.6, sy: 1.4},
        {x: 8.8, y: 2.5, sx: 0.5, sy: 1.2}
    ];

    boxes.forEach(b => {
        const minX = b.x - b.sx/2, maxX = b.x + b.sx/2;
        const minY = b.y - b.sy/2, maxY = b.y + b.sy/2;

        if (Math.abs(dx) > 1e-4) {
            let tx1 = (minX - rx)/dx, tx2 = (maxX - rx)/dx;
            [tx1, tx2].forEach(t => {
                if (t > 0 && t < minD) {
                    let py = ry + t * dy;
                    if (minY <= py && py <= maxY) minD = t;
                }
            });
        }
        if (Math.abs(dy) > 1e-4) {
            let ty1 = (minY - ry)/dy, ty2 = (maxY - ry)/dy;
            [ty1, ty2].forEach(t => {
                if (t > 0 && t < minD) {
                    let px = rx + t * dx;
                    if (minX <= px && px <= maxX) minD = t;
                }
            });
        }
    });

    return minD;
}

function addAccumulatedPoint(x, y, z) {
    // Deduplicate within 3cm
    for (let i = Math.max(0, mappedPoints.length - 400); i < mappedPoints.length; i++) {
        const p = mappedPoints[i];
        if (Math.hypot(p.x - x, p.y - y) < 0.03 && Math.abs(p.z - z) < 0.03) return;
    }

    // Add new permanent point
    mappedPoints.push({x, y, z});

    // Create 3D dot mesh immediately and add to permanent redDotsGroup
    const size = parseFloat(document.getElementById('slider-point-size').value) || 0.06;
    const sphereGeo = new THREE.SphereGeometry(size / 2, 6, 6);
    const sphereMat = new THREE.MeshBasicMaterial({ color: 0xff3366, transparent: true, opacity: 0.85 });
    const dotMesh = new THREE.Mesh(sphereGeo, sphereMat);
    dotMesh.position.set(x, y, z);
    redDotsGroup.add(dotMesh);

    document.getElementById('stat-points-count').innerText = mappedPoints.length.toLocaleString();
}

function renderPermanentRedHitDots(visible, size) {
    redDotsGroup.visible = visible;
}

function fitRANSACPlane(pts) {
    let ax = 0, ay = 0, az = 0;
    pts.forEach(p => { ax += p.x; ay += p.y; az += p.z; });
    ax /= pts.length; ay /= pts.length; az /= pts.length;

    ransacPlaneMesh.position.set(ax, ay, az);
    ransacPlaneMesh.visible = true;
}

function updateTrajectory() {
    pathPoints.push(new THREE.Vector3(robotPose.x, robotPose.y, 0.02));
    trajectoryLine.geometry.dispose();
    trajectoryLine.geometry = new THREE.BufferGeometry().setFromPoints(pathPoints);

    document.getElementById('stat-pose').innerText = `(${robotPose.x.toFixed(2)}, ${robotPose.y.toFixed(2)})`;
}

function animate() {
    requestAnimationFrame(animate);

    if (isAutoRunning) {
        stepAutonomousTrial();
    }

    updateToFRaysAndPoints();

    roverMesh.position.set(robotPose.x, robotPose.y, 0);
    roverMesh.rotation.z = robotPose.yaw;

    controls.update();
    renderer.render(scene, camera);
}

function stepAutonomousTrial() {
    const target = WAYPOINTS[autoWaypointIdx];
    const dx = target.x - robotPose.x;
    const dy = target.y - robotPose.y;
    const dist = Math.hypot(dx, dy);

    if (dist < 0.20) {
        autoWaypointIdx++;
        if (autoWaypointIdx >= WAYPOINTS.length) {
            isAutoRunning = false;
            document.getElementById('btn-run-ieee-benchmark').innerHTML = `<i class="fa-solid fa-play"></i> Run Autonomous Trial`;
            return;
        }
    }

    const targetHeading = Math.atan2(dy, dx);
    let headingError = targetHeading - robotPose.yaw;
    while (headingError > Math.PI) headingError -= 2 * Math.PI;
    while (headingError < -Math.PI) headingError += 2 * Math.PI;

    if (Math.abs(headingError) > 0.3) {
        robotPose.yaw += (headingError > 0 ? 0.04 : -0.04);
    } else {
        robotPose.x += 0.03 * Math.cos(robotPose.yaw);
        robotPose.y += 0.03 * Math.sin(robotPose.yaw);
        updateTrajectory();
    }
}

function setupUIControls() {
    document.getElementById('btn-clear-map').addEventListener('click', () => {
        mappedPoints = [];
        pathPoints = [];
        redDotsGroup.clear();
        trajectoryLine.geometry.dispose();
        trajectoryLine.geometry = new THREE.BufferGeometry();
        document.getElementById('stat-points-count').innerText = "0";
    });

    document.getElementById('btn-run-ieee-benchmark').addEventListener('click', () => {
        isAutoRunning = !isAutoRunning;
        const btn = document.getElementById('btn-run-ieee-benchmark');
        if (isAutoRunning) {
            autoWaypointIdx = 0;
            btn.innerHTML = `<i class="fa-solid fa-pause"></i> Pause Trial`;
        } else {
            btn.innerHTML = `<i class="fa-solid fa-play"></i> Run Autonomous Trial`;
        }
    });
}

function onWindowResize() {
    const container = document.getElementById('canvas-container');
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
}

window.onload = init3D;
