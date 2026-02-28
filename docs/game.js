const SW = window.innerWidth;
const SH = window.innerHeight;
const Tile = 4.0, HallH = 4.0;
const MaxLives = 3, MaxTurns = 5;
const TurnDur = 30.0, GracePer = 3.0;
const MonSpd = 1.2;

// --- State ---
let gs = "Menu"; // Menu, Ready, Playing, GameClear, GameOver, Scare
let turn = 1, lives = MaxLives, activeCh = 0;
let turnTimer = TurnDur, graceTimer = 0;
let monInView = false;
let scarePhase = 0, scareTimer = 0, curScareType = 0;

let points = [];
let segs = [];
let camPos = [], camYaw = [], camName = [], camBroken = [];
let path = [];

// Monster state
let monT = 0, monSpeed = MonSpd, spdTimer = 0;
let monPos = new THREE.Vector3(0,0,0);
let monFaceYaw = 0;

// Three.js State
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f0e0c);
// Fake fog to make it scary
scene.fog = new THREE.Fog(0x0f0e0c, 1, 35);
const camera = new THREE.PerspectiveCamera(80, SW / SH, 0.1, 100);
const renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('webgl-canvas') });
renderer.setSize(SW, SH);

// 2D Canvas State for UI
const uiCanvas = document.getElementById('ui-canvas');
const ctx = uiCanvas.getContext('2d');
uiCanvas.width = SW; uiCanvas.height = SH;

// Objects Group
const mapGroup = new THREE.Group();
scene.add(mapGroup);
let monMesh;

function init3D() {
    // 軽いライト
    const amb = new THREE.AmbientLight(0x404040, 1.0);
    scene.add(amb);
    const pointL = new THREE.PointLight(0xffddaa, 0.8, 20);
    scene.add(pointL);
    
    // モンスターを作る (赤い不気味な球体系)
    const g = new THREE.CylinderGeometry(0.3, 0.3, 1.8, 8);
    const m = new THREE.MeshBasicMaterial({ color: 0xaa1111 });
    monMesh = new THREE.Mesh(g, m);
    // 目玉
    const eg = new THREE.BoxGeometry(0.1, 0.1, 0.1);
    const em = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    const el = new THREE.Mesh(eg, em); el.position.set(-0.15, 0.7, 0.3); monMesh.add(el);
    const er = new THREE.Mesh(eg, em); er.position.set(0.15, 0.7, 0.3); monMesh.add(er);
    scene.add(monMesh);
}
init3D();

// --- Map Generator (Ported from C#) ---
function generateHallway() {
    points = []; segs = []; camPos = []; camYaw = []; camName = []; camBroken = []; path = [];
    while (mapGroup.children.length > 0) mapGroup.remove(mapGroup.children[0]);
    
    let maxPts = Math.min(10, 4 + turn * 2);
    let maxBase = Math.min(1 + turn, 4);
    
    points.push(new THREE.Vector3(0,0,0));
    let q = [0];
    const dirs = [
        new THREE.Vector3(1,0,0), new THREE.Vector3(-1,0,0),
        new THREE.Vector3(0,0,1), new THREE.Vector3(0,0,-1)
    ];
    
    while(q.length > 0 && points.length < maxPts) {
        let pi = q.shift();
        let p = points[pi];
        let numDirs = Math.floor(Math.random() * maxBase) + 1;
        dirs.sort(() => Math.random() - 0.5); // shuffle
        
        for(let i=0; i<numDirs && points.length < maxPts; i++) {
            let dir = dirs[i];
            let len = Math.floor(Math.random()*(turn*2 + 5 - 3)) + 3;
            let ext = new THREE.Vector3().copy(dir).multiplyScalar(len * Tile);
            let endPt = new THREE.Vector3().copy(p).add(ext);
            
            // Generate meshes
            let cc = new THREE.Color(0.2, 0.25, 0.2);
            let mat = new THREE.MeshBasicMaterial({ color: cc, wireframe: true });
            
            let dx = Math.abs(dir.x) > 0 ? (len+1)*Tile : Tile;
            let dz = Math.abs(dir.z) > 0 ? (len+1)*Tile : Tile;
            let boxG = new THREE.BoxGeometry(dx, HallH, dz);
            let cx = p.x + (dir.x * len * Tile)/2;
            let cz = p.z + (dir.z * len * Tile)/2;
            
            let bMesh = new THREE.Mesh(boxG, mat);
            bMesh.position.set(cx, HallH/2, cz);
            mapGroup.add(bMesh);
            
            points.push(endPt);
            q.push(points.length - 1);
            segs.push({ start: p, dir: dir, len: len+1 });
        }
    }
    
    // Assign Cams
    for(let i=0; i<Math.min(10, points.length); i++) {
        camPos.push(new THREE.Vector3(points[i].x, HallH * 0.85, points[i].z));
        let yaw = 0;
        if(segs[0]) {
            yaw = Math.atan2(segs[0].dir.x, -segs[0].dir.z) * 180 / Math.PI;
        }
        camYaw.push(yaw);
        camName.push("CH" + i);
        camBroken.push(turn >= 2 && Math.random() < 0.2);
    }
    
    // Walk Path (DFS Simplification)
    path = [...points]; 
    monT = 0; monSpeed = MonSpd; spdTimer = 0;
    activeCh = 0;
}

// --- Logic ---
function isMonVisible() {
    if(activeCh < 0 || activeCh >= camPos.length) return false;
    if(camBroken[activeCh]) return false;
    let cp = camPos[activeCh];
    let dist = cp.distanceTo(monPos);
    if(dist > Tile * 6) return false;
    // FOV check
    let toMon = new THREE.Vector3().copy(monPos).sub(cp).normalize();
    let cy = camYaw[activeCh] * Math.PI/180;
    let fwd = new THREE.Vector3(Math.sin(cy), 0, -Math.cos(cy));
    let dot = fwd.dot(toMon);
    if(dot > 0.6) return true;
    return false;
}

let lastTime = performance.now();
function update() {
    let now = performance.now();
    let dt = (now - lastTime) / 1000.0;
    lastTime = now;
    
    // Input
    if(keys['ArrowLeft']) camYaw[activeCh] -= 60 * dt;
    if(keys['ArrowRight']) camYaw[activeCh] += 60 * dt;
    
    if (gs === "Playing") {
        turnTimer -= dt;
        monInView = isMonVisible();
        if(!monInView) graceTimer -= dt;
        else graceTimer = Math.min(GracePer, graceTimer + dt * 2);
        
        if(graceTimer <= 0) {
            lives--; gs = "Scare"; scarePhase = 0; scareTimer = 0;
            curScareType = Math.floor(Math.random() * 6);
            if(lives <= 0) curScareType = 4; // GameOver Force Jump
        }
        if(turnTimer <= 0) {
            turn++;
            if(turn > MaxTurns) gs = "GameClear";
            else { gs = "Ready"; generateHallway(); }
        }
        
        // Monster AI
        if(path.length >= 2) {
            spdTimer -= dt;
            if(spdTimer <= 0) {
                let rv = Math.random();
                if(monInView && turn >= 2) {
                    // Dash away logic simplified
                    monSpeed = (rv > 0.5 ? MonSpd*4 : -MonSpd*4);
                    spdTimer = 0.8;
                } else if(camBroken[activeCh] && turn >= 2) {
                    monSpeed = MonSpd*3; spdTimer = 1.0;
                } else {
                    monSpeed = (rv > 0.8 ? MonSpd*2 : (rv > 0.5 ? 0 : MonSpd));
                    spdTimer = 1.0;
                }
            }
            monT += monSpeed * dt;
            if(monT < 0) { monT = 0; monSpeed = Math.abs(monSpeed); }
            if(monT > path.length - 1) { monT = path.length - 1; monSpeed = -Math.abs(monSpeed); }
            
            let idx = Math.floor(monT);
            let f = monT - idx;
            let p1 = path[idx]; let p2 = path[Math.min(idx+1, path.length-1)];
            monPos.lerpVectors(p1, p2, f);
            monPos.y = 0.9;
        }
        monMesh.position.copy(monPos);
    }
    
    render3D();
    renderUI();
    requestAnimationFrame(update);
}

// --- Render ---
function render3D() {
    if(camBroken[activeCh]) {
        document.getElementById('noise-layer').style.opacity = '0.7';
    } else {
        document.getElementById('noise-layer').style.opacity = '0';
        if(camPos[activeCh]) {
            let cp = camPos[activeCh];
            let cy = camYaw[activeCh] * Math.PI/180;
            camera.position.set(cp.x, cp.y, cp.z);
            let fwd = new THREE.Vector3(Math.sin(cy), -0.2, -Math.cos(cy));
            camera.lookAt(new THREE.Vector3().copy(camera.position).add(fwd));
            
            if(gs === "Scare") {
                scareTimer += 0.016;
                // Scare override camera view
                if(scareTimer > 2) {
                    camera.position.copy(monPos);
                    camera.position.z -= 1;
                    camera.lookAt(monPos);
                    if(scareTimer > 3) {
                        gs = (lives <= 0) ? "GameOver" : "Playing"; 
                        graceTimer = GracePer;
                    }
                }
            }
        }
        renderer.render(scene, camera);
    }
}

function renderUI() {
    ctx.clearRect(0,0,SW,SH);
    
    // Top Bar
    ctx.fillStyle = 'rgba(0,0,0,0.8)'; ctx.fillRect(0,0,SW,50);
    ctx.fillStyle = '#fff'; ctx.font = '22px monospace';
    ctx.fillText(`TURN ${turn}/${MaxTurns}`, 20, 32);
    ctx.fillStyle = '#e43232'; ctx.font = '26px monospace';
    for(let i=0; i<MaxLives; i++) {
        ctx.fillStyle = i < lives ? '#e33232' : '#411e1e';
        ctx.fillText("*", SW - 40 - i*25, 36);
    }
    
    if(gs === "Ready" || gs === "Playing") {
        if(gs==="Playing"){
             ctx.fillStyle = '#fff'; ctx.fillText(`${turnTimer.toFixed(1)}s`, SW/2 - 20, 30);
             if(!monInView && graceTimer < GracePer) {
                 ctx.fillStyle = 'red'; ctx.font = '26px monospace';
                 ctx.fillText(`!! MONSTER LOST: ${graceTimer.toFixed(1)}s !!`, SW/2 - 180, 80);
             }
        }
        // Cameras UI
        let btnW = 50, btnY = SH - 60;
        let startX = SW/2 - (camPos.length*55)/2;
        for(let i=0; i<camPos.length; i++) {
            let bx = startX + i*55;
            let sel = (i === activeCh);
            let err = camBroken[i];
            ctx.fillStyle = sel ? 'rgba(38,155,58,0.9)' : 'rgba(20,20,20,0.8)';
            ctx.fillRect(bx, btnY, btnW, 44);
            ctx.strokeStyle = sel ? '#4bd75f' : '#666'; ctx.lineWidth = sel ? 2 : 1;
            ctx.strokeRect(bx, btnY, btnW, 44);
            
            ctx.fillStyle = err ? '#ff5050' : (sel ? '#fff' : '#aaa');
            ctx.font = '14px monospace';
            ctx.fillText(err ? "[X]" : `[${i}]`, bx+6, btnY+16);
            ctx.fillText(err ? "ERR" : camName[i], bx+6, btnY+36);
        }
        
        // Minimap logic (Basic Draw)
        let mapOx = SW - 200, mapOy = Math.max(80, SH/2 - 150);
        let sc = 4;
        ctx.fillStyle = 'rgba(10,20,10,0.7)'; ctx.fillRect(mapOx, mapOy, 180, 180);
        
        points.forEach(p => {
            let mx = mapOx + 90 + p.x * sc;
            let mz = mapOy + 90 + p.z * sc;
            ctx.fillStyle = '#4a9c4a';
            ctx.fillRect(mx-3, mz-3, 6, 6);
        });
        
        // Cams on map
        camPos.forEach((cp, idx) => {
            let mx = mapOx + 90 + cp.x * sc;
            let mz = mapOy + 90 + cp.z * sc;
            ctx.fillStyle = (idx === activeCh) ? 'yellow' : (camBroken[idx]?'red':'#7dc37d');
            ctx.beginPath(); ctx.arc(mx, mz, idx===activeCh?4:2, 0, Math.PI*2); ctx.fill();
        });
        
        // Mon on map
        let mx = mapOx + 90 + monPos.x * sc;
        let mz = mapOy + 90 + monPos.z * sc;
        ctx.fillStyle = 'red';
        ctx.beginPath(); ctx.arc(mx, mz, 3, 0, Math.PI*2); ctx.fill();
    }
    else if(gs === "Scare") {
        ctx.fillStyle = 'rgba(100,0,0,'+(scareTimer/3)+')';
        ctx.fillRect(0,0,SW,SH);
        if(scareTimer > 2.5) {
            ctx.fillStyle = '#000'; ctx.fillRect(0,0,SW,SH);
        }
    }
    else if(gs === "GameOver") {
        ctx.fillStyle = '#000'; ctx.fillRect(0,0,SW,SH);
        ctx.fillStyle = 'red'; ctx.font = '60px monospace';
        ctx.fillText("GAME OVER", SW/2 - 150, SH/2);
    }
    else if(gs === "GameClear") {
        ctx.fillStyle = '#fff'; ctx.font = '50px monospace';
        ctx.fillText("SURVIVED", SW/2 - 120, SH/2);
    }
}

// Keys
let keys = {};
window.addEventListener('keydown', e => {
    keys[e.code] = true;
    if(e.key >= '0' && e.key <= '9') {
        let act = parseInt(e.key);
        if(act < camPos.length) activeCh = act;
    }
    if(e.code === 'Enter') {
        if(gs === "Menu") { gs = "Playing"; generateHallway(); }
        else if(gs === "Ready") { gs = "Playing"; }
    }
});
window.addEventListener('keyup', e => { keys[e.code] = false; });

// Start UI Logic
document.getElementById('start-btn').addEventListener('click', () => {
    document.getElementById('start-screen').style.display = 'none';
    gs = "Playing";
    generateHallway();
    requestAnimationFrame(update);
});
