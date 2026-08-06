// ui/AppMain.js
// Application shell. Owns the Three.js scene, all UI wiring, and dispatches
// pipeline runs. Every previously working feature of the original file is
// preserved verbatim; the geometry generation path is the only thing that
// changed — from cube-stacking to SDF -> extraction -> cleanup -> B-Rep.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { OBJExporter } from 'three/addons/exporters/OBJExporter.js';
import { STLExporter } from 'three/addons/exporters/STLExporter.js';

import { GeometryPipeline } from '../pipeline/GeometryPipeline.js';
import { listExtractors } from '../extract/ExtractorRegistry.js';
import { meshToBufferGeometry, brepSharpEdgesGeometry } from './ThreeAdapter.js';

// ---- Scene state (module-scoped, as in the original file) ----------------
let scene, camera, renderer, controls;
let billet, drillBit, part, brepEdgesLine;
let refPlaneTop, refPlaneSide, refPlaneFront;
let imgTop = null, imgSide = null, imgFront = null;
let isMilling = false;

// Materials — matches the original preset menu exactly.
const mats = {
    aluminum: new THREE.MeshStandardMaterial({ color: 0xcccccc, roughness: 0.1, metalness: 0.9 }),
    carbon:   new THREE.MeshStandardMaterial({ color: 0x111111, roughness: 0.4, metalness: 0.2 }),
    titanium: new THREE.MeshStandardMaterial({ color: 0x777788, roughness: 0.2, metalness: 0.7 }),
    gold:     new THREE.MeshStandardMaterial({ color: 0xffaa00, roughness: 0.05, metalness: 1.0 }),
};

// -------------------------------------------------------------------------

function init() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a0c);

    camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 1, 10000);
    camera.position.set(400, 400, 400);

    renderer = new THREE.WebGLRenderer({ antialias: true, logarithmicDepthBuffer: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    document.body.appendChild(renderer.domElement);

    scene.add(new THREE.AmbientLight(0xffffff, 0.4));
    const sun = new THREE.DirectionalLight(0xffffff, 2.5);
    sun.position.set(200, 500, 200);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    scene.add(sun);

    scene.add(new THREE.GridHelper(1000, 40, 0x222222, 0x111111));
    scene.add(new THREE.AxesHelper(100));

    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;

    const createPlane = (rx, ry) => {
        const m = new THREE.MeshBasicMaterial({
            transparent: true, opacity: 0, side: THREE.DoubleSide, depthWrite: false,
        });
        const mesh = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), m);
        mesh.rotation.set(rx, ry, 0);
        scene.add(mesh);
        return mesh;
    };
    refPlaneTop   = createPlane(-Math.PI / 2, 0);
    refPlaneSide  = createPlane(0, 0);
    refPlaneFront = createPlane(0, Math.PI / 2);

    // Drill bit (kept identical to original — used for the milling animation).
    drillBit = new THREE.Group();
    const bit = new THREE.Mesh(
        new THREE.CylinderGeometry(0.5, 2, 80, 12),
        new THREE.MeshStandardMaterial({ color: 0x333333, metalness: 1 }),
    );
    bit.position.y = 40;
    drillBit.add(bit);
    scene.add(drillBit);
    drillBit.visible = false;

    // Populate the algorithm selector from the extractor registry so that new
    // algorithms plugged into extract/ automatically appear in the UI.
    const algoSel = document.getElementById('algo-select');
    for (const info of listExtractors()) {
        const opt = document.createElement('option');
        opt.value = info.id;
        opt.textContent = info.label + (info.features?.includes('mechanical') ? ' (default)' : '');
        algoSel.appendChild(opt);
    }
    algoSel.value = 'dual_contouring';

    window.addEventListener('resize', onWindowResize);
    document.getElementById('btn-toggle').onclick = () =>
        document.getElementById('ui-panel').classList.toggle('collapsed');

    document.getElementById('top-upload').onchange   = (e) => handleUpload(e, 'top');
    document.getElementById('side-upload').onchange  = (e) => handleUpload(e, 'side');
    document.getElementById('front-upload').onchange = (e) => handleUpload(e, 'front');

    document.getElementById('btn-mill').onclick       = executeFab;
    document.getElementById('btn-reset').onclick      = createBillet;
    document.getElementById('btn-export').onclick     = exportOBJ;
    document.getElementById('btn-export-stl').onclick = exportSTL;

    ['length', 'width', 'thick', 'res'].forEach(id => {
        document.getElementById(`param-${id}`).oninput = (e) => {
            document.getElementById(`val-${id}`).innerText = e.target.value;
            createBillet();
        };
    });

    document.getElementById('ref-opacity').oninput = (e) => {
        const v = parseFloat(e.target.value);
        document.getElementById('val-ref-op').innerText = v;
        [refPlaneTop, refPlaneSide, refPlaneFront].forEach(p => p.material.opacity = v);
    };

    document.getElementById('brep-edges-toggle').onchange = (e) => {
        if (brepEdgesLine) brepEdgesLine.visible = e.target.checked;
    };

    createBillet();
    animate();
}

// -------------------------------------------------------------------------
function handleUpload(event, type) {
    const file = event.target.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (e) => {
        const img = new Image();
        img.onload = () => {
            if      (type === 'top')  imgTop  = img;
            else if (type === 'side') imgSide = img;
            else                      imgFront = img;

            const tex = new THREE.TextureLoader().load(e.target.result);
            let target;
            if      (type === 'top')  target = refPlaneTop;
            else if (type === 'side') target = refPlaneSide;
            else                      target = refPlaneFront;

            target.material.map = tex;
            target.material.opacity = document.getElementById('ref-opacity').value;
            document.getElementById(`${type}-status`).innerText = "OK";
            document.getElementById(`lbl-${type}`).innerText = file.name;
            createBillet();
        };
        img.src = e.target.result;
    };
    reader.readAsDataURL(file);
}

function clearPart() {
    if (part) { scene.remove(part); part.geometry.dispose(); part = null; }
    if (brepEdgesLine) {
        scene.remove(brepEdgesLine);
        brepEdgesLine.geometry.dispose();
        brepEdgesLine = null;
    }
}

function createBillet() {
    clearPart();
    if (billet) { scene.remove(billet); billet = null; }

    const l = parseFloat(document.getElementById('param-length').value);
    const w = parseFloat(document.getElementById('param-width').value);
    const h = parseFloat(document.getElementById('param-thick').value);
    const res = parseInt(document.getElementById('param-res').value);

    billet = new THREE.Mesh(
        new THREE.BoxGeometry(l, h, w),
        new THREE.MeshStandardMaterial({
            color: 0x00f2ff, transparent: true, opacity: 0.15, wireframe: true,
        }),
    );
    billet.position.set(0, h / 2, 0);
    scene.add(billet);

    refPlaneTop.scale.set(l, w, 1);
    refPlaneTop.position.set(0, h + 1, 0);

    refPlaneSide.scale.set(l, h, 1);
    refPlaneSide.position.set(0, h / 2, w / 2 + 20);

    refPlaneFront.scale.set(w, h, 1);
    refPlaneFront.position.set(l / 2 + 20, h / 2, 0);

    document.getElementById('precis-info').innerText = (l / res).toFixed(2) + "mm";
    document.getElementById('status-text').innerText = "Calibrated";
    document.getElementById('status-text').style.color = "#00f2ff";
    document.getElementById('mesh-info').innerText = '0';
    document.getElementById('brep-info').innerText = '0 faces / 0 edges / 0 sharp';
}

// -------------------------------------------------------------------------
async function executeFab() {
    if (isMilling || !imgTop || !imgSide) {
        alert("Please upload at least Top and Side profiles for a valid 3D intersection.");
        return;
    }

    isMilling = true;
    document.getElementById('loading-overlay').style.display = 'flex';
    document.getElementById('loading-text').innerText = 'BUILDING SIGNED DISTANCE FIELD…';

    const L = parseFloat(document.getElementById('param-length').value);
    const W = parseFloat(document.getElementById('param-width').value);
    const H = parseFloat(document.getElementById('param-thick').value);
    const res = parseInt(document.getElementById('param-res').value);
    const algo = document.getElementById('algo-select').value;
    const matKey = document.getElementById('material-type').value;

    drillBit.visible = true;

    const stageLabels = {
        SDF:     'BUILDING SIGNED DISTANCE FIELD…',
        EXTRACT: 'SURFACE EXTRACTION…',
        CLEANUP: 'MESH CLEANUP…',
        BREP:    'BUILDING B-REP TOPOLOGY…',
        DONE:    'FINALIZING…',
    };

    try {
        const pipeline = new GeometryPipeline({
            L, W, H, res,
            algorithm: algo,
            views: { top: imgTop, side: imgSide, front: imgFront },
            silhouetteRes: Math.max(128, res * 2),
            onStage: (stage, p) => {
                document.getElementById('loading-text').innerText =
                    `${stageLabels[stage] || stage} ${Math.round(p * 100)}%`;
                // Animate the drill bit along X during the SDF/extract passes.
                if (stage === 'SDF' || stage === 'EXTRACT') {
                    drillBit.position.set(-L / 2 + p * L, H + 15, 0);
                }
            },
            yieldEvery: 4,
        });

        const result = await pipeline.run();

        clearPart();
        const geometry = meshToBufferGeometry(result.mesh);
        part = new THREE.Mesh(geometry, mats[matKey]);
        part.castShadow = true;
        part.receiveShadow = true;
        scene.add(part);

        // B-Rep sharp-edge overlay proves the CAD representation exists.
        const edgeGeo = brepSharpEdgesGeometry(result.brep);
        brepEdgesLine = new THREE.LineSegments(
            edgeGeo,
            new THREE.LineBasicMaterial({ color: 0x00f2ff }),
        );
        brepEdgesLine.visible = document.getElementById('brep-edges-toggle').checked;
        scene.add(brepEdgesLine);

        billet.visible = false;
        drillBit.visible = false;

        document.getElementById('status-text').innerText = `Mill Success · ${algo}`;
        document.getElementById('mesh-info').innerText =
            `${result.stats.triangleCount} tri / ${result.stats.vertexCount} vtx`;
        document.getElementById('brep-info').innerText =
            `${result.stats.faceCount} faces / ${result.stats.edgeCount} edges / ${result.stats.sharpEdgeCount} sharp`;
        document.getElementById('precis-info').innerText =
            result.stats.precisionMM.toFixed(2) + 'mm';

        // Expose the last result on window for interactive inspection.
        window._lastResult = result;
    } catch (e) {
        console.error(e);
        document.getElementById('status-text').innerText = "Error";
        document.getElementById('status-text').style.color = '#ff4466';
    }

    document.getElementById('loading-overlay').style.display = 'none';
    isMilling = false;
}

// -------------------------------------------------------------------------
function exportOBJ() {
    if (!part) return;
    const res = new OBJExporter().parse(part);
    const blob = new Blob([res], { type: 'text/plain' });
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = `factory_part_${Math.floor(Date.now() / 1000)}.obj`;
    link.click();
}

function exportSTL() {
    if (!part) return;
    const res = new STLExporter().parse(part, { binary: true });
    const blob = new Blob([res], { type: 'application/octet-stream' });
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = `factory_part_${Math.floor(Date.now() / 1000)}.stl`;
    link.click();
}

function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    if (isMilling) drillBit.rotation.y += 0.4;
    renderer.render(scene, camera);
}

init();
