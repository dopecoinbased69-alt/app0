// ui/ThreeAdapter.js
// Bridges the pipeline's raw indexed mesh into a Three.js BufferGeometry
// for rendering + export, and provides a small B-Rep visualizer overlay
// (sharp edges highlighted) used to prove the CAD representation exists.

import * as THREE from 'three';

export function meshToBufferGeometry(mesh) {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(mesh.positions, 3));
    if (mesh.normals) {
        g.setAttribute('normal', new THREE.BufferAttribute(mesh.normals, 3));
    }
    g.setIndex(new THREE.BufferAttribute(mesh.indices, 1));
    if (!mesh.normals) g.computeVertexNormals();
    g.computeBoundingBox();
    g.computeBoundingSphere();
    return g;
}

export function brepSharpEdgesGeometry(brep) {
    const positions = [];
    for (const e of brep.edges) {
        if (!e.sharp) continue;
        const [a, b] = e.endpoints();
        positions.push(a.p[0], a.p[1], a.p[2], b.p[0], b.p[1], b.p[2]);
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    return g;
}
