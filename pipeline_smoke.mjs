// Headless smoke test — verifies the whole pipeline works without a browser.
// Tests: SDF sampling + all three extractors + mesh cleanup + B-Rep build.

import { Grid3D } from '../js/core/Grid3D.js';
import { SDFBox, SDFSphere, SDFCylinder } from '../js/sdf/SDFField.js';
import { SDFSampler } from '../js/sdf/SDFSampler.js';
import { getExtractor, listExtractors } from '../js/extract/ExtractorRegistry.js';
import { MeshCleanup } from '../js/mesh/MeshCleanup.js';
import { BRepBuilder } from '../js/brep/BRepBuilder.js';

// Side-effect imports register the extractors.
import '../js/extract/DualContouring.js';
import '../js/extract/MarchingCubes.js';
import '../js/extract/SurfaceNets.js';

// A mechanically interesting solid: box union sphere, minus a drilled cylinder.
// Exercises sharp corners (box), smooth surface (sphere), and precise hole (cylinder).
const box = new SDFBox([0, 0, 0], [10, 10, 10]);
const sph = new SDFSphere([8, 8, 8], 6);
const hole = new SDFCylinder([0, 0, 0], 3, 40, 'y');
const solid = box.union(sph).difference(hole);

const bounds = { min: [-15, -15, -15], max: [15, 15, 15] };
const grid = new Grid3D(bounds, [40, 40, 40]);
const sampler = new SDFSampler(grid, solid);
await sampler.sample(0);
console.log(`SDF sampled: ${grid.totalSamples} samples, cell=${grid.uniformCell().toFixed(3)}`);

console.log(`Registered extractors: ${listExtractors().map(e => e.id).join(', ')}`);

for (const info of listExtractors()) {
    const ext = getExtractor(info.id);
    const raw = await ext.extract(sampler);
    const cleaned = MeshCleanup.clean(raw, {
        weldEps: grid.uniformCell() * 0.25,
        smoothIterations: info.id === 'dual_contouring' ? 0 : 1,
    });
    const brep = BRepBuilder.build(cleaned, {
        coplanarNormalTolDeg: 4,
        coplanarOffsetTol: grid.uniformCell() * 0.5,
        sharpDihedralDeg: 25,
    });
    const tri = cleaned.indices.length / 3;
    const vtx = cleaned.positions.length / 3;
    const sharp = brep.edges.filter(e => e.sharp).length;
    console.log(
        `  ${info.id.padEnd(16)} → ${String(tri).padStart(5)} tris, ` +
        `${String(vtx).padStart(5)} vtx, ` +
        `${String(brep.faces.length).padStart(4)} B-Rep faces, ` +
        `${String(brep.edges.length).padStart(5)} edges, ` +
        `${sharp} sharp`
    );

    if (tri === 0) throw new Error(`${info.id} produced empty mesh`);
    if (brep.faces.length === 0) throw new Error(`${info.id} produced no B-Rep faces`);
}
console.log('\n✔ pipeline OK');
