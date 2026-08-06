// pipeline/GeometryPipeline.js
// End-to-end orchestration:
//
//   Input Orthographic Images
//         │
//         ▼
//   Signed Distance Field (SDF)        (sdf/*)
//         │
//         ▼
//   Dual Contouring / Marching Cubes / Surface Nets   (extract/*)
//         │
//         ▼
//   Mesh Cleanup                       (mesh/MeshCleanup)
//         │
//         ▼
//   Boundary Representation (B-Rep)    (brep/*)
//         │
//         ▼
//   Rendering & Export                 (ui / three.js)
//
// The pipeline never generates geometry as individual cubes. It samples a
// continuous SDF onto a grid, extracts a proper solid surface, cleans it,
// then lifts it into an editable B-Rep. Adding new solid-modeling operations
// (fillet/chamfer/shell/exact boolean/...) is a matter of composing new SDFs
// or extending BRepSolid — the core stays unchanged.

import { Grid3D } from '../core/Grid3D.js';
import { SDFBox } from '../sdf/SDFField.js';
import { SDFSampler } from '../sdf/SDFSampler.js';
import {
    silhouetteFromImage, signedDistance2D,
    Silhouette2DExtruded, TriViewSDF,
} from '../sdf/SilhouetteSDF.js';
import { getExtractor } from '../extract/ExtractorRegistry.js';
import { MeshCleanup } from '../mesh/MeshCleanup.js';
import { BRepBuilder } from '../brep/BRepBuilder.js';

// Register built-in extractors (side-effect imports).
import '../extract/DualContouring.js';
import '../extract/MarchingCubes.js';
import '../extract/SurfaceNets.js';

export class GeometryPipeline {
    /**
     * @param {object} params
     *   L,W,H          workpiece dimensions along X,Z,Y (world units, mm)
     *   res            grid density (samples per shortest axis)
     *   algorithm      'dual_contouring' | 'marching_cubes' | 'surface_nets'
     *   views          { top?:HTMLImageElement, side?:HTMLImageElement, front?:HTMLImageElement }
     *   silhouetteRes  optional independent resolution for 2D silhouettes
     *   silhouetteThreshold
     *   onStage        optional (stage:string, progress:number)=>void
     *   yieldEvery     event-loop yield cadence for SDF sampling
     */
    constructor(params) {
        this.params = Object.assign({
            algorithm: 'dual_contouring',
            silhouetteRes: 256,
            silhouetteThreshold: 127,
            onStage: null,
            yieldEvery: 6,
        }, params);
    }

    _stage(name, p = 0) {
        if (this.params.onStage) this.params.onStage(name, p);
    }

    // Build the composite SDF: intersection of the workpiece box with the
    // extruded silhouette prisms. Missing views degrade to no-constraint.
    buildSDF() {
        const { L, W, H, views, silhouetteRes, silhouetteThreshold } = this.params;

        const bounds = {
            min: [-L / 2, 0,     -W / 2],
            max: [ L / 2, H,      W / 2],
        };

        // Workpiece bounding solid (stock billet) — keeps the SDF finite outside
        // the extruded prisms and matches the physical "billet" the CNC starts with.
        const billet = new SDFBox(
            [0, H / 2, 0],
            [L / 2, H / 2, W / 2],
        );

        const layers = [billet];

        const canvas = document.createElement('canvas');
        const ctx = canvas.getContext('2d', { willReadFrequently: true });

        const addSilhouette = (img, plane) => {
            if (!img) return;
            let resU, resV, uSize, vSize;
            if (plane === 'XZ')      { resU = silhouetteRes; resV = silhouetteRes; uSize = L; vSize = W; }
            else if (plane === 'XY') { resU = silhouetteRes; resV = silhouetteRes; uSize = L; vSize = H; }
            else                     { resU = silhouetteRes; resV = silhouetteRes; uSize = W; vSize = H; }

            const mask = silhouetteFromImage(img, resU, resV, ctx, silhouetteThreshold);
            const sdf2d = signedDistance2D(mask, resU, resV);
            const pxWorld = Math.min(uSize / (resU - 1), vSize / (resV - 1));
            layers.push(new Silhouette2DExtruded({
                sdf2d, w: resU, h: resV, plane, bounds, pixelSizeWorld: pxWorld,
            }));
        };

        addSilhouette(views.top,   'XZ');
        addSilhouette(views.side,  'XY');
        addSilhouette(views.front, 'ZY');

        // Composite: intersection of billet with all provided view prisms.
        // TriViewSDF already implements max-of-distances, but for a stronger
        // billet clamp we combine explicitly so the billet is always enforced.
        const composite = layers.length === 1 ? billet : new TriViewSDF(layers);
        return { sdf: composite, bounds };
    }

    async run() {
        this._stage('SDF', 0);
        const { L, W, H, res } = this.params;
        const { sdf, bounds } = this.buildSDF();

        // Grid — cubic aspect, capped by user-selected density.
        const shortest = Math.min(L, W, H);
        const gx = Math.max(8, Math.round((L / shortest) * res));
        const gy = Math.max(8, Math.round((H / shortest) * res));
        const gz = Math.max(8, Math.round((W / shortest) * res));
        const grid = new Grid3D(bounds, [gx, gy, gz]);
        const sampler = new SDFSampler(grid, sdf);
        await sampler.sample(this.params.yieldEvery, (p) => this._stage('SDF', p));

        this._stage('EXTRACT', 0);
        const extractor = getExtractor(this.params.algorithm);
        let raw = await extractor.extract(sampler, (p) => this._stage('EXTRACT', p));

        this._stage('CLEANUP', 0);
        const smoothIter = this.params.algorithm === 'dual_contouring' ? 0
                        : this.params.algorithm === 'surface_nets'     ? 1
                        : 1;
        const weldEps = Math.max(1e-4, grid.uniformCell() * 0.25);
        const cleaned = MeshCleanup.clean(raw, {
            weldEps,
            smoothIterations: smoothIter,
            smoothLambda: 0.35,
        });
        this._stage('CLEANUP', 1);

        this._stage('BREP', 0);
        const brep = BRepBuilder.build(cleaned, {
            coplanarNormalTolDeg: 4,
            coplanarOffsetTol: Math.max(1e-3, grid.uniformCell() * 0.5),
            sharpDihedralDeg: 25,
        });
        brep.sourceSDF = sdf;
        this._stage('BREP', 1);

        this._stage('DONE', 1);
        return {
            mesh: cleaned,   // watertight indexed mesh with normals
            brep,            // editable CAD representation
            grid,            // sampling grid (for diagnostics)
            sampler,         // sampled SDF (for future queries)
            stats: {
                gridDims: grid.dims.slice(),
                vertexCount: cleaned.positions.length / 3,
                triangleCount: cleaned.indices.length / 3,
                faceCount: brep.faces.length,
                edgeCount: brep.edges.length,
                sharpEdgeCount: brep.edges.filter(e => e.sharp).length,
                precisionMM: grid.uniformCell(),
            },
        };
    }
}
