// extract/SurfaceNets.js
// Naive Surface Nets (S.F.F. Gibson 1998).
// For every cell whose 8 corners straddle the iso-surface, place a single
// vertex at the average of the sign-change edge crossings and stitch
// quads across active faces between neighboring cells.
//
// Trade-offs vs Marching Cubes:
//   + Lower polygon count (one vertex per active cell, not per edge).
//   + Faster generation, ideal for real-time preview.
//   + Smoother appearance than raw voxels; efficient topology.
//   - Slightly less crisp than Dual Contouring (no Hermite feature preservation).

import { registerExtractor } from './ExtractorRegistry.js';

const CELL_CORNERS = [
    [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
    [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1],
];
const CELL_EDGES = [
    [0, 1], [1, 2], [2, 3], [3, 0],
    [4, 5], [5, 6], [6, 7], [7, 4],
    [0, 4], [1, 5], [2, 6], [3, 7],
];

export class SurfaceNetsExtractor {
    constructor(opts = {}) {
        this.iso = opts.iso ?? 0.0;
    }

    async extract(sampler, onProgress = null) {
        const grid = sampler.grid;
        const [nx, ny, nz] = grid.dims;
        const V = sampler.values;
        const iso = this.iso;

        // Vertex index per active cell (-1 = inactive).
        const cellVertex = new Int32Array((nx - 1) * (ny - 1) * (nz - 1)).fill(-1);
        const cellIdx = (i, j, k) => i + j * (nx - 1) + k * (nx - 1) * (ny - 1);

        const positions = [];
        const corners = new Array(8);
        const cvals   = new Array(8);
        const p = [0, 0, 0];

        // First pass: place a single vertex in every active cell.
        for (let k = 0; k < nz - 1; k++) {
            for (let j = 0; j < ny - 1; j++) {
                for (let i = 0; i < nx - 1; i++) {
                    let mask = 0;
                    for (let c = 0; c < 8; c++) {
                        const [di, dj, dk] = CELL_CORNERS[c];
                        const v = V[grid.index(i + di, j + dj, k + dk)];
                        cvals[c] = v;
                        corners[c] = [i + di, j + dj, k + dk];
                        if (v < iso) mask |= (1 << c);
                    }
                    if (mask === 0 || mask === 0xff) continue;

                    // Average of edge crossings.
                    let sx = 0, sy = 0, sz = 0, count = 0;
                    for (let e = 0; e < 12; e++) {
                        const [a, b] = CELL_EDGES[e];
                        const va = cvals[a], vb = cvals[b];
                        if ((va < iso) === (vb < iso)) continue;
                        let t = (iso - va) / (vb - va);
                        if (!isFinite(t)) t = 0.5;
                        grid.writeWorldAt(corners[a][0], corners[a][1], corners[a][2], p);
                        const ax = p[0], ay = p[1], az = p[2];
                        grid.writeWorldAt(corners[b][0], corners[b][1], corners[b][2], p);
                        sx += ax + t * (p[0] - ax);
                        sy += ay + t * (p[1] - ay);
                        sz += az + t * (p[2] - az);
                        count++;
                    }
                    if (count === 0) continue;
                    const vi = positions.length / 3;
                    positions.push(sx / count, sy / count, sz / count);
                    cellVertex[cellIdx(i, j, k)] = vi;
                }
            }
            if (onProgress) onProgress(0.5 * k / (nz - 1));
        }

        // Second pass: for every sign-change edge on the +X/+Y/+Z axes at a grid
        // corner, stitch a quad across the four cells sharing that edge.
        const indices = [];
        const emitQuad = (a, b, c, d, flip) => {
            if (a < 0 || b < 0 || c < 0 || d < 0) return;
            if (flip) {
                indices.push(a, c, b);
                indices.push(a, d, c);
            } else {
                indices.push(a, b, c);
                indices.push(a, c, d);
            }
        };

        for (let k = 0; k < nz; k++) {
            for (let j = 0; j < ny; j++) {
                for (let i = 0; i < nx; i++) {
                    const v0 = V[grid.index(i, j, k)];

                    // Edge along +X
                    if (i < nx - 1 && j > 0 && k > 0 && j < ny && k < nz) {
                        const v1 = V[grid.index(i + 1, j, k)];
                        if ((v0 < iso) !== (v1 < iso)) {
                            const c0 = cellVertex[cellIdx(i, j - 1, k - 1)];
                            const c1 = cellVertex[cellIdx(i, j,     k - 1)];
                            const c2 = cellVertex[cellIdx(i, j,     k)];
                            const c3 = cellVertex[cellIdx(i, j - 1, k)];
                            emitQuad(c0, c1, c2, c3, v0 < iso);
                        }
                    }
                    // Edge along +Y
                    if (j < ny - 1 && i > 0 && k > 0) {
                        const v1 = V[grid.index(i, j + 1, k)];
                        if ((v0 < iso) !== (v1 < iso)) {
                            const c0 = cellVertex[cellIdx(i - 1, j, k - 1)];
                            const c1 = cellVertex[cellIdx(i,     j, k - 1)];
                            const c2 = cellVertex[cellIdx(i,     j, k)];
                            const c3 = cellVertex[cellIdx(i - 1, j, k)];
                            emitQuad(c0, c1, c2, c3, !(v0 < iso));
                        }
                    }
                    // Edge along +Z
                    if (k < nz - 1 && i > 0 && j > 0) {
                        const v1 = V[grid.index(i, j, k + 1)];
                        if ((v0 < iso) !== (v1 < iso)) {
                            const c0 = cellVertex[cellIdx(i - 1, j - 1, k)];
                            const c1 = cellVertex[cellIdx(i,     j - 1, k)];
                            const c2 = cellVertex[cellIdx(i,     j,     k)];
                            const c3 = cellVertex[cellIdx(i - 1, j,     k)];
                            emitQuad(c0, c1, c2, c3, v0 < iso);
                        }
                    }
                }
            }
            if (onProgress) onProgress(0.5 + 0.5 * k / (nz - 1));
        }

        return {
            positions: new Float32Array(positions),
            indices: new Uint32Array(indices),
        };
    }
}

registerExtractor('surface_nets', () => new SurfaceNetsExtractor(), {
    label: 'Surface Nets',
    features: ['low_poly', 'fast', 'preview'],
});
