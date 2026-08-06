// extract/MarchingCubes.js
// Classic Marching Cubes (Lorensen & Cline 1987) surface extraction.
// Consumes an SDFSampler and produces a smooth triangulated manifold
// mesh — best for organic geometry and voxel-artifact-free previews.
//
// Tables adapted from the well-known public-domain implementation by Paul Bourke.

import { MC_EDGE_TABLE, MC_TRI_TABLE } from './tables/MarchingCubesTables.js';
import { registerExtractor } from './ExtractorRegistry.js';

// Edge endpoint indices (cube corners each edge connects), matching the
// canonical MC corner ordering: 0..7 mapped to (i,j,k) offsets below.
const CORNER_OFFSETS = [
    [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
    [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1],
];
const EDGE_ENDPOINTS = [
    [0, 1], [1, 2], [2, 3], [3, 0],
    [4, 5], [5, 6], [6, 7], [7, 4],
    [0, 4], [1, 5], [2, 6], [3, 7],
];

export class MarchingCubesExtractor {
    constructor(opts = {}) {
        this.iso = opts.iso ?? 0.0;
    }

    async extract(sampler, onProgress = null) {
        const grid = sampler.grid;
        const [nx, ny, nz] = grid.dims;
        const iso = this.iso;
        const V = sampler.values;

        const positions = [];
        const indices = [];
        const cache = new Map(); // dedupe shared edge vertices

        const edgeKey = (i, j, k, edge) => `${i}_${j}_${k}_${edge}`;

        const corners = new Array(8);
        const values  = new Array(8);
        const p = [0, 0, 0];

        for (let k = 0; k < nz - 1; k++) {
            for (let j = 0; j < ny - 1; j++) {
                for (let i = 0; i < nx - 1; i++) {
                    // Sample corners
                    let cubeIndex = 0;
                    for (let c = 0; c < 8; c++) {
                        const [di, dj, dk] = CORNER_OFFSETS[c];
                        const ii = i + di, jj = j + dj, kk = k + dk;
                        const v = V[grid.index(ii, jj, kk)];
                        values[c] = v;
                        corners[c] = [ii, jj, kk];
                        if (v < iso) cubeIndex |= (1 << c);
                    }
                    const edgeMask = MC_EDGE_TABLE[cubeIndex];
                    if (edgeMask === 0) continue;

                    // Compute per-edge vertex indices (with caching).
                    const edgeVerts = new Array(12);
                    for (let e = 0; e < 12; e++) {
                        if ((edgeMask & (1 << e)) === 0) continue;
                        const [ea, eb] = EDGE_ENDPOINTS[e];
                        const [ai, aj, ak] = corners[ea];
                        const [bi, bj, bk] = corners[eb];
                        const key = edgeKey(
                            Math.min(ai, bi), Math.min(aj, bj), Math.min(ak, bk),
                            e
                        );
                        let idx = cache.get(key);
                        if (idx === undefined) {
                            const va = values[ea], vb = values[eb];
                            let t = (iso - va) / (vb - va);
                            if (!isFinite(t)) t = 0.5;
                            grid.writeWorldAt(ai, aj, ak, p);
                            const wax = p[0], way = p[1], waz = p[2];
                            grid.writeWorldAt(bi, bj, bk, p);
                            const wbx = p[0], wby = p[1], wbz = p[2];
                            idx = positions.length / 3;
                            positions.push(
                                wax + t * (wbx - wax),
                                way + t * (wby - way),
                                waz + t * (wbz - waz),
                            );
                            cache.set(key, idx);
                        }
                        edgeVerts[e] = idx;
                    }

                    // Emit triangles from MC_TRI_TABLE
                    const tri = MC_TRI_TABLE[cubeIndex];
                    for (let t = 0; tri[t] !== -1; t += 3) {
                        indices.push(edgeVerts[tri[t]], edgeVerts[tri[t + 1]], edgeVerts[tri[t + 2]]);
                    }
                }
            }
            if (onProgress) onProgress(k / (nz - 1));
        }

        return {
            positions: new Float32Array(positions),
            indices: new Uint32Array(indices),
        };
    }
}

registerExtractor('marching_cubes', () => new MarchingCubesExtractor(), {
    label: 'Marching Cubes',
    features: ['smooth', 'organic', 'manifold'],
});
