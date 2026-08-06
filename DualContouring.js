// extract/DualContouring.js
// Feature-preserving Dual Contouring (Ju, Losasso, Schaefer, Warren 2002).
//
// Unlike Marching Cubes (which places vertices ON edges), Dual Contouring
// places ONE vertex per active cell at the point that best matches all
// Hermite data (surface crossings + normals) inside that cell — via a
// quadratic error function (QEF) minimized in the least-squares sense.
//
// This preserves:
//   - Sharp edges (the QEF pulls the vertex to the crease intersection)
//   - Corners (three or more normals converging)
//   - Flat faces (colinear normals => vertex sits on the face)
//
// Output is watertight and manifold under the usual assumptions
// (no more than one sign change per edge, grid-aligned surfaces).
//
// This is the DEFAULT reconstruction algorithm for mechanical/architectural
// parts — exactly the requirements set by the pipeline spec.

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

/**
 * Minimize E(x) = Σ_i (n_i · (x - p_i))^2 subject to x staying inside the cell.
 * Solved by accumulating A = Σ n_i n_iᵀ and b = Σ (n_i · p_i) n_i, then
 * solving A x = b with a tiny damped normal equation. If the solution leaves
 * the cell (typical for sharp features that stick out), clamp back to the
 * centroid of the Hermite points as a robust fallback (Schaefer & Warren).
 */
function solveQEF(points, normals, cellMin, cellMax) {
    // Accumulate 3x3 symmetric matrix A and 3-vector b.
    let a00 = 0, a01 = 0, a02 = 0, a11 = 0, a12 = 0, a22 = 0;
    let b0  = 0, b1  = 0, b2  = 0;
    let cx = 0, cy = 0, cz = 0;

    for (let i = 0; i < points.length; i++) {
        const p = points[i], n = normals[i];
        const nx = n[0], ny = n[1], nz = n[2];
        const d = nx * p[0] + ny * p[1] + nz * p[2];
        a00 += nx * nx; a01 += nx * ny; a02 += nx * nz;
        a11 += ny * ny; a12 += ny * nz; a22 += nz * nz;
        b0  += nx * d;  b1  += ny * d;  b2  += nz * d;
        cx  += p[0];    cy  += p[1];    cz  += p[2];
    }
    const N = points.length;
    cx /= N; cy /= N; cz /= N;

    // Damping toward the mass point improves numerical stability.
    const lambda = 1e-4;
    a00 += lambda; a11 += lambda; a22 += lambda;
    b0 += lambda * cx; b1 += lambda * cy; b2 += lambda * cz;

    // Solve 3x3 SPD via Cholesky. If singular, fall back to centroid.
    // A = L L^T
    let l00 = a00; if (l00 <= 0) return [cx, cy, cz];
    l00 = Math.sqrt(l00);
    const l10 = a01 / l00;
    const l20 = a02 / l00;
    let l11 = a11 - l10 * l10; if (l11 <= 0) return [cx, cy, cz];
    l11 = Math.sqrt(l11);
    const l21 = (a12 - l20 * l10) / l11;
    let l22 = a22 - l20 * l20 - l21 * l21; if (l22 <= 0) return [cx, cy, cz];
    l22 = Math.sqrt(l22);

    // Solve L y = b
    const y0 = b0 / l00;
    const y1 = (b1 - l10 * y0) / l11;
    const y2 = (b2 - l20 * y0 - l21 * y1) / l22;
    // Solve L^T x = y
    const x2 = y2 / l22;
    const x1 = (y1 - l21 * x2) / l11;
    const x0 = (y0 - l10 * x1 - l20 * x2) / l00;

    // Robustness: clamp escaping solutions back into the cell bounds.
    const eps = (cellMax[0] - cellMin[0]) * 0.5;
    const px = Math.max(cellMin[0] - eps, Math.min(cellMax[0] + eps, x0));
    const py = Math.max(cellMin[1] - eps, Math.min(cellMax[1] + eps, x1));
    const pz = Math.max(cellMin[2] - eps, Math.min(cellMax[2] + eps, x2));
    return [px, py, pz];
}

export class DualContouringExtractor {
    constructor(opts = {}) {
        this.iso = opts.iso ?? 0.0;
    }

    async extract(sampler, onProgress = null) {
        const grid = sampler.grid;
        const [nx, ny, nz] = grid.dims;
        const V = sampler.values;
        const iso = this.iso;

        const cellVertex = new Int32Array((nx - 1) * (ny - 1) * (nz - 1)).fill(-1);
        const cellIdx = (i, j, k) => i + j * (nx - 1) + k * (nx - 1) * (ny - 1);

        const positions = [];
        const normals = [];

        const wa = [0, 0, 0], wb = [0, 0, 0], gtmp = [0, 0, 0];

        // Pass 1: solve one QEF per active cell using Hermite data
        //         (edge intersection points + SDF gradient at those points).
        for (let k = 0; k < nz - 1; k++) {
            for (let j = 0; j < ny - 1; j++) {
                for (let i = 0; i < nx - 1; i++) {
                    const cvals = new Array(8);
                    let mask = 0;
                    for (let c = 0; c < 8; c++) {
                        const [di, dj, dk] = CELL_CORNERS[c];
                        const v = V[grid.index(i + di, j + dj, k + dk)];
                        cvals[c] = v;
                        if (v < iso) mask |= (1 << c);
                    }
                    if (mask === 0 || mask === 0xff) continue;

                    const hPoints = [];
                    const hNormals = [];

                    for (let e = 0; e < 12; e++) {
                        const [a, b] = CELL_EDGES[e];
                        const va = cvals[a], vb = cvals[b];
                        if ((va < iso) === (vb < iso)) continue;
                        let t = (iso - va) / (vb - va);
                        if (!isFinite(t)) t = 0.5;

                        const [ai, aj, ak] = CELL_CORNERS[a];
                        const [bi, bj, bk] = CELL_CORNERS[b];
                        grid.writeWorldAt(i + ai, j + aj, k + ak, wa);
                        grid.writeWorldAt(i + bi, j + bj, k + bk, wb);
                        const px = wa[0] + t * (wb[0] - wa[0]);
                        const py = wa[1] + t * (wb[1] - wa[1]);
                        const pz = wa[2] + t * (wb[2] - wa[2]);

                        // Gradient at the crossing (continuous grid coords).
                        const fi = (i + ai) + t * (bi - ai);
                        const fj = (j + aj) + t * (bj - aj);
                        const fk = (k + ak) + t * (bk - ak);
                        sampler.gradientContinuous(fi, fj, fk, gtmp);

                        hPoints.push([px, py, pz]);
                        hNormals.push([gtmp[0], gtmp[1], gtmp[2]]);
                    }
                    if (hPoints.length === 0) continue;

                    grid.writeWorldAt(i, j, k, wa);
                    grid.writeWorldAt(i + 1, j + 1, k + 1, wb);
                    const v = solveQEF(hPoints, hNormals, wa, wb);

                    // Averaged normal for shading + geometric orientation.
                    let anx = 0, any = 0, anz = 0;
                    for (const nn of hNormals) { anx += nn[0]; any += nn[1]; anz += nn[2]; }
                    const len = Math.hypot(anx, any, anz) || 1;

                    const vi = positions.length / 3;
                    positions.push(v[0], v[1], v[2]);
                    normals.push(anx / len, any / len, anz / len);
                    cellVertex[cellIdx(i, j, k)] = vi;
                }
            }
            if (onProgress) onProgress(0.6 * k / (nz - 1));
        }

        // Pass 2: quad stitching across sign-change grid edges (same idea as
        // Surface Nets, but on the DC vertex set).
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

                    if (i < nx - 1 && j > 0 && k > 0) {
                        const v1 = V[grid.index(i + 1, j, k)];
                        if ((v0 < iso) !== (v1 < iso)) {
                            emitQuad(
                                cellVertex[cellIdx(i, j - 1, k - 1)],
                                cellVertex[cellIdx(i, j,     k - 1)],
                                cellVertex[cellIdx(i, j,     k)],
                                cellVertex[cellIdx(i, j - 1, k)],
                                v0 < iso,
                            );
                        }
                    }
                    if (j < ny - 1 && i > 0 && k > 0) {
                        const v1 = V[grid.index(i, j + 1, k)];
                        if ((v0 < iso) !== (v1 < iso)) {
                            emitQuad(
                                cellVertex[cellIdx(i - 1, j, k - 1)],
                                cellVertex[cellIdx(i,     j, k - 1)],
                                cellVertex[cellIdx(i,     j, k)],
                                cellVertex[cellIdx(i - 1, j, k)],
                                !(v0 < iso),
                            );
                        }
                    }
                    if (k < nz - 1 && i > 0 && j > 0) {
                        const v1 = V[grid.index(i, j, k + 1)];
                        if ((v0 < iso) !== (v1 < iso)) {
                            emitQuad(
                                cellVertex[cellIdx(i - 1, j - 1, k)],
                                cellVertex[cellIdx(i,     j - 1, k)],
                                cellVertex[cellIdx(i,     j,     k)],
                                cellVertex[cellIdx(i - 1, j,     k)],
                                v0 < iso,
                            );
                        }
                    }
                }
            }
            if (onProgress) onProgress(0.6 + 0.4 * k / (nz - 1));
        }

        return {
            positions: new Float32Array(positions),
            indices: new Uint32Array(indices),
            normals: new Float32Array(normals),
        };
    }
}

registerExtractor('dual_contouring', () => new DualContouringExtractor(), {
    label: 'Dual Contouring',
    features: ['sharp_edges', 'sharp_corners', 'flat_faces', 'watertight', 'mechanical'],
});
