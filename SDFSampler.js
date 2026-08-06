// sdf/SDFSampler.js
// Samples a continuous SDF onto a regular Grid3D and provides
// trilinear interpolation + central-difference gradient for
// surface extraction and normal computation.

export class SDFSampler {
    /**
     * @param {import('../core/Grid3D.js').Grid3D} grid
     * @param {import('./SDFField.js').SDF} sdf
     */
    constructor(grid, sdf) {
        this.grid = grid;
        this.sdf = sdf;
        this.values = null;
    }

    /**
     * Populate the scalar field on every grid corner.
     * Yields to the browser every `yieldEvery` rows to keep the UI responsive
     * and drive the drill-bit animation.
     */
    async sample(yieldEvery = 8, onProgress = null) {
        const g = this.grid;
        const [nx, ny, nz] = g.dims;
        const buf = new Float32Array(g.totalSamples);
        const p = [0, 0, 0];

        for (let k = 0; k < nz; k++) {
            for (let j = 0; j < ny; j++) {
                for (let i = 0; i < nx; i++) {
                    g.writeWorldAt(i, j, k, p);
                    buf[g.index(i, j, k)] = this.sdf.distance(p[0], p[1], p[2]);
                }
            }
            if (onProgress) onProgress(k / (nz - 1));
            if (yieldEvery > 0 && (k % yieldEvery) === 0) {
                await new Promise(r => (typeof requestAnimationFrame !== 'undefined'
                    ? requestAnimationFrame(r) : setTimeout(r, 0)));
            }
        }
        this.values = buf;
        return buf;
    }

    at(i, j, k) {
        return this.values[this.grid.index(i, j, k)];
    }

    /**
     * Trilinear interpolation at a continuous grid coordinate.
     */
    sampleTrilinear(fi, fj, fk) {
        const g = this.grid;
        const nx = g.dims[0] - 1, ny = g.dims[1] - 1, nz = g.dims[2] - 1;
        const i0 = Math.max(0, Math.min(nx - 1, Math.floor(fi)));
        const j0 = Math.max(0, Math.min(ny - 1, Math.floor(fj)));
        const k0 = Math.max(0, Math.min(nz - 1, Math.floor(fk)));
        const tx = fi - i0, ty = fj - j0, tz = fk - k0;

        const V = this.values;
        const idx = (i, j, k) => g.index(i, j, k);
        const c000 = V[idx(i0,   j0,   k0)];
        const c100 = V[idx(i0+1, j0,   k0)];
        const c010 = V[idx(i0,   j0+1, k0)];
        const c110 = V[idx(i0+1, j0+1, k0)];
        const c001 = V[idx(i0,   j0,   k0+1)];
        const c101 = V[idx(i0+1, j0,   k0+1)];
        const c011 = V[idx(i0,   j0+1, k0+1)];
        const c111 = V[idx(i0+1, j0+1, k0+1)];

        const c00 = c000 * (1 - tx) + c100 * tx;
        const c10 = c010 * (1 - tx) + c110 * tx;
        const c01 = c001 * (1 - tx) + c101 * tx;
        const c11 = c011 * (1 - tx) + c111 * tx;
        const c0 = c00 * (1 - ty) + c10 * ty;
        const c1 = c01 * (1 - ty) + c11 * ty;
        return c0 * (1 - tz) + c1 * tz;
    }

    /**
     * Central-difference gradient of the sampled field.
     * Returns a normalized normal vector (points from inside → outside).
     */
    gradient(i, j, k, out = [0, 0, 0]) {
        const g = this.grid;
        const V = this.values;
        const nx = g.dims[0], ny = g.dims[1], nz = g.dims[2];

        const iL = Math.max(0, i - 1),   iR = Math.min(nx - 1, i + 1);
        const jL = Math.max(0, j - 1),   jR = Math.min(ny - 1, j + 1);
        const kL = Math.max(0, k - 1),   kR = Math.min(nz - 1, k + 1);

        out[0] = (V[g.index(iR, j, k)] - V[g.index(iL, j, k)]) / ((iR - iL) * g.cell[0] || 1);
        out[1] = (V[g.index(i, jR, k)] - V[g.index(i, jL, k)]) / ((jR - jL) * g.cell[1] || 1);
        out[2] = (V[g.index(i, j, kR)] - V[g.index(i, j, kL)]) / ((kR - kL) * g.cell[2] || 1);

        const len = Math.hypot(out[0], out[1], out[2]) || 1;
        out[0] /= len; out[1] /= len; out[2] /= len;
        return out;
    }

    /**
     * Gradient at an arbitrary continuous grid coord (used by Dual Contouring
     * when placing feature-preserving vertices from Hermite data).
     */
    gradientContinuous(fi, fj, fk, out = [0, 0, 0]) {
        const eps = 0.75;
        const dx = this.sampleTrilinear(fi + eps, fj, fk) - this.sampleTrilinear(fi - eps, fj, fk);
        const dy = this.sampleTrilinear(fi, fj + eps, fk) - this.sampleTrilinear(fi, fj - eps, fk);
        const dz = this.sampleTrilinear(fi, fj, fk + eps) - this.sampleTrilinear(fi, fj, fk - eps);
        const len = Math.hypot(dx, dy, dz) || 1;
        out[0] = dx / len; out[1] = dy / len; out[2] = dz / len;
        return out;
    }
}
