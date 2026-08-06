// sdf/SilhouetteSDF.js
// Builds a continuous SDF from up to three orthographic silhouette images
// (Top / Side / Front). The silhouettes are precomputed into 2D signed
// distance maps; the 3D SDF is the boolean intersection of the three
// "extrusions" of those 2D fields — implemented as a max of distances.
//
// This replaces the old approach of stacking thousands of BoxGeometry cubes.
// The result is a true continuous solid: any point in space returns a real
// signed distance to the reconstructed surface.

import { SDF } from './SDFField.js';

/**
 * Extract a 2D binary silhouette (Uint8Array of 0/1) from an HTMLImageElement
 * at a given resolution. "Solid" is defined as luma < threshold OR alpha>0
 * against a light background — matching the original tri-view UX.
 */
export function silhouetteFromImage(img, resU, resV, ctx, threshold = 127) {
    ctx.canvas.width = resU;
    ctx.canvas.height = resV;
    ctx.clearRect(0, 0, resU, resV);
    ctx.drawImage(img, 0, 0, resU, resV);
    const data = ctx.getImageData(0, 0, resU, resV).data;
    const out = new Uint8Array(resU * resV);
    for (let v = 0; v < resV; v++) {
        for (let u = 0; u < resU; u++) {
            const i = (v * resU + u) * 4;
            const a = data[i + 3];
            const luma = (data[i] + data[i + 1] + data[i + 2]) / 3;
            // Treat transparent as background; dark pixels are the part.
            const solid = a > 10 && luma < threshold;
            out[v * resU + u] = solid ? 1 : 0;
        }
    }
    return out;
}

/**
 * Chamfer distance transform on a binary mask (fast, good enough for offsets).
 * Returns a Float32Array of signed distances in *pixel* units: negative inside.
 */
export function signedDistance2D(mask, w, h) {
    const INF = 1e9;
    const din  = new Float32Array(w * h).fill(INF); // distance to nearest outside pixel (for inside points)
    const dout = new Float32Array(w * h).fill(INF); // distance to nearest inside  pixel (for outside points)

    for (let i = 0; i < w * h; i++) {
        if (mask[i]) din[i]  = 0; // seed inside pixels for "din" propagation? no — inverted below
        else         dout[i] = 0;
    }
    // We actually want:
    //   dout[i]=0 on inside pixels (dist to inside from outside)
    //   din[i] =0 on outside pixels (dist to outside from inside)
    for (let i = 0; i < w * h; i++) {
        din[i]  = mask[i] ? INF : 0; // inside points measure to nearest outside
        dout[i] = mask[i] ? 0   : INF;
    }

    const relax = (d) => {
        // Forward pass
        for (let y = 0; y < h; y++) {
            for (let x = 0; x < w; x++) {
                const i = y * w + x;
                let v = d[i];
                if (x > 0)              v = Math.min(v, d[i - 1]     + 1);
                if (y > 0)              v = Math.min(v, d[i - w]     + 1);
                if (x > 0 && y > 0)     v = Math.min(v, d[i - w - 1] + 1.41421356);
                if (x < w - 1 && y > 0) v = Math.min(v, d[i - w + 1] + 1.41421356);
                d[i] = v;
            }
        }
        // Backward pass
        for (let y = h - 1; y >= 0; y--) {
            for (let x = w - 1; x >= 0; x--) {
                const i = y * w + x;
                let v = d[i];
                if (x < w - 1)                  v = Math.min(v, d[i + 1]     + 1);
                if (y < h - 1)                  v = Math.min(v, d[i + w]     + 1);
                if (x < w - 1 && y < h - 1)     v = Math.min(v, d[i + w + 1] + 1.41421356);
                if (x > 0     && y < h - 1)     v = Math.min(v, d[i + w - 1] + 1.41421356);
                d[i] = v;
            }
        }
    };
    relax(din);
    relax(dout);

    const sdf = new Float32Array(w * h);
    for (let i = 0; i < w * h; i++) {
        // inside -> negative distance = -distance-to-boundary
        sdf[i] = mask[i] ? -din[i] : dout[i];
    }
    return sdf;
}

/**
 * A 2D SDF sampled bilinearly, extruded implicitly along one axis.
 * `plane`: which two world axes the image lies in.
 *    'XZ' : u=X, v=Z  (Top view)
 *    'XY' : u=X, v=Y  (Side view, image v grows downward = -Y up)
 *    'ZY' : u=Z, v=Y  (Front view)
 */
export class Silhouette2DExtruded extends SDF {
    constructor({ sdf2d, w, h, plane, bounds, pixelSizeWorld }) {
        super();
        this.sdf = sdf2d;
        this.w = w;
        this.h = h;
        this.plane = plane;
        this.bounds = bounds; // world-space {min,max}
        this.px = pixelSizeWorld; // world units per pixel (uniform along u/v)
    }

    sample2D(u, v) {
        // Bilinear sample of the 2D SDF at continuous pixel coord (u,v).
        const w = this.w, h = this.h;
        const x0 = Math.max(0, Math.min(w - 2, Math.floor(u)));
        const y0 = Math.max(0, Math.min(h - 2, Math.floor(v)));
        const tx = Math.max(0, Math.min(1, u - x0));
        const ty = Math.max(0, Math.min(1, v - y0));
        const a = this.sdf[y0 * w + x0];
        const b = this.sdf[y0 * w + x0 + 1];
        const c = this.sdf[(y0 + 1) * w + x0];
        const d = this.sdf[(y0 + 1) * w + x0 + 1];
        return (a * (1 - tx) + b * tx) * (1 - ty)
             + (c * (1 - tx) + d * tx) * ty;
    }

    distance(x, y, z) {
        const b = this.bounds;
        let uWorld, vWorld, uMin, vMin, uMax, vMax;

        if (this.plane === 'XZ') {
            uWorld = x; vWorld = z;
            uMin = b.min[0]; vMin = b.min[2];
            uMax = b.max[0]; vMax = b.max[2];
        } else if (this.plane === 'XY') {
            uWorld = x; vWorld = y;
            uMin = b.min[0]; vMin = b.min[1];
            uMax = b.max[0]; vMax = b.max[1];
        } else { // 'ZY'
            uWorld = z; vWorld = y;
            uMin = b.min[2]; vMin = b.min[1];
            uMax = b.max[2]; vMax = b.max[1];
        }

        // Convert world -> pixel coords. Image v grows *downward*, so flip.
        const u = ((uWorld - uMin) / (uMax - uMin)) * (this.w - 1);
        const v = (1 - (vWorld - vMin) / (vMax - vMin)) * (this.h - 1);
        const dPixels = this.sample2D(u, v);
        return dPixels * this.px;
    }
}

/**
 * Compose 1..3 orthographic silhouettes into a single 3D SDF via boolean
 * intersection. Missing views degrade gracefully (an infinite prism = no
 * constraint), which lets the user start with just Top + Side (matching the
 * original tool) and refine by adding Front.
 */
export class TriViewSDF extends SDF {
    constructor(views /* array of Silhouette2DExtruded */) {
        super();
        this.views = views;
    }
    distance(x, y, z) {
        let d = -Infinity;
        for (let i = 0; i < this.views.length; i++) {
            const dv = this.views[i].distance(x, y, z);
            if (dv > d) d = dv;
        }
        return d;
    }
}
