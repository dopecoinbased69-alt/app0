// sdf/SDFField.js
// Signed Distance Field: a scalar field f(x,y,z) where the sign encodes inside/outside
// and the magnitude approximates the Euclidean distance to the nearest surface.
//
// This is the PRIMARY modeling representation. Every solid, hollow shell, cavity,
// hole, offset and boolean is expressed as an SDF and sampled onto a Grid3D by
// SDFSampler. Surface extraction (Dual Contouring / Marching Cubes / Surface Nets)
// consumes the sampled field; it never sees individual cubes.

// -------------------------------------------------------------
// Base class: every solid is a function of world position.
// -------------------------------------------------------------
export class SDF {
    /**
     * Signed distance to the surface at world point p=[x,y,z].
     * Negative = inside solid, positive = outside, zero = on the surface.
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @returns {number}
     */
    distance(x, y, z) { return Infinity; }

    // ---- Convenience constructive geometry (fluent) ----
    union(other, k = 0)         { return new SDFUnion(this, other, k); }
    difference(other, k = 0)    { return new SDFDifference(this, other, k); }
    intersection(other, k = 0)  { return new SDFIntersection(this, other, k); }
    offset(d)                   { return new SDFOffset(this, d); }
    shell(thickness)            { return new SDFShell(this, thickness); }
    invert()                    { return new SDFInvert(this); }
}

// -------------------------------------------------------------
// Primitive: axis-aligned box centered at c with half-extents h.
// -------------------------------------------------------------
export class SDFBox extends SDF {
    constructor(center, halfExtents) {
        super();
        this.c = center.slice();
        this.h = halfExtents.slice();
    }
    distance(x, y, z) {
        const dx = Math.abs(x - this.c[0]) - this.h[0];
        const dy = Math.abs(y - this.c[1]) - this.h[1];
        const dz = Math.abs(z - this.c[2]) - this.h[2];
        const ox = Math.max(dx, 0);
        const oy = Math.max(dy, 0);
        const oz = Math.max(dz, 0);
        const outside = Math.sqrt(ox * ox + oy * oy + oz * oz);
        const inside = Math.min(Math.max(dx, Math.max(dy, dz)), 0);
        return outside + inside;
    }
}

// -------------------------------------------------------------
// Primitive: sphere.
// -------------------------------------------------------------
export class SDFSphere extends SDF {
    constructor(center, radius) {
        super();
        this.c = center.slice();
        this.r = radius;
    }
    distance(x, y, z) {
        const dx = x - this.c[0], dy = y - this.c[1], dz = z - this.c[2];
        return Math.sqrt(dx * dx + dy * dy + dz * dz) - this.r;
    }
}

// -------------------------------------------------------------
// Primitive: capped cylinder along an arbitrary axis ('x','y','z').
// Useful for drilled holes.
// -------------------------------------------------------------
export class SDFCylinder extends SDF {
    constructor(center, radius, height, axis = 'y') {
        super();
        this.c = center.slice();
        this.r = radius;
        this.h = height / 2;
        this.axis = axis;
    }
    distance(x, y, z) {
        let px, py, pz;
        if (this.axis === 'y')      { px = x - this.c[0]; py = y - this.c[1]; pz = z - this.c[2]; }
        else if (this.axis === 'x') { px = y - this.c[1]; py = x - this.c[0]; pz = z - this.c[2]; }
        else                        { px = x - this.c[0]; py = z - this.c[2]; pz = y - this.c[1]; }
        const d1 = Math.sqrt(px * px + pz * pz) - this.r;
        const d2 = Math.abs(py) - this.h;
        const ox = Math.max(d1, 0);
        const oy = Math.max(d2, 0);
        return Math.min(Math.max(d1, d2), 0) + Math.sqrt(ox * ox + oy * oy);
    }
}

// -------------------------------------------------------------
// Adapter: any callable f(x,y,z) -> distance becomes an SDF.
// Used by the tri-view pipeline to lift the intersection of three
// orthographic silhouettes into a proper continuous SDF.
// -------------------------------------------------------------
export class SDFFromFunction extends SDF {
    constructor(fn) { super(); this.fn = fn; }
    distance(x, y, z) { return this.fn(x, y, z); }
}

// =============================================================
// Boolean / feature operators.
// All support an optional smoothing parameter k>0 for smooth blends;
// k=0 gives exact sharp booleans (required for mechanical parts).
// =============================================================

function smin(a, b, k) {
    if (k <= 0) return Math.min(a, b);
    const h = Math.max(k - Math.abs(a - b), 0) / k;
    return Math.min(a, b) - h * h * k * 0.25;
}
function smax(a, b, k) {
    return -smin(-a, -b, k);
}

export class SDFUnion extends SDF {
    constructor(a, b, k = 0) { super(); this.a = a; this.b = b; this.k = k; }
    distance(x, y, z) { return smin(this.a.distance(x, y, z), this.b.distance(x, y, z), this.k); }
}

export class SDFIntersection extends SDF {
    constructor(a, b, k = 0) { super(); this.a = a; this.b = b; this.k = k; }
    distance(x, y, z) { return smax(this.a.distance(x, y, z), this.b.distance(x, y, z), this.k); }
}

export class SDFDifference extends SDF {
    constructor(a, b, k = 0) { super(); this.a = a; this.b = b; this.k = k; }
    // A minus B
    distance(x, y, z) { return smax(this.a.distance(x, y, z), -this.b.distance(x, y, z), this.k); }
}

// Uniform inflate (d<0) / erode (d>0) — used for tolerances and offsets.
export class SDFOffset extends SDF {
    constructor(inner, d) { super(); this.inner = inner; this.d = d; }
    distance(x, y, z) { return this.inner.distance(x, y, z) - this.d; }
}

// Hollow shell of given thickness.
export class SDFShell extends SDF {
    constructor(inner, t) { super(); this.inner = inner; this.t = t; }
    distance(x, y, z) { return Math.abs(this.inner.distance(x, y, z)) - this.t * 0.5; }
}

// Flip inside/outside (used to turn a solid into a "cutter cavity" carrier).
export class SDFInvert extends SDF {
    constructor(inner) { super(); this.inner = inner; }
    distance(x, y, z) { return -this.inner.distance(x, y, z); }
}
