// mesh/MeshCleanup.js
// Post-extraction mesh hygiene.
//
// Responsibilities:
//   - Weld coincident vertices (produces indexed, watertight topology).
//   - Drop degenerate triangles (zero-area).
//   - Recompute per-vertex normals from face normals (area-weighted).
//   - Optional Laplacian smoothing pass for Marching Cubes / Surface Nets;
//     Dual Contouring meshes are usually already crisp and skip smoothing.
//
// Input/output format matches the extractors:
//   { positions:Float32Array, indices:Uint32Array, normals?:Float32Array }

export class MeshCleanup {
    /**
     * Weld vertices within `weldEps` world units.
     */
    static weldVertices(mesh, weldEps = 1e-5) {
        const P = mesh.positions;
        const I = mesh.indices;
        const inv = new Int32Array(P.length / 3);
        const map = new Map();
        const outPos = [];

        const q = 1 / weldEps;
        for (let v = 0; v < P.length; v += 3) {
            const kx = Math.round(P[v]     * q);
            const ky = Math.round(P[v + 1] * q);
            const kz = Math.round(P[v + 2] * q);
            const key = `${kx}_${ky}_${kz}`;
            let idx = map.get(key);
            if (idx === undefined) {
                idx = outPos.length / 3;
                outPos.push(P[v], P[v + 1], P[v + 2]);
                map.set(key, idx);
            }
            inv[v / 3] = idx;
        }
        const outIdx = new Uint32Array(I.length);
        for (let i = 0; i < I.length; i++) outIdx[i] = inv[I[i]];
        return { positions: new Float32Array(outPos), indices: outIdx };
    }

    /**
     * Remove triangles with two/three identical indices or zero area.
     */
    static dropDegenerates(mesh, areaEps = 1e-10) {
        const P = mesh.positions;
        const I = mesh.indices;
        const kept = [];
        for (let t = 0; t < I.length; t += 3) {
            const a = I[t], b = I[t + 1], c = I[t + 2];
            if (a === b || b === c || a === c) continue;
            const ax = P[3*a], ay = P[3*a+1], az = P[3*a+2];
            const bx = P[3*b], by = P[3*b+1], bz = P[3*b+2];
            const cx = P[3*c], cy = P[3*c+1], cz = P[3*c+2];
            const ux = bx - ax, uy = by - ay, uz = bz - az;
            const vx = cx - ax, vy = cy - ay, vz = cz - az;
            const nx = uy * vz - uz * vy;
            const ny = uz * vx - ux * vz;
            const nz = ux * vy - uy * vx;
            if (nx * nx + ny * ny + nz * nz < areaEps) continue;
            kept.push(a, b, c);
        }
        return { positions: mesh.positions, indices: new Uint32Array(kept) };
    }

    /**
     * Area-weighted per-vertex normals from face normals.
     */
    static computeNormals(mesh) {
        const P = mesh.positions;
        const I = mesh.indices;
        const N = new Float32Array(P.length);
        for (let t = 0; t < I.length; t += 3) {
            const a = I[t], b = I[t + 1], c = I[t + 2];
            const ax = P[3*a], ay = P[3*a+1], az = P[3*a+2];
            const bx = P[3*b], by = P[3*b+1], bz = P[3*b+2];
            const cx = P[3*c], cy = P[3*c+1], cz = P[3*c+2];
            const ux = bx - ax, uy = by - ay, uz = bz - az;
            const vx = cx - ax, vy = cy - ay, vz = cz - az;
            const nx = uy * vz - uz * vy;
            const ny = uz * vx - ux * vz;
            const nz = ux * vy - uy * vx;
            N[3*a]+=nx; N[3*a+1]+=ny; N[3*a+2]+=nz;
            N[3*b]+=nx; N[3*b+1]+=ny; N[3*b+2]+=nz;
            N[3*c]+=nx; N[3*c+1]+=ny; N[3*c+2]+=nz;
        }
        for (let v = 0; v < N.length; v += 3) {
            const l = Math.hypot(N[v], N[v + 1], N[v + 2]) || 1;
            N[v] /= l; N[v + 1] /= l; N[v + 2] /= l;
        }
        return { ...mesh, normals: N };
    }

    /**
     * Light Laplacian smoothing (n iterations, small step lambda).
     * Preserves boundary topology; used for MC / Surface Nets output.
     */
    static laplacianSmooth(mesh, iterations = 1, lambda = 0.35) {
        if (iterations <= 0) return mesh;
        const P = new Float32Array(mesh.positions);
        const I = mesh.indices;
        const nV = P.length / 3;

        const adj = new Array(nV);
        for (let v = 0; v < nV; v++) adj[v] = new Set();
        for (let t = 0; t < I.length; t += 3) {
            const a = I[t], b = I[t + 1], c = I[t + 2];
            adj[a].add(b); adj[a].add(c);
            adj[b].add(a); adj[b].add(c);
            adj[c].add(a); adj[c].add(b);
        }

        const tmp = new Float32Array(P.length);
        for (let it = 0; it < iterations; it++) {
            for (let v = 0; v < nV; v++) {
                const nbrs = adj[v];
                if (nbrs.size === 0) {
                    tmp[3*v]=P[3*v]; tmp[3*v+1]=P[3*v+1]; tmp[3*v+2]=P[3*v+2];
                    continue;
                }
                let sx = 0, sy = 0, sz = 0;
                for (const n of nbrs) { sx += P[3*n]; sy += P[3*n+1]; sz += P[3*n+2]; }
                const inv = 1 / nbrs.size;
                sx *= inv; sy *= inv; sz *= inv;
                tmp[3*v]   = P[3*v]   + lambda * (sx - P[3*v]);
                tmp[3*v+1] = P[3*v+1] + lambda * (sy - P[3*v+1]);
                tmp[3*v+2] = P[3*v+2] + lambda * (sz - P[3*v+2]);
            }
            P.set(tmp);
        }
        return { ...mesh, positions: P };
    }

    /**
     * Convenience: full cleanup chain used by the pipeline.
     */
    static clean(mesh, { weldEps = 1e-4, smoothIterations = 0, smoothLambda = 0.35 } = {}) {
        let m = MeshCleanup.weldVertices(mesh, weldEps);
        m = MeshCleanup.dropDegenerates(m);
        if (smoothIterations > 0) m = MeshCleanup.laplacianSmooth(m, smoothIterations, smoothLambda);
        m = MeshCleanup.computeNormals(m);
        return m;
    }
}
