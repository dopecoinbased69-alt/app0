// brep/BRepBuilder.js
// Builds a BRepSolid from a cleaned, indexed, watertight triangle mesh.
//
// Algorithm outline:
//   1. Ingest positions -> BVertex list.
//   2. For every triangle:
//        - create a temporary BFace_tri with three half-edges,
//        - link next/prev around the triangle loop,
//        - register directed edges (a,b) so the reverse (b,a) can be paired.
//   3. Pair opposite half-edges into BEdge records.
//   4. Group coplanar adjacent triangles (by normal + plane offset) into
//      real BFace regions via a flood fill. This is what makes face
//      selection meaningful ("select the top face") instead of "select
//      triangle #4712".
//   5. Mark sharp edges by dihedral angle.
//
// The result carries both the polygonal representation (triangles retained
// on each face for rendering) and the topological one (vertices/edges/faces
// with adjacency), giving future CAD ops a real substrate to operate on.

import { BRepSolid } from './BRep.js';

function triNormal(P, a, b, c, out = [0, 0, 0]) {
    const ax = P[3*a], ay = P[3*a+1], az = P[3*a+2];
    const bx = P[3*b], by = P[3*b+1], bz = P[3*b+2];
    const cx = P[3*c], cy = P[3*c+1], cz = P[3*c+2];
    const ux = bx - ax, uy = by - ay, uz = bz - az;
    const vx = cx - ax, vy = cy - ay, vz = cz - az;
    let nx = uy * vz - uz * vy;
    let ny = uz * vx - ux * vz;
    let nz = ux * vy - uy * vx;
    const l = Math.hypot(nx, ny, nz) || 1;
    out[0] = nx / l; out[1] = ny / l; out[2] = nz / l;
    return out;
}

export class BRepBuilder {
    /**
     * @param {{positions:Float32Array, indices:Uint32Array}} mesh cleaned mesh
     * @param {object} opts
     *   coplanarNormalTolDeg: max angle between face normals to merge (default 3°)
     *   coplanarOffsetTol:    max |plane offset| difference to merge (default 1e-3 world units)
     *   sharpDihedralDeg:     dihedral threshold for marking sharp edges (default 25°)
     */
    static build(mesh, opts = {}) {
        const P = mesh.positions;
        const I = mesh.indices;
        const nT = I.length / 3;

        const normalTol = Math.cos((opts.coplanarNormalTolDeg ?? 3) * Math.PI / 180);
        const offsetTol = opts.coplanarOffsetTol ?? 1e-3;
        const sharpTol  = (opts.sharpDihedralDeg ?? 25) * Math.PI / 180;

        const solid = new BRepSolid();
        solid.mesh = mesh;

        // Vertices
        for (let v = 0; v < P.length; v += 3) solid.addVertex(P[v], P[v + 1], P[v + 2]);

        // Per-triangle: half-edges + normals + plane offset
        const triNormals = new Float32Array(nT * 3);
        const triPlaneD  = new Float32Array(nT);
        const triHE      = new Array(nT * 3); // three half-edges per triangle
        const dirEdge    = new Map();         // "a_b" -> half-edge id

        const n = [0, 0, 0];
        for (let t = 0; t < nT; t++) {
            const a = I[3*t], b = I[3*t + 1], c = I[3*t + 2];
            triNormal(P, a, b, c, n);
            triNormals[3*t]=n[0]; triNormals[3*t+1]=n[1]; triNormals[3*t+2]=n[2];
            triPlaneD[t] = n[0]*P[3*a] + n[1]*P[3*a+1] + n[2]*P[3*a+2];

            const heAB = solid.addHalfEdge();
            const heBC = solid.addHalfEdge();
            const heCA = solid.addHalfEdge();
            heAB.origin = solid.vertices[a]; solid.vertices[a].halfEdge ??= heAB;
            heBC.origin = solid.vertices[b]; solid.vertices[b].halfEdge ??= heBC;
            heCA.origin = solid.vertices[c]; solid.vertices[c].halfEdge ??= heCA;
            heAB.next = heBC; heBC.next = heCA; heCA.next = heAB;
            heAB.prev = heCA; heBC.prev = heAB; heCA.prev = heBC;

            triHE[3*t]     = heAB.id;
            triHE[3*t + 1] = heBC.id;
            triHE[3*t + 2] = heCA.id;

            dirEdge.set(`${a}_${b}`, heAB.id);
            dirEdge.set(`${b}_${c}`, heBC.id);
            dirEdge.set(`${c}_${a}`, heCA.id);
        }

        // Pair half-edge twins and create shared BEdge records.
        const paired = new Uint8Array(solid.halfEdges.length);
        for (let t = 0; t < nT; t++) {
            const a = I[3*t], b = I[3*t + 1], c = I[3*t + 2];
            const pairs = [[a, b], [b, c], [c, a]];
            for (let p = 0; p < 3; p++) {
                const heId = triHE[3*t + p];
                if (paired[heId]) continue;
                const [u, v] = pairs[p];
                const twinId = dirEdge.get(`${v}_${u}`);
                if (twinId === undefined) continue; // boundary edge — rare after cleanup
                const he = solid.halfEdges[heId];
                const twin = solid.halfEdges[twinId];
                he.twin = twin; twin.twin = he;
                const edge = solid.addEdge(he);
                he.edge = edge; twin.edge = edge;
                paired[heId] = 1; paired[twinId] = 1;
            }
        }

        // Flood-fill triangles into coplanar BFaces.
        // Two triangles merge if:
        //   - they share an edge,
        //   - normals agree within normalTol,
        //   - plane offset (d = n·p) agrees within offsetTol.
        const triFace = new Int32Array(nT).fill(-1);
        for (let seed = 0; seed < nT; seed++) {
            if (triFace[seed] !== -1) continue;
            const face = solid.addFace();
            const seedN = [triNormals[3*seed], triNormals[3*seed+1], triNormals[3*seed+2]];
            const seedD = triPlaneD[seed];
            face.normal = seedN.slice();
            face.planeD = seedD;

            const stack = [seed];
            while (stack.length) {
                const t = stack.pop();
                if (triFace[t] !== -1) continue;
                triFace[t] = face.id;
                face.triangles.push(t);

                // Traverse 3 half-edges of this triangle to reach neighbors.
                for (let p = 0; p < 3; p++) {
                    const he = solid.halfEdges[triHE[3*t + p]];
                    he.face = face;
                    if (!face.halfEdge) face.halfEdge = he;
                    const twin = he.twin;
                    if (!twin) continue;
                    // Twin belongs to triangle floor(twin.id / 3) in our layout
                    // because we allocated half-edges sequentially by triangle.
                    const tn = Math.floor(twin.id / 3);
                    if (triFace[tn] !== -1) continue;
                    const dn = seedN[0]*triNormals[3*tn]
                             + seedN[1]*triNormals[3*tn+1]
                             + seedN[2]*triNormals[3*tn+2];
                    if (dn < normalTol) continue;
                    if (Math.abs(triPlaneD[tn] - seedD) > offsetTol) continue;
                    stack.push(tn);
                }
            }
        }

        // Any half-edge not yet assigned to a face (isolated triangle) — attach.
        for (let t = 0; t < nT; t++) {
            const face = solid.faces[triFace[t]];
            for (let p = 0; p < 3; p++) {
                const he = solid.halfEdges[triHE[3*t + p]];
                he.face = face;
            }
        }

        // Mark sharp edges by dihedral angle between the (already merged) faces
        // the two half-edges belong to. Cross-face edges get an angle test;
        // intra-face edges are implicitly smooth.
        for (const e of solid.edges) {
            const fa = e.halfEdge.face;
            const fb = e.halfEdge.twin ? e.halfEdge.twin.face : null;
            if (!fa || !fb || fa === fb) { e.sharp = false; continue; }
            const d = Math.max(-1, Math.min(1,
                fa.normal[0]*fb.normal[0] + fa.normal[1]*fb.normal[1] + fa.normal[2]*fb.normal[2]));
            e.sharp = Math.acos(d) > sharpTol;
        }

        return solid;
    }
}
