// brep/BRep.js
// Boundary Representation (B-Rep) data model.
//
// After surface extraction, the mesh is a raw triangle soup with implicit
// topology. This module lifts it into an editable CAD representation built
// from Vertices, half-edges, Edges, Loops, and Faces — the classic winged /
// half-edge topology used by every CAD kernel (OpenCascade, Parasolid, ACIS).
//
// The B-Rep here is intentionally a scaffold: it captures topology exactly
// (so face/edge/vertex selection and traversal work today) and reserves
// slots for surface fitting, so operations like exact booleans, chamfers,
// fillets, and parametric feature editing can be added without redesigning
// the pipeline. The SDF remains the computational solid representation;
// the B-Rep is the editable CAD representation.

// ---- Basic entities ------------------------------------------------------

export class BVertex {
    constructor(id, x, y, z) {
        this.id = id;
        this.p = [x, y, z];
        this.halfEdge = null;
    }
}

export class BHalfEdge {
    constructor(id) {
        this.id = id;
        this.origin = null;   // BVertex at the tail
        this.twin = null;     // opposite half-edge
        this.next = null;     // next around the same face loop
        this.prev = null;     // previous around the same face loop
        this.face = null;     // BFace owning this loop
        this.edge = null;     // shared undirected BEdge
    }
}

export class BEdge {
    constructor(id, he) {
        this.id = id;
        this.halfEdge = he;   // one of the two directed twins
        this.sharp = false;   // marked sharp by dihedral analysis
        this.curve = null;    // fit curve (line/circle/spline) — reserved
    }
    // Returns the two endpoint vertices.
    endpoints() {
        return [this.halfEdge.origin, this.halfEdge.twin.origin];
    }
    // Dihedral angle in radians between the two adjacent face normals.
    dihedral() {
        const nA = this.halfEdge.face.normal;
        const nB = this.halfEdge.twin.face ? this.halfEdge.twin.face.normal : nA;
        const d = Math.max(-1, Math.min(1,
            nA[0]*nB[0] + nA[1]*nB[1] + nA[2]*nB[2]));
        return Math.acos(d);
    }
}

export class BFace {
    constructor(id) {
        this.id = id;
        this.halfEdge = null;   // an arbitrary half-edge on the outer loop
        this.normal = [0, 0, 1];
        this.planeD = 0;        // n·p - d = 0
        this.surface = null;    // fit surface (plane/cylinder/spline) — reserved
        this.triangles = [];    // originating triangle indices (for rendering)
    }
    // Walk the outer loop yielding vertices in order.
    *vertices() {
        const start = this.halfEdge;
        let he = start;
        do {
            yield he.origin;
            he = he.next;
        } while (he && he !== start);
    }
    *edges() {
        const start = this.halfEdge;
        let he = start;
        do {
            yield he.edge;
            he = he.next;
        } while (he && he !== start);
    }
}

// ---- The B-Rep solid itself ---------------------------------------------

export class BRepSolid {
    constructor() {
        this.vertices  = [];
        this.halfEdges = [];
        this.edges     = [];
        this.faces     = [];
        // Optional: keep a reference to the source SDF for future exact ops.
        this.sourceSDF = null;
        // Optional: keep the underlying triangle mesh for rendering.
        this.mesh = null;
    }

    addVertex(x, y, z) {
        const v = new BVertex(this.vertices.length, x, y, z);
        this.vertices.push(v);
        return v;
    }

    addHalfEdge() {
        const he = new BHalfEdge(this.halfEdges.length);
        this.halfEdges.push(he);
        return he;
    }

    addEdge(he) {
        const e = new BEdge(this.edges.length, he);
        this.edges.push(e);
        return e;
    }

    addFace() {
        const f = new BFace(this.faces.length);
        this.faces.push(f);
        return f;
    }

    // ---- Selection scaffolding for future CAD tooling --------------------
    selectFacesByNormal(normal, tolDeg = 5) {
        const cosTol = Math.cos(tolDeg * Math.PI / 180);
        return this.faces.filter(f => {
            const d = f.normal[0]*normal[0] + f.normal[1]*normal[1] + f.normal[2]*normal[2];
            return d >= cosTol;
        });
    }
    selectSharpEdges(dihedralDeg = 25) {
        const thresh = dihedralDeg * Math.PI / 180;
        return this.edges.filter(e => e.sharp || e.dihedral() > thresh);
    }

    // ---- Placeholders for reserved parametric operations -----------------
    // These stubs define the intended API. Implementations plug into the
    // SDF core (SDFDifference/SDFOffset/SDFShell), guaranteeing that
    // additional solid-modeling operations can be added without redesigning
    // the pipeline.
    booleanExact(_other, _op) { throw new Error('booleanExact: reserved'); }
    chamferEdges(_edges, _size) { throw new Error('chamferEdges: reserved'); }
    filletEdges(_edges, _radius) { throw new Error('filletEdges: reserved'); }
    shellFaces(_faces, _thickness) { throw new Error('shellFaces: reserved'); }
}
