// core/Grid3D.js
// A regular 3D sampling grid over an axis-aligned bounding box in world space.
// Owns the mapping between (i,j,k) integer voxel coordinates and (x,y,z) world coords.
// This is the shared coordinate system consumed by the SDF and all surface-extraction algorithms.

export class Grid3D {
    /**
     * @param {{min:[number,number,number], max:[number,number,number]}} bounds  World-space AABB
     * @param {[number,number,number]} dims   Sample counts along X,Y,Z (inclusive corners)
     */
    constructor(bounds, dims) {
        this.min = bounds.min.slice();
        this.max = bounds.max.slice();
        this.dims = dims.slice();

        this.size = [
            this.max[0] - this.min[0],
            this.max[1] - this.min[1],
            this.max[2] - this.min[2],
        ];

        // Cell size (world units between two adjacent samples).
        this.cell = [
            this.size[0] / (this.dims[0] - 1),
            this.size[1] / (this.dims[1] - 1),
            this.size[2] / (this.dims[2] - 1),
        ];

        this.totalSamples = this.dims[0] * this.dims[1] * this.dims[2];
        this.strideY = this.dims[0];
        this.strideZ = this.dims[0] * this.dims[1];
    }

    index(i, j, k) {
        return i + j * this.strideY + k * this.strideZ;
    }

    worldAt(i, j, k) {
        return [
            this.min[0] + i * this.cell[0],
            this.min[1] + j * this.cell[1],
            this.min[2] + k * this.cell[2],
        ];
    }

    writeWorldAt(i, j, k, out) {
        out[0] = this.min[0] + i * this.cell[0];
        out[1] = this.min[1] + j * this.cell[1];
        out[2] = this.min[2] + k * this.cell[2];
        return out;
    }

    // Uniform cell size approximation (used by finite-difference gradient).
    uniformCell() {
        return (this.cell[0] + this.cell[1] + this.cell[2]) / 3;
    }
}
