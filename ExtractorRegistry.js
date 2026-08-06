// extract/ExtractorRegistry.js
// Pluggable registry of surface-extraction algorithms. Each extractor
// consumes an SDFSampler and returns a raw indexed mesh
// { positions:Float32Array, indices:Uint32Array, normals?:Float32Array }.
// New algorithms can be added without touching the pipeline core.

const _registry = new Map();

export function registerExtractor(id, factory, meta = {}) {
    _registry.set(id, { factory, meta });
}

export function getExtractor(id) {
    const entry = _registry.get(id);
    if (!entry) throw new Error(`Unknown extractor: ${id}`);
    return entry.factory();
}

export function listExtractors() {
    return [..._registry.entries()].map(([id, v]) => ({ id, ...v.meta }));
}
