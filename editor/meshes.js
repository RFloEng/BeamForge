// The vehicle's visible meshes (flexbodies) from the game's own .dae files, untextured.
//
// beamforge.beamng.configure gives the flexbodies of the active parts ({part, mesh, groups, pos, rot,
// scale}), the wheels it builds ({group, hubGroup, centre, ...}) and the nodes ({nodes, rest, groups}).
// A mesh is a named node of a .dae in the vehicle's folder or in vehicles/common (any source). This
// module finds it, reads it with three.js's ColladaLoader and places it:
//   body meshes   modelled in place (BeamNG axes, metres; checked on the small hatchback). Every vertex follows
//                 the displacement of its nearest node among the nodes of the flexbody's groups (a
//                 simple version of the game's binding), so slot offsets, tuning and the user's moves
//                 bend the mesh.
//   wheel meshes  their groups name a wheel's group or hubGroup: modelled around the origin, axle on x,
//                 put on the wheel centre with the row's rot and scale (pos ignored unless it is an
//                 absolute position, as for brake hubs).
//
// Axes: BeamNG (x left, y rear, z up) -> three.js (x, z, y), as v3() in app.js; the swap mirrors, so
// materials are double sided.
import * as THREE from 'three';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';

const names = new Map();     // .dae path -> Set of node names in it
const parsed = new Map();    // .dae path -> Promise of the parsed scene (DAE axes, no up-axis rotation)

export function forgetMeshes() { names.clear(); parsed.clear(); }

// the kind of surface, from the mesh and material names: 'glass', 'dark' (tyres, interior) or 'body';
// app.js picks the material (tinted with the part's colour). Meshes get a plain one until then.
const kindOf = (meshName, matName) => {
  const n = `${meshName} ${matName}`.toLowerCase();
  if (/glass|window|lens/.test(n)) return 'glass';
  if (/tire|tyre|rubber|seal|carpet|interior|dash|seat/.test(n)) return 'dark';
  return 'body';
};
const PLAIN = new THREE.MeshStandardMaterial({ color: 0xc9ced6, side: THREE.DoubleSide });

function scanNames(path, text) {
  const set = new Set();
  for (const m of text.matchAll(/<node\b[^>]*\bname="([^"]+)"/g)) set.add(m[1]);
  names.set(path, set);
  return set;
}

// read(path) is only called when the file is not parsed yet
function parseDae(path, read) {
  if (!parsed.has(path)) {
    parsed.set(path, (async () => {
      const clean = (await read(path)).replace(/<library_images>[\s\S]*?<\/library_images>/, '');   // no texture loading
      const c = new ColladaLoader().parse(clean, '');
      c.scene.rotation.set(0, 0, 0);                    // keep the file's own axes (Z up, as BeamNG)
      c.scene.updateMatrixWorld(true);
      return c.scene;
    })());
  }
  return parsed.get(path);
}

// words of a mesh or file name, for trying the likely .dae files first
const words = (s) => new Set(s.toLowerCase().split(/[^a-z0-9]+/).filter((w) => w.length > 2 && !/^\d+$/.test(w)));

// Find which .dae holds each wanted mesh name: the vehicle's own files first, then vehicles/common, the
// files sharing most words with the wanted names first; stops once all are found. read(path) -> text.
async function locate(wanted, model, daePaths, read, progress) {
  const where = new Map(), left = new Set(wanted);
  const own = daePaths.filter((p) => p.toLowerCase().startsWith(`vehicles/${model.toLowerCase()}/`));
  const common = daePaths.filter((p) => /^vehicles\/common\//i.test(p));
  const take = (p, set) => { for (const n of [...left]) if (set.has(n)) { where.set(n, p); left.delete(n); } };
  for (const p of names.keys()) if (daePaths.includes(p)) take(p, names.get(p));     // already scanned
  const order = [...own, ...common.map((p) => {
    const w = words(p.split('/').slice(2).join(' '));
    let score = 0;
    for (const n of left) for (const x of words(n)) if (w.has(x)) score++;
    return [p, score];
  }).sort((a, b) => b[1] - a[1]).map((x) => x[0])];
  let i = 0;
  for (const p of order) {
    if (!left.size) break;
    i++;
    if (names.has(p)) continue;
    progress(`Finding meshes: ${wanted.length - left.size} of ${wanted.length} (${p.split('/').pop()}, file ${i})`);
    take(p, scanNames(p, await read(p)));
  }
  return { where, missing: [...left] };
}

// BeamNG Euler rotation in degrees (x, y, z), applied x then y then z (to confirm in-game)
const rotation = (r) => new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler(
  THREE.MathUtils.degToRad(r[0]), THREE.MathUtils.degToRad(r[1]), THREE.MathUtils.degToRad(r[2]), 'XYZ'));

// Build the meshes of a configured vehicle. Returns { group, update(veh), missing: [names], count }.
export async function buildMeshes(veh, daePaths, readFile, progress = () => {}) {
  const texts = new Map();                              // each file read once per build (scan, then parse)
  const read = (p) => { if (!texts.has(p)) texts.set(p, readFile(p)); return texts.get(p); };
  const fbs = veh.flexbodies;
  const { where, missing } = await locate([...new Set(fbs.map((f) => f.mesh))], veh.model, daePaths, read, progress);
  const group = new THREE.Group();
  const items = [];
  const wheelOf = (f) => veh.wheels.find((w) => f.groups.includes(w.group) || f.groups.includes(w.hubGroup));
  const g = veh.geometry;
  let done = 0;
  for (const f of fbs) {
    const path = where.get(f.mesh);
    if (!path) continue;
    progress(`Placing meshes: ${++done} of ${fbs.length}`);
    const scene = await parseDae(path, read);
    const obj = scene.getObjectByName(f.mesh);
    if (!obj) continue;
    const wheel = wheelOf(f);
    const absolute = !wheel || Math.abs(f.pos[1]) > 0.01 || Math.abs(f.pos[2]) > 0.01;
    // flexbody transform in BeamNG space: translate(pos) * rot * scale (a wheel's centre is added at update)
    const T = new THREE.Matrix4().makeTranslation(...(absolute ? f.pos : [0, 0, 0]))
      .multiply(rotation(f.rot)).multiply(new THREE.Matrix4().makeScale(...f.scale));
    // the nodes a body mesh follows: its groups, else its part's nodes
    let follow = [];
    if (absolute) {
      follow = Object.keys(g.nodes).filter((n) => g.groups[n] && g.groups[n].some((x) => f.groups.includes(x)));
      if (!follow.length) follow = Object.keys(g.nodes).filter((n) => g.parts[n] === f.part);
    }
    obj.traverse((m) => {
      if (!m.isMesh) return;
      const geo = m.geometry.clone();
      geo.applyMatrix4(new THREE.Matrix4().multiplyMatrices(T, m.matrixWorld));
      const pos = geo.attributes.position;
      const base = Float32Array.from(pos.array);            // BeamNG space, before any node displacement
      let bind = null;
      if (absolute && follow.length) {                      // nearest node (rest position) per vertex
        const rest = follow.map((n) => g.rest[n]);
        bind = new Int32Array(pos.count);
        for (let v = 0; v < pos.count; v++) {
          const x = base[3 * v], y = base[3 * v + 1], z = base[3 * v + 2];
          let best = 0, bd = Infinity;
          for (let k = 0; k < rest.length; k++) {
            const r = rest[k], d = (r[0] - x) ** 2 + (r[1] - y) ** 2 + (r[2] - z) ** 2;
            if (d < bd) { bd = d; best = k; }
          }
          bind[v] = best;
        }
      }
      geo.deleteAttribute('normal');
      const mats = Array.isArray(m.material) ? m.material : [m.material];
      const kinds = mats.map((x) => kindOf(f.mesh, x.name));
      const mesh = new THREE.Mesh(geo, kinds.length > 1 ? kinds.map(() => PLAIN) : PLAIN);
      mesh.userData = { part: f.part, mesh: f.mesh, kinds };
      mesh.frustumCulled = false;
      group.add(mesh);
      items.push({ mesh, base, bind, follow, wheel: absolute ? null : wheel.name });
    });
  }
  // place every mesh for the vehicle's current nodes and wheels (after a move, tuning or slot change)
  function update(v) {
    const nodes = v.geometry.nodes, rest = v.geometry.rest;
    for (const it of items) {
      const out = it.mesh.geometry.attributes.position.array, b = it.base;
      let dx = 0, dy = 0, dz = 0;
      if (it.wheel) {
        const w = v.wheels.find((x) => x.name === it.wheel);
        if (w) [dx, dy, dz] = w.centre;
      }
      const disp = it.bind ? it.follow.map((n) => nodes[n] && rest[n] ? [nodes[n][0] - rest[n][0], nodes[n][1] - rest[n][1], nodes[n][2] - rest[n][2]] : [0, 0, 0]) : null;
      for (let i = 0, n = b.length / 3; i < n; i++) {
        let x = b[3 * i] + dx, y = b[3 * i + 1] + dy, z = b[3 * i + 2] + dz;
        if (disp) { const d = disp[it.bind[i]]; x += d[0]; y += d[1]; z += d[2]; }
        out[3 * i] = x; out[3 * i + 1] = z; out[3 * i + 2] = y;          // BeamNG -> three.js
      }
      it.mesh.geometry.attributes.position.needsUpdate = true;
      it.mesh.geometry.computeVertexNormals();
      it.mesh.geometry.computeBoundingSphere();
    }
  }
  update(veh);
  return { group, update, missing, count: items.length };
}
