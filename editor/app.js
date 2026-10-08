// BeamForge editor prototype: three.js view + Pyodide running the beamforge Python package.
// JavaScript reads files, draws and handles the panels; the vehicle and SVJ logic is in Python.
//
// Map of this file (sections in order):
//   Python engine     boot() loads Pyodide and FILES, imports beamforge.beamng (vehpy) and beamforge.svj (svjpy)
//   3D                scene, groups, camera; drawScene() rebuilds the vehicle and SVJ hardpoint groups
//   base vehicle      BeamNG folders (install, user folder, mods; remembered), vehicle list, parts and tuning
//   SVJ               import (.svj.json + meshes, or .zip bundle), glTF meshes, hardpoints, comparison panel
//   panels            inspector, budget bar, messages, timing
//   workspaces        Base vehicle, SVJ, Sketch, Suspension, Checks, Assembly: one view, the panels of each use
//   project           the work saved as a .beamforge.json (and autosaved in the browser), opened again later
//
// State (module globals):
//   folders, cat     the BeamNG folders (library.js) and the vehicle catalog (vehpy.catalog)
//   veh, vehEdit     the configured vehicle (vehpy.configure) and the user's slot, tuning and move edits
//   pick             the selected node, beam or part, moved with numeric x / y / z in the Move panel
//   partColor, hiddenParts, lockedParts   per part: its colour, and whether it is hidden or locked (cannot be picked)
//   meshes           the vehicle's flexbody meshes (meshes.js), rebuilt when the parts change, updated on moves
//   svjDoc           the last imported SVJ bundle (svjpy.load_bundle): document, meshes, bindings, notes
//
// Placement: the base vehicle stays where its jbeam puts it. An SVJ (origin at the front-axle centre on
// the ground) is placed on the vehicle's front axle line and ground height (veh.measure.front_axle_y,
// ground_z), so its meshes and hardpoints sit on the vehicle they are compared with.
//
// Axes: BeamNG metres (X left, Y rear, Z up); v3() maps them to three.js (Y up).
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { buildMeshes, forgetMeshes } from './meshes.js';
import { ZipWriter, BlobWriter, TextReader, BlobReader } from 'zipjs';
import { TEXT_FILE, canRemember, handleDir, listDir, zipSource, readFolder, rememberedHandles, rememberHandles, access } from './library.js';

// repo files copied into Pyodide's file system under /bf (add new Python modules here)
const FILES = ['beamforge/__init__.py', 'beamforge/jbeam.py', 'beamforge/beamng.py', 'beamforge/gltf.py', 'beamforge/svj.py',
  'beamforge/fit.py', 'beamforge/suspension.py', 'beamforge/export.py', 'beamforge/dae.py', 'beamforge/values.py',
  'beamforge/rigidity.py', 'beamforge/kinematics.py', 'beamforge/roles.py', 'beamforge/convert.py',
  'beamforge/archetype.py', 'beamforge/steering.py', 'beamforge/structure.py', 'beamforge/skeleton.py', 'beamforge/tubes.py', 'beamforge/powertrain.py', 'beamforge/donors.py', 'beamforge/scratch.py', 'beamforge/tyres.py', 'beamforge/aero.py', 'beamforge/components.py'];
const REPO = new URL('../', import.meta.url);

const $ = (id) => document.getElementById(id);
const fmt = (x, d = 1) => (x === null || x === undefined || Number.isNaN(x)) ? '–'
  : x.toLocaleString('en', { minimumFractionDigits: d, maximumFractionDigits: d });
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
// last line of a Python traceback, without the exception class
const pyError = (e) => String(e.message || e).trim().split('\n').pop().replace(/^\w+Error: /, '');

let py, vehpy, svjpy, fitpy, suspy, exppy, valpy, skpy, strpy, ptpy, donpy, scpy, typy, aepy, copy_, rigpy;
const timing = {};         // ms per step, shown in the footer by drawTiming

// ---------- Python engine ----------
async function boot() {
  const t0 = performance.now();
  py = await loadPyodide();
  timing.pyodide = performance.now() - t0;
  const t1 = performance.now();
  for (const f of FILES) {
    const res = await fetch(new URL(f, REPO), { cache: 'no-store' });
    if (!res.ok) throw new Error(`cannot load ${f} (${res.status}); serve the repo root over http`);
    py.FS.mkdirTree('/bf/' + f.slice(0, f.lastIndexOf('/')));
    py.FS.writeFile('/bf/' + f, await res.text());
  }
  py.runPython("import sys; sys.path.insert(0, '/bf')");
  vehpy = py.pyimport('beamforge.beamng');
  svjpy = py.pyimport('beamforge.svj');
  fitpy = py.pyimport('beamforge.fit');
  suspy = py.pyimport('beamforge.suspension');
  exppy = py.pyimport('beamforge.export');
  valpy = py.pyimport('beamforge.values');
  skpy = py.pyimport('beamforge.skeleton');
  strpy = py.pyimport('beamforge.structure');
  ptpy = py.pyimport('beamforge.powertrain');
  donpy = py.pyimport('beamforge.donors');
  scpy = py.pyimport('beamforge.scratch');
  typy = py.pyimport('beamforge.tyres');
  aepy = py.pyimport('beamforge.aero');
  rigpy = py.pyimport('beamforge.rigidity');
  copy_ = py.pyimport('beamforge.components');
  timing.files = performance.now() - t1;
  $('loading').remove();
  await restoreProject();
  redraw();
  fitCamera();
  await restoreFolders();
}

// ---------- 3D ----------
const view = $('view');
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(window.devicePixelRatio);
view.appendChild(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(40, 1, 0.05, 200);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f99, 1.1));
const sun = new THREE.DirectionalLight(0xffffff, 1.4);
sun.position.set(3, 5, 2);
scene.add(sun);
scene.add(new THREE.GridHelper(10, 50, 0xb8c0c8, 0xdde2e7));
// vehG: the base vehicle's nodes and beams (drawVehicle); meshG: SVJ glTF meshes (loadSvjMeshes);
// hpG: SVJ hardpoints (drawHardpoints)
// suspG: the SVJ suspension linkage at the travel slider (drawSuspension)
// skelG: a STEP skeleton built into nodes and beams (drawSkeleton)
const vehG = new THREE.Group(), meshG = new THREE.Group(), hpG = new THREE.Group(), bodyG = new THREE.Group(), suspG = new THREE.Group();
const skelG = new THREE.Group(), compG = new THREE.Group();   // compG: the components' masses (drawComponents)
scene.add(vehG, meshG, hpG, bodyG, suspG, skelG, compG);

// BeamNG axes (X left, Y rear, Z up) -> three.js (Y up)
const v3 = (p) => new THREE.Vector3(p[0], p[2], p[1]);

function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(view);

// aim the camera at the vehicle (or the SVJ meshes, or the origin)
function fitCamera() {
  const target = vehG.children.length ? vehG : meshG.children.length ? meshG : null;
  const box = target ? new THREE.Box3().setFromObject(target) : new THREE.Box3();
  const c = box.isEmpty() ? new THREE.Vector3(0, 0.6, 0) : box.getCenter(new THREE.Vector3());
  const size = box.isEmpty() ? 4 : box.getSize(new THREE.Vector3()).length();
  controls.target.copy(c);
  camera.position.set(c.x + 0.8 * size, c.y + 0.5 * size, c.z - 0.75 * size);
  controls.update();
}

(function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
})();

function drawScene() { drawVehicle(); placeSvj(); drawHardpoints(); drawSuspension(); drawSkeleton(); drawComponents(); }
function redraw() { drawScene(); drawSpaces(); drawVehList(); drawInspector(); drawBudget(); showIssues(issues()); drawTiming(); autosave(); }

// ---------- base vehicle: any BeamNG vehicle from the user's own install or mods ----------
// Nothing from the game ships with BeamForge. The user adds their folders once (library.js): the game
// install, the user folder (mods, unpacked mods and its own vehicles/), single mod folders or zips.
// Python resolves them into one file system as the game does (user folder over mods over the install),
// and only the text files (.jbeam, .pc, .json) are read, on demand, and handed to beamforge.beamng.
// It works like the game's Vehicle Config menu: a parts tree with the alternatives for each slot,
// tuning sliders by category, and a .pc configuration to save.
//
//   folders     picked folders and zips ({ name, role, sources, notes, handle? }), see library.js
//   resolved    vehpy.resolve over every source: { files: {path: source name}, rank, shadowed }
//   pending     remembered folder handles still waiting for the user to confirm access
let folders = [], resolved = null, pending = [];
let cat = null, veh = null, vehError = null, vehBusy = '';
// the user's edits of the base vehicle: slot choices, tuning values, and moves ({parts|nodes: {name: [dx, dy, dz]}})
// fit: the node moves of the last fit to the SVJ (fit.py), kept apart from the hand moves so a new fit
// replaces it; fitReport: its report rows, hardpoint mapping, placement and notes; fitOverrides: the
// hardpoint-to-node ties the user set ({"FL:lower_ball_joint": node})
const freshEdit = (model, config) => ({ model, config: config || null, parts: {}, vars: {}, moves: { parts: {}, nodes: {} },
  fit: {}, fitReport: null, fitOverrides: {}, beams: [] });
// the moves Python applies: the hand moves on top of the fit
function allMoves() {
  const nodes = {};
  for (const [n, d] of Object.entries(vehEdit.fit || {})) nodes[n] = [...d];
  for (const [n, d] of Object.entries(vehEdit.moves.nodes)) nodes[n] = nodes[n] ? nodes[n].map((x, i) => x + d[i]) : [...d];
  return { parts: vehEdit.moves.parts, nodes };
}
let vehEdit = freshEdit(null, null);
// per part of the open vehicle: a colour (kept while the vehicle is open), hidden, locked
let partColor = {}, hiddenParts = new Set(), lockedParts = new Set(), colorCount = 0;
const editable = (part) => !hiddenParts.has(part) && !lockedParts.has(part);
function colorParts() {                              // new parts take the next hue (golden-ratio steps)
  const walk = (n) => {
    if (n.part && !(n.part in partColor)) partColor[n.part] = new THREE.Color().setHSL((colorCount++ * 0.618034) % 1, 0.62, 0.52);
    n.children.forEach(walk);
  };
  walk(veh.tree);
}
const colorOf = (part) => $('partcolors').checked && partColor[part] ? partColor[part] : null;
const hexOf = (part) => '#' + (partColor[part] || new THREE.Color(0x8c959f)).getHexString();

let pick = null;           // { kind: 'node', id }, { kind: 'beam', a, b } (its two node ids) or { kind: 'part', name }

// index of the picked beam in veh.geometry.beams (-1 when it is gone)
const beamIndex = (g, b) => g.beams.findIndex(([a, c]) => (a === b.a && c === b.b) || (a === b.b && c === b.a));
// the part the pick belongs to, and whether it may be moved (not hidden, not locked)
function pickPart() {
  const g = veh.geometry;
  return pick.kind === 'part' ? pick.name : pick.kind === 'node' ? g.parts[pick.id] : g.beam_parts[beamIndex(g, pick)];
}
const pickEditable = () => editable(pickPart());
function pickValid() {
  const g = veh.geometry;
  if (pick.kind === 'node') return pick.id in g.nodes;
  if (pick.kind === 'beam') return beamIndex(g, pick) >= 0;
  return Object.values(g.parts).includes(pick.name);
}
let vehFilter = '', commonLoaded = false;

const allSources = () => folders.flatMap((f) => f.sources);
const sourceByName = () => Object.fromEntries(allSources().map((s) => [s.name, s]));
const inactive = (dbText) => new Set(JSON.parse(vehpy.inactive_mods(dbText)));

// read the resolved files that pass keep(path): {path: text}, and their source ranks
async function readResolved(keep, label) {
  const src = sourceByName(), texts = {}, ranks = {};
  const paths = Object.keys(resolved.files).filter(keep);
  let i = 0;
  for (const p of paths) {
    if (label && ++i % 50 === 0) { vehBusy = `${label}: ${i} of ${paths.length}`; drawVehList(); }
    const name = resolved.files[p];
    texts[p] = await src[name].files[p]();
    ranks[p] = resolved.rank[name];
  }
  return { texts, ranks };
}

// a folder (or zips) was added: resolve every source again, forget the held files, rebuild the vehicle list
async function libraryChanged() {
  cat = null; veh = null; vehError = null; commonLoaded = false;
  vehpy.reset();
  forgetMeshes(); meshes = null; meshKey = ''; bodyG.clear();
  resolved = JSON.parse(vehpy.resolve(JSON.stringify(allSources().map((s) => ({ name: s.name, kind: s.kind, paths: Object.keys(s.files) })))));
  await scanLibrary();
  await applyPendingBase();
  redraw();
}

// add a picked folder (showDirectoryPicker handle, or a webkitdirectory listing); replaces a folder of the same role
async function addFolder(dir, handle) {
  try {
    vehBusy = `Looking at ${dir.name}…`; drawVehList();
    const f = await readFolder(dir, inactive, (t) => { vehBusy = t; drawVehList(); });
    vehBusy = '';
    if (!f) {
      showIssues([{ level: 'WARN', rule: 'library', message: `${dir.name}: not a BeamNG install, user folder or mod folder (no content/vehicles/common.zip, mods/ or vehicles/).` }]);
      drawVehList();
      return;
    }
    f.handle = handle || null;
    folders = folders.filter((x) => !(x.role === f.role && (f.role !== 'mod' || x.name === f.name)));
    folders.push(f);
    if (handle) await rememberHandles(folders.map((x) => x.handle).filter(Boolean));
    await libraryChanged();
  } catch (err) { vehBusy = ''; vehError = pyError(err); redraw(); }
}

// A page cannot open a picker at a path it names. Each picker has its own id, so the browser reopens it where
// that picker was last used, and it starts in the folder already added for that purpose when there is one.
// kind: 'game' (the install) or 'mods' (the user folder or a mod folder)
async function pickFolder(kind) {
  const prev = folders.find((f) => f.handle && (kind === 'game' ? f.role === 'game' : f.role !== 'game'));
  let h;
  try {
    h = await window.showDirectoryPicker({ id: 'beamng-' + kind, mode: 'read', ...(prev ? { startIn: prev.handle } : {}) });
  } catch (e) { return; }   // cancelled, or a folder Chrome blocks
  await addFolder(handleDir(h), h);
}

// single zips, with their own remembered place (showOpenFilePicker), else the file input
async function pickZips() {
  if (!window.showOpenFilePicker) { $('bngzips').click(); return; }
  let hs;
  try {
    hs = await window.showOpenFilePicker({ id: 'beamng-zips', multiple: true, types: [{ description: 'Vehicle or mod zips', accept: { 'application/zip': ['.zip'] } }] });
  } catch (e) { return; }
  await addZips(await Promise.all(hs.map((h) => h.getFile())));
}

// remembered folders: open the ones still granted; the others wait for a click (reopenFolders)
async function restoreFolders() {
  for (const h of await rememberedHandles()) {
    if (await access(h).catch(() => 'denied') === 'granted') await addFolder(handleDir(h), h);
    else pending.push(h);
  }
  drawVehList();
}

async function reopenFolders() {
  const hs = pending;
  pending = [];
  for (const h of hs) if (await access(h, true).catch(() => 'denied') === 'granted') await addFolder(handleDir(h), h);
  drawVehList();
}

async function forgetFolders() {
  folders = []; pending = []; cat = null; veh = null; resolved = null; commonLoaded = false;
  vehpy.reset();
  await rememberHandles([]);
  redraw();
}

// the vehicle list: info.json and the configs of every vehicle folder, over every source
async function scanLibrary() {
  const isInfo = (p) => /^vehicles\/[^/]+\/info[^/]*\.json$/i.test(p);
  const entries = {}, src = sourceByName();
  const { texts } = await readResolved(isInfo, 'Reading the vehicle list');
  for (const p of Object.keys(resolved.files)) {
    if (!isInfo(p) && !/^vehicles\/[^/]+\/[^/]+\.pc$/i.test(p)) continue;
    const m = p.split('/')[1], base = p.split('/').pop();
    if (m.toLowerCase() === 'common') continue;
    const e = entries[m] || (entries[m] = { info: null, configs: {}, source: '' });
    if (base.toLowerCase() === 'info.json') { e.info = texts[p]; e.source = src[resolved.files[p]].kind; }
    else if (/\.pc$/i.test(base)) { const c = base.replace(/\.pc$/i, ''); if (!(c in e.configs)) e.configs[c] = null; }
    else e.configs[base.replace(/^info_/i, '').replace(/\.json$/i, '')] = texts[p];
  }
  // a folder with configs but no parts (saved .pc files of a vehicle that is not installed) is not a vehicle
  const withParts = new Set(Object.keys(resolved.files).filter((p) => /\.jbeam$/i.test(p)).map((p) => p.split('/')[1]));
  for (const m of Object.keys(entries)) if (!withParts.has(m)) delete entries[m];
  cat = JSON.parse(vehpy.catalog(JSON.stringify(entries)));
  vehBusy = '';
}

// load one vehicle's text files (and vehicles/common of every source the first time) into Python, then configure it
async function openVehicle(model, config) {
  const v = cat.find((x) => x.model === model);
  try {
    vehError = null;
    if (!commonLoaded) {
      const { texts, ranks } = await readResolved((p) => /^vehicles\/common\/.*\.jbeam$/i.test(p), 'Reading the shared parts (vehicles/common), once per library');
      vehBusy = `Parsing ${Object.keys(texts).length} shared part files…`; drawVehList();
      await new Promise((r) => setTimeout(r, 30));
      vehpy.add_files(JSON.stringify(texts), JSON.stringify(ranks));
      commonLoaded = true;
    }
    if (!JSON.parse(vehpy.held_models()).includes(model)) {
      vehBusy = `Reading ${v ? v.name : model}…`; drawVehList();
      const { texts, ranks } = await readResolved((p) => p.startsWith(`vehicles/${model}/`) && TEXT_FILE.test(p));
      await new Promise((r) => setTimeout(r, 30));
      vehpy.add_files(JSON.stringify(texts), JSON.stringify(ranks));
    }
    vehEdit = freshEdit(model, config);
    pick = null;
    partColor = {}; hiddenParts = new Set(); lockedParts = new Set(); colorCount = 0;
    exportForm = { id: '', name: '', brand: '', choices: {}, svj: true, replace: true, attach: {} }; exportNote = '';
    vehBusy = '';
    configureVehicle();
    fitCamera();
  } catch (err) { vehBusy = ''; vehError = pyError(err); redraw(); }
}

// rebuild the configured vehicle from the held files and the user's edits (fast: no parsing)
function configureVehicle() {
  const t = performance.now();
  try {
    veh = JSON.parse(vehpy.configure(vehEdit.model, vehEdit.config, JSON.stringify(vehEdit.parts), JSON.stringify(vehEdit.vars),
      JSON.stringify(allMoves())));
    if (pick && !pickValid()) pick = null;
    vehEdit.config = veh.config;
    vehError = null;
    if (skel) buildSkeleton();                        // placed on the vehicle's front axle and ground
    colorParts();
    syncMeshes();
  } catch (err) {
    vehError = pyError(err);
    if (veh && veh.model !== vehEdit.model) veh = null;     // do not leave the previous vehicle on screen
  }
  timing.configure = performance.now() - t;
  redraw();
}

// the vehicle's node-and-beam structure: beams as one line set, nodes as points. The picked part's nodes
// are drawn orange, the picked node (or the two nodes of the picked beam) as orange dots and the picked
// beam as an orange line, on top of everything; nodeIds lists the node ids in drawing order.
let nodeIds = [];
function drawVehicle() {
  vehG.clear();
  if (!veh) return;
  const g = veh.geometry, pos = [], col = [];
  const grey = new THREE.Color(0x8c959f), blue = new THREE.Color(0x2f6fdf);
  nodeIds = Object.keys(g.nodes).filter((n) => !hiddenParts.has(g.parts[n]));
  g.beams.forEach(([a, b], i) => {
    if (hiddenParts.has(g.beam_parts[i])) return;
    const c = colorOf(g.beam_parts[i]) || grey;
    pos.push(...v3(g.nodes[a]).toArray(), ...v3(g.nodes[b]).toArray());
    col.push(c.r, c.g, c.b, c.r, c.g, c.b);
  });
  const lines = new THREE.BufferGeometry();
  lines.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  lines.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
  vehG.add(new THREE.LineSegments(lines, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.7 })));
  const pts = new THREE.BufferGeometry();
  pts.setAttribute('position', new THREE.Float32BufferAttribute(nodeIds.flatMap((n) => v3(g.nodes[n]).toArray()), 3));
  pts.setAttribute('color', new THREE.Float32BufferAttribute(nodeIds.flatMap((n) => (colorOf(g.parts[n]) || blue).toArray()), 3));
  vehG.add(new THREE.Points(pts, new THREE.PointsMaterial({ vertexColors: true, size: 0.025 })));
  // the wheels the game builds (not in the jbeam): two circles per tyre, at its sides
  for (const w of veh.wheels) {
    if (!w.radius) continue;
    const ax = v3(w.axis).normalize(), c = v3(w.centre), half = (w.width || 0.2) / 2;
    const u = new THREE.Vector3().crossVectors(ax, Math.abs(ax.y) < 0.9 ? new THREE.Vector3(0, 1, 0) : new THREE.Vector3(1, 0, 0)).normalize();
    const v = new THREE.Vector3().crossVectors(ax, u);
    for (const side of [-half, half]) {
      const ring = [];
      for (let k = 0; k <= 32; k++) {
        const t = (k / 32) * Math.PI * 2;
        ring.push(c.clone().addScaledVector(ax, side).addScaledVector(u, Math.cos(t) * w.radius).addScaledVector(v, Math.sin(t) * w.radius));
      }
      vehG.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(ring), new THREE.LineBasicMaterial({ color: 0x55595f })));
    }
  }
  const part = pick && (pick.kind === 'part' ? pick.name : pick.kind === 'node' ? g.parts[pick.id] : g.beam_parts[beamIndex(g, pick)]);
  if (part) {
    const hp = new THREE.BufferGeometry();
    hp.setAttribute('position', new THREE.Float32BufferAttribute(nodeIds.filter((n) => g.parts[n] === part).flatMap((n) => v3(g.nodes[n]).toArray()), 3));
    vehG.add(new THREE.Points(hp, new THREE.PointsMaterial({ color: 0xe0782a, size: pick.kind === 'part' ? 0.05 : 0.035, depthTest: false })));
  }
  const dots = !pick ? [] : pick.kind === 'node' ? [pick.id] : pick.kind === 'beam' ? [pick.a, pick.b] : [];
  for (const id of dots) {
    const dot = new THREE.Mesh(new THREE.SphereGeometry(pick.kind === 'node' ? 0.03 : 0.02, 16, 12), new THREE.MeshBasicMaterial({ color: 0xff5a1f, depthTest: false }));
    dot.position.copy(v3(g.nodes[id]));
    dot.renderOrder = 2;
    vehG.add(dot);
  }
  if (pick && pick.kind === 'beam') {
    const bl = new THREE.BufferGeometry().setFromPoints([v3(g.nodes[pick.a]), v3(g.nodes[pick.b])]);
    const line = new THREE.Line(bl, new THREE.LineBasicMaterial({ color: 0xff5a1f, depthTest: false }));
    line.renderOrder = 2;
    vehG.add(line);
  }
  // beams added by hand (green), and the node a new beam starts from
  const added = (vehEdit.beams || []).filter((b) => g.nodes[b.a] && g.nodes[b.b]);
  if (added.length) {
    const ab = new THREE.BufferGeometry().setFromPoints(added.flatMap((b) => [v3(g.nodes[b.a]), v3(g.nodes[b.b])]));
    const ls = new THREE.LineSegments(ab, new THREE.LineBasicMaterial({ color: 0x2fbf71, depthTest: false }));
    ls.renderOrder = 2;
    vehG.add(ls);
  }
  if (beamFrom && g.nodes[beamFrom]) {
    const dot = new THREE.Mesh(new THREE.SphereGeometry(0.03, 16, 12), new THREE.MeshBasicMaterial({ color: 0x2fbf71, depthTest: false }));
    dot.position.copy(v3(g.nodes[beamFrom]));
    dot.renderOrder = 2;
    vehG.add(dot);
  }
  vehG.visible = $('showbeams').checked;
}

// ---------- the vehicle's meshes (flexbodies, from the game's .dae files; meshes.js) ----------
let meshes = null, meshKey = '', meshToken = 0, meshNote = '';
const meshMats = {};
async function syncMeshes() {
  if (!veh || !$('showmesh').checked || !resolved) { bodyG.visible = false; return; }
  bodyG.visible = true;
  const key = veh.model + '|' + veh.flexbodies.map((f) => f.part + ':' + f.mesh).join(',');
  if (meshes && key === meshKey) { meshes.update(baseShape()); styleMeshes(); return; }
  const token = ++meshToken;
  meshKey = key; meshes = null; bodyG.clear();
  const src = sourceByName();
  const read = (path) => src[resolved.files[path]].files[path]();
  const daePaths = Object.keys(resolved.files).filter((path) => /\.dae$/i.test(path));
  try {
    const built = await buildMeshes(veh, daePaths, read, (t) => { if (token === meshToken) { meshNote = t; drawTiming(); } });
    if (token !== meshToken) return;
    meshes = built;
    bodyG.add(built.group);
    meshes.update(baseShape());
    styleMeshes();
    meshNote = `${built.count} meshes` + (built.missing.length ? `, ${built.missing.length} not found` : '');
  } catch (err) {
    if (token === meshToken) { meshNote = 'meshes: ' + (err.message || err); meshKey = ''; }
  }
  drawTiming(); showIssues(issues());
}

// ---------- comparing the base vehicle's meshes with the SVJ's (the Compare panel in the view) ----------
// base: opacity, style ('parts' part colours, 'grey', 'wire'), before (the shape before the fit);
// svj: opacity, style ('orange', 'own' its own colours, 'wire')
const cmp = { base: { opacity: 1, style: 'parts', before: false }, svj: { opacity: 0.55, style: 'orange' } };
// what the base meshes follow: the vehicle's nodes, or, before the fit, its nodes without the fit's moves
function baseShape() {
  if (!cmp.base.before || !veh.geometry.rest) return veh;
  const nodes = {};
  for (const [n, p] of Object.entries(veh.geometry.nodes)) {
    const d = vehEdit.fit[n];
    nodes[n] = d ? p.map((x, i) => x - d[i]) : p;
  }
  return { ...veh, geometry: { ...veh.geometry, nodes } };
}

function drawCompare() {
  const el = $('compare');
  if (el.hidden) return;
  const opt = (v, cur, label) => `<option value="${v}" ${v === cur ? 'selected' : ''}>${label}</option>`;
  el.innerHTML = `<div class="cmprow"><b>Base</b>
      <input type="range" id="cmpbo" min="0" max="1" step="0.05" value="${cmp.base.opacity}" title="Opacity">
      <select id="cmpbs">${opt('parts', cmp.base.style, 'part colours')}${opt('grey', cmp.base.style, 'grey')}${opt('wire', cmp.base.style, 'wireframe')}</select>
      <label title="The base vehicle as it was, before the fit to the SVJ"><input type="checkbox" id="cmpbb" ${cmp.base.before ? 'checked' : ''}> before the fit</label></div>
    <div class="cmprow"><b>SVJ</b>
      <input type="range" id="cmpso" min="0" max="1" step="0.05" value="${cmp.svj.opacity}" title="Opacity">
      <select id="cmpss">${opt('orange', cmp.svj.style, 'orange')}${opt('own', cmp.svj.style, 'own colours')}${opt('wire', cmp.svj.style, 'wireframe')}</select>
      <button id="cmpswap" class="mini" title="Swap which one is solid and which is see-through">swap</button></div>`;
  $('cmpbo').oninput = (e) => { cmp.base.opacity = +e.target.value; styleMeshes(); };
  $('cmpso').oninput = (e) => { cmp.svj.opacity = +e.target.value; styleSvj(); };
  $('cmpbs').onchange = (e) => { cmp.base.style = e.target.value; styleMeshes(); };
  $('cmpss').onchange = (e) => { cmp.svj.style = e.target.value; styleSvj(); };
  $('cmpbb').onchange = (e) => { cmp.base.before = e.target.checked; if (meshes) { meshes.update(baseShape()); styleMeshes(); } };
  $('cmpswap').onclick = () => {
    [cmp.base.opacity, cmp.svj.opacity] = [cmp.svj.opacity, cmp.base.opacity];
    drawCompare(); styleMeshes(); styleSvj();
  };
}
$('cmpbtn').onclick = () => { $('compare').hidden = !$('compare').hidden; $('cmpbtn').classList.toggle('on', !$('compare').hidden); drawCompare(); };

// the SVJ meshes: one colour (orange) or their own, see-through as set, or wireframe
function styleSvj() {
  meshG.traverse((o) => {
    if (!o.isMesh) return;
    o.userData.own ||= o.material.color.clone();
    const op = cmp.svj.opacity;
    o.material.color.copy(cmp.svj.style === 'own' ? o.userData.own : new THREE.Color(0xff8c2a));
    Object.assign(o.material, { opacity: op, transparent: op < 1, depthWrite: op >= 1, wireframe: cmp.svj.style === 'wire' });
    o.material.needsUpdate = true;
  });
}

// mesh colours and visibility by part: tinted with the part's colour (glass stays clear)
function styleMeshes() {
  const mat = (kind, part) => {
    const c = cmp.base.style === 'parts' ? colorOf(part) : null, k = kind + '|' + (c ? part : '');
    if (meshMats[k]) return meshMats[k];
    const base = { body: 0xc9ced6, dark: 0x2b2d31, glass: 0x9fb4c8 }[kind];
    const color = new THREE.Color(base);
    if (c && kind !== 'glass') color.lerp(c, kind === 'dark' ? 0.35 : 0.6);
    return (meshMats[k] = new THREE.MeshStandardMaterial({ color, roughness: kind === 'glass' ? 0.1 : 0.7, metalness: 0.05, side: THREE.DoubleSide,
      ...(kind === 'glass' ? { transparent: true, opacity: 0.3, depthWrite: false } : {}) }));
  };
  bodyG.traverse((m) => {
    if (!m.isMesh) return;
    m.visible = !hiddenParts.has(m.userData.part);
    const kinds = m.userData.kinds;
    m.material = kinds.length > 1 ? kinds.map((k) => mat(k, m.userData.part)) : mat(kinds[0], m.userData.part);
  });
  // the Compare panel's opacity and wireframe on every base material (glass keeps its own transparency)
  for (const [k, mt] of Object.entries(meshMats)) {
    const glass = k.startsWith('glass'), op = cmp.base.opacity * (glass ? 0.3 : 1);
    Object.assign(mt, { opacity: op, transparent: op < 1, depthWrite: op >= 1, wireframe: cmp.base.style === 'wire' });
    mt.needsUpdate = true;
  }
}
$('showmesh').onchange = () => syncMeshes();
$('partcolors').onchange = () => { drawVehicle(); styleMeshes(); drawInspector(); };

// click in the view to pick (a drag orbits instead): the node nearest the pointer on screen within 10 px
// (the nearer to the camera on a tie), else the beam nearest within 6 px; a click on empty space clears the pick
let downAt = null;
renderer.domElement.addEventListener('pointerdown', (e) => { downAt = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener('pointerup', (e) => {
  if (ws === 'sketch' && skel && downAt && e.button === 0 && Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) <= 4) { sketchClick(e); return; }
  if (!downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 4 || e.button !== 0 || !veh || !vehG.visible) return;
  const r = renderer.domElement.getBoundingClientRect(), mx = e.clientX - r.left, my = e.clientY - r.top;
  let best = null, bestD = 10, bestZ = Infinity;
  const p = new THREE.Vector3();
  for (const id of nodeIds) {
    if (!retie && !editable(veh.geometry.parts[id])) continue;     // locked parts cannot be picked
    p.copy(v3(veh.geometry.nodes[id])).project(camera);
    if (p.z > 1) continue;                                          // behind the camera
    const d = Math.hypot((p.x + 1) / 2 * r.width - mx, (1 - p.y) / 2 * r.height - my);
    if (d < bestD - 0.5 || (Math.abs(d - bestD) <= 0.5 && p.z < bestZ)) { best = id; bestD = d; bestZ = p.z; }
  }
  if (retie) {                                                    // tying a hardpoint to this node
    if (best) { vehEdit.fitOverrides[retie.key] = best; rememberTie(retie.key, best); retie = null; runFit(); }
    return;
  }
  if (best) pick = { kind: 'node', id: best };
  else {
    const scr = (id) => { p.copy(v3(veh.geometry.nodes[id])).project(camera); return [(p.x + 1) / 2 * r.width, (1 - p.y) / 2 * r.height, p.z]; };
    let bi = -1, bd = 6;
    veh.geometry.beams.forEach(([a, b], i) => {
      if (!editable(veh.geometry.beam_parts[i])) return;
      const A = scr(a), B = scr(b);
      if (A[2] > 1 || B[2] > 1) return;
      const dx = B[0] - A[0], dy = B[1] - A[1], L = dx * dx + dy * dy;
      const t = L ? Math.max(0, Math.min(1, ((mx - A[0]) * dx + (my - A[1]) * dy) / L)) : 0;
      const d = Math.hypot(A[0] + t * dx - mx, A[1] + t * dy - my);
      if (d < bd) { bd = d; bi = i; }
    });
    pick = bi >= 0 ? { kind: 'beam', a: veh.geometry.beams[bi][0], b: veh.geometry.beams[bi][1] } : null;
  }
  drawVehicle(); drawInspector();
});
$('showbeams').onchange = () => { vehG.visible = $('showbeams').checked; };

// left panel: the BeamNG folders, then the vehicle list (cars first)
function libraryPanel() {
  const role = { game: 'Game', user: 'User folder', mod: 'Mod' };
  const rows = folders.map((f) => `<li><span><b>${role[f.role]}</b> ${esc(f.name)}${f.handle ? '' : ' <span class="q">(this session)</span>'}</span>
    <span class="q">${esc(f.notes.join('; '))}</span></li>`).join('');
  const shadowed = resolved ? Object.keys(resolved.shadowed).length : 0;
  return `<h2>BeamNG folders</h2>
    ${folders.length ? `<ul class="lib">${rows}</ul>` : `<p class="quiet">Add your BeamNG.drive <b>install</b> (the folder with content/vehicles) and your
      <b>user folder</b> (with mods/; by default %LOCALAPPDATA%\\BeamNG\\BeamNG.drive). Mods and your own vehicles/ are read the game's way:
      the user folder over mods over the install. Only jbeam, config and info files are read, in your browser; nothing is uploaded.</p>`}
    ${shadowed ? `<p class="quiet">${shadowed} files are replaced by a mod or the user folder.</p>` : ''}
    ${pending.length ? `<div class="inl"><button id="bngreopen" class="primary">Reopen ${pending.length === 1 ? esc(pending[0].name) : pending.length + ' remembered folders'}</button></div>` : ''}
    <div class="inl">${canRemember ? `<button id="bngpickgame" class="primary" title="The BeamNG.drive install (with content/vehicles); remembered for the next sessions">Game folder…</button>
      <button id="bngpickmods" title="The user folder (with mods/) or a mod folder; remembered for the next sessions">Mods folder…</button>` : ''}
      <button id="bngpickdir" title="Any folder, AppData included; picked again every session">${canRemember ? 'With file dialog…' : 'Add folder…'}</button>
      <button id="bngpickzips" title="Single vehicle or mod zips">Zips…</button>
      ${folders.length || pending.length ? '<button id="bngforget" title="Close the folders and forget the remembered ones">Forget</button>' : ''}</div>
    ${canRemember && !folders.some((f) => f.role === 'user') ? `<p class="quiet">Chrome does not open folders under AppData or Program Files with <i>Mods folder</i> or <i>Game folder</i>.
      Use <i>With file dialog</i> for them (every session), or move the user folder elsewhere in the BeamNG launcher so it can be remembered.</p>` : ''}`;
}

function drawVehList() {
  const el = $('leftveh');
  if (ws !== 'base') { el.innerHTML = projectPanel(); bindProject(el); return; }
  if (vehBusy) { el.innerHTML = `<h2>BeamNG folders</h2><p class="quiet">${esc(vehBusy)}</p>`; return; }
  let html = libraryPanel();
  if (cat) {
    const f = vehFilter.toLowerCase();
    const list = cat.filter((v) => !f || `${v.name} ${v.brand} ${v.model} ${v.type} ${v.source}`.toLowerCase().includes(f));
    html += `<h2>Vehicles <span class="q">(${cat.length})</span></h2>
      <div class="inl"><input id="vehfilter" placeholder="Filter" value="${esc(vehFilter)}"></div>
      <ul>${list.map((v) => `<li data-m="${esc(v.model)}" class="${veh && veh.model === v.model ? 'sel' : ''}">
        <span>${esc(v.brand ? v.brand + ' ' : '')}${esc(v.name)}${v.source && v.source !== 'vanilla' ? ` <span class="tag">${esc(v.source)}</span>` : ''}</span>
        <span class="q">${esc(v.type || '')} · ${v.configs.length}</span></li>`).join('')}</ul>`;
  }
  el.innerHTML = html;
  if ($('bngpickgame')) $('bngpickgame').onclick = () => pickFolder('game');
  if ($('bngpickmods')) $('bngpickmods').onclick = () => pickFolder('mods');
  $('bngpickdir').onclick = () => $('bngdir').click();
  $('bngpickzips').onclick = pickZips;
  if ($('bngreopen')) $('bngreopen').onclick = reopenFolders;
  if ($('bngforget')) $('bngforget').onclick = forgetFolders;
  if (!cat) return;
  $('vehfilter').oninput = (e) => { vehFilter = e.target.value; drawVehList(); $('vehfilter').focus(); $('vehfilter').setSelectionRange(vehFilter.length, vehFilter.length); };
  el.querySelectorAll('li[data-m]').forEach((li) => li.onclick = () => openVehicle(li.dataset.m));
}

// right panel, vehicle part: configuration, parts tree (a select per slot) and tuning sliders by category
function vehInspector() {
  if (!veh) return `<p class="quiet">${vehError ? esc(vehError) : 'Pick a vehicle on the left.'}</p>`;
  const v = cat && cat.find((x) => x.model === veh.model);
  const cfgTitle = (c) => { const ci = v && v.configs.find((x) => x.name === c); return ci ? ci.title : c; };
  const slotRow = (n, depth) => {
    const opts = n.options.length ? n.options : [[n.part, n.title]];
    const sel = `<select data-slot="${esc(n.slot)}">${n.core ? '' : `<option value="" ${n.part ? '' : 'selected'}>(empty)</option>`}
      ${opts.map(([p, t]) => `<option value="${esc(p)}" ${p === n.part ? 'selected' : ''}>${esc(t)}</option>`).join('')}</select>`;
    const kids = n.children.filter((c) => c.options.length || c.children.length || c.part);
    const hasNodes = n.part && Object.values(veh.geometry.parts).includes(n.part);
    const shown = n.part && (hasNodes || veh.flexbodies.some((f) => f.part === n.part));
    const mv = hasNodes
      ? `<button class="mini${pick && pick.kind === 'part' && pick.name === n.part ? ' on' : ''}" data-pickpart="${esc(n.part)}" title="Pick this part to move it"
          ${editable(n.part) ? '' : 'disabled'}>move</button>` : '';
    const tools = shown ? `<span class="sw" style="background:${hexOf(n.part)}"></span>${mv}
      <button class="mini${lockedParts.has(n.part) ? ' on' : ''}" data-lock="${esc(n.part)}" title="Locked: its nodes and beams cannot be picked or moved">lock</button>
      <button class="mini${hiddenParts.has(n.part) ? ' on' : ''}" data-hide="${esc(n.part)}" title="Hidden: not drawn and cannot be picked">hide</button>` : '';
    return `<li style="padding-left:${depth * 10}px"><span class="q">${esc(n.description || n.slot)}${tools}</span>${sel}</li>` +
      kids.map((c) => slotRow(c, depth + 1)).join('');
  };
  const cats = {};
  for (const x of veh.variables) if (!x.hidden) (cats[x.category + (x.subCategory ? ' · ' + x.subCategory.trim() : '')] ||= []).push(x);
  const tune = Object.entries(cats).sort().map(([c, xs]) => `<h3>${esc(c)}</h3><div class="kv">${xs.map((x) => {
    const step = x.step || (typeof x.max === 'number' && typeof x.min === 'number' ? (x.max - x.min) / 100 : 0.01);
    return `<span title="${esc(x.description)}">${esc(x.title)}</span><span><input type="range" data-var="${esc(x.name)}" min="${x.min}" max="${x.max}" step="${step}" value="${x.value}">
      <b>${fmt(Number(x.value), step < 0.01 ? 3 : step < 1 ? 2 : 0)}</b> ${esc(x.unit)}</span>`;
  }).join('')}</div>`).join('');
  return `<h2>${esc(v ? (v.brand ? v.brand + ' ' : '') + v.name : veh.model)}</h2>
    <div class="kv"><span>Configuration</span><span><select id="vehcfg">${veh.configs.map((c) => `<option value="${esc(c)}" ${c === veh.config ? 'selected' : ''}>${esc(cfgTitle(c))}</option>`).join('')}</select></span></div>
    ${veh.missing.length ? `<p class="bad">Not found in the added folders: ${veh.missing.map((m) => esc(m[1])).join(', ')}</p>` : ''}
    <div class="inl"><button id="vehsave" class="primary">Save configuration (.pc)…</button><button id="vehreset">Reset to the configuration</button></div>
    ${movePanel()}
    ${beamPanel()}
    <details open><summary><b>Parts</b> <span class="q">(as the game's Parts menu)</span></summary>
      ${hiddenParts.size || lockedParts.size ? `<p class="quiet">${hiddenParts.size} hidden, ${lockedParts.size} locked
        ${hiddenParts.size ? '<button id="showall" class="mini">show all</button>' : ''}${lockedParts.size ? '<button id="unlockall" class="mini">unlock all</button>' : ''}</p>` : ''}
      <ul class="vehparts">${slotRow(veh.tree, 0)}</ul></details>
    <details><summary><b>Tuning</b> <span class="q">(${veh.variables.length} variables)</span></summary>${tune}</details>
    <p class="quiet">Making the new vehicle is in <button class="mini" data-goto="assembly">Assembly</button>.</p>`;
}

// the Move panel: the picked node (its position), beam (its midpoint: both nodes move) or part (its offset),
// as numbers in m, BeamNG axes
const AXES = [['x', 'left +'], ['y', 'rear +'], ['z', 'up +']];
const moveCount = () => Object.keys(vehEdit.moves.parts).length + Object.keys(vehEdit.moves.nodes).length;
function movePanel() {
  const moved = moveCount();
  const all = moved ? `<p class="quiet">${moved} moved (${Object.keys(vehEdit.moves.parts).length} parts, ${Object.keys(vehEdit.moves.nodes).length} nodes).
    <button id="movereset" class="mini">Reset all moves</button> Moves are not saved in the .pc: writing them into a generated part comes later (roadmap step 3).</p>` : '';
  if (!pick) return `<details open><summary><b>Move</b></summary><p class="quiet">Click a node or a beam in the view, or <i>move</i> beside a part, to move it with numbers.</p>${all}</details>`;
  const inputs = (vals, kind) => `<div class="kv">${AXES.map(([a, hint], i) => `<span>${a} <span class="q">${hint}</span></span>
    <span><input type="number" step="0.001" data-move="${kind}" data-axis="${i}" value="${Number(vals[i]).toFixed(4)}"> m</span>`).join('')}</div>`;
  let body;
  if (pick.kind === 'node') {
    const part = veh.geometry.parts[pick.id], d = vehEdit.moves.nodes[pick.id];
    body = `<div class="kv"><span>Node</span><span><b>${esc(pick.id)}</b></span><span>Part</span><span>${esc(part)} <button class="mini" data-pickpart="${esc(part)}" ${editable(part) ? '' : 'disabled'}>move part</button></span></div>
      <p class="quiet">Position:</p>${inputs(veh.geometry.nodes[pick.id], 'node')}
      ${d ? `<p class="quiet">Moved by ${d.map((x) => fmt(x * 1000, 1)).join(' / ')} mm <button id="moveundo" class="mini">Reset node</button></p>` : ''}`;
  } else if (pick.kind === 'beam') {
    const g = veh.geometry, A = g.nodes[pick.a], B = g.nodes[pick.b], part = g.beam_parts[beamIndex(g, pick)];
    const mid = [0, 1, 2].map((i) => (A[i] + B[i]) / 2);
    const moved = vehEdit.moves.nodes[pick.a] || vehEdit.moves.nodes[pick.b];
    body = `<div class="kv"><span>Beam</span><span><b>${esc(pick.a)} – ${esc(pick.b)}</b> <span class="q">${fmt(Math.hypot(A[0] - B[0], A[1] - B[1], A[2] - B[2]) * 1000, 1)} mm</span></span>
      <span>Nodes</span><span><button class="mini" data-picknode="${esc(pick.a)}">${esc(pick.a)}</button><button class="mini" data-picknode="${esc(pick.b)}">${esc(pick.b)}</button></span>
      <span>Part</span><span>${esc(part)} <button class="mini" data-pickpart="${esc(part)}" ${editable(part) ? '' : 'disabled'}>move part</button></span></div>
      <p class="quiet">Midpoint (both nodes move):</p>${inputs(mid, 'beam')}
      ${moved ? '<p><button id="moveundo" class="mini">Reset both nodes</button></p>' : ''}`;
  } else {
    const d = vehEdit.moves.parts[pick.name] || [0, 0, 0];
    const n = Object.values(veh.geometry.parts).filter((p) => p === pick.name).length;
    body = `<div class="kv"><span>Part</span><span><b>${esc(pick.name)}</b> <span class="q">${n} nodes</span></span></div>
      <p class="quiet">Offset of its nodes and of the parts in its slots (as a slot nodeMove in the game):</p>${inputs(d, 'part')}
      ${vehEdit.moves.parts[pick.name] ? '<p><button id="moveundo" class="mini">Reset part</button></p>' : ''}`;
  }
  return `<details open><summary><b>Move</b> <button id="moveclear" class="mini">clear pick</button></summary>${body}${all}</details>`;
}

// the Beams panel: beams added by hand between two nodes, with values that keep the nodes as stable as vanilla cars'
// (rigidity.added_beam: the beam's own mode at the vanilla median, both nodes under the vanilla 90th percentile,
// counting the beams added before). Typed values are taken up to the room the nodes have. Written in the main part.
let beamFrom = null, beamDraft = {};
function beamPanel() {
  const list = vehEdit.beams || [];
  const rows = list.length ? `<table class="cmp"><tr><th>Beam</th><th>k N/m</th><th>c N·s/m</th><th></th></tr>
    ${list.map((b, i) => `<tr><td>${esc(b.a)} – ${esc(b.b)}</td><td>${fmt(b.beamSpring, 0)}</td><td>${fmt(b.beamDamp, 0)}</td><td><button class="mini" data-beamdel="${i}">remove</button></td></tr>`).join('')}</table>` : '';
  let body = '';
  const node = pick && pick.kind === 'node' ? pick.id : null;
  if (beamFrom && node && node !== beamFrom) {
    let x;
    try {
      x = JSON.parse(rigpy.added_beam_json(veh.model, JSON.stringify(veh), beamFrom, node, JSON.stringify(list),
        beamDraft.k != null ? beamDraft.k : null, beamDraft.c != null ? beamDraft.c : null));
    } catch (err) { return `<details open><summary><b>Beams</b></summary><p class="bad">${esc(pyError(err))}</p>
      <p><button id="beamcancel" class="mini">cancel</button></p>${rows}</details>`; }
    const nb = Object.entries(x.nodes).map(([id, n]) => `<span>${esc(id)}</span><span>${fmt(n.kg, 2)} kg, ${esc(n.part || '')}: <span class="band ${n.before}">${n.before}</span> → <span class="band ${n.after}">${n.after}</span></span>`).join('');
    body = `<div class="kv"><span>New beam</span><span><b>${esc(beamFrom)} – ${esc(node)}</b> <span class="q">${fmt(x.length * 1000, 0)} mm</span></span>${nb}
        <span>Spring k</span><span><input type="number" id="beamk" step="1000" value="${x.beamSpring}"> N/m <span class="q">up to ${fmt(x.k_room, 0)}</span></span>
        <span>Damping c</span><span><input type="number" id="beamc" step="10" value="${x.beamDamp}"> N·s/m <span class="q">up to ${fmt(x.c_room, 0)}</span></span>
        <span>Deform · strength</span><span>${fmt(x.beamDeform, 0)} · ${x.beamStrength == null ? 'unbreakable' : fmt(x.beamStrength, 0)} <span class="q">(as the beams on these nodes)</span></span></div>
      ${x.limited ? '<p class="bad">Held at what the nodes can take: stiffer, they would pass the vanilla cars\' 90th percentile.</p>' : ''}
      ${x.warnings.map((w) => `<p class="bad">${esc(w)}</p>`).join('')}
      <div class="inl"><button id="beamadd" class="primary">Add beam</button><button id="beamauto" class="mini" ${beamDraft.k != null || beamDraft.c != null ? '' : 'disabled'}>suggested values</button><button id="beamcancel" class="mini">cancel</button></div>`;
    beamDraft.last = { a: beamFrom, b: node, beamSpring: x.beamSpring, beamDamp: x.beamDamp, beamDeform: x.beamDeform, beamStrength: x.beamStrength };
  } else if (beamFrom) {
    body = `<p class="quiet">From <b>${esc(beamFrom)}</b>: click the second node in the view. <button id="beamcancel" class="mini">cancel</button></p>`;
  } else if (node) {
    body = `<p><button id="beamstart" class="mini">New beam from ${esc(node)}</button></p>`;
  } else {
    body = '<p class="quiet">Click a node in the view to start a beam from it.</p>';
  }
  return `<details ${beamFrom || list.length ? 'open' : ''}><summary><b>Beams</b> <span class="q">(${list.length} added by hand)</span></summary>${body}${rows}
    ${list.length ? '<p class="quiet">Written at the end of the main part\'s beams on export (Assembly).</p>' : ''}</details>`;
}

function bindBeams(el) {
  if ($('beamstart')) $('beamstart').onclick = () => { beamFrom = pick.id; beamDraft = {}; drawVehicle(); drawInspector(); };
  if ($('beamcancel')) $('beamcancel').onclick = () => { beamFrom = null; beamDraft = {}; drawVehicle(); drawInspector(); };
  if ($('beamauto')) $('beamauto').onclick = () => { beamDraft = {}; drawInspector(); };
  if ($('beamk')) $('beamk').onchange = () => { beamDraft.k = $('beamk').value === '' ? null : +$('beamk').value; drawInspector(); };
  if ($('beamc')) $('beamc').onchange = () => { beamDraft.c = $('beamc').value === '' ? null : +$('beamc').value; drawInspector(); };
  if ($('beamadd')) $('beamadd').onclick = () => {
    if (!beamDraft.last) return;
    vehEdit.beams = [...(vehEdit.beams || []), beamDraft.last];
    beamFrom = null; beamDraft = {}; redraw();
  };
  el.querySelectorAll('[data-beamdel]').forEach((b) => { b.onclick = () => {
    vehEdit.beams = vehEdit.beams.filter((_, i) => i !== +b.dataset.beamdel); redraw();
  }; });
}

// a typed value: a node's new position, or a beam's new midpoint, becomes a delta added to the node moves;
// a part's value is its offset
function shiftNode(id, axis, by) {
  const d = [...(vehEdit.moves.nodes[id] || [0, 0, 0])];
  d[axis] = Math.round((d[axis] + by) * 1e4) / 1e4;
  if (d.every((x) => x === 0)) delete vehEdit.moves.nodes[id]; else vehEdit.moves.nodes[id] = d;
}

function setMove(kind, axis, value) {
  if (!Number.isFinite(value) || !pick || !pickEditable()) return;
  const g = veh.geometry;
  if (kind === 'node') shiftNode(pick.id, axis, value - g.nodes[pick.id][axis]);
  else if (kind === 'beam') {
    const by = value - (g.nodes[pick.a][axis] + g.nodes[pick.b][axis]) / 2;
    shiftNode(pick.a, axis, by);
    shiftNode(pick.b, axis, by);
  } else {
    const d = [...(vehEdit.moves.parts[pick.name] || [0, 0, 0])];
    d[axis] = value;
    if (d.every((x) => x === 0)) delete vehEdit.moves.parts[pick.name]; else vehEdit.moves.parts[pick.name] = d;
  }
  configureVehicle();
}

function bindVehInspector(el) {
  if (!veh) return;
  bindBeams(el);
  el.querySelectorAll('[data-pickpart]').forEach((b) => b.onclick = (e) => {
    e.preventDefault(); e.stopPropagation();
    if (!editable(b.dataset.pickpart)) return;
    pick = { kind: 'part', name: b.dataset.pickpart }; drawVehicle(); drawInspector();
  });
  el.querySelectorAll('input[data-move]').forEach((i) => i.onchange = () => setMove(i.dataset.move, Number(i.dataset.axis), Number(i.value)));
  const toggle = (set, part) => {
    if (set.has(part)) set.delete(part); else set.add(part);
    if (pick && !pickEditable()) pick = null;
    drawVehicle(); styleMeshes(); drawInspector();
  };
  el.querySelectorAll('[data-lock]').forEach((b) => b.onclick = () => toggle(lockedParts, b.dataset.lock));
  el.querySelectorAll('[data-hide]').forEach((b) => b.onclick = () => toggle(hiddenParts, b.dataset.hide));
  if ($('showall')) $('showall').onclick = () => { hiddenParts.clear(); drawVehicle(); styleMeshes(); drawInspector(); };
  if ($('unlockall')) $('unlockall').onclick = () => { lockedParts.clear(); drawVehicle(); drawInspector(); };
  el.querySelectorAll('[data-picknode]').forEach((b) => b.onclick = () => { pick = { kind: 'node', id: b.dataset.picknode }; drawVehicle(); drawInspector(); });
  if ($('moveclear')) $('moveclear').onclick = (e) => { e.preventDefault(); pick = null; drawVehicle(); drawInspector(); };
  if ($('moveundo')) $('moveundo').onclick = () => {
    if (pick.kind === 'node') delete vehEdit.moves.nodes[pick.id];
    else if (pick.kind === 'beam') { delete vehEdit.moves.nodes[pick.a]; delete vehEdit.moves.nodes[pick.b]; }
    else delete vehEdit.moves.parts[pick.name];
    configureVehicle();
  };
  if ($('movereset')) $('movereset').onclick = () => { vehEdit.moves = { parts: {}, nodes: {} }; configureVehicle(); };
  $('vehcfg').onchange = (e) => { vehEdit = { ...freshEdit(veh.model, e.target.value), moves: vehEdit.moves, beams: vehEdit.beams }; configureVehicle(); };   // a new configuration drops the fit
  el.querySelectorAll('select[data-slot]').forEach((s) => s.onchange = () => { vehEdit.parts[s.dataset.slot] = s.value; configureVehicle(); });
  el.querySelectorAll('input[data-var]').forEach((r) => {
    r.oninput = () => { r.nextElementSibling.textContent = fmt(Number(r.value), Number(r.step) < 0.01 ? 3 : Number(r.step) < 1 ? 2 : 0); };
    r.onchange = () => { vehEdit.vars[r.dataset.var] = Number(r.value); configureVehicle(); };
  });
  $('vehreset').onclick = () => { vehEdit = freshEdit(veh.model, veh.config); pick = null; configureVehicle(); };
  $('vehsave').onclick = () => {
    const name = prompt('Name of the configuration (saved as <name>.pc; put it in your BeamNG user folder, vehicles/' + veh.model + '/)', 'beamforge_' + veh.config);
    if (!name) return;
    const blob = new Blob([vehpy.write_pc(JSON.stringify(veh.pc))], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = name.replace(/[^\w.-]+/g, '_').replace(/\.pc$/i, '') + '.pc';
    a.click();
  };
}

$('bngdir').onchange = async (e) => {
  if (!e.target.files.length) return;
  const dir = listDir(e.target.files);
  e.target.value = '';
  await addFolder(dir, null);
};
$('bngzips').onchange = async (e) => {
  const picked = [...e.target.files];
  e.target.value = '';
  if (picked.length) await addZips(picked);
};

// single zips: each one a mod source (a game zip picked this way still works, ranked as a mod)
async function addZips(picked) {
  const old = folders.find((x) => x.name === 'picked zips');
  const sources = old ? old.sources.filter((s) => !picked.some((f) => s.name === 'zip/' + f.name)) : [];
  for (const f of picked) {
    try { const s = await zipSource(f, 'zip/' + f.name, 'mod'); if (s) sources.push(s); } catch (err) { /* not a zip */ }
  }
  if (!sources.length) { showIssues([{ level: 'WARN', rule: 'library', message: 'No vehicle files (vehicles/…) in the picked zips.' }]); return; }
  folders = folders.filter((x) => x !== old);
  folders.push({ name: 'picked zips', role: 'mod', sources, notes: [`${sources.length} zips`], handle: null });
  await libraryChanged();
}

// ---------- SVJ (Standard Vehicle JSON) ----------
let svjDoc = null;         // svjpy.load_bundle result of the last import (kept in memory)
let svjSrc = null;         // where that import's SVJ file sits in the Python file system (to read it again with its meshes)
let svjHp = [];            // its hardpoints in BeamNG coordinates, placed on the base vehicle

// where the SVJ origin (front-axle centre on the ground) lands on the base vehicle
// (after a fit: where the fit placed it, so the overlay and the fitted nodes stay together)
const svjPlace = () => {
  const pl = Object.keys(vehEdit.fit || {}).length && vehEdit.fitReport?.place;
  return pl ? { yf: pl.yf, zg: pl.ground } : { yf: veh?.measure?.front_axle_y ?? 0, zg: veh?.measure?.ground_z ?? 0 };
};

// Import from files: an .svj.json with loose mesh files, or a .zip bundle ("Files…"; the folder import is below)
$('svjfile').onchange = async (e) => {
  const picked = [...e.target.files];
  e.target.value = '';
  if (!picked.length) return;
  const f = picked.find((x) => /\.zip$/i.test(x.name)) || picked.find((x) => /\.json$/i.test(x.name));
  if (!f) { showIssues([{ level: 'ERROR', rule: 'SVJ', message: 'Pick an .svj.json (with its mesh files) or a .zip bundle.' }]); return; }
  try {
    const raw = '/tmp/svj_raw';
    py.runPython(`import shutil; shutil.rmtree('${raw}', ignore_errors=True)`);
    py.FS.mkdirTree(raw + '/meshes');
    for (const x of picked) {                        // loose mesh files go beside the svj and under meshes/
      const data = new Uint8Array(await x.arrayBuffer());
      py.FS.writeFile(`${raw}/${x.name}`, data);
      if (!/\.(json|zip)$/i.test(x.name)) py.FS.writeFile(`${raw}/meshes/${x.name}`, data);
    }
    svjSrc = `${raw}/${f.name}`;
    await finishImport(f.name);
  } catch (err) { showIssues([{ level: 'ERROR', rule: 'SVJ', message: 'Could not import: ' + pyError(err) }]); }
};

// glTF (SVJ §22.4 axes) -> SAE -> BeamNG (x = -Y, y = yf - X, z = zg - Z) -> three.js (x, z, y)
function gltfToThree(axes, yf, zg) {
  const vec = (t) => { const s = t.startsWith('-') ? -1 : 1, a = t.replace('-', ''); return new THREE.Vector3(a === 'X' ? s : 0, a === 'Y' ? s : 0, a === 'Z' ? s : 0); };
  const u = vec(axes.up), f = vec(axes.forward), r = new THREE.Vector3().crossVectors(f, u);
  return new THREE.Matrix4().set(-r.x, -r.y, -r.z, 0, u.x, u.y, u.z, zg, -f.x, -f.y, -f.z, yf, 0, 0, 0, 1);
}

// load the imported glTF meshes into meshG as translucent geometry, one holder per asset
async function loadSvjMeshes() {
  meshG.clear();
  $('meshtoggle').hidden = !(svjDoc && svjDoc.meshes.some((m) => m.file));
  if (!svjDoc) return;
  const loader = new GLTFLoader();
  for (const m of svjDoc.meshes) {
    if (!m.file) continue;
    const bytes = py.FS.readFile(m.file);
    const dir = m.file.slice(0, m.file.lastIndexOf('/') + 1);
    loader.manager.setURLModifier((url) => {            // external .bin / textures of a .gltf, from the same folder
      if (/^(blob|data):/.test(url)) return url;
      try { return URL.createObjectURL(new Blob([py.FS.readFile(dir + url.split('/').pop())])); } catch (e) { return url; }
    });
    try {
      const gl = await new Promise((ok, bad) => loader.parse(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength), '', ok, bad));
      gl.scene.traverse((o) => {
        if (o.isMesh) {
          o.material = o.material.clone();
          Object.assign(o.material, { transparent: true, opacity: 0.55, side: THREE.DoubleSide, depthWrite: false });
        }
      });
      const holder = new THREE.Group();
      holder.matrixAutoUpdate = false;
      setTimeout(styleSvj, 0);
      holder.add(gl.scene);
      holder.userData.uri = m.uri;
      holder.userData.offset = m.offset_gltf;             // the mesh moved onto its wheels (svj.mesh_offset)
      meshG.add(holder);
    } catch (err) {
      showIssues([{ level: 'WARN', rule: 'SVJ §22', message: `could not load mesh ${m.uri}: ${err.message || err}` }]);
    }
  }
  placeSvj();
}
$('meshes').onchange = () => { meshG.visible = $('meshes').checked; };

// The SVJ names each mesh by a path relative to the SVJ file (assets.meshes[].uri), but a page cannot open a
// path beside a file it was given: it reads only what the user hands over. So the user hands over the SVJ's
// folder (Import SVJ), and the SVJ and its meshes are searched for below it: the .svj.json (the shallowest),
// then for each mesh its uri from the SVJ's folder, then its file name anywhere under that folder, then
// anywhere under the picked one. A mesh left over is asked for again with "Find the meshes folder".
const SVJ_SEARCH_DEPTH = 8, SVJ_SEARCH_FILES = 20000;

// every file below a directory (library.js directory interface): [{ path (lower case, from the folder), shown (as it is), name, file }]
async function indexDir(dir) {
  const out = [];
  const walk = async (d, prefix, depth, prefix0) => {
    for (const [k, c] of await d.entries()) {
      if (out.length >= SVJ_SEARCH_FILES) return;
      if (c.dir) { if (depth < SVJ_SEARCH_DEPTH) await walk(c.dir, prefix + k + '/', depth + 1, prefix0 + c.name + '/'); }
      else if (c.file) out.push({ path: prefix + k, shown: prefix0 + c.name, name: c.name, file: c.file });
    }
  };
  await walk(dir, '', 0, '');
  return out;
}

// a path joined to a folder, with ./ and ../ resolved (lower case: the index is)
function joinPath(folder, rel) {
  const parts = (folder + rel).replace(/\\/g, '/').toLowerCase().split('/');
  const out = [];
  for (const p of parts) { if (p === '..') out.pop(); else if (p && p !== '.') out.push(p); }
  return out.join('/');
}

// the files for the SVJ's meshes (meshes: [{ uri }]), written under `base` (a folder in the Python file system) at
// their uri; a .gltf takes the files beside it (.bin, textures). Returns the uris found.
async function supplyMeshes(files, jsonDir, meshes, base) {
  const byPath = new Map(files.map((f) => [f.path, f]));
  const found = [];
  for (const m of meshes) {
    const name = m.uri.replace(/\\/g, '/').split('/').pop().toLowerCase();
    const under = files.filter((f) => f.path.startsWith(jsonDir) && f.name.toLowerCase() === name);
    const hit = byPath.get(joinPath(jsonDir, m.uri)) || under.sort((a, b) => a.path.length - b.path.length)[0]
      || files.filter((f) => f.name.toLowerCase() === name).sort((a, b) => a.path.length - b.path.length)[0];
    if (!hit) continue;
    const to = base + m.uri.replace(/\\/g, '/').replace(/^\.\//, '');
    py.FS.mkdirTree(to.slice(0, to.lastIndexOf('/')));
    py.FS.writeFile(to, new Uint8Array(await (await hit.file()).arrayBuffer()));
    if (/\.gltf$/i.test(hit.name)) {
      const dirOf = hit.path.slice(0, hit.path.lastIndexOf('/') + 1);
      for (const s of files) {
        if (s !== hit && s.path.startsWith(dirOf) && !s.path.slice(dirOf.length).includes('/') && /\.(bin|png|jpe?g|webp)$/i.test(s.name)) {
          py.FS.writeFile(to.slice(0, to.lastIndexOf('/') + 1) + s.name, new Uint8Array(await (await s.file()).arrayBuffer()));
        }
      }
    }
    found.push(m.uri);
  }
  return found;
}

// the SVJ loaded from the Python file system (svjSrc): its meshes drawn, the panels redrawn
async function finishImport(name) {
  svjDoc = JSON.parse(svjpy.load_bundle(svjSrc, '/tmp/svj_in'));
  svjName = name;
  if (!ptEdit.touched) ptEdit = { ...JSON.parse(ptpy.from_svj_json(JSON.stringify(svjDoc.svj))), touched: false };
  if (!aeroTouched) aeroEdit = aeroFromSvj();
  studySuspension();
  await loadSvjMeshes();
  redraw();
  if (!veh) fitCamera();
  showIssues([{ level: 'PASS', rule: 'SVJ', message: `Loaded ${name}${svjDoc.summary.vehicle ? ' (' + svjDoc.summary.vehicle + ')' : ''}.` },
    ...svjDoc.notes.map((n) => ({ level: 'WARN', rule: 'SVJ', message: n })),
    ...svjDoc.meshes.filter((m) => m.file).map((m) => ({ level: 'INFO', rule: 'SVJ §22', message: `mesh ${m.uri}: nodes ${m.nodes.join(', ') || '(none)'}` }))]);
}

// Import SVJ: a folder holding the SVJ (a .svj.json, else a .zip bundle) and its meshes
async function importSvjFolder(dir) {
  const files = await indexDir(dir);
  const depth = (f) => f.path.split('/').length;
  const byDepth = (a, b) => depth(a) - depth(b) || a.path.localeCompare(b.path);
  const jsons = files.filter((f) => /\.svj\.json$/i.test(f.name)).sort(byDepth);
  const zip = files.filter((f) => /\.zip$/i.test(f.name)).sort(byDepth)[0];
  if (!jsons.length && !zip) {
    showIssues([{ level: 'ERROR', rule: 'SVJ', message: `${dir.name}: no .svj.json or .zip bundle in it or below it.` }]);
    return;
  }
  const raw = '/tmp/svj_raw';
  py.runPython(`import shutil; shutil.rmtree('${raw}', ignore_errors=True)`);
  py.FS.mkdirTree(raw);
  const pick = jsons[0] || zip;
  const data = new Uint8Array(await (await pick.file()).arrayBuffer());
  py.FS.writeFile(`${raw}/${pick.name}`, data);
  svjSrc = `${raw}/${pick.name}`;
  if (jsons.length) {                                // the meshes the SVJ names, found below its folder
    const jsonDir = pick.path.slice(0, pick.path.lastIndexOf('/') + 1);
    let doc = null;
    try { doc = JSON.parse(new TextDecoder().decode(data)); } catch (e) { /* load_bundle reports it */ }
    await supplyMeshes(files, jsonDir, (doc && doc.assets && doc.assets.meshes) || [], raw + '/');
  }
  await finishImport(pick.name);
  if (jsons.length > 1) {
    showIssues([{ level: 'INFO', rule: 'SVJ', message: `${jsons.length} SVJs in ${dir.name}: loaded ${pick.shown}; the others: ${jsons.slice(1, 6).map((f) => f.shown).join(', ')}${jsons.length > 6 ? '…' : ''}. Pick a single SVJ's folder to load another.` }]);
  }
}

// a mesh that was not found: the folder holding it
async function findSvjMeshes() {
  if (!svjDoc || !svjSrc) return;
  let dir;
  try {
    if (window.showDirectoryPicker) dir = handleDir(await window.showDirectoryPicker({ id: 'svj-meshes', mode: 'read' }));
    else { svjDirMode = 'meshes'; $('svjdir').click(); return; }
  } catch (e) { return; }                            // cancelled, or a folder Chrome blocks
  await useSvjFolder(dir);
}

async function useSvjFolder(dir) {
  const files = await indexDir(dir);
  const base = svjSrc.slice(0, svjSrc.lastIndexOf('/') + 1);
  const wanted = svjDoc.meshes.filter((m) => !m.file);
  const found = await supplyMeshes(files, '', wanted, base);
  svjDoc = JSON.parse(svjpy.load_bundle(svjSrc, '/tmp/svj_in'));
  await loadSvjMeshes();
  redraw();
  const missing = wanted.length - found.length;
  showIssues([{ level: found.length ? 'PASS' : 'WARN', rule: 'SVJ §22', message: `${found.length} SVJ mesh${found.length === 1 ? '' : 'es'} found in ${dir.name}`
    + (missing > 0 ? `; ${missing} still not found (looked for their uri, then their file name, in every folder below)` : '') + '.' }]);
}

// where the browser has no folder picker: a folder input, for either purpose
let svjDirMode = 'import';
$('svjdir').onchange = async (e) => {
  const files = [...e.target.files];
  e.target.value = '';
  if (!files.length) return;
  try {
    if (svjDirMode === 'meshes') await useSvjFolder(listDir(files));
    else await importSvjFolder(listDir(files));
  } catch (err) { showIssues([{ level: 'ERROR', rule: 'SVJ', message: 'Could not import: ' + pyError(err) }]); }
};

$('svjin').onclick = async () => {
  if (!window.showDirectoryPicker) { svjDirMode = 'import'; $('svjdir').click(); return; }
  let dir;
  try { dir = handleDir(await window.showDirectoryPicker({ id: 'svj-folder', mode: 'read' })); } catch (e) { return; }
  try { await importSvjFolder(dir); } catch (err) { showIssues([{ level: 'ERROR', rule: 'SVJ', message: 'Could not import: ' + pyError(err) }]); }
};
$('svjinfiles').onclick = () => $('svjfile').click();
$('hps').onchange = () => { hpG.visible = $('hps').checked; };

// put the SVJ meshes and hardpoints on the base vehicle's front axle and ground (after every reconfigure)
function placeSvj() {
  if (!svjDoc) { svjHp = []; return; }
  const { yf, zg } = svjPlace();
  const M = gltfToThree(svjDoc.axes, yf, zg);
  for (const h of meshG.children) {
    h.matrix.copy(M);
    const o = h.userData.offset;
    if (o) h.matrix.multiply(new THREE.Matrix4().makeTranslation(o[0], o[1], o[2]));
  }
  meshG.visible = $('meshes').checked;
  svjHp = JSON.parse(svjpy.hardpoints_json(JSON.stringify(svjDoc.svj), yf, zg));
}

// SVJ hardpoints: upright points orange, chassis (link inboard) points purple
function drawHardpoints() {
  hpG.clear();
  $('hptoggle').hidden = !svjHp.length;
  const mat = { upright: new THREE.MeshBasicMaterial({ color: 0xe0782a }), chassis: new THREE.MeshBasicMaterial({ color: 0x8250df }) };
  const geo = new THREE.SphereGeometry(0.018, 12, 8);
  for (const h of svjHp) {
    const s = new THREE.Mesh(geo, mat[h.kind]);
    s.position.copy(v3(h.pos));
    s.userData = h;
    hpG.add(s);
  }
  hpG.visible = $('hps').checked;
}

// right panel, SVJ part: file summary, bindings, and the base-vs-SVJ comparison
function svjInspector() {
  if (!svjDoc) return `<h2>SVJ</h2><p class="quiet">Import SVJ to see a vehicle's meshes and hardpoints on the base vehicle and compare their values.</p>`;
  const s = svjDoc.summary;
  let cmp = '';
  if (veh) {
    const rows = JSON.parse(svjpy.compare_json(JSON.stringify(svjDoc.svj), JSON.stringify(veh.measure), null));
    const v = (x, u) => x === null || x === undefined ? '–' : fmt(x, u === 'kg' ? 0 : 3);
    cmp = `<h2>Base vs SVJ</h2><table class="cmp"><tr><th></th><th>Base</th><th>SVJ</th><th>Δ</th></tr>
      ${rows.map((r) => `<tr><td>${esc(r.label)}</td><td>${v(r.base, r.unit)}</td><td>${v(r.svj, r.unit)}</td><td>${v(r.delta, r.unit)} ${r.delta === null ? '' : esc(r.unit)}</td></tr>`).join('')}</table>
      ${fitPanel()}
      ${valuesPanel()}`;
  }
  return `<h2>SVJ ${esc(s.version || '')}</h2>
    <div class="kv"><span>Vehicle</span><span>${esc(s.vehicle || '–')}</span>
      <span>Corners</span><span>${Object.entries(s.corners).map(([k, t]) => `${esc(k)} ${esc(t || '?')}`).join(', ') || '–'}</span>
      <span>Meshes</span><span>${svjDoc.meshes.length} (${svjDoc.bindings.length} bindings)</span>
      <span>Hardpoints</span><span>${svjHp.length}</span></div>
    ${svjDoc.meshes.some((m) => !m.file) ? `<p class="bad">${svjDoc.meshes.filter((m) => !m.file).length} of the SVJ's meshes were not handed over (it names them by a path beside the SVJ file, which a page cannot open: ${esc(svjDoc.meshes.filter((m) => !m.file).map((m) => m.uri).slice(0, 3).join(', '))}).
      <button id="svjfindmeshes">Find the meshes folder…</button></p>` : ''}
    ${svjDoc.bindings.length ? `<details><summary>Visual bindings</summary><div class="kv">${svjDoc.bindings.map((b) =>
      `<span>${esc(b.path)}</span><span>${esc(b.node)}</span>`).join('')}</div></details>` : ''}
    ${cmp}
    ${svjSusp ? '<p class="quiet">The suspension study is in <button class="mini" data-goto="suspension">Suspension</button>.</p>' : ''}`;
}

// ---------- fit the base vehicle to the SVJ (beamforge/fit.py, docs/fitting.md) ----------
const FIT_STAGES = [['wheelbase', '1 Wheelbase', 'Stretch between the axle lines to the SVJ wheelbase; the front axle stays'],
  ['body', '2 Body to the mesh', 'Overhangs, floor, width and roof line to the SVJ body mesh (the chassis visual binding)'],
  ['pickups', '3 Pickup points', 'Every SVJ hardpoint tied to a node by its role, moved exactly onto it; the nodes around follow']];
let fitStages = new Set(['wheelbase', 'body', 'pickups']);
let retie = null;            // { key, label }: the hardpoint waiting for the user to click its node

// The user's ties, remembered per suspension part (the part that owns the node) in this browser:
// { part: { "FL:lower_ball_joint": node } }. Every vehicle using that part starts from them.
const TIES_KEY = 'beamforge.ties';
function savedTies() { try { return JSON.parse(localStorage.getItem(TIES_KEY) || '{}'); } catch (e) { return {}; } }
function storeTies(t) { try { localStorage.setItem(TIES_KEY, JSON.stringify(t)); } catch (e) { /* private window: this session only */ } }
function rememberTie(key, node) {
  const t = savedTies();
  for (const ties of Object.values(t)) delete ties[key];          // one node per hardpoint
  if (node) {
    const part = veh.geometry.parts[node];
    if (part) (t[part] ||= {})[key] = node;
  }
  for (const [part, ties] of Object.entries(t)) if (!Object.keys(ties).length) delete t[part];
  storeTies(t);
}
// the ties for this vehicle: the remembered ones whose part (and node) it has, then this session's
function tiesFor() {
  const out = {}, t = savedTies(), active = new Set(Object.values(veh.geometry.parts));
  for (const [part, ties] of Object.entries(t)) {
    if (!active.has(part)) continue;
    for (const [key, node] of Object.entries(ties)) if (veh.geometry.parts[node] === part) out[key] = node;
  }
  return Object.assign(out, vehEdit.fitOverrides || {});
}

function fitPanel() {
  const r = vehEdit.fitReport;
  const v = (x) => x === null || x === undefined ? '–' : fmt(x, 3);
  const table = r && r.report.length ? `<table class="cmp"><tr><th></th><th>Base</th><th>SVJ</th><th>After</th></tr>
    ${r.report.map((x) => `<tr><td>${esc(x.label)}</td><td>${v(x.base)}</td><td>${v(x.target)}</td><td>${v(x.after)}</td></tr>`).join('')}</table>` : '';
  return `<h2>Fit to the SVJ</h2>
    <div class="kv">${FIT_STAGES.map(([k, label, hint]) => `<span><label title="${esc(hint)}"><input type="checkbox" data-fitstage="${k}" ${fitStages.has(k) ? 'checked' : ''}> ${esc(label)}</label></span><span></span>`).join('')}</div>
    <div class="inl"><button id="fitrun" class="primary">Fit</button>${Object.keys(vehEdit.fit).length ? '<button id="fitclear">Remove fit</button>' : ''}</div>
    ${table}${r && r.notes.length ? r.notes.map((n) => `<p class="bad">${esc(n)}</p>`).join('') : ''}
    ${r && r.changes && r.changes.length ? `<p class="quiet"><b>Largest changes to the base:</b> ${r.changes.map(esc).join(' · ')}</p>` : ''}
    ${uprightTable(r)}
    ${mappingTable(r)}
    <p class="quiet">The fit moves every node; hand moves stay on top. Your ties are remembered per suspension part in this browser, for every vehicle that uses it.</p>`;
}

// stage 3's check of each hub against the SVJ upright: shape gap before the fit, hub beams after, wheel angles
function uprightTable(r) {
  if (!r || !r.uprights || !r.uprights.length) return '';
  const mm = (x) => x === null || x === undefined ? '–' : fmt(x, 0) + ' mm';
  const ang = (a, b) => `${fmt(a, 2)}°${b === null || b === undefined ? '' : ` <span class="q">(SVJ ${fmt(b, 2)}°)</span>`}`;
  return `<details open><summary><b>Uprights: base against the SVJ</b> <span class="q">(how much each hub was reshaped)</span></summary>
    <table class="cmp"><tr><th>Corner</th><th>Shape difference</th><th>Hub beams changed</th><th>Camber</th><th>Toe</th></tr>
    ${r.uprights.map((u) => `<tr title="${esc(u.points.join(', '))}${u.worst_pair ? ` · worst pair ${u.worst_pair[0]}–${u.worst_pair[1]}: ${u.worst_pair[2]} mm on the base, ${u.worst_pair[3]} mm in the SVJ` : ''}">
      <td>${esc(u.corner)}</td>
      <td>${u.shape_rms_mm === null ? `<span class="q">${u.points.length} points</span>` : mm(u.shape_rms_mm)}</td>
      <td>${u.hub_beams_pct === null ? '–' : fmt(u.hub_beams_pct, 0) + ' %'}</td>
      <td>${ang(u.camber_deg, u.svj_camber_deg)}</td><td>${ang(u.toe_deg, u.svj_toe_deg)}</td></tr>`).join('')}</table>
    <p class="quiet">Shape difference: how far the base hub was from the SVJ upright before the fit (the gap left after the best rigid overlay of its tied points; 0 = the same upright; needs 3 points). The fit reshapes the hub towards the SVJ: its tied nodes go exactly onto the SVJ points and the rest follows. Hub beams changed: the largest length change inside the hub. The wheel axis takes the SVJ static camber and toe when the file has them, else keeps the base vehicle's. Hover a row for its points and the pair that differs most.</p></details>`;
}

// the hardpoint-to-node ties of stage 3: gap before the fit, how it was tied, re-tie by clicking a node
function mappingTable(r) {
  if (!r || !r.mapping || !r.mapping.length) return '';
  const nice = (s) => s.replace(/_/g, ' ').replace(/\.(\d)$/, (m, i) => ` ${+i + 1}`);
  const by = { wheel: 'wheel', guess: 'guessed', user: 'yours', none: '–', far: 'not tied: too far', same: 'same point', kept: 'base hub kept' };
  return `<details open><summary><b>Hardpoints and their nodes</b> <span class="q">(gap before the fit)</span></summary>
    ${retie ? `<p class="bad">Click the node for ${esc(retie.label)} in the view. <button id="retiecancel" class="mini">cancel</button></p>` : ''}
    <table class="cmp"><tr><th>Corner</th><th>Hardpoint</th><th>Node</th><th>Gap</th><th></th></tr>
    ${r.mapping.map((m) => {
      const key = `${m.corner}:${m.name}`;
      return `<tr><td>${esc(m.corner)}</td><td>${esc(nice(m.name))}</td>
        <td>${m.nodes.map((n) => `<button class="mini" data-picknode="${esc(n)}">${esc(n)}</button>`).join('') || '–'} <span class="q">${by[m.by] || ''}</span></td>
        <td>${m.distance === null ? '–' : fmt(m.distance * 1000, 0) + ' mm'}</td>
        <td>${m.kind === 'wheel' ? '' : `<button class="mini${retie && retie.key === key ? ' on' : ''}" data-retie="${esc(key)}" data-label="${esc(m.corner + ' ' + nice(m.name))}">re-tie</button>`}
          ${m.by === 'user' ? `<button class="mini" data-untie="${esc(key)}">auto</button>` : ''}</td></tr>`;
    }).join('')}</table></details>`;
}

// springs, dampers and tyres: base against SVJ, with a take box each (beamforge/values.py)
function valuesPanel() {
  let rows = [];
  try { rows = JSON.parse(valpy.table(veh.model, JSON.stringify(veh), JSON.stringify(svjDoc.svj), JSON.stringify({ corners: svjSusp?.corners || {} }))); }
  catch (err) { return `<p class="bad">${esc(pyError(err))}</p>`; }
  rows = rows.filter((r) => !PT_KEYS.includes(r.key));
  if (!rows.length) return '';
  const v = (x, u) => x === null || x === undefined ? '–' : typeof x === 'string' ? esc(x) : fmt(x, u === 'm' ? 3 : u === 'kW' || u === '' ? 2 : 0);
  let sugg = [];
  try { sugg = JSON.parse(valpy.suggest_parts(veh.model, JSON.stringify(veh), JSON.stringify(svjDoc.svj))); } catch (err) { sugg = []; }
  const suggestions = sugg.length ? `<p class="quiet">Parts of ${esc(veh.model)} closer to the SVJ:</p>
    <table class="cmp"><tr><th>Slot</th><th>Now</th><th>Closer</th></tr>${sugg.map((x) => `<tr title="${esc(x.why)}"><td>${esc(x.slot)}</td><td>${esc(x.current)}</td><td>${esc(x.suggested)}</td></tr>`).join('')}</table>
    <div class="inl"><button id="usesugg" title="Choose these parts (the fit is run again if there is one)">Use these parts</button></div>` : '';
  return `<h2>Values from the SVJ</h2>${suggestions}
    <table class="cmp"><tr><th></th><th>Base</th><th>SVJ</th><th>Take</th></tr>
    ${rows.map((r) => `<tr title="${esc(r.note || '')}"><td>${esc(r.label)} <span class="q">${esc(r.unit)}</span></td><td>${v(r.base, r.unit)}</td><td>${v(r.svj, r.unit)}</td>
      <td>${r.svj === null || r.svj === undefined || r.info ? '' : `<input type="checkbox" data-take="${esc(r.key)}" ${takeValues[r.key] !== false ? 'checked' : ''}>`}</td></tr>`).join('')}</table>
    <p class="quiet">Engine, gears, final drive and steering: <button class="mini" data-goto="powertrain">Powertrain</button>.</p>
    <p class="quiet">Taken values go into the new vehicle (Make a new vehicle): spring rates into the coil spring beams (the SVJ wheel rate over the spring's motion ratio squared), damping into the damper beams (slopes of the SVJ curves), tyre radius into the tyre parts, mass and CG as node weights, the torque curve into the engine (the exhaust's own change added back), gear ratios into the gearbox (reverse kept) and the final drive into the driven axle's differential. Hover a row for details.</p>`;
}

function bindFitPanel() {
  if ($('usesugg')) $('usesugg').onclick = () => {
    for (let round = 0; round < 4; round++) {        // a new wheel brings its own tyre slot: ask again
      const sugg = JSON.parse(valpy.suggest_parts(veh.model, JSON.stringify(veh), JSON.stringify(svjDoc.svj)))
        .filter((x) => vehEdit.parts[x.slot] !== x.suggested);
      if (!sugg.length) break;
      for (const x of sugg) vehEdit.parts[x.slot] = x.suggested;
      configureVehicle();
    }
    if (Object.keys(vehEdit.fit || {}).length) runFit();
  };
  document.querySelectorAll('[data-take]').forEach((c) => c.onchange = () => { takeValues[c.dataset.take] = c.checked; });
  document.querySelectorAll('[data-retie]').forEach((b) => b.onclick = () => { retie = { key: b.dataset.retie, label: b.dataset.label }; drawInspector(); });
  document.querySelectorAll('[data-untie]').forEach((b) => b.onclick = () => {
    delete vehEdit.fitOverrides[b.dataset.untie]; rememberTie(b.dataset.untie, null); runFit();
  });
  if ($('retiecancel')) $('retiecancel').onclick = () => { retie = null; drawInspector(); };
  document.querySelectorAll('[data-fitstage]').forEach((c) => c.onchange = () => { if (c.checked) fitStages.add(c.dataset.fitstage); else fitStages.delete(c.dataset.fitstage); });
  if ($('fitrun')) $('fitrun').onclick = runFit;
  if ($('svjfindmeshes')) $('svjfindmeshes').onclick = findSvjMeshes;
  if ($('fitclear')) $('fitclear').onclick = () => { vehEdit.fit = {}; vehEdit.fitReport = null; studySuspension(); configureVehicle(); };
}

function runFit() {
  if (!veh || !svjDoc) return;
  const files = Object.fromEntries(svjDoc.meshes.filter((m) => m.file).map((m) => [m.id, m.file]));
  try {
    const r = JSON.parse(fitpy.fit_json(JSON.stringify(veh.geometry), JSON.stringify(veh.wheels), JSON.stringify(svjDoc.svj),
      JSON.stringify(files), JSON.stringify([...fitStages]), JSON.stringify(tiesFor())));
    vehEdit.fit = r.moves;
    vehEdit.fitReport = r;
    studySuspension(r);
  } catch (err) { vehEdit.fitReport = { report: [], notes: ['fit failed: ' + pyError(err)] }; }
  configureVehicle();
}

// ---------- the SVJ's suspension: kinematics over wheel travel (beamforge/suspension.py, from FBeam) ----------
// svjSusp: suspy.study_svj of the imported SVJ, {corners: {front, rear}, notes}; each corner has its points
// (left side, front axle at y 0, ground at z 0), lines to draw, static values, curves and solver frames.
let svjSusp = null, suspCorner = 'front', suspTravel = 0;
const SCOL = { arm: 0x1a7f37, tie: 0x8250df, strut: 0xbf8700, upright: 0x8c959f };

// fit: a fit result; its hardpoint ties give the base vehicle's points (before the fit) for a second study,
// the SVJ's layout on those points ("base"), compared with the SVJ in the panel
function studySuspension(fit) {
  let base = null;
  if (fit && fit.mapping && fit.mapping.length && veh) {
    try { base = fitpy.base_points_json(JSON.stringify(veh.geometry), JSON.stringify(fit.mapping), JSON.stringify(fit.place)); } catch (err) { base = null; }
  }
  try { svjSusp = JSON.parse(suspy.study_svj_json(JSON.stringify(svjDoc.svj), 100, base)); }
  catch (err) { svjSusp = { corners: {}, notes: ['suspension: ' + pyError(err)] }; }
  if (!svjSusp.corners[suspCorner]) suspCorner = Object.keys(svjSusp.corners)[0] || 'front';
}

function frameAt(res, t) {          // solver frame nearest to travel t (mm)
  let best = res.frames[0];
  for (const f of res.frames) if (Math.abs(f.travel_mm - t) < Math.abs(best.travel_mm - t)) best = f;
  return best;
}

function tubeMesh(a, b, r, color, opacity = 1) {
  const dir = new THREE.Vector3().subVectors(b, a);
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(r, r, dir.length(), 12),
    new THREE.MeshStandardMaterial({ color, roughness: 0.5, metalness: 0.2, transparent: opacity < 1, opacity }));
  mesh.position.copy(a).addScaledVector(dir, 0.5);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
  return mesh;
}

// both sides of every corner at the slider travel, placed like the SVJ (front axle line, ground)
function drawSuspension() {
  suspG.clear();
  $('susptoggle').hidden = !(svjSusp && Object.keys(svjSusp.corners).length);
  if (!svjSusp || !$('showsusp').checked) return;
  const { yf, zg } = svjPlace();
  const rad = { arm: 0.011, tie: 0.008, strut: 0.02, upright: 0.014 };
  for (const res of Object.values(svjSusp.corners)) {
    const f = frameAt(res, suspTravel), P = { ...res.points, ...f.pts };
    for (const side of [1, -1]) {
      const at = (k) => {
        const m = k.startsWith('~'), q = P[m ? k.slice(1) : k];
        return q && v3([(m ? -q[0] : q[0]) * side, q[1] + yf, q[2] + zg]);
      };
      for (const [a, b, kind, once] of res.lines) {
        if (once && side < 0) continue;
        const pa = at(a), pb = at(b);
        if (pa && pb && pa.distanceTo(pb) > 1e-4) suspG.add(tubeMesh(pa, pb, rad[kind] || 0.01, SCOL[kind] || 0x8c959f));
      }
      const r = res.points.WC[2] > 0.1 ? res.points.WC[2] : 0.3;
      const w = new THREE.Mesh(new THREE.CylinderGeometry(r, r, 0.18, 32),
        new THREE.MeshStandardMaterial({ color: 0x57606a, transparent: true, opacity: 0.35, depthWrite: false }));
      w.position.copy(at('WC'));
      w.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), v3([f.spin[0] * side, f.spin[1], f.spin[2]]).normalize());
      suspG.add(w);
    }
  }
}
$('showsusp').onchange = () => drawSuspension();

// small SVG line chart of one curve over wheel travel, with the usable band and the slider position (from FBeam)
// second: optional {xs, ys} drawn dashed in orange (the base vehicle), sharing the axes
function chart(title, xs, ys, unit, cur, usable, digits = 1, second = null) {
  const ok = (p) => p[1] !== null && p[1] !== undefined && Number.isFinite(p[1]);
  const pts = xs.map((x, i) => [x, ys[i]]).filter(ok);
  const pts2 = second ? second.xs.map((x, i) => [x, second.ys[i]]).filter(ok) : [];
  if (pts.length < 2) return '';
  const W = 270, H = 104, L = 40, B = 18, T = 6;
  const both = pts.concat(pts2);
  const x0 = Math.min(...pts.map((p) => p[0])), x1 = Math.max(...pts.map((p) => p[0]));
  let y0 = Math.min(...both.map((p) => p[1])), y1 = Math.max(...both.map((p) => p[1]));
  if (y1 - y0 < 1e-6) { y0 -= 1; y1 += 1; }
  const pad = (y1 - y0) * 0.08; y0 -= pad; y1 += pad;
  const X = (x) => L + (x - x0) / (x1 - x0) * (W - L - 6), Y = (y) => T + (y1 - y) / (y1 - y0) * (H - T - B);
  const path = pts.map((p, i) => `${i ? 'L' : 'M'}${X(p[0]).toFixed(1)},${Y(p[1]).toFixed(1)}`).join('');
  const inX = pts2.filter((p) => p[0] >= x0 && p[0] <= x1);
  const path2 = inX.length > 1 ? inX.map((p, i) => `${i ? 'L' : 'M'}${X(p[0]).toFixed(1)},${Y(p[1]).toFixed(1)}`).join('') : '';
  const here2 = inX.length ? inX.reduce((b, p) => (Math.abs(p[0] - cur) < Math.abs(b[0] - cur) ? p : b), inX[0]) : null;
  const here = pts.reduce((b, p) => (Math.abs(p[0] - cur) < Math.abs(b[0] - cur) ? p : b), pts[0]);
  const zeroY = y0 < 0 && y1 > 0 ? `<line x1="${L}" x2="${W - 6}" y1="${Y(0)}" y2="${Y(0)}" stroke="currentColor" stroke-opacity=".25"/>` : '';
  const band = usable ? `<rect x="${X(Math.max(usable[0], x0))}" y="${T}" width="${Math.max(0, X(Math.min(usable[1], x1)) - X(Math.max(usable[0], x0)))}" height="${H - T - B}" fill="#2f6fdf" fill-opacity=".07"/>` : '';
  return `<div class="chart"><div class="t"><span>${title}</span><span>${fmt(here[1], digits)} ${unit}${here2 ? ` <span style="color:#e0782a">(base ${fmt(here2[1], digits)})</span>` : ''} at ${fmt(here[0], 0)} mm</span></div>
    <svg viewBox="0 0 ${W} ${H}" style="color:var(--ink)">${band}${zeroY}
      <line x1="${X(0)}" x2="${X(0)}" y1="${T}" y2="${H - B}" stroke="currentColor" stroke-opacity=".25"/>
      ${path2 ? `<path d="${path2}" fill="none" stroke="#e0782a" stroke-width="1.6" stroke-dasharray="4 3"/>` : ''}
      <path d="${path}" fill="none" stroke="#2f6fdf" stroke-width="1.8"/>
      <circle cx="${X(here[0])}" cy="${Y(here[1])}" r="3.5" fill="#cf222e"/>
      <text x="${L - 4}" y="${Y(y1 - pad) + 4}" text-anchor="end" font-size="10" fill="currentColor" fill-opacity=".6">${fmt(y1 - pad, digits)}</text>
      <text x="${L - 4}" y="${Y(y0 + pad) + 4}" text-anchor="end" font-size="10" fill="currentColor" fill-opacity=".6">${fmt(y0 + pad, digits)}</text>
      <text x="${L}" y="${H - 4}" font-size="10" fill="currentColor" fill-opacity=".6">${fmt(x0, 0)}</text>
      <text x="${W - 6}" y="${H - 4}" text-anchor="end" font-size="10" fill="currentColor" fill-opacity=".6">+${fmt(x1, 0)} mm</text>
    </svg></div>`;
}

function suspPanel() {
  if (!svjSusp) return '';
  const names = Object.keys(svjSusp.corners);
  const general = svjSusp.notes.map((n) => `<p class="bad">${esc(n)}</p>`).join('');
  if (!names.length) return `<h2>Suspension</h2>${general || '<p class="quiet">No suspension corners in this SVJ.</p>'}`;
  const res = svjSusp.corners[suspCorner] || svjSusp.corners[names[0]];
  const st = res.static, c = res.curves, t = suspTravel, usable = [-st.rebound_mm, st.bump_mm];
  const bres = svjSusp.base && svjSusp.base.corners[suspCorner];
  const bs = bres && bres.static, bc = bres && bres.curves;
  const d0 = (v, n, u) => v === null || v === undefined ? '–' : fmt(v, n) + u;
  const d = (v, n, u, key) => d0(v, n, u) + (bs && key ? ` <span style="color:#e0782a">· ${d0(bs[key], n, u)}</span>` : '');
  const row = (k, v) => `<span>${k}</span><span>${v}</span>`;
  const sec = (key) => bc ? { xs: bc.travel_mm, ys: bc[key] } : null;
  const moving = c.travel_mm.length > 1;
  return `<h2>Suspension <span class="q">(from the SVJ)</span></h2>
    <div class="inl">${names.map((n) => `<button data-sc="${n}" class="${n === suspCorner ? 'on' : ''}">${n} · ${esc(svjSusp.corners[n].corner)}</button>`).join(' ')}</div>
    <div class="kv">${row('Layout', esc(res.name))}
      ${row('Travel', `<input type="range" id="susptravel" min="-100" max="100" step="2.5" value="${t}"> <b>${fmt(t, 1)}</b> mm`)}</div>
    ${res.notes.map((n) => `<p class="bad">${esc(n)}</p>`).join('')}${general}
    <details open><summary><b>Static geometry</b></summary><div class="kv">
      ${bs ? row('', `SVJ <span style="color:#e0782a">· base</span>`) : ''}
      ${row('Camber', d(st.camber_deg, 2, '°', 'camber_deg'))}${row('Toe', d(st.toe_deg, 2, '°', 'toe_deg'))}
      ${row('Track', d(st.track_mm, 0, ' mm', 'track_mm'))}
      ${row('Kingpin inclination', d(st.kpi_deg, 1, '°', 'kpi_deg'))}${row('Caster', d(st.caster_deg, 1, '°', 'caster_deg'))}
      ${row('Scrub radius', d(st.scrub_radius_mm, 0, ' mm', 'scrub_radius_mm'))}${row('Mechanical trail', d(st.trail_mm, 0, ' mm', 'trail_mm'))}
      ${row('Roll-centre height', d(st.roll_centre_mm, 0, ' mm', 'roll_centre_mm'))}
      ${row('Motion ratio', d(st.motion_ratio, 2, '', 'motion_ratio'))}${row('Wheel rate', d(st.wheel_rate_N_per_mm, 1, ' N/mm'))}
      ${row('Travel bump / rebound', `${d0(st.bump_mm, 0, '')} / ${d0(st.rebound_mm, 0, ' mm')}`)}
    </div></details>
    ${moving ? `<details open><summary><b>Over wheel travel</b> <span class="q">(blue band: damper within its stroke)</span></summary>
      ${chart('Camber', c.travel_mm, c.camber_deg, '°', t, usable, 2, sec('camber_deg'))}
      ${chart('Toe, + = in (bump steer)', c.travel_mm, c.toe_deg, '°', t, usable, 2, sec('toe_deg'))}
      ${chart('Roll-centre height (heave)', c.travel_mm, c.roll_centre_mm, 'mm', t, usable, 0, sec('roll_centre_mm'))}
      ${c.axle_shift_mm ? chart('Axle sideways shift', c.travel_mm, c.axle_shift_mm, 'mm', t, usable, 1, sec('axle_shift_mm')) : chart('Track change (both sides)', c.travel_mm, c.track_change_mm, 'mm', t, usable, 1, sec('track_change_mm'))}
      ${chart('Motion ratio', c.travel_mm, c.motion_ratio, '', t, usable, 3, sec('motion_ratio'))}
      ${chart('Wheel rate', c.travel_mm, c.wheel_rate_N_per_mm, 'N/mm', t, usable, 1)}</details>` : '<p class="quiet">The corner does not move: no curves.</p>'}
    ${bres ? `<p class="quiet"><span style="color:#e0782a">Base</span>: the SVJ's layout solved on the base vehicle's nodes tied to its hardpoints, before the fit, so the two show what the fit changes. Where the base's own layout differs (another rear suspension type), the base values are an approximation.${bres.notes.length ? ' ' + bres.notes.map(esc).join(' ') : ''}</p>`
      : (svjSusp.base ? '<p class="quiet">No base comparison for this corner (no tied points).</p>' : '<p class="quiet">Run the fit (stage 3) to compare with the base vehicle.</p>')}
    <p class="quiet">Kinematics with the steering straight ahead; the right side is the mirror image. The base vehicle's own suspension comes once its nodes are tied to these hardpoints (docs/fitting.md, stage 3).</p>`;
}

function bindSuspPanel() {
  document.querySelectorAll('[data-sc]').forEach((b) => b.onclick = () => { suspCorner = b.dataset.sc; drawInspector(); });
  const r = $('susptravel');
  if (!r) return;
  r.oninput = () => { suspTravel = +r.value; r.nextElementSibling.textContent = fmt(suspTravel, 1); drawSuspension(); };
  r.onchange = () => { drawInspector(); };
}

// ---------- a new vehicle from the edited one, as a mod (beamforge/export.py) ----------
// exportForm: the new vehicle's id (folder), name, brand and the per-part choices ({part: 'reuse' | 'copy' |
// 'fit'}; parts not set take the proposal of exppy.plan); exportOpen keeps the section open across redraws.
let exportForm = { id: '', name: '', brand: '', choices: {}, svj: true, replace: true, attach: {} }, exportNote = '', exportOpen = false;
// values taken from the SVJ (springs, dampers, tyres): {row key: true | false}, all taken unless unticked
let takeValues = {};
const CHOICE = { fit: 'with the edits', copy: 'as it is', reuse: 'reuse (not copied)' };

function exportPanel() {
  if (!veh) return '';
  let plan = [];
  try { plan = JSON.parse(exppy.plan(veh.model, JSON.stringify(veh))); } catch (err) { return `<p class="bad">${esc(pyError(err))}</p>`; }
  const v = cat && cat.find((x) => x.model === veh.model);
  const id = exportForm.id || `${veh.model}_bf`, name = exportForm.name || `${v ? v.name : veh.model} BeamForge`;
  const choice = (p) => exportForm.choices[p.part] || p.choice;
  const fromVeh = plan.filter((p) => p.origin === 'vehicle'), shared = plan.filter((p) => p.origin === 'common');
  const row = (p) => `<tr><td title="${esc(p.file)}">${esc(p.part)}</td><td>${p.moved_mm ? fmt(p.moved_mm, 0) + ' mm' : '–'}</td>
    <td><select data-expchoice="${esc(p.part)}">${p.choices.map((c) => `<option value="${c}" ${c === choice(p) ? 'selected' : ''}>${CHOICE[c]}</option>`).join('')}</select></td></tr>`;
  const count = (k) => plan.filter((p) => choice(p) === k).length;
  return `<details id="exportsec" ${exportOpen ? 'open' : ''}><summary><b>Make a new vehicle (mod)</b></summary>
    <p class="quiet">A new vehicle with its own folder, made from this one with its edits: the game and ${esc(veh.model)} are not changed.
      Put the zip in your user folder's <i>mods</i> folder.</p>
    <div class="kv">
      <span>Id (folder)</span><span><input id="expid" value="${esc(id)}" spellcheck="false"></span>
      <span>Name</span><span><input id="expname" value="${esc(name)}"></span>
      <span>Brand</span><span><input id="expbrand" value="${esc(exportForm.brand)}" placeholder="${esc(v?.brand || '')}"></span></div>
    <p class="quiet">${count('fit')} parts with the edits, ${count('copy')} copied as they are, ${count('reuse')} shared parts reused from the game.</p>
    <div class="inl"><button id="expall" title="Every part copied into the new vehicle with the edits, shared ones under new names: fully self-contained">Regenerate all</button>
      <button id="expreset">Proposed choices</button><button id="exprun" class="primary">Export mod (.zip)…</button></div>
    ${exportNote ? `<p class="quiet">${exportNote}</p>` : ''}
    ${svjMeshRows(plan)}
    <details><summary>Parts of ${esc(veh.model)} <span class="q">(${fromVeh.length}: always copied, the game cannot see them from a new folder)</span></summary>
      <table class="cmp"><tr><th>Part</th><th>Moved</th><th>Write</th></tr>${fromVeh.map(row).join('')}</table></details>
    <details><summary>Shared parts <span class="q">(${shared.length}, vehicles/common: reused, or regenerated under a new name)</span></summary>
      <table class="cmp"><tr><th>Part</th><th>Moved</th><th>Write</th></tr>${shared.map(row).join('')}</table></details>
  </details>`;
}

// the SVJ meshes for the new vehicle: where each binding goes (exppy.svj_attach), changed by the user in
// exportForm.attach ({path: {part, groups}})
let attachCache = { key: null, rows: [] };
function svjAttach() {
  if (!svjDoc || !veh) return [];
  const files = Object.fromEntries(svjDoc.meshes.filter((m) => m.file).map((m) => [m.id, m.file]));
  const { yf, zg } = svjPlace();
  const key = [svjDoc.summary?.vehicle, veh.model, veh.config, Object.keys(vehEdit.fit || {}).length, yf, zg, JSON.stringify(files)].join('|');
  if (attachCache.key !== key) {                     // reading the mesh for its panels is slow: once per fit
    try {                                            // with the files and place: the body split into its panels
      attachCache = { key, rows: JSON.parse(exppy.svj_attach(JSON.stringify(veh), JSON.stringify(svjDoc.svj),
        JSON.stringify(vehEdit.fitReport?.mapping || []), JSON.stringify(files), JSON.stringify({ yf, ground: zg }))) };
    } catch (err) { return []; }
  }
  return attachCache.rows.map((r) => ({ ...r, ...(exportForm.attach[r.path] || {}) }));
}

function svjMeshRows(plan) {
  if (!svjDoc) return '';
  const rows = svjAttach().filter((r) => svjDoc.meshes.some((m) => m.file && (m.id === r.mesh_ref || svjDoc.meshes.length === 1)));
  if (!rows.length) return '<p class="quiet">The SVJ has no meshes to add (no glTF bound to its chassis, corners or bodies).</p>';
  const parts = plan.filter((p) => p.origin === 'vehicle' || p.choice !== 'reuse').map((p) => p.part);
  return `<details open><summary><b>SVJ meshes</b> <span class="q">(written as COLLADA, the format of the game's vehicles)</span></summary>
    <div class="kv"><span><label><input type="checkbox" id="expsvj" ${exportForm.svj ? 'checked' : ''}> Add the SVJ meshes</label></span><span></span>
      <span><label title="Leave out the base vehicle's body meshes (body, panels, glass, lights); wheels, tyres, brakes, interior and the running gear stay">
        <input type="checkbox" id="expreplace" ${exportForm.replace ? 'checked' : ''}> Use the SVJ body instead of the base's</label></span><span></span></div>
    <table class="cmp"><tr><th>SVJ mesh</th><th>On part</th><th>Node groups</th></tr>
    ${rows.map((r) => `<tr><td title="${esc(r.node)}">${esc(r.path)}</td>
      <td><select data-attpart="${esc(r.path)}">${parts.map((p) => `<option ${p === r.part ? 'selected' : ''}>${esc(p)}</option>`).join('')}</select></td>
      <td><input data-attgroups="${esc(r.path)}" value="${esc((r.groups || []).join(', '))}" size="12"></td></tr>`).join('')}</table>
    <p class="quiet">Each mesh becomes a flexbody of its part and follows that part's node groups. A part reused from the game cannot take one: choose a copied part.</p></details>`;
}

function bindExport() {
  if (!$('exportsec')) return;
  if ($('expsvj')) $('expsvj').onchange = (e) => { exportForm.svj = e.target.checked; };
  if ($('expreplace')) $('expreplace').onchange = (e) => { exportForm.replace = e.target.checked; };
  document.querySelectorAll('[data-attpart]').forEach((sel) => sel.onchange = () => {
    (exportForm.attach[sel.dataset.attpart] ||= {}).part = sel.value;
  });
  document.querySelectorAll('[data-attgroups]').forEach((inp) => inp.onchange = () => {
    (exportForm.attach[inp.dataset.attgroups] ||= {}).groups = inp.value.split(',').map((g) => g.trim()).filter(Boolean);
  });
  $('exportsec').ontoggle = (e) => { exportOpen = e.target.open; };
  $('expid').onchange = (e) => { exportForm.id = e.target.value.trim(); };
  $('expname').onchange = (e) => { exportForm.name = e.target.value.trim(); };
  $('expbrand').onchange = (e) => { exportForm.brand = e.target.value.trim(); };
  document.querySelectorAll('[data-expchoice]').forEach((sel) => sel.onchange = () => { exportForm.choices[sel.dataset.expchoice] = sel.value; drawInspector(); });
  $('expall').onclick = () => {
    for (const p of JSON.parse(exppy.plan(veh.model, JSON.stringify(veh)))) exportForm.choices[p.part] = 'fit';
    drawInspector();
  };
  $('expreset').onclick = () => { exportForm.choices = {}; drawInspector(); };
  $('exprun').onclick = runExport;
}

// the winning blob getter of a path over every source, highest priority first (user folder, mods, game)
function blobOf(path) {
  const src = sourceByName();
  for (const n of [...resolved.order].reverse()) if (src[n] && src[n].blobs && src[n].blobs[path]) return src[n].blobs[path];
  return null;
}

async function runExport() {
  const id = ($('expid').value || '').trim(), name = ($('expname').value || '').trim() || id, brand = ($('expbrand').value || '').trim();
  exportForm.id = id; exportForm.name = name; exportForm.brand = brand;
  const problems = JSON.parse(exppy.check_id_json(id, veh.model));
  if (problems.length) { exportNote = `<span class="bad">${problems.map(esc).join('; ')}</span>`; drawInspector(); return; }
  const say = (t) => { exportNote = esc(t); drawInspector(); };
  try {
    say('Writing the parts…');
    await new Promise((r) => setTimeout(r, 20));
    let svjOpt = null;
    if (svjDoc && exportForm.svj) {
      const files = Object.fromEntries(svjDoc.meshes.filter((m) => m.file).map((m) => [m.id, m.file]));
      const { yf, zg } = svjPlace();
      svjOpt = { svj: svjDoc.svj, files, place: { yf, ground: zg }, attach: svjAttach(), replace: exportForm.replace };
      say('Writing the SVJ meshes…');
      await new Promise((r) => setTimeout(r, 20));
    }
    if (svjDoc) {                                     // the SVJ values taken (springs, dampers, tyres)
      let rows = [];
      try { rows = JSON.parse(valpy.table(veh.model, JSON.stringify(veh), JSON.stringify(svjDoc.svj), JSON.stringify({ corners: svjSusp?.corners || {} }))); } catch (err) { rows = []; }
      const take = Object.fromEntries(rows.filter((r) => r.svj !== null && takeValues[r.key] !== false).map((r) => [r.key, true]));
      svjOpt = { ...(svjOpt || { svj: svjDoc.svj }), take, study: { corners: svjSusp?.corners || {} } };
    }
    {                                                 // engine, gears, final drive, steering: the Powertrain workspace's
      const pt = JSON.parse(ptpy.export_json(JSON.stringify(ptEdit), svjDoc ? JSON.stringify(svjDoc.svj) : null));
      if (Object.values(pt.take).some(Boolean) || svjOpt) {
        svjOpt = { ...(svjOpt || {}), svj: pt.svj, take: { ...((svjOpt && svjOpt.take) || {}), ...pt.take } };
      }
    }
    if (aeroEdit && aeroEdit.cd && aeroEdit.area) {        // the drag: the Aero workspace's (lift is not written)
      svjOpt = svjOpt || { svj: {}, take: {} };
      svjOpt.svj = { ...svjOpt.svj, aerodynamics: { reference: { frontal_area: aeroEdit.area }, coefficients: { Cd: aeroEdit.cd } } };
      svjOpt.take = { ...(svjOpt.take || {}), aero: true };
    } else if (svjOpt && svjOpt.take) svjOpt.take.aero = false;
    if (svjOpt && Object.keys(vehEdit.fit || {}).length && vehEdit.fitReport?.axles) {
      svjOpt.axles = vehEdit.fitReport.axles;        // the wheels' camber and toe, set by preloading the upright's beams
    }
    const out = JSON.parse(exppy.build(veh.model, id, name, JSON.stringify(veh), JSON.stringify(exportForm.choices), brand || null,
      svjOpt ? JSON.stringify(svjOpt) : null, (vehEdit.beams || []).length ? JSON.stringify(vehEdit.beams) : null));
    const all = new Set();
    for (const s of allSources()) for (const p of Object.keys(s.blobs || {})) if (p.startsWith(`vehicles/${veh.model}/`)) all.add(p);
    const copies = JSON.parse(exppy.assets(veh.model, id, JSON.stringify([...all])));
    const zw = new ZipWriter(new BlobWriter('application/zip'));   // default level: zip.js 2.7 flags entries as encrypted below level 4
    for (const [p, text] of Object.entries(out.files)) await zw.add(p, new TextReader(text));
    for (const [p, b64] of Object.entries(out.binary || {})) {      // the SVJ mesh textures
      const bin = atob(b64), u8 = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
      await zw.add(p, new BlobReader(new Blob([u8])));
    }
    let i = 0;
    const n = Object.keys(copies).length;
    for (const [src, dst] of Object.entries(copies)) {
      say(`Copying ${++i} of ${n}: ${src.split('/').pop()}`);
      const get = blobOf(src);
      if (get) await zw.add(dst, new BlobReader(await get()));
    }
    say('Packing the zip…');
    const blob = await zw.close();
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `${id}.zip`;
    a.click();
    const c = out.counts;
    exportNote = `${esc(id)}.zip: ${c.fitted} parts with the edits (${c.nodes} nodes), ${c.copied} copied, ${c.regenerated} shared parts regenerated, ${c.reused} reused, ${n} meshes, materials and other files copied${c.values ? `, SVJ values written into ${c.values} parts` : ''}${c.svj_meshes ? `, ${c.svj_meshes} SVJ meshes added` : ''}${c.base_meshes_dropped ? ` (${c.base_meshes_dropped} body meshes of the base left out)` : ''} (${fmt(blob.size / 1e6, 1)} MB). Put it in your user folder's mods folder; the new vehicle is "${esc(name)}".`
      + (out.notes.length ? ` <span class="bad">${out.notes.map(esc).join(' ')}</span>` : '');
  } catch (err) { exportNote = `<span class="bad">Export failed: ${esc(pyError(err))}</span>`; }
  drawInspector();
}

// ---------- panels ----------
function drawInspector() {
  const el = $('inspector');
  const html = {
    base: () => vehInspector(),
    svj: () => svjInspector(),
    sketch: () => skeletonPanel(),
    suspension: () => suspPanel() || `<h2>Suspension</h2><p class="quiet">Import an SVJ to study its corners over wheel travel:
      camber, toe, roll centre, motion ratio, with the linkage moving in the view.</p>`,
    checks: () => checksPanel(),
    powertrain: () => powertrainPanel(),
    wheels: () => wheelsPanel(),
    aero: () => aeroPanel(),
    components: () => componentsPanel(),
    assembly: () => assemblyPanel(),
  }[ws]();
  el.innerHTML = html;
  if (ws === 'base') bindVehInspector(el);
  if (ws === 'svj') bindFitPanel();
  if (ws === 'sketch') bindSkeleton(el);
  if (ws === 'suspension') bindSuspPanel();
  if (ws === 'checks') bindChecks(el);
  if (ws === 'powertrain') bindPowertrain(el);
  if (ws === 'wheels') bindWheels(el);
  if (ws === 'aero') bindAero(el);
  if (ws === 'components') bindComponents(el);
  if (ws === 'assembly') { bindExport(); bindAssembly(el); }
  el.querySelectorAll('[data-goto]').forEach((b) => { b.onclick = () => setWorkspace(b.dataset.goto); });
}

function drawBudget() {
  if (!veh) { $('budget').innerHTML = '<div><span>Base vehicle</span><b>none open</b></div>'; return; }
  const count = (n) => (n.part ? 1 : 0) + n.children.reduce((s, c) => s + count(c), 0);
  const m = veh.measure || {};
  $('budget').innerHTML = `<div><span>Vehicle</span><b>${esc(veh.model)}</b> <span>${esc(veh.config || '')}</span></div>
    <div><span>Parts</span><b>${count(veh.tree)}</b></div>
    <div><span>Nodes / beams</span><b>${Object.keys(veh.geometry.nodes).length} / ${veh.geometry.beams.length}</b></div>
    <div><span>Wheelbase</span><b>${m.wheelbase ? fmt(m.wheelbase * 1000, 0) + ' mm' : '–'}</b></div>
    <div><span>Tuning variables</span><b>${veh.variables.length}</b> <span>${Object.keys(vehEdit.vars).length} changed</span></div>
    <div><span>Moved</span><b>${moveCount()}</b> <span>parts and nodes</span></div>
    ${Object.keys(vehEdit.fit || {}).length ? `<div><span>Fitted to the SVJ</span><b>${Object.keys(vehEdit.fit).length}</b> <span>nodes</span></div>` : ''}`;
}

function issues() {
  if (vehError) return [{ level: 'ERROR', rule: 'vehicle', message: vehError }];
  if (!veh) return [{ level: 'INFO', rule: 'vehicle', message: 'Add your BeamNG folders, then pick a vehicle to use it as a base.' }];
  const out = veh.missing.map((m) => ({ level: 'WARN', rule: 'vehicle', message: `slot ${m[0]}: part ${m[1]} is not in the added folders (add the game install, and the mod that ships it)` }));
  if (meshes && meshes.missing.length) out.push({ level: 'INFO', rule: 'meshes', message: `${meshes.missing.length} meshes not found in any .dae (mods may ship only .cdae): ${meshes.missing.slice(0, 8).join(', ')}${meshes.missing.length > 8 ? '…' : ''}` });
  const changed = Object.keys(vehEdit.parts).length + Object.keys(vehEdit.vars).length + moveCount();
  out.push({ level: 'PASS', rule: 'vehicle', message: `${veh.model} · ${veh.config}: ${changed ? changed + ' change(s) from the configuration' : 'as configured'}. Save it as a .pc for BeamNG.` });
  return out;
}

function showIssues(list) {   // footer messages: [{ level, rule, message }]
  $('issues').innerHTML = list.map((f) =>
    `<div><span class="lvl ${f.level}">${f.level}</span><span class="quiet">[${f.rule}]</span> ${esc(f.message)}</div>`).join('');
}

function drawTiming() {
  const parts = [];
  if (timing.pyodide) parts.push(`Python engine ${fmt(timing.pyodide / 1000, 1)} s`);
  if (timing.files) parts.push(`BeamForge code ${fmt(timing.files / 1000, 1)} s`);
  if (timing.configure) parts.push(`last configure ${fmt(timing.configure, 0)} ms`);
  if (timing.skeleton && skel) parts.push(`skeleton ${fmt(timing.skeleton, 0)} ms`);
  if (timing.checks) parts.push(`checks ${fmt(timing.checks / 1000, 1)} s`);
  if (veh && meshNote) parts.push(meshNote);
  $('timing').textContent = parts.join(' · ');
}

// ---------- Sketch (beamforge/skeleton.py): mechanisms and structures drawn as points and lines ----------
// A sketch is an assembly of parts, each a rigid body given by its lines: the line ends are its nodes, and it gets the
// beams that make it rigid (its lines, bending beams at a frame's welded corners, braces, helper nodes where it is flat
// or straight). Parts with an end at the same place share that node (1 shared node: a ball joint, 2: a hinge, 3 or more:
// welded). Frame parts are welded tubes (FBeam's rule, beamforge/tubes.py), links rigid bodies. Beam values are stable
// by construction (rigidity.beam_values). Points are kept in the SVJ frame (X forward, Y right, Z down, origin on the
// ground under the front axle; shown in mm) and drawn on the base vehicle like an SVJ.
// A STEP import fills a sketch; the tools draw and change it; it saves as STEP again and lives in the project.
//   skel    { name, parts: [{ name, lines: [[a, b]], points: [p] }] (SVJ frame, m), opts: { tol, kinds, tubes, min_kg },
//             active (the part new lines go into), source (what the STEP import reported), res (the build), error }
//   skTool  'select' | 'line' | 'point';  skSel: { kind: 'node', id, pos } | { kind: 'line', part, i } | null
//   skStart the first end of the line being drawn (SVJ frame, m); a click on a snap target ends it and starts the next
let skel = null, skTool = 'select', skSel = null, skStart = null;
const skSnap = { sketch: true, vehicle: true, svj: true };
const skUndo = [], skRedo = [];
const SKEL_KIND = { line: 0xe0782a, bend: 0x2da44e, brace: 0x2f81f7, helper: 0x9aa4ae };
const SKEL_BAND = { ok: 0x2da44e, high: 0xd4a72c, extreme: 0xe0782a, beyond: 0xcf222e };
const PART_NAMES = ['frame', 'subframe', 'upright', 'hub', 'upper_wishbone', 'lower_wishbone', 'upper_link_front', 'upper_link_rear',
  'lower_link_front', 'lower_link_rear', 'trailing_arm', 'semi_trailing_arm', 'leading_arm', 'tie_rod', 'toe_link', 'camber_link',
  'drag_link', 'pushrod', 'pullrod', 'rocker', 'strut', 'torque_rod', 'panhard_rod', 'watts_link_rod', 'watts_pivot', 'axle_body',
  'spring', 'damper', 'arb', 'drop_link', 'rack'];

const skelPlace = () => { const m = (veh && veh.measure) || {}; return { yf: m.front_axle_y || 0, ground: m.ground_z || 0 }; };
const saeToBng = (p, pl = skelPlace()) => [-p[1], pl.yf - p[0], pl.ground - p[2]];     // svj.from_sae
const bngToSae = (p, pl = skelPlace()) => [pl.yf - p[1], -p[0], pl.ground - p[2]];     // svj.to_sae
const toMm = (x) => Math.round(x * 10000) / 10;
const same = (a, b, tol) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) <= tol;
const skState = () => JSON.stringify({ parts: skel.parts, active: skel.active, opts: skel.opts, name: skel.name });

function newSketch(name, parts = []) {
  skel = { name, parts, opts: { tol: 0.002, kinds: {}, tubes: {}, min_kg: 1 }, active: parts.length ? parts[0].name : null,
    source: null, res: null, error: null };
  skUndo.length = 0; skRedo.length = 0; skSel = null; skStart = null;
}

// an edit of the sketch: undoable, then built and drawn again
function skEdit(fn) {
  skUndo.push(skState());
  if (skUndo.length > 200) skUndo.shift();
  skRedo.length = 0;
  fn();
  buildSkeleton();
  redraw();
}

function skUndoRedo(from, to) {
  if (!skel || !from.length) return;
  to.push(skState());
  Object.assign(skel, JSON.parse(from.pop()));
  skSel = null; skStart = null;
  buildSkeleton();
  redraw();
}

function buildSkeleton() {
  if (!skel) return;
  const t0 = performance.now();
  try {
    const any = skel.parts.some((p) => p.lines.length || (p.points || []).length);
    skel.res = any ? JSON.parse(skpy.build_parts_json(JSON.stringify(skel.parts), JSON.stringify({ ...skel.opts, ...skelPlace() }))) : null;
    skel.error = null;
  } catch (e) {
    skel.res = null;
    skel.error = pyError(e);
  }
  timing.skeleton = performance.now() - t0;
}

function activePart(create = true) {
  let p = skel.parts.find((x) => x.name === skel.active);
  if (!p && create) {
    let k = 1;
    while (skel.parts.some((x) => x.name === `part_${k}`)) k++;
    p = { name: `part_${k}`, lines: [], points: [] };
    skel.parts.push(p);
    skel.active = p.name;
  }
  return p;
}

function drawSkeleton() {
  skelG.clear();
  $('skeltoggle').hidden = !skel;
  if (!skel) return;
  const N = skel.res ? skel.res.nodes : {};
  if (skel.res) {
    for (const kind of Object.keys(SKEL_KIND)) {
      const pts = [];
      for (const b of skel.res.beams) if (b.kind === kind) pts.push(v3(N[b.a].bng), v3(N[b.b].bng));
      if (!pts.length) continue;
      skelG.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts),
        new THREE.LineBasicMaterial({ color: SKEL_KIND[kind], transparent: kind !== 'line', opacity: kind === 'line' ? 1 : 0.7 })));
    }
  }
  const act = skel.parts.find((x) => x.name === skel.active);       // the active part's lines on top, brighter
  if (act && act.lines.length) {
    skelG.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(act.lines.flatMap(([a, b]) => [v3(saeToBng(a)), v3(saeToBng(b))])),
      new THREE.LineBasicMaterial({ color: 0xffb35c, depthTest: false })));
  }
  const geo = new THREE.SphereGeometry(0.012, 10, 6);
  for (const [id, n] of Object.entries(N)) {
    const color = n.reference ? 0x8250df : n.helper ? 0x9aa4ae : SKEL_BAND[n.band] || 0x57606a;
    const s = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color }));
    s.position.copy(v3(n.bng));
    skelG.add(s);
  }
  const mark = (pos, color, size) => {
    const s = new THREE.Mesh(new THREE.SphereGeometry(size, 14, 10), new THREE.MeshBasicMaterial({ color, depthTest: false }));
    s.position.copy(v3(saeToBng(pos)));
    s.renderOrder = 10;
    skelG.add(s);
  };
  if (skSel && skSel.kind === 'node') mark(skSel.pos, 0xcf222e, 0.022);
  if (skSel && skSel.kind === 'line') {
    const p = skel.parts.find((x) => x.name === skSel.part), l = p && p.lines[skSel.i];
    if (l) {
      skelG.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([v3(saeToBng(l[0])), v3(saeToBng(l[1]))]),
        new THREE.LineBasicMaterial({ color: 0xcf222e, depthTest: false })));
    }
  }
  if (skStart) mark(skStart, 0xf2cc60, 0.02);
  skelG.visible = $('showskel').checked;
}

// the snap target nearest the pointer on screen (within 10 px): a sketch node, an SVJ hardpoint, a vehicle node; else,
// for the select tool, the sketch line nearest within 6 px
function skelPick(mx, my, r, lines) {
  const p = new THREE.Vector3();
  const scr = (bng) => { p.copy(v3(bng)).project(camera); return [(p.x + 1) / 2 * r.width, (1 - p.y) / 2 * r.height, p.z]; };
  let best = null, bd = 10;
  const consider = (bng, hit) => {
    const s = scr(bng);
    if (s[2] > 1) return;
    const d = Math.hypot(s[0] - mx, s[1] - my);
    if (d < bd) { bd = d; best = hit; }
  };
  if (skSnap.sketch && skel.res) for (const [id, n] of Object.entries(skel.res.nodes)) if (!n.helper) consider(n.bng, { kind: 'node', id, pos: n.pos, label: id });
  if (skSnap.svj) for (const h of svjHp) consider(h.pos, { kind: 'point', pos: bngToSae(h.pos), label: `${h.corner} ${h.name}` });
  if (skSnap.vehicle && veh && vehG.visible) for (const id of nodeIds) consider(veh.geometry.nodes[id], { kind: 'point', pos: bngToSae(veh.geometry.nodes[id]), label: id });
  if (best || !lines) return best;
  let bl = null, bld = 6;
  skel.parts.forEach((part) => part.lines.forEach(([a, b], i) => {
    const A = scr(saeToBng(a)), B = scr(saeToBng(b));
    if (A[2] > 1 || B[2] > 1) return;
    const dx = B[0] - A[0], dy = B[1] - A[1], L = dx * dx + dy * dy;
    const t = L ? Math.max(0, Math.min(1, ((mx - A[0]) * dx + (my - A[1]) * dy) / L)) : 0;
    const d = Math.hypot(A[0] + t * dx - mx, A[1] + t * dy - my);
    if (d < bld) { bld = d; bl = { kind: 'line', part: part.name, i }; }
  }));
  return bl;
}

function sketchClick(e) {
  const r = renderer.domElement.getBoundingClientRect(), mx = e.clientX - r.left, my = e.clientY - r.top;
  const hit = skelPick(mx, my, r, skTool === 'select');
  if (skTool === 'select') {
    skSel = hit ? (hit.kind === 'point' ? null : hit) : null;
    if (hit && hit.kind === 'line') skel.active = hit.part;
    drawSkeleton(); drawInspector();
    return;
  }
  if (!hit) { showIssues([{ level: 'INFO', rule: 'sketch', message: 'Click a node, an SVJ hardpoint or a vehicle node (or type the coordinates in the panel).' }]); return; }
  if (skTool === 'point') { skEdit(() => { activePart().points.push(hit.pos.map((x) => +x.toFixed(6))); }); return; }
  if (!skStart) { skStart = hit.pos.slice(); drawSkeleton(); drawInspector(); return; }
  if (same(skStart, hit.pos, 1e-6)) return;
  const a = skStart.map((x) => +x.toFixed(6)), b = hit.pos.map((x) => +x.toFixed(6));
  skEdit(() => { activePart().lines.push([a, b]); skStart = b; });
}

// a node moved: every line end and point of every part at its place follows (the node is shared)
function moveSketchNode(from, to) {
  const tol = skel.opts.tol;
  skEdit(() => {
    for (const p of skel.parts) {
      p.lines = p.lines.map(([a, b]) => [same(a, from, tol) ? to.slice() : a, same(b, from, tol) ? to.slice() : b]);
      p.points = (p.points || []).map((q) => (same(q, from, tol) ? to.slice() : q));
    }
    skSel = { kind: 'node', id: skSel.id, pos: to.slice() };
  });
}

function deleteSketchNode(pos) {
  const tol = skel.opts.tol;
  skEdit(() => {
    for (const p of skel.parts) {
      p.lines = p.lines.filter(([a, b]) => !same(a, pos, tol) && !same(b, pos, tol));
      p.points = (p.points || []).filter((q) => !same(q, pos, tol));
    }
    skSel = null;
  });
}

// turn (90 degrees about an axis), move or scale the active part (about its centre) or the whole sketch (about the origin)
function transformSketch(f, onlyActive) {
  skEdit(() => {
    const parts = onlyActive ? skel.parts.filter((p) => p.name === skel.active) : skel.parts;
    const all = parts.flatMap((p) => p.lines.flat().concat(p.points || []));
    const c = onlyActive && all.length ? [0, 1, 2].map((i) => all.reduce((s, q) => s + q[i], 0) / all.length) : [0, 0, 0];
    const g = (q) => { const d = f([q[0] - c[0], q[1] - c[1], q[2] - c[2]]); return d.map((x, i) => +(x + c[i]).toFixed(6)); };
    for (const p of parts) { p.lines = p.lines.map(([a, b]) => [g(a), g(b)]); p.points = (p.points || []).map(g); }
    skSel = null; skStart = null;
  });
}
const TURN = { x: ([x, y, z]) => [x, -z, y], y: ([x, y, z]) => [z, y, -x], z: ([x, y, z]) => [-y, x, z] };

function renamePart(old, name) {
  name = name.trim().replace(/[^\w]+/g, '_');
  if (!name || name === old || skel.parts.some((p) => p.name === name)) return;
  skEdit(() => {
    skel.parts.find((p) => p.name === old).name = name;
    for (const k of ['kinds', 'tubes']) if (old in skel.opts[k]) { skel.opts[k][name] = skel.opts[k][old]; delete skel.opts[k][old]; }
    if (skel.active === old) skel.active = name;
  });
}

function skeletonPanel() {
  if (!skel) return `<h2>Sketch</h2><p class="quiet">Draw mechanisms and structures as points and lines, or import a STEP assembly of
    wireframe parts. Each part is a rigid body; parts with an end at the same place share it.</p>
    <p><button id="sknew" class="primary">New sketch</button> <button id="skimport">Import STEP…</button></p>`;
  const r = skel.res, o = skel.opts;
  const xyz = (id, q) => ['X', 'Y', 'Z'].map((a, i) => `<input type="number" step="1" id="${id}${i}" value="${q ? toMm(q[i]) : ''}" title="${a} (mm)">`).join('');
  const tools = [['select', 'Select', 'Click a node or a line to see and change it (V)'], ['line', 'Line', 'Click two points: a node, an SVJ hardpoint or a vehicle node; it goes on from the last end (L, Esc to stop)'],
    ['point', 'Point', 'Click a point to add it to the active part as a reference point (P)']];
  const act = skel.parts.find((p) => p.name === skel.active);
  let sel = '';
  if (skSel && skSel.kind === 'node') {
    const users = skel.parts.filter((p) => p.lines.some(([a, b]) => same(a, skSel.pos, o.tol) || same(b, skSel.pos, o.tol))).map((p) => p.name);
    sel = `<h3>Node ${esc(skSel.id)}</h3><div class="xyz">${xyz('sknode', skSel.pos)} <span class="q">mm</span></div>
      <p class="quiet">${users.length > 1 ? `Shared by ${users.map(esc).join(', ')}: ${users.length === 2 ? 'a joint' : 'a joint of several parts'}.` : `In ${esc(users[0] || '–')}.`}</p>
      <div class="inl"><button id="sknodemove" class="mini">Move here</button><button id="sknodestart" class="mini">Start a line here</button><button id="sknodedel" class="mini">Delete the node</button></div>`;
  } else if (skSel && skSel.kind === 'line') {
    const l = (skel.parts.find((p) => p.name === skSel.part) || { lines: [] }).lines[skSel.i];
    if (l) sel = `<h3>Line of ${esc(skSel.part)}</h3><div class="kv"><span>From</span><span>${l[0].map(toMm).join(' / ')} mm</span><span>To</span><span>${l[1].map(toMm).join(' / ')} mm</span>
      <span>Length</span><span>${fmt(Math.hypot(l[1][0] - l[0][0], l[1][1] - l[0][1], l[1][2] - l[0][2]) * 1000, 1)} mm</span></div>
      <div class="inl"><button id="sklinedel" class="mini">Delete the line</button></div>`;
  }
  let report = '';
  if (skel.error) report = `<p class="bad">${esc(skel.error)}</p>`;
  else if (r) {
    const rep = r.report;
    report = `<div class="kv"><span>Nodes</span><span>${rep.nodes} <span class="q">${rep.helper_nodes} helper, ${rep.splits} lines split</span></span>
        <span>Beams</span><span>${rep.beams} <span class="q">${rep.bending} bending, ${rep.braces} braces</span></span>
        <span>Rigid bodies</span><span>${rep.bodies}</span>
        <span>Joints</span><span>${rep.joints.ball} ball, ${rep.joints.hinge} hinge, ${rep.joints.weld} weld</span>
        <span>Free motions</span><span>${rep.free_motions} <span class="q">${rep.mechanism === null ? '' : `6 of the whole, ${rep.mechanism} of the mechanism`}</span></span>
        <span>Mass</span><span>${fmt(rep.node_mass, 1)} kg <span class="q">tubes ${fmt(rep.tube_mass, 1)} kg</span></span>
        <span>Stability</span><span>${Object.entries(rep.bands || {}).filter(([, n]) => n).map(([b, n]) => `<span class="band ${b}">${n} ${b}</span>`).join(' ')}</span></div>
      ${rep.not_rigid.length ? `<p class="bad">Not rigid: ${esc(rep.not_rigid.map((p) => p.join('+')).join(', '))}</p>` : ''}
      <details><summary>Joints</summary><div class="kv">${r.joints.map((j) => `<span>${esc(j.parts[0].join('+'))} – ${esc(j.parts[1].join('+'))}</span><span>${esc(j.type)}</span>`).join('') || '<span>none</span><span></span>'}</div></details>`;
  }
  const info = (pn) => (r && r.parts.find((x) => x.name === pn)) || {};
  const src = skel.source ? `<p class="quiet">From a STEP in ${esc(skel.source.unit_name)}${Object.keys(skel.source.skipped || {}).length ? `; not read: ${esc(Object.entries(skel.source.skipped).map(([k, n]) => `${n} ${k}`).join(', '))}` : ''}.</p>` : '';
  return `<h2>Sketch <input id="skname" value="${esc(skel.name)}" size="16"></h2>
    <div class="inl"><button id="skundo" class="mini" ${skUndo.length ? '' : 'disabled'} title="Ctrl+Z">undo</button><button id="skredo" class="mini" ${skRedo.length ? '' : 'disabled'} title="Ctrl+Y">redo</button>
      <button id="sknew" class="mini">new</button><button id="skimport" class="mini">import STEP</button><button id="skstep" class="mini">save STEP</button>
      <button id="skjbeam" class="mini" ${r ? '' : 'disabled'}>jbeam</button><button id="skclose" class="mini">close</button></div>
    ${src}
    <h3>Draw</h3>
    <div class="inl">${tools.map(([k, l, t]) => `<button class="${skTool === k ? 'on' : ''}" data-sktool="${k}" title="${esc(t)}">${l}</button>`).join('')}</div>
    <p class="quiet">Snap to <label><input type="checkbox" data-sksnap="sketch" ${skSnap.sketch ? 'checked' : ''}> sketch nodes</label>
      <label><input type="checkbox" data-sksnap="svj" ${skSnap.svj ? 'checked' : ''}> SVJ hardpoints</label>
      <label><input type="checkbox" data-sksnap="vehicle" ${skSnap.vehicle ? 'checked' : ''}> vehicle nodes</label></p>
    ${skStart ? `<p class="quiet">Drawing from ${skStart.map(toMm).join(' / ')} mm: click the next point. <button id="skstop" class="mini">stop (Esc)</button></p>` : ''}
    <details ${skStart || skTool === 'line' ? 'open' : ''}><summary>Type a line (mm, X forward, Y right, Z down)</summary>
      <div class="xyz"><span class="q">from</span>${xyz('skfrom', skStart || (skSel && skSel.kind === 'node' ? skSel.pos : null))}</div>
      <div class="xyz"><span class="q">to</span>${xyz('skto', null)}</div>
      <button id="skaddline" class="mini">Add to ${esc(act ? act.name : 'a new part')}</button></details>
    ${sel}
    <h3>Parts</h3>
    <table class="cmp"><tr><th></th><th>Part</th><th>Kind</th><th>Tube</th><th>kg</th><th></th></tr>
      ${skel.parts.map((p) => { const x = info(p.name); return `<tr class="${p.name === skel.active ? 'on' : ''}">
        <td><input type="radio" name="skactive" data-skactive="${esc(p.name)}" ${p.name === skel.active ? 'checked' : ''} title="New lines go into the active part"></td>
        <td><input data-skrename="${esc(p.name)}" value="${esc(p.name)}" size="13" list="skpartnames" title="${p.lines.length} lines, ${(p.points || []).length} points${x.role ? '; ' + x.role : ''}"></td>
        <td><select data-skkind="${esc(p.name)}">${['frame', 'link'].map((k) => `<option ${(x.kind || o.kinds[p.name] || '') === k ? 'selected' : ''}>${k}</option>`).join('')}</select></td>
        <td><input data-sktube="${esc(p.name)}" value="${esc(x.tube || o.tubes[p.name] || '')}" size="12" title="tube|sqtube DxT material (mm): steel_1018, steel_4130n, al_6061_t6"></td>
        <td>${fmt(x.mass, 1)}</td>
        <td><button class="mini" data-skmirror="${esc(p.name)}" title="A mirror image on the other side (Y to -Y), named for that side">mirror</button><button class="mini" data-skdel="${esc(p.name)}">×</button></td></tr>`; }).join('')}</table>
    <div class="inl"><input id="sknewpart" list="skpartnames" placeholder="lower_wishbone_fl, frame…" size="18"><button id="skaddpart" class="mini">Add part</button></div>
    <datalist id="skpartnames">${PART_NAMES.flatMap((n) => ['frame', 'subframe', 'rack', 'watts_pivot'].includes(n) ? [n] : [n + '_fl', n + '_fr', n + '_rl', n + '_rr']).map((n) => `<option value="${n}">`).join('')}</datalist>
    <details><summary>Turn, move, scale</summary>
      <p><label><input type="checkbox" id="skonlyact" checked> only the active part (about its centre)</label></p>
      <div class="inl">${['x', 'y', 'z'].map((a) => `<button class="mini" data-skturn="${a}">turn 90° about ${a.toUpperCase()}</button>`).join('')}</div>
      <div class="xyz"><span class="q">move</span>${xyz('skmove', [0, 0, 0])}<button id="skmoveok" class="mini">move</button></div>
      <div class="inl"><span class="q">scale ×</span><input type="number" id="skscale" value="1" step="0.001" style="width:6em"><button id="skscaleok" class="mini">scale</button>
        <span class="q" title="A STEP read in the wrong unit: 0.001 (it was in mm, read as m), 1000, 25.4…">unit fix</span></div></details>
    <details><summary>Settings</summary><div class="kv">
      <span>Merge tolerance (mm)</span><span><input type="number" step="0.5" min="0.1" id="sktol" value="${o.tol * 1000}"></span>
      <span title="A node weighs its share of the tubes, but no less than this: heavier nodes allow stiffer beams">Minimum node mass (kg)</span><span><input type="number" step="0.1" min="0.1" id="skminkg" value="${o.min_kg}"></span></div></details>
    <h3>Structure</h3>
    ${report || '<p class="quiet">Draw a line to start.</p>'}
    <p class="quiet">Lines: orange the parts' (tubes), green bending beams at welded corners, blue braces, grey to helper nodes; the active part's lines light.
      Nodes: green, amber, orange, red by stiffness against vanilla cars' (Checks).</p>`;
}

function bindSkeleton(el) {
  const val = (id) => [0, 1, 2].map((i) => +$(id + i).value / 1000);
  const filled = (id) => [0, 1, 2].every((i) => $(id + i) && $(id + i).value !== '' && Number.isFinite(+$(id + i).value));
  if ($('sknew')) $('sknew').onclick = () => { newSketch('sketch'); setWorkspace('sketch'); };
  if ($('skimport')) $('skimport').onclick = () => $('stepfile').click();
  if (!skel) return;
  $('skundo').onclick = () => skUndoRedo(skUndo, skRedo);
  $('skredo').onclick = () => skUndoRedo(skRedo, skUndo);
  $('skname').onchange = () => { skel.name = $('skname').value.trim() || 'sketch'; redraw(); };
  $('skclose').onclick = () => { if (confirm('Close the sketch? It is not kept unless saved (project or STEP).')) { skel = null; skSel = null; skStart = null; redraw(); } };
  $('skstep').onclick = () => download(skel.name.replace(/\.(step|stp)$/i, '') + '.step', skpy.step_json(JSON.stringify(skel.parts)), 'application/step');
  $('skjbeam').onclick = skeletonJbeam;
  el.querySelectorAll('[data-sktool]').forEach((b) => { b.onclick = () => { skTool = b.dataset.sktool; skStart = null; redraw(); }; });
  el.querySelectorAll('[data-sksnap]').forEach((c) => { c.onchange = () => { skSnap[c.dataset.sksnap] = c.checked; }; });
  if ($('skstop')) $('skstop').onclick = () => { skStart = null; redraw(); };
  $('skaddline').onclick = () => {
    if (!filled('skfrom') || !filled('skto')) { showIssues([{ level: 'ERROR', rule: 'sketch', message: 'Both ends need X, Y and Z (mm).' }]); return; }
    const a = val('skfrom'), b = val('skto');
    if (same(a, b, 1e-6)) return;
    skEdit(() => { activePart().lines.push([a, b]); skStart = b; });
  };
  if ($('sknodemove')) $('sknodemove').onclick = () => { if (filled('sknode')) moveSketchNode(skSel.pos, val('sknode')); };
  if ($('sknodestart')) $('sknodestart').onclick = () => { skStart = skSel.pos.slice(); skTool = 'line'; redraw(); };
  if ($('sknodedel')) $('sknodedel').onclick = () => deleteSketchNode(skSel.pos);
  if ($('sklinedel')) $('sklinedel').onclick = () => skEdit(() => { skel.parts.find((p) => p.name === skSel.part).lines.splice(skSel.i, 1); skSel = null; });
  el.querySelectorAll('[data-skactive]').forEach((x) => { x.onchange = () => { skel.active = x.dataset.skactive; redraw(); }; });
  el.querySelectorAll('[data-skrename]').forEach((x) => { x.onchange = () => renamePart(x.dataset.skrename, x.value); });
  el.querySelectorAll('[data-skkind]').forEach((x) => { x.onchange = () => skEdit(() => { skel.opts.kinds[x.dataset.skkind] = x.value; delete skel.opts.tubes[x.dataset.skkind]; }); });
  el.querySelectorAll('[data-sktube]').forEach((x) => { x.onchange = () => skEdit(() => { skel.opts.tubes[x.dataset.sktube] = x.value.trim(); }); });
  el.querySelectorAll('[data-skmirror]').forEach((x) => { x.onclick = () => {
    const p = skel.parts.find((q) => q.name === x.dataset.skmirror);
    const m = JSON.parse(skpy.mirror_json(JSON.stringify(p)));
    while (skel.parts.some((q) => q.name === m.name)) m.name += '_2';
    skEdit(() => {
      skel.parts.push(m);
      for (const k of ['kinds', 'tubes']) if (p.name in skel.opts[k]) skel.opts[k][m.name] = skel.opts[k][p.name];
      skel.active = m.name;
    });
  }; });
  el.querySelectorAll('[data-skdel]').forEach((x) => { x.onclick = () => skEdit(() => {
    skel.parts = skel.parts.filter((p) => p.name !== x.dataset.skdel);
    if (skel.active === x.dataset.skdel) skel.active = skel.parts.length ? skel.parts[0].name : null;
    skSel = null;
  }); });
  $('skaddpart').onclick = () => {
    const name = $('sknewpart').value.trim().replace(/[^\w]+/g, '_');
    if (!name || skel.parts.some((p) => p.name === name)) return;
    skEdit(() => { skel.parts.push({ name, lines: [], points: [] }); skel.active = name; });
  };
  el.querySelectorAll('[data-skturn]').forEach((b) => { b.onclick = () => transformSketch(TURN[b.dataset.skturn], $('skonlyact').checked); });
  $('skmoveok').onclick = () => { const d = val('skmove'); transformSketch((q) => [q[0] + d[0], q[1] + d[1], q[2] + d[2]], $('skonlyact').checked); };
  $('skscaleok').onclick = () => { const k = +$('skscale').value; if (k > 0 && k !== 1) transformSketch((q) => q.map((x) => x * k), $('skonlyact').checked); };
  $('sktol').onchange = () => skEdit(() => { skel.opts.tol = Math.max(0.0001, (+$('sktol').value || 2) / 1000); });
  $('skminkg').onchange = () => skEdit(() => { skel.opts.min_kg = Math.max(0.1, +$('skminkg').value || 1); });
}

function download(name, text, type = 'application/json') {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type }));
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

function skeletonJbeam() {
  download(skel.name.replace(/\.(step|stp)$/i, '') + '.jbeam',
    skpy.jbeam_parts_json(JSON.stringify(skel.parts), JSON.stringify({ ...skel.opts, ...skelPlace() }), 'skeleton'));
}

$('stepin').onclick = () => $('stepfile').click();
$('stepfile').onchange = async (e) => {
  const f = e.target.files[0];
  e.target.value = '';
  if (!f) return;
  try {
    const imp = JSON.parse(skpy.import_json(await f.text(), '{}'));
    newSketch(f.name.replace(/\.(step|stp)$/i, ''), imp.parts);
    skel.source = { unit_name: imp.unit_name, skipped: imp.skipped, notes: imp.notes };
  } catch (err) { showIssues([{ level: 'ERROR', rule: 'sketch', message: `Could not read ${f.name}: ${pyError(err)}` }]); return; }
  buildSkeleton();
  ws = 'sketch';
  redraw();
  if (!veh) {                                   // nothing else in the view: aim at the sketch
    const box = new THREE.Box3().setFromObject(skelG);
    if (!box.isEmpty()) {
      const c = box.getCenter(new THREE.Vector3()), size = box.getSize(new THREE.Vector3()).length() || 1;
      controls.target.copy(c);
      camera.position.set(c.x + 0.8 * size, c.y + 0.5 * size, c.z - 0.75 * size);
    }
  }
};
$('showskel').onchange = () => { skelG.visible = $('showskel').checked; };

// keys in the Sketch workspace (not while typing in a field)
document.addEventListener('keydown', (e) => {
  if (ws !== 'sketch' || !skel || /^(INPUT|TEXTAREA|SELECT)$/.test(document.activeElement && document.activeElement.tagName)) return;
  const k = e.key.toLowerCase();
  if ((e.ctrlKey || e.metaKey) && k === 'z' && !e.shiftKey) { e.preventDefault(); skUndoRedo(skUndo, skRedo); }
  else if ((e.ctrlKey || e.metaKey) && (k === 'y' || (k === 'z' && e.shiftKey))) { e.preventDefault(); skUndoRedo(skRedo, skUndo); }
  else if (k === 'escape') { skStart = null; skSel = null; redraw(); }
  else if (k === 'delete' && skSel) { if (skSel.kind === 'node') deleteSketchNode(skSel.pos); else skEdit(() => { skel.parts.find((p) => p.name === skSel.part).lines.splice(skSel.i, 1); skSel = null; }); }
  else if (!e.ctrlKey && !e.metaKey && !e.altKey && ({ v: 'select', l: 'line', p: 'point' })[k]) { skTool = { v: 'select', l: 'line', p: 'point' }[k]; skStart = null; redraw(); }
});

// ---------- workspaces: one view, the panels of each use ----------
// The 3D view, the vehicle and everything loaded are shared; a workspace only chooses the panels (right) and what the
// left panel shows (the BeamNG folders and vehicles in Base, the project elsewhere).
const WORKSPACES = [
  ['base', 'Base vehicle', 'The BeamNG vehicle to start from: its parts, tuning and moves'],
  ['svj', 'SVJ', 'A Standard Vehicle JSON laid over the base: compare, fit, take its values'],
  ['sketch', 'Sketch', 'Mechanisms and structures from points and lines (a STEP assembly): rigid parts, joints, tubes'],
  ['suspension', 'Suspension', "The SVJ's corners over wheel travel"],
  ['wheels', 'Wheels', "Rims and tyres from the vanilla cars' archetypes, your values, the SVJ's Pacejka tyre as a benchmark"],
  ['components', 'Components', 'The masses the car carries (engine, fuel, driver, battery, ballast): its mass and centre of gravity'],
  ['aero', 'Aero', 'Drag and downforce: the base, the SVJ or your own numbers, the forces per axle at speed'],
  ['powertrain', 'Powertrain', 'Engine, gearbox, final drive and steering: the base\'s, the SVJ\'s or your own; the transmission parts'],
  ['checks', 'Checks', 'Stability and structure of the vehicle and the sketch'],
  ['assembly', 'Assembly', 'Everything together: the new vehicle to export'],
];
const WS_KEY = 'beamforge.workspace';
let ws = (() => { try { return localStorage.getItem(WS_KEY); } catch { return null; } })();
if (!WORKSPACES.some(([k]) => k === ws)) ws = 'base';

function drawSpaces() {
  const has = { base: !!veh, svj: !!svjDoc, sketch: !!skel, suspension: !!svjSusp,
    powertrain: !!(ptEdit.engine || ptEdit.gears || ptEdit.final_drive || ptEdit.steering_turns),
    wheels: !!(wheelsForm.front.rim !== '' || wheelsForm.rear.rim !== ''), aero: !!aeroEdit, components: compItems.length > 0 };
  const el = document.querySelector('.spaces');
  el.innerHTML = WORKSPACES.map(([k, label, tip]) =>
    `<button class="${k === ws ? 'on' : ''}" data-ws="${k}" title="${esc(tip)}">${esc(label)}${has[k] ? ' <span class="dot"></span>' : ''}</button>`).join('');
  el.querySelectorAll('[data-ws]').forEach((b) => { b.onclick = () => setWorkspace(b.dataset.ws); });
}

function setWorkspace(k) {
  ws = k;
  try { localStorage.setItem(WS_KEY, k); } catch { /* private mode */ }
  if (k === 'sketch' && skel) { $('showskel').checked = true; skelG.visible = true; }
  if (k === 'suspension' && svjSusp) { $('showsusp').checked = true; drawSuspension(); }
  redraw();
}

// ---------- Powertrain: engine, gearbox, final drive, steering ----------
// What the new vehicle gets, group by group: the base's (kept), the SVJ's (taken), or typed by hand. The values form an
// SVJ-shaped block (beamforge/powertrain.py) written by the same code as an SVJ's values: the torque curve into the
// engine part, the ratios into the gearbox (reverse kept), the final drive into the driven axle, the turns into the
// steering. The transmission itself (gearbox type, differential, transfer case...) is chosen among the base's parts.
const PT_EMPTY = { engine: null, gears: null, final_drive: null, steering_turns: null, diffs: null, touched: false };
const DIFF_TYPES = [['', 'keep'], ['open', 'open'], ['lsd_clutch', 'limited slip'], ['lsd_viscous', 'viscous'], ['locked', 'locked']];

// the archetypes nearest the SVJ's engine (its configuration, fuel and power) and gearbox (type and gears)
function suggestFromSvj() {
  if (!library || !svjDoc) return null;
  const pt = svjDoc.svj.powertrain || {}, eng = pt.engine || {}, gb = pt.gearbox || {};
  const cfg = String(eng.configuration || '').toUpperCase().replace('FLAT', 'B').replace('BOXER', 'B');
  const [, power] = peakOf(eng.torque_curve || []);
  const text = JSON.stringify(eng).toLowerCase();
  const fuel = /electric/.test(cfg.toLowerCase() + text) ? 'electricEnergy' : /diesel/.test(text) ? 'diesel' : 'gasoline';   // petrol unless it says
  let pool = library.archetypes.engines.filter((a) => a.pick.fuel === fuel);
  const same = pool.filter((a) => cfg && a.pick.cylinders === cfg.replace(/[^A-Z0-9]/g, ''));
  if (same.length) pool = same;
  // nearest in power; within that, the common groups (a typical engine of its kind) a little ahead
  const score = (a) => Math.abs(a.pick.peak_power - power) / Math.max(power, 1) - 0.03 * Math.log(a.count);
  const e = power && pool.length ? pool.reduce((b2, a) => (score(a) < score(b2) ? a : b2), pool[0]) : null;
  const want = { manual: 'manual', sequential: 'sequential', dct: 'dct', auto: 'automatic', automatic: 'automatic', cvt: 'cvt' }[String(gb.type || 'manual')] || 'manual';
  const n = (gb.ratios || []).length;
  const gbs = library.archetypes.gearboxes.filter((a) => a.pick.type === want);
  const g = (gbs.length ? gbs : library.archetypes.gearboxes).reduce((b, a) => (Math.abs(a.pick.gears - n) < Math.abs(b.pick.gears - n) ? a : b), (gbs.length ? gbs : library.archetypes.gearboxes)[0]);
  return { engine: e, gearbox: g };
}

// ---------- the vanilla cars' parts: engines, gearboxes, tyres and rims, grouped into archetypes (beamforge/donors.py)
// Learnt from the user's install (only the files of those parts are read), kept in this browser. An archetype points at
// a real part (model and part name) with its numbers; a choice remembers which, for a car made from scratch.
const LIBRARY_KEY = 'beamforge.library';
let library = (() => { try { return JSON.parse(localStorage.getItem(LIBRARY_KEY) || 'null'); } catch { return null; } })();
let libraryBusy = '';

async function learnLibrary() {
  if (!resolved) { showIssues([{ level: 'INFO', rule: 'library', message: 'Add your BeamNG folders first (Base vehicle).' }]); return; }
  const want = (p) => /^vehicles\/[^/]+\/.*\.jbeam$/i.test(p) && /engine|motor|transmission|transaxle|gearbox|tire|tyre|wheel|rim/i.test(p.split('/').pop());
  try {
    libraryBusy = 'Reading the engines, gearboxes, tyres and rims of every vehicle…'; drawInspector();
    const { texts, ranks } = await readResolved(want, libraryBusy);
    vehBusy = '';
    libraryBusy = `Learning from ${Object.keys(texts).length} files…`; drawInspector();
    await new Promise((r) => setTimeout(r, 30));
    vehpy.add_files(JSON.stringify(texts), JSON.stringify(ranks));
    const models = (cat || []).filter((v) => /car|truck/i.test(v.type || 'car')).map((v) => v.model);
    library = { at: new Date().toISOString(), vehicles: models.length, archetypes: JSON.parse(donpy.archetypes_json(JSON.stringify(models))) };
    try { localStorage.setItem(LIBRARY_KEY, JSON.stringify(library)); } catch { /* too large to keep: kept for this session */ }
    if (veh) configureVehicle();                     // the vehicle's own parts were read again with the rest
  } catch (err) { showIssues([{ level: 'ERROR', rule: 'library', message: pyError(err) }]); }
  libraryBusy = '';
  redraw();
}

function libraryPicker(kind, label, chosen) {
  const list = library && library.archetypes[kind];
  if (!list || !list.length) return '';
  return `<div class="inl"><select data-libpick="${kind}"><option value="">${esc(label)}…</option>
    ${list.map((a, i) => `<option value="${i}" ${chosen && chosen.part === a.pick.part && chosen.model === a.pick.model ? 'selected' : ''}>${esc(a.name)} · ${a.count} (${esc(a.pick.model)})</option>`).join('')}</select></div>`;
}
const PT_KEYS = ['engine', 'engine_power', 'gears', 'final_drive', 'steering', 'aero'];   // values their own workspaces set
let ptEdit = { ...PT_EMPTY }, ptEditing = null;     // ptEditing: the group whose hand fields are open

const peakOf = (curve) => {
  if (!curve || !curve.length) return [null, null];
  const t = Math.max(...curve.map((p) => p[1])), p = Math.max(...curve.map(([r, n]) => n * r * 2 * Math.PI / 60 / 1000));
  return [t, p];
};
const ratioText = (r) => r && r.length ? r.map((x) => fmt(x, 3)).join('  ') : '–';

function lineChart(title, series, xunit, yunit, digits = 0) {
  const all = series.flatMap((s) => s.pts);
  if (all.length < 2) return '';
  const W = 270, H = 120, L = 40, B = 18, T = 6;
  const x0 = Math.min(...all.map((p) => p[0])), x1 = Math.max(...all.map((p) => p[0]));
  let y0 = Math.min(0, ...all.map((p) => p[1])), y1 = Math.max(...all.map((p) => p[1]));
  if (y1 - y0 < 1e-9) y1 = y0 + 1;
  const X = (x) => L + (x - x0) / ((x1 - x0) || 1) * (W - L - 6), Y = (y) => T + (y1 - y) / (y1 - y0) * (H - T - B);
  const paths = series.filter((s) => s.pts.length > 1).map((s) => `<path d="${s.pts.map((p, i) => `${i ? 'L' : 'M'}${X(p[0]).toFixed(1)},${Y(p[1]).toFixed(1)}`).join('')}"
    fill="none" stroke="${s.color}" stroke-width="1.8" ${s.dash ? `stroke-dasharray="${s.dash}"` : ''}/>`).join('');
  return `<div class="chart"><div class="t"><span>${title}</span><span>${series.map((s) => `<span style="color:${s.color}">${esc(s.label)}</span>`).join(' ')}</span></div>
    <svg viewBox="0 0 ${W} ${H}" style="color:var(--ink)">${paths}
      <text x="${L - 4}" y="${Y(y1) + 8}" text-anchor="end" font-size="10" fill="currentColor" fill-opacity=".6">${fmt(y1, digits)}</text>
      <text x="${L - 4}" y="${Y(y0)}" text-anchor="end" font-size="10" fill="currentColor" fill-opacity=".6">${fmt(y0, digits)} ${esc(yunit)}</text>
      <text x="${L}" y="${H - 4}" font-size="10" fill="currentColor" fill-opacity=".6">${fmt(x0, 0)}</text>
      <text x="${W - 6}" y="${H - 4}" text-anchor="end" font-size="10" fill="currentColor" fill-opacity=".6">${fmt(x1, 0)} ${esc(xunit)}</text>
    </svg></div>`;
}

function powertrainPanel() {
  if (!veh && !svjDoc) return `<h2>Powertrain</h2><p class="quiet">Open a base vehicle (its engine, gearbox and steering are the start),
    or import an SVJ, to set the engine's torque curve, the gear ratios, the final drive and the steering.</p>`;
  let s;
  try { s = JSON.parse(ptpy.summary_json(veh ? veh.model : null, veh ? JSON.stringify(veh) : null, svjDoc ? JSON.stringify(svjDoc.svj) : null)); }
  catch (err) { return `<p class="bad">${esc(pyError(err))}</p>`; }
  const b = s.base || {}, v = s.svj || {};
  const eng = ptEdit.engine || (b.torque ? { torque: b.torque, idle_rpm: b.idle_rpm, max_rpm: b.max_rpm } : null);
  const gears = ptEdit.gears || b.ratios, fd = ptEdit.final_drive || b.final_drive, turns = ptEdit.steering_turns || b.turns;
  const btns = (k, svjHas) => `<span class="ptbtns"><button class="mini${ptEdit[k] ? '' : ' on'}" data-ptkeep="${k}" ${veh ? '' : 'disabled'}>base</button>
    <button class="mini" data-ptsvj="${k}" ${svjHas ? '' : 'disabled'}>SVJ</button><button class="mini${ptEditing === k ? ' on' : ''}" data-ptedit="${k}">edit</button></span>`;
  const [bt, bp] = peakOf(b.torque), [vt, vp] = peakOf(v.torque), [ct, cp] = peakOf(eng && eng.torque);
  const power = (c) => (c || []).map(([r, n]) => [r, n * r * 2 * Math.PI / 60 / 1000]);
  const series = (f) => [b.torque && { label: 'base', color: '#e0782a', dash: '4 3', pts: f(b.torque) },
    v.torque && v.torque.length && { label: 'SVJ', color: '#8250df', dash: '2 3', pts: f(v.torque) },
    eng && { label: 'new', color: '#2f6fdf', pts: f(eng.torque) }].filter(Boolean);
  const r = s.tyre_radius;
  const speeds = (g, f, maxr) => g && f && r && maxr ? g.map((x, i) => `${i + 1}: ${fmt(maxr / (x * f) * 2 * Math.PI * r * 60 / 1000, 0)}`).join(' · ') + ' km/h' : '–';
  const parts = s.slots.length ? `<details ${ptEditing === 'parts' ? 'open' : ''}><summary><b>Transmission and engine parts</b> <span class="q">(the base's, as in its Parts menu)</span></summary>
    <div class="kv">${s.slots.map((x) => `<span style="padding-left:${Math.max(0, x.depth - 1) * 8}px">${esc(x.description)}</span><span><select data-ptslot="${esc(x.slot)}">
      ${x.core ? '' : `<option value="" ${x.part ? '' : 'selected'}>(empty)</option>`}${x.options.map(([p, t]) => `<option value="${esc(p)}" ${p === x.part ? 'selected' : ''}>${esc(t || p)}</option>`).join('')}</select></span>`).join('')}</div></details>` : '';
  const engEdit = ptEditing === 'engine' ? `<div class="ptedit"><p class="quiet">Torque curve, one point per line: rpm and Nm.</p>
    <textarea id="pttorque" rows="8">${(eng ? eng.torque : []).map(([r_, n]) => `${r_} ${n}`).join('\n')}</textarea>
    <div class="kv"><span>Idle rpm</span><span><input type="number" id="ptidle" value="${eng && eng.idle_rpm || ''}"></span>
      <span>Max rpm</span><span><input type="number" id="ptmax" value="${eng && eng.max_rpm || ''}"></span></div>
    <button id="ptengok" class="primary">Use this curve</button></div>` : '';
  const gearEdit = ptEditing === 'gears' ? `<div class="ptedit"><div class="kv"><span>Forward ratios</span><span><input id="ptgears" value="${(gears || []).join(' ')}" size="24"></span>
    <span>Final drive</span><span><input type="number" step="0.01" id="ptfd" value="${fd || ''}"></span></div><button id="ptgearok" class="primary">Use these</button></div>` : '';
  const steerEdit = ptEditing === 'steering' ? `<div class="ptedit"><div class="kv"><span>Turns lock to lock</span><span><input type="number" step="0.05" id="ptturns" value="${turns || ''}"></span></div>
    <button id="ptsteerok" class="primary">Use this</button></div>` : '';
  const lib = library ? `<p class="quiet">From the vanilla cars: ${library.vehicles} vehicles, learnt ${new Date(library.at).toLocaleDateString()}.
      <button id="liblearn" class="mini">learn again</button></p>`
    : `<p class="quiet">Engines and gearboxes of every vanilla car can be offered as archetypes (a typical I4, a V8, a 6-speed manual…).
      <button id="liblearn" class="mini" ${resolved ? '' : 'disabled title="Add your BeamNG folders first"'}>Learn from the install</button></p>`;
  return `<h2>Powertrain</h2>
    ${libraryBusy ? `<p class="quiet">${esc(libraryBusy)}</p>` : lib}
    ${library && svjDoc ? '<p><button id="ptsuggest" class="mini" title="The engine archetype nearest the SVJ\'s (cylinders, fuel, power) and the gearbox of its type and number of gears">Suggest archetypes from the SVJ</button></p>' : ''}
    ${parts}
    <h3>Engine ${btns('engine', v.torque && v.torque.length)}</h3>
    ${libraryPicker('engines', 'An engine of the vanilla cars', ptEdit.engine && ptEdit.engine.source)}
    ${ptEdit.engine && ptEdit.engine.source ? `<p class="quiet">From ${esc(ptEdit.engine.source.title)} (${esc(ptEdit.engine.source.model)}, ${esc(ptEdit.engine.source.part)}).</p>` : ''}
    <table class="cmp"><tr><th></th><th>Base</th><th>SVJ</th><th>New</th></tr>
      <tr><td>Peak torque Nm</td><td>${fmt(bt, 0)}</td><td>${fmt(vt, 0)}</td><td><b>${fmt(ct, 0)}</b></td></tr>
      <tr><td>Peak power kW</td><td>${fmt(bp, 0)}</td><td>${fmt(vp, 0)}</td><td><b>${fmt(cp, 0)}</b></td></tr>
      <tr><td>Idle / max rpm</td><td>${fmt(b.idle_rpm, 0)} / ${fmt(b.max_rpm, 0)}</td><td>${fmt(v.idle_rpm, 0)} / ${fmt(v.max_rpm, 0)}</td><td><b>${fmt(eng && eng.idle_rpm, 0)} / ${fmt(eng && eng.max_rpm, 0)}</b></td></tr></table>
    ${engEdit}
    <table class="cmp"><tr><th></th><th>Base</th><th>SVJ</th><th>New</th></tr>
      ${[['inertia', 'Inertia kg·m²', 3], ['friction', 'Friction Nm', 1], ['engine_brake', 'Engine braking Nm', 1]].map(([k, label, d]) =>
        `<tr><td>${label}</td><td>${fmt(b[k], d)}</td><td>${k === 'inertia' ? fmt(v.inertia, d) : '–'}</td>
          <td><input type="number" step="any" data-pteng="${k}" value="${ptEdit.engine && ptEdit.engine[k] != null ? ptEdit.engine[k] : ''}" placeholder="${b[k] != null ? b[k] : ''}" style="width:6em"></td></tr>`).join('')}
      <tr><td title="A car from scratch: the engine block's node weights are scaled to it">Mass kg (scratch)</td><td>–</td><td>${fmt((svjDoc && svjDoc.svj.powertrain && (svjDoc.svj.powertrain.engine || {}).mass) || null, 0)}</td>
        <td><input type="number" step="any" data-pteng="mass" value="${ptEdit.engine && ptEdit.engine.mass != null ? ptEdit.engine.mass : ''}" style="width:6em"></td></tr></table>
    ${lineChart('Torque', series((c) => c), 'rpm', 'Nm')}
    ${lineChart('Power', series(power), 'rpm', 'kW')}
    ${b.turbo ? '<p class="quiet">The base has a turbo or supercharger: its boost comes on top of the curve, as in the game.</p>' : ''}
    <h3>Gearbox and final drive ${btns('gears', (v.ratios && v.ratios.length) || v.final_drive)}</h3>
    ${libraryPicker('gearboxes', 'A gearbox of the vanilla cars', ptEdit.gearbox_source)}
    ${ptEdit.gearbox_source && ptEdit.gears ? `<p class="quiet">Ratios of ${esc(ptEdit.gearbox_source.title)} (${esc(ptEdit.gearbox_source.model)}, ${esc(ptEdit.gearbox_source.type)}).</p>` : ''}
    <table class="cmp"><tr><th></th><th>Ratios</th><th>Final</th></tr>
      <tr><td>Base</td><td>${ratioText(b.ratios)}</td><td>${fmt(b.final_drive, 2)}</td></tr>
      <tr><td>SVJ</td><td>${ratioText(v.ratios)}</td><td>${fmt(v.final_drive, 2)}</td></tr>
      <tr><td><b>New</b></td><td><b>${ratioText(gears)}</b></td><td><b>${fmt(fd, 2)}</b></td></tr></table>
    ${gearEdit}
    <p class="quiet">Road speed at ${fmt(eng && eng.max_rpm, 0)} rpm (tyre radius ${fmt(r, 3)} m): ${speeds(gears, fd, eng && eng.max_rpm)}</p>
    ${b.automatic ? '<p class="quiet">The base gearbox is automatic: its shift points follow the new ratios.</p>' : ''}
    <h3>Differentials ${btns('diffs', v.diffs && Object.keys(v.diffs).length)}</h3>
    <table class="cmp"><tr><th></th><th>Base</th><th>SVJ</th><th>New</th><th>Preload Nm</th><th>Lock power</th><th>Lock coast</th></tr>
      ${['front', 'rear'].map((ax) => { const d = (ptEdit.diffs || {})[ax] || {}; return `<tr><td>${ax}</td><td>${esc(((b.diffs || {})[ax] || {}).type || '–')}</td><td>${esc(((v.diffs || {})[ax] || {}).type || '–')}</td>
        <td><select data-ptdiff="${ax}">${DIFF_TYPES.map(([t, l]) => `<option value="${t}" ${(d.type || '') === t ? 'selected' : ''}>${l}</option>`).join('')}</select></td>
        ${['preload', 'lock_power', 'lock_coast'].map((k) => `<td><input type="number" step="any" data-ptdiffv="${ax}:${k}" value="${d[k] != null ? d[k] : ''}" style="width:4.5em" ${d.type && d.type.startsWith('lsd_clutch') ? '' : 'disabled'}></td>`).join('')}</tr>`; }).join('')}</table>
    <div class="kv"><span title="All-wheel drive: the front axle's share of the torque">AWD front share</span><span><input type="number" step="0.05" min="0" max="1" id="ptsplit" value="${ptEdit.diffs && ptEdit.diffs.split != null ? ptEdit.diffs.split : ''}" placeholder="${v.split != null ? v.split : '0.4'}" style="width:5em"></span></div>
    <h3>Steering ${btns('steering', v.turns)}</h3>
    <div class="kv"><span>Turns lock to lock</span><span>base ${fmt(b.turns, 2)} · SVJ ${fmt(v.turns, 2)} · <b>new ${fmt(turns, 2)}</b></span></div>
    ${steerEdit}
    <p class="quiet">Written into the new vehicle in <button class="mini" data-goto="assembly">Assembly</button>; "base" keeps the base vehicle's value.
      Kept with the project.</p>`;
}

function bindPowertrain(el) {
  const set = (k, val) => { ptEdit = { ...ptEdit, [k]: val, touched: true }; ptEditing = null; redraw(); };
  const svjEdit = () => (svjDoc ? JSON.parse(ptpy.from_svj_json(JSON.stringify(svjDoc.svj))) : {});
  el.querySelectorAll('[data-ptkeep]').forEach((x) => { x.onclick = () => {
    const k = x.dataset.ptkeep;
    if (k === 'gears') { ptEdit = { ...ptEdit, gears: null, final_drive: null, gearbox_source: null, touched: true }; ptEditing = null; redraw(); } else set(k === 'steering' ? 'steering_turns' : k, null);
  }; });
  el.querySelectorAll('[data-ptsvj]').forEach((x) => { x.onclick = () => {
    const k = x.dataset.ptsvj, e = svjEdit();
    if (k === 'gears') { ptEdit = { ...ptEdit, gears: e.gears || ptEdit.gears, final_drive: e.final_drive || ptEdit.final_drive, touched: true }; ptEditing = null; redraw(); }
    else if (k === 'diffs') set('diffs', e.diffs || null);
    else if (k === 'steering') set('steering_turns', e.steering_turns); else set(k, e[k]);
  }; });
  el.querySelectorAll('[data-ptedit]').forEach((x) => { x.onclick = () => { ptEditing = ptEditing === x.dataset.ptedit ? null : x.dataset.ptedit; drawInspector(); }; });
  el.querySelectorAll('[data-ptslot]').forEach((x) => { x.onchange = () => { vehEdit.parts[x.dataset.ptslot] = x.value; ptEditing = 'parts'; configureVehicle(); }; });
  if ($('ptengok')) $('ptengok').onclick = () => {
    const pts = $('pttorque').value.split('\n').map((l) => l.trim().split(/[\s,;]+/).map(Number)).filter((p) => p.length >= 2 && p.every(Number.isFinite)).map((p) => [p[0], p[1]]);
    if (pts.length < 2) { showIssues([{ level: 'ERROR', rule: 'powertrain', message: 'The torque curve needs at least two lines of rpm and Nm.' }]); return; }
    pts.sort((a, c) => a[0] - c[0]);
    set('engine', { torque: pts, idle_rpm: +$('ptidle').value || pts[0][0], max_rpm: +$('ptmax').value || pts[pts.length - 1][0] });
  };
  if ($('ptgearok')) $('ptgearok').onclick = () => {
    const g = $('ptgears').value.split(/[\s,;]+/).map(Number).filter((x) => Number.isFinite(x) && x > 0);
    ptEdit = { ...ptEdit, gears: g.length ? g : null, final_drive: +$('ptfd').value > 0 ? +$('ptfd').value : null, touched: true };
    ptEditing = null; redraw();
  };
  if ($('ptsteerok')) $('ptsteerok').onclick = () => set('steering_turns', +$('ptturns').value > 0 ? +$('ptturns').value : null);
  if ($('liblearn')) $('liblearn').onclick = learnLibrary;
  el.querySelectorAll('[data-pteng]').forEach((x) => { x.onchange = () => {
    const base = ptEdit.engine || (() => { try { const s = JSON.parse(ptpy.summary_json(veh ? veh.model : null, veh ? JSON.stringify(veh) : null, null)).base; return s && s.torque ? { torque: s.torque, idle_rpm: s.idle_rpm, max_rpm: s.max_rpm } : null; } catch { return null; } })();
    if (!base) { showIssues([{ level: 'INFO', rule: 'powertrain', message: 'Choose an engine first (base, SVJ or an archetype).' }]); return; }
    const val = x.value === '' ? null : +x.value;
    set('engine', { ...base, [x.dataset.pteng]: val });
  }; });
  el.querySelectorAll('[data-ptdiff]').forEach((x) => { x.onchange = () => {
    const d = { ...(ptEdit.diffs || {}) };
    if (x.value) d[x.dataset.ptdiff] = { ...(d[x.dataset.ptdiff] || {}), type: x.value }; else delete d[x.dataset.ptdiff];
    ptEdit = { ...ptEdit, diffs: Object.keys(d).length ? d : null, touched: true }; redraw();
  }; });
  el.querySelectorAll('[data-ptdiffv]').forEach((x) => { x.onchange = () => {
    const [ax, k] = x.dataset.ptdiffv.split(':'), d = { ...(ptEdit.diffs || {}) };
    d[ax] = { ...(d[ax] || {}), [k]: x.value === '' ? null : +x.value };
    ptEdit = { ...ptEdit, diffs: d, touched: true }; redraw();
  }; });
  if ($('ptsplit')) $('ptsplit').onchange = () => {
    const d = { ...(ptEdit.diffs || {}) };
    if ($('ptsplit').value === '') delete d.split; else d.split = Math.min(1, Math.max(0, +$('ptsplit').value));
    ptEdit = { ...ptEdit, diffs: Object.keys(d).length ? d : null, touched: true }; redraw();
  };
  if ($('ptsuggest')) $('ptsuggest').onclick = () => {
    const s = suggestFromSvj();
    if (!s || !s.engine) return;
    const p = s.engine.pick, src = { model: p.model, part: p.part, title: s.engine.name, device: p.device || null, type: null };
    const svjEng = ptEdit.engine && !ptEdit.engine.source ? ptEdit.engine : null;   // the SVJ's curve stays, the archetype gives the rest
    ptEdit = { ...ptEdit, engine: svjEng ? { ...svjEng, source: src } : { torque: p.torque, idle_rpm: p.idle_rpm, max_rpm: p.max_rpm, source: src },
      gearbox_source: s.gearbox ? { model: s.gearbox.pick.model, part: s.gearbox.pick.part, title: s.gearbox.name, device: s.gearbox.pick.device, type: s.gearbox.pick.type } : ptEdit.gearbox_source,
      touched: true };
    redraw();
  };
  el.querySelectorAll('[data-libpick]').forEach((x) => { x.onchange = () => {
    if (x.value === '') return;
    const a = library.archetypes[x.dataset.libpick][+x.value], p = a.pick;
    const source = { model: p.model, part: p.part, title: a.name, device: p.device || null, type: p.type || null };
    if (x.dataset.libpick === 'engines') set('engine', { torque: p.torque, idle_rpm: p.idle_rpm || (p.torque[0] || [800])[0], max_rpm: p.max_rpm || p.torque[p.torque.length - 1][0], source });
    else { ptEdit = { ...ptEdit, gears: p.ratios, gearbox_source: source, touched: true }; ptEditing = null; redraw(); }
  }; });
}

// ---------- Components: the masses the car carries, its mass and CG (beamforge/components.py) ----------
// compItems: [{ id, kind, mass (kg), position [x, y, z] (SVJ frame, m) }]. A car from scratch gets each one's mass on the
// frame nodes nearest its place; a base car's mass and CG are its own (and the SVJ's values, taken in SVJ).
let compItems = [];
const COMP_KINDS = ['payload', 'driver', 'fluid', 'electrical', 'powertrain', 'structural', 'ballast', 'other'];
const PRESETS = { driver: { id: 'driver', kind: 'driver', mass: 75, position: [-1.25, -0.35, -0.45] },
  fuel: { id: 'fuel', kind: 'fluid', mass: 37, position: [-2.2, 0, -0.35] },
  battery: { id: 'battery', kind: 'electrical', mass: 15, position: [0.35, 0.5, -0.6] },
  ballast: { id: 'ballast', kind: 'ballast', mass: 20, position: [-1.2, 0, -0.15] } };

function compEstimate() {
  if (!skel) return null;
  const front = Object.values(sketchWheels()).filter((w) => w.center[0] > -0.5);
  const fc = front.length ? [0, 1, 2].map((i) => front.reduce((s, w) => s + w.center[i], 0) / front.length) : [0, 0, -0.3];
  const eng = ptEdit.engine && ptEdit.engine.mass ? { mass: ptEdit.engine.mass, position: [fc[0] - (scratchForm.layout === 'FWD' ? 0.05 : 0.35), 0, fc[2] - 0.05] } : null;
  try {
    return JSON.parse(copy_.estimate_json(JSON.stringify({ parts: skel.parts, opts: skel.opts, engine: eng, wheels: sketchWheels(), components: compItems })));
  } catch (err) { return { error: pyError(err) }; }
}

function drawComponents() {
  compG.clear();
  if (ws !== 'components') return;
  const mat = new THREE.MeshBasicMaterial({ color: 0xe0782a, transparent: true, opacity: 0.55, depthTest: false });
  for (const c of compItems) {
    if (!c.position || !(c.mass > 0)) continue;
    const s = new THREE.Mesh(new THREE.SphereGeometry(0.03 * Math.cbrt(c.mass), 14, 10), mat);
    s.position.copy(v3(saeToBng(c.position)));
    s.renderOrder = 9;
    compG.add(s);
  }
  const e = compEstimate();
  if (e && e.cg) {
    const s = new THREE.Mesh(new THREE.SphereGeometry(0.05, 16, 12), new THREE.MeshBasicMaterial({ color: 0xcf222e, depthTest: false }));
    s.position.copy(v3(saeToBng(e.cg)));
    s.renderOrder = 10;
    compG.add(s);
  }
}

function componentsPanel() {
  const target = svjDoc ? JSON.parse(copy_.svj_json(JSON.stringify(svjDoc.svj))) : null;
  const e = compEstimate();
  const base = veh && !skel ? JSON.parse(copy_.base_json(veh.model, JSON.stringify(veh))) : null;
  const pos = (c, i) => `<input type="number" step="1" data-cpos="${i}:${['x', 'y', 'z'].indexOf(c)}" value="${toMm(compItems[i].position[['x', 'y', 'z'].indexOf(c)])}" style="width:4.6em">`;
  const sum = e && !e.error ? `<div class="kv"><span>Mass</span><span><b>${fmt(e.mass, 0)} kg</b>${target && target.mass ? ` <span class="q">SVJ ${fmt(target.mass, 0)} kg</span>` : ''}</span>
      <span>CG behind the front axle</span><span>${e.cg ? fmt(-e.cg[0], 3) + ' m' : '–'}${target && target.cg ? ` <span class="q">SVJ ${fmt(-target.cg[0], 3)}</span>` : ''}</span>
      <span>CG height</span><span>${e.cg ? fmt(-e.cg[2], 3) + ' m' : '–'}${target && target.cg ? ` <span class="q">SVJ ${fmt(-target.cg[2], 3)}</span>` : ''}</span>
      <span>On the front axle</span><span>${e.front_share != null ? fmt(e.front_share * 100, 1) + ' %' : '–'}</span></div>
    <details><summary>What weighs</summary><table class="cmp"><tr><th></th><th>kg</th><th>X / Z m</th></tr>
      ${e.rows.map((r) => `<tr><td>${esc(r.what)}</td><td>${fmt(r.mass, 1)}</td><td>${fmt(r.position[0], 2)} / ${fmt(r.position[2], 2)}</td></tr>`).join('')}</table></details>
    ${!(ptEdit.engine && ptEdit.engine.mass) ? '<p class="quiet">The engine counts once its mass is set (Powertrain, engine mass).</p>' : ''}` : e && e.error ? `<p class="bad">${esc(e.error)}</p>` : '';
  return `<h2>Components</h2>
    ${skel ? sum : base ? `<div class="kv"><span>Base vehicle</span><span>${fmt(base.mass, 0)} kg, CG ${fmt(base.cg_behind_front_axle, 3)} m behind the front axle, ${fmt(base.cg_height, 3)} m high</span></div>
      <p class="quiet">A base car keeps its own masses; the SVJ's mass and CG are taken in SVJ. The components below are for a car made from scratch.</p>` : '<p class="quiet">Make a sketch (a car from scratch) to weigh it.</p>'}
    <table class="cmp"><tr><th>Component</th><th>Kind</th><th>kg</th><th>X mm</th><th>Y</th><th>Z</th><th></th></tr>
      ${compItems.map((c, i) => `<tr><td><input data-cid="${i}" value="${esc(c.id)}" size="9"></td>
        <td><select data-ckind="${i}">${COMP_KINDS.map((k) => `<option ${c.kind === k ? 'selected' : ''}>${k}</option>`).join('')}</select></td>
        <td><input type="number" step="0.5" data-cmass="${i}" value="${c.mass}" style="width:4.6em"></td><td>${pos('x', i)}</td><td>${pos('y', i)}</td><td>${pos('z', i)}</td>
        <td><button class="mini" data-cdel="${i}">×</button></td></tr>`).join('')}</table>
    <div class="inl">${Object.keys(PRESETS).map((k) => `<button class="mini" data-cadd="${k}">+ ${k}</button>`).join('')}
      ${target && target.components.length ? '<button class="mini" id="csvj">from the SVJ</button>' : ''}
      ${e && !e.error && target && target.mass ? '<button class="mini" id="cballast" title="One ballast mass, and where, that brings the car to the SVJ\'s mass (and CG, where it gives one)">ballast to the SVJ</button>' : ''}</div>
    <p class="quiet">Positions in the SVJ frame, mm: X forward of the front axle (behind it is negative), Y right, Z down from the ground
      (above it is negative). In the view: the components orange, the car's CG red.</p>`;
}

function bindComponents(el) {
  const ch = () => redraw();
  el.querySelectorAll('[data-cid]').forEach((x) => { x.onchange = () => { compItems[+x.dataset.cid].id = x.value.trim() || 'component'; ch(); }; });
  el.querySelectorAll('[data-ckind]').forEach((x) => { x.onchange = () => { compItems[+x.dataset.ckind].kind = x.value; ch(); }; });
  el.querySelectorAll('[data-cmass]').forEach((x) => { x.onchange = () => { compItems[+x.dataset.cmass].mass = Math.max(0, +x.value || 0); ch(); }; });
  el.querySelectorAll('[data-cpos]').forEach((x) => { x.onchange = () => { const [i, k] = x.dataset.cpos.split(':').map(Number); compItems[i].position[k] = (+x.value || 0) / 1000; ch(); }; });
  el.querySelectorAll('[data-cdel]').forEach((x) => { x.onclick = () => { compItems.splice(+x.dataset.cdel, 1); ch(); }; });
  el.querySelectorAll('[data-cadd]').forEach((x) => { x.onclick = () => { compItems.push(JSON.parse(JSON.stringify(PRESETS[x.dataset.cadd]))); ch(); }; });
  if ($('csvj')) $('csvj').onclick = () => { compItems = compItems.concat(JSON.parse(copy_.svj_json(JSON.stringify(svjDoc.svj))).components); ch(); };
  if ($('cballast')) $('cballast').onclick = () => {
    compItems = compItems.filter((c) => c.id !== 'ballast_svj');
    const e = compEstimate(), t = JSON.parse(copy_.svj_json(JSON.stringify(svjDoc.svj)));
    const b = JSON.parse(copy_.ballast_json(e.mass, JSON.stringify(e.cg), t.mass, t.cg ? JSON.stringify(t.cg) : null));
    if (!b) { showIssues([{ level: 'INFO', rule: 'components', message: `The car is already ${fmt(e.mass, 0)} kg, at or above the SVJ's ${fmt(t.mass, 0)} kg: ballast cannot take mass away.` }]); return; }
    compItems.push({ id: 'ballast_svj', kind: 'ballast', mass: b.mass, position: b.position });
    ch();
  };
}

// ---------- Aero: drag and downforce (beamforge/aero.py) ----------
// aeroEdit: { cd, area, cl_front, cl_rear } the new vehicle's numbers, or null (the base's as they are). From the SVJ by
// default. The drag goes into a base car's aero triangles (scaled to the drag area) and a scratch car's drag plate; the
// lift is shown, not written (BeamNG's triangle aero law is not public, so a wing cannot be sized to a downforce yet).
let aeroEdit = null, aeroTouched = false;

function aeroFromSvj() {
  if (!svjDoc) return null;
  const wb = (veh && veh.measure && veh.measure.wheelbase) || svjDoc.summary.wheelbase || 2.5;
  return JSON.parse(aepy.edit_from_svj_json(JSON.stringify(svjDoc.svj), wb));
}
function aepyRead() { return JSON.parse(aepy.read_json(JSON.stringify(svjDoc.svj))); }

function aeroPanel() {
  const wb = (veh && veh.measure && veh.measure.wheelbase) || (svjDoc && svjDoc.summary.wheelbase) || 2.5;
  const base = veh ? JSON.parse(aepy.base_json(veh.model, JSON.stringify(veh))) : null;
  const s = svjDoc ? aepyRead() : null;
  const e = aeroEdit || {};
  const asAero = (x) => x ? { area: x.area, Cd: x.cd, Cl: (x.cl_front != null || x.cl_rear != null) ? (x.cl_front || 0) + (x.cl_rear || 0) : null,
    Cl_front: x.cl_front, Cl_rear: x.cl_rear, rho: x.rho || 1.225 } : null;
  const rows = aeroEdit ? JSON.parse(aepy.forces_json(JSON.stringify(asAero(aeroEdit)), wb)) : [];
  const sweep = aeroEdit ? (() => { const out = { drag: [], f: [], r: [] }; for (let v = 0; v <= 250; v += 10) {
    const q = 0.5 * 1.225 * (v / 3.6) ** 2; out.drag.push([v, q * (e.cd || 0) * (e.area || 0)]);
    out.f.push([v, -q * (e.cl_front || 0) * (e.area || 0) + 0]); out.r.push([v, -q * (e.cl_rear || 0) * (e.area || 0) + 0]); } return out; })() : null;
  const field = (k, label, step) => `<span>${label}</span><span><input type="number" step="${step}" data-aero="${k}" value="${e[k] != null ? e[k] : ''}" style="width:6em"></span>`;
  return `<h2>Aero</h2>
    <table class="cmp"><tr><th></th><th>Base</th><th>SVJ</th><th>New</th></tr>
      <tr><td>Drag area CdA m²</td><td>${base ? fmt(base.cda, 3) : '–'}</td><td>${s && s.Cd && s.area ? fmt(s.Cd * s.area, 3) : '–'}</td><td><b>${e.cd && e.area ? fmt(e.cd * e.area, 3) : 'the base\'s'}</b></td></tr>
      <tr><td>Cd · area</td><td>–</td><td>${s ? `${fmt(s.Cd, 3)} · ${fmt(s.area, 2)}` : '–'}</td><td>${e.cd ? `${fmt(e.cd, 3)} · ${fmt(e.area, 2)}` : '–'}</td></tr>
      <tr><td>Cl front · rear</td><td>–</td><td>${s ? `${fmt(s.Cl_front, 3)} · ${fmt(s.Cl_rear, 3)}${s.Cl != null && s.Cl_front == null ? ` (Cl ${fmt(s.Cl, 3)})` : ''}` : '–'}</td><td>${aeroEdit ? `${fmt(e.cl_front, 3)} · ${fmt(e.cl_rear, 3)}` : '–'}</td></tr></table>
    <div class="inl"><button class="mini${aeroEdit ? '' : ' on'}" id="aerokeep" ${veh ? '' : 'disabled'}>base</button><button class="mini" id="aerosvj" ${s && s.Cd ? '' : 'disabled'}>SVJ</button></div>
    <div class="kv">${field('cd', 'Cd', 0.01)}${field('area', 'Frontal area m²', 0.01)}${field('cl_front', 'Cl front (− downforce)', 0.01)}${field('cl_rear', 'Cl rear (− downforce)', 0.01)}</div>
    ${s && s.components.length ? `<details><summary>The SVJ's devices (${s.components.length})</summary><table class="cmp"><tr><th>Device</th><th>Type</th><th>Cd</th><th>Cl</th></tr>
      ${s.components.map((c) => `<tr><td>${esc(c.id || '')}</td><td>${esc(c.type || '')}</td><td>${fmt(c.Cd, 3)}</td><td>${fmt(c.Cl, 3)}</td></tr>`).join('')}</table></details>` : ''}
    ${rows.length ? `<table class="cmp"><tr><th>km/h</th><th>Drag N</th><th>Power kW</th><th>Down front N</th><th>Down rear N</th></tr>
      ${rows.map((r) => `<tr><td>${r.kmh}</td><td>${fmt(r.drag, 0)}</td><td>${fmt(r.power, 1)}</td><td>${fmt(r.down_front, 0)}</td><td>${fmt(r.down_rear, 0)}</td></tr>`).join('')}</table>
      ${lineChart('Drag', [{ label: 'drag', color: '#2f6fdf', pts: sweep.drag }], 'km/h', 'N')}
      ${e.cl_front != null || e.cl_rear != null ? lineChart('Downforce', [{ label: 'front', color: '#2f6fdf', pts: sweep.f }, { label: 'rear', color: '#e0782a', pts: sweep.r }], 'km/h', 'N') : ''}` : ''}
    <p class="quiet">The drag goes into the new vehicle: a base car's aero triangles scaled to this drag area (BeamForge's estimate of it),
      a car made from scratch gets a drag plate. The downforce is shown, not written: BeamNG's aero law for its triangles is not public,
      so a wing cannot be sized to a downforce yet. Air ${fmt(1.225, 3)} kg/m³, wheelbase ${fmt(wb, 2)} m.</p>`;
}

function bindAero(el) {
  if ($('aerokeep')) $('aerokeep').onclick = () => { aeroEdit = null; aeroTouched = true; redraw(); };
  if ($('aerosvj')) $('aerosvj').onclick = () => { aeroEdit = aeroFromSvj(); aeroTouched = true; redraw(); };
  el.querySelectorAll('[data-aero]').forEach((x) => { x.onchange = () => {
    aeroEdit = { ...(aeroEdit || {}), [x.dataset.aero]: x.value === '' ? null : +x.value };
    if (!aeroEdit.cd && !aeroEdit.area && aeroEdit.cl_front == null && aeroEdit.cl_rear == null) aeroEdit = null;
    aeroTouched = true; redraw();
  }; });
}

// ---------- Checks: the vehicle's and the sketch's stability and structure ----------
let checkRes = null;          // { veh (the configured vehicle it was run on), data (structure.checks_json) }

function checksPanel() {
  const bands = (b) => Object.entries(b).map(([k, n]) => `<span class="band ${k}">${n} ${k}</span>`).join(' ');
  let vehPart = '<p class="quiet">Open a base vehicle to check it.</p>';
  if (veh) {
    const d = checkRes && checkRes.data, stale = checkRes && checkRes.veh !== veh;
    vehPart = `<p><button id="chkrun" class="primary">${d ? 'Check again' : 'Check the vehicle'}</button>
      ${stale ? '<span class="q">the vehicle changed since</span>' : ''}</p>`;
    if (d) {
      vehPart += `<p class="quiet">Each node's stiffness and damping for its weight, against the nodes of ten vanilla cars at 2000 Hz:
          ok up to their 90th percentile, high to the 99th, extreme to the highest they run at, beyond past it (no vanilla
          car goes there). A vanilla car has about 10 % high and 1 % extreme.</p><p>${bands(d.bands)}</p>
        <details open><summary>Stiffest for their weight</summary><table class="cmp"><tr><th>Node</th><th>Part</th><th>k</th><th>c</th><th>kg</th></tr>
          ${d.worst.map(([n, k, c, kg, b, part]) => `<tr><td><button class="mini" data-chknode="${esc(n)}">${esc(n)}</button></td><td>${esc(part || '')}</td>
            <td><span class="band ${b}">${fmt(k, 2)}</span></td><td>${fmt(c, 2)}</td><td>${fmt(kg, 1)}</td></tr>`).join('')}</table></details>
        <details><summary>Held in fewer than three directions (${d.weak_count})</summary>
          <p class="quiet">A joint of a linkage is meant to move; a node meant to be structure that is here floats.</p>
          <table class="cmp"><tr><th>Node</th><th>Part</th><th>Held</th></tr>
          ${d.weak.map(([n, r, nb, part]) => `<tr><td><button class="mini" data-chknode="${esc(n)}">${esc(n)}</button></td><td>${esc(part || '')}</td><td>${r} of 3, ${nb} beams</td></tr>`).join('')}</table></details>`;
    }
  }
  let skPart = '<p class="quiet">Import a STEP in Sketch to check it.</p>';
  if (skel && skel.res) {
    const r = skel.res.report;
    skPart = `<div class="kv"><span>Stability</span><span>${bands(r.bands || {})}</span>
      <span>Free motions</span><span>${r.free_motions} <span class="q">${r.mechanism === null ? '' : `6 of the whole, ${r.mechanism} of the mechanism`}</span></span>
      <span>Parts not rigid</span><span>${r.not_rigid.length ? esc(r.not_rigid.map((p) => p.join('+')).join(', ')) : 'none'}</span></div>`;
  }
  return `<h2>Vehicle</h2>${vehPart}<h2>Sketch</h2>${skPart}`;
}

function bindChecks(el) {
  if ($('chkrun')) $('chkrun').onclick = () => {
    $('chkrun').textContent = 'Checking…';
    setTimeout(() => {
      const t0 = performance.now();
      try { checkRes = { veh, data: JSON.parse(strpy.checks_json(veh.model, JSON.stringify(veh))) }; }
      catch (err) { showIssues([{ level: 'ERROR', rule: 'checks', message: pyError(err) }]); }
      timing.checks = performance.now() - t0;
      drawInspector(); drawTiming();
    }, 20);
  };
  el.querySelectorAll('[data-chknode]').forEach((b) => { b.onclick = () => { pick = { kind: 'node', id: b.dataset.chknode }; drawVehicle(); }; });
}

// ---------- Assembly: everything together, the new vehicle ----------
function assemblyPanel() {
  const fitN = veh ? Object.keys(vehEdit.fit || {}).length : 0;
  const taken = Object.values(takeValues).filter(Boolean).length;
  return `<h2>Assembly</h2>
    <div class="kv"><span>Base vehicle</span><span>${veh ? `${esc(veh.model)} · ${esc(veh.config)}` : 'none'} <button class="mini" data-goto="base">Base</button></span>
      <span>Moved by hand</span><span>${veh ? moveCount() : 0}</span>
      <span>SVJ</span><span>${svjDoc ? esc(svjDoc.summary.vehicle || 'loaded') : 'none'} <button class="mini" data-goto="svj">SVJ</button></span>
      <span>Fitted to it</span><span>${fitN ? fitN + ' nodes' : 'no'}</span>
      <span>Values taken</span><span>${taken}</span>
      <span>Sketch</span><span>${skel ? esc(skel.name) : 'none'} <button class="mini" data-goto="sketch">Sketch</button></span></div>
    ${skel && skel.res ? `<p class="quiet">The sketch is not mounted on the base yet (next step). Its structure as jbeam parts:
      <button id="asmskjbeam" class="mini">Download jbeam</button></p>` : ''}
    ${scratchPanel()}
    ${veh ? exportPanel() : '<p class="quiet">Open a base vehicle in Base to make a new vehicle from it (or make one from scratch above).</p>'}`;
}

function bindAssembly(el) {
  if ($('asmskjbeam')) $('asmskjbeam').onclick = skeletonJbeam;
  bindScratch(el);
}

// ---------- a car from scratch (beamforge/scratch.py): the sketch, the Powertrain's engine and gearbox, rims and tyres
// of the vanilla cars. UNVERIFIED IN-GAME (the first version): the vehicle spawns where BeamNG puts new vehicles.
let scratchForm = { id: '', name: '', brand: '', layout: 'RWD', lock: 33, note: '' };

// ---------- Wheels: rims and tyres per axle (archetypes of the vanilla cars), the user's tyre values, the SVJ's Pacejka
// tyre as a benchmark (beamforge/tyres.py: grip against load, relative to the reference load; the fit matches BeamNG's
// load sensitivity to it). rim / tyre: indexes into the library's archetypes ('' = none / the rim's own tyre).
const WHEEL_EMPTY = () => ({ rim: '', tyre: '', overrides: {} });
let wheelsForm = { front: WHEEL_EMPTY(), rear: WHEEL_EMPTY(), same: true, axle: 'front' };
const TYRE_KEYS = [['radius', 'Radius m', 3], ['tireWidth', 'Width m', 3], ['frictionCoef', 'Grip (frictionCoef)', 3],
  ['slidingFrictionCoef', 'Sliding grip', 3], ['noLoadCoef', 'No-load coefficient', 3], ['fullLoadCoef', 'Full-load coefficient', 3],
  ['loadSensitivitySlope', 'Load sensitivity slope /N', 7], ['softnessCoef', 'Softness', 2], ['treadCoef', 'Tread', 2], ['pressurePSI', 'Pressure psi', 1]];
const axleOf = (ax) => (wheelsForm.same ? wheelsForm.front : wheelsForm[ax]);

function wheelChoice(ax) {
  const f = axleOf(ax), A = library ? library.archetypes : { rims: [], tyres: [] };
  const rim = f.rim === '' ? null : A.rims[+f.rim], tyre = f.tyre === '' ? null : A.tyres[+f.tyre];
  return { f, rim, tyre };
}

function wheelsPanel() {
  if (!library) return `<h2>Wheels</h2><p class="quiet">Learn the vanilla cars' rims and tyres first (Powertrain: Learn from the install).</p>`;
  const ax = wheelsForm.same ? 'front' : wheelsForm.axle;
  const { f, rim, tyre } = wheelChoice(ax);
  const A = library.archetypes;
  const fit = rim ? A.tyres.map((t, i) => [t, i]).filter(([t]) => t.pick.rim_in === rim.pick.diameter_in) : [];
  const base = tyre ? tyre.pick : {};
  const props = { ...base, ...f.overrides };
  let bench = null;
  try {
    bench = JSON.parse(typy.benchmark_json(svjDoc ? JSON.stringify(svjDoc.svj) : null, tyre ? JSON.stringify(props) : null, ax === 'front' ? 'FL' : 'RL'));
  } catch (err) { bench = { error: pyError(err) }; }
  const st = bench && bench.svj;
  const loads = st && bench.curves ? Object.keys(bench.curves.fy) : [];
  const colours = ['#8250df', '#2f6fdf', '#e0782a'];
  const curves = st && bench.curves ? `
      ${lineChart('Lateral force against slip angle (SVJ)', loads.map((l, i) => ({ label: `${+l / 1000} kN`, color: colours[i], pts: bench.curves.fy[l] })), '°', 'N')}
      ${lineChart('Longitudinal force against slip ratio (SVJ)', loads.map((l, i) => ({ label: `${+l / 1000} kN`, color: colours[i], pts: bench.curves.fx[l] })), '%', 'N')}
      ${bench.shape ? lineChart('Grip against load, relative to the reference load', [
        { label: 'SVJ lateral', color: '#8250df', pts: bench.shape.svj_y }, { label: 'SVJ longitudinal', color: '#8250df', dash: '3 3', pts: bench.shape.svj_x },
        bench.shape.bng && { label: 'BeamNG tyre', color: '#e0782a', pts: bench.shape.bng }].filter(Boolean), 'N', '', 2) : ''}
      ${bench.fit ? `<p><button id="whfit" class="mini" title="noLoadCoef, fullLoadCoef and loadSensitivitySlope that give the BeamNG tyre the SVJ tyre's loss of grip with load, its grip at the reference load kept">Fit the load sensitivity to the SVJ</button>
        <span class="q">rms ${fmt(bench.fit.rms, 4)}</span></p>` : ''}
      <p class="quiet">The SVJ's curves are its Magic Formula (${esc(st.pacejka ? st.pacejka.model || 'MF' : '–')}, pure slip, no camber). BeamNG's tyre grip comes out of its nodes sliding on the ground:
        its friction values are multipliers, not a friction coefficient, so what is compared is the shape (how grip falls with load).
        BeamNG takes each tread node's load (about ${fmt(bench.contact_nodes, 1)} in contact here).</p>` : '';
  return `<h2>Wheels</h2>
    <div class="inl"><label><input type="checkbox" id="whsame" ${wheelsForm.same ? 'checked' : ''}> the same front and rear</label>
      ${wheelsForm.same ? '' : ['front', 'rear'].map((a) => `<button class="mini${ax === a ? ' on' : ''}" data-whaxle="${a}">${a}</button>`).join('')}</div>
    <h3>${wheelsForm.same ? 'Both axles' : ax === 'front' ? 'Front axle' : 'Rear axle'}</h3>
    <div class="kv"><span>Rim</span><span><select id="whrim"><option value="">choose…</option>${A.rims.map((a, i) => `<option value="${i}" ${String(i) === f.rim ? 'selected' : ''}>${esc(a.name)} · ${a.count}</option>`).join('')}</select></span>
      <span>Tyre</span><span><select id="whtyre" ${rim ? '' : 'disabled'}><option value="">the rim's own</option>${fit.map(([t, i]) => `<option value="${i}" ${String(i) === f.tyre ? 'selected' : ''}>${esc(t.name)} · ${t.count}</option>`).join('')}</select></span></div>
    ${rim ? `<p class="quiet">Rim: ${esc(rim.pick.part)} (${esc(rim.pick.model)}). ${tyre ? `Tyre: ${esc(tyre.pick.part)}.` : ''}</p>` : ''}
    ${st ? `<div class="kv"><span>SVJ tyre</span><span>${esc(st.size || st.name)} <span class="q">R${st.rim_in || '?'}, radius ${fmt(st.radius, 3)} m, ${fmt(st.fz0, 0)} N reference</span></span></div>
      <p><button id="whnearest" class="mini" title="The archetype rim of the SVJ tyre's rim size, and its tyre nearest in width">Pick the nearest to the SVJ tyre</button></p>` : ''}
    ${tyre ? `<table class="cmp"><tr><th>Tyre</th><th>Archetype</th><th>New</th></tr>
      ${TYRE_KEYS.filter(([k]) => base[k] != null || f.overrides[k] != null).map(([k, label, d]) => `<tr><td>${label}</td><td>${fmt(base[k], d)}</td>
        <td><input type="number" step="any" data-whov="${k}" value="${f.overrides[k] != null ? f.overrides[k] : ''}" placeholder="${base[k] != null ? base[k] : ''}" style="width:7em"></td></tr>`).join('')}</table>
      <p class="quiet">Empty keeps the archetype's value. A tyre with values of its own is written as a part of the car made from scratch.</p>` : ''}
    ${bench && bench.error ? `<p class="bad">${esc(bench.error)}</p>` : ''}
    ${curves || (svjDoc ? '<p class="quiet">The SVJ has no Pacejka tyre to compare with.</p>' : '<p class="quiet">Import an SVJ with a Pacejka tyre to compare with.</p>')}`;
}

function bindWheels(el) {
  const ax = wheelsForm.same ? 'front' : wheelsForm.axle;
  const f = axleOf(ax);
  if ($('whsame')) $('whsame').onchange = () => { wheelsForm.same = $('whsame').checked; if (!wheelsForm.same) wheelsForm.rear = JSON.parse(JSON.stringify(wheelsForm.front)); redraw(); };
  el.querySelectorAll('[data-whaxle]').forEach((b) => { b.onclick = () => { wheelsForm.axle = b.dataset.whaxle; redraw(); }; });
  if ($('whrim')) $('whrim').onchange = () => { f.rim = $('whrim').value; f.tyre = ''; f.overrides = {}; redraw(); };
  if ($('whtyre')) $('whtyre').onchange = () => { f.tyre = $('whtyre').value; f.overrides = {}; redraw(); };
  el.querySelectorAll('[data-whov]').forEach((x) => { x.onchange = () => {
    if (x.value === '') delete f.overrides[x.dataset.whov]; else f.overrides[x.dataset.whov] = +x.value;
    redraw();
  }; });
  if ($('whfit')) $('whfit').onclick = () => {
    const { tyre } = wheelChoice(ax);
    const b = JSON.parse(typy.benchmark_json(JSON.stringify(svjDoc.svj), JSON.stringify({ ...tyre.pick, ...f.overrides }), ax === 'front' ? 'FL' : 'RL'));
    Object.assign(f.overrides, b.fit.values);
    redraw();
  };
  if ($('whnearest')) $('whnearest').onclick = () => {
    const st = JSON.parse(typy.benchmark_json(JSON.stringify(svjDoc.svj), null, ax === 'front' ? 'FL' : 'RL')).svj;
    const A = library.archetypes;
    const rims = A.rims.map((a, i) => [a, i]).filter(([a]) => a.pick.diameter_in === st.rim_in);
    if (!rims.length) { showIssues([{ level: 'INFO', rule: 'wheels', message: `No archetype rim of ${st.rim_in} in.` }]); return; }
    const w = (st.width || 0.2) * 1000;
    const [rim, ri] = rims.reduce((b, c) => (c[0].count > b[0].count ? c : b));          // the commonest rim of that size
    const tyres = A.tyres.map((t, i) => [t, i]).filter(([t]) => t.pick.rim_in === st.rim_in && t.pick.width_mm);
    // nearest in width; an everyday tyre (sport, standard) and a common group ahead of a special one of about the same width
    const score = ([t]) => Math.abs(t.pick.width_mm - w) + (['sport', 'standard'].includes(t.pick.use) ? 0 : 15) - 2 * Math.log(t.count);
    const best = tyres.length ? tyres.reduce((b, c) => (score(c) < score(b) ? c : b)) : null;
    f.rim = String(ri); f.tyre = best ? String(best[1]) : ''; f.overrides = {};
    redraw();
  };
}

// the sketch's wheels: each upright part's point is its wheel centre (SVJ frame, m); corner from the part's name
function sketchWheels() {
  const out = {};
  if (!skel) return out;
  for (const p of skel.parts) {
    const m = p.name.match(/^(upright|hub|knuckle)_(fl|fr|rl|rr)$/i);
    if (m && (p.points || []).length) out[m[2].toUpperCase()] = { center: p.points[0], hub: p.name };
  }
  return out;
}

// the rear (or front) twin of a part named for an axle: steelrim_01a_13x5_F <-> _R, tire_F_176_68_13 <-> tire_R_
const axlePart = (name, AX) => name.replace(/_[FR](?=_|$)/, `_${AX}`).replace(/^tire_[FR]_/, `tire_${AX}_`);

function scratchPanel() {
  if (!skel) return '';
  const wheels = sketchWheels(), wn = Object.keys(wheels).sort();
  const eng = ptEdit.engine && ptEdit.engine.source, gb = ptEdit.gearbox_source;
  const rim = wheelChoice('front').rim && wheelChoice('rear').rim;
  const rims = (library && library.archetypes.rims) || [];
  const ok = (c, t) => `<span class="${c ? 'good' : 'bad'}">${c ? '✓' : '✗'}</span> ${t}`;
  const frame = skel.res && skel.res.parts.some((p) => p.kind === 'frame' && p.nodes.length >= 4);
  const ready = frame && wn.length >= 3 && eng && gb && rim && library && rims.length;
  return `<h2>Make a car from scratch</h2>
    <p class="quiet">The sketch is the car: its frame, arms and uprights. The engine and gearbox, rims and tyres come from
      the vanilla cars' parts in your install. Unverified in the game: test it and tell what it does.</p>
    <div class="kv"><span>Frame</span><span>${ok(frame, 'a frame part with four nodes or more')}</span>
      <span>Wheels</span><span>${ok(wn.length >= 3, wn.length ? wn.join(', ') + ' <span class="q">(a point in each upright_fl… part: its centre)</span>' : 'put a point in each upright_fl, upright_fr… part: the wheel centre')}</span>
      <span>Engine</span><span>${ok(eng, eng ? esc(eng.title) : 'choose one in Powertrain, from the vanilla cars')} <button class="mini" data-goto="powertrain">Powertrain</button></span>
      <span>Gearbox</span><span>${ok(gb, gb ? esc(gb.title) : 'choose one in Powertrain')}</span>
      <span>Library</span><span>${ok(library, library ? `${library.vehicles} vehicles` : 'learn from the install in Powertrain')}</span></div>
    <div class="kv"><span>Wheels</span><span>${ok(rim, rim ? ['front', 'rear'].map((a) => { const c = wheelChoice(a); return `${a} ${esc(c.rim.name)}${c.tyre ? ', ' + esc(c.tyre.name) : ''}`; }).join(' · ') : 'choose rims and tyres')} <button class="mini" data-goto="wheels">Wheels</button></span>
      <span>Body</span><span>${svjDoc && svjDoc.meshes.some((m) => m.file) ? `<span class="good">✓</span> the SVJ's chassis mesh, on the frame` : '<span class="q">none (optional: an SVJ with its meshes gives the body)</span>'}</span></div>
    <div class="kv"><span>Layout</span><span><select id="sclayout">${['FWD', 'RWD', 'AWD'].map((l) => `<option ${scratchForm.layout === l ? 'selected' : ''}>${l}</option>`).join('')}</select></span>
      <span>Lock (deg)</span><span><input type="number" id="sclock" value="${scratchForm.lock}" min="15" max="60" step="1"></span>
      <span>Vehicle id</span><span><input id="scid" value="${esc(scratchForm.id || (skel.name || 'scratch').toLowerCase().replace(/[^a-z0-9_]+/g, '_'))}"></span>
      <span>Name</span><span><input id="scname" value="${esc(scratchForm.name || skel.name || 'Scratch car')}"></span>
      <span>Brand</span><span><input id="scbrand" value="${esc(scratchForm.brand || 'BeamForge')}"></span></div>
    <p><button id="scbuild" class="primary" ${ready ? '' : 'disabled'}>Make the car (mod zip)…</button></p>
    ${scratchForm.note ? `<p class="quiet">${scratchForm.note}</p>` : ''}`;
}

function bindScratch(el) {
  const keep = () => {
    for (const [id, k] of [['scid', 'id'], ['scname', 'name'], ['scbrand', 'brand']]) if ($(id)) scratchForm[k] = $(id).value.trim();
    if ($('sclock')) scratchForm.lock = +$('sclock').value || 33;
    if ($('sclayout')) scratchForm.layout = $('sclayout').value;
  };
  for (const id of ['scid', 'scname', 'scbrand', 'sclock', 'sclayout']) if ($(id)) $(id).onchange = keep;
  if ($('scbuild')) $('scbuild').onclick = () => { keep(); buildScratch(); };
}

async function buildScratch() {
  const say = (t) => { scratchForm.note = t; drawInspector(); };
  try {
    const A = library.archetypes, eng = ptEdit.engine, gb = ptEdit.gearbox_source;
    const W = { front: wheelChoice('front'), rear: wheelChoice('rear') };
    const src = (p, AX, over) => ({ model: p.model, part: axlePart(p.part, AX), ...(over && Object.keys(over).length ? { overrides: over } : {}) });
    // the donor parts' files, read again from the install (the library keeps only their numbers)
    const pick = (kind, s) => (A[kind] || []).find((a) => a.pick.model === s.model && a.pick.part === s.part);
    const files = new Set([W.front.rim.pick.file, W.rear.rim.pick.file, W.front.tyre && W.front.tyre.pick.file, W.rear.tyre && W.rear.tyre.pick.file,
      (pick('engines', eng.source) || {}).pick?.file, (pick('gearboxes', gb) || {}).pick?.file].filter(Boolean));
    if (!W.front.rim.pick.file) throw new Error('the library is from an older BeamForge: learn from the install again (Powertrain)');
    say('Reading the donor parts…');
    const { texts, ranks } = await readResolved((p) => files.has(p));
    vehBusy = '';
    vehpy.add_files(JSON.stringify(texts), JSON.stringify(ranks));
    const car = { id: scratchForm.id || 'scratch_car', name: scratchForm.name || 'Scratch car', brand: scratchForm.brand || 'BeamForge',
      sketch: { parts: skel.parts, opts: skel.opts }, wheels: sketchWheels(), layout: scratchForm.layout,
      engine: { source: eng.source, torque: eng.torque, idle_rpm: eng.idle_rpm, max_rpm: eng.max_rpm, inertia: eng.inertia, friction: eng.friction,
        engine_brake: eng.engine_brake, mass: eng.mass },
      diffs: ptEdit.diffs || {},
      aero: aeroEdit && aeroEdit.cd && aeroEdit.area ? { cda: +(aeroEdit.cd * aeroEdit.area).toFixed(4) } : {},
      components: compItems,
      gearbox: { source: gb, ratios: ptEdit.gears }, final_drive: ptEdit.final_drive || 4.0,
      rim: { front: src(W.front.rim.pick, 'F'), rear: src(W.rear.rim.pick, 'R') },
      tyre: Object.fromEntries(['front', 'rear'].filter((a) => W[a].tyre).map((a) => [a, src(W[a].tyre.pick, a === 'front' ? 'F' : 'R', W[a].f.overrides)])),
      steering: { lock_deg: scratchForm.lock, turns: ptEdit.steering_turns || 3 } };
    if (svjDoc && svjDoc.meshes.some((m) => m.file))        // the SVJ's body mesh, on the sketch's frame
      car.body = { svj: svjDoc.svj, files: Object.fromEntries(svjDoc.meshes.filter((m) => m.file).map((m) => [m.id, m.file])) };
    say('Making the car…');
    await new Promise((r) => setTimeout(r, 20));
    const out = JSON.parse(scpy.build(JSON.stringify(car)));
    const zw = new ZipWriter(new BlobWriter('application/zip'));
    for (const [p, text] of Object.entries(out.files)) await zw.add(p, new TextReader(text));
    for (const [p, b64] of Object.entries(out.binary || {})) {      // the SVJ mesh textures
      const bin = atob(b64), u8 = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
      await zw.add(p, new BlobReader(new Blob([u8])));
    }
    const blob = await zw.close();
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `${car.id}.zip`;
    a.click();
    const c = out.counts;
    say(`${esc(car.id)}.zip: ${c.nodes} nodes, ${c.beams} beams, ${c.wheels} wheels, ${fmt(c.mass, 0)} kg of nodes; stability ${Object.entries(c.bands).filter(([, n]) => n).map(([b, n]) => `${n} ${b}`).join(', ')}.
      Put it in your mods folder; the car is "${esc(car.name)}". ${out.notes.map(esc).join(' · ')}`);
  } catch (err) { vehBusy = ''; say(`<span class="bad">${esc(pyError(err))}</span>`); }
}

// ---------- project: the work saved, to continue later or pass on ----------
// A project holds what the user did, not game files: the base vehicle by name (whoever opens it needs that vehicle
// in their BeamNG folders), its configuration, slot choices, tuning, moves, fit and ties; the SVJ document (its meshes
// are asked for again: they can be large and sit beside it); the STEP sketch and its options; the values taken, the
// export form, the workspace. It is saved as a .beamforge.json, and autosaved in this browser after every change.
const PROJECT_VERSION = 1;
const AUTOSAVE_KEY = 'beamforge.autosave';
let pendingBase = null;       // a project's base vehicle, waiting for the BeamNG folders to be added
let autosaveNote = '', autosaveAt = null, restoring = false, svjName = null;

function projectData() {
  return {
    beamforge_project: PROJECT_VERSION, saved: new Date().toISOString(), workspace: ws,
    base: veh ? { model: vehEdit.model, config: vehEdit.config, parts: vehEdit.parts, vars: vehEdit.vars, moves: vehEdit.moves, beams: vehEdit.beams,
      fit: vehEdit.fit, fitReport: vehEdit.fitReport, fitOverrides: vehEdit.fitOverrides,
      hidden: [...hiddenParts], locked: [...lockedParts] } : pendingBase,
    svj: svjDoc ? { name: svjName || 'project.svj.json', doc: svjDoc.svj } : null,
    sketch: skel ? { name: skel.name, parts: skel.parts, opts: skel.opts, active: skel.active } : null,
    take: takeValues, export: exportForm, ties: savedTies(), powertrain: ptEdit, scratch: { ...scratchForm, note: '' }, wheels: wheelsForm, aero: aeroEdit, aero_touched: aeroTouched, components: compItems,
  };
}

function autosave() {
  if (restoring || !(veh || svjDoc || skel || pendingBase)) return;
  try {
    localStorage.setItem(AUTOSAVE_KEY, JSON.stringify(projectData()));
    autosaveAt = new Date(); autosaveNote = '';
  } catch (err) { autosaveNote = 'too large to keep in the browser: use Save'; }
}

function saveProject() {
  const name = (exportForm.name || (veh && veh.model) || (skel && skel.name.replace(/\.(step|stp)$/i, '')) || 'beamforge').replace(/[^\w.-]+/g, '_');
  download(`${name}.beamforge.json`, JSON.stringify(projectData(), null, 1));
}

async function applyProject(p) {
  if (!p || p.beamforge_project !== PROJECT_VERSION) throw new Error('not a BeamForge project file');
  restoring = true;
  try {
    if (p.ties) for (const [part, t] of Object.entries(p.ties)) storeTies({ ...savedTies(), [part]: { ...(savedTies()[part] || {}), ...t } });
    takeValues = p.take || {};
    if (p.export) exportForm = { ...exportForm, ...p.export };
    if (WORKSPACES.some(([k]) => k === p.workspace)) { ws = p.workspace; try { localStorage.setItem(WS_KEY, ws); } catch { /* */ } }
    pendingBase = p.base && p.base.model ? p.base : null;
    await applyPendingBase();
    if (p.svj && p.svj.doc) {
      const dir = '/tmp/svj_project';
      py.runPython(`import shutil; shutil.rmtree('${dir}', ignore_errors=True)`);
      py.FS.mkdirTree(dir);
      svjName = p.svj.name;
      svjSrc = `${dir}/${p.svj.name.replace(/[\\/]/g, '_')}`;
      py.FS.writeFile(svjSrc, JSON.stringify(p.svj.doc));
      await finishImport(p.svj.name);
    }
    if (p.powertrain) ptEdit = { ...PT_EMPTY, ...p.powertrain };
    if (p.scratch) scratchForm = { ...scratchForm, ...p.scratch };
    if (p.wheels) wheelsForm = { ...wheelsForm, ...p.wheels };
    if ('aero' in p) { aeroEdit = p.aero; aeroTouched = !!p.aero_touched; }
    if (Array.isArray(p.components)) compItems = p.components;
    if (p.sketch) {
      let parts = p.sketch.parts;
      if (!parts && p.sketch.text) parts = JSON.parse(skpy.import_json(p.sketch.text, JSON.stringify(p.sketch.opts || {}))).parts;   // a project saved before sketches had their own parts
      newSketch(p.sketch.name || 'sketch', parts || []);
      skel.opts = { ...skel.opts, ...(p.sketch.opts || {}) };
      for (const k of ['rot', 'offset', 'unit']) delete skel.opts[k];
      if (p.sketch.active) skel.active = p.sketch.active;
      buildSkeleton();
    }
  } finally { restoring = false; }
  redraw();
  showIssues([{ level: 'PASS', rule: 'project', message: `Project opened (saved ${new Date(p.saved).toLocaleString()}).` },
    ...(pendingBase ? [{ level: 'INFO', rule: 'project', message: `Add your BeamNG folders (Base vehicle) to reopen ${pendingBase.model}: the project's edits are applied once it is found.` }] : []),
    ...(svjDoc && svjDoc.meshes.some((m) => !m.file) ? [{ level: 'INFO', rule: 'project', message: "The SVJ's meshes are not in the project: Find the meshes folder in SVJ." }] : [])]);
}

// the project's base vehicle, opened with its edits once the vehicle list has it
async function applyPendingBase() {
  const b = pendingBase;
  if (!b || !cat || !cat.find((x) => x.model === b.model)) return;
  pendingBase = null;
  await openVehicle(b.model, b.config);
  vehEdit = { ...freshEdit(b.model, b.config), parts: b.parts || {}, vars: b.vars || {}, moves: b.moves || { parts: {}, nodes: {} },
    fit: b.fit || {}, fitReport: b.fitReport || null, fitOverrides: b.fitOverrides || {}, beams: b.beams || [] };
  hiddenParts = new Set(b.hidden || []); lockedParts = new Set(b.locked || []);
  configureVehicle();
  if (svjDoc) studySuspension();
}

async function restoreProject() {
  let p = null;
  try { p = JSON.parse(localStorage.getItem(AUTOSAVE_KEY) || 'null'); } catch { p = null; }
  if (!p) return;
  try { await applyProject(p); } catch (err) { console.warn('autosave not restored', err); }
}

function newProject() {
  if (!confirm('Start a new project? The current work stays only in a saved project file.')) return;
  try { localStorage.removeItem(AUTOSAVE_KEY); } catch { /* */ }
  location.reload();
}

function projectPanel() {
  return `<h2>Project</h2>
    <div class="kv"><span>Base vehicle</span><span>${veh ? esc(veh.model) : pendingBase ? esc(pendingBase.model) + ' <span class="q">(waiting for the folders)</span>' : 'none'}</span>
      <span>Configuration</span><span>${veh ? esc(veh.config) : '–'}</span>
      <span>SVJ</span><span>${svjDoc ? esc(svjDoc.summary.vehicle || svjName || 'loaded') : 'none'}</span>
      <span>Sketch</span><span>${skel ? esc(skel.name) : 'none'}</span>
      <span>Autosaved</span><span>${autosaveNote ? `<span class="bad">${esc(autosaveNote)}</span>` : autosaveAt ? autosaveAt.toLocaleTimeString() : '–'}</span></div>
    <div class="inl"><button id="pjsave" class="primary">Save project…</button><button id="pjopen">Open…</button><button id="pjnew">New</button></div>
    <p class="quiet">A project keeps the work, not game files: the base vehicle by name (it must be in the BeamNG folders of whoever
      opens it), its edits, the SVJ and the sketch. The work is also kept in this browser after every change.</p>
    <p><button class="mini" data-goto="base">BeamNG folders and vehicles</button></p>`;
}

function bindProject(el) {
  $('pjsave').onclick = saveProject;
  $('pjopen').onclick = () => $('projfile').click();
  $('pjnew').onclick = newProject;
  el.querySelectorAll('[data-goto]').forEach((b) => { b.onclick = () => setWorkspace(b.dataset.goto); });
}

$('projsave').onclick = saveProject;
$('projopen').onclick = () => $('projfile').click();
$('projfile').onchange = async (e) => {
  const f = e.target.files[0];
  e.target.value = '';
  if (!f) return;
  try { await applyProject(JSON.parse(await f.text())); }
  catch (err) { showIssues([{ level: 'ERROR', rule: 'project', message: `Could not open ${f.name}: ${err.message || err}` }]); }
};

// debug hook for automated UI tests. Not used by the editor itself.
window.beamforge = {
  get skeleton() { return skel; },
  get workspace() { return ws; },
  get powertrain() { return ptEdit; },
  get library() { return library; },
  get wheels() { return wheelsForm; },
  get components() { return compItems; },
  get sketchTool() { return skTool; },
  // where a sketch node is on screen (client px), to click it in a test
  skelScreen(id) {
    const r = renderer.domElement.getBoundingClientRect(), p = v3(skel.res.nodes[id].bng).project(camera);
    return [r.left + (p.x + 1) / 2 * r.width, r.top + (1 - p.y) / 2 * r.height];
  },
  setWorkspace,
  projectData,
  applyProject,
  get veh() { return veh; },
  get svj() { return svjDoc; },
  get hardpoints() { return svjHp; },
  get suspension() { return svjSusp; },
  get meshCount() { let n = 0; meshG.traverse((o) => { if (o.isMesh) n++; }); return n; },
  get folders() { return folders; },
  get catalog() { return cat; },
  get pick() { return pick; },
  get meshParts() { const o = {}; bodyG.traverse((m) => { if (m.isMesh) { const x = (o[m.userData.part] ||= [0, 0]); x[1]++; if (m.visible) x[0]++; } }); return o; },
  set pick(p) { pick = p; drawVehicle(); drawInspector(); },
  setMove,
  addFolder: (dir) => addFolder(dir, null),     // dir: the library.js directory interface
  openVehicle,
};

// start; a failure replaces the loading message
boot().catch((e) => { $('loading').textContent = 'Could not start: ' + e.message; console.error(e); });
