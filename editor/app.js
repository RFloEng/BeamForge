// BeamForge editor prototype: three.js view + Pyodide running the beamforge Python package.
// JavaScript reads files, draws and handles the panels; the vehicle and SVJ logic is in Python.
//
// Map of this file (sections in order):
//   Python engine     boot() loads Pyodide and FILES, imports beamforge.beamng (vehpy) and beamforge.svj (svjpy)
//   3D                scene, groups, camera; drawScene() rebuilds the vehicle and SVJ hardpoint groups
//   base vehicle      BeamNG folders (install, user folder, mods; remembered), vehicle list, parts and tuning
//   SVJ               import (.svj.json + meshes, or .zip bundle), glTF meshes, hardpoints, comparison panel
//   panels            inspector, budget bar, messages, timing
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
  'beamforge/archetype.py'];
const REPO = new URL('../', import.meta.url);

const $ = (id) => document.getElementById(id);
const fmt = (x, d = 1) => (x === null || x === undefined || Number.isNaN(x)) ? '–'
  : x.toLocaleString('en', { minimumFractionDigits: d, maximumFractionDigits: d });
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
// last line of a Python traceback, without the exception class
const pyError = (e) => String(e.message || e).trim().split('\n').pop().replace(/^\w+Error: /, '');

let py, vehpy, svjpy, fitpy, suspy, exppy, valpy;
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
  timing.files = performance.now() - t1;
  $('loading').remove();
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
const vehG = new THREE.Group(), meshG = new THREE.Group(), hpG = new THREE.Group(), bodyG = new THREE.Group(), suspG = new THREE.Group();
scene.add(vehG, meshG, hpG, bodyG, suspG);

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

function drawScene() { drawVehicle(); placeSvj(); drawHardpoints(); drawSuspension(); }
function redraw() { drawScene(); drawVehList(); drawInspector(); drawBudget(); showIssues(issues()); drawTiming(); }

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
  fit: {}, fitReport: null, fitOverrides: {} });
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
    <details open><summary><b>Parts</b> <span class="q">(as the game's Parts menu)</span></summary>
      ${hiddenParts.size || lockedParts.size ? `<p class="quiet">${hiddenParts.size} hidden, ${lockedParts.size} locked
        ${hiddenParts.size ? '<button id="showall" class="mini">show all</button>' : ''}${lockedParts.size ? '<button id="unlockall" class="mini">unlock all</button>' : ''}</p>` : ''}
      <ul class="vehparts">${slotRow(veh.tree, 0)}</ul></details>
    <details><summary><b>Tuning</b> <span class="q">(${veh.variables.length} variables)</span></summary>${tune}</details>
    ${exportPanel()}`;
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
  $('vehcfg').onchange = (e) => { vehEdit = { ...freshEdit(veh.model, e.target.value), moves: vehEdit.moves }; configureVehicle(); };   // a new configuration drops the fit
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
    ${suspPanel()}`;
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
    if (svjOpt && Object.keys(vehEdit.fit || {}).length && vehEdit.fitReport?.axles) {
      svjOpt.axles = vehEdit.fitReport.axles;        // the wheels' camber and toe, set by preloading the upright's beams
    }
    const out = JSON.parse(exppy.build(veh.model, id, name, JSON.stringify(veh), JSON.stringify(exportForm.choices), brand || null,
      svjOpt ? JSON.stringify(svjOpt) : null));
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
  el.innerHTML = vehInspector() + svjInspector();
  bindVehInspector(el);
  bindExport();
  bindFitPanel();
  bindSuspPanel();
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
  if (veh && meshNote) parts.push(meshNote);
  $('timing').textContent = parts.join(' · ');
}

// debug hook for automated UI tests. Not used by the editor itself.
window.beamforge = {
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
