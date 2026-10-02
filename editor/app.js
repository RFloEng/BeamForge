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
//   pick             the selected node or part, moved with numeric x / y / z in the Move panel
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
import { TEXT_FILE, canRemember, handleDir, listDir, zipSource, readFolder, rememberedHandles, rememberHandles, access } from './library.js';

// repo files copied into Pyodide's file system under /bf (add new Python modules here)
const FILES = ['beamforge/__init__.py', 'beamforge/jbeam.py', 'beamforge/beamng.py', 'beamforge/gltf.py', 'beamforge/svj.py'];
const REPO = new URL('../', import.meta.url);

const $ = (id) => document.getElementById(id);
const fmt = (x, d = 1) => (x === null || x === undefined || Number.isNaN(x)) ? '–'
  : x.toLocaleString('en', { minimumFractionDigits: d, maximumFractionDigits: d });
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
// last line of a Python traceback, without the exception class
const pyError = (e) => String(e.message || e).trim().split('\n').pop().replace(/^\w+Error: /, '');

let py, vehpy, svjpy;
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
const vehG = new THREE.Group(), meshG = new THREE.Group(), hpG = new THREE.Group();
scene.add(vehG, meshG, hpG);

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

function drawScene() { drawVehicle(); placeSvj(); drawHardpoints(); }
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
const freshEdit = (model, config) => ({ model, config: config || null, parts: {}, vars: {}, moves: { parts: {}, nodes: {} } });
let vehEdit = freshEdit(null, null);
let pick = null;           // { kind: 'node', id } or { kind: 'part', name }
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

async function pickFolder() {
  let h;
  try { h = await window.showDirectoryPicker({ id: 'beamng', mode: 'read' }); } catch (e) { return; }   // cancelled, or a folder Chrome blocks
  await addFolder(handleDir(h), h);
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
      JSON.stringify(vehEdit.moves)));
    if (pick && (pick.kind === 'node' ? !(pick.id in veh.geometry.nodes) : !Object.values(veh.geometry.parts).includes(pick.name))) pick = null;
    vehEdit.config = veh.config;
    vehError = null;
  } catch (err) {
    vehError = pyError(err);
    if (veh && veh.model !== vehEdit.model) veh = null;     // do not leave the previous vehicle on screen
  }
  timing.configure = performance.now() - t;
  redraw();
}

// the vehicle's node-and-beam structure: beams as one line set, nodes as points. The picked part's nodes
// are drawn orange, the picked node as a larger orange point; nodeIds maps a point index to its node id.
let nodeIds = [];
function drawVehicle() {
  vehG.clear();
  if (!veh) return;
  const g = veh.geometry, pos = [];
  nodeIds = Object.keys(g.nodes);
  for (const [a, b] of g.beams) pos.push(...v3(g.nodes[a]).toArray(), ...v3(g.nodes[b]).toArray());
  const lines = new THREE.BufferGeometry();
  lines.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  vehG.add(new THREE.LineSegments(lines, new THREE.LineBasicMaterial({ color: 0x8c959f, transparent: true, opacity: 0.55 })));
  const pts = new THREE.BufferGeometry();
  pts.setAttribute('position', new THREE.Float32BufferAttribute(Object.values(g.nodes).flatMap((p) => v3(p).toArray()), 3));
  vehG.add(new THREE.Points(pts, new THREE.PointsMaterial({ color: 0x2f6fdf, size: 0.025 })));
  const part = pick && (pick.kind === 'part' ? pick.name : g.parts[pick.id]);
  if (part) {
    const hp = new THREE.BufferGeometry();
    hp.setAttribute('position', new THREE.Float32BufferAttribute(nodeIds.filter((n) => g.parts[n] === part).flatMap((n) => v3(g.nodes[n]).toArray()), 3));
    vehG.add(new THREE.Points(hp, new THREE.PointsMaterial({ color: 0xe0782a, size: pick.kind === 'part' ? 0.05 : 0.035, depthTest: false })));
  }
  if (pick && pick.kind === 'node') {
    const dot = new THREE.Mesh(new THREE.SphereGeometry(0.03, 16, 12), new THREE.MeshBasicMaterial({ color: 0xff5a1f, depthTest: false }));
    dot.position.copy(v3(g.nodes[pick.id]));
    dot.renderOrder = 2;
    vehG.add(dot);
  }
  vehG.visible = $('showbeams').checked;
}

// click a node in the view to pick it (a drag orbits instead): the node nearest the pointer on screen,
// within 10 px, the nearer to the camera on a tie; a click on empty space clears the pick
let downAt = null;
renderer.domElement.addEventListener('pointerdown', (e) => { downAt = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener('pointerup', (e) => {
  if (!downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 4 || e.button !== 0 || !veh || !vehG.visible) return;
  const r = renderer.domElement.getBoundingClientRect(), mx = e.clientX - r.left, my = e.clientY - r.top;
  let best = null, bestD = 10, bestZ = Infinity;
  const p = new THREE.Vector3();
  for (const id of nodeIds) {
    p.copy(v3(veh.geometry.nodes[id])).project(camera);
    if (p.z > 1) continue;                                          // behind the camera
    const d = Math.hypot((p.x + 1) / 2 * r.width - mx, (1 - p.y) / 2 * r.height - my);
    if (d < bestD - 0.5 || (Math.abs(d - bestD) <= 0.5 && p.z < bestZ)) { best = id; bestD = d; bestZ = p.z; }
  }
  pick = best ? { kind: 'node', id: best } : null;
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
    <div class="inl">${canRemember ? '<button id="bngpick" class="primary" title="Remembered for the next sessions (Chrome, Edge)">Add folder…</button>' : ''}
      <button id="bngpickdir" title="Any folder, AppData included; picked again every session">${canRemember ? 'Add with file dialog…' : 'Add folder…'}</button>
      <button id="bngpickzips" title="Single vehicle or mod zips">Add zips…</button>
      ${folders.length || pending.length ? '<button id="bngforget" title="Close the folders and forget the remembered ones">Forget</button>' : ''}</div>
    ${canRemember && !folders.some((f) => f.role === 'user') ? `<p class="quiet">Chrome does not open folders under AppData or Program Files with <i>Add folder</i>.
      Use <i>Add with file dialog</i> for them (every session), or move the user folder elsewhere in the BeamNG launcher so it can be remembered.</p>` : ''}`;
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
  if ($('bngpick')) $('bngpick').onclick = pickFolder;
  $('bngpickdir').onclick = () => $('bngdir').click();
  $('bngpickzips').onclick = () => $('bngzips').click();
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
    const mv = n.part && Object.values(veh.geometry.parts).includes(n.part)
      ? `<button class="mini${pick && pick.kind === 'part' && pick.name === n.part ? ' on' : ''}" data-pickpart="${esc(n.part)}" title="Pick this part to move it">move</button>` : '';
    return `<li style="padding-left:${depth * 10}px"><span class="q">${esc(n.description || n.slot)}${mv}</span>${sel}</li>` +
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
    <details open><summary><b>Parts</b> <span class="q">(as the game's Parts menu)</span></summary><ul class="vehparts">${slotRow(veh.tree, 0)}</ul></details>
    <details><summary><b>Tuning</b> <span class="q">(${veh.variables.length} variables)</span></summary>${tune}</details>`;
}

// the Move panel: the picked node (its position) or part (its offset), as numbers in m, BeamNG axes
const AXES = [['x', 'left +'], ['y', 'rear +'], ['z', 'up +']];
const moveCount = () => Object.keys(vehEdit.moves.parts).length + Object.keys(vehEdit.moves.nodes).length;
function movePanel() {
  const moved = moveCount();
  const all = moved ? `<p class="quiet">${moved} moved (${Object.keys(vehEdit.moves.parts).length} parts, ${Object.keys(vehEdit.moves.nodes).length} nodes).
    <button id="movereset" class="mini">Reset all moves</button> Moves are not saved in the .pc: writing them into a generated part comes later (roadmap step 3).</p>` : '';
  if (!pick) return `<details open><summary><b>Move</b></summary><p class="quiet">Click a node in the view, or <i>move</i> beside a part, to move it with numbers.</p>${all}</details>`;
  const inputs = (vals, kind) => `<div class="kv">${AXES.map(([a, hint], i) => `<span>${a} <span class="q">${hint}</span></span>
    <span><input type="number" step="0.001" data-move="${kind}" data-axis="${i}" value="${Number(vals[i]).toFixed(4)}"> m</span>`).join('')}</div>`;
  let body;
  if (pick.kind === 'node') {
    const part = veh.geometry.parts[pick.id], d = vehEdit.moves.nodes[pick.id];
    body = `<div class="kv"><span>Node</span><span><b>${esc(pick.id)}</b></span><span>Part</span><span>${esc(part)} <button class="mini" data-pickpart="${esc(part)}">move part</button></span></div>
      <p class="quiet">Position:</p>${inputs(veh.geometry.nodes[pick.id], 'node')}
      ${d ? `<p class="quiet">Moved by ${d.map((x) => fmt(x * 1000, 1)).join(' / ')} mm <button id="moveundo" class="mini">Reset node</button></p>` : ''}`;
  } else {
    const d = vehEdit.moves.parts[pick.name] || [0, 0, 0];
    const n = Object.values(veh.geometry.parts).filter((p) => p === pick.name).length;
    body = `<div class="kv"><span>Part</span><span><b>${esc(pick.name)}</b> <span class="q">${n} nodes</span></span></div>
      <p class="quiet">Offset of its own nodes (not of the parts in its slots):</p>${inputs(d, 'part')}
      ${vehEdit.moves.parts[pick.name] ? '<p><button id="moveundo" class="mini">Reset part</button></p>' : ''}`;
  }
  return `<details open><summary><b>Move</b> <button id="moveclear" class="mini">clear pick</button></summary>${body}${all}</details>`;
}

// a typed value: a node's new position becomes a delta (added to its move); a part's value is its offset
function setMove(kind, axis, value) {
  if (!Number.isFinite(value)) return;
  if (kind === 'node') {
    const d = [...(vehEdit.moves.nodes[pick.id] || [0, 0, 0])];
    d[axis] += value - veh.geometry.nodes[pick.id][axis];
    vehEdit.moves.nodes[pick.id] = d.map((x) => Math.round(x * 1e4) / 1e4);
    if (vehEdit.moves.nodes[pick.id].every((x) => x === 0)) delete vehEdit.moves.nodes[pick.id];
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
    pick = { kind: 'part', name: b.dataset.pickpart }; drawVehicle(); drawInspector();
  });
  el.querySelectorAll('input[data-move]').forEach((i) => i.onchange = () => setMove(i.dataset.move, Number(i.dataset.axis), Number(i.value)));
  if ($('moveclear')) $('moveclear').onclick = (e) => { e.preventDefault(); pick = null; drawVehicle(); drawInspector(); };
  if ($('moveundo')) $('moveundo').onclick = () => {
    if (pick.kind === 'node') delete vehEdit.moves.nodes[pick.id]; else delete vehEdit.moves.parts[pick.name];
    configureVehicle();
  };
  if ($('movereset')) $('movereset').onclick = () => { vehEdit.moves = { parts: {}, nodes: {} }; configureVehicle(); };
  $('vehcfg').onchange = (e) => { vehEdit = { ...freshEdit(veh.model, e.target.value), moves: vehEdit.moves }; configureVehicle(); };
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
// single zips: each one a mod source (a game zip picked this way still works, ranked as a mod)
$('bngzips').onchange = async (e) => {
  const picked = [...e.target.files];
  e.target.value = '';
  if (!picked.length) return;
  const old = folders.find((x) => x.name === 'picked zips');
  const sources = old ? old.sources.filter((s) => !picked.some((f) => s.name === 'zip/' + f.name)) : [];
  for (const f of picked) {
    try { const s = await zipSource(f, 'zip/' + f.name, 'mod'); if (s) sources.push(s); } catch (err) { /* not a zip */ }
  }
  if (!sources.length) { showIssues([{ level: 'WARN', rule: 'library', message: 'No vehicle files (vehicles/…) in the picked zips.' }]); return; }
  folders = folders.filter((x) => x !== old);
  folders.push({ name: 'picked zips', role: 'mod', sources, notes: [`${sources.length} zips`], handle: null });
  await libraryChanged();
};

// ---------- SVJ (Standard Vehicle JSON) ----------
let svjDoc = null;         // svjpy.load_bundle result of the last import (kept in memory)
let svjHp = [];            // its hardpoints in BeamNG coordinates, placed on the base vehicle

// where the SVJ origin (front-axle centre on the ground) lands on the base vehicle
const svjPlace = () => ({ yf: veh?.measure?.front_axle_y ?? 0, zg: veh?.measure?.ground_z ?? 0 });

$('svjin').onclick = () => $('svjfile').click();
// Import: an .svj.json with loose mesh files, or a .zip bundle
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
    svjDoc = JSON.parse(svjpy.load_bundle(`${raw}/${f.name}`, '/tmp/svj_in'));
    await loadSvjMeshes();
    redraw();
    if (!veh) fitCamera();
    showIssues([{ level: 'PASS', rule: 'SVJ', message: `Loaded ${f.name}${svjDoc.summary.vehicle ? ' (' + svjDoc.summary.vehicle + ')' : ''}.` },
      ...svjDoc.notes.map((n) => ({ level: 'WARN', rule: 'SVJ', message: n })),
      ...svjDoc.meshes.filter((m) => m.file).map((m) => ({ level: 'INFO', rule: 'SVJ §22', message: `mesh ${m.uri}: nodes ${m.nodes.join(', ') || '(none)'}` }))]);
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
      holder.add(gl.scene);
      holder.userData.uri = m.uri;
      meshG.add(holder);
    } catch (err) {
      showIssues([{ level: 'WARN', rule: 'SVJ §22', message: `could not load mesh ${m.uri}: ${err.message || err}` }]);
    }
  }
  placeSvj();
}
$('meshes').onchange = () => { meshG.visible = $('meshes').checked; };
$('hps').onchange = () => { hpG.visible = $('hps').checked; };

// put the SVJ meshes and hardpoints on the base vehicle's front axle and ground (after every reconfigure)
function placeSvj() {
  if (!svjDoc) { svjHp = []; return; }
  const { yf, zg } = svjPlace();
  const M = gltfToThree(svjDoc.axes, yf, zg);
  for (const h of meshG.children) h.matrix.copy(M);
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
      <p class="quiet">Choosing which values to take onto the base vehicle comes next (docs/roadmap.md, step 3).</p>`;
  }
  return `<h2>SVJ ${esc(s.version || '')}</h2>
    <div class="kv"><span>Vehicle</span><span>${esc(s.vehicle || '–')}</span>
      <span>Corners</span><span>${Object.entries(s.corners).map(([k, t]) => `${esc(k)} ${esc(t || '?')}`).join(', ') || '–'}</span>
      <span>Meshes</span><span>${svjDoc.meshes.length} (${svjDoc.bindings.length} bindings)</span>
      <span>Hardpoints</span><span>${svjHp.length}</span></div>
    ${svjDoc.bindings.length ? `<details><summary>Visual bindings</summary><div class="kv">${svjDoc.bindings.map((b) =>
      `<span>${esc(b.path)}</span><span>${esc(b.node)}</span>`).join('')}</div></details>` : ''}
    ${cmp}`;
}

// ---------- panels ----------
function drawInspector() {
  const el = $('inspector');
  el.innerHTML = vehInspector() + svjInspector();
  bindVehInspector(el);
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
    <div><span>Moved</span><b>${moveCount()}</b> <span>parts and nodes</span></div>`;
}

function issues() {
  if (vehError) return [{ level: 'ERROR', rule: 'vehicle', message: vehError }];
  if (!veh) return [{ level: 'INFO', rule: 'vehicle', message: 'Add your BeamNG folders, then pick a vehicle to use it as a base.' }];
  const out = veh.missing.map((m) => ({ level: 'WARN', rule: 'vehicle', message: `slot ${m[0]}: part ${m[1]} is not in the added folders (add the game install, and the mod that ships it)` }));
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
  $('timing').textContent = parts.join(' · ');
}

// debug hook for automated UI tests. Not used by the editor itself.
window.beamforge = {
  get veh() { return veh; },
  get svj() { return svjDoc; },
  get hardpoints() { return svjHp; },
  get meshCount() { let n = 0; meshG.traverse((o) => { if (o.isMesh) n++; }); return n; },
  get folders() { return folders; },
  get catalog() { return cat; },
  get pick() { return pick; },
  set pick(p) { pick = p; drawVehicle(); drawInspector(); },
  setMove,
  addFolder: (dir) => addFolder(dir, null),     // dir: the library.js directory interface
  openVehicle,
};

// start; a failure replaces the loading message
boot().catch((e) => { $('loading').textContent = 'Could not start: ' + e.message; console.error(e); });
