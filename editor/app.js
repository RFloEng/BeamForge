// BeamForge editor prototype: three.js view + Pyodide running the beamforge Python package.
// JavaScript reads files, draws and handles the panels; the vehicle and SVJ logic is in Python.
//
// Map of this file (sections in order):
//   Python engine     boot() loads Pyodide and FILES, imports beamforge.beamng (vehpy) and beamforge.svj (svjpy)
//   3D                scene, groups, camera; drawScene() rebuilds the vehicle and SVJ hardpoint groups
//   base vehicle      library picking (folder, zips, unpacked mods), zip reading, vehicle list, parts and tuning
//   SVJ               import (.svj.json + meshes, or .zip bundle), glTF meshes, hardpoints, comparison panel
//   panels            inspector, budget bar, messages, timing
//
// State (module globals):
//   lib, cat         the picked files and the vehicle catalog (vehpy.catalog)
//   veh, vehEdit     the configured vehicle (vehpy.configure) and the user's slot and tuning edits
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
import { ZipReader, BlobReader, TextWriter } from 'zipjs';   // reads single entries of the game's large vehicle zips

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
// Nothing from the game ships with BeamForge. The user picks their BeamNG.drive/content/vehicles folder
// (or zips, or an unpacked mod folder); only the text files inside (.jbeam, .pc, .json) are read,
// entry by entry with zip.js, and handed to Python (beamforge.beamng). It works like the game's Vehicle
// Config menu: a parts tree with the alternatives for each slot, tuning sliders by category, and a
// .pc configuration to save.
let lib = null, cat = null, veh = null, vehError = null, vehBusy = '';
let vehEdit = { model: null, config: null, parts: {}, vars: {} };
let vehFilter = '', commonLoaded = false;
const TEXT_FILE = /\.(jbeam|pc|json)$/i;

// the picked files: zips by name, and loose files under vehicles/<model>/ (unpacked mods)
function libraryFrom(fileList) {
  const zips = {}, loose = {};
  for (const f of fileList) {
    const rel = (f.webkitRelativePath || f.name).replace(/\\/g, '/');
    if (/\.zip$/i.test(f.name)) zips[f.name.replace(/\.zip$/i, '')] = f;
    else if (TEXT_FILE.test(f.name)) {
      const i = rel.indexOf('vehicles/');
      if (i >= 0) loose[rel.slice(i)] = f;
    }
  }
  return { zips, loose };
}

// text entries of one zip that pass keep(path); reads only those entries, not the whole archive
async function zipTexts(file, keep) {
  const reader = new ZipReader(new BlobReader(file));
  const out = {};
  try {
    for (const e of await reader.getEntries()) {
      const name = e.filename.replace(/\\/g, '/');
      if (!e.directory && keep(name)) out[name] = await e.getData(new TextWriter());
    }
  } finally { await reader.close(); }
  return out;
}

// the vehicle list: info.json and the configs of every vehicle zip / loose folder
async function scanLibrary() {
  const entries = {};
  const zipNames = Object.keys(lib.zips).filter((n) => n.toLowerCase() !== 'common');
  let done = 0;
  for (const z of zipNames) {
    vehBusy = `Reading the vehicle list: ${++done} of ${zipNames.length} (${z})`;
    drawVehList();
    try {
      const texts = await zipTexts(lib.zips[z], (p) => /^vehicles\/[^/]+\/(info[^/]*\.json|[^/]+\.pc)$/i.test(p));
      for (const [p, t] of Object.entries(texts)) {
        const m = p.split('/')[1], base = p.split('/').pop();
        const e = entries[m] || (entries[m] = { info: null, configs: {}, zip: z });
        if (base.toLowerCase() === 'info.json') e.info = t;
        else if (/\.pc$/i.test(base)) { const c = base.replace(/\.pc$/i, ''); if (!(c in e.configs)) e.configs[c] = null; }
        else { const c = base.replace(/^info_/i, '').replace(/\.json$/i, ''); e.configs[c] = t; }
      }
    } catch (err) { /* not a vehicle zip */ }
  }
  for (const [p, f] of Object.entries(lib.loose)) {             // unpacked mods
    const m = p.split('/')[1], base = p.split('/').pop();
    if (!/^(info[^/]*\.json|[^/]+\.pc)$/i.test(base) || p.split('/').length !== 3) continue;
    const e = entries[m] || (entries[m] = { info: null, configs: {}, zip: null });
    const t = await f.text();
    if (base.toLowerCase() === 'info.json') e.info = t;
    else if (/\.pc$/i.test(base)) { const c = base.replace(/\.pc$/i, ''); if (!(c in e.configs)) e.configs[c] = null; }
    else e.configs[base.replace(/^info_/i, '').replace(/\.json$/i, '')] = t;
  }
  const keep = {};                                   // what Python needs: {model: {info, configs: {name: info text}}}
  for (const [m, e] of Object.entries(entries)) keep[m] = { info: e.info, configs: e.configs };
  cat = JSON.parse(vehpy.catalog(JSON.stringify(keep)));
  for (const v of cat) v.zip = entries[v.model].zip;
  vehBusy = '';
}

// load one vehicle's text files (and vehicles/common the first time) into Python, then configure it
async function openVehicle(model, config) {
  const v = cat.find((x) => x.model === model);
  try {
    vehError = null;
    if (!commonLoaded) {
      vehBusy = 'Reading the shared parts (vehicles/common): once per session…'; drawVehList();
      let texts = {};
      if (lib.zips.common) texts = await zipTexts(lib.zips.common, (p) => /^vehicles\/common\/.*\.jbeam$/i.test(p));
      for (const [p, f] of Object.entries(lib.loose)) if (p.startsWith('vehicles/common/')) texts[p] = await f.text();
      vehBusy = `Parsing ${Object.keys(texts).length} shared part files…`; drawVehList();
      await new Promise((r) => setTimeout(r, 30));
      vehpy.add_files(JSON.stringify(texts));
      commonLoaded = true;
    }
    if (!JSON.parse(vehpy.held_models()).includes(model)) {
      vehBusy = `Reading ${v ? v.name : model}…`; drawVehList();
      let texts = {};
      if (v && v.zip) texts = await zipTexts(lib.zips[v.zip], (p) => p.startsWith(`vehicles/${model}/`) && TEXT_FILE.test(p));
      for (const [p, f] of Object.entries(lib.loose)) if (p.startsWith(`vehicles/${model}/`)) texts[p] = await f.text();
      await new Promise((r) => setTimeout(r, 30));
      vehpy.add_files(JSON.stringify(texts));
    }
    vehEdit = { model, config: config || null, parts: {}, vars: {} };
    vehBusy = '';
    configureVehicle();
    fitCamera();
  } catch (err) { vehBusy = ''; vehError = pyError(err); redraw(); }
}

// rebuild the configured vehicle from the held files and the user's edits (fast: no parsing)
function configureVehicle() {
  const t = performance.now();
  try {
    veh = JSON.parse(vehpy.configure(vehEdit.model, vehEdit.config, JSON.stringify(vehEdit.parts), JSON.stringify(vehEdit.vars)));
    vehEdit.config = veh.config;
    vehError = null;
  } catch (err) { vehError = pyError(err); }
  timing.configure = performance.now() - t;
  redraw();
}

// the vehicle's node-and-beam structure: beams as one line set, nodes as points
function drawVehicle() {
  vehG.clear();
  if (!veh) return;
  const g = veh.geometry, pos = [];
  for (const [a, b] of g.beams) pos.push(...v3(g.nodes[a]).toArray(), ...v3(g.nodes[b]).toArray());
  const lines = new THREE.BufferGeometry();
  lines.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  vehG.add(new THREE.LineSegments(lines, new THREE.LineBasicMaterial({ color: 0x8c959f, transparent: true, opacity: 0.55 })));
  const pts = new THREE.BufferGeometry();
  pts.setAttribute('position', new THREE.Float32BufferAttribute(Object.values(g.nodes).flatMap((p) => v3(p).toArray()), 3));
  vehG.add(new THREE.Points(pts, new THREE.PointsMaterial({ color: 0x2f6fdf, size: 0.025 })));
  vehG.visible = $('showbeams').checked;
}
$('showbeams').onchange = () => { vehG.visible = $('showbeams').checked; };

// left panel: pick the install folder, then the vehicle list (cars first)
function drawVehList() {
  const el = $('leftveh');
  if (!lib) {
    el.innerHTML = `<h2>Base vehicle</h2>
      <p class="quiet">Start from any BeamNG vehicle. Pick the <b>content/vehicles</b> folder of your BeamNG.drive install
      (or a mod's unpacked folder, or single vehicle zips). Only its jbeam, config and info files are read, in your browser; nothing is uploaded.</p>
      <div class="inl"><button id="bngpick" class="primary">Choose vehicles folder…</button><button id="bngpickzips">Pick zips…</button></div>`;
    $('bngpick').onclick = () => $('bngdir').click();
    $('bngpickzips').onclick = () => $('bngzips').click();
    return;
  }
  if (vehBusy || !cat) { el.innerHTML = `<h2>Base vehicle</h2><p class="quiet">${esc(vehBusy || 'Reading…')}</p>`; return; }
  const f = vehFilter.toLowerCase();
  const list = cat.filter((v) => !f || `${v.name} ${v.brand} ${v.model} ${v.type}`.toLowerCase().includes(f));
  el.innerHTML = `<h2>Vehicles <span class="q">(${cat.length})</span></h2>
    <div class="inl"><input id="vehfilter" placeholder="Filter" value="${esc(vehFilter)}"></div>
    <ul>${list.map((v) => `<li data-m="${esc(v.model)}" class="${veh && veh.model === v.model ? 'sel' : ''}">
      <span>${esc(v.brand ? v.brand + ' ' : '')}${esc(v.name)}</span><span class="q">${esc(v.type || '')} · ${v.configs.length}</span></li>`).join('')}</ul>
    <div class="inl"><button id="bngagain">Choose another folder…</button><button id="bngmore">Add zips…</button></div>`;
  $('vehfilter').oninput = (e) => { vehFilter = e.target.value; drawVehList(); $('vehfilter').focus(); $('vehfilter').setSelectionRange(vehFilter.length, vehFilter.length); };
  el.querySelectorAll('li[data-m]').forEach((li) => li.onclick = () => openVehicle(li.dataset.m));
  $('bngagain').onclick = () => $('bngdir').click();
  $('bngmore').onclick = () => $('bngzips').click();
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
    return `<li style="padding-left:${depth * 10}px"><span class="q">${esc(n.description || n.slot)}</span>${sel}</li>` +
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
    ${veh.missing.length ? `<p class="bad">Not found in the picked files: ${veh.missing.map((m) => esc(m[1])).join(', ')}</p>` : ''}
    <div class="inl"><button id="vehsave" class="primary">Save configuration (.pc)…</button><button id="vehreset">Reset to the configuration</button></div>
    <details open><summary><b>Parts</b> <span class="q">(as the game's Parts menu)</span></summary><ul class="vehparts">${slotRow(veh.tree, 0)}</ul></details>
    <details><summary><b>Tuning</b> <span class="q">(${veh.variables.length} variables)</span></summary>${tune}</details>`;
}

function bindVehInspector(el) {
  if (!veh) return;
  $('vehcfg').onchange = (e) => { vehEdit = { model: veh.model, config: e.target.value, parts: {}, vars: {} }; configureVehicle(); };
  el.querySelectorAll('select[data-slot]').forEach((s) => s.onchange = () => { vehEdit.parts[s.dataset.slot] = s.value; configureVehicle(); });
  el.querySelectorAll('input[data-var]').forEach((r) => {
    r.oninput = () => { r.nextElementSibling.textContent = fmt(Number(r.value), Number(r.step) < 0.01 ? 3 : Number(r.step) < 1 ? 2 : 0); };
    r.onchange = () => { vehEdit.vars[r.dataset.var] = Number(r.value); configureVehicle(); };
  });
  $('vehreset').onclick = () => { vehEdit = { model: veh.model, config: veh.config, parts: {}, vars: {} }; configureVehicle(); };
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
  lib = libraryFrom(e.target.files); cat = null; veh = null; commonLoaded = false;
  e.target.value = '';
  await scanLibrary(); redraw();
};
$('bngzips').onchange = async (e) => {
  if (!e.target.files.length) return;
  const more = libraryFrom(e.target.files);
  lib = lib ? { zips: { ...lib.zips, ...more.zips }, loose: { ...lib.loose, ...more.loose } } : more;
  e.target.value = '';
  await scanLibrary(); redraw();
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
    <div><span>Tuning variables</span><b>${veh.variables.length}</b> <span>${Object.keys(vehEdit.vars).length} changed</span></div>`;
}

function issues() {
  if (vehError) return [{ level: 'ERROR', rule: 'vehicle', message: vehError }];
  if (!veh) return [{ level: 'INFO', rule: 'vehicle', message: 'Pick your BeamNG vehicles folder, then a vehicle, to use it as a base.' }];
  const out = veh.missing.map((m) => ({ level: 'WARN', rule: 'vehicle', message: `slot ${m[0]}: part ${m[1]} is not in the picked files (pick the whole content/vehicles folder, common.zip included)` }));
  const changed = Object.keys(vehEdit.parts).length + Object.keys(vehEdit.vars).length;
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
};

// start; a failure replaces the loading message
boot().catch((e) => { $('loading').textContent = 'Could not start: ' + e.message; console.error(e); });
