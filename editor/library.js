// The user's BeamNG files as sources: the game install, the user folder (mods, unpacked mods, its own
// vehicles/), single mod folders and picked zips. Only the paths and readers are gathered here; which
// source wins a path is decided in Python (beamforge.beamng.resolve), and files are read on demand.
//
//   source   { name, kind: 'vanilla' | 'mod' | 'user', files: { 'vehicles/...': () => Promise<text> } }
//   folder   { name, role: 'game' | 'user' | 'mod', sources, notes, handle? }   one picked folder
//
// Folders come from two pickers, wrapped as the same small directory interface (entries()):
//   showDirectoryPicker   Chrome / Edge. The handle is kept in IndexedDB, so a later session only
//                         confirms access. Chrome refuses folders under AppData and Program Files.
//   <input webkitdirectory>  any browser and any folder, picked again every session.
import { ZipReader, BlobReader, TextWriter } from 'zipjs';

export const TEXT_FILE = /\.(jbeam|pc|json)$/i;
export const canRemember = typeof window.showDirectoryPicker === 'function';

// ---------- the directory interface: entries() -> Map(lower-case name -> { name, dir } | { name, file }) ----------
export function handleDir(h) {
  let memo = null;
  return {
    name: h.name,
    async entries() {
      if (!memo) {
        memo = new Map();
        for await (const [n, c] of h.entries()) {
          memo.set(n.toLowerCase(), c.kind === 'directory' ? { name: n, dir: handleDir(c) } : { name: n, file: () => c.getFile() });
        }
      }
      return memo;
    },
  };
}

// a FileList from <input webkitdirectory> as a tree; the picked folder is the first path segment
export function listDir(fileList) {
  const mk = (name) => ({ name, kids: new Map() });
  let root = null;
  for (const f of fileList) {
    const parts = (f.webkitRelativePath || f.name).replace(/\\/g, '/').split('/');
    root ||= mk(parts.length > 1 ? parts[0] : '');
    let node = root;
    for (const p of parts.slice(parts.length > 1 ? 1 : 0, -1)) {
      const k = p.toLowerCase();
      if (!node.kids.has(k)) node.kids.set(k, mk(p));
      node = node.kids.get(k);
    }
    node.kids.set(parts[parts.length - 1].toLowerCase(), { name: parts[parts.length - 1], file: f });
  }
  const wrap = (n) => ({
    name: n.name,
    async entries() {
      const out = new Map();
      for (const [k, c] of n.kids) out.set(k, c.kids ? { name: c.name, dir: wrap(c) } : { name: c.name, file: async () => c.file });
      return out;
    },
  });
  return wrap(root || mk(''));
}

async function sub(dir, ...names) {
  for (const n of names) {
    const e = dir && (await dir.entries()).get(n.toLowerCase());
    if (!e || !e.dir) return null;
    dir = e.dir;
  }
  return dir;
}

async function has(dir, name) { return !!dir && (await dir.entries()).has(name.toLowerCase()); }

async function* walk(dir, prefix = '') {
  for (const e of (await dir.entries()).values()) {
    if (e.dir) yield* walk(e.dir, prefix + e.name + '/');
    else yield [prefix + e.name, e];
  }
}

// ---------- sources ----------
// a zip's text files under vehicles/ (the central directory only; entries are read when asked). null: no vehicle files.
// file: a File / Blob, or a zip.js reader (tests read the user's files over HTTP ranges)
export async function zipSource(file, name, kind) {
  const reader = new ZipReader(file instanceof Blob ? new BlobReader(file) : file);
  const files = {};
  for (const e of await reader.getEntries()) {
    const p = e.filename.replace(/\\/g, '/').replace(/^\.?\//, '');
    if (!e.directory && /^vehicles\//i.test(p) && TEXT_FILE.test(p)) files[p] = () => e.getData(new TextWriter());
  }
  return Object.keys(files).length ? { name, kind, files } : null;
}

// loose text files of a folder's vehicles/ subfolder
async function looseSource(vehDir, name, kind) {
  const files = {};
  for await (const [rel, e] of walk(vehDir)) {
    if (TEXT_FILE.test(rel)) files['vehicles/' + rel] = async () => (await e.file()).text();
  }
  return Object.keys(files).length ? { name, kind, files } : null;
}

// the install's vehicle zips: the folder picked may be the install, content/ or content/vehicles/
async function gameVehicles(root) {
  for (const d of [root, await sub(root, 'vehicles'), await sub(root, 'content', 'vehicles')]) {
    if (await has(d, 'common.zip')) return d;
  }
  return null;
}

// the user folder of one game version: the folder picked may be BeamNG.drive/ (with current/ or
// version folders such as 0.36/) or the version folder itself
const userMarks = async (d) => await has(d, 'mods') || await has(d, 'settings');
async function userVersion(root) {
  if (await userMarks(root)) return root;
  const cur = await sub(root, 'current');
  if (cur && await userMarks(cur)) return cur;
  const versions = [...(await root.entries()).values()].filter((e) => e.dir && /^\d+(\.\d+)+$/.test(e.name))
    .sort((a, b) => b.name.localeCompare(a.name, 'en', { numeric: true }));
  for (const v of versions) if (await userMarks(v.dir)) return v.dir;
  return null;
}

// What a picked folder is and its sources. inactive(dbText) -> Set of lower-case mod paths switched off
// in mods/db.json; progress(text) reports the zip being read.
export async function readFolder(root, inactive, progress = () => {}) {
  const game = await gameVehicles(root);
  if (game) {
    const zips = [...(await game.entries()).values()].filter((e) => e.file && /\.zip$/i.test(e.name));
    const sources = [];
    let i = 0;
    for (const z of zips) {
      progress(`Game: ${++i} of ${zips.length} (${z.name})`);
      try { const s = await zipSource(await z.file(), 'game/' + z.name, 'vanilla'); if (s) sources.push(s); } catch (err) { /* not a zip */ }
    }
    return { name: root.name, role: 'game', sources, notes: [`${sources.length} vehicle zips`] };
  }
  const user = await userVersion(root);
  if (user) {
    const sources = [], notes = [];
    const mods = await sub(user, 'mods');
    let off = new Set();
    const db = mods && (await mods.entries()).get('db.json');
    if (db && db.file) off = inactive(await (await db.file()).text());
    let skipped = 0, n = 0;
    if (mods) {
      const zips = [];
      for await (const [rel, e] of walk(mods)) if (/\.zip$/i.test(rel) && !/^unpacked\//i.test(rel)) zips.push(['/mods/' + rel, e]);
      for (const [name, e] of zips) {
        if (off.has(name.toLowerCase())) { skipped++; continue; }
        progress(`Mods: ${++n} of ${zips.length} (${e.name})`);
        try { const s = await zipSource(await e.file(), name, 'mod'); if (s) sources.push(s); } catch (err) { notes.push(`${name}: not a readable zip`); }
      }
      const unpacked = await sub(mods, 'unpacked');
      for (const e of unpacked ? (await unpacked.entries()).values() : []) {
        const name = '/mods/unpacked/' + e.name;
        if (!e.dir) continue;
        if (off.has(name.toLowerCase())) { skipped++; continue; }
        const veh = await sub(e.dir, 'vehicles');
        const s = veh && await looseSource(veh, name, 'mod');
        if (s) sources.push(s);
      }
    }
    const own = await sub(user, 'vehicles');
    const ownSource = own && await looseSource(own, '/vehicles', 'user');
    if (ownSource) sources.push(ownSource);
    notes.unshift(`${sources.length - (ownSource ? 1 : 0)} vehicle mods${skipped ? `, ${skipped} switched off in the game skipped` : ''}`
      + (ownSource ? `, ${Object.keys(ownSource.files).length} own vehicle files` : ''));
    return { name: user === root ? root.name : `${root.name}/${user.name}`, role: 'user', sources, notes };
  }
  const veh = await sub(root, 'vehicles');
  const s = veh && await looseSource(veh, '/folder/' + root.name, 'mod');
  if (s) return { name: root.name, role: 'mod', sources: [s], notes: [`${Object.keys(s.files).length} vehicle files`] };
  return null;
}

// ---------- remembered folders (IndexedDB) ----------
function idb() {
  return new Promise((ok, bad) => {
    const r = indexedDB.open('beamforge', 1);
    r.onupgradeneeded = () => r.result.createObjectStore('kv');
    r.onsuccess = () => ok(r.result);
    r.onerror = () => bad(r.error);
  });
}

async function kv(mode, fn) {
  const db = await idb();
  try {
    return await new Promise((ok, bad) => {
      const t = db.transaction('kv', mode), req = fn(t.objectStore('kv'));
      t.oncomplete = () => ok(req && req.result);
      t.onerror = () => bad(t.error);
    });
  } finally { db.close(); }
}

// the folder handles picked with showDirectoryPicker, in the order picked
export async function rememberedHandles() {
  try { return (await kv('readonly', (s) => s.get('folders'))) || []; } catch (e) { return []; }
}

export async function rememberHandles(handles) {
  try { await kv('readwrite', (s) => s.put(handles, 'folders')); } catch (e) { /* private window: not remembered */ }
}

// read access to a remembered handle: 'granted', or 'prompt' (ask() needs a click)
export async function access(h, ask = false) {
  const opt = { mode: 'read' };
  if (await h.queryPermission(opt) === 'granted') return 'granted';
  return ask ? await h.requestPermission(opt) : 'prompt';
}
