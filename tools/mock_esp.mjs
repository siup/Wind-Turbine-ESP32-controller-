#!/usr/bin/env node
// Firmware simulator for working on the web panel without a board.
//
//   node tools/mock_esp.mjs [options]          then open http://localhost:8080
//
// Node >= 18, no dependencies. Serves data/ with the same MIME types as the
// ESP and speaks protocol v1 (docs/PROTOCOL.md) over WebSocket /ws and HTTP
// /api/state + /api/cmd. The control logic is a port of the firmware (src/
// and lib/turbine_core/): ramps, the yaw deadman and twist limits, failsafe
// rules, battery thresholds, cfg validation and the telemetry schedule behave
// as on the device, so a panel that works here should work on the hardware.
// Not simulated: BLE, captive portal DNS, the Serial console, flash wear.
//
// Physics (mock only): the nacelle turns 360 deg in 20 s at 100 %, the
// battery sags under motor load and drains slowly. The simulated device
// starts with yawMs360 = 20000 (as if calibrated) so the angle estimate shows.
//
// Test hooks (not on the real device, never delayed by --lag):
//   GET  /__mock/frames[?since=<epoch ms>&t=<type>]  frames received from clients
//   POST /__mock/clear                               forget them
//   POST /__mock/drop                                kill every WebSocket (WiFi loss)
//   POST /__mock/set     {"bat":6.0,...}             change the simulation (keys below)
//   POST /__mock/reboot                              simulated restart
//   GET  /__mock/state                               internal simulation state
//
// /__mock/set keys: bat (V, 0 = no battery), instant (bool: restart the
// battery average so the new value shows within 1-2 s), pos (true nacelle
// angle, deg), magnet (bool, AS5600), sensorReversed (bool: the AS5600 is
// mounted the other way), motorReversed (bool: the yaw motor is wired the
// other way; yawInvert flips the physical direction too, as on the device),
// endL / endR (bool forces a switch, null = from the position), lag (ms),
// noWs (bool), drain (bool), sag (bool), verbose.

import crypto from 'node:crypto';
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));

const HELP = `Usage: node tools/mock_esp.mjs [options]

  --port <n>        HTTP port (default 8080)
  --host <addr>     listen address (default 0.0.0.0, reachable from a phone)
  --sensor <mode>   none | endstops | as5600 (default none)
  --bat <V>         battery voltage at start (default 9.1, 0 = USB power)
  --lag <ms>        delay every outgoing frame and API reply (default 0)
  --no-ws           refuse WebSocket upgrades (forces the HTTP fallback)
  --data <dir>      panel folder (default: data/ next to tools/)
  --verbose         print every frame received from clients
  --help            this text
`;

// ---------------------------------------------------------------------------
// Command line
// ---------------------------------------------------------------------------

function die(msg) {
  process.stderr.write(`mock_esp: ${msg}\n\n${HELP}`);
  process.exit(2);
}

function parseArgs(argv) {
  const o = {
    port: 8080,
    host: '0.0.0.0',
    sensor: 'none',
    bat: 9.1,
    lag: 0,
    noWs: false,
    verbose: false,
    data: path.resolve(HERE, '..', 'data'),
  };
  const num = (name, v, lo, hi, int) => {
    const n = Number(v);
    if (v === undefined || v === '' || !Number.isFinite(n) || n < lo || n > hi || (int && !Number.isInteger(n))) {
      die(`${name} expects a number ${lo}..${hi}`);
    }
    return n;
  };
  for (let i = 0; i < argv.length; i++) {
    let a = argv[i];
    let inline;
    const eq = a.indexOf('=');
    if (a.startsWith('--') && eq > 0) {
      inline = a.slice(eq + 1);
      a = a.slice(0, eq);
    }
    const val = () => {
      if (inline !== undefined) return inline;
      if (i + 1 >= argv.length) die(`${a} needs a value`);
      return argv[++i];
    };
    switch (a) {
      case '--port': o.port = num(a, val(), 1, 65535, true); break;
      case '--host': o.host = val(); break;
      case '--sensor':
        o.sensor = val();
        if (!['none', 'endstops', 'as5600'].includes(o.sensor)) die('--sensor is none, endstops or as5600');
        break;
      case '--bat': o.bat = num(a, val(), 0, 20, false); break;
      case '--lag': o.lag = num(a, val(), 0, 10000, true); break;
      case '--no-ws': o.noWs = true; break;
      case '--data': o.data = path.resolve(val()); break;
      case '--verbose': case '-v': o.verbose = true; break;
      case '--help': case '-h': process.stdout.write(HELP); process.exit(0); break;
      default: die(`unknown option ${a}`);
    }
  }
  return o;
}

const opt = parseArgs(process.argv.slice(2));
const DATA_DIR = opt.data;

// ---------------------------------------------------------------------------
// Constants mirrored from include/config.h and the firmware sources
// ---------------------------------------------------------------------------

const FW_VERSION = '1.0.0';
const PROTOCOL_VERSION = 1;
const BOARD_NAME = 'ESP32';
const MAC = '02:00:00:00:AB:12';  // locally administered, obviously not a real board
const MAX_CMD_LEN = 256;

const MOTOR_TICK_MS = 10;
const MAX_RAMP_DT_MS = 10 * MOTOR_TICK_MS;
const YAW_RAMP_MS = 120;
const FS_RAMP_MS = 200;
const YAW_SPEED_MIN_PCT = 20;
const YAW_SPEED_MAX_PCT = 100;
const YAW_HOLD_TIMEOUT_MS = 700;
const YAW_CAL_MIN_MS = 500;
const ENDSTOP_DEBOUNCE_MS = 20;
const ENDSTOP_RELEASE_DEG = 15;
const ENDSTOP_RELEASE_MIN_MS = 300;
const ENDSTOP_RELEASE_MS = 1500;
const ENDSTOP_SETTLE_MS = 150;  // EndstopSideCheck::kSettleMs
const SEQ_SLOTS = 4;            // StopSeqGuard
const SEQ_WINDOW_MS = 60000;
const ESTOP_LOG_GAP_MS = 1000;
const AS5600_PERIOD_MS = 50;
const AS_RETRY_MS = 500;
const AS_COUNTS = 4096;
const WRONG_WAY_DEG = 15;
const GOTO_PARAMS = { windowDeg: 2, slowZoneDeg: 10, slowPct: 30 };
const YAW_GOTO_TIMEOUT_MS = 10000;

const BAT_R_TOP = 100000;
const BAT_R_BOTTOM = 33000;
const BAT_SAMPLE_MS = 1000;
const BAT_AVG_SAMPLES = 8;
const BAT_OVERSAMPLE = 8;
const BAT_CRIT_HYST_MV = 300;
const BAT_LOW_HYST_MV = 100;
const BAT_CRIT_CONFIRM = 2;
const BAT_PRESENT_MIN_MV = 2500;
const BAT_PRESETS = [
  { batLow: 7000, batCrit: 6200, batEmpty: 6200, batFull: 9400 },  // 9V
  { batLow: 6600, batCrit: 6000, batEmpty: 6000, batFull: 8400 },  // 2S Li-ion
];

const FS_CHECK_MS = 50;
const TELEMETRY_PERIOD_MS = 200;
const TELEMETRY_MIN_GAP_MS = 50;
const TEL_CHECK_MS = 10;
const WS_SILENT_CLOSE_MS = 3000;
const WS_SLOTS = 16;
const WS_KEEP = 4;
const SILENT_CHECK_MS = 100;
const HOUSEKEEP_MS = 1000;
const EVENT_LOG_SIZE = 20;
const LOG_FRAME_CAP = 2304;  // commands.cpp g_logBuf
const AP_SSID_PREFIX = 'Turbina-';
const AP_PASSWORD = 'turbina123';
const OTA_USER = 'admin';
const OTA_REALM = 'Turbina';
const OTA_REBOOT_AFTER_MS = 1000;
const OTA_STALL_MS = 15000;
const OTA_IMAGE_MAGIC = 0xe9;

// Mock only
const PHYS_MS360 = 20000;        // true ms per turn at 100 %
const MOCK_YAW_MS360 = 20000;    // "already calibrated" at the first start
const ENDSTOP_DEG = 170;         // virtual switches at -170 / +170 deg
const HARD_STOP_DEG = 175;       // mechanical stop behind them
const REBOOT_OFFLINE_MS = 3000;  // the device is off the air this long
const FRAMES_KEPT = 10000;

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

const J = JSON.stringify;
const rha = (x) => (x < 0 ? -Math.round(-x) : Math.round(x));  // std::round / lroundf
const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
const sign = (v) => (v > 0 ? 1 : v < 0 ? -1 : 0);
const present = (v) => v !== undefined && v !== null;
const own = (o, k) => (Object.hasOwn(o, k) ? o[k] : undefined);
const isObj = (v) => v !== null && typeof v === 'object' && !Array.isArray(v);
const ascii = (s) => String(s ?? '').replace(/[^\x20-\x7e]/g, '?');

const knobs = {
  lag: opt.lag,
  noWs: opt.noWs,
  verbose: opt.verbose,
  magnet: true,
  sensorReversed: false,
  motorReversed: false,
  endL: null,
  endR: null,
  drain: true,
  sag: true,
};

function stamp(ms) {
  return `[${String(ms).padStart(9)}]`;
}

// Fixed-point with up to `d` decimals, trailing zeros trimmed, at least one
// kept: 1.0, 0.987 (frames.cpp JsonOut::fixed).
function fmtFixed(v, d) {
  if (!Number.isFinite(v)) return 'null';
  const scale = 10 ** d;
  let m = rha(v * scale);
  const neg = m < 0;
  m = Math.abs(m);
  let fp = String(m % scale).padStart(d, '0');
  while (fp.length > 1 && fp.endsWith('0')) fp = fp.slice(0, -1);
  return `${neg ? '-' : ''}${Math.floor(m / scale)}.${fp}`;
}

// ---------------------------------------------------------------------------
// Settings model (lib/turbine_core/src/settings_model.cpp)
// ---------------------------------------------------------------------------

const F_REBOOT = 1;
const F_SECRET = 2;
const F_IDLE = 4;

const FIELDS = [
  ['rotorMax', 'u8', 10, 100, 0],
  ['rotorMin', 'u8', 0, 60, 0],
  ['rotorRamp', 'u16', 50, 10000, 0],
  ['rotorDuty', 'u8', 50, 255, 0],
  ['rotorRev', 'bool', 0, 1, F_IDLE],
  ['rotorMaxRun', 'u16', 0, 3600, 0],
  ['yawSpeed', 'u8', 20, 100, 0],
  ['yawDuty', 'u8', 50, 255, 0],
  ['yawInvert', 'bool', 0, 1, F_IDLE],
  ['yawMaxBal', 'u32', 0, 600000, 0],
  ['yawMs360', 'u32', 0, 600000, 0],
  ['yawMaxDeg', 'u16', 0, 3600, 0],
  ['yawSensRev', 'bool', 0, 1, F_IDLE],
  ['batType', 'u8', 0, 2, 0],
  ['batCal', 'float', 0.5, 1.5, 0],
  ['batLow', 'u16', 3000, 20000, 0],
  ['batCrit', 'u16', 3000, 20000, 0],
  ['batEmpty', 'u16', 3000, 20000, 0],
  ['batFull', 'u16', 3000, 20000, 0],
  ['fsTimeout', 'u16', 400, 1000, 0],
  ['name', 'str', 0, 24, F_REBOOT],
  ['apPass', 'str', 8, 63, F_REBOOT | F_SECRET],
  ['apChan', 'u8', 1, 13, F_REBOOT],
  ['txPower', 'u8', 8, 78, 0],
  ['bleMode', 'u8', 0, 2, 0],
  ['captive', 'bool', 0, 1, F_REBOOT],
  ['led', 'bool', 0, 1, 0],
].map(([key, type, min, max, flags]) => ({ key, type, min, max, flags }));

const FIELD = Object.fromEntries(FIELDS.map((f) => [f.key, f]));
const fieldIndex = (key) => FIELDS.findIndex((f) => f.key === key);

function settingsDefaults() {
  return {
    rotorMax: 100, rotorMin: 0, rotorRamp: 500, rotorDuty: 153, rotorRev: false, rotorMaxRun: 0,
    yawSpeed: 60, yawDuty: 255, yawInvert: false, yawMaxBal: 0, yawMs360: 0, yawMaxDeg: 0, yawSensRev: false,
    batType: 0, batCal: 1.0, ...BAT_PRESETS[0],
    fsTimeout: 500,
    name: '', apPass: AP_PASSWORD, apChan: 1, txPower: 34, bleMode: 1, captive: false,
    led: true,
  };
}

function applyBatteryPreset(s, preset) {
  if (preset === 0 || preset === 1) Object.assign(s, BAT_PRESETS[preset]);
  s.batType = preset;
}

function numberAcceptable(f, v) {
  if (f.type === 'str' || typeof v !== 'number' || !Number.isFinite(v)) return false;
  if (v < f.min || v > f.max) return false;
  return f.type === 'float' || Number.isInteger(v);
}

// Printable ASCII without '"' and '\'.
function strAcceptable(f, v) {
  if (typeof v !== 'string' || v.length < f.min || v.length > f.max) return false;
  return /^[\x20-\x21\x23-\x5b\x5d-\x7e]*$/.test(v);
}

function setNumber(s, f, v) {
  if (!numberAcceptable(f, v)) return false;
  if (f.type === 'bool') s[f.key] = v !== 0;
  else if (f.type === 'float') s[f.key] = Math.fround(v);
  else s[f.key] = v;
  return true;
}

// Index of the first invalid field, or -1.
function settingsValidate(s) {
  for (let i = 0; i < FIELDS.length; i++) {
    const f = FIELDS[i];
    const v = s[f.key];
    if (f.type === 'str') {
      if (!strAcceptable(f, v)) return i;
    } else if (f.type === 'bool') {
      if (typeof v !== 'boolean') return i;
    } else if (!numberAcceptable(f, v)) {
      return i;
    }
  }
  if (s.batCrit >= s.batLow) return fieldIndex('batCrit');
  if (s.batEmpty >= s.batFull) return fieldIndex('batFull');
  return -1;
}

function fieldDiffers(a, b, f) {
  return a[f.key] !== b[f.key];
}

// ---------------------------------------------------------------------------
// Command parser (lib/turbine_core/src/cmd_parser.cpp)
// ---------------------------------------------------------------------------

const CMDS = ['rotor', 'yaw', 'stop', 'ping', 'cfg', 'info', 'log', 'reboot'];

const ERR_MSG = {
  bad_json: 'frame is not a JSON object',
  bad_type: 'missing field or wrong type',
  out_of_range: 'value out of range',
  no_sensor: 'requires the AS5600 sensor',
  failsafe: 'failsafe active',
  low_battery: 'battery critical, start blocked',
  busy_ota: 'update in progress',
  unknown_cmd: 'unknown command',
  too_long: 'frame too long',
  twist_limit: 'cable twist limit reached',
  endstop: 'endstop reached',
  busy: 'motors running or queue full',
  stale: 'sent before the last stop, ignored',
};

// Best-effort "t" of a frame that may be truncated (peekCommandType).
function peekType(text) {
  try {
    const o = JSON.parse(text);
    if (isObj(o)) return typeof o.t === 'string' && CMDS.includes(o.t) ? o.t : null;
  } catch {
    // fall through to the scan
  }
  const m = /^\s*\{[^]*?"t"\s*:\s*"([a-z]+)"/.exec(text);
  return m && CMDS.includes(m[1]) ? m[1] : null;
}

function parseFail(err, field, cmd) {
  return { err, field: field ?? null, cmd };
}

function parseCommand(text, base) {
  const cmd = { type: null, hasId: false, id: 0 };
  const ok = { err: null, field: null, cmd };
  if (Buffer.byteLength(text, 'utf8') > MAX_CMD_LEN) {
    cmd.type = peekType(text);
    return parseFail('too_long', null, cmd);
  }
  let obj;
  try {
    obj = JSON.parse(text);
  } catch {
    return parseFail('bad_json', null, cmd);
  }
  if (!isObj(obj)) return parseFail('bad_json', null, cmd);

  // "id" first so that every later error can still be paired by the client.
  let idBad = false;
  const idv = own(obj, 'id');
  if (present(idv)) {
    if (Number.isInteger(idv) && idv >= 0 && idv <= 0xffffffff) {
      cmd.hasId = true;
      cmd.id = idv;
    } else {
      idBad = true;
    }
  }
  const t = own(obj, 't');
  if (typeof t !== 'string') return parseFail('bad_type', 't', cmd);
  if (!CMDS.includes(t)) return parseFail('unknown_cmd', null, cmd);
  cmd.type = t;
  if (idBad) return parseFail('bad_type', 'id', cmd);
  cmd.hasSeq = false;
  const sq = own(obj, 'seq');
  if (present(sq)) {
    if (!(Number.isInteger(sq) && sq >= 0 && sq <= 0xffffffff)) return parseFail('bad_type', 'seq', cmd);
    cmd.hasSeq = true;
    cmd.seq = sq;
  }

  let r = null;
  if (t === 'rotor') r = parseRotor(obj, cmd);
  else if (t === 'yaw') r = parseYaw(obj, cmd);
  else if (t === 'cfg') r = parseCfg(obj, cmd, base);
  return r ?? ok;
}

function roundedPct(v, lo, hi) {
  if (!Number.isFinite(v)) return null;
  const r = rha(v) + 0;  // + 0 turns -0 into 0
  return r < lo || r > hi ? null : r;
}

function parseRotor(obj, cmd) {
  const sp = own(obj, 'speed');
  if (typeof sp !== 'number') return parseFail('bad_type', 'speed', cmd);
  const r = roundedPct(sp, 0, 100);
  if (r === null) return parseFail('out_of_range', 'speed', cmd);
  cmd.speed = r;
  cmd.brake = false;
  const br = own(obj, 'brake');
  if (present(br)) {
    if (typeof br !== 'boolean') return parseFail('bad_type', 'brake', cmd);
    cmd.brake = br;
  }
  return null;
}

function parseYaw(obj, cmd) {
  const keys = ['dir', 'hold', 'goto', 'zero', 'cal'];
  let which = null;
  for (const k of keys) {
    if (!present(own(obj, k))) continue;
    if (which !== null) return parseFail('bad_type', k, cmd);  // exactly one allowed
    which = k;
  }
  if (which === null) return parseFail('bad_type', 'dir', cmd);
  const v = own(obj, which);
  cmd.yawSpeed = 0;
  switch (which) {
    case 'dir': {
      if (typeof v !== 'string') return parseFail('bad_type', 'dir', cmd);
      cmd.action = 'move';
      if (v === 'stop') {
        cmd.dir = 0;
        return null;  // a stop never fails on an optional field
      }
      if (v !== 'left' && v !== 'right') return parseFail('out_of_range', 'dir', cmd);
      cmd.dir = v === 'left' ? -1 : 1;
      const sp = own(obj, 'speed');
      if (present(sp)) {
        if (typeof sp !== 'number') return parseFail('bad_type', 'speed', cmd);
        const r = roundedPct(sp, 20, 100);
        if (r === null) return parseFail('out_of_range', 'speed', cmd);
        cmd.yawSpeed = r;
      }
      return null;
    }
    case 'hold':
      if (typeof v !== 'string') return parseFail('bad_type', 'hold', cmd);
      if (v !== 'left' && v !== 'right') return parseFail('out_of_range', 'hold', cmd);
      cmd.action = 'hold';
      cmd.dir = v === 'left' ? -1 : 1;
      return null;
    case 'goto':
      if (typeof v !== 'number') return parseFail('bad_type', 'goto', cmd);
      if (!Number.isFinite(v) || v < 0 || v >= 360) return parseFail('out_of_range', 'goto', cmd);
      cmd.action = 'goto';
      cmd.gotoDeg = Math.fround(v) >= 360 ? 0 : v;
      return null;
    case 'zero':
      if (typeof v !== 'boolean') return parseFail('bad_type', 'zero', cmd);
      if (!v) return parseFail('out_of_range', 'zero', cmd);
      cmd.action = 'zero';
      return null;
    default:
      if (typeof v !== 'string') return parseFail('bad_type', 'cal', cmd);
      if (v === 'start') cmd.action = 'calStart';
      else if (v === 'stop') cmd.action = 'calStop';
      else if (v === 'cancel') cmd.action = 'calCancel';
      else return parseFail('out_of_range', 'cal', cmd);
      return null;
  }
}

function applyField(work, f, v, cmd) {
  if (f.type === 'str') {
    if (typeof v !== 'string') return parseFail('bad_type', f.key, cmd);
    if (!strAcceptable(f, v)) return parseFail('out_of_range', f.key, cmd);
    work[f.key] = v;
  } else if (f.type === 'bool') {
    if (typeof v !== 'boolean') return parseFail('bad_type', f.key, cmd);
    work[f.key] = v;
  } else {
    if (typeof v !== 'number') return parseFail('bad_type', f.key, cmd);
    if (!setNumber(work, f, v)) return parseFail('out_of_range', f.key, cmd);
  }
  return null;
}

// cfg get / reset / atomic patch. With `base` the patch is applied to a copy
// and cross-checked; cmd.next is set only if the whole patch is valid.
function parseCfg(obj, cmd, base) {
  cmd.cfgGet = false;
  cmd.cfgReset = false;
  cmd.cfgKeys = [];
  const get = own(obj, 'get');
  const reset = own(obj, 'reset');
  if (present(get) && typeof get !== 'boolean') return parseFail('bad_type', 'get', cmd);
  if (present(reset) && typeof reset !== 'boolean') return parseFail('bad_type', 'reset', cmd);
  if (reset === true) {
    cmd.cfgReset = true;
    return null;
  }
  if (get === true) {
    cmd.cfgGet = true;
    return null;
  }
  const work = { ...(base ?? settingsDefaults()) };
  // Battery preset first, so explicit thresholds in the same patch win.
  const bt = own(obj, 'batType');
  if (present(bt)) {
    const r = applyField(work, FIELD.batType, bt, cmd);
    if (r) return r;
    if (work.batType !== 2) applyBatteryPreset(work, work.batType);
  }
  for (const f of FIELDS) {
    const v = own(obj, f.key);
    if (!present(v)) continue;
    const r = applyField(work, f, v, cmd);
    if (r) return r;
    cmd.cfgKeys.push(f.key);
  }
  if (cmd.cfgKeys.length === 0) {
    cmd.cfgGet = true;  // {"t":"cfg"} or only unknown keys: a read
    return null;
  }
  if (base) {
    const bad = settingsValidate(work);
    if (bad >= 0) return parseFail('out_of_range', FIELDS[bad].key, cmd);
    cmd.next = work;
  }
  return null;
}

// ---------------------------------------------------------------------------
// Frame builders (lib/turbine_core/src/frames.cpp), same key order
// ---------------------------------------------------------------------------

function buildAck(of, hasId, id) {
  return `{"t":"ack","of":${J(of ?? 'unknown')}${hasId ? `,"id":${id}` : ''}}`;
}

function buildErr(of, code, field, msg, hasId, id) {
  let s = `{"t":"err","of":${J(of ?? 'unknown')},"code":${J(code)}`;
  if (field) s += `,"field":${J(field)}`;
  s += `,"msg":${J(ascii(msg ?? ERR_MSG[code] ?? ''))}`;
  if (hasId) s += `,"id":${id}`;
  return `${s}}`;
}

function buildPong(hasId, id, up) {
  return `{"t":"pong"${hasId ? `,"id":${id}` : ''},"up":${up}}`;
}

function buildState(s, brief) {
  const o = { t: 'state', v: PROTOCOL_VERSION };
  o.rotor = brief
    ? { target: s.rotorTarget, actual: s.rotorActual }
    : { target: s.rotorTarget, actual: s.rotorActual, run: s.rotorRunS };
  const movingName = s.yawMoving < 0 ? 'left' : s.yawMoving > 0 ? 'right' : null;
  if (brief) {
    o.yaw = { moving: movingName, angle: s.hasAngle ? s.angle : null };
    o.bat = { pct: s.batPct, crit: s.batCrit };
  } else {
    o.yaw = {
      moving: movingName,
      speed: s.yawSpeed,
      angle: s.hasAngle ? s.angle : null,
      limitL: s.limitL,
      limitR: s.limitR,
      twist: s.twist < 0 ? 'L' : s.twist > 0 ? 'R' : null,
      bal: s.bal,
      balMax: s.balMax,
      pos: s.hasPos ? s.pos : null,
      est: s.posEst,
      posMax: s.hasPosMax ? s.posMax : null,
      goto: s.gotoActive ? s.gotoTarget : null,
      sensor: s.sensor,
      sensorOk: s.sensorOk,
      cal: s.calActive,
    };
    o.bat = { mv: s.batMv, pct: s.batPct, low: s.batLow, crit: s.batCrit, present: s.batPresent };
    o.link = { rssi: s.hasRssi ? s.rssi : null, clients: s.ws + s.ble, ws: s.ws, ble: s.ble, sta: s.sta };
  }
  o.fs = { active: s.fsActive, reason: s.fsReason };
  if (!brief) {
    o.ota = s.ota;
    o.cfgRev = s.cfgRev;
    o.up = s.upMs;
  }
  return J(o);
}

// What counts as "changed" for the immediate send (stateSignature): no up,
// rssi, bat.mv, rotor.run; an idle yaw angle / pos only on a 2 degree step.
function stateSignature(s) {
  const idle = s.yawMoving === 0 && !s.gotoActive;
  const half = (v) => (idle ? Math.trunc(v / 2) : v);
  return [
    s.rotorTarget, s.rotorActual, s.yawMoving, s.yawSpeed, s.hasAngle, s.hasAngle ? half(s.angle) : '',
    s.limitL, s.limitR, s.twist, s.bal, s.balMax, s.hasPos, s.hasPos ? half(s.pos) : '', s.posEst,
    s.hasPosMax, s.hasPosMax ? s.posMax : '', s.gotoActive, s.gotoActive ? s.gotoTarget : '', s.sensor,
    s.sensorOk, s.calActive, s.batPct, s.batLow, s.batCrit, s.batPresent, s.ws, s.ble, s.sta,
    s.fsActive, s.fsReason, s.ota, s.cfgRev,
  ].join('|');
}

function buildCfg(s, rev, rebootPending) {
  let out = `{"t":"cfg","rev":${rev},"reboot":${rebootPending},"apPassSet":${s.apPass.length > 0}`;
  for (const f of FIELDS) {
    if (f.flags & F_SECRET) continue;
    out += `,${J(f.key)}:${f.type === 'float' ? fmtFixed(s[f.key], 3) : J(s[f.key])}`;
  }
  return `${out}}`;
}

function logItemJson(it) {
  return `"ms":${it.ms},"lvl":${J(it.lvl)},"code":${J(it.code)},"msg":${J(it.msg)}`;
}

// ---------------------------------------------------------------------------
// Ramp and motor (lib/turbine_core/src/ramp.h, src/motors.cpp)
// ---------------------------------------------------------------------------

class Ramp {
  static FULL = 100000;  // 100 % in milli-percent

  constructor(fullScaleMs) {
    this.value = 0;
    this.target = 0;
    this.fullScaleMs = fullScaleMs;
    this.rem = 0;
  }

  setTargetMilli(m) {
    m = clamp(m, -Ramp.FULL, Ramp.FULL);
    if (m !== this.target) this.rem = 0;
    this.target = m;
  }

  jumpTo(pct) {
    this.setTargetMilli(pct * 1000);
    this.value = this.target;
    this.rem = 0;
  }

  update(dt) {
    if (this.value === this.target) {
      this.rem = 0;
      return;
    }
    if (this.fullScaleMs === 0) {
      this.value = this.target;
      this.rem = 0;
      return;
    }
    const num = dt * Ramp.FULL + this.rem;
    const step = Math.floor(num / this.fullScaleMs);
    this.rem = num % this.fullScaleMs;
    if (step === 0) return;
    const diff = this.target - this.value;
    if (Math.abs(diff) <= step) {
      this.value = this.target;
      this.rem = 0;
    } else {
      this.value += diff < 0 ? -step : step;
    }
  }

  settled() {
    return this.value === this.target;
  }

  static pct(m) {
    return m >= 0 ? Math.trunc((m + 500) / 1000) : Math.trunc((m - 500) / 1000);
  }
}

class Motor {
  constructor(rampMs) {
    this.rampMs = rampMs;
    this.ramp = new Ramp(rampMs);
    this.override = 0;
    this.lastTick = 0;
  }

  begin(now) {
    this.ramp.jumpTo(0);
    this.lastTick = now;
  }

  configure(rampMs) {
    this.rampMs = rampMs;
    if (!this.override) this.ramp.fullScaleMs = rampMs;
  }

  setSpeed(pct) {
    const m = clamp(pct, -100, 100) * 1000;
    // A running failsafe ramp-down survives a "speed 0" that does not change the target.
    if (this.override && m !== this.ramp.target) {
      this.override = 0;
      this.ramp.fullScaleMs = this.rampMs;
    }
    this.ramp.setTargetMilli(m);
  }

  setSpeedWithRamp(pct, rampMs) {
    this.override = rampMs || 1;
    this.ramp.fullScaleMs = this.override;
    this.ramp.setTargetMilli(clamp(pct, -100, 100) * 1000);
  }

  // Coast (and brake, which looks the same from outside): output 0 now.
  stop() {
    this.override = 0;
    this.ramp.fullScaleMs = this.rampMs;
    this.ramp.jumpTo(0);
  }

  brake() {
    this.stop();
  }

  tick(now) {
    let dt = now - this.lastTick;
    if (dt < MOTOR_TICK_MS) return;
    this.lastTick = now;
    if (dt > MAX_RAMP_DT_MS) dt = MAX_RAMP_DT_MS;
    this.ramp.update(dt);
    if (this.override && this.ramp.settled()) {
      this.override = 0;
      this.ramp.fullScaleMs = this.rampMs;
    }
  }

  target() { return Ramp.pct(this.ramp.target); }
  actual() { return Ramp.pct(this.ramp.value); }
  actualMilli() { return this.ramp.value; }
  idle() { return this.ramp.target === 0 && this.ramp.value === 0; }
}

// ---------------------------------------------------------------------------
// Battery (lib/turbine_core/src/battery_math.cpp, src/battery.cpp)
// ---------------------------------------------------------------------------

class MovingAverage {
  constructor(n) {
    this.n = n;
    this.reset();
  }

  reset() {
    this.buf = [];
    this.sum = 0;
  }

  push(v) {
    if (this.buf.length === this.n) this.sum -= this.buf.shift();
    this.buf.push(v);
    this.sum += v;
  }

  count() { return this.buf.length; }

  mean() {
    const c = this.buf.length;
    return c ? Math.floor((this.sum + Math.floor(c / 2)) / c) : 0;
  }
}

function batteryMvFromAdc(adcMv, cal) {
  const mv = (adcMv * (BAT_R_TOP + BAT_R_BOTTOM)) / BAT_R_BOTTOM * cal;
  return mv > 0 ? Math.floor(mv + 0.5) : 0;
}

function batteryPercent(mv, emptyMv, fullMv) {
  if (fullMv <= emptyMv) return mv >= fullMv ? 100 : 0;
  if (mv <= emptyMv) return 0;
  if (mv >= fullMv) return 100;
  const range = fullMv - emptyMv;
  return Math.floor(((mv - emptyMv) * 100 + Math.floor(range / 2)) / range);
}

class BatteryMonitor {
  constructor() {
    this.cfg = { lowMv: 7000, critMv: 6200 };
    this.reset();
  }

  reset() {
    this.low = false;
    this.crit = false;
    this.present = true;
    this.below = 0;
  }

  // Returns true if the crit latch changed.
  update(mv) {
    const wasCrit = this.crit;
    this.present = mv >= BAT_PRESENT_MIN_MV;
    if (!this.present) {
      // No new latch while absent, but an existing one is never cleared.
      this.below = 0;
      this.low = this.crit;
      return false;
    }
    if (this.crit) {
      if (mv >= this.cfg.critMv + BAT_CRIT_HYST_MV) {
        this.crit = false;
        this.below = 0;
      }
    } else if (mv < this.cfg.critMv) {
      this.below++;
      if (this.below >= BAT_CRIT_CONFIRM) this.crit = true;
    } else {
      this.below = 0;
    }
    if (this.crit || mv < this.cfg.lowMv) this.low = true;
    else if (mv >= this.cfg.lowMv + BAT_LOW_HYST_MV) this.low = false;
    return this.crit !== wasCrit;
  }
}

class BatteryMeter {
  constructor(dev) {
    this.dev = dev;
    this.avg = new MovingAverage(BAT_AVG_SAMPLES);
    this.mon = new BatteryMonitor();
    this.cal = 1;
    this.emptyMv = BAT_PRESETS[0].batEmpty;
    this.fullMv = BAT_PRESETS[0].batFull;
    this.mv = 0;
    this.adcMv = 0;
    this.pct = 0;
    this.nextMs = 0;
    this.samplePresent = false;
    this.wasLow = false;
    this.wasPresent = true;
  }

  begin(now) {
    this.mon.cfg = { lowMv: BAT_PRESETS[0].batLow, critMv: BAT_PRESETS[0].batCrit };
    this.mon.reset();
    this.avg.reset();
    this.wasLow = false;
    this.wasPresent = this.mon.present;
    this.samplePresent = false;
    this.sample();
    this.nextMs = now + BAT_SAMPLE_MS;
  }

  configure(s) {
    this.cal = s.batCal;
    this.mon.cfg = { lowMv: s.batLow, critMv: s.batCrit };
    this.emptyMv = s.batEmpty;
    this.fullMv = s.batFull;
    if (this.avg.count() > 0) {
      this.mv = batteryMvFromAdc(this.avg.mean(), this.cal);
      this.pct = this.mon.present ? batteryPercent(this.mv, this.emptyMv, this.fullMv) : 0;
    }
  }

  tick(now) {
    if (now - this.nextMs < 0) return;
    this.nextMs = now + BAT_SAMPLE_MS;
    this.sample();
  }

  // Test hook: forget the average so a new voltage shows at once.
  restart() {
    this.avg.reset();
    this.sample();
    this.nextMs = this.dev.millis() + BAT_SAMPLE_MS;
  }

  sample() {
    let sum = 0;
    for (let i = 0; i < BAT_OVERSAMPLE; i++) sum += phys.adcPinMv();
    this.adcMv = Math.floor((sum + BAT_OVERSAMPLE / 2) / BAT_OVERSAMPLE);
    const isPresent = batteryMvFromAdc(this.adcMv, this.cal) >= BAT_PRESENT_MIN_MV;
    if (isPresent !== this.samplePresent) {
      this.avg.reset();
      this.samplePresent = isPresent;
    }
    this.avg.push(this.adcMv);
    this.mv = batteryMvFromAdc(this.avg.mean(), this.cal);
    const critChanged = this.mon.update(this.mv);
    this.pct = this.mon.present ? batteryPercent(this.mv, this.emptyMv, this.fullMv) : 0;
    const ev = this.dev.events;
    if (this.mon.present !== this.wasPresent) {
      this.wasPresent = this.mon.present;
      if (this.wasPresent) ev.add('i', 'bat_present', `${this.mv} mV`);
      else ev.add('w', 'bat_absent', `${this.mv} mV, USB power?`);
    }
    if (critChanged) {
      if (this.mon.crit) ev.add('e', 'bat_crit', `${this.mv} mV, motors locked`);
      else ev.add('i', 'bat_ok', `${this.mv} mV, crit released`);
    }
    if (this.mon.low && !this.wasLow && !this.mon.crit) ev.add('w', 'bat_low', `${this.mv} mV`);
    this.wasLow = this.mon.low;
  }

  low() { return this.mon.low; }
  crit() { return this.mon.crit; }
  present() { return this.mon.present; }
}

// ---------------------------------------------------------------------------
// Yaw logic (lib/turbine_core/src/yaw_logic.cpp)
// ---------------------------------------------------------------------------

class YawBalance {
  constructor() { this.reset(); }

  reset() {
    this.acc = 0;  // balance * 1000
    this.refL = false;
    this.refR = false;
    this.refLAcc = 0;
    this.refRAcc = 0;
  }

  restore(balance) {
    this.reset();
    this.acc = balance * 1000;
  }

  balance() {
    return this.acc >= 0 ? Math.trunc((this.acc + 500) / 1000) : Math.trunc((this.acc - 500) / 1000);
  }

  integrate(dir, pct, dt) {
    if (!dir || !pct || dt <= 0) return;
    const d = dt * Math.min(pct, 100) * 10;
    this.acc = clamp(this.acc + (dir > 0 ? d : -d), -2e12, 2e12);
  }

  blocked(dir, limit) {
    if (!limit || !dir) return false;
    const b = this.balance();
    return dir > 0 ? b >= limit : b <= -limit;
  }

  onEndstop(side) {
    if (side < 0) {
      if (this.refL) this.acc = this.refLAcc;
      else { this.refL = true; this.refLAcc = this.acc; }
    } else if (side > 0) {
      if (this.refR) this.acc = this.refRAcc;
      else { this.refR = true; this.refRAcc = this.acc; }
    }
  }

  estimateDeg(ms360) {
    return ms360 ? (this.acc / 1000 / ms360) * 360 : null;
  }
}

class EndstopDebouncer {
  constructor() { this.reset(false, 0); }

  reset(raw, now) {
    this.stable = raw;
    this.last = raw;
    this.rose = false;
    this.changedAt = now;
  }

  update(raw, now, debounceMs) {
    this.rose = false;
    if (raw !== this.last) {
      this.last = raw;
      this.changedAt = now;
    }
    if (this.last !== this.stable && now - this.changedAt >= debounceMs) {
      this.stable = this.last;
      this.rose = this.stable;
    }
    return this.stable;
  }
}

// lib/turbine_core/src/yaw_logic.cpp EndstopSideCheck: a switch that closes
// while the yaw drives away from it (or stays closed while it does) means
// the motor turns the other way; that direction is refused.
class EndstopSideCheck {
  constructor() { this.reset(); }

  reset() {
    this.refused = 0;
    this.held = 0;
    this.heldBal = 0;
    this.droveL = false;
    this.droveR = false;
    this.lastL = 0;
    this.lastR = 0;
  }

  moveStarted(dir, lActive, rActive, balance) {
    this.held = 0;
    if (lActive && !rActive && dir > 0) this.held = -1;
    if (rActive && !lActive && dir < 0) this.held = 1;
    this.heldBal = balance;
  }

  recently(side, now) {
    return side < 0 ? this.droveL && now - this.lastL < ENDSTOP_SETTLE_MS : this.droveR && now - this.lastR < ENDSTOP_SETTLE_MS;
  }

  blocked(dir) { return dir !== 0 && dir === this.refused; }

  // in: { now, drive, commanded, lActive, rActive, lRose, rRose, balance, releaseTravel }
  update(inp) {
    const r = { wrong: 0, refL: false, refR: false };
    if (inp.drive < 0) {
      this.droveL = true;
      this.lastL = inp.now;
    } else if (inp.drive > 0) {
      this.droveR = true;
      this.lastR = inp.now;
    }
    const edge = (side) => {
      const toward = inp.drive === side || this.recently(side, inp.now);
      const away = inp.drive === -side || (inp.drive === 0 && this.recently(-side, inp.now));
      if (away && !toward) r.wrong = -side;
      else if (away) {
        this.held = side;
        this.heldBal = inp.balance;
      } else if (side < 0) r.refL = true;
      else r.refR = true;
    };
    if (inp.lRose) edge(-1);
    if (inp.rRose) edge(1);
    if (r.wrong === 0 && this.held !== 0) {
      const closed = this.held < 0 ? inp.lActive : inp.rActive;
      if (!closed || inp.commanded === 0) this.held = 0;
      else if (inp.commanded === -this.held && inp.drive === -this.held && Math.abs(inp.balance - this.heldBal) >= inp.releaseTravel) {
        r.wrong = -this.held;
      }
    }
    if (r.wrong !== 0) {
      this.refused = r.wrong;
      this.held = 0;
      r.refL = r.refR = false;
    } else if (this.refused !== 0 && !inp.lActive && !inp.rActive) {
      this.refused = 0;
    }
    return r;
  }
}

function median3(a, b, c) {
  if (a > b) [a, b] = [b, a];
  if (c <= a) return a;
  if (c >= b) return b;
  return c;
}

function wrap360(deg) {
  if (!Number.isFinite(deg)) return 0;
  let r = deg % 360;
  if (r < 0) r += 360;
  if (r >= 360) r = 0;
  return r;
}

function shortestDelta(fromAbs, toAbs) {
  let d = wrap360(toAbs - fromAbs);
  if (d > 180) d -= 360;
  return d;
}

class AngleUnwrapper {
  constructor() { this.reset(0); }

  reset(hint) {
    this.init = false;
    this.hint = Number.isFinite(hint) ? hint : 0;
    this.lastAbs = wrap360(this.hint);
    this.cont = this.hint;
  }

  update(absDeg) {
    const a = wrap360(absDeg);
    let turns;
    if (!this.init) {
      turns = rha((this.hint - a) / 360);
      this.init = true;
    } else {
      turns = rha((this.cont - this.lastAbs) / 360);
      const step = a - this.lastAbs;
      if (step > 180) turns -= 1;
      if (step < -180) turns += 1;
    }
    this.lastAbs = a;
    this.cont = turns * 360 + a;
    return this.cont;
  }
}

function planGoto(currentCont, targetAbs, cruisePct, maxTwistDeg, p) {
  const st = { dir: 0, speedPct: 0, done: false, impossible: false };
  if (!Number.isFinite(currentCont) || !Number.isFinite(targetAbs)) {
    st.impossible = true;
    return st;
  }
  const shortWay = shortestDelta(wrap360(currentCont), targetAbs);
  if (Math.abs(shortWay) <= p.windowDeg) {
    st.done = true;
    return st;
  }
  let delta = shortWay;
  if (maxTwistDeg > 0) {
    const longWay = shortWay > 0 ? shortWay - 360 : shortWay + 360;
    const shortOk = Math.abs(currentCont + shortWay) <= maxTwistDeg;
    const longOk = Math.abs(currentCont + longWay) <= maxTwistDeg;
    if (shortOk) delta = shortWay;
    else if (longOk) delta = longWay;
    else {
      st.impossible = true;
      return st;
    }
  }
  const slow = Math.min(p.slowPct, 100);
  const cruise = Math.min(cruisePct, 100);
  st.dir = delta > 0 ? 1 : -1;
  st.speedPct = Math.abs(delta) <= p.slowZoneDeg ? slow : Math.max(cruise, slow);
  return st;
}

// ---------------------------------------------------------------------------
// Physics (mock only): true nacelle angle, sensors, battery
// ---------------------------------------------------------------------------

class Physics {
  constructor() {
    this.truePos = 0;               // deg, continuous
    this.ocvMv = opt.bat * 1000;    // open-circuit battery voltage
    this.rotorLoad = 0;
    this.yawLoad = 0;
    this.lastAt = performance.now();
  }

  // Called every loop pass with the current motor outputs.
  advance(yawMilli, rotorMilli, s) {
    const t = performance.now();
    const dt = Math.min(t - this.lastAt, 200);
    this.lastAt = t;
    if (yawMilli) {
      // yawInvert is applied at the bridge output, the wiring is the knob.
      const dir = (s.yawInvert ? -1 : 1) * (knobs.motorReversed ? -1 : 1);
      this.truePos += ((dir * yawMilli / Ramp.FULL) * 360 / PHYS_MS360) * dt;
      if (opt.sensor === 'endstops') this.truePos = clamp(this.truePos, -HARD_STOP_DEG, HARD_STOP_DEG);
    }
    this.rotorLoad = (Math.abs(rotorMilli) / Ramp.FULL) * (s.rotorDuty / 255);
    this.yawLoad = (Math.abs(yawMilli) / Ramp.FULL) * (s.yawDuty / 255);
    if (knobs.drain && this.ocvMv >= BAT_PRESENT_MIN_MV) {
      this.ocvMv -= (0.03 + 2.0 * this.rotorLoad + 1.0 * this.yawLoad) * (dt / 1000);
    }
  }

  terminalMv() {
    if (this.ocvMv < 100) return 0;
    const sag = knobs.sag ? 600 * this.rotorLoad + 300 * this.yawLoad : 0;
    return Math.max(0, this.ocvMv - sag);
  }

  // analogReadMilliVolts() on the divider: 11 dB range ends near 3.1 V.
  adcPinMv() {
    const pin = (this.terminalMv() * BAT_R_BOTTOM) / (BAT_R_TOP + BAT_R_BOTTOM);
    if (pin <= 0) return 0;
    return clamp(Math.round(pin + (Math.random() * 8 - 4)), 0, 3100);
  }

  endstopRaw(side) {
    const forced = side < 0 ? knobs.endL : knobs.endR;
    if (forced !== null && forced !== undefined) return forced;
    return side < 0 ? this.truePos <= -ENDSTOP_DEG : this.truePos >= ENDSTOP_DEG;
  }

  // AS5600 raw counts (12 bit) with a little noise, null when the magnet is gone.
  as5600Raw() {
    if (!knobs.magnet) return null;
    const deg = wrap360(knobs.sensorReversed ? -this.truePos : this.truePos);
    const noise = Math.random() < 0.3 ? (Math.random() < 0.5 ? -1 : 1) : 0;
    return (((Math.round((deg * AS_COUNTS) / 360) + noise) % AS_COUNTS) + AS_COUNTS) % AS_COUNTS;
  }
}

// ---------------------------------------------------------------------------
// Event log (src/event_log.cpp)
// ---------------------------------------------------------------------------

class EventLog {
  constructor(clock) {
    this.clock = clock;
    this.items = [];
    this.pending = [];
    this.rev = 0;
  }

  add(lvl, code, msg) {
    lvl = lvl === 'e' || lvl === 'w' ? lvl : 'i';
    const full = ascii(msg);
    const it = { ms: this.clock(), lvl, code: ascii(code).slice(0, 15), msg: full.slice(0, 47) };
    this.items.push(it);
    if (this.items.length > EVENT_LOG_SIZE) this.items.shift();
    this.pending.push(it);
    if (this.pending.length > EVENT_LOG_SIZE) this.pending.shift();
    this.rev++;
    process.stdout.write(`${stamp(it.ms)} ${lvl.toUpperCase()} ${code}: ${full}\n`);
  }
}

// ---------------------------------------------------------------------------
// Yaw controller (src/yaw.cpp)
// ---------------------------------------------------------------------------

const sideName = (d) => (d < 0 ? 'left' : 'right');

class Yaw {
  constructor(dev) {
    this.dev = dev;
    this.endstops = opt.sensor === 'endstops';
    this.as5600 = opt.sensor === 'as5600';
    this.motor = new Motor(YAW_RAMP_MS);
    // settings
    this.speedDefault = 60;
    this.maxBal = 0;
    this.ms360 = 0;
    this.maxDeg = 0;
    this.sensRev = false;
    this.holdTimeoutMs = YAW_HOLD_TIMEOUT_MS;
    // motion
    this.dir = 0;
    this.speed = 0;
    this.holdArmed = false;
    this.lastHoldMs = 0;
    this.gotoActive = false;
    this.gotoTarget = 0;
    this.gotoStartMs = 0;
    this.calActive = false;
    this.calStartBal = 0;
    this.bal = new YawBalance();
    this.lastIntegrateMs = 0;
    this.endL = new EndstopDebouncer();
    this.endR = new EndstopDebouncer();
    this.side = new EndstopSideCheck();
    // AS5600
    this.asOk = false;
    this.asReported = false;
    this.asOff = 0;
    this.asRaw = 0;
    this.asAbs = 0;
    this.cont = 0;
    this.contValid = false;
    this.zeroPending = false;
    this.unwrap = new AngleUnwrapper();
    this.asNextMs = 0;
    this.wrongWay = 0;
    this.lastCont = 0;
    this.contAtLoss = 0;
    this.balAtLoss = 0;
  }

  get ev() { return this.dev.events; }

  begin(now) {
    this.motor.begin(now);
    this.lastIntegrateMs = now;
    this.lastHoldMs = now;
    const p = store.yaw;
    this.bal.restore(p.bal);
    if (this.endstops) {
      this.endL.reset(phys.endstopRaw(-1), now);
      this.endR.reset(phys.endstopRaw(1), now);
    }
    if (this.as5600) {
      this.asOff = p.asOff & (AS_COUNTS - 1);
      this.cont = p.asCont;
      this.lastCont = this.cont;
      this.unwrap.reset(this.cont);
      this.asNextMs = now;  // first reading in the first tick, after configure()
    }
  }

  persist() {
    const p = store.yaw;
    p.bal = this.bal.balance();
    if (this.as5600) {
      p.asOff = this.asOff;
      if (this.contValid) p.asCont = this.cont;
    }
  }

  motionDir() {
    const d = sign(this.motor.target());
    return d !== 0 ? d : sign(this.motor.actualMilli());
  }

  angleValid() { return this.as5600 && this.asOk && this.contValid; }

  endstopBlocked(d) {
    if (!this.endstops || d === 0) return false;
    if (this.side.blocked(d)) return true;
    return d < 0 ? this.endL.stable : this.endR.stable;
  }

  // AS5600 angle against yawMaxDeg when valid and set, else the time balance.
  twistBlocked(d) {
    if (d === 0) return false;
    if (this.angleValid() && this.maxDeg > 0) {
      return d > 0 ? this.cont >= this.maxDeg : this.cont <= -this.maxDeg;
    }
    return this.bal.blocked(d, this.maxBal);
  }

  endReleaseTravel() {
    if (this.ms360 === 0) return ENDSTOP_RELEASE_MS;
    return Math.max(Math.floor((this.ms360 * ENDSTOP_RELEASE_DEG + 180) / 360), ENDSTOP_RELEASE_MIN_MS);
  }

  integrate(now) {
    const dt = now - this.lastIntegrateMs;
    if (dt <= 0) return;
    this.lastIntegrateMs = now;
    const m = this.motor.actualMilli();
    if (m === 0) return;
    const pct = Math.min(Math.floor((Math.abs(m) + 500) / 1000), 100);
    this.bal.integrate(sign(m), pct, dt);
  }

  halt(brake) {
    if (brake) this.motor.brake();
    else this.motor.stop();
    this.dir = 0;
    this.holdArmed = false;
    this.gotoActive = false;
    this.wrongWay = 0;
  }

  stopNow(brake) {
    this.integrate(this.dev.millis());
    this.halt(brake);
    this.persist();
  }

  setDrive(d, pct) {
    if (d !== this.dir) {
      this.wrongWay = 0;
      this.lastCont = this.cont;
    }
    this.dir = d;
    this.speed = pct;
    this.motor.setSpeed(d * pct);
  }

  applyRev(raw) {
    return this.sensRev ? (AS_COUNTS - raw) & (AS_COUNTS - 1) : raw;
  }

  zeroPosition() {
    this.bal.reset();
    this.calActive = false;
    if (!this.as5600) return;
    if (this.asOk) {
      this.asOff = this.applyRev(this.asRaw);
      this.asAbs = 0;
      this.cont = 0;
      this.lastCont = 0;
      this.contValid = true;
      this.unwrap.reset(0);
      this.zeroPending = false;
    } else {
      this.zeroPending = true;
    }
  }

  setSensorState(ok, why) {
    if (this.asReported && ok === this.asOk) return;
    const first = !this.asReported;
    this.asReported = true;
    if (!ok) {
      if (this.asOk || first) {
        this.contAtLoss = this.cont;
        this.balAtLoss = this.bal.balance();
      }
      this.asOk = false;
      this.ev.add('e', 'as5600_err', why || '?');
      if (this.gotoActive) {
        this.stopNow(true);
        this.ev.add('w', 'goto_abort', 'sensor lost');
      }
    } else {
      this.asOk = true;
      this.ev.add('i', 'as5600_ok', `angle ${rha(this.asAbs)} deg`);
    }
  }

  sampleSensor(now) {
    const raw = [phys.as5600Raw(), phys.as5600Raw(), phys.as5600Raw()];
    if (raw[0] === null || raw[1] === null || raw[2] === null) {
      this.setSensorState(false, 'magnet missing');
      this.asNextMs = now + AS_RETRY_MS;
      return;
    }
    // Median of 3 across the 0/4095 wrap: deltas relative to the first sample.
    const rel = (d) => {
      d = ((d % AS_COUNTS) + AS_COUNTS) % AS_COUNTS;
      return d >= AS_COUNTS / 2 ? d - AS_COUNTS : d;
    };
    const med = raw[0] + median3(0, rel(raw[1] - raw[0]), rel(raw[2] - raw[0]));
    this.asRaw = ((rha(med) % AS_COUNTS) + AS_COUNTS) % AS_COUNTS;

    let seeded = false;
    if (this.zeroPending) {
      this.zeroPending = false;
      this.asOff = this.applyRev(this.asRaw);
      this.unwrap.reset(0);
      this.lastCont = 0;
      seeded = true;
    }
    const counts = (this.applyRev(this.asRaw) - this.asOff) & (AS_COUNTS - 1);
    this.asAbs = wrap360((counts * 360) / AS_COUNTS);
    if (!seeded && !this.asOk && this.asReported) {
      // Back after an outage: seed the turn count from the time balance.
      let hint = this.contAtLoss;
      if (this.ms360 > 0) hint += ((this.bal.balance() - this.balAtLoss) * 360) / this.ms360;
      this.unwrap.reset(hint);
    }
    this.cont = this.unwrap.update(this.asAbs);
    if (!this.contValid) {
      this.contValid = true;
      this.lastCont = this.cont;
    }
    this.setSensorState(true, null);
  }

  // Driving right while the angle falls (or left while it rises) = reversed sensor.
  checkDirection() {
    const d = this.motionDir();
    const dc = this.cont - this.lastCont;
    this.lastCont = this.cont;
    if (d === 0 || this.dir === 0 || d !== this.dir) {
      this.wrongWay = 0;
      return;
    }
    this.wrongWay = Math.max(0, this.wrongWay - dc * d);
    if (this.wrongWay >= WRONG_WAY_DEG) {
      this.stopNow(true);
      this.ev.add('e', 'yaw_sens_dir', 'angle runs backwards, check yawSensRev');
    }
  }

  gotoStep() {
    const st = planGoto(this.cont, this.gotoTarget, this.speedDefault, this.maxDeg, GOTO_PARAMS);
    if (st.impossible) {
      this.stopNow(true);
      this.ev.add('w', 'goto_fail', 'twist limit both ways');
    } else if (st.done || st.dir === 0) {
      this.stopNow(true);
      this.ev.add('i', 'goto_done', `at ${rha(this.asAbs)} deg`);
    } else {
      this.setDrive(st.dir, st.speedPct);
    }
  }

  configure(prev, next) {
    this.speedDefault = clamp(next.yawSpeed, YAW_SPEED_MIN_PCT, YAW_SPEED_MAX_PCT);
    this.maxBal = next.yawMaxBal;
    this.ms360 = next.yawMs360;
    this.maxDeg = next.yawMaxDeg;
    this.holdTimeoutMs = Math.max(next.fsTimeout, YAW_HOLD_TIMEOUT_MS);
    this.motor.configure(YAW_RAMP_MS);
    const invertChanged = prev.yawInvert !== next.yawInvert;
    const revChanged = this.as5600 && prev.yawSensRev !== next.yawSensRev;
    this.sensRev = next.yawSensRev;
    if (invertChanged || revChanged) {
      if (!this.idle()) this.stopNow(true);
      if (invertChanged) this.side.reset();
      this.zeroPosition();
      this.persist();
      this.ev.add('w', 'yaw_zeroed', `${invertChanged ? 'yawInvert' : 'yawSensRev'} changed`);
    }
  }

  move(dir, speedPct, now) {
    if (dir === 0) {
      this.stop(true);
      return null;
    }
    this.integrate(now);
    let e = null;
    if (this.endstopBlocked(dir)) e = 'endstop';
    else if (this.twistBlocked(dir)) e = 'twist_limit';
    if (e) {
      if (!this.idle()) this.stopNow(true);
      return e;
    }
    this.gotoActive = false;
    if (this.endstops && !(this.holdArmed && this.dir === dir)) {
      this.side.moveStarted(dir, this.endL.stable, this.endR.stable, this.bal.balance());
    }
    this.setDrive(dir, clamp(speedPct || this.speedDefault, YAW_SPEED_MIN_PCT, YAW_SPEED_MAX_PCT));
    this.holdArmed = true;
    this.lastHoldMs = now;
    return null;
  }

  hold(dir, now) {
    if (dir !== 0 && this.holdArmed && this.dir === dir) this.lastHoldMs = now;
  }

  gotoAngle(deg, now) {
    if (!this.hasAngleSensor()) return 'no_sensor';
    const target = wrap360(deg);
    const st = planGoto(this.cont, target, this.speedDefault, this.maxDeg, GOTO_PARAMS);
    if (st.impossible) return 'twist_limit';
    this.integrate(now);
    if (st.done || st.dir === 0) {
      if (!this.idle()) this.stopNow(true);
      return null;
    }
    this.holdArmed = false;
    this.gotoActive = true;
    this.gotoTarget = target;
    this.gotoStartMs = now;
    this.setDrive(st.dir, st.speedPct);
    return null;
  }

  stop(brake) {
    this.stopNow(brake);
  }

  zero() {
    if (!this.idle()) return 'busy';
    this.zeroPosition();
    this.persist();
    this.ev.add('i', 'yaw_zero', 'position set to 0');
    return null;
  }

  // { err } or { err: null, ms } for calStop.
  calibrate(action) {
    if (this.as5600) return { err: 'no_sensor' };
    if (action === 'calStart') {
      this.calActive = true;
      this.calStartBal = this.bal.balance();
      return { err: null };
    }
    if (action === 'calCancel') {
      this.calActive = false;
      return { err: null };
    }
    if (!this.calActive) return { err: 'out_of_range' };
    this.calActive = false;
    this.integrate(this.dev.millis());
    const d = Math.abs(this.bal.balance() - this.calStartBal);
    if (d < YAW_CAL_MIN_MS || d > FIELD.yawMs360.max) return { err: 'out_of_range' };
    this.ev.add('i', 'yaw_cal', `${d} ms per turn`);
    return { err: null, ms: d };
  }

  tick(now) {
    this.motor.tick(now);
    this.integrate(now);
    if (this.dir !== 0 && this.motor.target() === 0) {
      this.dir = 0;
      this.holdArmed = false;
      this.gotoActive = false;
    }
    if (this.endstops) {
      this.endL.update(phys.endstopRaw(-1), now, ENDSTOP_DEBOUNCE_MS);
      this.endR.update(phys.endstopRaw(1), now, ENDSTOP_DEBOUNCE_MS);
      const r = this.side.update({
        now, drive: sign(this.motor.actualMilli()), commanded: this.dir,
        lActive: this.endL.stable, rActive: this.endR.stable, lRose: this.endL.rose, rRose: this.endR.rose,
        balance: this.bal.balance(), releaseTravel: this.endReleaseTravel(),
      });
      if (r.refL) this.bal.onEndstop(-1);
      if (r.refR) this.bal.onEndstop(1);
      if (r.wrong !== 0) {
        this.stopNow(true);
        this.ev.add('e', 'yaw_end_dir', `switch ${r.wrong < 0 ? 'R' : 'L'} while going ${sideName(r.wrong)}, both ways blocked`);
      }
    }
    if (this.as5600 && now - this.asNextMs >= 0) {
      this.asNextMs = now + AS5600_PERIOD_MS;
      this.sampleSensor(now);
      if (this.angleValid()) {
        this.checkDirection();
        if (this.gotoActive) this.gotoStep();
      }
    }
    if (this.gotoActive && now - this.gotoStartMs >= YAW_GOTO_TIMEOUT_MS) {
      this.stopNow(true);
      const short = rha(Math.abs(shortestDelta(this.asAbs, this.gotoTarget)));
      this.ev.add('w', 'goto_timeout', `${short} deg short of ${rha(this.gotoTarget)}`);
    }
    if (this.holdArmed && now - this.lastHoldMs > this.holdTimeoutMs) {
      this.stopNow(true);
      this.ev.add('w', 'yaw_deadman', `no hold for ${this.holdTimeoutMs} ms`);
    }
    const md = this.motionDir();
    if (md !== 0) {
      if (this.endstopBlocked(md)) {
        this.stopNow(true);
        this.ev.add('i', 'endstop', sideName(md));
      } else if (this.twistBlocked(md)) {
        this.stopNow(true);
        if (this.angleValid() && this.maxDeg > 0) this.ev.add('w', 'twist_limit', `${sideName(md)} at ${rha(this.cont)} deg`);
        else this.ev.add('w', 'twist_limit', `${sideName(md)}, balance ${this.bal.balance()}`);
      }
    }
    this.persist();
  }

  fill(t) {
    t.yawMoving = this.motionDir();
    t.yawSpeed = this.speed;
    const av = this.angleValid();
    t.hasAngle = av;
    t.angle = av ? rha(this.asAbs) % 360 : 0;
    const twL = this.twistBlocked(-1);
    const twR = this.twistBlocked(1);
    t.limitL = twL || this.endstopBlocked(-1);
    t.limitR = twR || this.endstopBlocked(1);
    t.twist = twL ? -1 : twR ? 1 : 0;
    t.bal = this.bal.balance();
    t.balMax = this.maxBal;
    if (av) {
      t.hasPos = true;
      t.pos = rha(this.cont);
      t.posEst = false;
      t.hasPosMax = this.maxDeg > 0;
      t.posMax = this.maxDeg;
    } else {
      const est = this.bal.estimateDeg(this.ms360);
      t.hasPos = est !== null;
      t.pos = t.hasPos ? rha(est) : 0;
      t.posEst = true;
      t.hasPosMax = this.maxBal > 0 && this.ms360 > 0;
      t.posMax = t.hasPosMax ? Math.floor((this.maxBal * 360 + Math.floor(this.ms360 / 2)) / this.ms360) : 0;
    }
    t.gotoActive = this.gotoActive;
    t.gotoTarget = this.gotoActive ? rha(this.gotoTarget) % 360 : 0;
    t.sensor = this.sensorName();
    if (this.as5600) t.sensorOk = this.asOk;
    else if (this.endstops) t.sensorOk = !(this.endL.stable && this.endR.stable) && this.side.refused === 0;
    else t.sensorOk = true;
    t.calActive = this.calActive;
  }

  idle() { return this.motor.idle() && !this.gotoActive; }
  hasAngleSensor() { return this.angleValid(); }
  sensorName() { return opt.sensor; }
}

// ---------------------------------------------------------------------------
// Failsafe (lib/turbine_core/src/failsafe_logic.cpp + src/failsafe.cpp)
// ---------------------------------------------------------------------------

const FSR = { runtime: 1, timeout: 2, disconnect: 3, battery: 4, ota: 5 };
const FSR_NAMES = [null, 'runtime', 'timeout', 'disconnect', 'battery', 'ota'];
const FSR_ORDER = ['ota', 'battery', 'disconnect', 'timeout', 'runtime'];
const fsBit = (r) => 1 << FSR[r];
const sameClient = (a, b) => !!a && !!b && a.src === b.src && a.id === b.id;

function formatClient(c) {
  if (!c) return 'none';
  if (c.src === 'http') return `http@${c.id}`;
  if (c.src === 'ws' || c.src === 'ble') return `${c.src}#${c.id}`;
  return c.src;
}

class Failsafe {
  constructor(dev) {
    this.dev = dev;
    this.timeoutMs = 500;
    this.mask = 0;
    this.owner = null;
    this.ownerSeen = 0;
    this.ownerLost = false;
    this.logicLastClients = 0;
    this.maxRunS = 0;
    this.running = false;
    this.runStart = 0;
    this.lastCheck = 0;
    this.checkNow = true;
    this.lastMask = 0;
    this.lastClients = 0;
  }

  configure(s) {
    this.timeoutMs = s.fsTimeout;
    this.maxRunS = s.rotorMaxRun;
  }

  has(r) { return (this.mask & fsBit(r)) !== 0; }
  set(r) { this.mask |= fsBit(r); }
  clear(r) { this.mask &= ~fsBit(r); }
  active() { return this.mask !== 0; }

  reason(mask = this.mask) {
    for (let i = 5; i > 0; i--) if (mask & (1 << i)) return FSR_NAMES[i];
    return null;
  }

  releaseOwner() {
    this.owner = null;
    this.ownerLost = false;
  }

  onFrame(c, now) {
    const owner = this.owner;
    if (owner && sameClient(c, owner)) this.ownerSeen = now;
    this.clear('timeout');
    this.clear('disconnect');
    this.logEdges(now, owner);
  }

  onMotionCommand(c, now) {
    this.owner = c;
    this.ownerSeen = now;
    this.ownerLost = false;
    this.logEdges(now, c);
  }

  onClientDisconnected(c) {
    if (this.owner && sameClient(c, this.owner)) this.ownerLost = true;
    this.checkNow = true;
  }

  // FailsafeLogic::update
  update(inp) {
    let zero = false;
    if (inp.batteryCrit) this.set('battery');
    else this.clear('battery');
    if (inp.otaActive) this.set('ota');
    else this.clear('ota');
    if (inp.runLimitHit) {
      this.set('runtime');
      zero = true;
    }
    if (this.logicLastClients > 0 && inp.clients === 0) {
      this.set('disconnect');
      zero = true;
    }
    this.logicLastClients = inp.clients;
    if (this.ownerLost) {
      this.ownerLost = false;
      this.set('disconnect');
      zero = true;
    }
    if (this.owner && inp.now - this.ownerSeen > this.timeoutMs) {
      this.set('timeout');
      zero = true;
    }
    const hard = this.has('battery') || this.has('ota');
    if (zero || hard) this.releaseOwner();
    return { zero, hard };
  }

  runSeconds(now) {
    return this.running ? Math.floor((now - this.runStart) / 1000) : 0;
  }

  rotorRunS() { return this.runSeconds(this.dev.millis()); }

  logTrip(r, now, owner) {
    const who = formatClient(owner);
    const ev = this.dev.events;
    if (r === 'timeout') ev.add('w', 'fs_timeout', `owner ${who} silent ${now - this.ownerSeen} ms`);
    else if (r === 'disconnect') {
      if (owner) ev.add('w', 'fs_disconnect', `owner ${who}, clients ${this.lastClients}`);
      else ev.add('w', 'fs_disconnect', 'last client left');
    } else if (r === 'battery') ev.add('w', 'fs_battery', `${this.dev.battery.mv} mV, motors stopped`);
    else if (r === 'ota') ev.add('w', 'fs_ota', 'update running, motors locked');
    else if (r === 'runtime') ev.add('w', 'fs_runtime', `rotor ran ${this.runSeconds(now)} s, owner ${who}`);
  }

  logEdges(now, owner) {
    const added = this.mask & ~this.lastMask;
    for (const r of FSR_ORDER) if (added & fsBit(r)) this.logTrip(r, now, owner);
    if (this.mask === 0 && this.lastMask !== 0) this.dev.events.add('i', 'fs_clear', `was ${this.reason(this.lastMask)}`);
    this.lastMask = this.mask;
  }

  tick(now) {
    const dev = this.dev;
    dev.motorsIdle = dev.allIdle();
    if (!this.checkNow && now - this.lastCheck < FS_CHECK_MS) return;
    this.checkNow = false;
    this.lastCheck = now;

    if (dev.rotor.actual() > 0) {
      if (!this.running) {
        this.running = true;
        this.runStart = now;
      }
    } else {
      this.running = false;
    }

    this.lastClients = net.wsCount();
    const inp = {
      now,
      clients: this.lastClients,
      batteryCrit: dev.battery.crit(),
      otaActive: dev.otaActive,
      runLimitHit: this.maxRunS > 0 && this.runSeconds(now) >= this.maxRunS,
    };
    const ownerBefore = this.owner;
    const r = this.update(inp);
    const hardEdge = (this.mask & (fsBit('battery') | fsBit('ota')) & ~this.lastMask) !== 0;
    if (r.zero) {
      if (!dev.rotor.idle()) dev.rotor.setSpeedWithRamp(0, FS_RAMP_MS);
      if (!dev.yaw.idle()) dev.yaw.stop(true);
    }
    if (hardEdge) {
      dev.rotor.stop();
      dev.yaw.stop(true);
    } else if (r.hard || inp.otaActive) {
      if (!dev.rotor.idle()) dev.rotor.stop();
      if (!dev.yaw.idle()) dev.yaw.stop(true);
    }
    this.logEdges(now, ownerBefore);
    if (dev.allIdle()) this.releaseOwner();
    dev.motorsIdle = dev.allIdle();
  }

  allowMotion() {
    if (this.dev.otaActive || this.has('ota')) return 'busy_ota';
    if (this.has('battery') || this.dev.battery.crit()) return 'low_battery';
    return null;
  }

  clearRuntime() {
    if (this.has('runtime')) this.running = false;  // restart the run timer
    this.clear('runtime');
    this.logEdges(this.dev.millis(), this.owner);
  }
}

// ---------------------------------------------------------------------------
// The simulated device (src/main.cpp + src/commands.cpp + src/settings.cpp)
// ---------------------------------------------------------------------------

// Survives simulated restarts, like NVS.
const store = {
  settings: { ...settingsDefaults(), yawMs360: MOCK_YAW_MS360 },
  yaw: { bal: 0, asOff: 0, asCont: 0 },
};

const BUILD = (() => {
  const d = new Date();
  const mon = d.toLocaleString('en-US', { month: 'short' });
  const p2 = (n) => String(n).padStart(2, '0');
  return `${mon} ${String(d.getDate()).padStart(2)} ${d.getFullYear()} ${p2(d.getHours())}:${p2(d.getMinutes())}:${p2(d.getSeconds())}`;
})();

const OTA_ALLOWED = new Set(['stop', 'ping', 'info', 'log']);
const SILENT_SINK = { reply() {}, wantsAck: () => false, wantsReplies: false };

class Device {
  constructor(resetReason) {
    this.bootAt = performance.now();
    this.resetReason = resetReason;
    this.events = new EventLog(() => this.millis());
    this.events.add(['poweron', 'sw'].includes(resetReason) ? 'i' : 'w', 'boot', `fw ${FW_VERSION}, reset ${resetReason}`);
    this.bootSettings = { ...store.settings };
    this.rev = 1;
    this.name = store.settings.name || `${AP_SSID_PREFIX}${MAC.slice(-5).replace(':', '')}`;
    this.otaActive = false;
    this.motorsIdle = true;
    this.ota = { phase: 'idle' };
    this.heapMin = 999999;
    this.rebootAt = null;

    const now = this.millis();
    this.rotor = new Motor(500);
    this.rotor.begin(now);
    this.yaw = new Yaw(this);
    this.battery = new BatteryMeter(this);
    this.battery.begin(now);
    this.yaw.begin(now);
    this.fs = new Failsafe(this);
    this.applySettings(this.settings, this.settings);

    // telemetry scheduler
    this.telFirst = true;
    this.telCheckMs = 0;
    this.wsSentMs = 0;
    this.wsSig = '';
    this.stateCache = '';
    this.stateCacheMs = 0;
    this.lastSilentMs = 0;
    this.lastHouseMs = 0;
    this.stations = 0;
    this.rssi = -55;
  }

  get settings() { return store.settings; }

  millis() { return Math.floor(performance.now() - this.bootAt); }

  allIdle() { return this.rotor.idle() && this.yaw.idle(); }

  rebootPending() {
    return FIELDS.some((f) => (f.flags & F_REBOOT) && fieldDiffers(this.settings, this.bootSettings, f));
  }

  applySettings(prev, next) {
    this.rotor.configure(next.rotorRamp);
    // rotorMax caps the running speed too (main.cpp appApplySettings).
    if (this.rotor.target() > next.rotorMax) this.rotor.setSpeed(next.rotorMax);
    this.yaw.configure(prev, next);
    this.battery.configure(next);
    this.fs.configure(next);
  }

  // --- frames ---------------------------------------------------------------

  snapshot() {
    const t = {
      rotorTarget: Math.max(0, this.rotor.target()),
      rotorActual: Math.max(0, this.rotor.actual()),
      rotorRunS: this.fs.rotorRunS(),
    };
    this.yaw.fill(t);
    t.batMv = this.battery.mv;
    t.batPct = this.battery.pct;
    t.batLow = this.battery.low();
    t.batCrit = this.battery.crit();
    t.batPresent = this.battery.present();
    t.ws = net.wsCount();
    t.ble = 0;
    t.sta = this.stations;
    t.hasRssi = t.sta > 0;
    t.rssi = this.rssi;
    t.fsActive = this.fs.active();
    t.fsReason = this.fs.reason();
    t.ota = this.otaActive;
    t.cfgRev = this.rev;
    t.upMs = this.millis();
    return t;
  }

  buildStateFrame() { return buildState(this.snapshot(), false); }

  buildCfgFrame() { return buildCfg(this.settings, this.rev, this.rebootPending()); }

  buildInfoFrame() {
    const heap = 151000 - net.wsCount() * 2600 - Math.floor(Math.random() * 600);
    this.heapMin = Math.min(this.heapMin, heap - 9000);
    return J({
      t: 'info', v: PROTOCOL_VERSION, fw: FW_VERSION, build: BUILD, name: this.name, chip: BOARD_NAME,
      mac: MAC, sensor: this.yaw.sensorName(), wifi: true, ble: true, ota: true, heap,
      heapMin: this.heapMin, reset: this.resetReason, up: this.millis(),
    });
  }

  // Oldest items dropped until the frame fits the firmware buffer.
  buildLogFrame() {
    const items = this.events.items;
    for (let first = 0; ; first++) {
      const s = `{"t":"log","items":[${items.slice(first).map((it) => `{${logItemJson(it)}}`).join(',')}]}`;
      if (s.length < LOG_FRAME_CAP || first >= items.length) return s;
    }
  }

  sendErr(sink, c, code, field) {
    if (sink.wantsReplies) sink.reply(buildErr(c.type, code, field, null, c.hasId, c.id));
  }

  sendAck(sink, c) {
    if (sink.wantsReplies && sink.wantsAck(c.hasId)) sink.reply(buildAck(c.type, c.hasId, c.id));
  }

  broadcastCfg() { net.broadcast(this.buildCfgFrame()); }

  // --- commands -------------------------------------------------------------

  // A frame from a transport. A stop jumps the queue (estop flag) before
  // anything else, exactly like CmdQueue + processQueue().
  receive(src, client, text, sink) {
    if (peekType(text) === 'stop') this.emergencyStop('stop request');
    this.handle(src, client, text, sink);
    // The firmware publishes this in the same loop pass (Failsafe::tick right
    // after the queue), so an upload racing a start command sees it.
    this.motorsIdle = this.allIdle();
  }

  handle(src, client, text, sink) {
    const now = this.millis();
    const pr = parseCommand(text, this.settings);
    const c = pr.cmd;
    if (pr.err) return this.sendErr(sink, c, pr.err, pr.field);
    const who = { src, id: client };
    this.fs.onFrame(who, now);
    if (this.otaActive && !OTA_ALLOWED.has(c.type)) return this.sendErr(sink, c, 'busy_ota');
    switch (c.type) {
      case 'ping':
        if (sink.wantsReplies) sink.reply(buildPong(c.hasId, c.id, now));
        return undefined;
      case 'stop':
        if (!this.allIdle()) this.emergencyStop(formatClient(who));
        return this.sendAck(sink, c);
      case 'rotor': return this.doRotor(who, c, sink, now);
      case 'yaw': return this.doYaw(who, c, sink, now);
      case 'cfg': return this.doCfg(c, sink);
      case 'info': if (sink.wantsReplies) sink.reply(this.buildInfoFrame()); return undefined;
      case 'log': if (sink.wantsReplies) sink.reply(this.buildLogFrame()); return undefined;
      case 'reboot':
        this.sendAck(sink, c);
        this.requestReboot(300, 'reboot command');
        return undefined;
      default: return this.sendErr(sink, c, 'unknown_cmd');
    }
  }

  // A WS client whose frames are still queued after it left must not start
  // anything (commands.cpp clientGone). The mock runs frames synchronously,
  // so this only mirrors the rule.
  gone(who) { return who.src === 'ws' && !net.clients.has(who.id); }

  doRotor(who, c, sink, now) {
    if (c.speed > 0) {
      if (this.gone(who)) return undefined;
      const e = this.fs.allowMotion();
      if (e) return this.sendErr(sink, c, e);
      this.fs.clearRuntime();
      this.rotor.setSpeed(Math.min(c.speed, this.settings.rotorMax));
      this.fs.onMotionCommand(who, now);
    } else if (c.brake) {
      this.rotor.brake();
    } else {
      this.rotor.setSpeed(0);
    }
    return this.sendAck(sink, c);
  }

  doYaw(who, c, sink, now) {
    let e = null;
    switch (c.action) {
      case 'move':
        if (c.dir === 0) {
          this.yaw.stop(true);
          break;
        }
        if (this.gone(who)) return undefined;
        e = this.fs.allowMotion() ?? this.yaw.move(c.dir, c.yawSpeed, now);
        if (!e) this.fs.onMotionCommand(who, now);
        break;
      case 'hold':
        this.yaw.hold(c.dir, now);
        return undefined;  // no ack: holds repeat every 200 ms
      case 'goto':
        if (this.gone(who)) return undefined;
        e = this.fs.allowMotion() ?? this.yaw.gotoAngle(c.gotoDeg, now);
        if (!e) this.fs.onMotionCommand(who, now);
        break;
      case 'zero':
        e = this.yaw.zero();
        break;
      case 'calStart':
      case 'calCancel':
        e = this.yaw.calibrate(c.action).err;
        break;
      case 'calStop': {
        const r = this.yaw.calibrate('calStop');
        e = r.err;
        if (e) break;
        const prev = { ...this.settings };
        const next = { ...this.settings };
        if (!setNumber(next, FIELD.yawMs360, r.ms)) {
          e = 'out_of_range';
          break;
        }
        this.commitNext(prev, next);
        this.sendAck(sink, c);
        this.broadcastCfg();
        return undefined;
      }
      default:
        e = 'bad_type';
    }
    if (e) return this.sendErr(sink, c, e);
    return this.sendAck(sink, c);
  }

  commitNext(prev, next) {
    if (!FIELDS.some((f) => fieldDiffers(prev, next, f))) return;
    store.settings = next;
    this.rev++;
    this.applySettings(prev, next);
  }

  doCfg(c, sink) {
    if (c.cfgGet) {
      if (sink.wantsReplies) sink.reply(this.buildCfgFrame());
      return;
    }
    const prev = { ...this.settings };
    const next = c.cfgReset ? settingsDefaults() : c.next;
    const busy = FIELDS.find((f) => (f.flags & F_IDLE) && fieldDiffers(prev, next, f));
    if (busy && !this.allIdle()) {
      this.sendErr(sink, c, 'busy', busy.key);
      return;
    }
    if (c.cfgReset) {
      store.settings = settingsDefaults();
      this.rev++;
      this.events.add('i', 'cfg_reset', 'factory defaults');
      this.applySettings(prev, this.settings);
    } else {
      this.commitNext(prev, next);
    }
    this.sendAck(sink, c);
    this.broadcastCfg();
  }

  // Stops every time; one log entry per press although the panel sends the
  // same STOP over WS and HTTP (failsafe.cpp emergencyStop).
  emergencyStop(why) {
    this.rotor.stop();
    this.yaw.stop(true);
    this.fs.releaseOwner();
    this.motorsIdle = this.allIdle();
    const now = this.millis();
    if (this.estopLogAt !== undefined && now - this.estopLogAt < ESTOP_LOG_GAP_MS) return;
    this.estopLogAt = now;
    this.events.add('i', 'estop', why || 'stop');
  }

  requestReboot(delayMs, why) {
    this.emergencyStop('reboot');
    this.events.add('i', 'reboot', `${why || 'requested'}, in ${delayMs} ms`);
    this.rebootAt = this.millis() + delayMs;
  }

  // --- transports -------------------------------------------------------------

  onConnected(client) {
    client.send(this.buildInfoFrame());
    client.send(this.buildCfgFrame());
    client.send(this.buildStateFrame());
  }

  onDisconnected(src, id) {
    this.fs.onClientDisconnected({ src, id });
  }

  // --- loop -------------------------------------------------------------------

  loop() {
    const now = this.millis();
    phys.advance(this.yaw.motor.actualMilli(), this.rotor.actualMilli(), this.settings);
    this.fs.tick(now);
    this.rotor.tick(now);
    this.yaw.tick(now);
    this.battery.tick(now);
    this.wifiTick(now);
    this.otaTick(now);
    this.telemetry(now);
    this.broadcastEvents();
    if (this.rebootAt !== null && now >= this.rebootAt) {
      this.rebootAt = null;
      rebootDevice('sw');
    }
  }

  wifiTick(now) {
    if (now - this.lastSilentMs >= SILENT_CHECK_MS) {
      this.lastSilentMs = now;
      net.closeSilent();
    }
    if (this.telFirst || now - this.lastHouseMs >= HOUSEKEEP_MS) {
      this.lastHouseMs = now;
      net.cleanup();
      this.stations = net.stations();
      this.rssi = clamp(this.rssi + Math.round(Math.random() * 4 - 2), -68, -44);
    }
  }

  telemetry(now) {
    if (!this.telFirst && now - this.telCheckMs < TEL_CHECK_MS) return;
    this.telCheckMs = now;
    const ws = net.wsCount();
    const sinceWs = now - this.wsSentMs;
    const cacheDue = this.telFirst || now - this.stateCacheMs >= TELEMETRY_PERIOD_MS;
    const wsMaySend = ws > 0 && sinceWs >= TELEMETRY_MIN_GAP_MS;
    if (wsMaySend || cacheDue) {
      const snap = this.snapshot();
      const sig = stateSignature(snap);
      const frame = buildState(snap, false);
      const send = wsMaySend && (sinceWs >= TELEMETRY_PERIOD_MS || sig !== this.wsSig);
      if (send) {
        net.broadcast(frame);
        this.wsSentMs = now;
        this.wsSig = sig;
      }
      if (send || cacheDue) {
        this.stateCache = frame;
        this.stateCacheMs = now;
      }
    }
    this.telFirst = false;
  }

  broadcastEvents() {
    const items = this.events.pending;
    if (items.length === 0) return;
    this.events.pending = [];
    if (net.wsCount() === 0) return;
    for (const it of items) net.broadcast(`{"t":"evt",${logItemJson(it)}}`);
  }

  otaTick(now) {
    const o = this.ota;
    if (o.phase === 'idle') return;
    const kind = o.isFs ? 'fs' : 'fw';
    if (!o.loggedStart) {
      o.loggedStart = true;
      this.events.add('w', 'ota_start', `${kind} upload started, motors locked`);
    }
    if (o.phase === 'running' && !o.loggedEnd && Date.now() - o.lastDataAt > OTA_STALL_MS) {
      o.loggedEnd = true;
      o.endedAt = now;
      this.events.add('e', 'ota_fail', `${kind} upload stalled ${Date.now() - o.lastDataAt} ms`);
    }
    if (o.phase === 'ended' && !o.loggedEnd) {
      o.loggedEnd = true;
      o.endedAt = now;
      if (o.ok) this.events.add('i', 'ota_ok', `${kind} update ok, ${o.written} B`);
      else this.events.add('e', 'ota_fail', `${kind}: ${o.result}`);
    }
    if (o.loggedEnd && !o.rebootAsked && now - o.endedAt >= OTA_REBOOT_AFTER_MS) {
      o.rebootAsked = true;
      this.requestReboot(100, 'ota');
    }
  }

  // Basic Auth password for /update: the password the AP started with
  // (a new apPass works after a restart, like the network itself).
  otaPassword() {
    const p = this.bootSettings.apPass;
    return p && p.length >= 8 && p.length <= 63 ? p : AP_PASSWORD;
  }
}

// ---------------------------------------------------------------------------
// WebSocket (RFC 6455, by hand: text, ping / pong, close; no fragmentation)
// ---------------------------------------------------------------------------

const WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11';
const WS_MAX_FRAME = 64 * 1024;

function wsFrame(opcode, payload) {
  const len = payload.length;
  let hdr;
  if (len < 126) {
    hdr = Buffer.from([0x80 | opcode, len]);
  } else if (len < 65536) {
    hdr = Buffer.alloc(4);
    hdr[0] = 0x80 | opcode;
    hdr[1] = 126;
    hdr.writeUInt16BE(len, 2);
  } else {
    hdr = Buffer.alloc(10);
    hdr[0] = 0x80 | opcode;
    hdr[1] = 127;
    hdr.writeUInt32BE(0, 2);
    hdr.writeUInt32BE(len, 6);
  }
  return Buffer.concat([hdr, payload]);
}

function closePayload(code, reason) {
  const r = Buffer.from(reason || '', 'utf8').subarray(0, 123);
  const b = Buffer.alloc(2 + r.length);
  b.writeUInt16BE(code, 0);
  r.copy(b, 2);
  return b;
}

class WsClient {
  constructor(socket, id, ip) {
    this.socket = socket;
    this.id = id;
    this.ip = ip;
    this.buf = Buffer.alloc(0);
    this.closing = false;
    this.closed = false;
    this.announced = false;
    this.lastRx = Date.now();
    this.nextAt = 0;
    this.sink = { reply: (t) => this.send(t), wantsAck: () => true, wantsReplies: true };
  }

  // Outgoing bytes, delayed by --lag with the order kept.
  write(buf) {
    if (this.socket.destroyed) return;
    if (knobs.lag > 0) {
      const at = Math.max(Date.now() + knobs.lag, this.nextAt);
      this.nextAt = at;
      setTimeout(() => {
        if (!this.socket.destroyed) this.socket.write(buf);
      }, at - Date.now());
    } else {
      this.socket.write(buf);
    }
  }

  send(text) {
    if (this.closing || this.closed) return false;
    this.write(wsFrame(0x1, Buffer.from(text, 'utf8')));
    return true;
  }

  // Like a full AsyncWebSocket queue: drop telemetry for a stuck client.
  canSend() {
    return !this.closing && !this.closed && this.socket.writableLength < 32 * 1024;
  }

  close(code, reason) {
    if (this.closing || this.closed) return;
    this.closing = true;
    this.write(wsFrame(0x8, closePayload(code, reason)));
    setTimeout(() => this.socket.destroy(), knobs.lag + 1000).unref();
  }

  onData(chunk) {
    this.buf = this.buf.length ? Buffer.concat([this.buf, chunk]) : chunk;
    for (;;) {
      const b = this.buf;
      if (b.length < 2) return;
      const fin = (b[0] & 0x80) !== 0;
      const opcode = b[0] & 0x0f;
      const masked = (b[1] & 0x80) !== 0;
      let len = b[1] & 0x7f;
      let off = 2;
      if (len === 126) {
        if (b.length < 4) return;
        len = b.readUInt16BE(2);
        off = 4;
      } else if (len === 127) {
        if (b.length < 10) return;
        if (b.readUInt32BE(2) !== 0) return this.close(1009, 'too big');
        len = b.readUInt32BE(6);
        off = 10;
      }
      if (!masked) return this.close(1002, 'unmasked frame');
      if (len > WS_MAX_FRAME) return this.close(1009, 'too big');
      if (b.length < off + 4 + len) return;
      const mask = b.subarray(off, off + 4);
      const payload = Buffer.allocUnsafe(len);
      for (let i = 0; i < len; i++) payload[i] = b[off + 4 + i] ^ mask[i & 3];
      this.buf = b.subarray(off + 4 + len);
      this.onFrame(fin, opcode, payload);
      if (this.closed) return;
    }
  }

  onFrame(fin, opcode, payload) {
    switch (opcode) {
      case 0x0:  // continuation: commands are single frames, ignored like on the ESP
      case 0x2:  // binary: ignored
        this.lastRx = Date.now();
        return;
      case 0x1:
        this.lastRx = Date.now();
        if (!fin) return;  // a fragmented message is ignored as a whole
        net.onText(this, payload);
        return;
      case 0x8: {
        const code = payload.length >= 2 ? payload.readUInt16BE(0) : 1000;
        if (!this.closing) {
          this.closing = true;
          this.write(wsFrame(0x8, closePayload(code === 1005 ? 1000 : code, '')));
        }
        setTimeout(() => this.socket.end(), knobs.lag + 10).unref();
        return;
      }
      case 0x9:
        this.write(wsFrame(0xa, payload.subarray(0, 125)));
        return;
      case 0xa:
        return;
      default:
        this.close(1002, 'bad opcode');
    }
  }
}

// ---------------------------------------------------------------------------
// Network: WebSocket clients, stations, broadcast, recorded frames
// ---------------------------------------------------------------------------

const net = {
  clients: new Map(),  // id -> WsClient, announced to the device
  all: new Set(),      // every open WsClient (also refused ones)
  nextId: 1,
  seen: new Map(),     // remote ip -> last request (Date.now())
  frames: [],

  wsCount() { return this.clients.size; },

  touch(ip) { this.seen.set(ip, Date.now()); },

  // "Associated stations": distinct client addresses seen lately.
  stations() {
    const now = Date.now();
    const ips = new Set();
    for (const c of this.clients.values()) ips.add(c.ip);
    for (const [ip, at] of this.seen) {
      if (now - at < 5000) ips.add(ip);
      else this.seen.delete(ip);
    }
    return ips.size;
  },

  broadcast(text) {
    for (const c of this.clients.values()) if (c.canSend()) c.send(text);
  },

  record(src, client, raw) {
    let frame = null;
    try {
      frame = JSON.parse(raw);
    } catch {
      // keep raw only
    }
    this.frames.push({ ts: Date.now(), up: dev ? dev.millis() : null, src, client, raw, frame });
    if (this.frames.length > FRAMES_KEPT) this.frames.splice(0, this.frames.length - FRAMES_KEPT);
    if (knobs.verbose) process.stdout.write(`${stamp(dev ? dev.millis() : 0)} <- ${client} ${raw}\n`);
  },

  onText(client, payload) {
    const text = payload.toString('utf8');
    this.record('ws', `ws#${client.id}`, text);
    if (!dev || !client.announced) return;
    if (payload.length > MAX_CMD_LEN) {
      client.send(buildErr(peekType(text), 'too_long', null, null, false, 0));
      return;
    }
    dev.receive('ws', client.id, text, client.sink);
  },

  accept(socket, ip) {
    const c = new WsClient(socket, this.nextId++, ip);
    this.all.add(c);
    socket.on('data', (d) => {
      try {
        c.onData(d);
      } catch (e) {
        process.stderr.write(`mock_esp: ws error: ${e.stack || e}\n`);
        socket.destroy();
      }
    });
    socket.on('close', () => this.gone(c));
    socket.on('error', () => {});
    if (this.clients.size >= WS_SLOTS) {
      c.close(1013, 'too many clients');
      return c;
    }
    c.announced = true;
    this.clients.set(c.id, c);
    if (dev) dev.onConnected(c);
    return c;
  },

  gone(c) {
    if (c.closed) return;
    c.closed = true;
    this.all.delete(c);
    if (this.clients.delete(c.id) && dev) dev.onDisconnected('ws', c.id);
  },

  // Close clients that sent nothing for WS_SILENT_CLOSE_MS.
  closeSilent() {
    const now = Date.now();
    for (const c of this.clients.values()) {
      if (!c.closing && now - c.lastRx > WS_SILENT_CLOSE_MS) c.close(1001, 'silent');
    }
  },

  // cleanupClients(WS_KEEP): the oldest client goes when there are too many.
  cleanup() {
    if (this.clients.size > WS_KEEP) this.clients.values().next().value.close(1000, '');
  },

  // WiFi gone: sockets die without a close frame.
  dropAll() {
    const n = this.all.size;
    for (const c of [...this.all]) c.socket.destroy();
    return n;
  },
};

// ---------------------------------------------------------------------------
// Device lifecycle
// ---------------------------------------------------------------------------

const phys = new Physics();
let dev = null;
let offlineTimer = null;

function boot(reason) {
  dev = new Device(reason);
}

// Restart: every connection dies, the device is off the air for a while,
// then boots with the settings and yaw position it had (NVS).
function rebootDevice(reason) {
  if (!dev) return;
  process.stdout.write(`${stamp(dev.millis())} I restarting\n`);
  dev = null;
  net.dropAll();
  clearTimeout(offlineTimer);
  offlineTimer = setTimeout(() => {
    offlineTimer = null;
    boot(reason);
  }, REBOOT_OFFLINE_MS);
}

// ---------------------------------------------------------------------------
// HTTP
// ---------------------------------------------------------------------------

// AsyncWebServer content types (WebResponses.cpp), nothing more.
const MIME = {
  '.html': 'text/html', '.htm': 'text/html', '.css': 'text/css', '.js': 'text/javascript',
  '.mjs': 'text/javascript', '.json': 'application/json', '.png': 'image/png', '.ico': 'image/x-icon',
  '.svg': 'image/svg+xml', '.jpg': 'image/jpeg', '.webp': 'image/webp', '.avif': 'image/avif',
  '.gif': 'image/gif', '.woff2': 'font/woff2', '.woff': 'font/woff', '.ttf': 'font/ttf',
  '.xml': 'text/xml', '.pdf': 'application/pdf', '.txt': 'text/plain',
};
const LONG_CACHE = new Set(['.png', '.svg', '.ico', '.webp']);

function remoteIp(req) {
  return (req.socket.remoteAddress || '?').replace(/^::ffff:/, '');
}

function readBody(req, keep) {
  return new Promise((resolve) => {
    const parts = [];
    let have = 0;
    let total = 0;
    req.on('data', (d) => {
      total += d.length;
      if (have < keep) {
        const part = d.subarray(0, keep - have);
        parts.push(part);
        have += part.length;
      }
    });
    req.on('end', () => resolve({ buf: Buffer.concat(parts), total }));
    req.on('error', () => resolve({ buf: Buffer.concat(parts), total }));
  });
}

function sendRaw(res, code, type, body, extra = {}) {
  if (res.writableEnded) return;
  res.writeHead(code, { 'Content-Type': type, 'Content-Length': Buffer.byteLength(body), ...extra });
  res.end(body);
}

function sendJson(res, code, body, lag = true) {
  const go = () => sendRaw(res, code, 'application/json', body, { 'Cache-Control': 'no-store' });
  if (lag && knobs.lag > 0) setTimeout(go, knobs.lag);
  else go();
}

// lib/turbine_core/src/stop_seq_guard.cpp: an HTTP motion frame whose "seq"
// is older than the client's last stop of the same axis is refused (every
// POST is its own TCP connection and can overtake an earlier one).
const seqGuard = {
  slots: [],  // { client, touch, rotor: { seq, at } | null, yaw: ... }
  kinds(c) {
    if (c.type === 'stop') return { rotor: 'stop', yaw: 'stop' };
    if (c.type === 'rotor') return { rotor: c.speed > 0 ? 'motion' : 'stop', yaw: null };
    if (c.type === 'yaw' && c.action === 'move') return { rotor: null, yaw: c.dir === 0 ? 'stop' : 'motion' };
    if (c.type === 'yaw' && c.action === 'goto') return { rotor: null, yaw: 'motion' };
    return { rotor: null, yaw: null };
  },
  older: (a, b) => ((a - b) | 0) < 0,
  accept(client, c, now) {
    if (!c.hasSeq) return true;
    const k = this.kinds(c);
    if (!k.rotor && !k.yaw) return true;
    let sl = this.slots.find((x) => x.client === client);
    if (k.rotor === 'stop' || k.yaw === 'stop') {
      if (!sl) {
        if (this.slots.length >= SEQ_SLOTS) this.slots.splice(this.slots.indexOf(this.slots.reduce((a, b) => (b.touch < a.touch ? b : a))), 1);
        sl = { client, touch: now, rotor: null, yaw: null };
        this.slots.push(sl);
      }
      sl.touch = now;
      for (const ax of ['rotor', 'yaw']) {
        if (k[ax] !== 'stop') continue;
        const a = sl[ax];
        const keep = a && now - a.at < SEQ_WINDOW_MS && !this.older(a.seq, c.seq);
        sl[ax] = { seq: keep ? a.seq : c.seq, at: now };
      }
      return true;
    }
    const a = sl && sl[k.rotor === 'motion' ? 'rotor' : 'yaw'];
    if (!a || now - a.at >= SEQ_WINDOW_MS) return true;
    return !this.older(c.seq, a.seq);
  },
};

async function handleApiCmd(req, res) {
  const ip = remoteIp(req);
  const { buf, total } = await readBody(req, MAX_CMD_LEN + 1);
  const d = dev;
  if (!d) return req.socket.destroy();
  let body = buf.toString('utf8');
  if (/application\/x-www-form-urlencoded/i.test(req.headers['content-type'] || '')) {
    const p = new URLSearchParams(body);
    if (p.has('body')) body = p.get('body');
  }
  net.record('http', `http@${ip}`, body);
  const declared = Number(req.headers['content-length'] || 0);
  if (declared > MAX_CMD_LEN || total > MAX_CMD_LEN || Buffer.byteLength(body) > MAX_CMD_LEN) {
    const head = buf.subarray(0, MAX_CMD_LEN).toString('utf8');
    return sendJson(res, 200, buildErr(peekType(head), 'too_long', null, null, false, 0));
  }
  // Pre-validation in the handler (no settings base: types and ranges only).
  const pr = parseCommand(body, null);
  const c = pr.cmd;
  if (pr.err) return sendJson(res, 200, buildErr(c.type, pr.err, pr.field, null, c.hasId, c.id));
  if (!seqGuard.accept(ip, c, d.millis())) {
    if (knobs.verbose) process.stdout.write(`${stamp(d.millis())} W http: stale ${c.type} (seq ${c.seq}) dropped\n`);
    return sendJson(res, 200, buildErr(c.type, 'stale', null, null, c.hasId, c.id));
  }
  let reply;
  if (c.type === 'ping') reply = buildPong(c.hasId, c.id, d.millis());
  else if (c.type === 'cfg' && c.cfgGet && !c.cfgReset) reply = d.buildCfgFrame();
  else if (c.type === 'info') reply = d.buildInfoFrame();
  else if (c.type === 'log') reply = d.buildLogFrame();
  else reply = buildAck(c.type, c.hasId, c.id);
  // The loop runs it; execution errors show up only in the telemetry.
  d.receive('http', ip, body, SILENT_SINK);
  return sendJson(res, 200, reply);
}

function handleApiState(req, res) {
  if (!dev.stateCache) {
    return sendJson(res, 503, buildErr(null, 'busy', null, 'no telemetry yet', false, 0));
  }
  return sendJson(res, 200, dev.stateCache);
}

function checkBasicAuth(req, password) {
  const m = /^Basic\s+(\S+)$/i.exec(req.headers.authorization || '');
  if (!m) return false;
  const s = Buffer.from(m[1], 'base64').toString('utf8');
  const i = s.indexOf(':');
  return i >= 0 && s.slice(0, i) === OTA_USER && s.slice(i + 1) === password;
}

// POST /update?type=fw|fs, like src/ota.cpp: the decision falls on the first
// chunk of the file part; once accepted the device always restarts.
function handleUpdatePost(req, res, url) {
  const d = dev;
  const auth = checkBasicAuth(req, d.otaPassword());
  const type = url.searchParams.get('type');
  const isFs = type === 'fs';
  const ctype = req.headers['content-type'] || '';
  const bm = /boundary=(?:"([^"]+)"|([^;\s]+))/i.exec(ctype);
  const boundary = /^multipart\/form-data/i.test(ctype) && bm ? bm[1] || bm[2] : null;
  const ctx = { started: false, status: 0, err: '', head: Buffer.alloc(0), fileStart: -1, total: 0, tail: Buffer.alloc(0), accepted: false };

  const refuse = (status, err) => {
    ctx.status = status;
    ctx.err = err;
  };
  const finish = (ok, msg) => {
    d.ota.phase = 'ended';
    d.ota.ok = ok;
    d.ota.result = msg;
  };
  const start = (firstByte) => {
    ctx.started = true;
    if (!auth) return refuse(401, '');
    if (type !== null && type !== 'fw' && type !== 'fs') return refuse(400, 'bad type, use fw or fs');
    if (d.ota.phase !== 'idle') return refuse(409, 'busy');
    if (!d.motorsIdle) return refuse(409, 'motors running');
    if (isFs && firstByte === OTA_IMAGE_MAGIC) return refuse(400, 'firmware image sent as fs');
    if (!isFs && firstByte >= 0 && firstByte !== OTA_IMAGE_MAGIC) return refuse(400, 'not a firmware image');
    d.otaActive = true;
    d.ota = { phase: 'running', isFs, written: 0, lastDataAt: Date.now() };
    ctx.accepted = true;
    ctx.status = 200;
    return undefined;
  };

  req.on('data', (chunk) => {
    ctx.total += chunk.length;
    ctx.tail = Buffer.concat([ctx.tail, chunk]);
    if (ctx.tail.length > 512) ctx.tail = ctx.tail.subarray(ctx.tail.length - 512);
    if (boundary && !ctx.started && ctx.head.length < 65536) {
      ctx.head = Buffer.concat([ctx.head, chunk]);
      const fn = ctx.head.indexOf('filename=');
      const he = fn >= 0 ? ctx.head.indexOf('\r\n\r\n', fn) : -1;
      if (he >= 0 && ctx.head.length > he + 4) {
        ctx.fileStart = he + 4;
        start(ctx.head[he + 4]);
      }
    }
    if (ctx.accepted && d.ota.phase === 'running') {
      d.ota.written = Math.max(0, ctx.total - ctx.fileStart);
      d.ota.lastDataAt = Date.now();
    }
  });
  req.on('close', () => {
    if (!req.complete && ctx.accepted && d.ota.phase === 'running') finish(false, 'aborted: client disconnected');
  });
  req.on('end', () => {
    if (!ctx.started && boundary && ctx.head.indexOf('filename=') >= 0) {
      // the file part started in the very last chunk
      const fn = ctx.head.indexOf('filename=');
      const he = ctx.head.indexOf('\r\n\r\n', fn);
      if (he >= 0) {
        ctx.fileStart = he + 4;
        start(ctx.head.length > he + 4 ? ctx.head[he + 4] : -1);
      }
    }
    if (!ctx.started) {
      if (!auth) return send401(req, res);
      return sendJson(res, 400, J({ ok: false, error: 'no file in request' }));
    }
    if (ctx.status === 401) return send401(req, res);
    if (ctx.accepted && d.ota.phase === 'running') {
      const end = ctx.tail.lastIndexOf(`\r\n--${boundary}`);
      if (end < 0) {
        refuse(500, 'incomplete upload');
        finish(false, 'incomplete upload');
      } else {
        const fileLen = ctx.total - ctx.tail.length + end - ctx.fileStart;
        d.ota.written = Math.max(0, fileLen);
        if (fileLen <= 0) {
          refuse(400, 'empty file');
          finish(false, 'empty file');
        } else {
          finish(true, 'ok');
        }
      }
    }
    if (ctx.status === 200) return sendJson(res, 200, J({ ok: true }));
    return sendJson(res, ctx.status, J({ ok: false, error: ctx.err }));
  });
}

// Like ota.cpp: a request that already carried credentials (the panel's XHR)
// gets a plain JSON 401, so the browser does not open its own login dialog.
function send401(req, res) {
  if (req.headers.authorization) return sendJson(res, 401, J({ ok: false, error: 'bad password' }));
  sendRaw(res, 401, 'text/plain', '', { 'WWW-Authenticate': `Basic realm="${OTA_REALM}"` });
}

const FALLBACK_PAGE = `<!doctype html><html lang="pl"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Turbina (mock)</title>
<body style="font:16px system-ui;background:#0f1114;color:#d6d9de;padding:24px">
<h1 style="font-size:20px">Brak panelu</h1><p>Mock nie znalazł pliku index.html w folderze:</p>
<pre>${DATA_DIR.replace(/[<>&]/g, '')}</pre><p>API działa: <a style="color:#f2a900" href="/api/state">/api/state</a></p>`;

async function serveStatic(req, res, pathname) {
  let rel;
  try {
    rel = decodeURIComponent(pathname);
  } catch {
    rel = pathname;
  }
  if (rel.endsWith('/')) rel += 'index.html';
  const file = path.resolve(DATA_DIR, `.${rel}`);
  if (file !== DATA_DIR && !file.startsWith(DATA_DIR + path.sep)) return notFound(res);
  const ext = path.extname(file).toLowerCase();
  const headers = {
    'Content-Type': MIME[ext] || 'application/octet-stream',
    'Cache-Control': LONG_CACHE.has(ext) ? 'max-age=86400' : 'no-cache',
  };
  let body = null;
  for (const [candidate, gz] of [[file, false], [`${file}.gz`, true]]) {
    try {
      const st = await fs.promises.stat(candidate);
      if (!st.isFile()) continue;
      body = await fs.promises.readFile(candidate);
      if (gz) headers['Content-Encoding'] = 'gzip';
      break;
    } catch {
      // try the next one
    }
  }
  if (!body) {
    if (rel === '/index.html') return sendRaw(res, 200, 'text/html; charset=utf-8', FALLBACK_PAGE, { 'Cache-Control': 'no-store' });
    return notFound(res);
  }
  headers['Content-Length'] = body.length;
  res.writeHead(200, headers);
  res.end(req.method === 'HEAD' ? undefined : body);
  return undefined;
}

function notFound(res) {
  sendRaw(res, 404, 'text/plain', '404: nie ma takiej strony');
}

// --- test hooks ---------------------------------------------------------------

const KNOB_TYPES = {
  bat: 'number', instant: 'boolean', pos: 'number', magnet: 'boolean', sensorReversed: 'boolean', motorReversed: 'boolean',
  endL: 'boolean|null', endR: 'boolean|null', lag: 'number', noWs: 'boolean', drain: 'boolean',
  sag: 'boolean', verbose: 'boolean',
};

function mockState() {
  return {
    online: !!dev,
    up: dev ? dev.millis() : null,
    sensor: opt.sensor,
    truePos: Math.round(phys.truePos * 100) / 100,
    batOcvMv: Math.round(phys.ocvMv),
    batTerminalMv: Math.round(phys.terminalMv()),
    knobs,
    ws: [...net.clients.values()].map((c) => ({ id: c.id, ip: c.ip, silentMs: Date.now() - c.lastRx })),
    failsafe: dev ? { active: dev.fs.active(), reason: dev.fs.reason(), owner: formatClient(dev.fs.owner) } : null,
    ota: dev ? { active: dev.otaActive, phase: dev.ota.phase } : null,
    yawBalance: store.yaw.bal,
    cfgRev: dev ? dev.rev : null,
  };
}

async function handleMock(req, res, url) {
  const p = url.pathname;
  const reply = (code, obj) => sendJson(res, code, J(obj), false);
  if (p === '/__mock/frames' && req.method === 'GET') {
    const since = Number(url.searchParams.get('since') || 0);
    const t = url.searchParams.get('t');
    return reply(200, net.frames.filter((f) => f.ts >= since && (!t || (f.frame && f.frame.t === t))));
  }
  if (p === '/__mock/state' && req.method === 'GET') return reply(200, mockState());
  if (req.method !== 'POST') return reply(405, { ok: false, error: 'use POST' });
  if (p === '/__mock/clear') {
    net.frames = [];
    return reply(200, { ok: true });
  }
  if (p === '/__mock/drop') return reply(200, { ok: true, dropped: net.dropAll() });
  if (p === '/__mock/reboot') {
    reply(200, { ok: true });
    if (dev) dev.requestReboot(50, 'mock');
    return undefined;
  }
  if (p === '/__mock/set') {
    const { buf } = await readBody(req, 4096);
    let o;
    try {
      o = JSON.parse(buf.toString('utf8') || '{}');
    } catch {
      return reply(400, { ok: false, error: 'body is not JSON' });
    }
    if (!isObj(o)) return reply(400, { ok: false, error: 'body must be an object' });
    for (const [k, v] of Object.entries(o)) {
      const want = KNOB_TYPES[k];
      if (!want) return reply(400, { ok: false, error: `unknown key ${k}`, keys: Object.keys(KNOB_TYPES) });
      const okType = want.split('|').some((w) => (w === 'null' ? v === null : typeof v === w));
      if (!okType || (typeof v === 'number' && !Number.isFinite(v))) return reply(400, { ok: false, error: `${k} must be ${want}` });
    }
    if ('bat' in o) phys.ocvMv = clamp(o.bat, 0, 20) * 1000;
    if ('pos' in o) phys.truePos = o.pos;
    if ('lag' in o) knobs.lag = clamp(Math.round(o.lag), 0, 10000);
    for (const k of ['magnet', 'sensorReversed', 'motorReversed', 'endL', 'endR', 'noWs', 'drain', 'sag', 'verbose']) {
      if (k in o) knobs[k] = o[k];
    }
    if (o.instant && dev) dev.battery.restart();
    return reply(200, { ok: true, state: mockState() });
  }
  return reply(404, { ok: false, error: 'unknown hook' });
}

async function onRequest(req, res) {
  const url = new URL(req.url, 'http://mock');
  const p = url.pathname;
  if (p.startsWith('/__mock/')) return handleMock(req, res, url);
  if (!dev) return req.socket.destroy();  // restarting: nobody answers
  net.touch(remoteIp(req));
  if (p === '/api/state' && req.method === 'GET') return handleApiState(req, res);
  if (p === '/api/cmd' && req.method === 'POST') return handleApiCmd(req, res);
  if (p === '/update' && req.method === 'GET') {
    res.writeHead(302, { Location: '/#ota', 'Content-Length': 0 });
    return res.end();
  }
  if (p === '/update' && req.method === 'POST') return handleUpdatePost(req, res, url);
  if (req.method === 'GET' || req.method === 'HEAD') return serveStatic(req, res, p);
  return notFound(res);
}

function rejectUpgrade(socket, code, text) {
  socket.end(`HTTP/1.1 ${code} ${text}\r\nContent-Length: 0\r\nConnection: close\r\n\r\n`);
}

function onUpgrade(req, socket, head) {
  socket.on('error', () => {});
  const url = new URL(req.url, 'http://mock');
  if (!dev) return socket.destroy();
  if (url.pathname !== '/ws') return rejectUpgrade(socket, 404, 'Not Found');
  if (knobs.noWs) return rejectUpgrade(socket, 404, 'Not Found');
  const key = req.headers['sec-websocket-key'];
  if ((req.headers.upgrade || '').toLowerCase() !== 'websocket' || !key) return rejectUpgrade(socket, 400, 'Bad Request');
  const accept = crypto.createHash('sha1').update(key + WS_GUID).digest('base64');
  socket.write(
    'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n' +
      `Sec-WebSocket-Accept: ${accept}\r\n\r\n`,
  );
  socket.setNoDelay(true);
  const ip = remoteIp(req);
  net.touch(ip);
  const c = net.accept(socket, ip);
  if (head && head.length) c.onData(head);
  return undefined;
}

// ---------------------------------------------------------------------------
// Start
// ---------------------------------------------------------------------------

const server = http.createServer((req, res) => {
  Promise.resolve(onRequest(req, res)).catch((e) => {
    process.stderr.write(`mock_esp: ${req.method} ${req.url}: ${e.stack || e}\n`);
    if (!res.headersSent) sendRaw(res, 500, 'text/plain', 'mock error');
    else res.destroy();
  });
});
server.on('upgrade', (req, socket, head) => {
  try {
    onUpgrade(req, socket, head);
  } catch (e) {
    process.stderr.write(`mock_esp: upgrade: ${e.stack || e}\n`);
    socket.destroy();
  }
});
server.on('clientError', (err, socket) => socket.destroy());
server.on('error', (e) => {
  process.stderr.write(`mock_esp: ${e.code === 'EADDRINUSE' ? `port ${opt.port} is taken, try --port` : e.message}\n`);
  process.exit(1);
});

boot('poweron');
server.listen(opt.port, opt.host, () => {
  const where = opt.host === '0.0.0.0' ? 'localhost' : opt.host;
  process.stdout.write(
    `mock_esp: http://${where}:${opt.port}  sensor ${opt.sensor}, battery ${opt.bat} V` +
      `${knobs.lag ? `, lag ${knobs.lag} ms` : ''}${knobs.noWs ? ', WebSocket off' : ''}\n` +
      `mock_esp: panel from ${DATA_DIR}${fs.existsSync(path.join(DATA_DIR, 'index.html')) ? '' : ' (index.html missing)'}\n`,
  );
});

setInterval(() => {
  if (!dev) return;
  try {
    dev.loop();
  } catch (e) {
    process.stderr.write(`mock_esp: loop: ${e.stack || e}\n`);
  }
}, 5);

process.on('SIGINT', () => process.exit(0));
process.on('SIGTERM', () => process.exit(0));
