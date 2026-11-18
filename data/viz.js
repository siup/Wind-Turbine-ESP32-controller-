// 3D view of the turbine for the panel. index.html loads this file the first
// time the user switches to 3D (button in the status bar).
//
// Plain canvas 2D with a small perspective projection, no libraries: the file
// ships gzipped from LittleFS next to index.html.
//
// The view has no link of its own. It reads the panel's state (st, cfg, blk)
// and sends through the panel's own code (rotSend, setSp, the yaw hold timer,
// yStop), so failsafe, deadman, STOP ordering, HTTP and BLE behave exactly as
// with the buttons. The model shows telemetry, not the finger: when the
// controller refuses a command, the model does not move.
//
// Gestures:
//   blades   swipe around the hub like spinning a toy windmill: the swipe
//            speed spins the rotor up, then it slows down by itself and
//            stops (free spin); another swipe adds speed, a swipe against
//            the spin brakes, a tap on the hub stops it at once
//   below    drag sideways: hold-to-run yaw (further = faster), release stops;
//            with an AS5600 the drag sets a target angle, release goes there
'use strict';
(() => {
const PI = Math.PI, TAU = 2 * PI, RAD = PI / 180;
const CL = {tx: '#d7dce2', dm: '#7d8590', am: '#f2a900', ok: '#3ddc84', al: '#ff3b30', in: '#4cc9f0', bd: '#2a2f37', k: '#0b0d10'};
const FONT = 'ui-monospace,"SF Mono","Roboto Mono","DejaVu Sans Mono",Menlo,Consolas,monospace';

// World units, y up, the tower stands on the origin. Heading a (degrees, the
// panel's compass angle, clockwise seen from above) points the rotor along
// hd(a); heading 0 faces the camera side.
const TOWER_H = 2.6, TOWER_R0 = 0.13, TOWER_R1 = 0.07;
const NAC_Y = 2.78, NAC_F = 0.36, NAC_B = -0.62, NAC_W = 0.16, NAC_H = 0.17;
const HUB_D = 0.5, HUB_R = 0.15, BLADE_L = 1.35, RING_R = 1.75;
const ROTOR_RPS = 1.6;   // blade turns per second drawn at 100 % (a look, not a measurement)
const YAW_DPS = 40;      // nacelle deg/s at 100 % when there is no position at all
const DZ = 24;           // px of sideways drag before the nacelle starts
const BLADE = [[0.12, 0.05], [0.45, 0.1], [BLADE_L, 0.025], [BLADE_L, -0.01], [0.45, -0.05], [0.12, -0.05]];

const hd = a => [-Math.sin(a * RAD), 0, -Math.cos(a * RAD)];
const lat = a => [Math.cos(a * RAD), 0, -Math.sin(a * RAD)];
const add = (p, v, k) => [p[0] + v[0] * k, p[1] + v[1] * k, p[2] + v[2] * k];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const m360 = a => ((a % 360) + 360) % 360;
const LIGHT = (() => {const l = [-0.45, 0.75, -0.5], n = Math.hypot(...l); return l.map(v => v / n)})();

// Camera: fixed, a little to the side and above so the model reads as 3D.
const CAM_T = [0, 1.9, 0];
let cp, fw, rt, cu, sgnR;
(() => {
 const az = 28 * RAD, el = 14 * RAD, d = 8.4;
 cp = [CAM_T[0] - Math.sin(az) * Math.cos(el) * d, CAM_T[1] + Math.sin(el) * d, CAM_T[2] - Math.cos(az) * Math.cos(el) * d];
 const f = [CAM_T[0] - cp[0], CAM_T[1] - cp[1], CAM_T[2] - cp[2]], n = Math.hypot(...f);
 fw = f.map(v => v / n);
 const r = [fw[2], 0, -fw[0]], m = Math.hypot(...r);   // up x forward
 rt = r.map(v => v / m);
 cu = [fw[1] * rt[2] - fw[2] * rt[1], fw[2] * rt[0] - fw[0] * rt[2], fw[0] * rt[1] - fw[1] * rt[0]];
})();

let S = 1, cx = 0, cy = 0;
function pj(p) {
 const x = p[0] - cp[0], y = p[1] - cp[1], z = p[2] - cp[2], d = x * fw[0] + y * fw[1] + z * fw[2], k = S / d;
 return [cx + (x * rt[0] + y * rt[1] + z * rt[2]) * k, cy - (x * cu[0] + y * cu[1] + z * cu[2]) * k, d];
}
// Turntable: a sideways drag moves the near side of the nacelle with the
// finger. sgnR is +1 when that means a growing angle (yaw RIGHT).
(() => {
 S = 1; cx = cy = 0;
 const a = Math.atan2(-cp[0], -cp[2]) / RAD;  // heading that points at the camera
 sgnR = pj(add([0, 0, 0], hd(a + 1), RING_R))[0] > pj(add([0, 0, 0], hd(a), RING_R))[0] ? 1 : -1;
})();

// DOM
const box = $('vz'), cv = D.createElement('canvas'), ctx = cv.getContext('2d');
const bg = D.createElement('canvas'), bx = bg.getContext('2d');
box.append(cv);
let W = 0, H = 0, dpr = 1, shown = false, raf = 0, lastT = 0;
let towerG = null;

function fitBox() {
 const top = box.getBoundingClientRect().top + scrollY, ef = $('ef').offsetHeight;
 box.style.height = Math.max(280, Math.floor(innerHeight - top - ef - 4)) + 'px';
}
function resize() {
 W = box.clientWidth; H = box.clientHeight;
 if (!W || !H) return;
 dpr = Math.min(2, devicePixelRatio || 1);
 for (const c of [cv, bg]) {c.width = Math.round(W * dpr); c.height = Math.round(H * dpr)}
 // fit the scene (ring with labels, blades at any heading) between the
 // readouts at the top and the hints at the bottom
 S = 1; cx = cy = 0;
 let x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
 const grow = (x, y) => {x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y)};
 for (let a = 0; a < 360; a += 15) {
  const q = pj(add([0, 0, 0], hd(a), RING_R + 0.38)), h = pj(add([0, NAC_Y, 0], hd(a), HUB_D)), r = (BLADE_L + 0.08) / h[2];
  grow(q[0], q[1]); grow(h[0] - r, h[1] - r); grow(h[0] + r, h[1] + r);
 }
 const top = 66, bot = 44, side = 10, aw = W - 2 * side, ah = H - top - bot;
 S = Math.min(aw / (x1 - x0), ah / (y1 - y0));
 cx = side + (aw - (x1 - x0) * S) / 2 - x0 * S;
 cy = top + (ah - (y1 - y0) * S) / 2 - y0 * S;
 towerG = null;
 drawGround();
}

// Static layer: compass ring on the ground.
function drawGround() {
 bx.setTransform(dpr, 0, 0, dpr, 0, 0);
 bx.clearRect(0, 0, W, H);
 const O = [0, 0, 0];
 bx.beginPath();
 for (let a = 0; a <= 360; a += 5) {const p = pj(add(O, hd(a), RING_R)); a ? bx.lineTo(p[0], p[1]) : bx.moveTo(p[0], p[1])}
 bx.fillStyle = '#13161b'; bx.fill();
 bx.strokeStyle = '#3a4049'; bx.lineWidth = 2; bx.stroke();
 for (let a = 0; a < 360; a += 10) {
  const M = a % 90 == 0, m = a % 30 == 0, p = pj(add(O, hd(a), RING_R)), q = pj(add(O, hd(a), RING_R - (M ? 0.22 : m ? 0.14 : 0.07)));
  bx.beginPath(); bx.moveTo(p[0], p[1]); bx.lineTo(q[0], q[1]);
  bx.strokeStyle = M ? '#8d949d' : '#4a515b'; bx.lineWidth = M ? 2 : 1; bx.stroke();
 }
 bx.font = '600 12px ' + FONT; bx.textAlign = 'center'; bx.textBaseline = 'middle';
 for (const a of [0, 90, 180, 270]) {
  const p = pj(add(O, hd(a), RING_R + 0.3));
  bx.fillStyle = a ? CL.dm : CL.am;
  bx.fillText(a + '°', p[0], p[1]);
 }
 // tower foot
 const b = pj(O), r = S * TOWER_R0 / b[2];
 bx.beginPath(); bx.ellipse(b[0], b[1], r * 2.2, r * 0.7, 0, 0, TAU); bx.fillStyle = '#23272e'; bx.fill();
}

const shade = (c, k) => 'rgb(' + (c[0] * k | 0) + ',' + (c[1] * k | 0) + ',' + (c[2] * k | 0) + ')';
const BODY = [200, 206, 213], NAC = [176, 183, 191], BLD = [222, 226, 231];
function poly(pts, fill, stroke) {
 ctx.beginPath();
 pts.forEach((p, i) => i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]));
 ctx.closePath();
 if (fill) {ctx.fillStyle = fill; ctx.fill()}
 if (stroke) {ctx.strokeStyle = stroke; ctx.stroke()}
}

function drawTower() {
 const b = pj([0, 0, 0]), t = pj([0, TOWER_H, 0]), rb = S * TOWER_R0 / b[2], rtp = S * TOWER_R1 / t[2];
 if (!towerG) {
  towerG = ctx.createLinearGradient(b[0] - rb, 0, b[0] + rb, 0);
  towerG.addColorStop(0, '#5a616a'); towerG.addColorStop(0.35, '#d0d5db'); towerG.addColorStop(1, '#4d535b');
 }
 poly([[b[0] - rb, b[1]], [b[0] + rb, b[1]], [t[0] + rtp, t[1]], [t[0] - rtp, t[1]]], towerG);
}

// Nacelle box: six faces, the visible ones (convex, so no sorting needed).
function boxFaces(a, ghost) {
 const h = hd(a), l = lat(a), C = [0, NAC_Y, 0];
 const P = (f, s, u) => add(add(add(C, h, f), l, s * NAC_W), [0, 1, 0], u * NAC_H);
 const F = [
  [h, [P(NAC_F, -1, -1), P(NAC_F, 1, -1), P(NAC_F, 1, 1), P(NAC_F, -1, 1)]],
  [[-h[0], 0, -h[2]], [P(NAC_B, -1, -1), P(NAC_B, -1, 1), P(NAC_B, 1, 1), P(NAC_B, 1, -1)]],
  [l, [P(NAC_B, 1, -1), P(NAC_B, 1, 1), P(NAC_F, 1, 1), P(NAC_F, 1, -1)]],
  [[-l[0], 0, -l[2]], [P(NAC_B, -1, -1), P(NAC_F, -1, -1), P(NAC_F, -1, 1), P(NAC_B, -1, 1)]],
  [[0, 1, 0], [P(NAC_B, -1, 1), P(NAC_F, -1, 1), P(NAC_F, 1, 1), P(NAC_B, 1, 1)]],
  [[0, -1, 0], [P(NAC_B, -1, -1), P(NAC_B, 1, -1), P(NAC_F, 1, -1), P(NAC_F, -1, -1)]],
 ];
 for (const [n, q] of F) {
  const c = q[0], v = [cp[0] - c[0], cp[1] - c[1], cp[2] - c[2]];
  if (dot(n, v) <= 0) continue;
  const pts = q.map(pj);
  if (ghost) {ctx.setLineDash([5, 4]); ctx.lineWidth = 1.5; poly(pts, null, ghost); ctx.setLineDash([])}
  else {ctx.lineWidth = 1; poly(pts, shade(NAC, 0.42 + 0.58 * Math.max(0, dot(n, LIGHT))), '#0b0d1066')}
 }
}

function drawRotor(a, th, front) {
 const h = hd(a), l = lat(a), U = [0, 1, 0], hub = add([0, NAC_Y, 0], h, HUB_D);
 const k = 0.5 + 0.4 * Math.abs(dot(h, LIGHT));
 const blades = () => {
  for (let i = 0; i < 3; i++) {
   const f = th + i * TAU / 3, c = Math.cos(f), s = Math.sin(f);
   const e = [U[0] * c + l[0] * s, U[1] * c + l[1] * s, U[2] * c + l[2] * s];
   const w = [-U[0] * s + l[0] * c, -U[1] * s + l[1] * c, -U[2] * s + l[2] * c];
   ctx.lineWidth = 1;
   poly(BLADE.map(([r, q]) => pj(add(add(hub, e, r), w, q))), shade(BLD, front ? k : k * 0.7), '#0b0d1080');
  }
 };
 const spinner = () => {
  const p = pj(hub), r = S * HUB_R / p[2], g = ctx.createRadialGradient(p[0] - r * 0.35, p[1] - r * 0.35, r * 0.1, p[0], p[1], r);
  g.addColorStop(0, '#f1f3f5'); g.addColorStop(1, '#7c838c');
  ctx.beginPath(); ctx.arc(p[0], p[1], r, 0, TAU); ctx.fillStyle = g; ctx.fill();
  if (front) {ctx.beginPath(); ctx.arc(p[0], p[1], r * 0.32, 0, TAU); ctx.fillStyle = CL.am; ctx.fill()}
 };
 if (front) {blades(); spinner()} else {spinner(); blades()}
}

// State read from the panel each frame
let hdg = 0, hdgOk = false, th = 0, lastSt = null, stT = 0, angPrev = null, angRate = 0;
const Y = () => (st && st.yaw) || {};
const R = () => (st && st.rotor) || {};
const hasAngle = () => num(Y().angle);
function posNow() {
 const y = Y();
 return num(y.angle) ? y.angle : num(y.pos) && y.est ? m360(y.pos) : null;
}

function update(dt, t) {
 const y = Y(), r = R(), a = posNow();
 if (st !== lastSt) {
  if (a != null && angPrev != null && lastSt) {
   const d = ((a - angPrev + 540) % 360) - 180, s = Math.max(0.05, (t - stT) / 1000);
   angRate = y.moving ? Math.max(-400, Math.min(400, d / s)) : 0;
  }
  angPrev = a; lastSt = st; stT = t;
 }
 if (a != null) {
  const pred = a + (y.moving ? angRate * Math.min(0.25, (t - stT) / 1000) : 0);
  if (!hdgOk) {hdg = pred; hdgOk = true}
  hdg = m360(hdg + (((pred - hdg + 540) % 360) - 180) * (1 - Math.exp(-dt * 12)));
 } else {
  hdgOk = false;
  if (y.moving) {
   const ms = cfg && cfg.yawMs360 > 0 ? cfg.yawMs360 : 0, v = (ms ? 360000 / ms : YAW_DPS) * (num(y.speed) ? y.speed : 60) / 100;
   hdg = m360(hdg + (y.moving == 'right' ? v : -v) * dt);
  }
 }
 const act = num(r.actual) ? r.actual : 0;
 th = (th + dt * TAU * ROTOR_RPS * act / 100) % TAU;
}

// Gestures
let g = null, ghost = null;
let hubS = [0, 0], rotS = 1, hubR = 1, spinS = 1;
function evXY(e) {const b = cv.getBoundingClientRect(); return [e.clientX - b.left, e.clientY - b.top]}
// Free spin. After a swipe the panel lowers the setpoint a little every
// 200 ms, like friction on a toy windmill: dv/dt = -(FR_A + FR_B * v), about
// 10 s from 100 % to a standstill, ending with speed 0 (coast, no brake).
// Anything else that sets the rotor ends it and is left alone: START/STOP,
// the slider or E-STOP in the panel (they change rLast), a failsafe, the
// controller dropping the target, or the controls locking (blk).
const FR_A = 2.5, FR_B = 0.22;
let co = null;
const coEnd = () => {co = null};
const coMin = () => Math.max(5, cfg && num(cfg.rotorMin) ? cfg.rotorMin : 0);
function coSend(v, k) {const ok = rotSend(v, k); if (ok && co) {co.sent = v; co.ts = now()} return ok}
function coTick() {
 if (!co) return;
 const t = now(), r = R(), dt = (t - co.t) / 1000;
 co.t = t;
 if (blk || (st && st.fs && st.fs.active) || rLast !== co.sent || t - co.ts > 700 && r.target === 0) return coEnd();
 co.v -= dt * (FR_A + FR_B * co.v);
 if (co.v < coMin()) {coEnd(); thrClear(); rotSend(0, 2); return}
 const v = Math.round(co.v);
 if (v != co.sent) coSend(v, 1);
}
setInterval(coTick, 200);
function spin(v) {
 thrClear();
 if (v <= 0) {coEnd(); rotSend(0, 2); return}
 v = Math.min(v, rotMax());
 co = {v, t: now(), sent: -1, ts: 0};
 setSp(v);
 coSend(Math.round(v), 2);
}
const curSpeed = () => co ? co.v : rotRun() ? (num(R().target) && R().target > 0 ? R().target : sp) : 0;

// Angular speed of the finger around the hub over the last 120 ms, signed
// along the spin. 0.6 rad/s (a slow, deliberate swipe) is worth 10 %,
// 12 rad/s (a quick flick, about 1500 px/s halfway out on the blade) 100 %.
// Along the spin it spins the rotor up (or adds half of it when already
// faster), against the spin it takes that much off. A finger that stopped
// before lifting does nothing. Returns the new speed, 0 for stop, or null.
function flick(release, t) {
 const q = g.q;
 if (q.length < 2 || release && t - q[q.length - 1][0] > 120) return null;
 const a = q[0], b = q[q.length - 1], w = (b[1] - a[1]) / Math.max(0.016, (b[0] - a[0]) / 1000) * spinS, m = Math.abs(w);
 if (m < 0.6) return null;
 const p = Math.min(100, Math.max(10, 10 + (m - 0.6) / 11.4 * 90)), cur = curSpeed();
 if (w > 0) return Math.round(Math.min(rotMax(), Math.max(p, cur + p / 2)));
 if (!cur) return null;
 return cur - p < coMin() ? 0 : Math.round(cur - p);
}
function yStart(d, spd) {
 hold = d; K(yb[d], 'on', 1);
 send({t: 'yaw', dir: d, speed: spd}, 2);
 hT = setInterval(() => send({t: 'yaw', hold: d}, 2), 200);
 g.dir = d; g.spd = spd; g.t = now();
 vib(10);
}
function yDrag(dx) {
 const ad = Math.abs(dx), y = Y();
 const d = dx * sgnR > 0 ? 'right' : 'left';
 if (ad < DZ || (d == 'left' ? y.limitL : y.limitR)) {
  if (g.dir) {if (hold) yStop(); g.dir = null}
  return;
 }
 const spd = Math.round(Math.min(100, 20 + (ad - DZ) / (W * 0.32) * 80) / 10) * 10;
 if (d != g.dir || hold != d) {if (hold) yStop(); yStart(d, spd)}
 else if (spd != g.spd && now() - g.t > 150) {send({t: 'yaw', dir: d, speed: spd}, 2); g.spd = spd; g.t = now()}
}
function gEnd(commit, t) {
 if (!g) return;
 const k = g.k;
 t = t || now();
 if (k == 'r' && commit) {
  if (g.moved < 10 && t - g.t0 < 400) {
   if (g.hub && (co || rotRun())) {spin(0); vib(15)}
  } else {
   const v = flick(true, t);
   if (v != null) {spin(v); vib(15)}
  }
 } else if (k == 'y') {
  if (hold) yStop();
  else if (commit && g.moved < 10 && num(Y().goto)) yStop();
 } else if (k == 'g' && commit) {
  if (g.tgt != null && g.moved >= 10) {
   const v = Math.round(m360(g.tgt)) % 360;
   send({t: 'yaw', goto: v}); vib(15);
   ghost = {a: v, t: now()};
  } else if (g.moved < 10 && num(Y().goto)) yStop();
 }
 g = null;
}
on(cv, 'pointerdown', e => {
 if (g || e.button > 0) return;
 e.preventDefault();
 if (blk) {vib(30); return}
 const [x, y] = evXY(e), dx = x - hubS[0], dy = y - hubS[1], r = Math.hypot(dx, dy);
 try {cv.setPointerCapture(e.pointerId)} catch (z) {}
 if (r <= rotS * 1.08) {
  const t = e.timeStamp;
  g = {k: 'r', id: e.pointerId, t0: t, x0: x, y0: y, moved: 0, hub: r <= Math.max(30, hubR * 1.8), a: Math.atan2(dy, dx), acc: 0, q: [[t, 0]], pv: null};
 } else {
  g = {k: hasAngle() ? 'g' : 'y', id: e.pointerId, t0: now(), x0: x, y0: y, moved: 0, dir: null, spd: 0, t: 0, h0: hdg, tgt: null};
 }
});
on(cv, 'pointermove', e => {
 if (!g || e.pointerId !== g.id) return;
 if (blk) return gEnd(false);
 const [x, y] = evXY(e);
 g.moved = Math.max(g.moved, Math.hypot(x - g.x0, y - g.y0));
 if (g.k == 'r') {
  const a = Math.atan2(y - hubS[1], x - hubS[0]);
  let d = a - g.a; d -= TAU * Math.round(d / TAU);
  g.a = a; g.acc += d;
  const t = e.timeStamp;
  g.q.push([t, g.acc]);
  while (g.q.length > 2 && t - g.q[0][0] > 120) g.q.shift();
  g.pv = flick(false, t);
 } else if (g.k == 'y') yDrag(x - g.x0);
 else if (g.moved >= 10) g.tgt = m360(g.h0 + (x - g.x0) * sgnR * 360 / (W * 1.1));
});
for (const n of ['pointerup', 'pointercancel', 'lostpointercapture']) on(cv, n, e => {if (g && e.pointerId === g.id) gEnd(n == 'pointerup', e.timeStamp)});
on(cv, 'contextmenu', e => e.preventDefault());
on(window, 'blur', () => gEnd(false));

// Drawing
function txt(s, x, y, size, color, align, weight) {
 ctx.font = (weight || 600) + ' ' + size + 'px ' + FONT;
 ctx.textAlign = align || 'left'; ctx.fillStyle = color;
 ctx.fillText(s, x, y);
}
function ringMark(a, color, size) {
 const O = [0, 0, 0], p = pj(add(O, hd(a), RING_R + 0.02)), q = pj(add(O, hd(a), RING_R + 0.2));
 const ang = Math.atan2(p[1] - q[1], p[0] - q[0]), s = size || 7;
 ctx.beginPath();
 ctx.moveTo(p[0], p[1]);
 ctx.lineTo(q[0] + Math.cos(ang + PI / 2) * s, q[1] + Math.sin(ang + PI / 2) * s);
 ctx.lineTo(q[0] - Math.cos(ang + PI / 2) * s, q[1] - Math.sin(ang + PI / 2) * s);
 ctx.closePath(); ctx.fillStyle = color; ctx.fill();
}
function twist() {
 const y = Y(), ms = cfg && cfg.yawMs360 > 0 ? cfg.yawMs360 : 0;
 const deg = num(y.pos) ? y.pos : num(y.bal) && ms ? y.bal / ms * 360 : null;
 if (deg == null) return;
 const lim = y.posMax > 0 ? y.posMax : y.balMax > 0 && ms ? y.balMax / ms * 360 : 0, O = [0, 0, 0];
 if (lim > 0) for (const s of [-1, 1]) {
  const a = s * lim, p = pj(add(O, hd(a), RING_R - 0.3)), q = pj(add(O, hd(a), RING_R + 0.08));
  ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); ctx.strokeStyle = CL.al; ctx.lineWidth = 3; ctx.stroke();
 }
 if (Math.abs(deg) < 1) return;
 const fr = lim > 0 ? Math.abs(deg) / lim : 0, end = Math.max(-2160, Math.min(2160, deg)), st5 = end > 0 ? 5 : -5;
 ctx.beginPath();
 for (let a = 0; ; a += st5) {
  const b = Math.abs(a) >= Math.abs(end) ? end : a, p = pj(add(O, hd(b), RING_R - 0.32 - 0.08 * Math.abs(b) / 360));
  a ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]);
  if (b == end) break;
 }
 ctx.strokeStyle = fr >= 0.9 ? CL.al : CL.am; ctx.lineWidth = 3; ctx.lineCap = 'round'; ctx.stroke(); ctx.lineCap = 'butt';
}

function render(t) {
 ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
 ctx.clearRect(0, 0, W, H);
 const y = Y(), r = R(), dim = !!blk || !st;
 ctx.globalAlpha = dim ? 0.45 : 1;
 ctx.drawImage(bg, 0, 0, W, H);
 twist();
 const gt = num(y.goto) ? y.goto : null;
 if (gt != null) ringMark(gt, CL.in);
 ringMark(hdg, CL.am, 8);

 const h = hd(hdg), hub = add([0, NAC_Y, 0], h, HUB_D), hp = pj(hub);
 const front = dot(h, [cp[0] - hub[0], cp[1] - hub[1], cp[2] - hub[2]]) > 0;
 hubS = hp; rotS = S * BLADE_L / hp[2]; hubR = S * HUB_R / hp[2];
 const pu = pj(add(hub, [0, 1, 0], 0.5)), pl = pj(add(hub, lat(hdg), 0.5));
 spinS = (pu[0] - hp[0]) * (pl[1] - hp[1]) - (pu[1] - hp[1]) * (pl[0] - hp[0]) >= 0 ? 1 : -1;
 if (!front) drawRotor(hdg, th, false);
 drawTower();
 boxFaces(hdg);
 if (front) drawRotor(hdg, th, true);

 // targets: goto from telemetry, the angle being dragged, a goto just sent
 const ga = g && g.k == 'g' && g.tgt != null ? g.tgt : ghost && t - ghost.t < 1500 && gt == null ? ghost.a : gt;
 if (ga != null) {boxFaces(ga, g && g.k == 'g' ? CL.am : CL.in); if (g && g.k == 'g') ringMark(ga, CL.am)}

 // speed ring around the rotor: actual (green), target (amber mark)
 const act = num(r.actual) ? r.actual : 0, tg = num(r.target) ? r.target : 0, rr = rotS + 10;
 ctx.lineWidth = 3;
 ctx.beginPath(); ctx.arc(hp[0], hp[1], rr, 0, TAU); ctx.strokeStyle = CL.bd; ctx.stroke();
 if (act > 0) {ctx.beginPath(); ctx.arc(hp[0], hp[1], rr, -PI / 2, -PI / 2 + TAU * act / 100); ctx.strokeStyle = CL.ok; ctx.stroke()}
 if (tg > 0) {
  const a = -PI / 2 + TAU * tg / 100;
  ctx.beginPath(); ctx.moveTo(hp[0] + Math.cos(a) * (rr - 7), hp[1] + Math.sin(a) * (rr - 7)); ctx.lineTo(hp[0] + Math.cos(a) * (rr + 7), hp[1] + Math.sin(a) * (rr + 7));
  ctx.strokeStyle = CL.am; ctx.lineWidth = 4; ctx.stroke();
 }
 ctx.globalAlpha = 1;

 // gesture preview
 if (g && g.k == 'r' && g.pv != null) {
  const v = g.pv;
  if (v > 0) {ctx.beginPath(); ctx.setLineDash([6, 5]); ctx.arc(hp[0], hp[1], rr + 9, -PI / 2, -PI / 2 + TAU * v / 100); ctx.strokeStyle = CL.am; ctx.lineWidth = 3; ctx.stroke(); ctx.setLineDash([])}
  txt(v ? 'SPIN ' + v + '%' : 'STOP', hp[0], hp[1] - rr - 18, 15, CL.am, 'center', 700);
 }
 if (g && g.k == 'y' && g.dir) txt((g.dir == 'left' ? '◀ LEFT ' : 'RIGHT ▶ ') + g.spd + '%', W / 2, H - 52, 15, CL.am, 'center', 700);
 if (g && g.k == 'g' && g.tgt != null) txt('GO TO ' + Math.round(m360(g.tgt)) % 360 + '°', W / 2, H - 52, 15, CL.am, 'center', 700);

 // readouts (the panel already formats them; reuse its text)
 const run = tg > 0 || act > 0;
 txt('M1 · ROTOR', 12, 18, 11, CL.dm);
 txt(st ? act + '%' : '--', 12, 46, 26, run ? CL.ok : CL.tx, 'left', 700);
 txt('TARGET ' + (st ? tg + '%' : '--') + (co ? ' · FREE SPIN' : ''), 12, 62, 11, co ? CL.am : CL.dm);
 txt('M2 · NACELLE', W - 12, 18, 11, CL.dm, 'right');
 txt($('yA').textContent, W - 12, 46, 26, y.moving || gt != null ? CL.am : CL.tx, 'right', 700);
 const tw = $('tw').hidden ? $('yS').textContent : 'TWIST ' + $('twV').textContent;
 txt(tw, W - 12, 62, 11, CL.dm, 'right');
 txt('SWIPE BLADES = SPIN · TAP HUB = STOP', W / 2, H - 24, 11, CL.dm, 'center');
 txt(hasAngle() ? 'DRAG SIDEWAYS = TARGET ANGLE' : 'DRAG SIDEWAYS = TURN NACELLE', W / 2, H - 8, 11, CL.dm, 'center');

 if (blk) {
  ctx.font = '600 13px ' + FONT;
  const w = Math.min(W - 40, ctx.measureText('⚠ ' + blk).width + 40), x = (W - w) / 2, yy = H / 2 - 22;
  ctx.fillStyle = '#0f1114e6'; ctx.fillRect(x, yy, w, 44);
  ctx.setLineDash([4, 3]); ctx.strokeStyle = '#6e5313'; ctx.lineWidth = 1; ctx.strokeRect(x + 0.5, yy + 0.5, w - 1, 43); ctx.setLineDash([]);
  txt('⚠ ' + blk, W / 2, yy + 27, 13, CL.am, 'center');
 }
}

function frame(t) {
 raf = 0;
 if (!shown || D.hidden) return;
 const dt = Math.min(0.1, Math.max(0, (t - lastT) / 1000));
 lastT = t;
 update(dt, t);
 render(t);
 raf = requestAnimationFrame(frame);
}
function go() {if (shown && !raf && !D.hidden) {lastT = performance.now(); raf = requestAnimationFrame(frame)}}

new ResizeObserver(() => {resize(); if (shown && W) render(performance.now())}).observe(box);
on(window, 'resize', () => {if (shown) fitBox()});
on(D, 'visibilitychange', () => {if (D.hidden) gEnd(false); else go()});

window.VZ = {
 on(v) {
  if (v == shown) return;
  shown = v;
  if (v) {fitBox(); resize(); go()}
  else {gEnd(false); if (raf) cancelAnimationFrame(raf); raf = 0}
 },
 // for tests and debugging: where the hub is on screen and which way is which
 dbg: () => ({hub: [hubS[0], hubS[1]], rotor: rotS, spin: spinS, sgnR, hdg, W, H, shown, coast: co && Math.round(co.v)})
};
vwDraw();
})();
