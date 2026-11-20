// 탁구공 검출·궤적 분석 (브라우저와 Node에서 같이 쓰는 순수 함수)
//
// 1) detectCandidates: 앞·뒤 프레임과 비교해 "이 프레임에만 있는 작고 밝은 물체"를 찾는다 (3프레임 차분)
// 2) buildSegments:   거의 등속으로 움직이는 후보 3개를 씨앗으로 잡고 앞뒤로 늘려 궤적 조각을 만든다
// 3) analyzeRally:    궤적에서 스트로크(내 쪽에서 방향 전환)와 바운드(테이블 위에서 아래→위 전환)를 찾는다

export const PROC_W = 960;   // 분석용 프레임 폭 (높이는 영상 비율대로)

export function luminance(rgba, n) {
  const Y = new Uint8Array(n);
  for (let i = 0, j = 0; i < n; i++, j += 4) Y[i] = (rgba[j] * 77 + rgba[j + 1] * 150 + rgba[j + 2] * 29) >> 8;
  return Y;
}

// prevY, curY, nextY: 밝기 배열. 반환: [{x, y, a}] (분석 해상도 좌표)
export function detectCandidates(prevY, curY, nextY, W, H, opt = {}) {
  const T = opt.diff ?? 20, minY = opt.minY ?? 90, maxA = opt.maxArea ?? 600, maxS = opt.maxSize ?? 50;
  const n = W * H, mask = new Uint8Array(n);
  for (let i = 0; i < n; i++) {
    const y = curY[i];
    if (y > minY && y - prevY[i] > T && y - nextY[i] > T) mask[i] = 1;
  }
  const out = [], stack = [];
  for (let i = 0; i < n; i++) {
    if (mask[i] !== 1) continue;
    let a = 0, sx = 0, sy = 0, x0 = W, x1 = -1, y0 = H, y1 = -1;
    mask[i] = 2; stack.push(i);
    while (stack.length) {
      const k = stack.pop(), x = k % W, y = (k - x) / W;
      a++; sx += x; sy += y;
      if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y;
      for (let dy = -1; dy <= 1; dy++) {
        const yy = y + dy; if (yy < 0 || yy >= H) continue;
        for (let dx = -1; dx <= 1; dx++) {
          const xx = x + dx; if (xx < 0 || xx >= W) continue;
          const kk = yy * W + xx; if (mask[kk] === 1) { mask[kk] = 2; stack.push(kk); }
        }
      }
    }
    const bw = x1 - x0 + 1, bh = y1 - y0 + 1;
    if (a >= 4 && a <= maxA && bw <= maxS && bh <= maxS && a / (bw * bh) > 0.25) out.push({ x: sx / a, y: sy / a, a });
  }
  // 사람이 크게 움직이거나 카메라가 흔들린 프레임은 후보가 너무 많다 → 버림
  return out.length > (opt.maxCands ?? 30) ? [] : out;
}

// frames: [{f, t, c: [{x, y, a}]}] (f = 프레임 번호, 연속이 아니어도 됨)
// scale: 탁구대 근처에서 1m가 몇 px인지
export function buildSegments(frames, scale, fps) {
  const byF = new Map(frames.map(fr => [fr.f, fr]));
  const used = new Set(), key = (f, i) => f * 1000 + i;
  const vmin = 1.2 / fps * scale, vmax = 28 / fps * scale;   // 공 속도 1.2 ~ 28 m/s
  const tolOf = (v, gap) => 6 + 0.25 * v * gap + 1.5 * gap * gap;

  const seeds = [];
  for (const A of frames) {
    const B = byF.get(A.f + 1), C = byF.get(A.f + 2); if (!B || !C) continue;
    A.c.forEach((a, ia) => B.c.forEach((b, ib) => {
      const vx = b.x - a.x, vy = b.y - a.y, v = Math.hypot(vx, vy);
      if (v < vmin || v > vmax) return;
      C.c.forEach((c, ic) => {
        const e = Math.hypot(c.x - (b.x + vx), c.y - (b.y + vy));
        const v2 = Math.hypot(c.x - b.x, c.y - b.y);
        if (e < tolOf(v, 1) && v2 > vmin) seeds.push({ e: e / v, pts: [[A.f, ia], [B.f, ib], [C.f, ic]] });
      });
    }));
  }
  seeds.sort((p, q) => p.e - q.e);

  const P = (f, i) => byF.get(f).c[i];
  const segs = [];
  for (const s of seeds) {
    if (s.pts.some(([f, i]) => used.has(key(f, i)))) continue;
    const pts = s.pts.map(([f, i]) => ({ f, i, ...P(f, i) }));
    // 앞으로, 뒤로 늘리기
    for (const dir of [1, -1]) {
      for (;;) {
        const arr = dir > 0 ? pts : pts.slice().reverse();
        const p1 = arr[arr.length - 1], p0 = arr[arr.length - 2];
        const df = p1.f - p0.f, vx = (p1.x - p0.x) / df, vy = (p1.y - p0.y) / df, v = Math.hypot(vx, vy);
        let best = null;
        for (let gap = 1; gap <= 3 && !best; gap++) {
          const f = p1.f + dir * gap, F = byF.get(f); if (!F) continue;
          const px = p1.x + vx * gap, py = p1.y + vy * gap, tol = tolOf(v, gap);
          F.c.forEach((c, i) => {
            if (used.has(key(f, i)) || pts.some(p => p.f === f)) return;
            const e = Math.hypot(c.x - px, c.y - py);
            if (e < tol && (!best || e < best.e)) best = { e, f, i, ...c };
          });
        }
        if (!best) break;
        delete best.e;
        if (dir > 0) pts.push(best); else pts.unshift(best);
      }
    }
    if (pts.length < 4) continue;
    pts.forEach(p => used.add(key(p.f, p.i)));
    segs.push({ pts });
  }
  segs.sort((a, b) => a.pts[0].f - b.pts[0].f);
  return { segs, used, byF };
}

function lineIntersect(a0, a1, b0, b1) {
  const d1x = a1.x - a0.x, d1y = a1.y - a0.y, d2x = b1.x - b0.x, d2y = b1.y - b0.y;
  const den = d1x * d2y - d1y * d2x;
  if (Math.abs(den) < 1e-6) return null;
  const s = ((b0.x - a0.x) * d2y - (b0.y - a0.y) * d2x) / den;
  return { x: a0.x + d1x * s, y: a0.y + d1y * s, s };
}

// geo: { axis: [ax, ay] (내 쪽 끝선 → 네트, 화면 단위벡터), origin: {x,y} (내 끝선 중앙, 화면),
//        sNet: 끝선→네트 화면 거리(px), scale: px/m, toTable(x,y) → [u, v] (m), W, L }
export function analyzeRally(frames, geo, fps) {
  const { segs, used, byF } = buildSegments(frames, geo.scale, fps);
  const S = p => (p.x - geo.origin.x) * geo.axis[0] + (p.y - geo.origin.y) * geo.axis[1];
  const vmin = 1.2 / fps * geo.scale, vFast = 1.5 / fps * geo.scale;   // 네트 방향 속도 기준

  for (const g of segs) {
    const a = g.pts[0], b = g.pts[g.pts.length - 1];
    g.f0 = a.f; g.f1 = b.f; g.s0 = S(a); g.s1 = S(b); g.ds = g.s1 - g.s0;
    g.dir = g.ds > 0 ? 1 : -1;                 // +1: 상대 쪽으로 (내가 친 공), -1: 내 쪽으로 (오는 공)
    g.vs = Math.abs(g.ds) / Math.max(1, g.f1 - g.f0);
    g.fast = g.vs >= vFast;
  }

  // ---- 오는 공: 네트 쪽에서 출발해 내 쪽으로 빠르게 오는 궤적 (0.5초 안의 조각은 같은 공)
  const incoming = [];
  for (const g of segs) {
    if (g.dir > 0 || !g.fast || -g.ds < 0.3 * geo.scale || g.s0 < 0.3 * geo.sNet) continue;
    const last = incoming[incoming.length - 1];
    if (last && g.f0 - last.f1 <= 0.5 * fps) { last.f1 = Math.max(last.f1, g.f1); last.segs.push(g); continue; }
    incoming.push({ f: g.f0, f1: g.f1, segs: [g] });
  }

  // ---- 스트로크: 오는 공 다음에 공이 처음으로 상대 쪽으로 움직이기 시작한 순간
  const outs = segs.filter(g => g.dir > 0 && g.ds >= 0.12 * geo.scale && g.s0 < geo.sNet && g.vs >= vmin);
  const reachFrom = (f0) => Math.max(...outs.filter(o => o.f0 >= f0 && o.f0 <= f0 + 0.5 * fps).map(o => o.s1));
  const strokes = [];
  incoming.forEach((ball, k) => {
    const until = Math.min(ball.f1 + 1.2 * fps, incoming[k + 1] ? incoming[k + 1].f : Infinity);
    const first = outs.filter(o => o.f0 >= Math.max(ball.f + 2, ball.f1 - 6) && o.f0 < until).sort((a, b) => a.f0 - b.f0)[0];
    if (!first) { ball.missed = true; return; }
    const reach = reachFrom(first.f0);
    const p = first.pts[0];
    strokes.push({ f: first.f0, x: p.x, y: p.y, ok: reach >= 0.85 * geo.sNet, ball });
    ball.stroke = strokes[strokes.length - 1];
  });
  // 서브처럼 오는 공 없이 친 공: 빠르고 네트까지 가는 것만
  for (const o of outs) {
    if (!o.fast || o.s1 < 0.85 * geo.sNet) continue;
    if (strokes.some(st => o.f0 >= st.f - 0.6 * fps && o.f0 <= st.f + 0.6 * fps)) continue;
    const p = o.pts[0];
    strokes.push({ f: o.f0, x: p.x, y: p.y, ok: true, serve: true });
  }
  strokes.sort((a, b) => a.f - b.f);

  // ---- 바운드: 빠른 공이 내려오다 탁구대 위에서 다시 올라가는 지점
  const bounces = [];
  const addBounce = (pIn0, pIn1, pOut0, pOut1, f, dir) => {
    // 들어오는 선과 튀어 나가는 선의 교점. 마지막 관측점에서 1.5프레임 이동 거리 안쪽만 믿는다
    const X = lineIntersect(pIn0, pIn1, pOut0, pOut1);
    const q = X && X.s > 0.8 && X.s < 2.6 ? X
      : { x: pIn1.x + (pIn1.x - pIn0.x) * 0.5, y: pIn1.y + (pIn1.y - pIn0.y) * 0.5 };
    const [u, v] = geo.toTable(q.x, q.y);
    if (!(u > -0.04 && u < geo.W + 0.04 && v > -0.04 && v < geo.L + 0.04)) return;
    if (bounces.some(b => Math.abs(b.f - f) < 4)) return;
    bounces.push({ f, x: q.x, y: q.y, u, v, dir });
  };
  for (const g of segs) {
    if (!g.fast || Math.abs(g.ds) < 0.25 * geo.scale) continue;
    const p = g.pts;
    for (let i = 2; i < p.length - 2; i++) {
      const vyIn = (p[i].y - p[i - 2].y) / (p[i].f - p[i - 2].f), vyOut = (p[i + 2].y - p[i].y) / (p[i + 2].f - p[i].f);
      if (vyIn > 1.5 && vyOut < -1.5 && p[i].y >= p[i - 1].y && p[i].y >= p[i + 1].y) addBounce(p[i - 2], p[i - 1], p[i + 1], p[i + 2], p[i].f, g.dir);
    }
    // 내려가며 끝난 조각: 바로 뒤에서 같은 방향으로 올라가는 후보 2개를 찾는다 (바운드 직후는 끊기기 쉬움)
    const n = p.length, e1 = p[n - 1], e0 = p[n - 2];
    const vx = (e1.x - e0.x) / (e1.f - e0.f), vy = (e1.y - e0.y) / (e1.f - e0.f);
    if (vy <= 1 || Math.abs(vx) < 2) continue;
    const ax = Math.abs(vx);
    let best = null;
    for (let g1 = 1; g1 <= 3; g1++) {
      const F1 = byF.get(e1.f + g1); if (!F1) continue;
      for (const c1 of F1.c) {
        const k1 = (c1.x - e1.x) / (vx * g1);
        if (k1 < 0.3 || k1 > 1.6) continue;
        // 튀어 오른 공은 수평 속도보다 가파르게 올라가지 못한다
        if (c1.y > e1.y + Math.abs(vy) * g1 + 8 || c1.y < e1.y - 1.2 * ax * g1 - 12) continue;
        for (let g2 = 1; g2 <= 2; g2++) {
          const F2 = byF.get(e1.f + g1 + g2); if (!F2) continue;
          for (const c2 of F2.c) {
            const k2 = (c2.x - c1.x) / (vx * g2);
            if (k2 < 0.3 || k2 > 1.6 || c2.y > c1.y - 1 || c2.y < c1.y - 1.2 * ax * g2 - 12) continue;
            const score = Math.abs(k1 - k2) + Math.abs(k1 - 0.8) * 0.5 + (g1 - 1) * 0.15 + (g2 - 1) * 0.15;
            if (!best || score < best.score) best = { score, pts: [{ f: e1.f + g1, ...c1 }, { f: e1.f + g1 + g2, ...c2 }] };
          }
        }
      }
    }
    if (best) { addBounce(e0, e1, best.pts[0], best.pts[1], e1.f + 1, g.dir); g.tail = best.pts; }
  }
  bounces.sort((a, b) => a.f - b.f);
  for (const b of bounces) b.side = b.v < geo.L / 2 ? 'mine' : 'theirs';
  return { segs: segs.filter(g => g.fast || g.tail), strokes, bounces, incoming, usedCount: used.size };
}

// 화면 표시용: 프레임별 공 위치 (궤적 조각에 속한 점만)
export function ballTrack(result) {
  const m = new Map();
  for (const g of result.segs) for (const p of [...g.pts, ...(g.tail || [])]) m.set(p.f, { x: p.x, y: p.y });
  return m;
}
