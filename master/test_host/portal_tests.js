#!/usr/bin/env node
/*
 * The portal's teach-by-doing analysis, run in Node on made-up bus traffic.
 *
 *     node master/test_host/portal_tests.js
 *
 * run.py runs it after the simulator scenarios (as "portal_teach"). The code
 * under test is the page's own: every block of tools/portal_page.html marked
 * teach-core is pure - payloads in, candidates out - and is evaluated here
 * unchanged, so what passes here is what the phone runs.
 *
 * The traffic is made the way the page sees it: a frame every few
 * milliseconds on the car's clock, and the phone asking /api/bus?e=1 over and
 * over - each answer a snapshot of the last payload of each identifier and the
 * master's per-bit flip counters, taken while the master serves it - and
 * filing each answer with the page's own teachSlot.
 */
'use strict';
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const page = fs.readFileSync(path.join(__dirname, '..', 'tools', 'portal_page.html'), 'utf8');
const core = [...page.matchAll(/\/\*<teach-core>\*\/([\s\S]*?)\/\*<\/teach-core>\*\//g)].map(m => m[1]).join('\n');
if (!core) { console.log('FAIL: no teach-core block in portal_page.html'); process.exit(1); }
const T = {};
vm.createContext(T);
vm.runInContext(core + '\n;globalThis.__x={teachFind,teachBits,teachPositions,teachTurns,teachSlot,posValue,posLabel,extract,hb,TEACH_MIN_READS};', T);
const { teachFind, teachSlot, posValue, posLabel, TEACH_MIN_READS } = T.__x;

let failures = 0;
function check(ok, what, info) {
    console.log(`${ok ? '  ok  ' : '  FAIL'} ${what}${info ? ' - ' + info : ''}`);
    if (!ok) failures++;
}

/** A tiny deterministic random source, so every run sees the same traffic. */
function rng(seed) {
    let s = seed >>> 0;
    return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
}

/**
 * One teach session. @p steps: [{key, ms}] in order. @p frame(tMs, n, key)
 * returns {id: payload bytes} for the frames on the bus at that moment (called
 * every @p frameMs), @p key being the step the driver is acting on - @p react
 * ms behind the page's. The phone asks as teachStart does: a request, its
 * answer @p latency(k) ms later (a snapshot taken half-way, while the master
 * serves it; past 1500 ms the page gives up on it), the next one no sooner
 * than @p pollMs after the last, from @p start on. Each answer is filed with
 * the page's teachSlot, which drops the first @p skipMs of every step and
 * anything answered after its step ended. @p edges false = a master without
 * flip counters.
 */
function session(steps, frame, { frameMs = 20, pollMs = 150, skipMs = 1500, edges = true,
                                 latency = () => 40, react = 0, start = 0 } = {}) {
    const durs = steps.map(s => s.ms), total = durs.reduce((a, b) => a + b, 0);
    const keyAt = t => { let s = 0; for (const st of steps) { s += st.ms; if (t < s) return st.key; } return steps[steps.length - 1].key; };
    const S = {};
    steps.forEach(s => S[s.key] = []);
    const last = {}, flips = {};
    let n = 0, k = 0, tq = start, lat = latency(0), snap = null;
    for (let t = 0; t < total; t++) {
        if (t % frameMs === 0) {
            const f = frame(t, n++, keyAt(Math.max(0, t - react)));
            for (const [id, bytes] of Object.entries(f)) {
                const prev = last[id];
                const fl = flips[id] || (flips[id] = new Array(64).fill(0));
                if (prev)
                    for (let b = 0; b < 8; b++) {
                        const x = (prev[b] || 0) ^ (bytes[b] || 0);
                        for (let q = 0; q < 8; q++) if ((x >> q) & 1) fl[8 * b + q] = (fl[8 * b + q] + 1) & 255;
                    }
                last[id] = bytes.slice();
            }
        }
        if (t < tq) continue;
        if (lat <= 1500 && snap === null && t >= tq + lat / 2)
            snap = new Map(Object.keys(last).map(id => [+id, { b: last[id].slice(), e: edges ? flips[id].slice() : null }]));
        const done = tq + Math.min(lat, 1500);
        if (t >= done) {
            if (lat <= 1500 && snap) {
                const i = teachSlot(durs, skipMs, tq, done);
                if (i >= 0) S[steps[i].key].push(snap);
            }
            tq = Math.max(tq + pollMs, done);
            lat = latency(++k);
            snap = null;
        }
    }
    return S;
}
const setBit = (b, bit, v) => { if (v) b[bit >> 3] |= 1 << (bit & 7); else b[bit >> 3] &= ~(1 << (bit & 7)); };
/* The page's steps for On / off. */
const OFF_ON_OFF = [{ key: 'A', ms: 5000 }, { key: 'B', ms: 6000 }, { key: 'C', ms: 5000 }];
const ABC = ['A', 'B', 'C'];

console.log('--- which step a reading counts for');
{
    const d = [5000, 6000, 5000];
    check(teachSlot(d, 1500, 1400, 1450) === -1, 'asked in the first 1.5 s of a step: not read');
    check(teachSlot(d, 1500, 1500, 1600) === 0, 'asked and answered inside the step: that step');
    check(teachSlot(d, 1500, 4900, 5100) === -1, 'answered after the step ended: dropped, not filed under the next');
    check(teachSlot(d, 1500, 6600, 6700) === 1, 'the next step once its first 1.5 s have gone');
    check(teachSlot(d, 1500, 7000, 11000) === 1 && teachSlot(d, 1500, 7000, 11001) === -1, 'up to the last millisecond of it');
    check(teachSlot(d, 1500, 16000, 16010) === -1, 'after the last step: nothing');
}

/*
 * The car's body frame 0x351: light switch (bit 43), high beam (bit 42), and
 * a rolling counter in the low nibble of byte 7 that ticks every frame - the
 * kind of bit that must never be taken for a switch. The left turn signal is
 * the flasher's own output on bit 40: on about 0.35 s, off 0.35 s, only while
 * the lever is on (step B).
 */
function bodyFrame(flashMs = 714) {
    return (t, n, step) => {
        const b = [0x10, 0, 0, 0, 0, 0x08, 0, n & 0x0F];
        const lampOn = step === 'B' && (t % flashMs) < flashMs / 2;
        setBit(b, 40, lampOn);
        return { 0x351: b, 0x231: [n & 0xFF, 0x03, 0, 0, 0, 0, 0, 0] };
    };
}

console.log('--- turn signal that flashes');
{
    const r = teachFind('bit', session(OFF_ON_OFF, bodyFrame()), ABC);
    const hit = r.find(x => x.id === 0x351 && x.s === 40);
    check(!!hit, 'the flashing bit is found', JSON.stringify(r.map(x => [x.id.toString(16), x.s, +x.score.toFixed(2)])));
    check(hit && r[0] === hit, 'and ranked first');
    check(hit && hit.blink >= 3, 'as flashing, with its flashes counted', hit && `flashed ${hit.blink}`);
    check(hit && !hit.inv, 'not inverted');
    check(!r.some(x => x.id === 0x351 && x.s >= 56), 'the rolling counter is not taken for it');
    check(!r.some(x => x.id === 0x231), 'nor anything in the engine frame');
}

console.log('--- flashing, a driver who is slow to switch it on and off');
{
    // 2 s late with each change: the lamp still flashes into the first half
    // second of the OFF step's reading window.
    const r = teachFind('bit', session(OFF_ON_OFF, bodyFrame(), { react: 2000 }), ABC);
    check(r.some(x => x.id === 0x351 && x.s === 40 && x.blink), 'still found', JSON.stringify(r.map(x => [x.id.toString(16), x.s])));
}

console.log('--- flashing, a master that answers slowly and unevenly');
{
    // 60 ms to 1.2 s per answer: fewer readings, still enough.
    let found = 0, thin = 0;
    for (let seed = 1; seed <= 10; seed++) {
        const rnd = rng(seed);
        const S = session(OFF_ON_OFF, bodyFrame(), { latency: () => 60 + Math.floor(rnd() * 1140) });
        if (ABC.some(k => S[k].length < TEACH_MIN_READS)) { thin++; continue; }
        const r = teachFind('bit', S, ABC);
        if (r[0] && r[0].id === 0x351 && r[0].s === 40) found++;
    }
    check(thin === 0 && found === 10, 'the lamp\'s bit is found first, every time', `${found} of 10, ${thin} too thin`);
}

console.log('--- a master slower still: never a wrong answer without a warning');
{
    // Up to 2 s per answer, a quarter of them given up on: most runs end with
    // a step under TEACH_MIN_READS readings, which the page reports as too
    // slow. Two readings in a step can make a slow counter bit look like a
    // switch - which is why it says so rather than trusting the answer.
    let wrong = 0, warned = 0, right = 0;
    for (let seed = 1; seed <= 30; seed++) {
        const rnd = rng(seed);
        const S = session(OFF_ON_OFF, bodyFrame(), { latency: () => 60 + Math.floor(rnd() * 1940) });
        if (ABC.some(k => S[k].length < TEACH_MIN_READS)) { warned++; continue; }
        const r = teachFind('bit', S, ABC);
        if (r[0] && r[0].id === 0x351 && r[0].s === 40) right++; else wrong++;
    }
    check(wrong === 0, 'every run is either right or reported as too slow', `${right} right, ${warned} reported, ${wrong} wrong`);
}

console.log('--- flashing, polled exactly once a flash (every payload the phone sees is dark)');
{
    // One request per flash, each answer taken in the dark half: the payloads
    // show the lamp off throughout, and only the master's counters see it.
    const S = session(OFF_ON_OFF, bodyFrame(714), { pollMs: 714, frameMs: 7, start: 400 });
    const B = S.B.map(p => p.get(0x351).b);
    check(B.length >= 3 && B.every(b => ((b[5] >> 0) & 1) === 0), 'the snapshots never catch the lamp lit', `${B.length} snapshots`);
    const r = teachFind('bit', S, ABC);
    check(r.some(x => x.id === 0x351 && x.s === 40 && x.blink), 'the flip counters still find it');
}

console.log('--- flashing, from a master without flip counters');
{
    const r = teachFind('bit', session(OFF_ON_OFF, bodyFrame(), { edges: false }), ABC);
    check(r.some(x => x.id === 0x351 && x.s === 40 && x.blink), 'found from the payloads alone');
}

console.log('--- the lever itself: a steady bit, as before');
{
    const S = session(OFF_ON_OFF, (t, n, step) => {
        const b = [0, 0, 0, 0, 0, 0x08, 0, n & 0x0F];
        setBit(b, 44, step === 'B');
        return { 0x351: b };
    });
    const r = teachFind('bit', S, ABC);
    const hit = r.find(x => x.id === 0x351 && x.s === 44);
    check(hit && !hit.blink && hit.score >= 0.99, 'a steady switch is found as a switch', JSON.stringify(hit));
    check(r.length === 1, 'and nothing else', `${r.length} results`);
}

console.log('--- a bit that flips all the time is not a switch');
{
    const S = session(OFF_ON_OFF, (t, n) => {
        const b = [0, 0, 0, 0, 0, 0, 0, 0];
        setBit(b, 3, (t % 500) < 250);          // a 2 Hz status bit, on and off in every step
        return { 0x4B1: b };
    });
    check(teachFind('bit', S, ABC).length === 0, 'nothing reported');
}

/*
 * The indicators in one run, as the page asks: off, left, off, right, off,
 * hazard lights, off. Frame 0x282 carries the left lamp (bit 12) and the right
 * lamp (bit 13) - the flasher's outputs, lit 0.35 s in every 0.7 s - and bit
 * 20, which flashes for either side (the tick-tock relay). Frame 0x3B1 has the
 * stalk, held left (bit 1) or right (bit 2) while it is, and the hazard
 * switch (bit 8). Byte 7 of 0x282 is a rolling counter, bit 30 a 2 Hz status
 * bit that never stops.
 */
const TURN_STEPS = [{ key: 'A', ms: 5000 }, { key: 'L', ms: 6000 }, { key: 'B', ms: 5000 }, { key: 'R', ms: 6000 },
                    { key: 'C', ms: 5000 }, { key: 'H', ms: 6000 }, { key: 'D', ms: 5000 }];
function indicatorFrames({ lamps = true, flashMs = 700 } = {}) {
    return (t, n, step) => {
        const lit = (t % flashMs) < flashMs / 2;
        const left = step === 'L' || step === 'H', right = step === 'R' || step === 'H';
        const a = [0, 0, 0, 0, 0, 0, 0, n & 0xFF], s = [0, 0, 0, 0, 0, 0, 0, 0];
        if (lamps) { setBit(a, 12, left && lit); setBit(a, 13, right && lit); }
        setBit(a, 20, (left || right) && lit);
        setBit(a, 30, (t % 500) < 250);
        setBit(s, 1, step === 'L');
        setBit(s, 2, step === 'R');
        setBit(s, 8, step === 'H');
        return { 0x282: a, 0x3B1: s };
    };
}
const role = (r, k) => r.filter(x => x.role === k);

console.log('--- left, right and hazard lights in one run');
{
    const r = teachFind('turn', session(TURN_STEPS, indicatorFrames()));
    const L = role(r, 'L'), R = role(r, 'R'), H = role(r, 'H'), both = role(r, 'both');
    check(L[0] && L[0].id === 0x282 && L[0].s === 12 && L[0].how === 'flash' && L[0].metric === 0x1201,
          'left: the left lamp, flashing, as the left turn signal', JSON.stringify(L[0]));
    check(R[0] && R[0].id === 0x282 && R[0].s === 13 && R[0].how === 'flash' && R[0].metric === 0x1202,
          'right: the right lamp, as the right turn signal', JSON.stringify(R[0]));
    check(H[0] && H[0].id === 0x3B1 && H[0].s === 8 && H[0].how === 'held' && H[0].metric === 0x120C,
          'hazard lights: their switch', JSON.stringify(H[0]));
    check(L.some(x => x.id === 0x3B1 && x.s === 1 && x.how === 'held'), 'the stalk held left comes second for the left');
    check(R.some(x => x.id === 0x3B1 && x.s === 2 && x.how === 'held'), '...and held right for the right');
    check(both.length === 1 && both[0].s === 20 && both[0].metric === undefined,
          'the bit that flashes for either side is reported, and not offered for either', JSON.stringify(both));
    check(!r.some(x => x.id === 0x282 && (x.s === 30 || x.s >= 56)), 'no counter, no status bit');
    const order = r.map(x => x.role).join('');
    check(/^L+R+H+(both)*$/.test(order), 'grouped left, right, hazard, then either', order);
}

console.log('--- the same run with a slow driver and a slow, uneven master');
{
    const rnd = rng(11);
    const S = session(TURN_STEPS, indicatorFrames(), { react: 1300, latency: () => 60 + Math.floor(rnd() * 1300) });
    const r = teachFind('turn', S);
    check(role(r, 'L')[0] && role(r, 'L')[0].s === 12, 'left still the left lamp', JSON.stringify(role(r, 'L')[0]));
    check(role(r, 'R')[0] && role(r, 'R')[0].s === 13, 'right still the right lamp', JSON.stringify(role(r, 'R')[0]));
    check(role(r, 'H')[0] && role(r, 'H')[0].s === 8, 'hazard still its switch', JSON.stringify(role(r, 'H')[0]));
}

console.log('--- a car that reports only the stalk, the switch and one flasher for both sides');
{
    const r = teachFind('turn', session(TURN_STEPS, indicatorFrames({ lamps: false })));
    check(role(r, 'L')[0] && role(r, 'L')[0].id === 0x3B1 && role(r, 'L')[0].s === 1, 'left: the stalk (the displays flash it)',
          JSON.stringify(role(r, 'L')[0]));
    check(role(r, 'R')[0] && role(r, 'R')[0].s === 2, 'right: the stalk');
    check(role(r, 'both').length === 1 && !role(r, 'L').some(x => x.s === 20), 'the shared flasher is never taken for a side');
}

console.log('--- the indicators from a master without flip counters');
{
    const r = teachFind('turn', session(TURN_STEPS, indicatorFrames(), { edges: false }));
    check(role(r, 'L')[0] && role(r, 'L')[0].s === 12 && role(r, 'R')[0] && role(r, 'R')[0].s === 13,
          'left and right lamps from the payloads alone');
}

/*
 * The gear lever. The transmission's frame 0x252 carries the position in bits
 * 52-54 (byte 6): P 0, R 1, N 2, D 4 - bit 55 beside it is a constant, and
 * byte 7 a rolling counter. Five seconds per position, the first 1.5 s not
 * read (the lever is moving).
 */
const PRND = ['P', 'R', 'N', 'D'];
const posSteps = toks => toks.map((t, i) => ({ key: 'p' + i, ms: 5000 }));
const posKeys = toks => toks.map((t, i) => 'p' + i);
function leverFrame(code) {
    return (t, n, step) => {
        const j = +step.slice(1);
        const b = [0x40, 0x80, 0, 0, 0, 0, 0x80, n & 0xFF];
        b[6] |= code[j] << 4;
        return { 0x252: b, 0x351: [0, 0, 0, 0, 0, 0x08, 0, n & 0x0F] };
    };
}

console.log('--- gear lever positions in one field');
{
    const S = session(posSteps(PRND), leverFrame([0, 1, 2, 4]));
    const r = teachFind('pos', S, posKeys(PRND), PRND);
    const f = r[0];
    check(f && f.id === 0x252 && f.codes, 'the combination is found first', JSON.stringify(r[0]));
    check(f && JSON.stringify(f.bits) === '[52,53,54]', 'bits 52-54: every bit that moved, and no more', f && JSON.stringify(f.bits));
    check(f && JSON.stringify(f.codes) === '[0,1,2,4]', 'with each position\'s code', f && JSON.stringify(f.codes));
    const map = f ? f.codes.map((c, g) => [c, posValue(0x100F, f.names[g], g)]) : [];
    check(JSON.stringify(map) === '[[0,80],[1,82],[2,78],[4,68]]', 'the table it saves: code to the letter\'s code', JSON.stringify(map));
    check(map.map(p => posLabel(0x100F, p[1])).join('') === 'PRND', 'which reads back as P R N D');
    check(r.length === 1, 'and it is the only answer: nothing per position, nothing from the body frame', `${r.length} results`);
}

console.log('--- all in one byte, scattered, beside bits that do other things');
{
    // Byte 6: P bit 48, R bit 50, N bit 53, D bit 55. Bit 49 is always set,
    // bit 51 flickers (2 Hz) - neither is part of the lever.
    const S = session(posSteps(PRND), (t, n, step) => {
        const j = +step.slice(1);
        const b = [0, 0, 0, 0, 0, 0, 0x02, n & 0xFF];
        setBit(b, [48, 50, 53, 55][j], 1);
        setBit(b, 51, (t % 500) < 250);
        return { 0x252: b };
    });
    const r = teachFind('pos', S, posKeys(PRND), PRND);
    const f = r[0];
    check(f && JSON.stringify(f.bits) === '[48,50,53,55]', 'the combination is those four bits, not the byte',
          f && JSON.stringify(f.bits));
    check(f && JSON.stringify(f.codes) === '[1,2,4,8]', 'one bit each', f && JSON.stringify(f.codes));
}

console.log('--- the positions spread over several bytes: still one value');
{
    // P on bit 8, R on bit 54, N as bit 15 held LOW only in N, D on bit 33.
    const spread = (t, n, step) => {
        const j = +step.slice(1);
        const b = [0, 0x80, 0, 0, 0, 0, 0, n & 0xFF];
        setBit(b, 8, j === 0);
        setBit(b, 54, j === 1);
        setBit(b, 15, j !== 2);
        setBit(b, 33, j === 3);
        return { 0x252: b };
    };
    const r = teachFind('pos', session(posSteps(PRND), spread), posKeys(PRND), PRND);
    check(r.length === 1 && r[0].codes, 'one combination, not one switch per position', `${r.length} results`);
    const f = r[0];
    check(f && JSON.stringify(f.bits) === '[8,15,33,54]', 'bits 8, 15, 33 and 54', f && JSON.stringify(f.bits));
    // code bit i = bits[i]: P 8+15 = 0b0011, R 15+54 = 0b1010, N none, D 15+33 = 0b0110
    check(f && JSON.stringify(f.codes) === '[3,10,0,6]', 'each position its own pattern', f && JSON.stringify(f.codes));
}

console.log('--- the lever\'s own field, beside other bits that change with it');
{
    /*
     * What the car showed on 2026-09-30 (frame 0x253): a two-bit field in bits
     * 28-29 - P 3, R 2, N 1, D 0 - and, changing with the lever too, bit 9 (set
     * in P and N: the park/neutral switch) and byte 6 (0x1B in P, N and D, 0x40
     * in R). Taught as all eight, one of the others changing while driving
     * leaves a code in no table. The field alone is the lever.
     */
    const PRNDP = ['P', 'R', 'N', 'D', 'P'];
    const S = session(posSteps(PRNDP), (t, n, step) => {
        const lv = PRNDP[+step.slice(1)];
        const b = [0x20, 0, 0x5A, 0, 0, 0, 0, n & 0xFF];
        setBit(b, 9, lv === 'P' || lv === 'N');
        b[3] |= { P: 3, R: 2, N: 1, D: 0 }[lv] << 4;
        b[6] = lv === 'R' ? 0x40 : 0x1B;
        return { 0x253: b };
    });
    const r = teachFind('pos', S, posKeys(PRNDP), PRNDP);
    const f = r[0];
    check(f && f.id === 0x253 && JSON.stringify(f.bits) === '[28,29]', 'bits 28-29 alone', f && JSON.stringify(f.bits));
    check(f && JSON.stringify(f.codes) === '[3,2,1,0]', 'P 3, R 2, N 1, D 0', f && JSON.stringify(f.codes));
    check(r.length === 1, 'one answer', `${r.length} results`);
}

console.log('--- back in P at the end: a bit that only happened to change is dropped');
{
    // Bit 60 flips once, 12 s in (a slow timer): between the first positions it
    // looks like part of the lever. The second visit to P reads it the other
    // way, so it is not.
    const PRNDP = ['P', 'R', 'N', 'D', 'P'];
    const S = session(posSteps(PRNDP), (t, n, step) => {
        const f = leverFrame([0, 1, 2, 4, 0])(t, n, step);
        setBit(f[0x252], 60, t > 12000);
        return f;
    });
    const r = teachFind('pos', S, posKeys(PRNDP), PRNDP);
    const f = r[0];
    check(f && JSON.stringify(f.bits) === '[52,53,54]', 'the timer bit is left out', f && JSON.stringify(f.bits));
    check(f && JSON.stringify(f.names) === '["P","R","N","D"]' && JSON.stringify(f.codes) === '[0,1,2,4]',
          'four positions, P once in the table', f && JSON.stringify([f.names, f.codes]));
    // One pass alone cannot tell the timer bit from the lever's - but the
    // lever's own field (bits 52-54) tells every position apart without it.
    const S4 = session(posSteps(PRND), (t, n, step) => {
        const f2 = leverFrame([0, 1, 2, 4])(t, n, step);
        setBit(f2[0x252], 60, t > 12000);
        return f2;
    });
    const r4 = teachFind('pos', S4, posKeys(PRND), PRND);
    check(r4[0] && JSON.stringify(r4[0].bits) === '[52,53,54]', 'one pass: the field, still without it', r4[0] && JSON.stringify(r4[0].bits));
}

console.log('--- positions in different frames: one switch per position');
{
    // Park only in 0x252 (bit 8), reverse only in 0x3D1 (bit 3): no one frame
    // tells every position apart.
    const S = session(posSteps(PRND), (t, n, step) => {
        const j = +step.slice(1);
        const a = [0, 0, 0, 0, 0, 0, 0, n & 0xFF], c = [0, 0, 0, 0, 0, 0, 0, n & 0xFF];
        setBit(a, 8, j === 0);
        setBit(c, 3, j === 1);
        return { 0x252: a, 0x3D1: c };
    });
    const r = teachFind('pos', S, posKeys(PRND), PRND);
    const as = m => r.find(x => x.metric === m);
    check(!r.some(x => x.codes), 'no combination');
    check(as(0x1209) && as(0x1209).id === 0x252 && as(0x1209).s === 8, 'P: 0x252 bit 8, as Lever in P');
    check(as(0x1208) && as(0x1208).id === 0x3D1 && as(0x1208).s === 3, 'R: 0x3D1 bit 3, as Reverse');
}

console.log('--- position values');
{
    check(['P', 'R', 'N', 'D', 'M', '3'].map((t, i) => posValue(0x100F, t, i)).join() === '80,82,78,68,77,51', 'the lever: letter codes');
    check(['I', 'S', 'S#'].map((t, i) => posValue(0x2039, t, i)).join() === '3,1,2', 'SI-Drive: the ECU\'s numbers');
    check(['N', '1', '2', 'R'].map((t, i) => posValue(0x1005, t, i)).join() === '0,1,2,-1', 'the gear: N 0, R -1');
    check([3, 1, 2].map(v => posLabel(0x2039, v)).join(' ') === 'I S S#', 'and back');
}

console.log('--- a moving value, as before');
{
    const S = session([{ key: 'A', ms: 3000 }, { key: 'B', ms: 9000 }], (t, n, step) => {
        const v = step === 'B' ? Math.round(2000 + 1800 * Math.sin((t - 3000) / 900)) : 2000;
        return { 0x331: [v & 0xFF, v >> 8, 0, 0, 0, 0, 0, n & 0x0F] };
    }, { skipMs: 800 });
    const r = teachFind('val', S, ['A', 'B']);
    check(r[0] && r[0].id === 0x331 && r[0].s === 0 && r[0].l === 16 && !r[0].be, 'bytes 0-1, Intel, found first',
          JSON.stringify(r[0]));
}

console.log(failures ? `FAILED: ${failures} check(s)` : 'all portal checks passed');
process.exit(failures ? 1 : 0);
