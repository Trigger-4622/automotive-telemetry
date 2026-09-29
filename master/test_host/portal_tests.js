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
 * milliseconds on the car's clock, and the phone polling /api/bus?e=1 every
 * ~150 ms, keeping the last payload of each identifier and the master's
 * per-bit flip counters.
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
vm.runInContext(core + '\n;globalThis.__x={teachFind,teachBits,teachPositions,posValue,posLabel,extract,hb};', T);
const { teachFind, posValue, posLabel } = T.__x;

let failures = 0;
function check(ok, what, info) {
    console.log(`${ok ? '  ok  ' : '  FAIL'} ${what}${info ? ' - ' + info : ''}`);
    if (!ok) failures++;
}

/**
 * One teach session. @p steps: [{key, ms}] in order; @p frame(tMs, n, key)
 * returns {id: payload bytes} for the frames on the bus at that moment (called
 * every @p frameMs); polls every @p pollMs keep the latest payloads and flip
 * counters, skipping the first @p skipMs of each step as the page does.
 * @p edges false = a master without flip counters.
 */
function session(steps, frame, { frameMs = 20, pollMs = 150, skipMs = 800, edges = true } = {}) {
    const S = {}, last = {}, flips = {};
    let t = 0, n = 0;
    for (const st of steps) {
        S[st.key] = [];
        const end = t + st.ms;
        let nextPoll = t + pollMs;
        const t0 = t;
        for (; t < end; t += frameMs, n++) {
            const f = frame(t, n, st.key);
            for (const [id, bytes] of Object.entries(f)) {
                const prev = last[id];
                const fl = flips[id] || (flips[id] = new Array(64).fill(0));
                if (prev)
                    for (let b = 0; b < 8; b++) {
                        const x = (prev[b] || 0) ^ (bytes[b] || 0);
                        for (let k = 0; k < 8; k++) if ((x >> k) & 1) fl[8 * b + k] = (fl[8 * b + k] + 1) & 255;
                    }
                last[id] = bytes.slice();
            }
            if (t >= nextPoll) {
                nextPoll += pollMs;
                if (t - t0 > skipMs)
                    S[st.key].push(new Map(Object.keys(last).map(id =>
                        [+id, { b: last[id].slice(), e: edges ? flips[id].slice() : null }])));
            }
        }
    }
    return S;
}
const setBit = (b, bit, v) => { if (v) b[bit >> 3] |= 1 << (bit & 7); else b[bit >> 3] &= ~(1 << (bit & 7)); };
const OFF_ON_OFF = [{ key: 'A', ms: 4000 }, { key: 'B', ms: 4000 }, { key: 'C', ms: 4000 }];

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
    const r = teachFind('bit', session(OFF_ON_OFF, bodyFrame()), ['A', 'B', 'C']);
    const hit = r.find(x => x.id === 0x351 && x.s === 40);
    check(!!hit, 'the flashing bit is found', JSON.stringify(r.map(x => [x.id.toString(16), x.s, +x.score.toFixed(2)])));
    check(hit && r[0] === hit, 'and ranked first');
    check(hit && hit.blink >= 3, 'as flashing, with its flashes counted', hit && `flashed ${hit.blink}`);
    check(hit && !hit.inv, 'not inverted');
    check(!r.some(x => x.id === 0x351 && x.s >= 56), 'the rolling counter is not taken for it');
    check(!r.some(x => x.id === 0x231), 'nor anything in the engine frame');
}

console.log('--- flashing, polled exactly once a flash (every payload the phone sees is dark)');
{
    // Polls every 714 ms land in the dark half of every flash: the payloads
    // show the lamp off throughout, and only the master's counters see it.
    const S = session(OFF_ON_OFF, (t, n, step) => {
        const f = bodyFrame(714)(t, n, step);
        return f;
    }, { pollMs: 714, frameMs: 7 });
    const B = S.B.map(p => p.get(0x351).b);
    check(B.every(b => ((b[5] >> 0) & 1) === 0), 'the snapshots never catch the lamp lit');
    const r = teachFind('bit', S, ['A', 'B', 'C']);
    check(r.some(x => x.id === 0x351 && x.s === 40 && x.blink), 'the flip counters still find it');
}

console.log('--- flashing, from a master without flip counters');
{
    const r = teachFind('bit', session(OFF_ON_OFF, bodyFrame(), { edges: false }), ['A', 'B', 'C']);
    check(r.some(x => x.id === 0x351 && x.s === 40 && x.blink), 'found from the payloads alone');
}

console.log('--- the lever itself: a steady bit, as before');
{
    const S = session(OFF_ON_OFF, (t, n, step) => {
        const b = [0, 0, 0, 0, 0, 0x08, 0, n & 0x0F];
        setBit(b, 44, step === 'B');
        return { 0x351: b };
    });
    const r = teachFind('bit', S, ['A', 'B', 'C']);
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
    check(teachFind('bit', S, ['A', 'B', 'C']).length === 0, 'nothing reported');
}

/*
 * The gear lever. The transmission's frame 0x252 carries the position in bits
 * 52-54 (byte 6): P 0, R 1, N 2, D 4 - bit 55 beside it is a constant, and
 * byte 7 a rolling counter. Five seconds per position, the first 1.5 s not
 * sampled (the lever is moving).
 */
const PRND = ['P', 'R', 'N', 'D'];
const posSteps = toks => toks.map((t, i) => ({ key: 'p' + i, ms: 5000 }));
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
    const S = session(posSteps(PRND), leverFrame([0, 1, 2, 4]), { skipMs: 1500 });
    const r = teachFind('pos', S, PRND.map((t, i) => 'p' + i), PRND);
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
    }, { skipMs: 1500 });
    const r = teachFind('pos', S, PRND.map((t, i) => 'p' + i), PRND);
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
    const r = teachFind('pos', session(posSteps(PRND), spread, { skipMs: 1500 }), PRND.map((t, i) => 'p' + i), PRND);
    check(r.length === 1 && r[0].codes, 'one combination, not one switch per position', `${r.length} results`);
    const f = r[0];
    check(f && JSON.stringify(f.bits) === '[8,15,33,54]', 'bits 8, 15, 33 and 54', f && JSON.stringify(f.bits));
    // code bit i = bits[i]: P 8+15 = 0b0011, R 15+54 = 0b1010, N none, D 15+33 = 0b0110
    check(f && JSON.stringify(f.codes) === '[3,10,0,6]', 'each position its own pattern', f && JSON.stringify(f.codes));
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
    }, { skipMs: 1500 });
    const r = teachFind('pos', S, PRNDP.map((t, i) => 'p' + i), PRNDP);
    const f = r[0];
    check(f && JSON.stringify(f.bits) === '[52,53,54]', 'the timer bit is left out', f && JSON.stringify(f.bits));
    check(f && JSON.stringify(f.names) === '["P","R","N","D"]' && JSON.stringify(f.codes) === '[0,1,2,4]',
          'four positions, P once in the table', f && JSON.stringify([f.names, f.codes]));
    // Without the second P it would have been taken in:
    const S4 = session(posSteps(PRND), (t, n, step) => {
        const f2 = leverFrame([0, 1, 2, 4])(t, n, step);
        setBit(f2[0x252], 60, t > 12000);
        return f2;
    }, { skipMs: 1500 });
    const r4 = teachFind('pos', S4, PRND.map((t, i) => 'p' + i), PRND);
    check(r4[0] && r4[0].bits.includes(60), '(one pass alone cannot tell)', r4[0] && JSON.stringify(r4[0].bits));
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
    }, { skipMs: 1500 });
    const r = teachFind('pos', S, PRND.map((t, i) => 'p' + i), PRND);
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
    });
    const r = teachFind('val', S, ['A', 'B']);
    check(r[0] && r[0].id === 0x331 && r[0].s === 0 && r[0].l === 16 && !r[0].be, 'bytes 0-1, Intel, found first',
          JSON.stringify(r[0]));
}

console.log(failures ? `FAILED: ${failures} check(s)` : 'all portal checks passed');
process.exit(failures ? 1 : 0);
