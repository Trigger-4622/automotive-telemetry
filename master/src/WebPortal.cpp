/**
 * @file WebPortal.cpp
 * @brief AP, REST API and the embedded configuration page.
 *
 * The page source lives in the project as a readable HTML file and is folded
 * into this file by a build script; what is compiled is the single string
 * below. Four tabs: Dashboard (what is happening, in plain words), Bus (the
 * raw traffic and hand mapping), Sources (where each value comes from, in the
 * order the master prefers them) and Settings.
 */
#include "WebPortal.h"

#include <ArduinoJson.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFi.h>

#include "Learner.h"
#include "MasterConfig.h"
#include "MasterTelemetry.h"
#include "Ssm2.h"

WebPortal Portal;

static WebServer s_server(80);
static DNSServer s_dns;

/*
 * The page lives in flash rather than a filesystem. It is one file, it changes
 * only when the firmware does, and keeping it here means the master needs no
 * `uploadfs` step — see the note in MasterConfig.h about why that matters.
 */
static const char PORTAL_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>CAN Master</title><style>
:root{color-scheme:dark;--bg:#0a0f14;--card:#111922;--card2:#0c131a;--line:#1d2834;--text:#e4ecf3;
 --muted:#83929f;--acc:#00e5ff;--ok:#4ade80;--warn:#fbbf24;--crit:#f87171;
 --raw:#4ade80;--obd:#38bdf8;--ssm:#f59e0b;--calc:#a78bfa}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font:14px/1.45 system-ui,-apple-system,Segoe UI,sans-serif;padding-bottom:76px}
header{position:sticky;top:0;z-index:5;background:var(--card);border-bottom:1px solid var(--line)}
.top{display:flex;align-items:center;gap:10px;padding:12px 16px 4px}
.top h1{font-size:16px;font-weight:600}.top h1 b{color:var(--acc)}
.pill{display:inline-flex;align-items:center;gap:6px;padding:3px 10px;border-radius:99px;font-size:12px;
 background:var(--card2);border:1px solid var(--line);color:var(--muted)}
.pill i{width:7px;height:7px;border-radius:50%;background:currentColor}
.pill.ok{color:var(--ok)}.pill.warn{color:var(--warn)}.pill.crit{color:var(--crit)}
#conn{margin-left:auto}
nav{display:flex;padding:0 8px;overflow-x:auto}
nav button{background:none;border:0;border-bottom:2px solid transparent;color:var(--muted);padding:10px 12px;
 font-size:14px;cursor:pointer;white-space:nowrap}
nav button.on{color:var(--text);border-bottom-color:var(--acc)}
main{max-width:1000px;margin:0 auto;padding:14px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:12px}
.card>h2{font-size:14px;font-weight:600;margin-bottom:10px;display:flex;align-items:center;gap:8px;flex-wrap:wrap}
.card>h2 small{font-weight:400;color:var(--muted);font-size:12px}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;margin-bottom:10px}
.tile{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px}
.tile .k{font-size:12px;color:var(--muted)}.tile .v{font-size:22px;font-weight:600;margin-top:2px}
.tile .s{font-size:12px;color:var(--muted);margin-top:2px}
.say{font-size:13px;color:var(--muted);margin:2px 2px 12px}
.say b{color:var(--text);font-weight:500}
.vals{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:8px}
.val{background:var(--card2);border:1px solid var(--line);border-radius:10px;padding:8px 10px}
.val .n{font-size:12px;color:var(--muted);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.val .x{font-size:19px;font-weight:600;white-space:nowrap}
.val .x u{font-size:12px;font-weight:400;color:var(--muted);text-decoration:none;margin-left:3px}
.val.stale{opacity:.4}
.tag{display:inline-block;font-size:10px;padding:0 6px;border-radius:5px;line-height:17px;vertical-align:middle}
.tag.raw{color:var(--raw);background:#4ade801c}.tag.obd{color:var(--obd);background:#38bdf81c}
.tag.ssm{color:var(--ssm);background:#f59e0b1c}.tag.calc{color:var(--calc);background:#a78bfa1c}
.tag.no{color:var(--muted);background:#ffffff0d}
.val .tag{float:right;margin-top:1px}
.grp{font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:.07em;margin:14px 0 6px}
.grp:first-child{margin-top:0}
table{width:100%;border-collapse:collapse;font-size:13px}
th{font-size:11px;font-weight:500;color:var(--muted);text-align:left;padding:6px;border-bottom:1px solid var(--line);white-space:nowrap}
td{padding:6px;border-bottom:1px solid var(--line);white-space:nowrap;vertical-align:middle}
tr:last-child td{border-bottom:0}
.wrap{overflow-x:auto}
input,select,textarea{background:var(--card2);border:1px solid var(--line);color:var(--text);border-radius:8px;padding:7px 9px;font-size:13px}
td input,td select{padding:4px 6px;width:100%;min-width:60px}
input[type=checkbox]{width:18px;height:18px;min-width:18px;accent-color:var(--acc);vertical-align:middle}
textarea{width:100%;min-height:320px;font:12px ui-monospace,Consolas,monospace}
button.b,label.b{display:inline-block;background:var(--card2);border:1px solid var(--line);color:var(--text);padding:7px 12px;border-radius:8px;cursor:pointer;font-size:13px}
button.b:hover,label.b:hover{border-color:var(--acc)}
button.p{background:var(--acc);border-color:var(--acc);color:#002a33;font-weight:600}
button.s{padding:3px 9px;font-size:12px}
button.d{color:var(--crit)}
button.x{background:none;border:0;color:var(--muted);cursor:pointer;font-size:16px;padding:0 4px}
button:disabled{opacity:.5}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.f{display:flex;justify-content:space-between;align-items:center;gap:14px;padding:11px 0;border-bottom:1px solid var(--line)}
.f:last-child{border-bottom:0}
.f .l b{display:block;font-weight:500}.f .l span{font-size:12px;color:var(--muted)}
.f input:not([type=checkbox]),.f select{width:170px;flex:none}
.muted{color:var(--muted)}.ok{color:var(--ok)}.warn{color:var(--warn)}.crit{color:var(--crit)}.acc{color:var(--acc)}
.bar{height:6px;border-radius:3px;background:#ffffff12;overflow:hidden;min-width:80px}
.bar i{display:block;height:100%;background:var(--acc);border-radius:3px}
.hx,.mono{font:12px ui-monospace,Consolas,monospace}.hx span{padding:0 2px;color:#4a5967}
.hx span.l{color:var(--text)}.hx span.h{color:var(--acc);background:#00e5ff22;border-radius:3px}
details summary{cursor:pointer;color:var(--muted);font-size:13px;margin-top:10px}
#save{position:fixed;left:0;right:0;bottom:0;background:var(--card);border-top:1px solid var(--line);padding:10px 16px;
 display:none;justify-content:flex-end;align-items:center;gap:10px;z-index:6}
#save.on{display:flex}
#toast{position:fixed;top:64px;left:50%;transform:translateX(-50%);background:var(--card);border:1px solid var(--acc);
 padding:8px 16px;border-radius:10px;display:none;z-index:9;font-size:13px}
.empty{color:var(--muted);font-size:13px;padding:6px 0}
.lr{padding:9px 0;border-bottom:1px solid var(--line)}.lr:last-child{border-bottom:0}
.lr .lt{display:flex;justify-content:space-between;align-items:center;gap:10px;margin-bottom:5px}
.lr .ls{font-size:12px;color:var(--muted);margin-top:4px;display:flex;justify-content:space-between;gap:10px;align-items:center}
.step{font-size:18px;font-weight:600;color:var(--acc);margin:10px 0 4px}
</style></head><body>
<header>
 <div class="top"><h1>CAN <b>Master</b></h1><span id="conn" class="pill"><i></i><span>connecting</span></span></div>
 <nav>
  <button data-t="dash" class="on">Dashboard</button>
  <button data-t="bus">Bus</button>
  <button data-t="src">Sources</button>
  <button data-t="set">Settings</button>
  <button data-t="adv">Advanced</button>
  <button data-t="diag">Diagnostics</button>
 </nav>
</header>
<main>

<section id="t-dash">
 <div class="tiles">
  <div class="tile"><div class="k">CAN bus</div><div class="v" id="dBus">–</div><div class="s" id="dBusS">&nbsp;</div></div>
  <div class="tile"><div class="k">Heard on the bus</div><div class="v ok" id="dRaw">–</div><div class="s">values, nothing asked</div></div>
  <div class="tile"><div class="k">Requested</div><div class="v" id="dReq">–</div><div class="s" id="dReqS">&nbsp;</div></div>
  <div class="tile"><div class="k">Sent to displays</div><div class="v" id="dEsp">–</div><div class="s" id="dEspS">&nbsp;</div></div>
 </div>
 <p class="say" id="dSay"></p>
 <div class="card"><h2>Learning <small id="lSub"></small>
   <label class="muted" style="margin-left:auto;font-size:12px"><input type="checkbox" id="lAll"> show all</label></h2>
  <p class="say" style="margin:0 0 6px">Every value the master requests is matched against the bus. <b>Listen now</b> takes the best
   match so far and reads that value from the bus straight away, even if it is not fully confirmed.</p>
  <div id="lTab"></div></div>
 <div class="card"><h2>Live values <small id="vCount"></small>
  <input id="vq" placeholder="Filter…" style="margin-left:auto;width:140px"></h2>
  <div id="vals"></div></div>
</section>

<section id="t-bus" hidden>
 <div class="card" id="gCard" hidden style="border-color:var(--warn)"><h2 class="warn">Requests paused to protect the bus</h2>
  <p class="say" id="gSay" style="margin:0 0 10px"></p>
  <div class="row"><button class="b p" onclick="post('/api/guard/reset','Requests resumed')">Resume requests</button></div></div>
 <div class="tiles">
  <div class="tile"><div class="k">Bus load</div><div class="v" id="bLoad">–</div><div class="bar" style="margin-top:6px"><i id="bBar" style="width:0"></i></div></div>
  <div class="tile"><div class="k">Our requests</div><div class="v" id="bTx">–</div><div class="s">frames per second</div></div>
  <div class="tile"><div class="k">Bus errors</div><div class="v" id="bErr">–</div><div class="s" id="bErrS">&nbsp;</div></div>
  <div class="tile"><div class="k">Errors after our frames</div><div class="v" id="bErrTx">–</div><div class="s" id="bErrTxS">&nbsp;</div></div>
  <div class="tile"><div class="k">Frames</div><div class="v" id="bFps">–</div><div class="s" id="bIds">&nbsp;</div></div>
 </div>
 <p class="say" id="bSay"></p>
 <div class="card" id="ekCard" hidden><h2>What the errors are <small>from the CAN controller itself</small></h2>
  <p class="say" style="margin:0 0 8px">Every bus error, the controller records its type, whether it was sending or receiving, and where in the frame it happened. Errors while <b>sending</b> in the ACK slot mean our own transceiver is not driving the bus as it should (supply, ground, its mode pin); errors while <b>receiving</b> in the data or CRC mean the link or the bit timing.</p>
  <div class="wrap"><table><thead><tr><th>Error</th><th>Mode</th><th>Count</th></tr></thead><tbody id="ekTab"></tbody></table></div>
  <div id="lkBox"></div></div>
 <div class="card" id="teach"><h2>Teach by doing <small>for what the ECU cannot report</small></h2>
  <p class="say" style="margin:0 0 10px">Turn signals, doors, handbrake, steering, wheel speeds, the gear lever… Pick what you are about to
   operate, press Start and follow the steps. The master finds the bits or bytes that followed you. A flashing turn signal is
   fine: leave it flashing through the ON step. For the gear lever, pick <b>Gear lever</b>, foot on the brake, and move it
   through the positions as asked: it ends back in P, so every bit is checked twice. The master learns the lever as one value
   from whichever bits change with it, in one byte or several.</p>
  <div class="row"><select id="tM" style="flex:1;min-width:180px"></select>
   <select id="tK"><option value="bit">On / off</option><option value="val">A moving value</option><option value="pos">Positions</option></select>
   <button class="b p" id="tBtn" onclick="teachStart()">Start</button></div>
  <div class="row" id="tPRow" hidden style="margin-top:8px"><span class="muted">Positions, in the order you will select them:</span>
   <input id="tP" style="flex:1;min-width:120px" value="P R N D"></div>
  <div id="tStep"></div><div id="tRes"></div></div>
 <div class="card" id="mapCard" hidden><h2>Use a value from <span id="mapId" class="acc"></span>
   <button class="x" style="margin-left:auto" onclick="mapClose()">✕</button></h2>
  <p class="say" style="margin:0 0 10px">A value you map here is read from the bus and replaces the requested one.</p>
  <div class="f"><div class="l"><b>Metric</b></div><select id="mpM" onchange="mapPrev()"></select></div>
  <div class="f"><div class="l"><b>Start bit</b><span>byte k starts at bit 8·k</span></div><input type="number" id="mpS" min="0" max="63" oninput="mapPrev()"></div>
  <div class="f"><div class="l"><b>Length</b><span>bits</span></div><input type="number" id="mpL" min="1" max="32" oninput="mapPrev()"></div>
  <div class="f"><div class="l"><b>Byte order</b></div><select id="mpBE" onchange="mapPrev()"><option value="0">Intel (little-endian)</option><option value="1">Motorola (big-endian)</option></select></div>
  <div class="f"><div class="l"><b>Signed</b></div><input type="checkbox" id="mpSg" onchange="mapPrev()"></div>
  <div class="f"><div class="l"><b>Scale · Offset</b><span>value = raw × scale + offset</span></div>
   <div class="row"><input id="mpK" style="width:80px" oninput="mapPrev()"><input id="mpO" style="width:80px" oninput="mapPrev()"></div></div>
  <p style="margin:10px 0">Raw <b id="mpRaw">–</b> → <b id="mpVal" class="ok">–</b> <span class="muted" id="mpRef"></span></p>
  <div class="row"><button class="b p" onclick="mapAdd()">Use this value</button><button class="b" onclick="mapClose()">Cancel</button></div>
 </div>
 <div class="card"><h2>Frames on the bus <small id="bN"></small>
  <button class="b" style="margin-left:auto" onclick="post('/api/census/reset','Cleared')">Clear</button></h2>
  <p class="say" style="margin:0 0 8px"><span class="hx"><span class="h">Bright</span></span> bytes just changed,
   <span class="hx"><span class="l">white</span></span> ones change sometimes, grey ones never.</p>
  <div class="wrap"><table><thead><tr><th>ID</th><th>Hz</th><th>Data</th><th></th></tr></thead><tbody id="bTab"></tbody></table></div></div>
</section>

<section id="t-src" class="cfg" hidden>
 <p class="say">Where every value comes from, in the order the master prefers.
  <label style="float:right" class="muted"><input type="checkbox" id="adv" onchange="renderSources()"> Edit details</label></p>
 <div class="card"><h2><span class="tag raw">1</span> Heard on the bus <small>free — nothing is asked</small></h2>
  <div class="wrap"><table><thead id="sHead"></thead><tbody id="sTab"></tbody></table></div>
  <div class="row" style="margin-top:10px"><button class="b" onclick="go('bus')">Teach or map a value…</button></div></div>
 <div class="card"><h2><span class="tag obd">2</span> OBD-II requests <small id="oSub"></small>
  <label class="muted" style="margin-left:auto;font-size:12px"><input type="checkbox" id="oAll" onchange="renderSources()"> show unsupported</label></h2>
  <div class="wrap"><table><thead id="oHead"></thead><tbody id="oTab"></tbody></table></div></div>
 <div class="card"><h2><span class="tag ssm">3</span> Subaru SSM2 requests <small id="mSub"></small>
  <label class="muted" style="margin-left:auto;font-size:12px"><input type="checkbox" id="mAll" onchange="renderSources()"> show unsupported</label></h2>
  <div class="wrap"><table><thead id="mHead"></thead><tbody id="mTab"></tbody></table></div>
  <div class="row" style="margin-top:10px"><button class="b" onclick="post('/api/ssm/reinit','Asking the ECU again')">Ask the ECU what it supports again</button></div></div>
</section>

<section id="t-set" class="cfg" hidden>
 <div class="card"><h2>Requests</h2>
  <div class="f"><div class="l"><b>Allow OBD-II requests</b><span>asks the engine ECU for values it does not broadcast</span></div><input type="checkbox" id="sObd"></div>
  <div class="f"><div class="l"><b>Allow Subaru SSM2 requests</b><span>only for what OBD-II cannot give: knock, A/F learning, switches</span></div><input type="checkbox" id="sSsm"></div>
  <div class="f"><div class="l"><b>Learn automatically</b><span>matches requested values to bus traffic, then stops requesting them</span></div><input type="checkbox" data-k="learn"></div>
  <div class="f"><div class="l"><b>Let errors pass</b><span>never sends an error frame: a frame the master reads as bad is dropped by the master alone and reaches every other module untouched. It still requests, receives and acknowledges good frames</span></div><input type="checkbox" data-k="tx_passive"></div>
  <div class="f"><div class="l"><b>Bus guard</b><span>pauses requests if bus errors follow our frames</span></div><input type="checkbox" data-k="guard"></div>
  <div class="f"><div class="l"><b>Receive-error guard</b><span>switches to listen-only if our controller starts corrupting other modules' frames (a receive-error storm) — the protection against the transmission ECU losing the engine ECU's messages</span></div><input type="checkbox" data-k="rx_guard"></div>
  <div class="f"><div class="l"><b>Wait after power-on</b><span>seconds of listening only while the car's bus settles, before the first request · 0 = no wait</span></div><input type="number" data-k="start_delay_s" min="0" max="300"></div>
  <p class="say" style="margin:8px 0 0">With both requests off the master only listens and cannot transmit at all - the CAN controller switches to listen-only the moment you save.</p>
 </div>
 <div class="card"><h2>Displays</h2>
  <div class="f"><div class="l"><b>Night mode from</b></div><select data-k="night_source"><option value="2">The car's light switch</option><option value="1">Light sensor on GPIO</option><option value="0">Never</option></select></div>
  <div class="f"><div class="l"><b>Update every</b><span>ms</span></div><input type="number" data-k="broadcast_ms" min="20" max="1000"></div>
  <div class="f"><div class="l"><b>Drop a value after</b><span>ms without news</span></div><input type="number" data-k="metric_ttl_ms" min="200"></div>
 </div>
 <div class="card"><h2>Power</h2>
  <div class="f"><div class="l"><b>Sleep when the car is off</b><span>wakes on the first CAN activity</span></div><input type="checkbox" data-k="sleep_enabled"></div>
  <div class="f"><div class="l"><b>Sleep after</b><span>seconds of silence on the bus</span></div><input type="number" data-k="sleep_idle_s" min="10" max="3600"></div>
 </div>
 <div class="card"><h2>Connection <small>reboot to apply</small></h2>
  <div class="f"><div class="l"><b>CAN bitrate</b><span>500 kbit/s on the OBD port</span></div>
   <select data-k="bitrate_kbps"><option value="125">125 kbit/s</option><option value="250">250 kbit/s</option><option value="500">500 kbit/s</option><option value="1000">1 Mbit/s</option></select></div>
  <div class="f"><div class="l"><b>CAN bit timing</b><span>87.5 % with triple sampling rides out spikes a single 80 % sample reads as errors</span></div>
   <select data-k="can_timing"><option value="2">87.5 %, triple sampling (recommended)</option><option value="1">87.5 %</option><option value="0">ESP-IDF preset (80 %)</option></select></div>
  <div class="f"><div class="l"><b>Display channel</b><span>must match every display</span></div><input type="number" data-k="wifi_channel" min="1" max="13"></div>
  <div class="f"><div class="l"><b>Wi-Fi name</b><span>1-32 characters</span></div><input data-k="ap_ssid" data-t="str" maxlength="32"></div>
  <div class="f"><div class="l"><b>Wi-Fi password</b><span>8-63 characters, or empty for open</span></div><input data-k="ap_pass" data-t="str" maxlength="63"></div>
 </div>
 <div class="card"><h2>Maintenance</h2>
  <p class="say" id="sEcu" style="margin:0 0 10px"></p>
  <div class="row">
   <button class="b" onclick="post('/api/learn/reset','Learning restarted')">Restart learning</button>
   <button class="b" onclick="if(confirm('Delete every automatically learned value?'))post('/api/learn/forget','Learned values deleted',()=>load(true))">Forget learned values</button>
   <button class="b" onclick="post('/api/reboot','Rebooting')">Reboot</button>
   <button class="b p" onclick="if(confirm('Put the CAN bus settings back to the safe defaults and reboot?\n\nRequests on (Auto), errors let pass, 87.5 % triple sampling, both guards on, P2CAN 50 ms, 10 s wait after power-on.\n\nKept: the bitrate, your request pacing, displays, Wi-Fi and every learned value.'))post('/api/bus_defaults','Safe bus settings - rebooting')">Safe bus settings</button>
   <button class="b d" onclick="if(confirm('Restore factory settings and reboot?\n\nThis also deletes every learned value. For the bus settings alone, use Safe bus settings.'))post('/api/defaults','Restored - rebooting')">Factory reset</button>
  </div>
  <p class="say" style="margin:10px 0 0">Every other setting is on the <a href="#" onclick="go('adv');return false" class="acc">Advanced</a> tab.</p></div>
</section>

<section id="t-adv" class="cfg" hidden>
 <p class="say">Every setting the master has. Changes apply when you press Save; the ones marked <span class="warn">reboot</span>
  take effect after a reboot.</p>
 <div id="advBody"></div>
 <div class="card"><h2>Everything, as JSON <small>tables included</small></h2>
  <p class="say" style="margin:0 0 8px">The complete configuration. Edit it here, or download it as a backup and upload it later.</p>
  <textarea id="json" spellcheck="false"></textarea>
  <div class="row" style="margin-top:8px">
   <button class="b" onclick="jsonShow()">Show current</button>
   <button class="b p" onclick="jsonApply()">Apply</button>
   <button class="b" onclick="jsonDownload()">Download</button>
   <label class="b">Upload<input type="file" id="jf" accept=".json,application/json" hidden onchange="jsonUpload(this)"></label>
  </div></div>
</section>

<section id="t-diag" hidden>
 <div class="card"><h2>Evidence log <small id="evN"></small>
  <button class="b" style="margin-left:auto" onclick="if(confirm('Clear the evidence log?'))post('/api/evlog/clear','Cleared',pollEvlog)">Clear</button></h2>
  <p class="say" style="margin:0 0 8px">Kept on the master's flash across reboots and firmware updates: every boot with its reset reason,
   the bus coming up and settling, every guard trip with the error counters and what the master was transmitting at the time,
   bus-off and sleep. Clear it before a test drive, read it after.</p>
  <p class="say" id="evSay"></p>
  <div class="wrap"><table><thead><tr><th>Uptime</th><th>Event</th><th>Detail</th><th>Master was</th></tr></thead><tbody id="evTab"></tbody></table></div></div>
</section>
</main>
<div id="save"><span class="muted" id="saveMsg">Unsaved changes</span><button class="b" onclick="load()">Discard</button><button class="b p" onclick="save()">Save</button></div>
<div id="toast"></div>
<script>
'use strict';
const META={257:["MIL lamp", "", "Status & switches", 0],259:["Fuel system status", "", "Status & switches", 0],260:["Engine load", "%", "Engine", 1],261:["Coolant", "°C", "Temperatures", 0],262:["STFT B1", "%", "Fuel & mixture", 1],263:["LTFT B1", "%", "Fuel & mixture", 1],264:["STFT B2", "%", "Fuel & mixture", 1],265:["LTFT B2", "%", "Fuel & mixture", 1],266:["Fuel press", "kPa", "Fuel & mixture", 0],267:["MAP", "kPa", "Engine", 0],268:["RPM", "rpm", "Engine", 0],269:["Speed", "km/h", "Driving", 0],270:["Timing", "°", "Engine", 1],271:["IAT", "°C", "Temperatures", 0],272:["MAF", "g/s", "Engine", 1],273:["Throttle", "%", "Engine", 1],276:["O2 B1S1", "V", "Fuel & mixture", 2],277:["O2 B1S2", "V", "Fuel & mixture", 2],280:["O2 B2S1", "V", "Fuel & mixture", 2],281:["O2 B2S2", "V", "Fuel & mixture", 2],287:["Run time", "s", "Driving", 0],289:["Dist MIL", "km", "Driving", 0],290:["Fuel rail (vac)", "kPa", "Fuel & mixture", 0],291:["Fuel rail gauge", "kPa", "Fuel & mixture", 0],292:["WB B1S1", "λ", "Fuel & mixture", 3],296:["WB B2S1", "λ", "Fuel & mixture", 3],300:["EGR cmd", "%", "Fuel & mixture", 1],301:["EGR error", "%", "Fuel & mixture", 1],302:["Evap purge", "%", "Fuel & mixture", 1],303:["Fuel level", "%", "Fuel & mixture", 1],304:["Warm-ups", "", "Driving", 0],305:["Dist cleared", "km", "Driving", 0],306:["Evap vapor", "Pa", "Fuel & mixture", 0],307:["Barometric", "kPa", "Engine", 0],308:["WB B1S1 λ (I)", "λ", "Fuel & mixture", 3],312:["WB B2S1 λ (I)", "λ", "Fuel & mixture", 3],316:["Cat temp", "°C", "Temperatures", 0],317:["Cat temp B2S1", "°C", "Temperatures", 0],318:["Cat temp B1S2", "°C", "Temperatures", 0],319:["Cat temp B2S2", "°C", "Temperatures", 0],322:["Battery", "V", "Electrical", 2],323:["Abs load", "%", "Engine", 1],324:["Cmd", "λ", "Engine", 3],325:["Rel throttle", "%", "Engine", 1],326:["Ambient", "°C", "Temperatures", 0],327:["Abs throttle B", "%", "Engine", 1],328:["Throttle C", "%", "Engine", 1],329:["Pedal D", "%", "Engine", 1],330:["Pedal E", "%", "Engine", 1],331:["Pedal F", "%", "Engine", 1],332:["Cmd throttle", "%", "Engine", 1],333:["Time MIL on", "min", "Driving", 0],334:["Time since clear", "min", "Driving", 0],337:["Fuel type", "", "Status & switches", 0],338:["Ethanol", "%", "Fuel & mixture", 1],339:["Evap abs", "kPa", "Fuel & mixture", 0],345:["Fuel rail abs", "kPa", "Fuel & mixture", 0],346:["Relative pedal", "%", "Engine", 1],347:["Hybrid SOC", "%", "Engine", 1],348:["Oil temp", "°C", "Temperatures", 0],349:["Injection timing", "°", "Fuel & mixture", 1],350:["Fuel rate", "L/h", "Fuel & mixture", 1],353:["Demand torque", "%", "Engine", 1],354:["Actual torque", "%", "Engine", 1],355:["Reference torque", "Nm", "Engine", 0],422:["Odometer", "km", "Driving", 0],769:["DTC count", "", "Status & switches", 0],788:["O2 B1S1 trim", "%", "Fuel & mixture", 1],789:["O2 B1S2 trim", "%", "Fuel & mixture", 1],792:["O2 B2S1 trim", "%", "Fuel & mixture", 1],793:["O2 B2S2 trim", "%", "Fuel & mixture", 1],804:["WB B1S1", "V", "Fuel & mixture", 2],808:["WB B2S1", "V", "Fuel & mixture", 2],820:["WB B1S1", "mA", "Fuel & mixture", 1],824:["WB B2S1", "mA", "Fuel & mixture", 1],4097:["Boost", "bar", "Engine", 2],4098:["AFR", "AFR", "Fuel & mixture", 1],4099:["Oil press", "bar", "Engine", 2],4100:["EGT", "°C", "Temperatures", 0],4101:["Gear", "", "Driving", 0],4102:["Steering", "°", "Driving", 1],4103:["Brake", "bar", "Driving", 2],4104:["Wheel FL", "km/h", "Driving", 0],4105:["Wheel FR", "km/h", "Driving", 0],4106:["Wheel RL", "km/h", "Driving", 0],4107:["Wheel RR", "km/h", "Driving", 0],4108:["Lateral G", "g", "Driving", 2],4109:["Longitudinal G", "g", "Driving", 2],4110:["Yaw rate", "°/s", "Driving", 1],4111:["Gear lever", "", "Driving", 0],4112:["Cruise set", "km/h", "Driving", 0],4113:["ATF temp", "°C", "Temperatures", 0],4353:["Batt current", "A", "Electrical", 1],4354:["Batt SOC", "%", "Electrical", 1],4355:["Batt SOH", "%", "Electrical", 1],4356:["Batt temp", "°C", "Temperatures", 0],4357:["Alternator", "V", "Electrical", 2],4358:["Alternator", "A", "Electrical", 1],4359:["Alternator load", "%", "Electrical", 1],4360:["12V rail", "V", "Electrical", 2],4361:["Elec load", "W", "Electrical", 0],4362:["Charge status", "", "Status & switches", 0],4609:["Left turn signal", "", "Status & switches", 0],4610:["Right turn signal", "", "Status & switches", 0],4611:["High beam", "", "Status & switches", 0],4612:["Door open", "", "Status & switches", 0],4613:["Handbrake", "", "Status & switches", 0],4614:["Seatbelt unfastened", "", "Status & switches", 0],4615:["Cruise active", "", "Status & switches", 0],4616:["Reverse", "", "Status & switches", 0],4617:["Lever in P", "", "Status & switches", 0],4618:["Lever in N", "", "Status & switches", 0],4619:["Lever in D", "", "Status & switches", 0],4620:["Hazard lights", "", "Status & switches", 0],4621:["Cruise main", "", "Status & switches", 0],4622:["Front fog", "", "Status & switches", 0],4623:["Rear fog", "", "Status & switches", 0],4624:["Door FL", "", "Status & switches", 0],4625:["Door FR", "", "Status & switches", 0],4626:["Door RL", "", "Status & switches", 0],4627:["Door RR", "", "Status & switches", 0],4628:["Trunk open", "", "Status & switches", 0],4629:["Hood open", "", "Status & switches", 0],4630:["VDC off", "", "Status & switches", 0],4631:["Passenger belt unfastened", "", "Status & switches", 0],7937:["Night sense", "", "System", 0],7938:["Master uptime", "s", "System", 0],7939:["CAN frames/s", "", "System", 0],7940:["CAN IDs", "", "System", 0],7941:["OBD replies/s", "", "System", 0],7942:["SSM2 exch/s", "", "System", 0],8193:["Knock corr", "°", "Engine", 1],8194:["Knock corr fine", "°", "Engine", 1],8195:["A/F learn", "%", "Fuel & mixture", 1],8196:["A/F corr", "%", "Fuel & mixture", 1],8197:["Inj duty", "%", "Fuel & mixture", 1],8198:["Target boost", "bar", "Engine", 2],8199:["Wastegate", "%", "Engine", 1],8200:["A/F corr 2", "%", "Fuel & mixture", 1],8201:["A/F learn 2", "%", "Fuel & mixture", 1],8202:["Front O2 #1", "V", "Fuel & mixture", 2],8203:["Rear O2", "V", "Fuel & mixture", 2],8204:["Front O2 #2", "V", "Fuel & mixture", 2],8205:["MAF sensor", "V", "Engine", 2],8206:["TPS", "V", "Engine", 2],8207:["Atmospheric", "kPa", "Engine", 0],8208:["Manifold rel", "kPa", "Engine", 0],8209:["Tank press", "kPa", "Engine", 0],8210:["Learned timing", "°", "Engine", 1],8211:["Accel pedal", "%", "Engine", 1],8212:["Fuel temp", "°C", "Temperatures", 0],8213:["Wastegate 2", "%", "Engine", 1],8214:["CPC duty", "%", "Fuel & mixture", 1],8215:["ISC duty", "%", "Engine", 1],8216:["A/F lean", "%", "Fuel & mixture", 1],8217:["A/F heater", "%", "Fuel & mixture", 1],8218:["ISC step", "", "Engine", 0],8219:["EGR steps", "", "Fuel & mixture", 0],8220:["Alternator duty", "%", "Electrical", 1],8221:["Fuel pump", "%", "Fuel & mixture", 1],8222:["Intake VVT R", "°", "Engine", 1],8223:["Intake VVT L", "°", "Engine", 1],8224:["Intake OCV R", "%", "Engine", 1],8225:["Intake OCV L", "%", "Engine", 1],8226:["A/F current", "mA", "Fuel & mixture", 1],8227:["Lambda", "λ", "Fuel & mixture", 3],8228:["Lambda 2", "λ", "Fuel & mixture", 3],8229:["Throttle motor", "%", "Engine", 1],8230:["Main TPS", "V", "Engine", 2],8231:["Main APS", "V", "Engine", 2],8232:["Brake boost", "kPa", "Driving", 0],8233:["Fuel HP", "MPa", "Fuel & mixture", 2],8235:["Exhaust VVT R", "°", "Engine", 1],8236:["Exhaust VVT L", "°", "Engine", 1],8237:["Rough cyl 1", "", "Engine", 0],8238:["Rough cyl 2", "", "Engine", 0],8239:["Rough cyl 3", "", "Engine", 0],8240:["Rough cyl 4", "", "Engine", 0],8241:["Inj 2 pulse", "ms", "Fuel & mixture", 2],8242:["Cold start inj", "ms", "Engine", 2],8243:["Alternator mode", "", "Status & switches", 0],8244:["Fuel sender", "V", "Fuel & mixture", 2],8245:["Radiator fan", "%", "Engine", 1],8246:["Learned ign corr", "°", "Engine", 1],8247:["Boost feedback", "%", "Engine", 1],8248:["Target RPM", "rpm", "Engine", 0],8249:["SI-Drive", "", "Status & switches", 0],8250:["Odometer (SSM)", "km", "Driving", 0],8251:["EPS current", "A", "Electrical", 1],8252:["Fuel pump", "A", "Fuel & mixture", 1],8254:["Inj 1 pulse", "ms", "Fuel & mixture", 2],8449:["Brake switch", "", "Status & switches", 0],8450:["Clutch switch", "", "Status & switches", 0],8451:["Neutral switch", "", "Status & switches", 0],8452:["A/C switch", "", "Status & switches", 0],8453:["Idle switch", "", "Status & switches", 0],8454:["Knock signal", "", "Status & switches", 0],8455:["Electrical load", "", "Status & switches", 0],8456:["Light switch", "", "Status & switches", 0],8457:["Radiator fan 1", "", "Status & switches", 0],8458:["Radiator fan 2", "", "Status & switches", 0],8459:["A/C compressor", "", "Status & switches", 0],8460:["Fuel pump relay", "", "Status & switches", 0],8461:["Oil pressure switch", "", "Status & switches", 0],8462:["Starter", "", "Status & switches", 0],8463:["Rear defogger", "", "Status & switches", 0],8464:["Blower", "", "Status & switches", 0],8465:["Wiper switch", "", "Status & switches", 0],8466:["Ignition", "", "Status & switches", 0],8467:["Stop lamp switch", "", "Status & switches", 0],8468:["Knock signal 2", "", "Status & switches", 0],12289:["Custom 1", "", "Custom", 0],12290:["Custom 2", "", "Custom", 0],12291:["Custom 3", "", "Custom", 0],12292:["Custom 4", "", "Custom", 0],12293:["Custom 5", "", "Custom", 0],12294:["Custom 6", "", "Custom", 0],12295:["Custom 7", "", "Custom", 0],12296:["Custom 8", "", "Custom", 0],12297:["Custom 9", "", "Custom", 0],12298:["Custom 10", "", "Custom", 0],12299:["Custom 11", "", "Custom", 0],12300:["Custom 12", "", "Custom", 0],12301:["Custom 13", "", "Custom", 0],12302:["Custom 14", "", "Custom", 0],12303:["Custom 15", "", "Custom", 0],12304:["Custom 16", "", "Custom", 0]};
const $=q=>document.querySelector(q);
const hex=(n,w)=>'0x'+Number(n).toString(16).toUpperCase().padStart(w,'0');
const esc=s=>String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const nm=id=>META[id]?META[id][0]:hex(id,4);
const un=id=>META[id]?META[id][1]:'';
const fmt=(id,v)=>{if(id===0x100F&&v>32&&v<127)return String.fromCharCode(Math.round(v));const d=META[id]?META[id][3]:2;return (+v).toFixed(d)};
const SRC={1:['SSM2','ssm'],2:['OBD-II','obd'],3:['calc','calc'],4:['bus','raw']};
let C={},ORIG={},L=null,LOADED=false,DIRTY=false,TAB='dash',PREV=null,RATES={load:0,tx:0,errs:0},LSIG='',LLOAD=Date.now();
function toast(m){const t=$('#toast');t.textContent=m;t.style.display='block';clearTimeout(t._h);t._h=setTimeout(()=>t.style.display='none',2400)}
async function post(u,msg,then){try{const r=await fetch(u,{method:'POST'});if(!r.ok)throw 0;toast(msg);if(then)setTimeout(then,500)}catch(e){toast('Failed')}}
function setDirty(v){DIRTY=v;$('#save').classList.toggle('on',v)}
function go(t){TAB=t;document.querySelectorAll('nav button').forEach(b=>b.classList.toggle('on',b.dataset.t===t));
 ['dash','bus','src','set','adv','diag'].forEach(x=>$('#t-'+x).hidden=x!==t);
 if(t==='bus')pollBus();if(t==='src')renderSources();if(t==='dash'&&L)renderDash();if(t==='adv'){renderAdv();jsonShow()}
 if(t==='diag')pollEvlog()}
document.querySelectorAll('nav button').forEach(b=>b.onclick=()=>go(b.dataset.t));

/* ───────────── settings: every input bound to the config by data-k ───────────── */
const MODE_TOG={0:[1,1],1:[0,1],2:[1,0],3:[1,1],4:[0,0]};
const togToMode=(o,s)=>o&&s?0:s?1:o?2:4;
function parseEl(el){
 if(el.type==='checkbox')return el.checked;
 if(el.dataset.t==='str')return el.value;
 const v=parseFloat(el.value);return Number.isFinite(v)?v:undefined}
function fillBound(){
 document.querySelectorAll('[data-k]').forEach(el=>{const v=C[el.dataset.k];if(v===undefined)return;
  if(el.type==='checkbox')el.checked=!!v;else if(document.activeElement!==el)el.value=v});
 const t=MODE_TOG[C.diag_mode]||[1,1];$('#sObd').checked=!!t[0];$('#sSsm').checked=!!t[1]}
/* Settings that would lock this page away: the AP will not start without a
   name, nor with a key WPA2 cannot use; and with the Wi-Fi off at boot only
   the BOOT button brings the page back. */
function refused(k,v){
 const len=s=>new TextEncoder().encode(s).length;
 if(k==='ap_ssid'&&(!len(v)||len(v)>32))return'The Wi-Fi name takes 1 to 32 characters';
 if(k==='ap_pass'&&len(v)>63)return'The Wi-Fi password takes at most 63 characters';
 if(k==='ap_pass'&&len(v)&&len(v)<8)toast('Under 8 characters the Wi-Fi stays open');
 if(k==='portal_on'&&v===false&&!confirm('Switch this Wi-Fi off at boot?\n\nThis page is then out of reach. '+
   'To get it back, hold the master\'s BOOT button for 3 seconds while it runs (ignition on).'))return'Wi-Fi kept on';
 return''}
document.addEventListener('change',e=>{
 const el=e.target;
 if(el.dataset&&el.dataset.k){const v=parseEl(el);
  const why=v===undefined?'':refused(el.dataset.k,v);
  if(why){toast(why);if(el.type==='checkbox')el.checked=!!C[el.dataset.k];else el.value=C[el.dataset.k]??'';return}
  if(v!==undefined){C[el.dataset.k]=v;fillBound();setDirty(true)}return}
 if(el.id==='sObd'||el.id==='sSsm'){C.diag_mode=togToMode($('#sObd').checked,$('#sSsm').checked);fillBound();setDirty(true);return}
 if(el.closest('#t-src')&&!['adv','oAll','mAll'].includes(el.id))setDirty(true)});
const same=(a,b)=>JSON.stringify(a)===JSON.stringify(b),clone=o=>JSON.parse(JSON.stringify(o));
const inside=sel=>{const a=document.activeElement;return!!(a&&a.closest&&a.closest(sel))};
/* soft: keep every edit not saved yet and take the master's version of the rest -
   the master changes its own tables (learner, verifier) while the page is open. */
async function load(soft){
 let n;try{n=await fetch('/api/config').then(r=>r.json())}catch(e){return}
 if(soft&&LOADED){
  // A table row being edited right now keeps its data: its inputs write back
  // by row number, and a refreshed table may have rows in other places.
  const hold=inside('#t-src')?['signals','pids','ssm']:[];
  for(const k of Object.keys(n)){
   if(hold.includes(k))continue;
   if(!DIRTY||same(C[k],ORIG[k])){C[k]=n[k];ORIG[k]=clone(n[k])}}}
 else{C=n;ORIG=clone(n);setDirty(false)}
 LOADED=true;fillBound();
 if(!inside('#t-src'))renderSources();
 if(TAB==='adv'&&!inside('#t-adv')){renderAdv();jsonShow()}}
const REBOOT_KEYS=['bitrate_kbps','wifi_channel','ap_ssid','ap_pass','portal_on','can_sp875','can_timing'];
const RO_KEYS=['cfg_ver','ecu_id','ecu_sys_id','ecu_flags'];
const sigKey=s=>[s.can_id,s.ext?1:0,s.start,s.len,s.be?1:0,s.metric].join(':');
/* Three-way merge of the signal table: what the master holds now, plus exactly
   the edits made on this page since it loaded. Writing back the table as it was
   at load time would delete everything learned in the meantime. */
function mergeSignals(base,mine,theirs){
 const B=new Map(base.map(s=>[sigKey(s),s])),M=new Map(mine.map(s=>[sigKey(s),s])),out=[],seen=new Set();
 for(const t of theirs){const k=sigKey(t);
  if(B.has(k)&&!M.has(k))continue;                          // removed here
  const m=M.get(k);out.push(m&&(!B.has(k)||!same(m,B.get(k)))?m:t);seen.add(k)}
 for(const m of mine){const k=sigKey(m);
  if(seen.has(k))continue;
  if(B.has(k)&&same(m,B.get(k)))continue;                   // the master dropped it; untouched here
  out.push(m)}
 return out}
/* Only what changed is sent, so a save can never overwrite something the master
   changed by itself (learned signals, the remembered ECU identity). */
async function save(){
 if(!LOADED){toast('Not loaded yet');return}
 const body={};
 for(const k of Object.keys(C))if(!RO_KEYS.includes(k)&&!same(C[k],ORIG[k]))body[k]=C[k];
 if(!Object.keys(body).length){setDirty(false);toast('Nothing changed');return}
 const reboot=REBOOT_KEYS.some(k=>k in body);
 try{
  if(body.signals){const now=await fetch('/api/config').then(r=>r.json());
   body.signals=mergeSignals(ORIG.signals||[],C.signals||[],now.signals||[])}
  if(body.pids)body.pids=body.pids.map(({name,metrics,...p})=>p);   // read-only extras stay here
  const r=await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  if(!r.ok)throw 0;setDirty(false);toast(reboot?'Saved - reboot to apply':'Saved');await load()}
 catch(e){toast('Save failed')}}

/* ───────────── advanced: every setting ───────────── */
const ADV=[
 ['Requests',[
  ['diag_mode','Mode','sel',[[0,'Auto: listen → OBD-II → SSM2'],[1,'SSM2 requests only'],[2,'OBD-II requests only'],[3,'Both, no preference'],[4,'Listen only - nothing is transmitted']]],
  ['start_delay_s','Listen only after the bus comes up for','num','s · 0 = no wait',0,300],
  ['obd_addr','OBD-II addressing','sel',[[0,'Auto (engine ECU first)'],[1,'Engine ECU only (0x7E0)'],[2,'All ECUs (0x7DF)']]],
  ['obd_gap','Pause between OBD-II requests','num','ms',0,255],
  ['obd_to','OBD-II reply timeout','num','ms',20,2000],
  ['obd_p2can','After an unanswered request, wait at least (ISO 15765-4 P2CAN)','num','ms from the send · 0 = off',0,2000],
  ['req_max_hz','Request budget','num','requests/s · 0 = no cap',0,200],
  ['ssm_gap','Pause between SSM2 requests','num','ms',50,5000],
  ['ssm_batch','Values per SSM2 request','num','2–33',2,33],
  ['ssm_to','SSM2 reply timeout','num','ms',100,5000],
  ['ssm_switches','Read SSM2 switch inputs','bool'],
  ['cover_ms','Skip a request while a better source is fresher than','num','ms',100,30000]]],
 ['Learning',[
  ['learn','Learn automatically','bool'],
  ['learn_r2','Fit needed to call it a match','num','0.5–0.99999',0.5,0.99999,0.0001],
  ['learn_min_n','Samples needed','num','',10,5000],
  ['learn_steady','Steady-sample tolerance','num','× the movement needed',0.01,10,0.01],
  ['learn_phi','Switch correlation needed','num','0.5–1',0.5,1,0.01],
  ['verify_n','Confirmations before trusting a match','num','samples',5,250]]],
 ['Bus guard',[
  ['guard','Bus guard on','bool'],
  ['guard_errs','Errors after our frames that trip it','num','',1,100],
  ['guard_win','…within','num','s',1,120],
  ['guard_pause','Pause requests for','num','s',1,3600],
  ['guard_trips','Trips before switching to listen-only','num','',1,50],
  ['rx_guard','Receive-error guard: listen-only when our controller corrupts other modules\' frames','bool'],
  ['rx_guard_rec','…receive-error counter that trips it','num','96 = error-warning level',16,255],
  ['rx_guard_errs','…or receive errors within the guard window that trip it','num','',1,250]]],
 ['CAN controller',[
  ['bitrate_kbps','Bitrate','sel',[[125,'125 kbit/s'],[250,'250 kbit/s'],[500,'500 kbit/s'],[1000,'1 Mbit/s']],1],
  ['tx_passive','Let errors pass: never send an error frame','bool'],
  ['can_timing','Bit timing','sel',[[2,'87.5 %, triple sampling (recommended)'],[1,'87.5 %'],[0,'ESP-IDF preset (80 %)']],1],
  ['can_sp875','With the ESP-IDF preset: 87.5 % instead','bool',null,null,null,null,1]]],
 ['Displays',[
  ['radio_dbm','Radio transmit power','num','dBm - lower draws less from the supply',2,20],
  ['broadcast_ms','Send every','num','ms',20,1000],
  ['keepalive_ms','Re-send unchanged values every','num','ms',50,1400],
  ['metric_ttl_ms','Drop a value after','num','ms',200,10000],
  ['hold_ms','A better source holds a value for','num','ms',0,5000],
  ['night_source','Night mode from','sel',[[2,"The car's light switch"],[1,'Light sensor on GPIO'],[0,'Never']]],
  ['ldr_dark','Light sensor: night below','num','ADC',0,4095],
  ['ldr_light','Light sensor: day above','num','ADC',0,4095]]],
 ['Power',[
  ['sleep_enabled','Sleep when the car is off','bool'],
  ['sleep_idle_s','Sleep after','num','s',10,3600],
  ['tx_hold','Hold the CAN TX line recessive across sleep','bool']]],
 ['Network',[
  ['wifi_channel','Display channel','num','1–13',1,13,1,1],
  ['ap_ssid','Wi-Fi name','str','1–32 characters',null,32,null,1],
  ['ap_pass','Wi-Fi password','str','8–63 characters or empty',null,63,null,1],
  ['portal_on','Start this Wi-Fi at boot','bool',null,null,null,null,1]]]];
function renderAdv(){
 $('#advBody').innerHTML=ADV.map(([title,rows])=>`<div class="card"><h2>${title}</h2>`+rows.map(r=>{
  const [k,label,type,a,min,max,step]=r,tag=REBOOT_KEYS.includes(k)?' <span class="warn">reboot</span>':'';
  let input;
  if(type==='bool')input=`<input type="checkbox" data-k="${k}">`;
  else if(type==='sel')input=`<select data-k="${k}">${a.map(o=>`<option value="${o[0]}">${esc(o[1])}</option>`).join('')}</select>`;
  else if(type==='str')input=`<input data-k="${k}" data-t="str"${max?` maxlength="${max}"`:''}>`;
  else input=`<input type="number" data-k="${k}"${min!=null?` min="${min}"`:''}${max!=null?` max="${max}"`:''}${step?` step="${step}"`:''}>`;
  const hint=type==='num'||type==='str'?(a?`<span>${esc(a)}</span>`:''):'';
  return`<div class="f"><div class="l"><b>${esc(label)}${tag}</b>${hint}</div>${input}</div>`}).join('')+'</div>').join('');
 fillBound()}
function jsonShow(){$('#json').value=JSON.stringify(C,null,1)}
function jsonApply(){
 let o;try{o=JSON.parse($('#json').value)}catch(e){toast('Not valid JSON: '+e.message);return}
 if(typeof o!=='object'||!o){toast('Expected an object');return}
 C=Object.assign(C,o);fillBound();renderSources();renderAdv();setDirty(true);toast('Applied - press Save to keep it')}
function jsonDownload(){const a=document.createElement('a');
 a.href=URL.createObjectURL(new Blob([JSON.stringify(C,null,1)],{type:'application/json'}));
 a.download='can-master-config.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000)}
function jsonUpload(inp){const f=inp.files[0];if(!f)return;const r=new FileReader();
 r.onload=()=>{$('#json').value=r.result;jsonApply()};r.readAsText(f);inp.value=''}

/* ───────────── dashboard ───────────── */
const LS=[['Waiting for the ECU','muted'],['Needs movement','warn'],['Searching','acc'],['Found — confirming','acc'],['Listening','ok'],['Not found yet','muted']];
function hint(id){
 if(id===0x010C)return'hold a few different revs for a couple of seconds each';
 if(id===0x010D||id===0x1005)return'drive, at a few different speeds';
 if(id===0x0105||id===0x010F||id===0x015C)return'let it warm up';if(id===0x012F)return'drive a while';
 if(id===0x0142)return'start and stop the engine';if((id&0xFF00)===0x2100||id===0x0101)return'switch it on and off a few times';
 return'hold it at a few different levels for a couple of seconds each'}
function field(x){
 if(!x.id)return'<span class="muted">—</span>';
 const tbl=x.map&&x.map.length?' • '+x.map.map(p=>esc(posLabel(x.m,p[1]))).join(' '):'';
 if(x.bits&&x.bits.length)return`${hex(x.id,3)} <span class="muted">bit${x.bits.length>1?'s':''} ${x.bits.join(' ')}${tbl}</span>`;
 if(x.l===1)return`${hex(x.id,3)} <span class="muted">bit ${x.s}</span>`;
 const k=x.s>>3;
 if(x.l<8&&!x.be)return`${hex(x.id,3)} <span class="muted">bits ${x.s}–${x.s+x.l-1}${tbl}</span>`;
 return`${hex(x.id,3)} <span class="muted">${x.l===8?'byte '+k:'bytes '+k+'–'+(k+1)}${x.l===12||x.l===14?' ('+x.l+' bit)':''}${x.be&&x.l>8?' BE':''}${tbl}</span>`}
async function learnAct(a,m){await post(`/api/learn/${a}?m=${m}`,a==='accept'?`Listening for ${nm(m)}`:`Requesting ${nm(m)} again`,()=>load(true))}
function renderDash(){
 const d=L,live=d.metrics.filter(m=>m.age<2500);
 const by=s=>live.filter(m=>m.src===s).length;
 const nRaw=by(4),nObd=by(2),nSsm=by(1);
 $('#dBus').textContent=d.bus.alive?`${(RATES.fps||0).toFixed(0)}/s`:'No signal';
 $('#dBus').className='v '+(d.bus.alive?'':'crit');
 $('#dBusS').textContent=d.bus.alive?`${RATES.load.toFixed(0)}% load · ${d.bus.err?d.bus.err+' errors':'no errors'}`:'ignition off or wiring';
 $('#dRaw').textContent=nRaw;
 $('#dReq').textContent=nObd+nSsm;
 $('#dReqS').innerHTML=`<span class="tag obd">OBD-II ${nObd}</span> <span class="tag ssm">SSM2 ${nSsm}</span> · ${RATES.tx.toFixed(0)} req/s`;
 $('#dEsp').textContent=`${(d.esp.mps||0).toFixed(0)}/s`;$('#dEspS').textContent=`values, in ${(d.esp.fps||0).toFixed(0)} packets/s`;
 const learned=d.learn.filter(x=>x.st===4).length,todo=d.learn.filter(x=>x.st>=1&&x.st<=3).length;
 let say;
 if(!d.bus.alive)say='<b>No CAN traffic.</b> The ignition is off, or the transceiver is not wired to the OBD port.';
 else if(d.silent)say='<b>Listening only.</b> Nothing is transmitted; every value shown comes from the car\'s own broadcasts.';
 else if(d.settle)say=`<b>Letting the bus settle.</b> The car just woke, so nothing is transmitted for another ${Math.ceil(d.settle/1000)} s; until then every value shown comes from the car's own broadcasts.`;
 else{
  say=`<b>${nRaw} ${nRaw===1?'value is':'values are'} read by listening.</b> `;
  if(nObd)say+=`OBD-II is asked for ${nObd} the car does not broadcast`+(nSsm?`, and SSM2 for ${nSsm} more OBD-II has no PID for. `:'. ');
  else if(nSsm)say+=`SSM2 is asked for ${nSsm}. `;
  if(d.obd.answers===2&&d.ssm.answers===2)say+='<span class="warn">The ECU answers neither OBD-II nor SSM2.</span> ';
  if(d.guard.rx_silent)say+='<span class="crit">Our controller was corrupting other modules\' frames, so the master switched itself to listen-only.</span> See the Bus and Diagnostics tabs. ';
  else if(d.guard.silent)say+='<span class="crit">Bus errors kept following our requests, so the master switched itself to listen-only.</span> See the Bus tab. ';
  else if(d.guard.pause)say+=`<span class="warn">Requests paused for ${Math.ceil(d.guard.pause/1000)} s after bus errors.</span> `;
  if(todo)say+=`${todo} more ${todo===1?'is':'are'} being learned.`;
 }
 $('#dSay').innerHTML=say;
 $('#lSub').textContent=d.silent?'needs requests turned on':`${learned} listening · ${todo} in progress · ${d.learn_f} fields, ${d.learn_b} bits watched`;
 const order=[2,3,1,5,4,0],all=$('#lAll').checked;
 const rows=d.learn.filter(x=>all||x.st!==0).sort((a,b)=>order.indexOf(a.st)-order.indexOf(b.st)||b.fit-a.fit);
 $('#lTab').innerHTML=rows.map(x=>{
  const s=LS[x.st]||LS[0];let w=0,col='var(--acc)',sub='',act='';
  if(x.st===4){w=100;col='var(--ok)';const sg=(C.signals||[]).find(s=>s.metric===x.m&&(s.mode===1||s.mode===3));
   sub='read from '+(sg?field({id:sg.can_id,s:sg.start,l:sg.len,be:sg.be}):'the bus');
   act=`<button class="b s" onclick="learnAct('unlearn',${x.m})">Request instead</button>`}
  else if(x.st===0){sub=d.silent?'turn requests on to learn':'not requested yet'}
  else{
   w=x.st===1?x.p*100:Math.max(0,x.fit)*100;col=x.st===1?'var(--warn)':'var(--acc)';
   sub=x.st===1?hint(x.m):'';
   if(x.id)sub+=(sub?' · ':'')+`best so far ${field(x)} · ${(x.fit*100).toFixed(1)}% ${x.l===1?'follow':'fit'}`;
   if(x.id)act=`<button class="b s" onclick="learnAct('accept',${x.m})">Listen now</button>`}
  return`<div class="lr"><div class="lt"><b>${esc(nm(x.m))}</b><span class="${s[1]}">${s[0]}</span></div>
   <div class="bar"><i style="width:${w.toFixed(0)}%;background:${col}"></i></div><div class="ls"><span>${sub}</span>${act}</div></div>`}).join('')||
  '<p class="empty">Nothing to learn yet — the master learns from the values it requests.</p>';
 const q=$('#vq').value.trim().toLowerCase();
 const vrows=d.metrics.filter(m=>(m.id&0xFF00)!==0x1F00&&(!q||nm(m.id).toLowerCase().includes(q)));
 $('#vCount').textContent=`${live.length} live`;
 const groups={};vrows.forEach(m=>{const g=META[m.id]?META[m.id][2]:'Other';(groups[g]=groups[g]||[]).push(m)});
 const GO=['Engine','Fuel & mixture','Temperatures','Driving','Electrical','Status & switches','Custom','Other'];
 $('#vals').innerHTML=GO.filter(g=>groups[g]).map(g=>`<div class="grp">${g}</div><div class="vals">`+
  groups[g].sort((a,b)=>a.id-b.id).map(m=>{const s=SRC[m.src]||['?','no'];
   return`<div class="val${m.age>2500?' stale':''}"><div class="n"><span class="tag ${s[1]}">${s[0]}</span>${esc(nm(m.id))}</div>
   <div class="x">${fmt(m.id,m.v)}<u>${esc(un(m.id))}</u></div></div>`}).join('')+'</div>').join('')||
  '<p class="empty">No values yet.</p>'}
$('#vq').oninput=()=>L&&renderDash();
$('#lAll').onchange=()=>L&&renderDash();

/* ───────────── bus ───────────── */
let ZP={},ZC={},ZT=0,ZHZ={},MAP=null,BUS=null;
/*<teach-core>*/
const hb=h=>{const b=[];for(let i=0;i<h.length;i+=2)b.push(parseInt(h.substr(i,2),16));return b};
function extract(b,st,len,be,sg){
 let w=0n;
 if(!be){for(let i=7;i>=0;i--)w=(w<<8n)|BigInt(b[i]||0);w>>=BigInt(st)}
 else{for(let i=0;i<8;i++)w=(w<<8n)|BigInt(b[i]||0);const msb=Math.floor(st/8)*8+(7-st%8),sh=64-msb-len;w=sh>=0?(w>>BigInt(sh)):0n}
 let r=Number(w&((1n<<BigInt(len))-1n));if(sg&&len<53&&r>=2**(len-1))r-=2**len;return r}
/*</teach-core>*/
async function pollBus(){
 if(TAB!=='bus'||TEACH)return;
 try{BUS=await fetch('/api/bus').then(r=>r.json())}catch(e){return}
 const now=Date.now(),dt=ZT?(now-ZT)/1000:0;ZT=now;
 $('#bN').textContent=BUS.census.length+' IDs';
 $('#bTab').innerHTML=BUS.census.map(c=>{
  const k=(c.ext?'x':'')+c.id,b=hb(c.d||''),ch=hb(c.c||''),pb=ZP[k]||[];
  if(dt&&ZC[k]!==undefined)ZHZ[k]=(c.count-ZC[k])/dt;ZC[k]=c.count;ZP[k]=b;
  const by=b.map((v,i)=>`<span class="${pb.length&&pb[i]!==v?'h':ch[i]?'l':''}">${v.toString(16).toUpperCase().padStart(2,'0')}</span>`).join('');
  const used=(C.signals||[]).filter(s=>s.can_id===c.id&&(s.mode===1||s.mode===3)).map(s=>nm(s.metric));
  return`<tr><td>${c.ext?'ext ':''}${hex(c.id,3)}</td><td>${ZHZ[k]!==undefined?ZHZ[k].toFixed(0):'–'}${c.age>2000?' <span class="warn">idle</span>':''}</td>
   <td class="hx">${by}${used.length?` <span class="tag raw">${esc(used.join(', '))}</span>`:''}</td>
   <td><button class="b s" onclick="mapOpen(${c.id},${c.ext?1:0})">Map</button></td></tr>`}).join('')||'<tr><td colspan="4" class="empty">Nothing heard yet.</td></tr>';
 if(MAP)mapPrev()}
const EK_SEG={3:'start of frame',2:'identifier',6:'identifier',7:'identifier',15:'identifier',14:'identifier',
 4:'control bits',5:'control bits',12:'control bits',13:'control bits',9:'control bits',11:'length code',10:'data field',
 8:'CRC sequence',24:'CRC delimiter',25:'ACK slot',27:'ACK delimiter',26:'end of frame',18:'intermission',
 17:'active error flag',22:'passive error flag',19:'dominant bits after an error flag',23:'error delimiter',28:'overload flag'};
function ekText(c){return['bit error','form error','stuff error','error'][c>>6]+' while '+((c&32)?'receiving':'sending')+' in '+(EK_SEG[c&31]||'segment '+(c&31))}
/* Where the errors come from: the car's regular frames the master missed
   (a gap in an ID's rhythm is a frame it misread), by mode, and the errors
   checked against our own radio. */
function renderLink(d){
 const k=d.bus.link;if(!k){$('#lkBox').innerHTML='';return false}
 const pc=(a,b)=>b?(100*a/b).toFixed(a&&100*a/b<1?2:1)+' %':'–';
 const modes=[['Normal mode',0],['Listen-only',1]].filter(([,i])=>k.exp[i]||k.rx[i]);
 let h='';
 if(modes.length)h+='<h3 style="margin:14px 0 6px">Where they come from</h3><p class="say" style="margin:0 0 8px">'+modes.map(([n,i])=>
  `<b>${n}</b>: missed ${k.miss[i]} of ${k.exp[i]} of the car's regular frames (${pc(k.miss[i],k.exp[i])}); ${k.rx[i]?(1000*k.err[i]/k.rx[i]).toFixed(1):'–'} bus errors per 1000 frames`).join('<br>')+'</p>';
 if(k.chk>=20){const near=100*k.near/k.chk,hot=k.hot/10;
  h+=`<p class="say" style="margin:0 0 8px">${near.toFixed(0)} % of ${k.chk} bus errors came while our radio was transmitting or within 3 ms of it, and it is that busy ${hot.toFixed(0)} % of the time. `+
   (near>=30&&near>=2*hot?'<span class="warn">Far above chance: the radio is disturbing the CAN side - its 3.3 V supply or ground. Try Advanced → Radio transmit power 2 dBm and compare.</span>':'About chance: the radio is not what causes them.')+'</p>'}
 if(k.ids&&k.ids.length)h+='<div class="wrap"><table><thead><tr><th>ID</th><th>Every</th><th>Missed</th><th>Share</th></tr></thead><tbody>'+
  k.ids.map(r=>`<tr><td>${r.x?'ext ':''}${hex(r.i,3)}</td><td>${r.p} ms</td><td>${r.m} of ${r.e}</td><td>${pc(r.m,r.e)}</td></tr>`).join('')+'</tbody></table></div>'+
  '<p class="say" style="margin:8px 0 0">Misses on one module\'s IDs only: the link to that module, where the master taps in. Spread over every module: the master\'s own side - supply, ground, transceiver. More in normal mode than listen-only: its own transceiver driving the bus upsets it.</p>';
 $('#lkBox').innerHTML=h;return !!h}
function renderBusTiles(){
 const d=L;
 const ks=d.bus.kinds||[];const lk=renderLink(d);$('#ekCard').hidden=!ks.length&&!lk;
 $('#ekTab').innerHTML=ks.map(k=>`<tr><td>${ekText(k.c)}</td><td>${k.s?'listen-only':'normal'}</td><td>${k.n}</td></tr>`).join('');
 $('#bErrTx').textContent=d.bus.err_tx;$('#bErrTx').className='v '+(d.bus.err_tx?'crit':'ok');
 $('#bErrTxS').textContent=`${d.bus.err_idle} while we were quiet`;
 const g=d.guard;$('#gCard').hidden=!(g.silent||g.pause);
 const trips=C.guard_trips||3;
 if(g.rx_silent)$('#gSay').innerHTML='Our controller\'s receive-error counter climbed into the error-warning region: it was error-flagging — destroying — other modules\' frames, which is how the transmission ECU loses the engine ECU\'s messages. It was switched to <b>listen-only</b>. Check the transceiver ground and stub length, try the late sample point (Advanced), then resume. The Diagnostics tab has the record.';
 else if(g.silent)$('#gSay').innerHTML='Bus errors kept following our frames, so the controller was switched to <b>listen-only</b> — it cannot disturb the car now. Fix the wiring (see below), then resume. The guard can be turned off in Settings.';
 else if(g.pause)$('#gSay').innerHTML=`Bus errors followed our frames, so all requests are paused for another ${Math.ceil(g.pause/1000)} s (pause ${g.trips} of ${trips}; the last one switches to listen-only).`;
 $('#bLoad').textContent=d.bus.alive?RATES.load.toFixed(0)+'%':'–';
 const bar=$('#bBar');bar.style.width=Math.min(100,RATES.load)+'%';
 bar.style.background=RATES.load>70?'var(--crit)':RATES.load>50?'var(--warn)':'var(--ok)';
 $('#bTx').textContent=RATES.tx.toFixed(0);
 $('#bErr').textContent=d.bus.err;$('#bErr').className='v '+(RATES.errs>0?'crit':d.bus.err?'warn':'ok');
 $('#bErrS').textContent=`error counters ${d.bus.tec} / ${d.bus.rec}`+(C.tx_passive&&!d.silent&&d.bus.tec>=128?' (high on purpose: errors let pass)':'');
 $('#bFps').textContent=(RATES.fps||0).toFixed(0)+'/s';$('#bIds').textContent=d.bus.ids+' different IDs';
 let s;
 if(!d.bus.alive)s='No traffic.';
 else{
  s=RATES.load<50?`Load is comfortable — CAN only struggles past about 70%. Our requests add ${RATES.tx.toFixed(0)} frames/s.`:
   `Load is high (${RATES.load.toFixed(0)}%); our share is ${RATES.tx.toFixed(0)} frames/s.`;
  if(RATES.errs>0||d.bus.tec||d.bus.rec){
   s+=d.bus.err_tx>d.bus.err_idle?' <span class="crit">Errors mostly follow our own frames.</span> The link from the master to the bus is marginal: check the transceiver ground to OBD pin 4/5, that its 120 Ω terminator is removed, and the stub length. The late sample point (Advanced) may also help.':
    ' <span class="warn">Errors also happen while we are quiet</span> — they are not ours (cranking and modules waking produce a few).';
  }else s+=' <span class="ok">No errors</span> — every frame is getting through.';
  if(d.bus.missed)s+=` <span class="muted">${d.bus.missed} frames dropped on our side (harmless).</span>`;
 }
 $('#bSay').innerHTML=s}
function metricOpts(sel,filter){return Object.keys(META).map(Number).filter(id=>(id&0xFF00)!==0x1F00&&(!filter||filter(id))).sort((a,b)=>a-b)
 .map(id=>`<option value="${id}" ${id===sel?'selected':''}>${esc(nm(id))}${un(id)?' ('+esc(un(id))+')':''}</option>`).join('')}
function mapOpen(id,ext,p){p=p||{};MAP={id,ext:!!ext};$('#mapCard').hidden=false;$('#mapId').textContent=hex(id,3);
 $('#mpS').value=p.s||0;$('#mpL').value=p.l||8;$('#mpK').value=p.k||1;$('#mpO').value=p.o||0;$('#mpBE').value=p.be?'1':'0';
 $('#mpSg').checked=!!p.sg;$('#mpM').innerHTML=metricOpts(p.m||0x010C);mapPrev();
 $('#mapCard').scrollIntoView({behavior:'smooth'})}
function mapClose(){MAP=null;$('#mapCard').hidden=true}
function mapPrev(){
 if(!MAP||!BUS)return;const c=BUS.census.find(x=>x.id===MAP.id&&!!x.ext===MAP.ext);if(!c)return;
 const raw=extract(hb(c.d),+$('#mpS').value,+$('#mpL').value||8,$('#mpBE').value==='1',$('#mpSg').checked);
 const m=+$('#mpM').value,v=raw*(parseFloat($('#mpK').value)||0)+(parseFloat($('#mpO').value)||0);
 $('#mpRaw').textContent=raw;$('#mpVal').textContent=v.toFixed(2)+' '+un(m);
 const cur=L&&L.metrics.find(x=>x.id===m);$('#mpRef').textContent=cur?`(currently ${fmt(m,cur.v)} from ${(SRC[cur.src]||['?'])[0]})`:''}
async function addSignal(sg){(C.signals=C.signals||[]).push(sg);await save()}
async function mapAdd(){
 if(!MAP||!LOADED)return;const m=+$('#mpM').value;
 await addSignal({name:nm(m).slice(0,15),can_id:MAP.id,ext:MAP.ext,start:+$('#mpS').value,len:+$('#mpL').value,
  be:$('#mpBE').value==='1',signed:$('#mpSg').checked,scale:parseFloat($('#mpK').value)||1,offset:parseFloat($('#mpO').value)||0,
  metric:m,mode:1,ref:0,learned:false});
 mapClose()}

/* ───────────── teach by doing ───────────── */
/*<teach-core>*/
/* The analysis is pure - payloads in, candidates out, no page state - so
   test_host/portal_tests.js runs it in Node. */
const POS_BOOL={P:0x1209,R:0x1208,N:0x120A,D:0x120B};   // a lever position with a bit of its own
const TEACH_POS={0x100F:'P R N D P',0x2039:'I S S# I',0x1005:'N 1 2 3 4 5 R N'};
/** What a position publishes: the gear lever sends its letter's code, SI-Drive
    the ECU's own numbers, the gear 0 for N and -1 for R. */
function posValue(m,tok,i){
 if(m===0x100F)return tok.charCodeAt(0);
 if(m===0x2039){const v={S:1,'S#':2,I:3}[tok.toUpperCase()];return v===undefined?i+1:v}
 if(m===0x1005){if(/^N$/i.test(tok))return 0;if(/^R$/i.test(tok))return -1}
 const v=parseInt(tok,10);return isFinite(v)?v:i}
/** A published position back as text, for the tables. */
function posLabel(m,v){
 if(m===0x100F)return v>32&&v<127?String.fromCharCode(v):String(v);
 if(m===0x2039)return {1:'S',2:'S#',3:'I'}[v]||String(v);
 if(m===0x1005)return v<0?'R':v===0?'N':String(v);
 return String(v)}
const bitOf=(b,bit)=>(b[bit>>3]>>(bit&7))&1;
/** Flips of @p bit across polls: from the master's flip counters (@p E, one
    64-entry array per poll, wrapping at 256) when it sent them - every flash
    counted - or else between successive payloads. */
function flipsOf(P,E,bit){
 let n=0;
 if(E&&E.length===P.length&&E.every(e=>e&&e.length===64)){for(let i=1;i<E.length;i++)n+=(E[i][bit]-E[i-1][bit])&255;return n}
 for(let i=1;i<P.length;i++)n+=bitOf(P[i],bit)!==bitOf(P[i-1],bit)?1:0;return n}
/** On/off: A and C are the OFF steps, B the ON one. */
function teachBits(id,A,B,C,EA,EB,EC,dlc){
 const res=[],p=(arr,bit)=>arr.filter(b=>bitOf(b,bit)).length/arr.length,haveC=C.length>=3;
 for(let bit=0;bit<dlc*8;bit++){
  const pa=p(A,bit),pb=p(B,bit),pc=haveC?p(C,bit):pa;
  // A real switch holds still within each step and flips between them; a
  // counter bit flips inside a step, so it is not steady anywhere.
  const steady=Math.min(...[pa,pb,pc].map(x=>Math.max(x,1-x)));
  const score=Math.min(Math.abs(pb-pa),Math.abs(pb-pc));
  if(score>=0.7&&steady>=0.85){res.push({id,s:bit,l:1,be:false,score,inv:pb<(pa+pc)/2});continue}
  // A turn signal flashes, so it is on for only part of the ON step and the
  // test above never passes. It sits still through both OFF steps and flips
  // again and again in the ON one.
  const rest=pa<0.5?0:1;
  if(Math.max(pa,1-pa)<0.9||Math.max(pc,1-pc)<0.9||(pc<0.5?0:1)!==rest)continue;
  const fa=flipsOf(A,EA,bit),fb=flipsOf(B,EB,bit),fc=haveC?flipsOf(C,EC,bit):fa;
  if(fa>1||fc>1||fb<3)continue;
  res.push({id,s:bit,l:1,be:false,score:0.7+0.3*Math.min(1,fb/8),inv:rest===1,blink:Math.max(1,Math.round(fb/2))})}
 return res}
/** A moving value: A held still, B moved through its range. */
function teachValues(id,A,B,dlc){
 const res=[],v=a=>{const mu=a.reduce((x,y)=>x+y,0)/a.length;return a.reduce((x,y)=>x+(y-mu)**2,0)/a.length};
 const judge=(st,len,be,sg)=>{
  const xa=A.map(b=>extract(b,st,len,be,sg)),xb=B.map(b=>extract(b,st,len,be,sg));
  const mn=Math.min(...xb),mx=Math.max(...xb),rng=mx-mn;if(rng<3)return null;
  // A physical value takes many values, not a couple (a sign byte flips 00/FF).
  if(new Set(xb).size<6)return null;
  // It must move far more while you move it than while you hold still…
  const moved=Math.log10((v(xb)+1)/(v(xa)+1));if(moved<1)return null;
  // …and it must change smoothly. The same bytes read in the wrong order,
  // or a counter beside them, jump around relative to their range.
  let jump=0;for(let i=1;i<xb.length;i++)jump+=Math.abs(xb[i]-xb[i-1]);jump/=Math.max(1,xb.length-1);
  const score=Math.min(moved,3)*Math.max(0,1-4*jump/rng);
  return score>1?{id,s:st,l:len,be,sg,score,mn,mx}:null};
 for(let k=0;k<dlc;k++){
  const fl=[[8*k,8,false],[8*k,8,true]];
  if(k+1<dlc)fl.push([8*k,16,false],[8*k+7,16,true]);
  let best=null;
  for(const [st,len,be] of fl){
   const u=judge(st,len,be,false),sgd=judge(st,len,be,true);
   /* A value that crosses zero wraps round as unsigned (10 → 65500), which
      looks like a huge swing. If the signed reading is a small, continuous
      range while the unsigned one spans nearly everything, it is signed. */
   const full=2**len;
   let pick=u;
   if(sgd&&(!u||(u.mx-u.mn>0.9*full&&sgd.mx-sgd.mn<0.5*full)))pick=sgd;
   if(pick&&(!best||pick.score>best.score))best=pick}
  if(best)res.push(best)}
 return res}
/** Positions: P[j] are the payloads while the lever sat in position j, named
    toks[j]. A name may come back ("P R N D P"): every visit to it must read
    the same. The lever is learnt as a combination - every bit that holds still
    in each position and reads differently between them, wherever it sits: side
    by side in one byte, scattered through it, or across bytes. */
function teachPositions(id,P,toks,dlc){
 const res=[],np=P.length;
 if(np<2||P.some(a=>a.length<3))return res;
 // The positions, once each, in the order first visited; and each visit's.
 const names=[],grp=[];
 for(let j=0;j<np;j++){const t=String(toks[j]||j).toUpperCase();let g=names.indexOf(t);if(g<0){g=names.length;names.push(t)}grp.push(g)}
 if(names.length<2)return res;
 const maj=[],stead=[];
 for(let bit=0;bit<dlc*8;bit++){
  const m=[],st=[];
  for(const a of P){const f=a.filter(b=>bitOf(b,bit)).length/a.length;m.push(f>=0.5?1:0);st.push(Math.max(f,1-f))}
  maj.push(m);stead.push(Math.min(...st))}
 // A lever bit holds still in every visit, reads the same whenever the lever
 // is back in a position, and differs between some positions. A bit that
 // only happened to change between two visits (a slow timer) fails the
 // second test when a position is visited twice.
 const byGroup=bit=>{const v=new Array(names.length).fill(-1);
  for(let j=0;j<np;j++){if(v[grp[j]]>=0&&v[grp[j]]!==maj[bit][j])return null;v[grp[j]]=maj[bit][j]}return v};
 const moving=[],gv={};
 for(let bit=0;bit<dlc*8;bit++){
  if(stead[bit]<0.85)continue;
  const v=byGroup(bit);if(!v||v.every(x=>x===v[0]))continue;
  moving.push(bit);gv[bit]=v}
 if(!moving.length)return res;
 // The combination: bit i of the code is moving[i]. Bits beside them that did
 // nothing here are left out, so they are free to change later.
 if(moving.length<=16){
  const codeOf=b=>moving.reduce((c,bit,i)=>c|(bitOf(b,bit)<<i),0);
  const codes=new Array(names.length).fill(null),hold=[];
  for(let j=0;j<np;j++){
   const cnt=new Map();P[j].forEach(b=>{const x=codeOf(b);cnt.set(x,(cnt.get(x)||0)+1)});
   let best=0,n=0;cnt.forEach((c,x)=>{if(c>n){n=c;best=x}});
   if(codes[grp[j]]===null)codes[grp[j]]=best;else if(codes[grp[j]]!==best){codes.fill(null);break}
   hold.push(n/P[j].length)}
  if(codes.every(c=>c!==null)&&new Set(codes).size===names.length&&Math.min(...hold)>=0.8)
   res.push({id,s:moving[0],l:moving.length,be:false,bits:moving.slice(),names:names.slice(),codes,
    score:1+Math.min(...hold)-0.01*moving.length})}
 // A position with a bit of its own, when no one frame tells every position
 // apart (reverse from one module, park from another): that position alone.
 for(let g=0;g<names.length;g++){
  const bm=POS_BOOL[names[g]];if(!bm)continue;
  for(const bit of moving){
   const v=gv[bit];
   if(v.every((x,k)=>k===g||x!==v[g]))res.push({id,s:bit,l:1,be:false,score:stead[bit],inv:v[g]===0,pos:g,names:names.slice(),metric:bm})}}
 return res}
/** Every candidate, best first, one per stretch of bytes. @p S maps each step
    to its polls; a poll maps an identifier to {b: payload, e: flip counters}. */
function teachFind(kind,S,steps,toks){
 const ids=new Set();(S[steps[0]]||[]).forEach(poll=>poll.forEach((x,id)=>ids.add(id)));
 const res=[];
 for(const id of ids){
  if(id>=0x7DF&&id<=0x7EF)continue;
  const ser=ph=>(S[ph]||[]).map(poll=>poll.get(id)).filter(Boolean);
  if(kind==='pos'){
   const P=steps.map(ph=>ser(ph).map(x=>x.b));
   if(P.some(a=>a.length<3))continue;
   res.push(...teachPositions(id,P,toks||[],Math.max(...P.flat().map(b=>b.length))));continue}
  const A=ser('A'),B=ser('B'),Cc=ser('C');
  if(A.length<3||B.length<3)continue;
  const dlc=Math.max(...A.map(x=>x.b.length),...B.map(x=>x.b.length)),pl=a=>a.map(x=>x.b),ed=a=>a.map(x=>x.e);
  if(kind==='bit')res.push(...teachBits(id,pl(A),pl(B),pl(Cc),ed(A),ed(B),ed(Cc),dlc));
  else res.push(...teachValues(id,pl(A),pl(B),dlc))}
 // A combination that tells every position apart, in any frame, is the
 // answer: one value per position is only for when there is none.
 if(kind==='pos'&&res.some(r=>r.codes)){const c=res.filter(r=>r.codes);res.length=0;res.push(...c)}
 res.sort((a,b)=>b.score-a.score);
 // One answer per stretch of bytes: drop anything overlapping a better hit.
 const bytesOf=r=>{if(r.bits)return[...new Set(r.bits.map(b=>b>>3))];const k=r.s>>3;return r.l===1?[k]:r.s%8+r.l<=8?[k]:[k,k+1]};
 const out=[];
 for(const r of res){
  if(r.l!==1&&out.some(t=>t.id===r.id&&t.l!==1&&bytesOf(t).some(b=>bytesOf(r).includes(b))))continue;
  out.push(r);if(out.length>=8)break}
 return out}
/*</teach-core>*/
const isSwitch=id=>(id&0xFF00)===0x1200||(id&0xFF00)===0x2100||id===0x0101;
const TEACH_FIRST=id=>(id&0xFF00)===0x1200||(id>=0x1005&&id<=0x10FF)||(id&0xFF00)===0x3000||(id&0xFF00)===0x2100;
let TEACH=null,TRES=[],TTOK=[];
$('#tM').innerHTML='<optgroup label="Body, chassis and custom">'+metricOpts(0x1201,TEACH_FIRST)+'</optgroup><optgroup label="Everything else">'+metricOpts(0,id=>!TEACH_FIRST(id))+'</optgroup>';
function teachKindShown(){$('#tPRow').hidden=$('#tK').value!=='pos'}
$('#tM').onchange=()=>{const m=+$('#tM').value;
 $('#tK').value=TEACH_POS[m]?'pos':isSwitch(m)?'bit':'val';
 if(TEACH_POS[m])$('#tP').value=TEACH_POS[m];teachKindShown()};
$('#tK').onchange=teachKindShown;
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
async function teachStart(){
 const m=+$('#tM').value,kind=$('#tK').value;
 TTOK=kind==='pos'?$('#tP').value.trim().split(/[\s,]+/).filter(Boolean).slice(0,8):[];
 if(kind==='pos'&&TTOK.length<2){toast('List at least two positions');return}
 const phases=kind==='bit'?[['A','Leave it OFF',4000],['B','Turn it ON now, and hold it (flashing is fine)',4000],['C','Turn it OFF again',4000]]:
  kind==='pos'?TTOK.map((t,i)=>['p'+i,`Put it in <b>${esc(t)}</b> and hold it there`,5000]):
  [['A','Hold it still',3000],['B','Move it slowly through its whole range, back and forth',9000]];
 // Moving a lever takes a moment: its first second and a half is not sampled.
 const skip=kind==='pos'?1500:800;
 TEACH={m,kind,S:{}};phases.forEach(([k])=>TEACH.S[k]=[]);$('#tBtn').disabled=true;$('#tRes').innerHTML='';
 for(const [k,txt,dur] of phases){
  const t0=Date.now();
  while(Date.now()-t0<dur){
   $('#tStep').innerHTML=`<div class="step">${txt}</div><span class="muted">${Math.ceil((dur-(Date.now()-t0))/1000)} s</span>`;
   try{const b=await fetch('/api/bus?e=1').then(r=>r.json());
    if(Date.now()-t0>skip)TEACH.S[k].push(new Map(b.census.filter(c=>!c.ext).map(c=>[c.id,{b:hb(c.d||''),e:c.e?hb(c.e):null}])))}catch(e){}
   await sleep(60)}}
 $('#tStep').innerHTML='<div class="step">Done</div>';
 teachAnalyze(phases.map(p=>p[0]));TEACH=null;$('#tBtn').disabled=false}
function teachAnalyze(steps){
 const {S,kind,m}=TEACH;
 TRES=teachFind(kind,S,steps,TTOK);
 if(!TRES.length){$('#tRes').innerHTML=`<p class="say">Nothing on the bus followed that. ${kind==='pos'?'Hold each position until the next is asked for, or the lever may not be broadcast.':'Try again with a clearer on/off, or the value may not be broadcast.'}</p>`;return}
 const pat=(c,n)=>Array.from({length:n},(_,i)=>(c>>i)&1).join('');
 const say=r=>r.codes?r.names.map((t,g)=>`${esc(t)} <span class="mono">${pat(r.codes[g],r.bits.length)}</span>`).join(' • '):
  r.pos!==undefined?`on only in ${esc(r.names[r.pos])}${r.inv?' (inverted)':''} - as ${esc(nm(r.metric))}`:
  r.l===1?(r.blink?`flashed ${r.blink} time${r.blink>1?'s':''}${r.inv?' (inverted)':''}`:`followed ${(Math.min(1,r.score)*100).toFixed(0)}%${r.inv?' (inverted)':''}`):
  `raw ${r.mn} … ${r.mx} while moving`;
 $('#tRes').innerHTML='<p class="say" style="margin-top:8px">These followed you, best first:</p>'+TRES.map((r,i)=>
  `<div class="lr"><div class="lt"><span>${field({id:r.id,s:r.s,l:r.l,be:r.be,bits:r.bits})}${r.sg?' <span class="muted">signed</span>':''}</span>
   <button class="b s p" onclick="teachUse(${i})">${r.l===1||r.codes?'Use':'Use…'}</button></div>
   <div class="ls"><span>${say(r)}</span></div></div>`).join('')}
async function teachUse(i){
 const r=TRES[i],m=+$('#tM').value,bm=r.metric||m;
 if((C.signals||[]).some(g=>g.metric===bm&&g.can_id===r.id&&g.start===r.s&&g.len===r.l&&String(g.bits||'')===String(r.bits||''))){toast(`${nm(bm)} is already read from there`);return}
 if(r.codes){
  await addSignal({name:nm(m).slice(0,15),can_id:r.id,ext:false,start:r.s,len:r.l,be:false,signed:false,scale:1,offset:0,
   metric:m,mode:1,ref:0,learned:false,bits:r.bits,map:r.codes.map((c,g)=>[c,posValue(m,r.names[g],g)])});
  toast(`${nm(m)} is now read from the bus`);$('#tRes').innerHTML='';return}
 if(r.l===1){
  await addSignal({name:nm(bm).slice(0,15),can_id:r.id,ext:false,start:r.s,len:1,be:false,signed:false,
   scale:r.inv?-1:1,offset:r.inv?1:0,metric:bm,mode:1,ref:0,learned:false});
  toast(`${nm(bm)} is now read from the bus`);if(r.metric===undefined)$('#tRes').innerHTML='';return}
 BUS=BUS||{census:[]};mapOpen(r.id,0,{s:r.s,l:r.l,be:r.be,sg:r.sg,m});
 toast('Set the scale so the value reads right, then Use this value')}

/* ───────────── sources ───────────── */
const SMODE=['Off','On','Checking','Verified','Rejected'];
function curVal(id){const m=L&&L.metrics.find(x=>x.id===id);if(!m)return'<span class="muted">—</span>';
 const s=SRC[m.src]||['?','no'];return`${fmt(id,m.v)} <span class="muted">${esc(un(id))}</span> <span class="tag ${s[1]}">${s[0]}</span>`}
function renderSources(){
 if(!LOADED)return;const adv=$('#adv').checked;
 const sig=C.signals||[];
 $('#sHead').innerHTML=adv?'<tr><th>Mode</th><th>Metric</th><th>ID</th><th>Start</th><th>Len</th><th>BE</th><th>Sgn</th><th>Scale</th><th>Offset</th><th>Verify vs</th><th></th></tr>':
  '<tr><th>Value</th><th>Where</th><th>Status</th><th>Now</th><th></th></tr>';
 $('#sTab').innerHTML=sig.map((s,i)=>{
  const st=s.mode===3?'<span class="ok">Verified</span>':s.mode===1?'<span class="ok">In use</span>':s.mode===2?'<span class="acc">Checking…</span>':s.mode===4?'<span class="crit">Rejected</span>':'<span class="muted">Off</span>';
  const by=s.learned?' <span class="muted">(learned)</span>':'';
  if(!adv)return`<tr><td>${esc(nm(s.metric))}${by}</td><td>${field({id:s.can_id,s:s.start,l:s.len,be:s.be,map:s.map,m:s.metric,bits:s.bits})}</td><td>${st}</td><td>${curVal(s.metric)}</td>
   <td><button class="x" title="Remove" onclick="C.signals.splice(${i},1);renderSources();setDirty(true)">✕</button></td></tr>`;
  return`<tr><td><select onchange="C.signals[${i}].mode=+this.value">${SMODE.map((t,k)=>`<option value="${k}" ${s.mode===k?'selected':''}>${t}</option>`).join('')}</select></td>
   <td><select onchange="C.signals[${i}].metric=+this.value">${metricOpts(s.metric)}</select></td>
   <td><input value="${hex(s.can_id,3)}" onchange="C.signals[${i}].can_id=parseInt(this.value,16)||0"></td>
   ${s.bits&&s.bits.length?`<td colspan="2" class="muted">bits ${s.bits.join(' ')}</td>`:
   `<td><input type="number" value="${s.start}" onchange="C.signals[${i}].start=+this.value"></td>
   <td><input type="number" value="${s.len}" onchange="C.signals[${i}].len=+this.value"></td>`}
   <td><input type="checkbox" ${s.be?'checked':''} onchange="C.signals[${i}].be=this.checked"></td>
   <td><input type="checkbox" ${s.signed?'checked':''} onchange="C.signals[${i}].signed=this.checked"></td>
   ${s.map&&s.map.length?`<td colspan="2" class="muted">${s.map.map(p=>`${p[0]}→${esc(posLabel(s.metric,p[1]))}`).join(' ')}</td>`:
   `<td><input value="${s.scale}" onchange="C.signals[${i}].scale=+this.value"></td>
   <td><input value="${s.offset}" onchange="C.signals[${i}].offset=+this.value"></td>`}
   <td><input value="${s.ref?hex(s.ref,4):''}" onchange="C.signals[${i}].ref=parseInt(this.value,16)||0"></td>
   <td><button class="x" onclick="C.signals.splice(${i},1);renderSources();setDirty(true)">✕</button></td></tr>`}).join('')||
  '<tr><td colspan="5" class="empty">Nothing yet — the learner adds values here as it finds them.</td></tr>';
 const known=L&&L.pid_sup,showAllO=$('#oAll').checked||!known;
 const pids=(C.pids||[]).map((p,i)=>[p,i]).filter(([p])=>showAllO||known.indexOf(p.pid)>=0);
 $('#oHead').innerHTML=`<tr><th>Ask</th><th>PID</th><th>Value</th><th>Now</th><th>Every</th>${adv?'<th>Gives</th>':''}</tr>`;
 $('#oTab').innerHTML=pids.map(([p,i])=>{const sup=!known?'':known.indexOf(p.pid)>=0?'':' <span class="muted">(not supported)</span>';
  const m=(p.metrics||[])[0];
  return`<tr><td><input type="checkbox" ${p.enabled?'checked':''} onchange="C.pids[${i}].enabled=this.checked"></td>
   <td class="muted">${hex(p.pid,2)}</td><td>${esc(p.name||'')}${sup}</td><td>${m?curVal(m):''}</td>
   <td><input type="number" value="${p.period}" style="width:64px" onchange="C.pids[${i}].period=+this.value"> <span class="muted">ms</span></td>
   ${adv?`<td class="muted">${(p.metrics||[]).map(x=>esc(nm(x))).join(', ')}</td>`:''}</tr>`}).join('')||
  '<tr><td colspan="5" class="empty">The ECU has not been asked yet.</td></tr>';
 const sup=L&&L.ssm_sup&&L.ssm_sup.length===(C.ssm||[]).length?L.ssm_sup:null,showAllM=$('#mAll').checked||!sup;
 const ssm=(C.ssm||[]).map((e,i)=>[e,i]).filter(([e,i])=>showAllM||sup[i]!==0);
 $('#mHead').innerHTML=adv?'<tr><th>Ask</th><th>Name</th><th>Address</th><th>Bytes</th><th>Sgn</th><th>Scale</th><th>Offset</th><th>Metric</th><th>Every</th><th></th></tr>':
  '<tr><th>Ask</th><th>Value</th><th>Now</th><th>Every</th></tr>';
 $('#mTab').innerHTML=ssm.map(([e,i])=>{
  const ns=sup&&sup[i]===0?' <span class="muted">(not supported)</span>':'';
  const per=`<input type="number" value="${e.period||0}" style="width:64px" onchange="C.ssm[${i}].period=+this.value"> <span class="muted">ms</span>`;
  if(!adv)return`<tr><td><input type="checkbox" ${e.enabled?'checked':''} onchange="C.ssm[${i}].enabled=this.checked"></td>
   <td>${esc(e.name||nm(e.metric))}${ns}</td><td>${curVal(e.metric)}</td><td>${per}</td></tr>`;
  return`<tr><td><input type="checkbox" ${e.enabled?'checked':''} onchange="C.ssm[${i}].enabled=this.checked"></td>
   <td><input value="${esc(e.name||'')}" onchange="C.ssm[${i}].name=this.value"></td>
   <td><input value="${hex(e.addr,6)}" style="min-width:84px" onchange="C.ssm[${i}].addr=parseInt(this.value,16)||0"></td>
   <td><input type="number" min="1" max="2" value="${e.bytes}" onchange="C.ssm[${i}].bytes=+this.value"></td>
   <td><input type="checkbox" ${e.signed?'checked':''} onchange="C.ssm[${i}].signed=this.checked"></td>
   <td><input value="${e.scale}" onchange="C.ssm[${i}].scale=+this.value"></td>
   <td><input value="${e.offset}" onchange="C.ssm[${i}].offset=+this.value"></td>
   <td><select onchange="C.ssm[${i}].metric=+this.value">${metricOpts(e.metric)}</select></td><td>${per}</td>
   <td><button class="x" onclick="C.ssm.splice(${i},1);renderSources();setDirty(true)">✕</button></td></tr>`}).join('')||
  '<tr><td colspan="4" class="empty">Nothing to show.</td></tr>';
 updateSourceText()}
function updateSourceText(){
 if(!L)return;const o=L.obd,s=L.ssm;
 $('#oSub').textContent=o.answers===1?`ECU supports ${o.supported} PIDs · ${o.active?'active':'idle'} · ${o.physical?'engine ECU only':'all ECUs'}`:o.answers===2?'the ECU does not answer':'not asked yet';
 $('#mSub').textContent=(s.init?`ECU supports ${s.supported} of ${s.total} · ${s.active?(s.rate||0).toFixed(0)+' requests/s, '+s.batch+' values each':'idle — OBD-II covers it'}`:s.answers===2?'the ECU does not answer':'waits for OBD-II')+
  (s.refused?` · ${s.refused} refused by the ECU`:'')+(s.last_err?` · last problem: ${s.last_err}`:'');
 $('#sEcu').innerHTML=s.ecu_id?`ECU <b>${esc(s.ecu_id)}</b> · SYS ${esc(s.sys_id)} · ${s.flags} capability bytes${s.init?'':' (remembered)'} · this master ${esc(L.mac)}`:`This master ${esc(L.mac)}`}

/* ───────────── diagnostics: the evidence log ───────────── */
const EVN={0:'Boot',1:'Bus up',2:'Bus settled',3:'Bus guard trip',4:'Receive-error guard',5:'Bus-off',6:'Sleep',7:'Resumed'};
const RST={0:'unknown',1:'power-on',2:'external reset',3:'software reset',4:'panic',5:'interrupt watchdog',6:'task watchdog',7:'watchdog',8:'wake from deep sleep',9:'brown-out',10:'SDIO'};
const ACTS=a=>{const p=[];if(a&4)p.push('listen-only');else if(a&8)p.push('free to transmit');if(a&1)p.push('OBD-II polling');if(a&2)p.push('settling');
 p.push('mode '+(['auto','SSM2','OBD-II','both','silent'][(a>>4)&15]||'?'));return p.join(', ')};
function evDetail(e){
 if(e.t===0)return`reset: <b class="${e.a===9?'crit':''}">${RST[e.a]||e.a}</b> · config v${e.b}`;
 if(e.t===1)return`bus session ${e.a}`;
 if(e.t===2)return`after ${e.a} s of listening`;
 if(e.t===3)return`trip ${e.a}${e.b?' → listen-only':''}`;
 if(e.t===4)return`REC ${e.a}, ${e.b} receive errors in the window → listen-only`;
 if(e.t===5)return`TEC ${e.a}${e.b?' → listen-only (bus guard off)':''}`;
 return''}
async function pollEvlog(){
 if(TAB!=='diag')return;
 let d;try{d=await fetch('/api/evlog').then(r=>r.json())}catch(e){return}
 const ev=d.events||[],rows=ev.slice().reverse();
 $('#evN').textContent=ev.length+' events';
 $('#evTab').innerHTML=rows.map(e=>`<tr><td class="muted">${(e.ms/1000).toFixed(0)} s</td><td><b>${EVN[e.t]||e.t}</b></td><td>${evDetail(e)}</td><td class="muted">${ACTS(e.act)}</td></tr>`).join('')||
  '<tr><td colspan="4" class="empty">Nothing recorded yet.</td></tr>';
 const boots=ev.filter(e=>e.t===0),brown=boots.filter(e=>e.a===9).length,rx=ev.filter(e=>e.t===4).length,tx=ev.filter(e=>e.t===3).length,off=ev.filter(e=>e.t===5).length;
 let s='';
 if(brown)s+=`<span class="crit">${brown} brown-out reset${brown>1?'s':''}</span> — the master rebooted on a voltage dip (cranking); the settle wait and the TX hold matter. `;
 if(rx)s+=`<span class="crit">${rx} receive-error guard trip${rx>1?'s':''}</span> — our controller was corrupting other modules' frames on a marginal link: check the ground, the stub length and try the late sample point. `;
 if(tx)s+=`<span class="warn">${tx} bus-guard trip${tx>1?'s':''}</span> — errors followed our own requests. `;
 if(off)s+=`<span class="crit">${off} bus-off</span>. `;
 if(!s)s=`<span class="ok">${boots.length} boot${boots.length===1?'':'s'}, no brown-out, no guard trip, no bus-off</span> — if the lamp still came on, the master was not the one disturbing the bus.`;
 $('#evSay').innerHTML=s}

/* ───────────── polling ───────────── */
async function poll(){
 let d;try{d=await fetch('/api/live').then(r=>r.json())}catch(e){
  const c=$('#conn');c.className='pill crit';c.lastChild.textContent='offline';return}
 const now=Date.now();
 // The master restarted (its counters went back): no rate from across that.
 if(PREV&&(d.bus.rx<PREV.rx||d.bus.tx<PREV.tx||d.bus.err<PREV.err))PREV=null;
 if(PREV){const dt=(now-PREV.t)/1000;if(dt>0){
  RATES.fps=(d.bus.rx-PREV.rx)/dt;RATES.tx=(d.bus.tx-PREV.tx)/dt;RATES.errs=d.bus.err-PREV.err;
  RATES.load=100*((d.bus.bits-PREV.bits)+(d.bus.tx-PREV.tx)*125)/(dt*(d.bus.bitrate||500)*1000)}}
 PREV={t:now,rx:d.bus.rx,tx:d.bus.tx,err:d.bus.err,bits:d.bus.bits};
 L=d;
 // Pick up what the learner and verifier did to the signal table.
 const lsig=(d.learn||[]).filter(x=>x.st>=3).map(x=>x.m+':'+x.st).join();
 if(LOADED&&(lsig!==LSIG||now-LLOAD>30000)){LSIG=lsig;LLOAD=now;load(true)}
 const c=$('#conn');
 if(!d.bus.alive){c.className='pill warn';c.lastChild.textContent='no CAN'}
 else if(d.silent){c.className='pill ok';c.lastChild.textContent='listening only'}
 else if(d.settle){c.className='pill ok';c.lastChild.textContent=`settling · ${Math.ceil(d.settle/1000)} s`}
 else if(d.guard.pause||d.guard.silent){c.className='pill warn';c.lastChild.textContent='requests paused'}
 else{c.className='pill ok';c.lastChild.textContent='live'}
 if(TAB==='dash')renderDash();
 if(TAB==='bus')renderBusTiles();
 if(TAB==='src'||TAB==='set')updateSourceText()}
load();poll();setInterval(poll,1000);setInterval(pollBus,1000);setInterval(pollEvlog,3000);
</script></body></html>
)HTML";

void WebPortal::begin() {
    if (!Cfg.portalOn) {
        log_i("portal disabled by configuration");
        return;
    }

    /*
     * AP mode, pinned to the telemetry channel.
     *
     * softAP's channel argument is not cosmetic here: the ESP-NOW peers listen
     * on Cfg.wifiChannel, and a radio can only be on one channel at a time. Let
     * the AP choose its own and every display goes silent the moment someone
     * connects to the portal — with no error anywhere to explain it.
     */
    WiFi.mode(WIFI_AP);
    bool secured = strlen(Cfg.apPass) >= 8;
    const char *ssid = Cfg.apSsid;
    if (!WiFi.softAP(ssid, secured ? Cfg.apPass : nullptr, Cfg.wifiChannel)) {
        // A name or key the AP will not take must not lock the only way into
        // these settings away: the built-in name, open, instead.
        log_e("portal: AP '%s' would not start - '%s', open, instead", ssid,
              MasterConfig::AP_SSID_DEFAULT);
        ssid = MasterConfig::AP_SSID_DEFAULT;
        secured = false;
        WiFi.softAP(ssid, nullptr, Cfg.wifiChannel);
    }

    s_dns.start(53, "*", WiFi.softAPIP());
    setupRoutes();
    s_server.begin();
    _running = true;

    log_i("portal up: SSID '%s' (%s) at %s, channel %u", ssid,
          secured ? "WPA2" : "open", WiFi.softAPIP().toString().c_str(),
          Cfg.wifiChannel);
}

void WebPortal::loop() {
    if (!_running) return;
    s_dns.processNextRequest();
    s_server.handleClient();
}

/**
 * @brief Hands the client 1 KB at a time. ArduinoJson writes byte by byte,
 *        and a TCP write per byte would crawl.
 */
class ClientBuffer : public Print {
public:
    explicit ClientBuffer(WiFiClient &c) : _c(c) {}
    ~ClientBuffer() { push(); }
    size_t write(uint8_t b) override {
        _b[_n++] = b;
        if (_n == sizeof(_b)) push();
        return 1;
    }
    size_t write(const uint8_t *p, size_t n) override {
        for (size_t i = 0; i < n; i++) write(p[i]);
        return n;
    }
    void push() { if (_n) { _c.write(_b, _n); _n = 0; } }
private:
    WiFiClient &_c;
    uint8_t     _b[1024];
    size_t      _n = 0;
};

/**
 * @brief Stream a document to the client. Never built as one String first:
 *        the full configuration runs to tens of kilobytes, and one
 *        allocation that size can fail on a fragmented heap - which the page
 *        would see as a settings load that silently never finishes.
 */
static void sendJson(JsonDocument &doc) {
    s_server.setContentLength(measureJson(doc));
    s_server.send(200, "application/json", "");
    WiFiClient client = s_server.client();
    ClientBuffer out(client);
    serializeJson(doc, out);
}

static void sendOk() { s_server.send(200, "application/json", "{\"ok\":true}"); }

void WebPortal::setupRoutes() {
    s_server.on("/", HTTP_GET, []() {
        s_server.send_P(200, "text/html", PORTAL_HTML);
    });

    s_server.on("/api/config", HTTP_GET, []() {
        JsonDocument doc;
        Cfg.toJson(doc);
        sendJson(doc);
    });

    s_server.on("/api/config", HTTP_POST, []() {
        JsonDocument doc;
        if (deserializeJson(doc, s_server.arg("plain")) ||
            !Cfg.fromJson(doc.as<JsonVariantConst>(), true)) {   // not an object
            s_server.send(400, "application/json", "{\"error\":\"bad json\"}");
            return;
        }
        const bool ok = Cfg.save();
        s_server.send(ok ? 200 : 500, "application/json",
                      ok ? "{\"ok\":true}" : "{\"error\":\"fs write\"}");
    });

    /*
     * Everything the Dashboard, Sources and Settings tabs show, in one
     * document polled once a second. Totals rather than rates: the page
     * divides by its own poll interval, so nothing here has to keep history.
     */
    s_server.on("/api/live", HTTP_GET, []() {
        MasterStats st;
        masterGetStats(st);
        Ssm2Status ss;
        ssm2GetStatus(ss);

        JsonDocument doc;
        doc["uptime"] = millis() / 1000;
        doc["heap"]   = ESP.getFreeHeap();
        // The MAC the displays see is the one the broadcast leaves from, and
        // with the portal up that is the AP interface, not the station's.
        doc["mac"]    = Portal.running() ? WiFi.softAPmacAddress() : WiFi.macAddress();
        doc["mode"]   = Cfg.diagMode;
        doc["silent"] = st.silent;
        doc["settle"] = st.settleMs;
        doc["night"]  = st.night;

        JsonObject b = doc["bus"].to<JsonObject>();
        b["alive"]   = st.busAlive;
        b["off"]     = st.busOff;
        b["rx"]      = st.canRx;
        b["tx"]      = st.canTx;
        b["bits"]    = st.busBitsRx;
        b["bitrate"] = st.bitrate;
        b["tec"]     = st.tec;
        b["rec"]     = st.rec;
        b["err"]     = st.busErrors;
        b["missed"]  = st.rxMissed + st.rxOverrun;
        b["err_tx"]  = st.errWhileTx;
        b["err_idle"] = st.errIdle;
        {
            // What the controller says the errors are: code, mode, count.
            ErrKindView ek[6];
            const size_t nk = masterErrorKinds(ek, 6);
            JsonArray ka = b["kinds"].to<JsonArray>();
            for (size_t i = 0; i < nk; i++) {
                JsonObject k = ka.add<JsonObject>();
                k["c"] = ek[i].code;
                k["s"] = ek[i].silent;
                k["n"] = ek[i].errors;
            }
        }

        {
            // Where the errors come from: missed frames by mode, the radio.
            JsonObject lk = b["link"].to<JsonObject>();
            JsonArray rx = lk["rx"].to<JsonArray>(), er = lk["err"].to<JsonArray>();
            JsonArray mi = lk["miss"].to<JsonArray>(), ex = lk["exp"].to<JsonArray>();
            for (int m = 0; m < 2; m++) {
                rx.add(st.rxByMode[m]); er.add(st.errByMode[m]);
                mi.add(st.missed[m]);   ex.add(st.expected[m]);
            }
            lk["near"] = st.errNearRadio;
            lk["chk"]  = st.errRadioChecked;
            lk["hot"]  = st.radioHotPermille;
            MissView mv[6];
            const size_t nm = masterMissedIds(mv, 6);
            JsonArray ia = lk["ids"].to<JsonArray>();
            for (size_t i = 0; i < nm; i++) {
                JsonObject o = ia.add<JsonObject>();
                o["i"] = mv[i].id;
                o["x"] = mv[i].extd;
                o["p"] = mv[i].periodMs;
                o["m"] = mv[i].missed[0] + mv[i].missed[1];
                o["e"] = mv[i].expected[0] + mv[i].expected[1];
            }
        }

        JsonObject g = doc["guard"].to<JsonObject>();
        g["trips"]  = st.guardTrips;
        g["pause"]  = st.guardPauseMs;
        g["silent"] = st.guardSilent;
        g["rx_silent"] = st.rxGuardSilent;
        {
            static CensusView cen[128];
            b["ids"] = masterCensusRaw(cen, 128);
        }

        JsonObject o = doc["obd"].to<JsonObject>();
        o["answers"]   = st.obdAnswers;
        o["active"]    = st.obdActive;
        o["physical"]  = st.obdPhysical;
        o["supported"] = st.obdSupported;
        o["rx"]        = st.obdRx;

        JsonObject so = doc["ssm"].to<JsonObject>();
        so["answers"]   = st.ssmAnswers;
        so["init"]      = ss.initOk;
        so["active"]    = ss.active;
        so["ecu_id"]    = ss.initOk ? ss.ecuId : Cfg.ecuId;
        so["sys_id"]    = ss.initOk ? ss.sysId : Cfg.ecuSysId;
        so["flags"]     = ss.initOk ? ss.flagCount : (uint8_t)(strlen(Cfg.ecuFlags) / 2);
        so["supported"] = ss.supported;
        so["total"]     = ss.total;
        so["rate"]      = ss.exchPerSec;
        so["ok"]        = ss.responses;
        so["err"]       = ss.errors;
        so["nrc"]       = ss.nrcs;
        so["refused"]   = ss.refused;
        so["batch"]     = ss.batch;
        so["last_err"]  = ss.lastErr;

        JsonObject e = doc["esp"].to<JsonObject>();
        {
            static uint32_t lastTx = 0, lastMet = 0, lastMs = 0;
            const uint32_t now = millis();
            float fps = 0, mps = 0;
            if (lastMs && now > lastMs) {
                const float dt = (now - lastMs) / 1000.0f;
                fps = (st.espTx - lastTx) / dt;
                mps = (st.espMetrics - lastMet) / dt;
            }
            lastTx = st.espTx; lastMet = st.espMetrics; lastMs = now;
            e["fps"]  = fps;
            e["mps"]  = mps;
            e["fail"] = st.espFail;
        }

        // Every slot, plus every value already read from the bus.
        static LearnView lv[32 + MAX_RT_SIGNALS];
        const size_t nl = learnerStatus(lv, sizeof(lv) / sizeof(lv[0]));
        JsonArray la = doc["learn"].to<JsonArray>();
        for (size_t i = 0; i < nl; i++) {
            JsonObject x = la.add<JsonObject>();
            x["m"]   = lv[i].metric;
            x["st"]  = lv[i].state;
            x["p"]   = lv[i].progress;
            x["fit"] = lv[i].fit;
            x["n"]   = lv[i].samples;
            x["id"]  = lv[i].canId;
            x["s"]   = lv[i].start;
            x["l"]   = lv[i].len;
            x["be"]  = lv[i].be;
        }
        uint16_t lf = 0, lb = 0;
        learnerCounts(lf, lb);
        doc["learn_f"] = lf;
        doc["learn_b"] = lb;

        JsonArray sup = doc["ssm_sup"].to<JsonArray>();
        Cfg.lock();
        for (const auto &x : Cfg.ssm) sup.add(ssm2Supported(x));
        Cfg.unlock();
        if (st.obdAnswers == 1) {
            JsonArray ps = doc["pid_sup"].to<JsonArray>();
            for (uint16_t p = 1; p < 256; p++)
                if (masterPidSupported((uint8_t)p)) ps.add(p);
        }

        static MetricViewM mv[192];
        const size_t nm = masterGetMetrics(mv, 192);
        JsonArray ma = doc["metrics"].to<JsonArray>();
        for (size_t i = 0; i < nm; i++) {
            JsonObject m = ma.add<JsonObject>();
            m["id"]  = mv[i].id;
            m["v"]   = mv[i].value;
            m["age"] = mv[i].ageMs;
            m["src"] = mv[i].source;
        }
        sendJson(doc);
    });

    /* The Bus tab's frame list, polled only while that tab is open. */
    s_server.on("/api/bus", HTTP_GET, []() {
        JsonDocument doc;
        static CensusView cen[128];
        const size_t nc = masterGetCensus(cen, 128);
        JsonArray ca = doc["census"].to<JsonArray>();
        char hx[17], chx[17];
        // ?e=1 (teach by doing): each bit's flip counter as well, 128 hex digits.
        const bool edges = s_server.arg("e") == "1";
        char ehx[129];
        uint8_t ed[64];
        for (size_t i = 0; i < nc; i++) {
            JsonObject o = ca.add<JsonObject>();
            o["id"]    = cen[i].id;
            o["count"] = cen[i].count;
            o["ext"]   = cen[i].extd;
            o["age"]   = cen[i].ageMs;
            for (uint8_t k = 0; k < cen[i].dlc; k++) {
                snprintf(hx + 2 * k, 3, "%02X", cen[i].data[k]);
                snprintf(chx + 2 * k, 3, "%02X", cen[i].changed[k]);
            }
            hx[2 * cen[i].dlc] = chx[2 * cen[i].dlc] = '\0';
            o["d"] = hx;
            o["c"] = chx;
            if (edges && masterCensusEdges(cen[i].id, cen[i].extd, ed)) {
                for (uint8_t k = 0; k < 64; k++) snprintf(ehx + 2 * k, 3, "%02X", ed[k]);
                o["e"] = ehx;
            }
        }
        sendJson(doc);
    });

    /*
     * The evidence log: boot and reset reasons, the bus coming up and settling,
     * every guard trip with the error counters and what the master was
     * transmitting at the time, bus-off and sleep. Persisted on the master's
     * flash, so one drive can say which fault it is.
     */
    s_server.on("/api/evlog", HTTP_GET, []() {
        JsonDocument doc;
        static EvView ev[48];
        const size_t n = masterEventLog(ev, 48);
        doc["now"] = millis();
        JsonArray a = doc["events"].to<JsonArray>();
        for (size_t i = 0; i < n; i++) {
            JsonObject o = a.add<JsonObject>();
            o["ms"]  = ev[i].ms;
            o["t"]   = ev[i].type;
            o["n"]   = masterEventName(ev[i].type);
            o["act"] = ev[i].act;
            o["a"]   = ev[i].a;
            o["b"]   = ev[i].b;
        }
        sendJson(doc);
    });
    s_server.on("/api/evlog/clear", HTTP_POST, []() { masterEventLogClear(); sendOk(); });

    s_server.on("/api/census/reset", HTTP_POST, []() { masterResetCensus(); sendOk(); });
    s_server.on("/api/ssm/reinit",   HTTP_POST, []() { ssm2Reinit(); sendOk(); });
    s_server.on("/api/learn/reset",  HTTP_POST, []() {
        learnerReset();
        masterResetSignalVerify();
        sendOk();
    });
    s_server.on("/api/learn/forget", HTTP_POST, []() { learnerForget(); sendOk(); });
    s_server.on("/api/guard/reset",  HTTP_POST, []() { diagGuardReset(); sendOk(); });
    // Per value: "Listen now" (use the best match so far) and "Request instead".
    s_server.on("/api/learn/accept", HTTP_POST, []() {
        learnerAccept((uint16_t)s_server.arg("m").toInt());
        sendOk();
    });
    s_server.on("/api/learn/unlearn", HTTP_POST, []() {
        learnerUnlearn((uint16_t)s_server.arg("m").toInt());
        sendOk();
    });

    // "Safe bus settings": the bus behaviour alone back to its defaults
    // (MasterConfig::loadBusDefaults) - learned values, pacing, displays and
    // Wi-Fi stay. Rebooted like a factory reset: the bit timing needs it.
    s_server.on("/api/bus_defaults", HTTP_POST, []() {
        Cfg.loadBusDefaults();
        Cfg.save();
        masterBeforeRestart();
        sendOk();
        delay(300);
        ESP.restart();
    });

    s_server.on("/api/defaults", HTTP_POST, []() {
        Cfg.loadDefaults();
        Cfg.save();
        masterBeforeRestart();
        sendOk();
        delay(300);
        ESP.restart();
    });

    s_server.on("/api/reboot", HTTP_POST, []() {
        masterBeforeRestart();
        sendOk();
        delay(300);
        ESP.restart();
    });

    // Captive-portal probes and anything unknown land on the page itself.
    s_server.onNotFound([]() {
        s_server.sendHeader("Location",
                            String("http://") + WiFi.softAPIP().toString() + "/",
                            true);
        s_server.send(302, "text/plain", "");
    });
}
