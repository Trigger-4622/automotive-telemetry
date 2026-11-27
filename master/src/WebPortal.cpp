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

#include <LittleFS.h>

#include "DiagCodes.h"
#include "FwUpdate.h"
#include "DriveLog.h"
#include "RawRec.h"
#include "BodyUnit.h"
#include "Learner.h"
#include "MasterConfig.h"
#include "MasterTelemetry.h"
#include "Ssm2.h"

WebPortal Portal;

/**
 * The library's server handles one connection at a time, and one that has
 * connected but not yet sent its request holds every other one up for up to
 * HTTP_MAX_DATA_WAIT (5 s). Browsers open such connections ahead of need
 * (Chrome's preconnect), so the page's requests queued behind an idle socket
 * and the portal froze for seconds at a time. Here that socket is let go as
 * soon as another connection is waiting; a browser opens a fresh one when it
 * has something to ask.
 */
class PortalServer : public WebServer {
public:
    using WebServer::WebServer;
    void handleClient() {
        if (_currentStatus == HC_WAIT_READ && millis() - _statusChange >= IDLE_MS &&
            !_currentClient.available() && _server.hasClient()) {
            _currentClient.stop();
            _currentClient = WiFiClient();
            _currentStatus = HC_NONE;
        }
        WebServer::handleClient();
    }
private:
    /** A request normally follows its connection within milliseconds. */
    static constexpr uint32_t IDLE_MS = 150;
};

static PortalServer s_server(80);
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
/* A class that sets display outranks the browser's own [hidden] rule: without
   this, a hidden .row (the positions list) showed for every kind of teaching. */
[hidden]{display:none!important}
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
/* A section heading between cards (Sources, Advanced) */
.sec{display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin:22px 2px 10px}.sec h2{font-size:15px;font-weight:600}
/* Home: one line for each thing that needs you, linking to where it is dealt with */
.note{display:flex;align-items:center;gap:10px;background:var(--card);border:1px solid var(--line);border-left:3px solid var(--warn);
 border-radius:10px;padding:10px 12px;margin-bottom:8px;cursor:pointer;font-size:13px}
.note.crit{border-left-color:var(--crit)}.note .go{margin-left:auto;color:var(--muted);white-space:nowrap}
/* The map dialog, over whichever tab opened it */
.dlg{position:fixed;inset:0;z-index:8;background:#000b;display:flex;justify-content:center;align-items:flex-start;
 padding:56px 12px 12px;overflow-y:auto}
.dlg>.card{width:100%;max-width:560px}
</style></head><body>
<header>
 <div class="top"><h1>CAN <b>Master</b></h1><span id="conn" class="pill"><i></i><span>connecting</span></span></div>
 <nav>
  <button data-t="home" class="on">Home</button>
  <button data-t="sig">Signals</button>
  <button data-t="bus">Bus</button>
  <button data-t="car">Car</button>
  <button data-t="set">Settings</button>
 </nav>
</header>
<main>

<!-- Home: how the master is doing, what needs you, the values themselves -->
<section id="t-home">
 <div class="tiles">
  <div class="tile"><div class="k">CAN bus</div><div class="v" id="dBus">–</div><div class="s" id="dBusS">&nbsp;</div></div>
  <div class="tile"><div class="k">Heard on the bus</div><div class="v ok" id="dRaw">–</div><div class="s">values, nothing asked</div></div>
  <div class="tile"><div class="k">Requested</div><div class="v" id="dReq">–</div><div class="s" id="dReqS">&nbsp;</div></div>
  <div class="tile"><div class="k">Sent to displays</div><div class="v" id="dEsp">–</div><div class="s" id="dEspS">&nbsp;</div></div>
 </div>
 <div id="hNote"></div>
 <p class="say" id="dSay"></p>
 <div class="card"><h2>Live values <small id="vCount"></small>
  <input id="vq" placeholder="Filter…" style="margin-left:auto;width:140px"></h2>
  <div id="vals"></div></div>
</section>

<!-- Signals: where every value comes from - learned, taught, mapped, asked -->
<section id="t-sig" hidden>
 <p class="say">Where every value comes from. The master learns most of them by itself, from what it asks the ECU;
  teach it the rest by doing.</p>
 <div class="card"><h2>Learning <small id="lSub"></small>
   <label class="muted" style="margin-left:auto;font-size:12px"><input type="checkbox" id="lAll"> show all</label></h2>
  <p class="say" style="margin:0 0 6px">Every value the master requests is matched against the bus. <b>Listen now</b> takes the best
   match so far and reads that value from the bus straight away, even if it is not fully confirmed.</p>
  <div id="lTab"></div>
  <div class="row" style="margin-top:12px">
   <button class="b" onclick="post('/api/learn/reset','Learning restarted')">Restart learning</button>
   <button class="b d" onclick="if(confirm('Delete every automatically learned value?\n\nWhat was mapped or taught by hand stays.'))post('/api/learn/forget','Learned values deleted',()=>load(true))">Forget learned values…</button></div>
  <p class="say" style="margin:8px 0 0"><b>Restart learning</b> starts the search again; what was learned stays.
   <b>Forget learned values</b> deletes every learned value, and the fields turned down, so all of it is learned afresh.</p></div>
 <div class="card" id="teach"><h2>Teach by doing <small>for what the ECU cannot report</small></h2>
  <p class="say" style="margin:0 0 8px">Turn signals, doors, handbrake, steering, wheel speeds, the gear lever, the G-meter… Pick
   it, press <b>Start</b> and do what each step asks: the master finds the bits or bytes that followed you.</p>
  <div class="row"><select id="tM" style="flex:1;min-width:180px"></select>
   <select id="tK"></select>
   <button class="b p" id="tBtn" onclick="teachStart()">Start</button></div>
  <p class="say" id="tHow" style="margin:8px 2px 0"></p>
  <div class="row" id="tPRow" hidden style="margin-top:8px"><span class="muted">Positions, in the order you will select them:</span>
   <input id="tP" style="flex:1;min-width:120px" value="P R N D"></div>
  <div id="tStep"></div><div id="tRes"></div></div>
 <div id="srcBox">
  <div class="sec"><h2>Sources</h2><span class="muted">in the order the master prefers</span>
   <label class="muted" style="margin-left:auto;font-size:12px"><input type="checkbox" id="srcEd" onchange="renderSources()"> edit details</label></div>
  <div class="card"><h2><span class="tag raw">1</span> Heard on the bus <small>free — nothing is asked</small></h2>
   <div class="wrap"><table><thead id="sHead"></thead><tbody id="sTab"></tbody></table></div>
   <div class="row" style="margin-top:10px"><button class="b" onclick="go('bus')">Map a frame by hand…</button></div></div>
  <div class="card"><h2><span class="tag obd">2</span> OBD-II requests <small id="oSub"></small>
   <label class="muted" style="margin-left:auto;font-size:12px"><input type="checkbox" id="oAll" onchange="renderSources()"> show unsupported</label></h2>
   <div class="wrap"><table><thead id="oHead"></thead><tbody id="oTab"></tbody></table></div></div>
  <div class="card"><h2><span class="tag ssm">3</span> Subaru SSM2 requests <small id="mSub"></small>
   <label class="muted" style="margin-left:auto;font-size:12px"><input type="checkbox" id="mAll" onchange="renderSources()"> show unsupported</label></h2>
   <div class="wrap"><table><thead id="mHead"></thead><tbody id="mTab"></tbody></table></div>
   <div class="row" style="margin-top:10px"><button class="b" onclick="post('/api/ssm/reinit','Asking the ECU again')">Ask the ECU what it supports again</button></div></div>
 </div>
</section>

<!-- Bus: the link to the car - load, errors, the frames, the evidence log -->
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
 <div class="card"><h2>Frames on the bus <small id="bN"></small>
  <button class="b" style="margin-left:auto" onclick="post('/api/census/reset','Cleared')">Clear</button></h2>
  <p class="say" style="margin:0 0 8px"><span class="hx"><span class="h">Bright</span></span> bytes just changed,
   <span class="hx"><span class="l">white</span></span> ones change sometimes, grey ones never. <b>Map</b> reads a value from a frame by hand.</p>
  <div class="wrap"><table><thead><tr><th>ID</th><th>Hz</th><th>Data</th><th></th></tr></thead><tbody id="bTab"></tbody></table></div></div>
 <div class="card"><h2>Evidence log <small id="evN"></small>
  <button class="b" style="margin-left:auto" onclick="if(confirm('Clear the evidence log?'))post('/api/evlog/clear','Cleared',pollEvlog)">Clear</button></h2>
  <p class="say" style="margin:0 0 8px">Kept on the master's flash across reboots and firmware updates: every boot with its reset reason,
   the bus coming up and settling, every guard trip with the error counters and what the master was transmitting at the time,
   bus-off and sleep. Clear it before a test drive, read it after.</p>
  <p class="say" id="evSay"></p>
  <div class="wrap"><table><thead><tr><th>Uptime</th><th>Event</th><th>Detail</th><th>Master was</th></tr></thead><tbody id="evTab"></tbody></table></div></div>
 <div class="card"><h2>Raw recording <small id="rrSub"></small></h2>
  <p class="say" style="margin:0 0 8px">Every frame on the bus, and the master's own requests, to the 8 MB log space: a recording for
   analysis off the car (<b>canrec.py</b>, in the firmware's tools folder, turns the download into a candump log). Starting it
   <b>erases the log space, every drive log with it</b> - up to a minute, with the car standing. It then records whenever the car
   is on, across sleeps and restarts, until you stop it or the space is full. Meanwhile drive logging is off, and what the master
   saves by itself (learned values, the evidence log) waits for the end, or for a sleep.</p>
  <div class="bar" id="rrBarBox" hidden><i id="rrBar" style="width:0"></i></div>
  <p class="say" id="rrSay" style="margin:8px 0"></p>
  <div class="row" id="rrBtns"></div></div>
</section>

<!-- Car: what the car says - trouble codes, oil change, drive logs, its computer -->
<section id="t-car" hidden>
 <div class="card"><h2>Trouble codes <small id="dcSub"></small>
   <button class="b" style="margin-left:auto" onclick="post('/api/codes/read','Reading the codes',()=>setTimeout(pollCodes,2500))">Read now</button>
   <button class="b d" onclick="codesClear()">Clear…</button></h2>
  <p class="say" style="margin:0 0 8px">Stored, pending and permanent codes from the engine and the transmission computer (OBD-II
   services 03, 07 and 0A), read when the car comes on and every minute after. The screens show them on the check-engine card.</p>
  <div class="wrap"><table><thead><tr><th>Code</th><th>Meaning</th><th>Computer</th><th>Kind</th></tr></thead><tbody id="dcTab"></tbody></table></div>
  <p class="say" id="dcSay" style="margin:8px 0 0"></p></div>
 <div class="card"><h2>Oil change <small id="svSub"></small></h2>
  <p class="say" id="svSay" style="margin:0 0 10px"></p>
  <div class="row"><button class="b" onclick="serviceReset()">Oil changed now</button></div>
  <div id="carOil" style="margin-top:6px"></div></div>
 <div class="card"><h2>Drive logs <small id="lgSub"></small>
   <button class="b d" style="margin-left:auto" onclick="if(confirm('Delete every drive log?'))post('/api/log/clear','Deleted',pollLogs)">Delete all</button></h2>
  <p class="say" id="lgSay" style="margin:0 0 8px"></p>
  <div class="wrap"><table><thead><tr><th>Drive</th><th>Length</th><th>Rows</th><th>Size</th><th></th></tr></thead><tbody id="lgTab"></tbody></table></div>
  <div class="grp" style="margin-top:16px">Recording</div>
  <div id="carLog"></div>
  <div class="f" style="display:block"><div class="l"><b>Values recorded</b><span id="lgEst"></span></div>
   <div id="lgIds" class="row" style="margin-top:8px"></div>
   <div class="row" style="margin-top:8px"><select id="lgAdd" style="flex:1;min-width:180px"></select><button class="b" onclick="logAdd()">Add</button></div></div></div>
 <div class="card"><h2>Body computer <small id="bdSub"></small>
   <button class="b" id="bdBtn" style="margin-left:auto" onclick="bodyScan()">Find the car's computers</button></h2>
  <p class="say" style="margin:0 0 8px">The doors, the boot, the handbrake and the light switch as the body computer (BIU) knows
   them - asked for, where the bus does not carry them. <b>Find the car's computers</b>, standing with the ignition on, asks every
   diagnostic address from 0x700 to 0x7F7 "are you there" (3E) and each that answers for its first data block (21 50): only reads,
   nothing in a computer changes. About half a minute, with the OBD-II requests going ahead of it.</p>
  <div class="bar" id="bdBarBox" hidden><i id="bdBar" style="width:0"></i></div>
  <div class="wrap"><table><thead><tr><th>Asked on</th><th>Answers on</th><th>What it said</th><th></th></tr></thead><tbody id="bdTab"></tbody></table></div>
  <p class="say" id="bdSay" style="margin:8px 0 0"></p>
  <div class="grp" style="margin-top:16px">Reading it</div>
  <div id="bdRead"></div></div>
 <div class="card"><h2>Engine computer</h2>
  <p class="say" style="margin:0 0 8px">RomRaider lists its ROM definitions by the <b>calibration ID</b> (its "internal ID") and its
   logger definitions by the <b>ROM ID</b> SSM2 reports. If its definition files list this calibration ID, RomRaider can read and
   edit this ROM: check that before buying a cable.</p>
  <div class="wrap"><table><tbody id="idTab"></tbody></table></div></div>
</section>

<!-- Settings: every setting once - the everyday ones, the device, then Advanced -->
<section id="t-set" class="cfg" hidden>
 <p class="say">Changes apply when you press <b>Save</b>; the ones marked <span class="warn">reboot</span> after a reboot.
  The oil change and the drive log are set on the <a href="#car" class="acc" onclick="go('car');return false">Car</a> tab.</p>
 <div id="setMain"></div>
 <div class="card"><h2>Device</h2>
  <p class="say" id="sEcu" style="margin:0"></p>
  <div class="grp">Firmware</div>
  <p class="say" id="fwInfo" style="margin:0 0 10px">…</p>
  <div class="row">
   <input type="file" id="fwFile" accept=".bin,application/octet-stream">
   <button class="b p" id="fwBtn" onclick="fwUpload()">Update firmware</button>
  </div>
  <p class="say" id="fwSay" style="margin:10px 0 0">The master's <b>firmware.bin</b> from a build (GitHub: the CI run's <b>master-firmware</b> download). It goes into the spare slot and the master restarts into it; if the new one does not run properly, it goes back to this one by itself. Not while driving.</p>
  <div class="grp">Settings file</div>
  <p class="say" style="margin:0 0 8px">Everything the master keeps - settings, learned values, the request tables - in one file: a backup, or the way to move them to another master.</p>
  <div class="row">
   <button class="b" onclick="jsonDownload()">Download</button>
   <label class="b">Upload…<input type="file" id="jf" accept=".json,application/json" hidden onchange="jsonUpload(this)"></label>
  </div>
  <div class="grp">Restart and reset</div>
  <div class="row">
   <button class="b" onclick="post('/api/reboot','Rebooting')">Reboot</button>
   <button class="b p" onclick="if(confirm('Put the CAN bus settings back to the safe defaults and reboot?\n\nRequests on (Auto), errors let pass, 87.5 % triple sampling, both guards on, P2CAN 50 ms, 10 s wait after power-on.\n\nKept: the bitrate, your request pacing, displays, Wi-Fi and every learned value.'))post('/api/bus_defaults','Safe bus settings - rebooting')">Safe bus settings</button>
   <button class="b d" onclick="if(confirm('Restore factory settings and reboot?\n\nThis also deletes every learned value. For the bus settings alone, use Safe bus settings.'))post('/api/defaults','Restored - rebooting')">Factory reset</button>
  </div>
  <p class="say" style="margin:10px 0 0"><b>Safe bus settings</b> puts back only what the master does on the bus, and keeps everything learned.</p></div>
 <div class="sec"><h2>Advanced</h2><span class="muted">timings, thresholds and the whole configuration</span>
  <button class="b" id="advBtn" style="margin-left:auto" onclick="advShow(!ADV_ON)">Show</button></div>
 <div id="advBody" hidden>
  <p class="say">The defaults suit the car: change one of these for a reason, and note what it was.</p>
  <div id="setAdv"></div>
  <div class="card"><h2>Everything, as JSON <small>tables included</small></h2>
   <p class="say" style="margin:0 0 8px">The complete configuration: edit it here and press <b>Apply</b>, then <b>Save</b>.</p>
   <textarea id="json" spellcheck="false"></textarea>
   <div class="row" style="margin-top:8px">
    <button class="b" onclick="jsonShow()">Show current</button>
    <button class="b p" onclick="jsonApply()">Apply</button>
   </div></div>
 </div>
</section>
</main>
<!-- Map a value from a frame by hand: opened from a frame on the Bus tab, or from a taught field -->
<div id="mapDlg" class="dlg" hidden onclick="if(event.target===this)mapClose()">
 <div class="card" id="mapCard"><h2>Use a value from <span id="mapId" class="acc"></span>
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
 </div></div>
<div id="save"><span class="muted" id="saveMsg">Unsaved changes</span><button class="b" onclick="load()">Discard</button>
 <button class="b" id="saveRb" hidden onclick="save(true)">Save &amp; reboot</button><button class="b p" onclick="save()">Save</button></div>
<div id="toast"></div>
<script>
'use strict';
const META={257:["MIL lamp", "", "Status & switches", 0],259:["Fuel system status", "", "Status & switches", 0],260:["Engine load", "%", "Engine", 1],261:["Coolant", "°C", "Temperatures", 0],262:["STFT B1", "%", "Fuel & mixture", 1],263:["LTFT B1", "%", "Fuel & mixture", 1],264:["STFT B2", "%", "Fuel & mixture", 1],265:["LTFT B2", "%", "Fuel & mixture", 1],266:["Fuel press", "kPa", "Fuel & mixture", 0],267:["MAP", "kPa", "Engine", 0],268:["RPM", "rpm", "Engine", 0],269:["Speed", "km/h", "Driving", 0],270:["Timing", "°", "Engine", 1],271:["IAT", "°C", "Temperatures", 0],272:["MAF", "g/s", "Engine", 1],273:["Throttle", "%", "Engine", 1],276:["O2 B1S1", "V", "Fuel & mixture", 2],277:["O2 B1S2", "V", "Fuel & mixture", 2],280:["O2 B2S1", "V", "Fuel & mixture", 2],281:["O2 B2S2", "V", "Fuel & mixture", 2],287:["Run time", "s", "Driving", 0],289:["Dist MIL", "km", "Driving", 0],290:["Fuel rail (vac)", "kPa", "Fuel & mixture", 0],291:["Fuel rail gauge", "kPa", "Fuel & mixture", 0],292:["WB B1S1", "λ", "Fuel & mixture", 3],296:["WB B2S1", "λ", "Fuel & mixture", 3],300:["EGR cmd", "%", "Fuel & mixture", 1],301:["EGR error", "%", "Fuel & mixture", 1],302:["Evap purge", "%", "Fuel & mixture", 1],303:["Fuel level", "%", "Fuel & mixture", 1],304:["Warm-ups", "", "Driving", 0],305:["Dist cleared", "km", "Driving", 0],306:["Evap vapor", "Pa", "Fuel & mixture", 0],307:["Barometric", "kPa", "Engine", 0],308:["WB B1S1 λ (I)", "λ", "Fuel & mixture", 3],312:["WB B2S1 λ (I)", "λ", "Fuel & mixture", 3],316:["Cat temp", "°C", "Temperatures", 0],317:["Cat temp B2S1", "°C", "Temperatures", 0],318:["Cat temp B1S2", "°C", "Temperatures", 0],319:["Cat temp B2S2", "°C", "Temperatures", 0],322:["Battery", "V", "Electrical", 2],323:["Abs load", "%", "Engine", 1],324:["Cmd", "λ", "Engine", 3],325:["Rel throttle", "%", "Engine", 1],326:["Ambient", "°C", "Temperatures", 0],327:["Abs throttle B", "%", "Engine", 1],328:["Throttle C", "%", "Engine", 1],329:["Pedal D", "%", "Engine", 1],330:["Pedal E", "%", "Engine", 1],331:["Pedal F", "%", "Engine", 1],332:["Cmd throttle", "%", "Engine", 1],333:["Time MIL on", "min", "Driving", 0],334:["Time since clear", "min", "Driving", 0],337:["Fuel type", "", "Status & switches", 0],338:["Ethanol", "%", "Fuel & mixture", 1],339:["Evap abs", "kPa", "Fuel & mixture", 0],345:["Fuel rail abs", "kPa", "Fuel & mixture", 0],346:["Relative pedal", "%", "Engine", 1],347:["Hybrid SOC", "%", "Engine", 1],348:["Oil temp", "°C", "Temperatures", 0],349:["Injection timing", "°", "Fuel & mixture", 1],350:["Fuel rate", "L/h", "Fuel & mixture", 1],353:["Demand torque", "%", "Engine", 1],354:["Actual torque", "%", "Engine", 1],355:["Reference torque", "Nm", "Engine", 0],422:["Odometer", "km", "Driving", 0],769:["DTC count", "", "Status & switches", 0],788:["O2 B1S1 trim", "%", "Fuel & mixture", 1],789:["O2 B1S2 trim", "%", "Fuel & mixture", 1],792:["O2 B2S1 trim", "%", "Fuel & mixture", 1],793:["O2 B2S2 trim", "%", "Fuel & mixture", 1],804:["WB B1S1", "V", "Fuel & mixture", 2],808:["WB B2S1", "V", "Fuel & mixture", 2],820:["WB B1S1", "mA", "Fuel & mixture", 1],824:["WB B2S1", "mA", "Fuel & mixture", 1],1024:["Trouble codes", "", "Status & switches", 0],1025:["Trouble code 1", "", "Status & switches", 0],1026:["Trouble code 2", "", "Status & switches", 0],1027:["Trouble code 3", "", "Status & switches", 0],1028:["Trouble code 4", "", "Status & switches", 0],1029:["Trouble code 5", "", "Status & switches", 0],1030:["Trouble code 6", "", "Status & switches", 0],1031:["Trouble code 7", "", "Status & switches", 0],1032:["Trouble code 8", "", "Status & switches", 0],4097:["Boost", "bar", "Engine", 2],4098:["AFR", "AFR", "Fuel & mixture", 1],4099:["Oil press", "bar", "Engine", 2],4100:["EGT", "°C", "Temperatures", 0],4101:["Gear", "", "Driving", 0],4102:["Steering", "°", "Driving", 1],4103:["Brake", "bar", "Driving", 2],4104:["Wheel FL", "km/h", "Driving", 0],4105:["Wheel FR", "km/h", "Driving", 0],4106:["Wheel RL", "km/h", "Driving", 0],4107:["Wheel RR", "km/h", "Driving", 0],4108:["Lateral G", "g", "Driving", 2],4109:["Longitudinal G", "g", "Driving", 2],4110:["Yaw rate", "°/s", "Driving", 1],4111:["Gear lever", "", "Driving", 0],4112:["Cruise set", "km/h", "Driving", 0],4113:["ATF temp", "°C", "Temperatures", 0],4353:["Batt current", "A", "Electrical", 1],4354:["Batt SOC", "%", "Electrical", 1],4355:["Batt SOH", "%", "Electrical", 1],4356:["Batt temp", "°C", "Temperatures", 0],4357:["Alternator", "V", "Electrical", 2],4358:["Alternator", "A", "Electrical", 1],4359:["Alternator load", "%", "Electrical", 1],4360:["12V rail", "V", "Electrical", 2],4361:["Elec load", "W", "Electrical", 0],4362:["Charge status", "", "Status & switches", 0],4609:["Left turn signal", "", "Status & switches", 0],4610:["Right turn signal", "", "Status & switches", 0],4611:["High beam", "", "Status & switches", 0],4612:["Door open", "", "Status & switches", 0],4613:["Handbrake", "", "Status & switches", 0],4614:["Seatbelt unfastened", "", "Status & switches", 0],4615:["Cruise active", "", "Status & switches", 0],4616:["Reverse", "", "Status & switches", 0],4617:["Lever in P", "", "Status & switches", 0],4618:["Lever in N", "", "Status & switches", 0],4619:["Lever in D", "", "Status & switches", 0],4620:["Hazard lights", "", "Status & switches", 0],4621:["Cruise main", "", "Status & switches", 0],4622:["Front fog", "", "Status & switches", 0],4623:["Rear fog", "", "Status & switches", 0],4624:["Door FL", "", "Status & switches", 0],4625:["Door FR", "", "Status & switches", 0],4626:["Door RL", "", "Status & switches", 0],4627:["Door RR", "", "Status & switches", 0],4628:["Trunk open", "", "Status & switches", 0],4629:["Hood open", "", "Status & switches", 0],4630:["VDC off", "", "Status & switches", 0],4631:["Passenger belt unfastened", "", "Status & switches", 0],7937:["Night sense", "", "System", 0],7938:["Master uptime", "s", "System", 0],7939:["CAN frames/s", "", "System", 0],7940:["CAN IDs", "", "System", 0],7941:["OBD replies/s", "", "System", 0],7942:["SSM2 exch/s", "", "System", 0],7952:["Oil change due", "km", "System", 0],8193:["Knock corr", "°", "Engine", 1],8194:["Knock corr fine", "°", "Engine", 1],8195:["A/F learn", "%", "Fuel & mixture", 1],8196:["A/F corr", "%", "Fuel & mixture", 1],8197:["Inj duty", "%", "Fuel & mixture", 1],8198:["Target boost", "bar", "Engine", 2],8199:["Wastegate", "%", "Engine", 1],8200:["A/F corr 2", "%", "Fuel & mixture", 1],8201:["A/F learn 2", "%", "Fuel & mixture", 1],8202:["Front O2 #1", "V", "Fuel & mixture", 2],8203:["Rear O2", "V", "Fuel & mixture", 2],8204:["Front O2 #2", "V", "Fuel & mixture", 2],8205:["MAF sensor", "V", "Engine", 2],8206:["TPS", "V", "Engine", 2],8207:["Atmospheric", "kPa", "Engine", 0],8208:["Manifold rel", "kPa", "Engine", 0],8209:["Tank press", "kPa", "Engine", 0],8210:["Learned timing", "°", "Engine", 1],8211:["Accel pedal", "%", "Engine", 1],8212:["Fuel temp", "°C", "Temperatures", 0],8213:["Wastegate 2", "%", "Engine", 1],8214:["CPC duty", "%", "Fuel & mixture", 1],8215:["ISC duty", "%", "Engine", 1],8216:["A/F lean", "%", "Fuel & mixture", 1],8217:["A/F heater", "%", "Fuel & mixture", 1],8218:["ISC step", "", "Engine", 0],8219:["EGR steps", "", "Fuel & mixture", 0],8220:["Alternator duty", "%", "Electrical", 1],8221:["Fuel pump", "%", "Fuel & mixture", 1],8222:["Intake VVT R", "°", "Engine", 1],8223:["Intake VVT L", "°", "Engine", 1],8224:["Intake OCV R", "%", "Engine", 1],8225:["Intake OCV L", "%", "Engine", 1],8226:["A/F current", "mA", "Fuel & mixture", 1],8227:["Lambda", "λ", "Fuel & mixture", 3],8228:["Lambda 2", "λ", "Fuel & mixture", 3],8229:["Throttle motor", "%", "Engine", 1],8230:["Main TPS", "V", "Engine", 2],8231:["Main APS", "V", "Engine", 2],8232:["Brake boost", "kPa", "Driving", 0],8233:["Fuel HP", "MPa", "Fuel & mixture", 2],8235:["Exhaust VVT R", "°", "Engine", 1],8236:["Exhaust VVT L", "°", "Engine", 1],8237:["Rough cyl 1", "", "Engine", 0],8238:["Rough cyl 2", "", "Engine", 0],8239:["Rough cyl 3", "", "Engine", 0],8240:["Rough cyl 4", "", "Engine", 0],8241:["Inj 2 pulse", "ms", "Fuel & mixture", 2],8242:["Cold start inj", "ms", "Engine", 2],8243:["Alternator mode", "", "Status & switches", 0],8244:["Fuel sender", "V", "Fuel & mixture", 2],8245:["Radiator fan", "%", "Engine", 1],8246:["Learned ign corr", "°", "Engine", 1],8247:["Boost feedback", "%", "Engine", 1],8248:["Target RPM", "rpm", "Engine", 0],8249:["SI-Drive", "", "Status & switches", 0],8250:["Odometer (SSM)", "km", "Driving", 0],8251:["EPS current", "A", "Electrical", 1],8252:["Fuel pump", "A", "Fuel & mixture", 1],8254:["Inj 1 pulse", "ms", "Fuel & mixture", 2],8449:["Brake switch", "", "Status & switches", 0],8450:["Clutch switch", "", "Status & switches", 0],8451:["Neutral switch", "", "Status & switches", 0],8452:["A/C switch", "", "Status & switches", 0],8453:["Idle switch", "", "Status & switches", 0],8454:["Knock signal", "", "Status & switches", 0],8455:["Electrical load", "", "Status & switches", 0],8456:["Light switch", "", "Status & switches", 0],8457:["Radiator fan 1", "", "Status & switches", 0],8458:["Radiator fan 2", "", "Status & switches", 0],8459:["A/C compressor", "", "Status & switches", 0],8460:["Fuel pump relay", "", "Status & switches", 0],8461:["Oil pressure switch", "", "Status & switches", 0],8462:["Starter", "", "Status & switches", 0],8463:["Rear defogger", "", "Status & switches", 0],8464:["Blower", "", "Status & switches", 0],8465:["Wiper switch", "", "Status & switches", 0],8466:["Ignition", "", "Status & switches", 0],8467:["Stop lamp switch", "", "Status & switches", 0],8468:["Knock signal 2", "", "Status & switches", 0],12289:["Custom 1", "", "Custom", 0],12290:["Custom 2", "", "Custom", 0],12291:["Custom 3", "", "Custom", 0],12292:["Custom 4", "", "Custom", 0],12293:["Custom 5", "", "Custom", 0],12294:["Custom 6", "", "Custom", 0],12295:["Custom 7", "", "Custom", 0],12296:["Custom 8", "", "Custom", 0],12297:["Custom 9", "", "Custom", 0],12298:["Custom 10", "", "Custom", 0],12299:["Custom 11", "", "Custom", 0],12300:["Custom 12", "", "Custom", 0],12301:["Custom 13", "", "Custom", 0],12302:["Custom 14", "", "Custom", 0],12303:["Custom 15", "", "Custom", 0],12304:["Custom 16", "", "Custom", 0]};
const $=q=>document.querySelector(q);
const hex=(n,w)=>'0x'+Number(n).toString(16).toUpperCase().padStart(w,'0');
/* The body computer's answers are in the frame table as extended IDs of their
   own (BodyUnit.h): BODY | block << 8 | which eight bytes of it. */
const BODY=0x1F5B0000,isBody=id=>id>=BODY&&id<=BODY+0xFFFF;
const canName=id=>isBody(id)?`BIU ${(id>>8&0xFF).toString(16).toUpperCase()} · ${(id&0xFF)*8}-${(id&0xFF)*8+7}`:hex(id,3);
const esc=s=>String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const nm=id=>META[id]?META[id][0]:hex(id,4);
const un=id=>META[id]?META[id][1]:'';
const fmt=(id,v)=>{if(id===0x100F&&v>32&&v<127)return String.fromCharCode(Math.round(v));
 if(id>0x400&&id<=0x408)return dtcName(v);const d=META[id]?META[id][3]:2;return (+v).toFixed(d)};
const SRC={1:['SSM2','ssm'],2:['OBD-II','obd'],3:['calc','calc'],4:['bus','raw']};
let C={},ORIG={},L=null,LOADED=false,DIRTY=false,TAB='home',PREV=null,RATES={load:0,tx:0,errs:0},LSIG='',LLOAD=Date.now();
function toast(m,ms){const t=$('#toast');t.textContent=m;t.style.display='block';clearTimeout(t._h);t._h=setTimeout(()=>t.style.display='none',ms||2400)}
/* Every request has a deadline. The master answers one connection at a time,
   and a request the browser never gives up on - a lost Wi-Fi frame, a busy
   master - froze whatever was waiting for it, the teach countdown included. */
async function req(u,o,ms){
 const c=new AbortController(),t=setTimeout(()=>c.abort(),ms||6000);
 try{const r=await fetch(u,Object.assign({},o,{signal:c.signal}));
  if(!r.ok){let m='HTTP '+r.status;try{const j=await r.json();if(j&&j.error)m=j.error}catch(e){}throw new Error(m)}  // the master's reason
  return await r.json()}
 catch(e){if(e.name==='AbortError')throw new Error('no answer from the master');throw e}
 finally{clearTimeout(t)}}
/* One request of each kind at a time: a poll still waiting is not joined by
   the next one, so a slow moment cannot pile up into a slow minute. */
const BUSY={};
/* A firmware upload, or a raw recording's download, in progress: nothing else
   is polled (the master serves one connection at a time, and it holds it). */
let UPD=false;
async function solo(k,fn){if(BUSY[k])return;BUSY[k]=1;try{await fn()}finally{BUSY[k]=0}}
async function post(u,msg,then){try{await req(u,{method:'POST'},8000);toast(msg);if(then)setTimeout(then,500)}catch(e){toast('Failed')}}
/* The save bar: what is not saved yet, and whether part of it waits for a reboot. */
function setDirty(v){DIRTY=v;$('#save').classList.toggle('on',v);if(!v)return;
 const rb=Object.keys(C).filter(k=>REBOOT_KEYS.includes(k)&&!same(C[k],ORIG[k])).length;
 $('#saveMsg').textContent=rb?`Unsaved changes · ${rb===1?'one applies':'some apply'} after a reboot`:'Unsaved changes';
 $('#saveRb').hidden=!rb}
/* Five tabs, each polling only what it shows (the master answers one request
   at a time): Home and Signals /api/live, Bus also /api/bus and the evidence
   log, Car the codes, the drive logs and the body computer, Settings the
   firmware once. */
const TABS=['home','sig','bus','car','set'];
function go(t){if(!TABS.includes(t))t='home';TAB=t;
 document.querySelectorAll('nav button').forEach(b=>b.classList.toggle('on',b.dataset.t===t));
 TABS.forEach(x=>$('#t-'+x).hidden=x!==t);
 try{history.replaceState(null,'','#'+t)}catch(e){}
 if(t==='home'&&L)renderHome();if(t==='sig'){renderSources();if(L)renderLearn()}
 if(t==='bus'){pollBus();pollEvlog();pollRaw()}if(t==='car'){pollCodes();pollLogs();pollBody()}
 if(t==='set'){fwInfo();if(ADV_ON&&LOADED&&!JSON_ED)jsonShow()}}
document.querySelectorAll('nav button').forEach(b=>b.onclick=()=>go(b.dataset.t));

/* ───────────── settings: every input bound to the config by data-k ───────────── */
function parseEl(el){
 if(el.type==='checkbox')return el.checked;
 if(el.dataset.t==='str')return el.value;
 const v=parseFloat(el.value);return Number.isFinite(v)?v:undefined}
function fillBound(){
 document.querySelectorAll('[data-k]').forEach(el=>{const v=C[el.dataset.k];if(v===undefined)return;
  if(el.type==='checkbox')el.checked=!!v;else if(document.activeElement!==el)el.value=v});
 renderLogIds()}
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
 // An edit in the source tables (the switches above them only change the view).
 if(el.closest('#srcBox')&&!['srcEd','oAll','mAll'].includes(el.id))setDirty(true)});
const same=(a,b)=>JSON.stringify(a)===JSON.stringify(b),clone=o=>JSON.parse(JSON.stringify(o));
const inside=sel=>{const a=document.activeElement;return!!(a&&a.closest&&a.closest(sel))};
/* soft: keep every edit not saved yet and take the master's version of the rest -
   the master changes its own tables (learner, verifier) while the page is open. */
async function load(soft){
 let n;try{n=await req('/api/config',null,10000)}catch(e){
  if(!LOADED)setTimeout(()=>load(),2000);      // nothing on the settings tabs works without it
  return}
 if(soft&&LOADED){
  // A table row being edited right now keeps its data: its inputs write back
  // by row number, and a refreshed table may have rows in other places.
  const hold=inside('#srcBox')?['signals','pids','ssm']:[];
  for(const k of Object.keys(n)){
   if(hold.includes(k))continue;
   if(!DIRTY||same(C[k],ORIG[k])){C[k]=n[k];ORIG[k]=clone(n[k])}}}
 else{C=n;ORIG=clone(n);setDirty(false)}
 LOADED=true;fillBound();
 if(!inside('#srcBox'))renderSources();
 if(TAB==='set'&&ADV_ON&&!JSON_ED)jsonShow()}
const RO_KEYS=['cfg_ver','ecu_id','ecu_sys_id','ecu_flags','ecu_cal_id','vin'];
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
   changed by itself (learned signals, the remembered ECU identity).
   @p andReboot: then reboot, so the settings marked reboot apply. */
async function save(andReboot){
 if(!LOADED){toast('Not loaded yet');return}
 const body={};
 for(const k of Object.keys(C))if(!RO_KEYS.includes(k)&&!same(C[k],ORIG[k]))body[k]=C[k];
 if(!Object.keys(body).length){setDirty(false);toast('Nothing changed');return}
 const reboot=REBOOT_KEYS.some(k=>k in body);
 try{
  if(body.signals){const now=await req('/api/config',null,10000);
   body.signals=mergeSignals(ORIG.signals||[],C.signals||[],now.signals||[])}
  if(body.pids)body.pids=body.pids.map(({name,metrics,...p})=>p);   // read-only extras stay here
  // As a form's file: the master writes it to its flash as it comes and reads
  // it from there (a car's whole settings did not fit in its memory as text,
  // its parsed copy and the save's at once). Generous: it saves before it answers.
  const fd=new FormData();fd.append('cfg',new Blob([JSON.stringify(body)],{type:'application/json'}),'config.json');
  await req('/api/config',{method:'POST',body:fd},20000);
  setDirty(false);
  if(andReboot){await post('/api/reboot','Saved - rebooting');return}
  toast(reboot?'Saved - reboot to apply':'Saved');await load()}
 catch(e){toast('Save failed: '+e.message,8000)}}

/* ───────────── settings: every one in one place ───────────── */
/*<settings>*/
/* Every setting the master has, each in exactly one row (portal_tests.js
   checks it against MasterConfig::toJson): the everyday ones on the Settings
   tab, the rest under its Advanced section, the oil change and the drive log
   on the Car tab. A row: [key, label, type, arg, about] - type 'bool', 'sel'
   (arg: [[value, text], ...]), 'num' (arg: [min, max, step]) or 'str' (arg:
   its longest, in bytes); about: the line under the label. */
const REBOOT_KEYS=['bitrate_kbps','wifi_channel','ap_ssid','ap_pass','portal_on','can_sp875','can_timing'];
const SET_MAIN=[
 ['Requests and learning',[
  ['diag_mode','What the master may ask','sel',[[0,'Auto: listen → OBD-II → SSM2'],[1,'SSM2 requests only'],[2,'OBD-II requests only'],[3,'Both, no preference'],[4,'Listen only - nothing is transmitted']],
   'Listen only: nothing is transmitted at all - the CAN controller switches to listen-only the moment you save'],
  ['learn','Learn automatically','bool',0,'matches requested values to bus traffic, then stops requesting them'],
  ['ssm_switches','Read the ECU\'s switch inputs','bool',0,'SSM2, every 200 ms, wherever no frame gives them: A/C, brake, clutch, lights, the oil pressure switch…'],
  ['start_delay_s','Wait after power-on','num',[0,300],'seconds of listening only while the car\'s bus settles, before the first request · 0 = no wait']]],
 ['Bus protection',[
  ['tx_passive','Let errors pass','bool',0,'never sends an error frame: a frame the master reads as bad is dropped by the master alone and reaches every other module untouched. It still requests, receives and acknowledges good frames'],
  ['guard','Bus guard','bool',0,'pauses requests if bus errors follow our frames'],
  ['rx_guard','Receive-error guard','bool',0,'switches to listen-only if our controller starts corrupting other modules\' frames (a receive-error storm) — the protection against the transmission ECU losing the engine ECU\'s messages']]],
 ['CAN connection',[
  ['bitrate_kbps','CAN bitrate','sel',[[125,'125 kbit/s'],[250,'250 kbit/s'],[500,'500 kbit/s'],[1000,'1 Mbit/s']],'500 kbit/s on the OBD port'],
  ['can_timing','CAN bit timing','sel',[[2,'87.5 %, triple sampling (recommended)'],[1,'87.5 %'],[0,'ESP-IDF preset (80 %)']],'87.5 % with triple sampling rides out spikes a single 80 % sample reads as errors']]],
 ['Displays',[
  ['night_source','Night mode from','sel',[[2,"The car's light switch"],[1,'Light sensor on GPIO'],[0,'Never']]],
  ['broadcast_ms','Update every','num',[20,1000],'ms'],
  ['metric_ttl_ms','Drop a value after','num',[200,10000],'ms without news']]],
 ['Power',[
  ['sleep_enabled','Sleep when the car is off','bool',0,'wakes on the first CAN activity'],
  ['sleep_idle_s','Sleep after','num',[10,3600],'seconds of silence on the bus']]],
 ['Wi-Fi',[
  ['wifi_channel','Display channel','num',[1,13],'must match every display · 1–13'],
  ['ap_ssid','Wi-Fi name','str',32,'1–32 characters'],
  ['ap_pass','Wi-Fi password','str',63,'8–63 characters, or empty for open']]]];
const SET_ADV=[
 ['Requests',[
  ['obd_addr','OBD-II addressing','sel',[[0,'Auto (engine ECU first)'],[1,'Engine ECU only (0x7E0)'],[2,'All ECUs (0x7DF)']]],
  ['obd_gap','Pause between OBD-II requests','num',[0,255],'ms'],
  ['obd_order','When more OBD-II values are due than there is time for','sel',[[0,'The most overdue for its period first'],[1,'Each in turn']]],
  ['obd_to','OBD-II reply timeout','num',[20,2000],'ms'],
  ['obd_p2can','After an unanswered request, wait at least (ISO 15765-4 P2CAN)','num',[0,2000],'ms from the send · 0 = off'],
  ['req_max_hz','Request budget','num',[0,200],'requests/s · 0 = no cap'],
  ['ssm_gap','Pause between SSM2 requests','num',[50,5000],'ms'],
  ['ssm_batch','Values per SSM2 request','num',[2,33],'2–33'],
  ['ssm_to','SSM2 reply timeout','num',[100,5000],'ms'],
  ['cover_ms','Skip a request while a better source is fresher than','num',[100,30000],'ms'],
  ['dtc_poll_s','Read the trouble codes every','num',[0,3600],'s while the car is on · 0 = only when asked'],
  ['biu_tx','Body computer (BIU): its request ID','num',[0,2047],'decimal · 0 = not read · the Car tab\'s <b>Find the car\'s computers</b> fills it in'],
  ['biu_rx','Body computer: its reply ID','num',[0,2047],'decimal · 0 = the request ID + 8'],
  ['biu_ms','Read the body computer every','num',[100,10000],'ms']]],
 ['Learning',[
  ['learn_r2','Fit needed to call it a match','num',[0.5,0.99999,0.0001],'0.5–0.99999'],
  ['learn_min_n','Samples needed','num',[10,5000]],
  ['learn_steady','Steady-sample tolerance','num',[0.01,10,0.01],'× the movement needed'],
  ['learn_phi','Switch correlation needed','num',[0.5,1,0.01],'0.5–1'],
  ['verify_n','Confirmations before trusting a match','num',[5,250],'samples'],
  ['learn_recheck_s','Check learned values against the ECU every','num',[0,3600],'s per value · 0 = off']]],
 ['Bus protection',[
  ['guard_errs','Bus guard: errors after our frames that trip it','num',[1,100]],
  ['guard_win','…within','num',[1,120],'s'],
  ['guard_pause','Pause requests for','num',[1,3600],'s'],
  ['guard_trips','Trips before switching to listen-only','num',[1,50]],
  ['rx_guard_rec','Receive-error guard: the receive-error counter that trips it','num',[16,255],'96 = error-warning level'],
  ['rx_guard_errs','…or receive errors within the guard window that trip it','num',[1,250]],
  ['tx_hold','Hold the CAN TX line recessive across sleep','bool']]],
 ['CAN connection',[
  ['can_sp875','With the ESP-IDF preset: 87.5 % instead','bool']]],
 ['Displays',[
  ['radio_dbm','Radio transmit power','num',[2,20],'dBm - lower draws less from the supply'],
  ['keepalive_ms','Re-send unchanged values every','num',[50,1400],'ms'],
  ['hold_ms','A better source holds a value for','num',[0,5000],'ms'],
  ['ldr_dark','Light sensor: night below','num',[0,4095],'ADC'],
  ['ldr_light','Light sensor: day above','num',[0,4095],'ADC']]],
 ['Wi-Fi',[
  ['portal_on','Start this Wi-Fi at boot','bool',0,'off: only the BOOT button, held 3 s while the master runs, brings this page back']]]];
const SET_CAR=[
 ['Oil change',[
  ['oil_every_km','Change the oil every','num',[500,100000],'km'],
  ['oil_at_km','Last changed at','num',[-1,9999999],'odometer reading, km · -1 = no reminder']]],
 ['Drive log',[
  ['log_on','Record every drive','bool',0,'from the car coming on to it going off, one file a drive'],
  ['log_hz','Rows per second','num',[1,20],'1–20'],
  ['log_max_kb','Space for logs','num',[64,8000],'KB of the logs\' own 8 MB partition (1200 at most on an older partition table) · the oldest drive goes first']]]];
/*</settings>*/
function fieldRow([k,label,type,arg,about]){
 const tag=REBOOT_KEYS.includes(k)?' <span class="warn">reboot</span>':'';
 let input;
 if(type==='bool')input=`<input type="checkbox" data-k="${k}">`;
 else if(type==='sel')input=`<select data-k="${k}">${arg.map(o=>`<option value="${o[0]}">${esc(o[1])}</option>`).join('')}</select>`;
 else if(type==='str')input=`<input data-k="${k}" data-t="str" maxlength="${arg}">`;
 else{const [mn,mx,st]=arg;input=`<input type="number" data-k="${k}" min="${mn}" max="${mx}"${st?` step="${st}"`:''}>`}
 return`<div class="f"><div class="l"><b>${esc(label)}${tag}</b>${about?`<span>${esc(about)}</span>`:''}</div>${input}</div>`}
const fieldCard=([title,rows])=>`<div class="card"><h2>${esc(title)}</h2>${rows.map(fieldRow).join('')}</div>`;
function renderSettings(){
 $('#setMain').innerHTML=SET_MAIN.map(fieldCard).join('');
 $('#setAdv').innerHTML=SET_ADV.map(fieldCard).join('');
 $('#carOil').innerHTML=SET_CAR[0][1].map(fieldRow).join('');
 $('#carLog').innerHTML=SET_CAR[1][1].map(fieldRow).join('');
 fillBound()}
/* The Advanced section, open or closed as it was left on this phone. */
let ADV_ON=false;
function advShow(on){ADV_ON=!!on;$('#advBody').hidden=!ADV_ON;$('#advBtn').textContent=ADV_ON?'Hide':'Show';
 try{localStorage.setItem('advOn',ADV_ON?'1':'')}catch(e){}
 if(ADV_ON&&LOADED&&!JSON_ED)jsonShow()}
/* The JSON editor shows the configuration as it is, refreshed with it - until
   it is edited: then it is left alone until Apply or Show current. */
let JSON_ED=false;
function jsonShow(){$('#json').value=JSON.stringify(C,null,1);JSON_ED=false}
$('#json').oninput=()=>{JSON_ED=true};
function jsonApply(){
 let o;try{o=JSON.parse($('#json').value)}catch(e){toast('Not valid JSON: '+e.message);return}
 if(typeof o!=='object'||!o){toast('Expected an object');return}
 C=Object.assign(C,o);JSON_ED=false;fillBound();renderSources();setDirty(true);toast('Applied - press Save to keep it')}
function jsonDownload(){const a=document.createElement('a');
 a.href=URL.createObjectURL(new Blob([JSON.stringify(C,null,1)],{type:'application/json'}));
 a.download='can-master-config.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000)}
function jsonUpload(inp){const f=inp.files[0];if(!f)return;const r=new FileReader();
 r.onload=()=>{$('#json').value=r.result;jsonApply()};r.readAsText(f);inp.value=''}

/* ───────────── home, and the learning card on Signals ───────────── */
const LS=[['Waiting for the ECU','muted'],['Needs movement','warn'],['Searching','acc'],['Found — confirming','acc'],['Listening','ok'],['Not found yet','muted']];
function hint(id){
 if(id===0x010C)return'hold a few different revs for a couple of seconds each';
 if(id===0x010D||id===0x1005)return'drive, at a few different speeds';
 if(id===0x0105||id===0x010F||id===0x015C)return'let it warm up';if(id===0x012F)return'drive a while';
 if(id===0x0142)return'start and stop the engine';if((id&0xFF00)===0x2100||id===0x0101)return'switch it on and off a few times';
 return'hold it at a few different levels for a couple of seconds each'}
function field(x){
 if(!x.id)return'<span class="muted">—</span>';
 const on01=x.map&&x.bits&&(x.m>>8)===0x12&&x.map.every(p=>p[1]===0||p[1]===1);
 const tbl=on01?' • on when '+x.map.filter(p=>p[1]===1).map(p=>x.bits.map((b,i)=>`${b} ${(p[0]>>i)&1?'set':'clear'}`).join(', ')).join(' or '):
  x.map&&x.map.length?' • '+x.map.map(p=>esc(posLabel(x.m,p[1]))).join(' '):'';
 if(x.bits&&x.bits.length)return`${canName(x.id)} <span class="muted">bit${x.bits.length>1?'s':''} ${x.bits.join(' ')}${tbl}</span>`;
 if(x.l===1)return`${canName(x.id)} <span class="muted">bit ${x.s}</span>`;
 const k=x.s>>3;
 if(x.l<8&&!x.be)return`${canName(x.id)} <span class="muted">bits ${x.s}–${x.s+x.l-1}${tbl}</span>`;
 return`${canName(x.id)} <span class="muted">${x.l===8?'byte '+k:'bytes '+k+'–'+(k+1)}${x.l===12||x.l===14?' ('+x.l+' bit)':''}${x.be&&x.l>8?' BE':''}${tbl}</span>`}
async function learnAct(a,m){await post(`/api/learn/${a}?m=${m}`,a==='accept'?`Listening for ${nm(m)}`:`Requesting ${nm(m)} again`,()=>load(true))}
/* What needs the driver, from what /api/live already carries: one line each,
   linking to the tab where it is dealt with. */
function homeNotes(d){
 const v=id=>{const m=d.metrics.find(x=>x.id===id&&x.age<5000);return m?m.v:null},out=[];
 if(d.guard.rx_silent||d.guard.silent)out.push(['crit','<b>Listen-only to protect the bus</b>: bus errors followed our frames','bus']);
 else if(d.guard.pause)out.push(['warn',`<b>Requests paused</b> for ${Math.ceil(d.guard.pause/1000)} s after bus errors`,'bus']);
 const n=v(0x0400),mil=v(0x0101);
 if(n>0)out.push(['crit',`<b>${Math.round(n)} trouble code${n>=2?'s':''}</b>${mil?' · check-engine lamp on':''}`,'car']);
 else if(mil)out.push(['crit','<b>Check-engine lamp on</b>','car']);
 const km=v(0x1F10);
 if(km!==null&&km<=0)out.push(['crit',`<b>Oil change overdue</b> by ${Math.round(-km)} km`,'car']);
 else if(km!==null&&km<500)out.push(['warn',`<b>Oil change due</b> in ${Math.round(km)} km`,'car']);
 const TAB_NAME={bus:'Bus',car:'Car'};
 return out.map(([c,t,tab])=>`<div class="note ${c}" onclick="go('${tab}')"><span>${t}</span><span class="go">${TAB_NAME[tab]} ›</span></div>`).join('')}
function renderHome(){
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
 const todo=d.learn.filter(x=>x.st>=1&&x.st<=3).length;
 let say;
 if(!d.bus.alive)say='<b>No CAN traffic.</b> The ignition is off, or the transceiver is not wired to the OBD port.';
 else if(d.silent)say='<b>Listening only.</b> Nothing is transmitted; every value shown comes from the car\'s own broadcasts.';
 else if(d.settle)say=`<b>Letting the bus settle.</b> The car just woke, so nothing is transmitted for another ${Math.ceil(d.settle/1000)} s; until then every value shown comes from the car's own broadcasts.`;
 else{
  say=`<b>${nRaw} ${nRaw===1?'value is':'values are'} read by listening.</b> `;
  if(nObd)say+=`OBD-II is asked for ${nObd} the car does not broadcast`+(nSsm?`, and SSM2 for ${nSsm} more OBD-II has no PID for. `:'. ');
  else if(nSsm)say+=`SSM2 is asked for ${nSsm}. `;
  if(d.obd.answers===2&&d.ssm.answers===2)say+='<span class="warn">The ECU answers neither OBD-II nor SSM2.</span> ';
  if(d.guard.rx_silent)say+='<span class="crit">Our controller was corrupting other modules\' frames, so the master switched itself to listen-only.</span> See the Bus tab. ';
  else if(d.guard.silent)say+='<span class="crit">Bus errors kept following our requests, so the master switched itself to listen-only.</span> See the Bus tab. ';
  else if(d.guard.pause)say+=`<span class="warn">Requests paused for ${Math.ceil(d.guard.pause/1000)} s after bus errors.</span> `;
  if(todo)say+=`${todo} more ${todo===1?'is':'are'} being learned (Signals tab).`;
 }
 $('#dSay').innerHTML=say;$('#hNote').innerHTML=homeNotes(d);
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
function renderLearn(){
 const d=L,learned=d.learn.filter(x=>x.st===4).length,todo=d.learn.filter(x=>x.st>=1&&x.st<=3).length;
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
  '<p class="empty">Nothing to learn yet — the master learns from the values it requests.</p>'}
$('#vq').oninput=()=>L&&renderHome();
$('#lAll').onchange=()=>L&&renderLearn();

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
function pollBus(){return solo('bus',pollBusOnce)}
async function pollBusOnce(){
 if((TAB!=='bus'&&!MAP)||TEACH||UPD)return;
 try{BUS=await req('/api/bus',null,4000)}catch(e){return}
 const now=Date.now(),dt=ZT?(now-ZT)/1000:0;ZT=now;
 $('#bN').textContent=BUS.census.length+' IDs';
 $('#bTab').innerHTML=BUS.census.map(c=>{
  const k=(c.ext?'x':'')+c.id,b=hb(c.d||''),ch=hb(c.c||''),pb=ZP[k]||[];
  if(dt&&ZC[k]!==undefined)ZHZ[k]=(c.count-ZC[k])/dt;ZC[k]=c.count;ZP[k]=b;
  const by=b.map((v,i)=>`<span class="${pb.length&&pb[i]!==v?'h':ch[i]?'l':''}">${v.toString(16).toUpperCase().padStart(2,'0')}</span>`).join('');
  const used=(C.signals||[]).filter(s=>s.can_id===c.id&&(s.mode===1||s.mode===3)).map(s=>nm(s.metric));
  return`<tr><td>${isBody(c.id)?canName(c.id):(c.ext?'ext ':'')+hex(c.id,3)}</td><td>${ZHZ[k]!==undefined?ZHZ[k].toFixed(0):'–'}${c.age>2000?' <span class="warn">idle</span>':''}</td>
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
   (near>=30&&near>=2*hot?'<span class="warn">Far above chance: the radio is disturbing the CAN side - its 3.3 V supply or ground. Try Settings → Advanced → Displays: radio transmit power 2 dBm, and compare.</span>':'About chance: the radio is not what causes them.')+'</p>'}
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
 if(g.rx_silent)$('#gSay').innerHTML='Our controller\'s receive-error counter climbed into the error-warning region: it was error-flagging — destroying — other modules\' frames, which is how the transmission ECU loses the engine ECU\'s messages. It was switched to <b>listen-only</b>. Check the transceiver ground and stub length, try the late sample point (Settings → CAN connection: bit timing), then resume. The evidence log below has the record.';
 else if(g.silent)$('#gSay').innerHTML='Bus errors kept following our frames, so the controller was switched to <b>listen-only</b> — it cannot disturb the car now. Fix the wiring (see below), then resume. The guard can be turned off in Settings → Bus protection.';
 else if(g.pause)$('#gSay').innerHTML=`Bus errors followed our frames, so all requests are paused for another ${Math.ceil(g.pause/1000)} s (pause ${g.trips} of ${trips}; the last one switches to listen-only).`;
 $('#bLoad').textContent=d.bus.alive?RATES.load.toFixed(0)+'%':'–';
 const bar=$('#bBar');bar.style.width=Math.min(100,RATES.load)+'%';
 bar.style.background=RATES.load>70?'var(--crit)':RATES.load>50?'var(--warn)':'var(--ok)';
 $('#bTx').textContent=RATES.tx.toFixed(0);
 $('#bErr').textContent=d.bus.err;$('#bErr').className='v '+(RATES.errs>0?'crit':d.bus.err?'warn':'ok');
 $('#bErrS').textContent=`error counters ${d.bus.tec} / ${d.bus.rec}`+(C.tx_passive&&!d.silent&&d.bus.tec>=128?' (high on purpose: errors let pass)':'')+
  (d.bus.lom_rp?` · listen-only kept error-passive ${d.bus.lom_rp}×`:'');
 $('#bFps').textContent=(RATES.fps||0).toFixed(0)+'/s';$('#bIds').textContent=d.bus.ids+' different IDs';
 let s;
 if(!d.bus.alive)s='No traffic.';
 else{
  s=RATES.load<50?`Load is comfortable — CAN only struggles past about 70%. Our requests add ${RATES.tx.toFixed(0)} frames/s.`:
   `Load is high (${RATES.load.toFixed(0)}%); our share is ${RATES.tx.toFixed(0)} frames/s.`;
  if(RATES.errs>0||d.bus.tec||d.bus.rec){
   s+=d.bus.err_tx>d.bus.err_idle?' <span class="crit">Errors mostly follow our own frames.</span> The link from the master to the bus is marginal: check the transceiver ground to OBD pin 4/5, that its 120 Ω terminator is removed, and the stub length. The late sample point (Settings → CAN connection: bit timing) may also help.':
    ' <span class="warn">Errors also happen while we are quiet</span> — they are not ours (cranking and modules waking produce a few).';
  }else s+=' <span class="ok">No errors</span> — every frame is getting through.';
  if(d.bus.missed)s+=` <span class="muted">${d.bus.missed} frames dropped on our side (harmless).</span>`;
 }
 $('#bSay').innerHTML=s}
function metricOpts(sel,filter){return Object.keys(META).map(Number).filter(id=>(id&0xFF00)!==0x1F00&&(!filter||filter(id))).sort((a,b)=>a-b)
 .map(id=>`<option value="${id}" ${id===sel?'selected':''}>${esc(nm(id))}${un(id)?' ('+esc(un(id))+')':''}</option>`).join('')}
/* The map dialog sits over the tab that opened it; while it is open the frames
   are read (pollBus), so its preview follows the bus. */
function mapOpen(id,ext,p){p=p||{};MAP={id,ext:!!ext};$('#mapDlg').hidden=false;$('#mapId').textContent=canName(id);
 $('#mpS').value=p.s||0;$('#mpL').value=p.l||8;$('#mpK').value=p.k||1;$('#mpO').value=p.o||0;$('#mpBE').value=p.be?'1':'0';
 $('#mpSg').checked=!!p.sg;$('#mpM').innerHTML=metricOpts(p.m||0x010C);mapPrev();pollBus()}
function mapClose(){MAP=null;$('#mapDlg').hidden=true}
document.addEventListener('keydown',e=>{if(e.key==='Escape'&&MAP)mapClose()});
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
/** The indicators are taught in one run; what it finds for each role. */
const TURN_ROLE={L:0x1201,R:0x1202,H:0x120C};
/** Fewer readings than this in a step and the page says the master was too
    slow: two readings can make a slow counter bit look like a switch. */
const TEACH_MIN_READS=3;
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
/** The step a reading counts for: the one it was both asked for and answered
    in, after that step's first @p skip ms. @p durs are the steps' lengths,
    times are ms from the start. -1: none - a reading that straddles a change
    of step, or comes back after it, belongs to neither. */
/** A run sized to how fast the master answers (@p answerMs, the slowest of a
    few timed readings): the deadline for one reading, and each step made long
    enough, after its first @p skip ms, for TEACH_MIN_READS+1 readings to be
    asked and answered inside it. A fast master keeps the given lengths. */
function teachPlan(durs,skip,answerMs){
 const per=Math.max(150,answerMs);
 const deadline=Math.min(8000,Math.max(1500,Math.ceil(2*per)));
 return{deadline,durs:durs.map(d=>Math.max(d,Math.ceil(skip+(TEACH_MIN_READS+2)*per)))}}
function teachSlot(durs,skip,tReq,tResp){
 let s=0;
 for(let i=0;i<durs.length;i++){const e=s+durs[i];if(tReq>=s+skip&&tResp<=e)return i;s=e}
 return -1}
/** One bit through one step's polls (@p P payloads, @p E flip counters):
    its flips (n), the share of polls it was away from @p rest (p), and per
    poll whether anything happened there (act: away from rest, or flipped
    since the poll before). */
function bitTrace(P,E,bit,rest){
 const act=[];let n=0,away=0;
 for(let i=0;i<P.length;i++){
  let d=0;
  if(i){const a=E&&E[i-1],b=E&&E[i];
   d=a&&b&&a.length===64&&b.length===64?(b[bit]-a[bit])&255:bitOf(P[i],bit)!==bitOf(P[i-1],bit)?1:0}
  const aw=bitOf(P[i],bit)!==rest;n+=d;away+=aw?1:0;act.push(aw||d>0)}
 return{n,p:P.length?away/P.length:0,act}}
/** An OFF step at rest - allowing for the driver. A hand still on the lever
    when the step began (@p lag: the step before had this bit going) leaves
    flashes at its start; one already on it for the next step (@p early: the
    step after has it going) leaves flashes at its end. Whatever happened
    must lie at those ends, around a still middle of at least 40 % of the
    step's polls: a counter or a status bit has none. On real Subaru traffic
    a driver two seconds late used to lose the lamps' bits altogether. */
function restsIn(act,lag,early){
 const n=act.length;if(!n)return false;
 if(!act.includes(true))return true;
 let best=0,from=0;
 for(let i=0,run=0;i<n;i++){run=act[i]?0:run+1;if(run>best){best=run;from=i-run+1}}
 if(best<Math.max(2,Math.ceil(0.4*n)))return false;
 return(lag||!act.slice(0,from).includes(true))&&(early||!act.slice(from+best).includes(true))}
/** Flashing in a step: three flips or more, or - from a frame sent slower than
    the lamp flashes - two, with the bit lit for part of the step. */
const flashIn=t=>t.n>=3||(t.n>=2&&t.p>=0.15&&t.p<=0.85);
/** Held away from rest in a step: switched once or twice at most, so in one
    stretch, and for half the step or more - a driver late on the lever holds
    it for the rest of the step only. */
const heldIn=t=>t.p>=0.5&&t.n<=2;
/** Nothing at all: not one flip, never away from rest. */
const silentIn=t=>t.n===0&&t.p===0;
/** The value a bit rests at: what it reads in most of the OFF steps' polls. */
const restOf=(offP,bit)=>offP.filter(b=>bitOf(b,bit)).length*2>offP.length?1:0;
/** On/off: A and C are the OFF steps, B the ON one. */
function teachBits(id,A,B,C,EA,EB,EC,dlc){
 const res=[],p=(arr,bit)=>arr.filter(b=>bitOf(b,bit)).length/arr.length,haveC=C.length>=2;
 for(let bit=0;bit<dlc*8;bit++){
  const pa=p(A,bit),pb=p(B,bit),pc=haveC?p(C,bit):pa;
  // A real switch holds still within each step and flips between them; a
  // counter bit flips inside a step, so it is not steady anywhere.
  const steady=Math.min(...[pa,pb,pc].map(x=>Math.max(x,1-x)));
  const score=Math.min(Math.abs(pb-pa),Math.abs(pb-pc));
  if(score>=0.7&&steady>=0.8){res.push({id,s:bit,l:1,be:false,score,inv:pb<(pa+pc)/2});continue}
  // A turn signal flashes, so it is on for only part of the ON step and the
  // test above never passes. It rests through both OFF steps - flashes at
  // their ends are the driver's hand, late or early on the lever - and
  // flashes in the ON one.
  const rest=restOf(haveC?A.concat(C):A,bit);
  const ta=bitTrace(A,EA,bit,rest),tb=bitTrace(B,EB,bit,rest),tc=haveC?bitTrace(C,EC,bit,rest):ta;
  if(!flashIn(tb)||!restsIn(ta.act,false,true)||!restsIn(tc.act,true,false))continue;
  res.push({id,s:bit,l:1,be:false,score:0.7+0.3*Math.min(1,tb.n/8),inv:rest===1,blink:Math.max(1,Math.round(tb.n/2))})}
 return res}
/** The indicators in one run, steps A L B R C H D: off, left, off, right,
    off, hazard lights, off.
    - A lamp that flashes with the left indicator and is silent with the
      right is the left one, and the other way round; one that flashes with
      both cannot tell the side and is only reported (both). Silent means
      not one flip: a bit that pulses on some flashes of either side (the
      2012 Impreza's 0x361 bit 51) used to pass for a side when the right
      step happened to catch few of its pulses.
    - A bit held for one side is its lever (or the body computer's request) -
      the displays flash those themselves. It may be held in the hazard step
      as well: many cars set both sides' bits for the hazard lights.
    - A bit held, or flashing, in the hazard step alone is the hazard lights.
    - A car that sends one flasher for both sides and the side as a bit of
      its own in the same frame: the side's signal is the flasher while that
      bit says so - a bit combination with a value table (how 'gated').
    Every OFF step rests, but for the driver's hand at its ends (restsIn).
    A bit of a number is never a lamp: where another bit of its byte keeps
    changing while nothing is on, at no steady pace (a counter keeps one),
    the byte is a value. The owner's 2010 Legacy (2026-11-19) sends no lamp
    on the OBD port's bus, and the one thing that moved with the indicators
    was a value - 0x432 byte 5, at 61 or 64 at rest, 63 and 66 with each
    flash (the lamps' load): its bit 41, quiet through the left step by
    chance, was offered as the right lamp and then flashed with either side.
    Such bits are kept apart (the result's .numbers) to be named, not offered.
    Candidates carry the role they fill (L, R, H or both) and, except for
    both, the metric; best first within each role, three at most. */
function teachTurns(S){
 const OFF=['A','B','C','D'],ON=['L','R','H'],ALL=['A','L','B','R','C','H','D'];
 const ids=new Set();ALL.forEach(k=>(S[k]||[]).forEach(poll=>poll.forEach((x,id)=>ids.add(id))));
 const res=[],numbers=[];
 for(const id of ids){
  if(id>=0x7DF&&id<=0x7EF)continue;
  const st={};ALL.forEach(k=>st[k]=(S[k]||[]).map(poll=>poll.get(id)).filter(Boolean));
  if(ALL.some(k=>st[k].length<2))continue;
  const dlc=Math.max(...ALL.map(k=>Math.max(...st[k].map(x=>x.b.length)))),offP=OFF.flatMap(k=>st[k].map(x=>x.b));
  const mine=[],tr=[];
  for(let bit=0;bit<dlc*8;bit++){
   const rest=restOf(offP,bit),t={};
   for(const k of ALL)t[k]=bitTrace(st[k].map(x=>x.b),st[k].map(x=>x.e),bit,rest);
   tr[bit]=t}
  // Another bit of the same byte that changed level in that step too (not
  // flashing): the two are bits of a value - the steering angle, nudged by
  // the hand on the lever (both 2012 Imprezas: 0x002, 0x0D0) - not a switch.
  const valueBit=(bit,k)=>{for(let o=bit&~7;o<(bit&~7)+8;o++)
   if(o!==bit&&tr[o]&&tr[o][k].p>=0.3&&!flashIn(tr[o][k]))return true;return false};
  // The other bits of the byte that are busy while nothing is on (in three
  // OFF steps or more, eight flips in all). A rolling counter's are a run of
  // neighbours, each flipping about half as often as the one below it, the
  // lowest at a steady pace; any others make the byte a number - 0x432 byte 5
  // stepping between 61 and 64 flips six bits at once, the same count each.
  const restBusy=o=>OFF.filter(k=>tr[o][k].n>0).length>=3&&OFF.reduce((s,k)=>s+tr[o][k].n,0)>=8;
  const pace=o=>OFF.map(k=>tr[o][k].n/Math.max(1,tr[o][k].act.length));
  const rate=o=>{const r=pace(o);return r.reduce((a,b)=>a+b,0)/r.length};
  // And where the bit flashes, another bit of its byte (not the counter's)
  // flipping there too, out of step with it: a number dipping with each
  // flash (the other Impreza's 0x361 byte 6, still at rest). Two lamps, or a
  // lamp and a flasher, flash in step: read together they agree throughout.
  const numberBit=bit=>{
   const lo=bit&~7,B=[];for(let o=lo;o<lo+8;o++)if(o!==bit&&tr[o]&&restBusy(o))B.push(o);
   if(B.length){
    const r0=pace(B[0]);if(Math.min(...r0)*1.6<Math.max(...r0))return true;  // no steady pace
    for(let i=1;i<B.length;i++){const q=rate(B[i])/rate(B[i-1]);
     if(B[i]!==B[i-1]+1||q<0.3||q>0.7)return true}}                            // not a counter's run
   for(const k of ON){
    if(!flashIn(tr[bit][k]))continue;
    const P=st[k].map(x=>x.b);
    for(let o=lo;o<lo+8;o++){
     if(o===bit||B.includes(o)||!tr[o]||tr[o][k].n<2)continue;
     const agree=P.filter(b=>bitOf(b,o)===bitOf(b,bit)).length/P.length;
     if(agree>0.15&&agree<0.85)return true}}
   return false};
  for(let bit=0;bit<dlc*8;bit++){
   const t=tr[bit],rest=restOf(offP,bit);
   const fl=k=>flashIn(t[k]),hd=k=>heldIn(t[k]),sil=k=>silentIn(t[k]),busy=k=>fl(k)||hd(k);
   let role=null,how='flash';
   if(fl('L')&&fl('R'))role='both';
   else if(fl('L')&&sil('R'))role='L';
   else if(fl('R')&&sil('L'))role='R';
   else if(fl('H')&&sil('L')&&sil('R'))role='H';
   else if(hd('L')&&sil('R')&&(hd('H')||sil('H'))){role='L';how='held'}
   else if(hd('R')&&sil('L')&&(hd('H')||sil('H'))){role='R';how='held'}
   else if(hd('H')&&sil('L')&&sil('R')){role='H';how='held'}
   if(!role)continue;
   if(how==='held'&&valueBit(bit,role))continue;
   // ...and at rest in every OFF step, but for the driver's hand at its ends.
   if(!restsIn(t.A.act,false,busy('L'))||!restsIn(t.B.act,busy('L'),busy('R'))||
      !restsIn(t.C.act,busy('R'),busy('H'))||!restsIn(t.D.act,busy('H'),false))continue;
   if(numberBit(bit)){numbers.push({id,s:bit,role,how});continue}
   const n=ON.filter(fl).reduce((s,k)=>s+t[k].n,0),nf=ON.filter(fl).length||1;
   // The car's own flasher beats a lever bit (the displays then flash with
   // the car); for the hazard lights, their switch beats a lamp of their own.
   // A side's lamp flashes with the hazard lights as well: a bit that flashed
   // through one side's step alone is more likely something else that was
   // busy just then (the 2012 Impreza's 0x140 bit 48, an engine bit that
   // flipped as fast as the lamp for a while).
   const withH=(role==='L'||role==='R')&&fl('H');
   const score=how==='flash'?1.1+0.3*Math.min(1,n/(8*nf))+(withH?0.4:0):(role==='H'?1.6:0.9)+0.1*(hd('H')&&role!=='H'?0:t[role].p);
   const r={id,s:bit,l:1,be:false,inv:rest===1,role,how,blink:how==='flash'?Math.max(1,Math.round(n/2)):0,score,rest,heldH:hd('H'),withH};
   if(TURN_ROLE[role])r.metric=TURN_ROLE[role];
   mine.push(r)}
  // One flasher for both sides and the side as a bit of its own: the side
  // that has no lamp of its own is the flasher while that bit is at rest,
  // and the other the flasher while it is set (both flash with the car).
  const flashers=mine.filter(r=>r.role==='both'),sides=mine.filter(r=>r.how==='held'&&(r.role==='L'||r.role==='R'));
  for(const F of flashers.slice(0,1))for(const D of sides.slice(0,1)){
   const code=(f,d)=>(f?1:0)|(d?2:0),fOn=1-F.rest,dOn=1-D.rest,other=D.role==='L'?'R':'L';
   const gate=(role,dState)=>{const hit=code(fOn,dState);
    return{id,s:F.s,l:1,be:false,bits:[F.s,D.s],map:[0,1,2,3].map(c=>[c,c===hit?1:0]),role,how:'gated',
     flasher:F.s,side:D.s,sideSet:dState===dOn,blink:F.blink,score:1.45,metric:TURN_ROLE[role]}};
   mine.push(gate(D.role,dOn),gate(other,D.rest))}
  res.push(...mine)}
 // Where one side's lamp flashed with the hazard lights as well, a bit that
 // flashed in that side's step alone is something else that was busy then.
 const lampWithH=k=>res.some(r=>r.role===k&&r.how==='flash'&&r.withH);
 const keep=res.filter(r=>!(r.how==='flash'&&(r.role==='L'||r.role==='R')&&!r.withH&&lampWithH(r.role)));
 const order=['L','R','H','both'];
 keep.sort((a,b)=>order.indexOf(a.role)-order.indexOf(b.role)||b.score-a.score);
 const per={},out=keep.filter(r=>(per[r.role]=(per[r.role]||0)+1)<=3);
 out.numbers=numbers;
 return out}
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
    the same. The lever is learnt from the bits that hold still in each
    position and read differently between them: its own field where it has
    one, or else the combination of all of them, wherever they sit - side by
    side in one byte, scattered through it, or across bytes. */
function teachPositions(id,P,toks,dlc){
 const res=[],np=P.length;
 if(np<2||P.some(a=>a.length<2))return res;
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
 // Each position's code read from @p bits (bit i of the code is bits[i]):
 // null unless every visit reads one code throughout and no two positions
 // share one.
 const codesOf=bits=>{
  const codeOf=b=>bits.reduce((c,bit,i)=>c|(bitOf(b,bit)<<i),0);
  const codes=new Array(names.length).fill(null),hold=[];
  for(let j=0;j<np;j++){
   const cnt=new Map();P[j].forEach(b=>{const x=codeOf(b);cnt.set(x,(cnt.get(x)||0)+1)});
   let best=0,n=0;cnt.forEach((c,x)=>{if(c>n){n=c;best=x}});
   if(codes[grp[j]]===null)codes[grp[j]]=best;else if(codes[grp[j]]!==best)return null;
   hold.push(n/P[j].length)}
  return new Set(codes).size===names.length&&Math.min(...hold)>=0.8?{codes,hold:Math.min(...hold)}:null};
 // The lever's own field, where it has one: the shortest run of neighbouring
 // bits, every one of them moving with the lever, that tells every position
 // apart by itself. Other bits that changed with it - a park/neutral switch,
 // a reverse lamp - are left out: while driving they can change on their own
 // and spoil the reading.
 let pick=null;
 for(let a=0;a<moving.length;a++)
  for(let b=a;b<moving.length&&b-a<16&&moving[b]===moving[a]+(b-a);b++){
   if(pick&&b-a+1>=pick.bits.length)break;
   const bits=moving.slice(a,b+1),c=codesOf(bits);
   if(c)pick=Object.assign({bits},c)}
 // Otherwise the combination of every bit that moved. Bits beside them that
 // did nothing here are left out, so they are free to change later.
 if(!pick&&moving.length<=16){const c=codesOf(moving);if(c)pick=Object.assign({bits:moving.slice()},c)}
 if(pick)res.push({id,s:pick.bits[0],l:pick.bits.length,be:false,bits:pick.bits,names:names.slice(),codes:pick.codes,
  score:1+pick.hold-0.01*pick.bits.length});
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
 if(kind==='turn')return teachTurns(S);
 const ids=new Set();(S[steps[0]]||[]).forEach(poll=>poll.forEach((x,id)=>ids.add(id)));
 const res=[];
 for(const id of ids){
  if(id>=0x7DF&&id<=0x7EF)continue;
  const ser=ph=>(S[ph]||[]).map(poll=>poll.get(id)).filter(Boolean);
  if(kind==='pos'){
   const P=steps.map(ph=>ser(ph).map(x=>x.b));
   if(P.some(a=>a.length<2))continue;
   res.push(...teachPositions(id,P,toks||[],Math.max(...P.flat().map(b=>b.length))));continue}
  const A=ser('A'),B=ser('B'),Cc=ser('C');
  if(A.length<2||B.length<2)continue;
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
/** G and yaw: what a field should read at each poll, from what the master
    knows then. @p P: one {t (ms), v (km/h), st (steering °), yaw (°/s), lat (g)}
    per poll, null where unknown. Longitudinal G is the speed's slope (least
    squares over ±0.6 s: OBD-II speed comes in whole km/h). Lateral G is speed
    × yaw rate when the yaw rate is known, else the steering's turn at this
    speed (ratio 15, wheelbase 2.67 m: the scale is then approximate, and the
    sign is whatever the steering's is). Yaw rate likewise. */
const STEER_RATIO=15,WHEELBASE=2.67,G0=9.80665;
function motionRef(m,P){
 const out=new Array(P.length).fill(null),rad=Math.PI/180;
 const sp=P.filter(p=>p.v!=null).filter((p,i,a)=>!i||p.t!==a[i-1].t);
 for(let i=0;i<P.length;i++){
  const p=P[i];if(p.v==null)continue;const ms=p.v/3.6;
  if(m===0x100D){
   const w=sp.filter(q=>Math.abs(q.t-p.t)<=600);if(w.length<4)continue;
   const n=w.length,mt=w.reduce((a,q)=>a+q.t,0)/n,mv=w.reduce((a,q)=>a+q.v,0)/n;
   let sxy=0,sxx=0;for(const q of w){sxy+=(q.t-mt)*(q.v-mv);sxx+=(q.t-mt)**2}
   if(sxx>0)out[i]=sxy/sxx*1000/3.6/G0}
  else if(m===0x100C){
   if(p.yaw!=null)out[i]=ms*p.yaw*rad/G0;
   else if(p.st!=null)out[i]=ms*ms*Math.tan(p.st/STEER_RATIO*rad)/WHEELBASE/G0}
  else if(m===0x100E){
   if(p.lat!=null&&p.v>5)out[i]=p.lat*G0/ms/rad;
   else if(p.st!=null)out[i]=ms*Math.tan(p.st/STEER_RATIO*rad)/WHEELBASE/rad}}
 return out}
/** The field that follows a reference: @p X the payloads, @p R what the value
    should be at each (null = unknown). The best |r| of the 8- and 16-bit
    readings, signed or not, with up to 3 polls of lag either way (the
    reference comes from slower values); a straight-line fit gives the scale
    and offset that turn its raw value into the reference's units. */
function teachMotion(id,X,R,dlc){
 const res=[],idx=[];for(let i=0;i<R.length;i++)if(R[i]!=null&&Number.isFinite(R[i]))idx.push(i);
 if(idx.length<12)return res;
 const ry=idx.map(i=>R[i]);if(!(Math.max(...ry)-Math.min(...ry)>0))return res;
 const fit=(x,y)=>{const n=x.length;let mx=0,my=0;for(let i=0;i<n;i++){mx+=x[i];my+=y[i]}mx/=n;my/=n;
  let a=0,b=0,c=0;for(let i=0;i<n;i++){const dx=x[i]-mx,dy=y[i]-my;a+=dx*dy;b+=dx*dx;c+=dy*dy}
  return b>0&&c>0?{r:a/Math.sqrt(b*c),k:a/b,o:my-a/b*mx}:null};
 for(let k=0;k<dlc;k++){
  const fl=[[8*k,8,false]];if(k+1<dlc)fl.push([8*k,16,false],[8*k+7,16,true]);
  let best=null;
  for(const [st,len,be] of fl)for(const sg of [false,true]){
   const xs=X.map(b=>extract(b,st,len,be,sg));
   if(new Set(xs).size<6)continue;
   for(let lag=-3;lag<=3;lag++){
    const x=[],y=[];
    for(const i of idx){const j=i+lag;if(j>=0&&j<xs.length){x.push(xs[j]);y.push(R[i])}}
    if(x.length<10)continue;
    const f=fit(x,y);if(!f)continue;const r=Math.abs(f.r);
    if(r>=0.8&&(!best||r>best.score+1e-9))
     best={id,s:st,l:len,be,sg,score:r,k:f.k,o:f.o,lag,mn:Math.min(...xs),mx:Math.max(...xs)}}}
  if(best)res.push(best)}
 return res}
/** teachMotion over every identifier: @p S maps each step to its polls (an
    identifier to {b}), @p R each step to the reference per poll. Best first,
    one per stretch of bytes, at most 8. */
function teachFindMotion(S,R,steps){
 const ids=new Set();steps.forEach(k=>(S[k]||[]).forEach(poll=>poll.forEach((x,id)=>ids.add(id))));
 const res=[];
 for(const id of ids){
  if(id>=0x7DF&&id<=0x7EF)continue;
  const X=[],Y=[];
  for(const k of steps)(S[k]||[]).forEach((poll,i)=>{const x=poll.get(id);if(x){X.push(x.b);Y.push((R[k]||[])[i])}});
  if(X.length<12)continue;
  res.push(...teachMotion(id,X,Y,Math.max(...X.map(b=>b.length))))}
 res.sort((a,b)=>b.score-a.score);
 const out=[];
 for(const r of res){
  const k0=r.s>>3,by=r.l>8?[k0,k0+1]:[k0];
  if(out.some(t=>t.id===r.id&&by.some(b=>t.by.includes(b))))continue;
  out.push({...r,by});if(out.length>=8)break}
 return out}
/** A trouble-code channel's value (raw | kind << 16 | module << 18) as "P1718". */
function dtcName(v){const raw=(Math.round(v)>>>0)&0xFFFF,h=x=>x.toString(16).toUpperCase();
 return'PCBU'[raw>>14]+((raw>>12)&3)+h((raw>>8)&15)+h((raw>>4)&15)+h(raw&15)}
/** A drive log (.tlg, see DriveLog.h) as CSV: time in seconds, then one column
    per value, headed by @p name(id); a value that was not fresh is empty. */
function tlgToCsv(b,name){
 if(b.length<16||String.fromCharCode(b[0],b[1],b[2],b[3])!=='TLG1')return'';
 const dv=new DataView(b.buffer,b.byteOffset,b.byteLength),n=b[6],ids=[];
 for(let i=0;i<n;i++)ids.push(dv.getUint16(16+2*i,true));
 const row=4+4*n,q=x=>'"'+String(x).replace(/"/g,'""')+'"',lines=['time_s,'+ids.map(id=>q(name(id))).join(',')];
 for(let p=16+2*n;p+row<=b.length;p+=row){
  const c=[(dv.getUint32(p,true)/1000).toFixed(3)];
  for(let i=0;i<n;i++){const v=dv.getFloat32(p+4+4*i,true);c.push(Number.isFinite(v)?String(+v.toPrecision(7)):'')}
  lines.push(c.join(','))}
 return lines.join('\r\n')+'\r\n'}
/*</teach-core>*/
const isSwitch=id=>(id&0xFF00)===0x1200||(id&0xFF00)===0x2100||id===0x0101;
const TEACH_FIRST=id=>(id&0xFF00)===0x1200||(id>=0x1005&&id<=0x10FF)||(id&0xFF00)===0x3000||(id&0xFF00)===0x2100;
let TEACH=null,TRES=[],TTOK=[];
$('#tM').innerHTML='<optgroup label="Body, chassis and custom">'+metricOpts(0x1201,TEACH_FIRST)+'</optgroup><optgroup label="Everything else">'+metricOpts(0,id=>!TEACH_FIRST(id))+'</optgroup>';
/** What can be taught for @p m: positions only for a lever or selector, the
    indicators' run only for the indicators. */
const isMotion=id=>id>=0x100C&&id<=0x100E;
function teachKinds(m){
 if(TEACH_POS[m])return[['pos','Positions'],['val','A moving value']];
 if(isMotion(m))return[['mot','A corner or a hard stop'],['val','A moving value']];
 if(m===TURN_ROLE.L||m===TURN_ROLE.R||m===TURN_ROLE.H)return[['turn','Left, right and hazard, in one go'],['bit','On / off']];
 return isSwitch(m)?[['bit','On / off'],['val','A moving value']]:[['val','A moving value'],['bit','On / off']]}
/* What the chosen kind of teaching asks of the driver, said before Start. */
function teachHow(m,kind){
 if(kind==='turn')return'One run of about a minute teaches all three: the left indicator, the right one, then the hazard lights, 8 seconds each with a pause between - so the master can tell the two sides apart. Do each when the page asks (a second or two late is fine), and switch it off when it says Off.';
 if(kind==='pos')return m===0x100F?'Foot on the brake, and move the lever through the positions as asked. It ends back where it started, so every bit is checked twice.':
  'Select the positions as asked. End on the first one again, so every bit is checked twice.';
 if(kind==='mot')return(m===0x100D?'Drive straight on, then brake firmly to a stop when asked.':'Drive straight on, then through a corner, or a few, when asked.')+
  ' The master compares every field with what the speed and the steering say, and works out the scale.';
 if(kind==='bit')return'Leave it off, switch it on when asked (flashing is fine), then off again.';
 return'Hold it still, then move it slowly through its whole range, back and forth.'}
function teachKindShown(){const k=$('#tK').value;$('#tPRow').hidden=k!=='pos';$('#tHow').textContent=teachHow(+$('#tM').value,k)}
function teachPick(){const m=+$('#tM').value;
 $('#tK').innerHTML=teachKinds(m).map(([v,t])=>`<option value="${v}">${t}</option>`).join('');
 if(TEACH_POS[m])$('#tP').value=TEACH_POS[m];teachKindShown()}
$('#tM').onchange=teachPick;
$('#tK').onchange=teachKindShown;
teachPick();
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
const plain=h=>h.replace(/<[^>]+>/g,'');
async function teachStart(){
 if(TEACH)return;
 const m=+$('#tM').value,kind=$('#tK').value;
 TTOK=kind==='pos'?$('#tP').value.trim().split(/[\s,]+/).filter(Boolean).slice(0,8):[];
 if(kind==='pos'&&TTOK.length<2){toast('List at least two positions');return}
 if(kind==='mot'&&!isMotion(m)){toast('A corner or a hard stop teaches lateral G, longitudinal G or yaw rate');return}
 // Long steps: a driver a few seconds late or early on the lever still leaves
 // each step a clear middle (restsIn) - 6 and 5 s were too short for one 3 s late.
 const off=k=>[k,'<b>Off</b>: indicators and hazard lights off',7000];
 const steps=kind==='mot'?[['A','Drive straight on at a steady speed',4000],
   ['B',m===0x100D?'Now brake firmly to a stop (or accelerate hard)':'Now drive through a corner, or a few',10000]]:
  kind==='bit'?[['A','Leave it OFF',5000],['B','Turn it ON now, and leave it on (flashing is fine)',6000],['C','Turn it OFF again',5000]]:
  kind==='turn'?[off('A'),['L','<b>Left</b> indicator on - let it flash',8000],off('B'),['R','<b>Right</b> indicator on - let it flash',8000],
   off('C'),['H','<b>Hazard lights</b> on',8000],off('D')]:
  kind==='pos'?TTOK.map((t,i)=>['p'+i,`Put it in <b>${esc(t)}</b> and hold it there`,5000]):
  [['A','Hold it still',3000],['B','Move it slowly through its whole range, back and forth',9000]];
 // A hand takes a moment to do what a step asks: the start of each is not
 // read. A drive is read throughout: it is compared with the car's own speed
 // and steering, poll by poll.
 const skip=kind==='mot'?0:kind==='val'?800:1500;
 const url=kind==='mot'?'/api/bus?m=269,4102,4110,4108':'/api/bus?e=1';
 TEACH={m,kind,S:{},P:{}};steps.forEach(([k])=>{TEACH.S[k]=[];TEACH.P[k]=[]});
 $('#tBtn').disabled=true;$('#tRes').innerHTML='';
 // How fast does the master answer right now? With the engine running and the
 // bus busy it can take seconds; a deadline and steps sized for a fast one
 // would then throw every reading away. Three timed readings, the slowest
 // counts, and the run is planned around it (teachPlan).
 $('#tStep').innerHTML='<div class="step">Get ready</div><span class="muted">checking how fast the master answers…</span>';
 let slowest=0,answered=0;
 for(let n=0;n<3;n++){const q=performance.now();
  try{await req(url,null,8000);answered++;slowest=Math.max(slowest,performance.now()-q)}catch(e){}}
 if(!answered){toast('The master does not answer - is this phone still on its Wi-Fi?');$('#tStep').innerHTML='';TEACH=null;$('#tBtn').disabled=false;return}
 const plan=teachPlan(steps.map(s=>s[2]),skip,slowest),durs=plan.durs,total=durs.reduce((a,b)=>a+b,0);
 TLAST={v:1,kind,m,toks:TTOK.slice(),steps:steps.map((s,i)=>[s[0],durs[i]]),skip,answer_ms:Math.round(slowest),deadline:plan.deadline,reads:[]};
 // The steps run on the clock, not on the master's answers: the countdown is
 // redrawn five times a second whatever the Wi-Fi does, and a reading counts
 // only for the step it was asked for and answered in (teachSlot) - one that
 // comes back late is dropped, not filed under the step after.
 const t0=performance.now(),at=()=>performance.now()-t0;
 const draw=()=>{const t=at();let s=0,i=0;while(i<durs.length&&t>=s+durs[i])s+=durs[i++];if(i>=durs.length)return;
  const next=steps[i+1]?` · then: ${plain(steps[i+1][1])}`:'';
  $('#tStep').innerHTML=`<div class="step">${steps[i][1]}</div><span class="muted">${Math.ceil((s+durs[i]-t)/1000)} s · step ${i+1} of ${steps.length}${esc(next)}</span>`};
 draw();const tick=setInterval(draw,200);
 let asked=0,late=0;
 try{
  while(at()<total){
   const tq=at();let b=null;asked++;
   try{b=await req(url,null,plan.deadline)}catch(e){}
   const tr=at(),i=b?teachSlot(durs,skip,tq,tr):-1;
   if(b)TLAST.reads.push({t:[Math.round(tq),Math.round(tr)],step:i,c:b.census.filter(c=>!c.ext||isBody(c.id)).map(c=>[c.id,c.d||'',c.e||'']),m:b.m||null});
   if(i>=0){const k=steps[i][0];
    TEACH.S[k].push(new Map(b.census.filter(c=>!c.ext||isBody(c.id)).map(c=>[c.id,{b:hb(c.d||''),e:c.e?hb(c.e):null}])));
    if(kind==='mot'){
     // The car as the master knows it at this poll: speed, steering, and a yaw
     // rate or lateral G already read from the bus (never one derived from these).
     const g=(id,raw)=>{const x=b.m&&b.m[id];return x&&x[1]<1500&&(!raw||x[2]===4)?x[0]:null},sa=b.m&&b.m[269];
     TEACH.P[k].push({t:Date.now()-(sa?sa[1]:0),v:g(269),st:g(4102),yaw:g(4110,1),lat:g(4108,1)})}}
   else if(!b||tr-tq>plan.deadline/2)late++;
   const wait=150-(at()-tq);if(wait>0)await sleep(wait)}
  $('#tStep').innerHTML='<div class="step">Done</div>';
  const thin=steps.map(([k,t],i)=>TEACH.S[k].length<TEACH_MIN_READS?`step ${i+1} (${plain(t)})`:'').filter(Boolean);
  const got=steps.reduce((n,[k])=>n+TEACH.S[k].length,0);
  if(got<steps.length*2){
   // Too little to judge: say so, rather than "nothing followed".
   $('#tRes').innerHTML=`<p class="say warn" style="margin-top:8px">The master did not answer in time: only ${got} usable reading${got===1?'':'s'} in ${steps.length} steps (${late} of ${asked} late or lost; it took up to ${Math.round(slowest)} ms before the start). Nothing could be judged. Stay close to the master, close other pages open on it, and try again.</p>`}
  else{
   teachAnalyze(steps.map(p=>p[0]));
   if(thin.length)$('#tRes').insertAdjacentHTML('afterbegin',`<p class="say warn" style="margin-top:8px">The master answered slowly: too few readings in ${esc(thin.join(', '))} - ${late} of ${asked} came back late or not at all. `+
    `${TRES.length?'What was found may be wrong or incomplete. ':''}Stay close to the master, close other pages open on it, and try again.</p>`)}
  $('#tRes').insertAdjacentHTML('beforeend',`<p class="say" style="margin-top:8px"><a href="#" class="acc" onclick="teachSave();return false">Save these readings</a> - a file with everything the master sent during this run, to look into a teach that did not work.</p>`)}
 finally{clearInterval(tick);TEACH=null;$('#tBtn').disabled=false}}
let TLAST=null;
function teachSave(){if(!TLAST)return;
 const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([JSON.stringify(TLAST)],{type:'application/json'}));
 a.download=`teach-${TLAST.kind}-${new Date().toISOString().slice(0,19).replace(/[:T]/g,'-')}.json`;a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000)}
const ROLE_NAME={L:'Left turn signal',R:'Right turn signal',H:'Hazard lights',both:'Either indicator'};
const ROLE_SAY={L:' with the left indicator only',R:' with the right indicator only',H:' with the hazard lights only',
 both:' with either indicator: it cannot tell the side, so it is not used'};
function teachAnalyze(steps){
 const {S,kind,m}=TEACH;
 if(kind==='mot'){const R={};steps.forEach(k=>R[k]=motionRef(m,TEACH.P[k]));TRES=teachFindMotion(S,R,steps)}
 else TRES=teachFind(kind,S,steps,TTOK);
 const nums=kind==='turn'&&TRES.numbers&&TRES.numbers.length?TRES.numbers:null;
 if(!TRES.length&&nums){
  // Only a value moved with the lamps (their load): say what it was, offer nothing.
  const ids=[...new Set(nums.map(x=>x.id))];
  $('#tRes').innerHTML=`<p class="say">Nothing on the bus says the indicators are on. Only part of a number moved with them - `+
   ids.map(id=>field({id,s:nums.find(x=>x.id===id).s,l:1})).join(', ')+
   ` - a value that changes all the time; it moved with the lamps' load, whichever side was on, so it is not offered. `+
   `This car may not send its indicators on this bus. Save the readings below and send them.</p>`;return}
 if(!TRES.length){$('#tRes').innerHTML=`<p class="say">Nothing on the bus followed that. ${kind==='pos'?'Hold each position until the next is asked for, or the lever may not be broadcast.':
  kind==='turn'?'Let each one flash through its step and switch it off when the page says Off. If it still finds nothing, the indicators may not be broadcast: save the readings below and send them.':
  'Try again with a clearer on/off, or the value may not be broadcast.'}</p>`;return}
 const pat=(c,n)=>Array.from({length:n},(_,i)=>(c>>i)&1).join('');
 const say=r=>r.how==='gated'?`the flasher (bit ${r.flasher}, both sides) while bit ${r.side} is ${r.sideSet?'set':'clear'} - this car sends the side apart from the flashing`:
  r.role?(r.how==='flash'?`flashed ${r.blink} time${r.blink>1?'s':''}`:'held on')+ROLE_SAY[r.role]+(r.heldH&&r.role!=='H'?' (and with the hazard lights)':'')+(r.inv?' (inverted)':''):
  r.codes?r.names.map((t,g)=>`${esc(t)} <span class="mono">${pat(r.codes[g],r.bits.length)}</span>`).join(' • '):
  r.pos!==undefined?`on only in ${esc(r.names[r.pos])}${r.inv?' (inverted)':''} - as ${esc(nm(r.metric))}`:
  kind==='mot'?`followed ${(r.score*100).toFixed(0)}% · scale ${+r.k.toPrecision(3)}${Math.abs(r.o)>1e-3?', offset '+(+r.o.toPrecision(3)):''}${m!==0x100D?' (approximate)':''}`:
  r.l===1?(r.blink?`flashed ${r.blink} time${r.blink>1?'s':''}${r.inv?' (inverted)':''}`:`followed ${(Math.min(1,r.score)*100).toFixed(0)}%${r.inv?' (inverted)':''}`):
  `raw ${r.mn} … ${r.mx} while moving`;
 let head='<p class="say" style="margin-top:8px">These followed you, best first:</p>';
 if(kind==='turn'){
  const got=['L','R','H'].filter(k=>TRES.some(r=>r.role===k)),miss=['L','R','H'].filter(k=>!got.includes(k));
  const either=TRES.find(r=>r.role==='both'),sides=miss.filter(k=>k!=='H');
  head=`<p class="say" style="margin-top:8px">${got.length?'Found '+got.map(k=>'<b>'+ROLE_NAME[k]+'</b>').join(', ')+'. ':''}`+
   `${miss.length?'Nothing only for '+miss.map(k=>ROLE_NAME[k].toLowerCase()).join(' or ')+'. ':''}`+
   (sides.length&&either?`${field({id:either.id,s:either.s,l:1})} flashes for either side, and nothing in the run said which side it was - save the readings below and send them. `:'')+
   (miss.includes('H')&&got.includes('L')&&got.includes('R')?'With both sides found, the screens show the hazard lights as both arrows anyway. ':'')+
   `The first under each is the best.</p>`+
   (got.length>1?'<div class="row" style="margin:0 0 6px"><button class="b p" onclick="teachUseAll()">Use the first of each</button></div>':'')}
 $('#tRes').innerHTML=head+TRES.map((r,i)=>
  `<div class="lr"><div class="lt"><span>${r.role?`<b>${ROLE_NAME[r.role]}</b> · `:''}${field({id:r.id,s:r.s,l:r.l,be:r.be,bits:r.bits})}${r.sg?' <span class="muted">signed</span>':''}</span>
   ${r.role==='both'?'':`<button class="b s p" onclick="teachUse(${i})">${r.l===1||r.codes?'Use':'Use…'}</button>`}</div>
   <div class="ls"><span>${say(r)}</span></div></div>`).join('')}
/** The raw signals feeding @p metric now: taught, mapped by hand or learned. */
const feeding=metric=>(C.signals||[]).filter(g=>g.metric===metric&&(g.mode===1||g.mode===3));
const sameSig=(g,x)=>g.can_id===x.can_id&&g.start===x.start&&g.len===x.len&&String(g.bits||'')===String(x.bits||'');
const where=g=>canName(g.can_id)+(g.bits&&g.bits.length?' bits '+g.bits.join(' '):g.len===1?' bit '+g.start:` bits ${g.start}-${g.start+g.len-1}`);
function teachSig(r,m){
 const s={name:nm(m).slice(0,15),can_id:r.id,ext:isBody(r.id),start:r.s,len:r.l,be:false,signed:false,scale:1,offset:0,metric:m,mode:1,ref:0,learned:false};
 if(r.codes){s.bits=r.bits;s.map=r.codes.map((c,g)=>[c,posValue(m,r.names[g],g)])}
 else if(r.map){s.bits=r.bits;s.map=r.map}            // the flasher while the side's bit says so
 else if(r.inv){s.scale=-1;s.offset=1}
 return s}
/** Put taught signals in, each replacing whatever fed its value before - once
    asked. Two sources for one value would fight over it: an old lever table
    beside a new one, or both indicators on the bit that flashes for either. */
async function teachAdd(list){
 if(!LOADED){toast('Not loaded yet');return false}
 const add=[],old=[],say=[];
 for(const s of list){
  const o=feeding(s.metric);
  if(o.some(g=>sameSig(g,s))){toast(`${nm(s.metric)} is already read from there`);continue}
  add.push(s);if(o.length){old.push(...o);say.push(`${nm(s.metric)}: ${o.map(where).join(' and ')}`)}}
 if(!add.length)return false;
 if(say.length&&!confirm(`Read from the bus already:\n\n${say.join('\n')}\n\nReplace ${say.length>1?'these':'it'} with what you just taught?`))return false;
 C.signals=(C.signals||[]).filter(g=>!old.includes(g)).concat(add);
 setDirty(true);await save();return true}
async function teachUse(i){
 const r=TRES[i],m=r.metric||+$('#tM').value;
 if(!r.codes&&r.l!==1){BUS=BUS||{census:[]};
  if(r.k!==undefined){mapOpen(r.id,isBody(r.id)?1:0,{s:r.s,l:r.l,be:r.be,sg:r.sg,m,k:+r.k.toPrecision(3),o:+r.o.toPrecision(3)});
   toast('Check it reads right (round the scale), then Use this value');return}
  mapOpen(r.id,isBody(r.id)?1:0,{s:r.s,l:r.l,be:r.be,sg:r.sg,m});
  toast('Set the scale so the value reads right, then Use this value');return}
 if(!await teachAdd([teachSig(r,m)]))return;
 toast(`${nm(m)} is now read from the bus`);
 if(r.metric===undefined)$('#tRes').innerHTML=''}   // several of these can be used, one by one
async function teachUseAll(){
 const pick=['L','R','H'].map(k=>TRES.find(r=>r.role===k)).filter(Boolean);
 if(!await teachAdd(pick.map(r=>teachSig(r,r.metric))))return;
 toast(pick.map(r=>nm(r.metric)).join(', ')+': now read from the bus');$('#tRes').innerHTML=''}

/* ───────────── sources ───────────── */
const SMODE=['Off','On','Checking','Verified','Rejected'];
function curVal(id){const m=L&&L.metrics.find(x=>x.id===id);if(!m)return'<span class="muted">—</span>';
 const s=SRC[m.src]||['?','no'];return`${fmt(id,m.v)} <span class="muted">${esc(un(id))}</span> <span class="tag ${s[1]}">${s[0]}</span>`}
function renderSources(){
 if(!LOADED)return;const adv=$('#srcEd').checked;
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
 $('#oSub').textContent=o.answers===1?`ECU supports ${o.supported} PIDs · ${o.active?'active':'idle'} · ${o.physical?'engine ECU only':'all ECUs'}`+
  (o.lat?` · answers in ${o.lat.toFixed(0)} ms (slowest ${o.lat_max.toFixed(0)} ms)`:'')+(o.late?` · ${o.late} came late`:''):o.answers===2?'the ECU does not answer':'not asked yet';
 $('#mSub').textContent=(s.init?`ECU supports ${s.supported} of ${s.total} · ${s.active?(s.rate||0).toFixed(0)+' requests/s, '+s.batch+' values each':'idle — OBD-II covers it'}`:s.answers===2?'the ECU does not answer':'waits for OBD-II')+
  (s.refused?` · ${s.refused} refused by the ECU`:'')+(s.last_err?` · last problem: ${s.last_err}`:'');
 $('#sEcu').innerHTML=s.ecu_id?`ECU <b>${esc(s.ecu_id)}</b> · SYS ${esc(s.sys_id)} · ${s.flags} capability bytes${s.init?'':' (remembered)'} · this master ${esc(L.mac)}`:`This master ${esc(L.mac)}`}

/* ───────────── the evidence log (Bus tab) ───────────── */
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
function pollEvlog(){return solo('ev',pollEvlogOnce)}
async function pollEvlogOnce(){
 if(TAB!=='bus'||TEACH||UPD)return;
 let d;try{d=await req('/api/evlog',null,5000)}catch(e){return}
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

/* ───────────── the raw recording (Bus tab) ───────────── */
let RR=null;
function pollRaw(){return solo('rr',pollRawOnce)}
async function pollRawOnce(){
 if(TAB!=='bus'||TEACH||UPD)return;
 try{RR=await req('/api/rawrec',null,5000)}catch(e){return}
 renderRaw()}
function renderRaw(){
 const d=RR;if(!d)return;
 const st=d.state,has=['recording','full','stopped'].includes(st),left=d.rate&&d.cap>d.used?(d.cap-d.used)/d.rate:0;
 $('#rrSub').textContent=has?`${kb(d.used)} of ${kb(d.cap)}`+(st==='recording'&&left?` · about ${dur(left)} of bus left`:''):'';
 $('#rrBarBox').hidden=!(has||st==='erasing');
 $('#rrBar').style.width=(st==='erasing'?d.erase:d.cap?100*d.used/d.cap:0).toFixed(1)+'%';
 const counts=`${d.frames} frames since the master started (${d.tx} of them its own)`+
  (d.lost?`, <span class="warn">${d.lost} lost</span>`:', none lost')+(d.rate?` · ${kb(d.rate*60)} a minute`:'');
 $('#rrSay').innerHTML=
  st==='none'?'<span class="warn">This master\'s partition table has no log space</span>: a USB upload of this firmware adds it.':
  st==='off'?'Not recording: the log space is the drive logs\'.':
  st==='erasing'?`Erasing the log space: ${d.erase} %…`:
  st==='recording'?`<span class="ok">● Recording</span> whenever the car is on, session ${d.session}: ${counts}.`:
  `<span class="warn">${esc((d.note||st).replace(/^./,c=>c.toUpperCase()))}</span>`+(d.size>4096?` · ${kb(d.size)} to download.`:'.')+
   (d.frames?`<br>${counts}.`:'');
 const b=(t,f,c)=>`<button class="b${c?' '+c:''}" onclick="${f}">${t}</button>`;
 $('#rrBtns').innerHTML=st==='off'?b('Start recording…','rawArm()','p'):
  st==='recording'?b('Stop','rawStop()','d')+b('Download so far','rawGet()'):
  st==='full'||st==='stopped'?b('Download','rawGet()','p')+b('Record again…','rawArm()')+b('Back to drive logs…','rawRelease()','d'):''}
async function rawAct(u,msg){
 try{const r=await fetch(u,{method:'POST'}),j=await r.json();toast(r.ok?msg:j.error||'Failed')}catch(e){toast('Failed')}
 setTimeout(pollRaw,500)}
function rawArm(){
 if(confirm('Erase the log space - every drive log in it - and record everything on the bus?\n\nThe erase takes up to a minute and '+
  'needs the car standing: the master hears little of the bus meanwhile. The recording then runs whenever the car is on, until '+
  'you stop it or the 8 MB are full (roughly 20 minutes of this car\'s bus).'))rawAct('/api/rawrec/arm','Erasing the log space…')}
function rawStop(){if(confirm('Stop the raw recording? It stays on the master for download until you record again.'))rawAct('/api/rawrec/stop','Stopped')}
function rawRelease(){if(confirm('Erase the raw recording and give the log space back to the drive logs?'))rawAct('/api/rawrec/release','Drive logs back')}
/* The download, read here as it comes (up to 8 MB over the master's Wi-Fi,
   half a minute or more), with nothing else polled meanwhile. */
async function rawGet(){
 if(UPD)return;
 UPD=true;
 const c=new AbortController();let stall=setTimeout(()=>c.abort(),15000);
 try{const r=await fetch('/api/rawrec/file',{signal:c.signal});
  if(!r.ok)throw new Error('HTTP '+r.status);
  const total=+r.headers.get('Content-Length')||0,rd=r.body.getReader(),parts=[];let got=0,shown=0;
  for(;;){const {done,value}=await rd.read();if(done)break;
   clearTimeout(stall);stall=setTimeout(()=>c.abort(),15000);
   parts.push(value);got+=value.length;
   if(Date.now()-shown>300){shown=Date.now();$('#rrSay').textContent=`Downloading… ${kb(got)}${total?' of '+kb(total):''}`}}
  if(total&&got!==total)throw new Error('cut off');
  const a=document.createElement('a');a.href=URL.createObjectURL(new Blob(parts,{type:'application/octet-stream'}));
  a.download='canrec-'+new Date().toISOString().slice(0,16).replace(/[-:]/g,'').replace('T','-')+'.bin';a.click();
  setTimeout(()=>URL.revokeObjectURL(a.href),1000);toast('Downloaded')}
 catch(e){toast('Download failed')}
 finally{clearTimeout(stall);UPD=false;pollRaw()}}

/* ───────────── the Car tab: trouble codes, the engine computer, oil change, drive logs ───────────── */
const KIND=['stored','pending','permanent'],MOD=['Engine','Transmission'],LIST_SAY=['not asked yet','answered','no reply','not kept'];
let CODES=null,LOGS=null;
const ago=s=>s<90?`${s} s ago`:s<5400?`${Math.round(s/60)} min ago`:`${Math.round(s/3600)} h ago`;
function pollCodes(){return solo('codes',pollCodesOnce)}
async function pollCodesOnce(){
 if(TAB!=='car'||TEACH||UPD)return;
 try{CODES=await req('/api/codes',null,5000)}catch(e){return}
 renderCodes()}
function renderCodes(){
 const d=CODES;if(!d)return;
 $('#dcSub').textContent=d.busy?'reading…':d.read?`read ${ago(d.age)}`:'not read yet';
 $('#dcTab').innerHTML=d.codes.length?d.codes.map(c=>`<tr><td class="mono ${c.kind===0?'crit':'warn'}"><b>${esc(c.code)}</b></td>
  <td style="white-space:normal">${esc(c.text)}</td><td>${MOD[c.mod]||'?'}</td><td>${KIND[c.kind]||'?'}</td></tr>`).join(''):
  `<tr><td colspan="4" class="empty">${d.read?'No codes in either computer.':'Read a few seconds after the car comes on, or press Read now.'}</td></tr>`;
 const lists=(d.lists||[]).map((row,m)=>`${MOD[m]}: `+row.map((v,k)=>`${KIND[k]} ${LIST_SAY[v]||'?'}`).join(', ')).join(' · ');
 $('#dcSay').innerHTML=(d.note?`<b>${esc(d.note)}</b>. `:'')+esc(lists)+(d.tcm_func?' · the transmission answers only on 0x7DF':'')+
  (d.clear?`<br>Last clear: ${esc(d.clear.replace(/^Clear: ?/,''))}`:'');
 const id=d.ident||{},row=(k,v,s)=>`<tr><td class="muted">${k}</td><td class="mono"><b>${v?esc(v):'<span class="muted">—</span>'}</b>${s?` <span class="muted">${s}</span>`:''}</td></tr>`;
 $('#idTab').innerHTML=row('Calibration ID',id.cal,id.cal2?'and '+esc(id.cal2):'OBD-II 09 04')+row('ROM ID',id.rom,'SSM2')+
  row('SYS ID',id.sys,'SSM2')+row('VIN',id.vin,'')+row('CVN',id.cvn,'')+
  (id.cal||id.rom?'':`<tr><td colspan="2" class="empty">Read with the ignition on.</td></tr>`)+
  (id.cal||id.rom?`<tr><td colspan="2" class="muted" style="white-space:normal">${id.live?'Read from the car just now.':'Remembered from the last time the car answered.'}</td></tr>`:'');
 const sv=d.service||{};
 if(sv.at<0)$('#svSay').innerHTML='Set the odometer reading of the last oil change, or press <b>Oil changed now</b>: the screens then show a lamp when the next one is due.';
 else{const due=sv.at+sv.every;
  $('#svSay').innerHTML=sv.odo==null?`Next oil change at <b>${due} km</b>. The odometer has not been read yet (SSM2, with the ignition on).`:
   sv.odo>=due?`<span class="crit">Overdue by <b>${sv.odo-due} km</b></span> (odometer ${sv.odo} km, due at ${due} km).`:
   `Next oil change at <b>${due} km</b>: <b>${due-sv.odo} km</b> to go (odometer ${sv.odo} km).`}
 $('#svSub').textContent=sv.odo!=null?`odometer ${sv.odo} km`:''}
async function codesClear(){
 if(!confirm('Clear the trouble codes in the engine and the transmission computer?\n\nThe check-engine lamp goes out and the readiness '+
  'monitors start again: an emissions test then needs a few drives first. A code whose fault is still there comes back.\n\n'+
  'The car must be standing; if the engine computer refuses, switch the engine off and leave the ignition on.'))return;
 try{const r=await fetch('/api/codes/clear',{method:'POST'}),j=await r.json();toast(r.ok?'Clearing…':j.error||'Failed');
  setTimeout(pollCodes,3000)}catch(e){toast('Failed')}}
async function serviceReset(){
 if(!confirm('Oil changed now: the reminder starts again from the odometer?'))return;
 try{const r=await fetch('/api/service/reset',{method:'POST'}),j=await r.json();toast(r.ok?'Reminder reset':j.error||'Failed');
  if(r.ok)await load(true);pollCodes()}catch(e){toast('Failed')}}
/* ───────────── the body computer (Car tab) ───────────── */
/* Where FastECU's table for the 2005-09 cars' body computer (over K-line)
   puts these in its block 50 - bit = byte x 8 + bit, after "61 50": a place
   to start looking on this car, not a fact about it. */
const BIU_BITS=[[8,"Driver's door",0x1210],[9,"Passenger's door",0x1211],[11,'Rear left door',0x1212],
 [10,'Rear right door',0x1213],[12,'Boot',0x1214],[32,'Handbrake',0x1205],[4,'Light switch',0x2108]];
const OBD_NAME={0x7E0:'the engine computer',0x7E1:'the transmission computer'};
let BODYS=null;
function pollBody(){return solo('body',pollBodyOnce)}
async function pollBodyOnce(){
 if(TAB!=='car'||TEACH||UPD)return;
 try{BODYS=await req('/api/body',null,5000)}catch(e){return}
 renderBody();
 if(BODYS.scanning)setTimeout(pollBody,1000)}
const hexs=a=>a&&a.length?a.map(b=>b.toString(16).toUpperCase().padStart(2,'0')).join(' '):'—';
function renderBody(){
 const d=BODYS;if(!d)return;
 const r=d.reader||{},found=d.found||[],rx0=tx=>C.biu_rx||tx+8;
 $('#bdSub').textContent=d.scanning?`asking ${hex(d.at,3)}…`:r.tx?`reading ${hex(r.tx,3)}`:'';
 $('#bdBarBox').hidden=!d.scanning;$('#bdBar').style.width=(d.pct||0)+'%';$('#bdBtn').disabled=!!d.scanning;
 const said=f=>f.biu?`<span class="ok">a body computer</span>: block 50, ${f.len} bytes`:
  f.reply[0]===0x7E?'present':f.reply[0]===0x7F?`answers, but not to 3E (refusal ${hex(f.reply[2]||0,2)})`:'answers: '+hexs(f.reply);
 $('#bdTab').innerHTML=found.length?found.map(f=>`<tr><td class="mono">${hex(f.tx,3)}</td><td class="mono">${hex(f.rx,3)}</td>
  <td style="white-space:normal">${said(f)}${OBD_NAME[f.tx]?` <span class="muted">(${OBD_NAME[f.tx]})</span>`:''}</td>
  <td>${!f.biu?'':C.biu_tx===f.tx&&rx0(f.tx)===f.rx?'<span class="ok">in use</span>':`<button class="b s p" onclick="bodyUse(${f.tx},${f.rx})">Use</button>`}</td></tr>`).join(''):
  `<tr><td colspan="4" class="empty">${d.scanning?'Asking…':d.scanned?'Nothing answered.':'Not looked yet.'}</td></tr>`;
 $('#bdSay').innerHTML=(d.note?`<span class="warn">Stopped: ${esc(d.note)}</span>. `:'')+
  (d.scanned&&!found.some(f=>f.biu)?'No computer answered 21 50 as a body computer: on this car its address or its requests are '+
   'others. Record the bus while your scan tool reads the body computer (Bus tab, Raw recording, with the master\'s requests off) '+
   'and send the file: the tool\'s requests and the answers are in it. ':'')+
  (d.others&&d.others.length?`Also heard during the scan, from no address asked: ${d.others.map(x=>hex(x,3)).join(', ')}.`:'');
 if(!r.tx){$('#bdRead').innerHTML='<p class="say" style="margin:0">Off. <b>Use</b> on a body computer the scan found starts reading it; '+
  'its addresses can also be set by hand (Settings, Advanced, Requests).</p>';return}
 const b0=(r.blocks||[])[0]||[],b1=(r.blocks||[])[1]||[];
 const bit=(blk,n)=>blk.length>n>>3?(blk[n>>3]>>(n&7))&1:null,row=(lid,n)=>BODY|lid<<8|n>>6;
 $('#bdRead').innerHTML=`<p class="say" style="margin:0 0 8px">${hex(r.tx,3)} → ${hex(r.rx,3)}, every ${C.biu_ms||500} ms: ${r.reads} answers, `+
  `${r.fails} failed · ${r.age==null?'no answer yet':r.age<2000?'last answer just now':'last answer '+ago(Math.round(r.age/1000))}`+
  `${r.note?` · <span class="warn">${esc(r.note)}</span>`:''}</p>`+
  `<div class="wrap"><table><thead><tr><th>Block 50, as FastECU names it</th><th>Bit</th><th>Now</th><th></th></tr></thead><tbody>`+
  BIU_BITS.map(([n,name,m])=>{const v=bit(b0,n);return`<tr><td>${name}</td><td class="mono">${n}</td>
   <td>${v==null?'<span class="muted">—</span>':v?'<b class="warn">on</b>':'off'}</td>
   <td><button class="b s" onclick="mapOpen(${row(0x50,n)},1,{s:${n&63},l:1,m:${m}})">Map</button></td></tr>`}).join('')+
  `</tbody></table></div><p class="say mono" style="margin:8px 0 0">50: ${hexs(b0)}<br>51: ${hexs(b1)}</p>`+
  `<p class="say" style="margin:6px 0 0">Check each on this car before you use it - open the door, watch its bit. Every byte is also `+
  `in the Bus tab's frame table (<b>BIU 50 · 0-7</b> and on), where <b>Map</b> and teach by doing work on them as on any frame.</p>`}
async function bodyScan(){
 try{const r=await fetch('/api/body/scan',{method:'POST'}),j=await r.json();toast(r.ok?'Asking every address…':j.error||'Failed')}
 catch(e){toast('Failed')}
 setTimeout(pollBody,500)}
async function bodyUse(tx,rx){
 if(!LOADED){toast('Not loaded yet');return}
 C.biu_tx=tx;C.biu_rx=rx===tx+8?0:rx;setDirty(true);await save();renderBody()}
const kb=b=>b<1024?b+' B':b<1048576?(b/1024).toFixed(0)+' KB':(b/1048576).toFixed(2)+' MB';
const dur=s=>{s=Math.round(s);return s<60?s+' s':Math.floor(s/60)+' min '+String(s%60).padStart(2,'0')+' s'};
function pollLogs(){return solo('logs',pollLogsOnce)}
async function pollLogsOnce(){
 if(TAB!=='car'||TEACH||UPD)return;
 try{LOGS=await req('/api/logs',null,5000)}catch(e){return}
 const d=LOGS;
 $('#lgSub').textContent=d.raw?'':`${d.used_kb} of ${d.cap_kb} KB used · ${d.free_kb} KB free on ${d.own?'the logs partition':'the settings partition (older partition table)'}`;
 $('#lgSay').innerHTML=d.raw?'<span class="warn">Off while a raw bus recording has the log space</span> (Bus tab: Raw recording).':
  !d.on?'Recording is off: switch it on below, under Recording.':
  d.rec?`<span class="ok">Recording <b>${esc(d.file)}</b></span>: ${d.rows} rows so far.`:
  d.note?`<span class="warn">${esc(d.note)}</span>`:'Recording starts when the car comes on.';
 $('#lgTab').innerHTML=d.files.length?d.files.slice().reverse().map(f=>`<tr><td class="mono">${esc(f.name.replace('.tlg',''))}${f.rec?' <span class="ok">● now</span>':''}</td>
  <td>${dur(f.secs)}</td><td>${f.rows}</td><td>${kb(f.bytes)}</td>
  <td><button class="b s p" onclick="logGet('${esc(f.name)}')">CSV</button> <button class="b s d" onclick="logDel('${esc(f.name)}')">Delete</button></td></tr>`).join(''):
  '<tr><td colspan="5" class="empty">No drives recorded.</td></tr>'}
async function logGet(n){
 toast('Downloading…');
 try{const b=await fetch('/api/log?f='+encodeURIComponent(n)).then(r=>{if(!r.ok)throw 0;return r.arrayBuffer()});
  const csv=tlgToCsv(new Uint8Array(b),id=>nm(id)+(un(id)?' ('+un(id)+')':''));if(!csv)throw 0;
  const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([csv],{type:'text/csv'}));
  a.download='drive-'+n.replace('.tlg','')+'.csv';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000)}
 catch(e){toast('Download failed')}}
function logDel(n){if(confirm('Delete drive '+n.replace('.tlg','')+'?'))post('/api/log/delete?f='+encodeURIComponent(n),'Deleted',pollLogs)}
function renderLogIds(){
 const ids=C.log_ids||[];if(!$('#lgIds'))return;
 $('#lgIds').innerHTML=ids.map((id,i)=>`<span class="pill">${esc(nm(id))}<button class="x" onclick="logRm(${i})">✕</button></span>`).join('')||
  '<span class="muted">Nothing chosen: nothing is recorded.</span>';
 const hz=C.log_hz||10,bps=(4+4*ids.length)*hz,cap=(C.log_max_kb||8000)*1024;
 $('#lgEst').textContent=ids.length?`${ids.length} of 24 · ${(bps/1024).toFixed(2)} KB a second · about ${Math.round(cap/bps/60)} min of driving fit`:'';
 if(document.activeElement!==$('#lgAdd'))$('#lgAdd').innerHTML='<option value="0">Add a value…</option>'+metricOpts(-1,id=>!ids.includes(id))}
function logAdd(){const id=+$('#lgAdd').value,ids=C.log_ids||[];if(!id||ids.length>=24)return;C.log_ids=ids.concat(id);setDirty(true);renderLogIds()}
function logRm(i){C.log_ids=(C.log_ids||[]).filter((_,k)=>k!==i);setDirty(true);renderLogIds()}

/* ───────────── polling ─────────────
   While a teach runs, nothing else is asked: its readings have the master to
   themselves, and are not held up behind a full status document. */
function poll(){return solo('live',pollOnce)}
async function pollOnce(){
 if(TEACH||UPD)return;
 let d;try{d=await req('/api/live',null,4000)}catch(e){
  const c=$('#conn');c.className='pill crit';c.lastChild.textContent='offline';return}
 const now=Date.now();
 // The master restarted (its counters went back): no rate from across that.
 // (bits is a 32-bit count that wraps every few hours: >>>0 takes the difference across it)
 if(PREV&&(d.bus.rx<PREV.rx||d.bus.tx<PREV.tx||d.bus.err<PREV.err))PREV=null;
 if(PREV){const dt=(now-PREV.t)/1000;if(dt>0){
  RATES.fps=(d.bus.rx-PREV.rx)/dt;RATES.tx=(d.bus.tx-PREV.tx)/dt;RATES.errs=d.bus.err-PREV.err;
  RATES.load=100*(((d.bus.bits-PREV.bits)>>>0)+(d.bus.tx-PREV.tx)*125)/(dt*(d.bus.bitrate||500)*1000)}}
 PREV={t:now,rx:d.bus.rx,tx:d.bus.tx,err:d.bus.err,bits:d.bus.bits};
 L=d;
 // Pick up what the learner and verifier did to the signal table.
 const lsig=(d.learn||[]).filter(x=>x.st>=3).map(x=>x.m+':'+x.st).join();
 if(LOADED&&(lsig!==LSIG||now-LLOAD>30000)){LSIG=lsig;LLOAD=now;solo('cfg',()=>load(true))}
 const c=$('#conn');
 if(!d.bus.alive){c.className='pill warn';c.lastChild.textContent='no CAN'}
 else if(d.silent){c.className='pill ok';c.lastChild.textContent='listening only'}
 else if(d.settle){c.className='pill ok';c.lastChild.textContent=`settling · ${Math.ceil(d.settle/1000)} s`}
 else if(d.guard.pause||d.guard.silent){c.className='pill warn';c.lastChild.textContent='requests paused'}
 else{c.className='pill ok';c.lastChild.textContent='live'}
 if(TAB==='home')renderHome();
 if(TAB==='sig'){renderLearn();updateSourceText()}
 if(TAB==='bus')renderBusTiles();
 if(TAB==='set')updateSourceText()}
/* ───────────── firmware update ───────────── */
async function fwInfo(){
 try{const d=await req('/api/update',{},5000);
  $('#fwInfo').innerHTML=`Version <b>${esc(d.version)}</b> · build ${esc(d.build.slice(0,8))} · slot ${esc(d.slot)}`+
   (d.trial?' · <b>new, on trial</b>: kept once it has run 30 s':'')+
   (d.rolled_back?' · <b>the last update did not run properly</b> - the master went back to this firmware':'')+
   (d.allowed?'':` · updates refused now: ${esc(d.why)}`);
  return d}catch(e){return null}}
async function fwUpload(){
 const f=$('#fwFile').files[0],say=m=>$('#fwSay').textContent=m;
 if(!f){toast('Choose the firmware .bin first');return}
 if(!/\.bin$/i.test(f.name)||f.size<200000||f.size>3300000){toast('That is not a master firmware .bin');return}
 const before=await fwInfo();
 if(!before){toast('The master does not answer - is this phone on its Wi-Fi?');return}
 if(!before.allowed){toast('Not now: '+before.why);return}
 if(!confirm(`Update the master with ${f.name} (${Math.round(f.size/1024)} KB)?\n\nIt restarts into the new firmware and keeps it once it has run 30 s; if it does not run properly, it goes back to the one it has now. Stay near the car until it is back.`))return;
 UPD=true;$('#fwBtn').disabled=true;
 const x=new XMLHttpRequest(),fd=new FormData();fd.append('fw',f,f.name);
 x.upload.onprogress=e=>{if(e.lengthComputable)say(`Sending… ${Math.round(100*e.loaded/e.total)} %`)};
 const done=m=>{UPD=false;$('#fwBtn').disabled=false;say(m)};
 x.onload=()=>{let j={};try{j=JSON.parse(x.responseText)}catch(e){}
  if(x.status===200){say('Checked - the master is restarting into it. If this phone leaves its Wi-Fi, join it again.');fwWait(before.build,done)}
  else done('Not updated: '+(j.error||('HTTP '+x.status))+'. The master runs as before.')};
 x.onerror=x.ontimeout=()=>done('The upload was cut off - nothing changed. Try again.');
 x.timeout=300000;x.open('POST','/api/update');x.send(fd)}
function fwWait(old,done){const t0=Date.now();
 const tick=async()=>{const d=await fwInfo();
  if(d&&d.build!==old&&!d.trial){done('Updated: the new firmware runs and is kept.');load(true);return}
  if(d&&d.build===old&&d.rolled_back){done('The new firmware did not run properly - the master went back to the one it had. Nothing else changed.');return}
  if(d&&d.trial)$('#fwSay').textContent='The new firmware is running - kept once it has run 30 s…';
  if(Date.now()-t0>240000){done('No answer from the master yet - join its Wi-Fi again and open this page.');return}
  setTimeout(tick,3000)};
 setTimeout(tick,6000)}
renderSettings();
try{advShow(localStorage.getItem('advOn')==='1')}catch(e){}
go(location.hash.slice(1));
load();poll();setInterval(poll,1000);setInterval(pollBus,1000);setInterval(pollEvlog,3000);setInterval(pollRaw,2000);
setInterval(pollCodes,3000);setInterval(pollLogs,3000);setInterval(pollBody,3000);
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
static void sendJson(JsonDocument &doc, int code = 200) {
    s_server.setContentLength(measureJson(doc));
    s_server.send(code, "application/json", "");
    WiFiClient client = s_server.client();
    ClientBuffer out(client);
    serializeJson(doc, out);
}

static void sendOk() { s_server.send(200, "application/json", "{\"ok\":true}"); }

/*
 * A settings upload (POST /api/config) is written to the flash as it arrives,
 * 1.4 KB at a time, and applied from there (MasterConfig::applyFile). Held in
 * memory - the server keeps a request's body twice while reading it, and once
 * more until the request is done - the owner's 36 KB file did not fit beside
 * its parsed document and the save's: "Save failed" (2026-11-18). The page
 * sends it as a form's file: a bare body's last piece is read with a 5 s
 * timeout (the server asks for 1436 bytes at a time), a form's is not.
 */
enum UpState : uint8_t { UP_NONE, UP_WRITING, UP_STORED, UP_TOO_BIG, UP_FAILED };
static UpState s_up = UP_NONE;
static File s_upFile;
static size_t s_upBytes = 0;
/** Far above any car's settings (2026-11-17's, 45 signals: 36 KB). */
static constexpr size_t UPLOAD_MAX = 128 * 1024;

static void upChunk(HTTPRawStatus st, const uint8_t *buf, size_t n) {
    switch (st) {
    case RAW_START:
        masterBusStall();                    // flash writes hold the CAN interrupt off
        if (s_upFile) s_upFile.close();
        s_upFile = LittleFS.open(MasterConfig::UPLOAD_PATH, "w");
        s_upBytes = 0;
        s_up = s_upFile ? UP_WRITING : UP_FAILED;
        break;
    case RAW_WRITE:
        if (s_up != UP_WRITING || !n) break;
        if (s_upBytes + n > UPLOAD_MAX) { s_up = UP_TOO_BIG; s_upFile.close(); break; }
        masterBusStall();
        if (s_upFile.write(buf, n) != n) { s_up = UP_FAILED; s_upFile.close(); break; }
        s_upBytes += n;
        break;
    case RAW_END:
        if (s_up == UP_WRITING) { s_upFile.close(); s_up = UP_STORED; }
        break;
    case RAW_ABORTED:                        // the phone went away: nothing answered, nothing kept
        if (s_upFile) s_upFile.close();
        masterBusStall();
        LittleFS.remove(MasterConfig::UPLOAD_PATH);
        s_up = UP_NONE;
        break;
    }
}

/*
 * The lists below are read a row at a time (a copy of each was kept in RAM
 * for these pages alone: 13 KB), in functions of their own that are never
 * inlined: their buffers are on the stack only while the rows are added, not
 * under the sending of the answer, which is the deep part.
 */

/** Every task's stack: {"name":[least left, size]}. */
static void __attribute__((noinline)) addStacks(JsonObject so) {
    StackView sv[12];
    const size_t ns = masterStacks(sv, 12);
    for (size_t i = 0; i < ns; i++) {
        JsonArray a = so[sv[i].name].to<JsonArray>();
        a.add(sv[i].minFree);
        a.add(sv[i].size);
    }
}

/** Every learner slot, plus every value already read from the bus. */
static void __attribute__((noinline)) addLearnRows(JsonArray la) {
    const size_t nl = learnerStatusRows();
    for (size_t i = 0; i < nl; i++) {
        LearnView lv;
        if (!learnerStatusRow(i, lv)) continue;
        JsonObject x = la.add<JsonObject>();
        x["m"]   = lv.metric;
        x["st"]  = lv.state;
        x["p"]   = lv.progress;
        x["fit"] = lv.fit;
        x["n"]   = lv.samples;
        x["id"]  = lv.canId;
        x["s"]   = lv.start;
        x["l"]   = lv.len;
        x["be"]  = lv.be;
    }
}

/** The metric store, slot by slot. */
static void __attribute__((noinline)) addMetricRows(JsonArray ma) {
    const size_t nm = masterMetricSlots();
    for (size_t i = 0; i < nm; i++) {
        MetricViewM mv;
        if (!masterMetricAt(i, mv)) continue;
        JsonObject m = ma.add<JsonObject>();
        m["id"]  = mv.id;
        m["v"]   = mv.value;
        m["age"] = mv.ageMs;
        m["src"] = mv.source;
    }
}

/** The census in identifier order; @p edges: each bit's flip counter as well. */
static void __attribute__((noinline)) addCensusRows(JsonArray ca, bool edges) {
    uint8_t order[128];
    const size_t nc = masterCensusOrder(order, sizeof order);
    char hx[17], chx[17], ehx[129];
    uint8_t ed[64];
    for (size_t i = 0; i < nc; i++) {
        CensusView cen;
        if (!masterCensusRow(order[i], cen, edges ? ed : nullptr)) continue;   // reset meanwhile
        const uint8_t dlc = cen.dlc > 8 ? 8 : cen.dlc;
        JsonObject o = ca.add<JsonObject>();
        o["id"]    = cen.id;
        o["count"] = cen.count;
        o["ext"]   = cen.extd;
        o["age"]   = cen.ageMs;
        for (uint8_t k = 0; k < dlc; k++) {
            snprintf(hx + 2 * k, 3, "%02X", cen.data[k]);
            snprintf(chx + 2 * k, 3, "%02X", cen.changed[k]);
        }
        hx[2 * dlc] = chx[2 * dlc] = '\0';
        o["d"] = hx;
        o["c"] = chx;
        if (edges) {
            for (uint8_t k = 0; k < 64; k++) snprintf(ehx + 2 * k, 3, "%02X", ed[k]);
            o["e"] = ehx;
        }
    }
}

void WebPortal::setupRoutes() {
    static const char *HEADERS[] = {"Content-Type"};   // which body an upload is (upChunk)
    s_server.collectHeaders(HEADERS, 1);

    s_server.on("/", HTTP_GET, []() {
        s_server.send_P(200, "text/html", PORTAL_HTML);
    });

    s_server.on("/api/config", HTTP_GET, []() {
        JsonDocument doc;
        Cfg.toJson(doc);
        sendJson(doc);
    });

    /*
     * Settings in (the page's Save, and Upload): the body goes to the flash as
     * it arrives and is applied from there (see upChunk).
     */
    s_server.on("/api/config", HTTP_POST, []() {
        char why[120] = "";
        bool ok = false;
        if (s_up == UP_STORED) {
            ok = Cfg.applyFile(MasterConfig::UPLOAD_PATH, why, sizeof why);
        } else {
            snprintf(why, sizeof why, s_up == UP_TOO_BIG ? "too big for a settings file - nothing changed"
                     : s_up == UP_NONE ? "no settings in the request - nothing changed"
                                       : "the master could not store the upload - nothing changed");
            masterBusStall();
            LittleFS.remove(MasterConfig::UPLOAD_PATH);
        }
        s_up = UP_NONE;
        if (ok) { sendOk(); return; }
        log_w("portal: settings not saved: %s", why);
        JsonDocument d;
        d["error"] = why;
        sendJson(d, 400);
    }, []() {
        // A form's file (the page) or a bare body (anything else): the server
        // keeps one or the other, and says which only in the content type.
        if (s_server.header("Content-Type").startsWith("multipart/")) {
            HTTPUpload &u = s_server.upload();
            upChunk(u.status == UPLOAD_FILE_START ? RAW_START : u.status == UPLOAD_FILE_WRITE ? RAW_WRITE
                    : u.status == UPLOAD_FILE_END ? RAW_END : RAW_ABORTED, u.buf, u.currentSize);
        } else {
            HTTPRaw &r = s_server.raw();
            upChunk(r.status, r.buf, r.currentSize);
        }
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
        // The lowest it has been since boot, and the largest single block: a
        // save needs ~30 KB of the car's settings at once.
        doc["heap_min"]   = ESP.getMinFreeHeap();
        doc["heap_block"] = ESP.getMaxAllocHeap();
        // The least stack any CAN-side task has had left: an overflow is a
        // panic, and a panic restarts the master mid-drive.
        doc["stack"]      = st.stackMinFree;
        doc["stack_task"] = st.stackMinTask;
        addStacks(doc["stacks"].to<JsonObject>());
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
        b["lom_tec"] = st.lomTecRead;
        b["lom_rp"]  = st.lomRepins;
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
        b["ids"] = masterCensusCount();

        JsonObject o = doc["obd"].to<JsonObject>();
        o["answers"]   = st.obdAnswers;
        o["active"]    = st.obdActive;
        o["physical"]  = st.obdPhysical;
        o["supported"] = st.obdSupported;
        o["rx"]        = st.obdRx;
        // How long the ECU takes to answer [ms], and answers that came after
        // the wait for them had ended (obd_to / obd_p2can shorter than the ECU).
        o["lat"]       = st.obdLatUs / 1000.0f;
        o["lat_max"]   = st.obdLatMaxUs / 1000.0f;
        o["late"]      = st.obdLate;

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

        addLearnRows(doc["learn"].to<JsonArray>());
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

        addMetricRows(doc["metrics"].to<JsonArray>());
        sendJson(doc);
    });

    /* The Bus tab's frame list, polled only while that tab is open. */
    s_server.on("/api/bus", HTTP_GET, []() {
        JsonDocument doc;
        JsonArray ca = doc["census"].to<JsonArray>();
        // ?e=1 (teach by doing): each bit's flip counter as well, 128 hex digits.
        const bool edges = s_server.arg("e") == "1";
        // ?m=269,4102 (teach a G or yaw field): those values with their age,
        // sampled with the frames: {"269":[value, age ms]}.
        if (s_server.hasArg("m")) {
            JsonObject mo = doc["m"].to<JsonObject>();
            String list = s_server.arg("m");
            int from = 0;
            for (int k = 0; k < 8 && from < (int)list.length(); k++) {
                int comma = list.indexOf(',', from);
                if (comma < 0) comma = list.length();
                const long id = list.substring(from, comma).toInt();
                from = comma + 1;
                float v; uint32_t age; uint8_t src;
                if (id > 0 && id <= 0xFFFF && metricLookup((uint16_t)id, v, age, src)) {
                    JsonArray a = mo[String(id)].to<JsonArray>();
                    a.add(v); a.add(age); a.add(src);
                }
            }
        }
        addCensusRows(ca, edges);
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
    /*
     * Trouble codes and the engine computer's identity (DiagCodes), the oil
     * change reminder, and the drive logs.
     */
    s_server.on("/api/codes", HTTP_GET, []() {
        static CodesStatus cs;
        codesGetStatus(cs);
        Ssm2Status ss;
        ssm2GetStatus(ss);
        JsonDocument doc;
        doc["read"]  = cs.haveRead;
        doc["age"]   = cs.readAgeMs / 1000;
        doc["busy"]  = cs.busy;
        doc["note"]  = cs.note;
        doc["clear"] = cs.clearNote;
        doc["tcm_func"] = cs.tcmFunctional;
        JsonArray la = doc["lists"].to<JsonArray>();
        for (auto &row : cs.list) { JsonArray r = la.add<JsonArray>(); for (uint8_t v : row) r.add(v); }
        JsonArray ca = doc["codes"].to<JsonArray>();
        for (uint8_t i = 0; i < cs.n; i++) {
            JsonObject o = ca.add<JsonObject>();
            char c[6];
            codesName(cs.codes[i].raw, c);
            o["code"] = c;
            o["text"] = codesText(cs.codes[i].raw);
            o["kind"] = cs.codes[i].kind;
            o["mod"]  = cs.codes[i].module;
        }
        JsonObject id = doc["ident"].to<JsonObject>();
        id["vin"]    = cs.vin[0] ? cs.vin : Cfg.vin;
        id["cal"]    = cs.calId[0][0] ? cs.calId[0] : Cfg.ecuCalId;
        id["cal2"]   = cs.calId[1];
        id["cvn"]    = cs.cvn[0];
        id["rom"]    = ss.initOk ? ss.ecuId : Cfg.ecuId;
        id["sys"]    = ss.initOk ? ss.sysId : Cfg.ecuSysId;
        id["live"]   = cs.vin[0] || cs.calId[0][0];
        JsonObject sv = doc["service"].to<JsonObject>();
        float odo;
        if (masterOdometer(odo)) sv["odo"] = lroundf(odo);
        sv["every"] = Cfg.oilEveryKm;
        sv["at"]    = Cfg.oilAtKm;
        sendJson(doc);
    });
    s_server.on("/api/codes/read", HTTP_POST, []() { codesRequestRead(); sendOk(); });
    s_server.on("/api/codes/clear", HTTP_POST, []() {
        char why[64];
        if (codesRequestClear(why, sizeof why)) { sendOk(); return; }
        JsonDocument d;
        d["error"] = why;
        String out;
        serializeJson(d, out);
        s_server.send(409, "application/json", out);
    });
    s_server.on("/api/service/reset", HTTP_POST, []() {
        char why[80];
        if (masterServiceReset(why, sizeof why)) { sendOk(); return; }
        JsonDocument d;
        d["error"] = why;
        String out;
        serializeJson(d, out);
        s_server.send(409, "application/json", out);
    });

    s_server.on("/api/logs", HTTP_GET, []() {
        DriveLogStatus st;
        driveLogStatus(st);
        // The list is wanted only while this page is built: from the heap for
        // that long, not 2 KB set aside for good.
        static constexpr size_t MAX_LIST = 64;
        LogFileInfo *li = (LogFileInfo *)malloc(MAX_LIST * sizeof(LogFileInfo));
        const size_t n = li ? driveLogList(li, MAX_LIST) : 0;
        JsonDocument doc;
        doc["on"]      = st.on;
        doc["rec"]     = st.recording;
        doc["file"]    = st.file;
        doc["rows"]    = st.rows;
        doc["used_kb"] = st.usedKb;
        doc["cap_kb"]  = st.capKb;
        doc["free_kb"] = st.freeKb;
        doc["own"]     = st.own;
        doc["part_kb"] = st.partKb;
        doc["note"]    = st.note;
        doc["raw"]     = rawRecOwnsPartition();
        JsonArray fa = doc["files"].to<JsonArray>();
        for (size_t i = 0; i < n; i++) {
            JsonObject o = fa.add<JsonObject>();
            o["name"] = li[i].name;
            o["bytes"] = li[i].bytes;
            o["rows"] = li[i].rows;
            o["secs"] = li[i].seconds;
            o["hz"]   = li[i].hz;
            o["cols"] = li[i].cols;
            o["rec"]  = li[i].recording;
        }
        sendJson(doc);
        free(li);
    });
    // The file as recorded; the page turns it into CSV with the metrics' names.
    s_server.on("/api/log", HTTP_GET, []() {
        char path[32];
        driveLogFlush();                       // the rows still in RAM, too
        if (!driveLogPath(s_server.arg("f").c_str(), path, sizeof path)) {
            s_server.send(404, "application/json", "{\"error\":\"no such log\"}");
            return;
        }
        File f = driveLogOpen(path);         // on the logs' own partition
        if (!f) { s_server.send(404, "application/json", "{\"error\":\"no such log\"}"); return; }
        s_server.streamFile(f, "application/octet-stream");
        f.close();
    });
    s_server.on("/api/log/delete", HTTP_POST, []() {
        if (driveLogDelete(s_server.arg("f").c_str())) sendOk();
        else s_server.send(404, "application/json", "{\"error\":\"no such log\"}");
    });
    s_server.on("/api/log/clear", HTTP_POST, []() { driveLogDeleteAll(); sendOk(); });

    /*
     * The car's other computers (BodyUnit): the scan of 0x700-0x7F7 and the
     * body computer's blocks as the reader last had them.
     */
    s_server.on("/api/body", HTTP_GET, []() {
        BodyStatus st;                         // ~230 B of the loop task's 12 KB, not static RAM
        bodyGetStatus(st);
        JsonDocument d;
        d["scanning"] = st.scanning;
        d["at"]       = st.at;
        d["pct"]      = st.pct;
        d["scanned"]  = st.scanned;
        d["note"]     = st.scanNote;
        JsonArray fa = d["found"].to<JsonArray>();
        for (size_t i = 0; i < st.found; i++) {
            BodyFound f;
            if (!bodyFoundAt(i, f)) break;
            JsonObject o = fa.add<JsonObject>();
            o["tx"]  = f.tx;
            o["rx"]  = f.rx;
            o["biu"] = f.biu;
            o["len"] = f.biuLen;
            JsonArray r = o["reply"].to<JsonArray>();
            for (uint8_t k = 0; k < f.replyLen; k++) r.add(f.reply[k]);
        }
        JsonArray oa = d["others"].to<JsonArray>();
        for (uint8_t i = 0; i < st.otherN; i++) oa.add(st.others[i]);
        JsonObject rd = d["reader"].to<JsonObject>();
        rd["tx"]    = st.tx;
        rd["rx"]    = st.rx;
        rd["reads"] = st.reads;
        rd["fails"] = st.fails;
        if (st.ageMs != UINT32_MAX) rd["age"] = st.ageMs;
        rd["note"]  = st.note;
        JsonArray ba = rd["blocks"].to<JsonArray>();
        for (int b = 0; b < 2; b++) {
            JsonArray r = ba.add<JsonArray>();
            for (uint8_t k = 0; k < st.blockLen[b]; k++) r.add(st.block[b][k]);
        }
        sendJson(d);
    });
    s_server.on("/api/body/scan", HTTP_POST, []() {
        char why[96] = "";
        if (bodyRequestScan(why, sizeof why)) { sendOk(); return; }
        JsonDocument d;
        d["error"] = why;
        sendJson(d, 409);
    });

    /*
     * The raw bus recording (RawRec): every frame to the logs partition, for
     * analysis off the car (master/tools/canrec.py makes a candump log of the
     * download). Arming erases the partition, drive logs and all.
     */
    s_server.on("/api/rawrec", HTTP_GET, []() {
        static const char *const STATE[] = {"none", "off", "erasing", "recording", "full", "stopped"};
        RawRecStatus st;
        rawRecStatus(st);
        JsonDocument d;
        d["state"]   = STATE[st.state < 6 ? st.state : 0];
        d["used"]    = st.used;
        d["cap"]     = st.cap;
        d["frames"]  = st.frames;
        d["tx"]      = st.txFrames;
        d["lost"]    = st.lost;
        d["session"] = st.session;
        d["rate"]    = st.rate;
        d["erase"]   = st.erasePct;
        d["size"]    = rawRecSize();
        d["note"]    = st.note;
        sendJson(d);
    });
    s_server.on("/api/rawrec/arm", HTTP_POST, []() {
        char why[120] = "";
        if (rawRecArm(why, sizeof why)) { sendOk(); return; }
        JsonDocument d;
        d["error"] = why;
        sendJson(d, 409);
    });
    s_server.on("/api/rawrec/stop", HTTP_POST, []() { rawRecStop(); sendOk(); });
    s_server.on("/api/rawrec/release", HTTP_POST, []() {
        char why[120] = "";
        if (rawRecRelease(why, sizeof why)) { sendOk(); return; }
        JsonDocument d;
        d["error"] = why;
        sendJson(d, 409);
    });
    // The header sector and every record so far, read 2 KB at a time (from
    // the heap for the length of the download).
    s_server.on("/api/rawrec/file", HTTP_GET, []() {
        rawRecFlush();
        const uint32_t size = rawRecSize();
        uint8_t *buf = size ? (uint8_t *)malloc(2048) : nullptr;
        if (!buf) {
            s_server.send(size ? 503 : 404, "application/json",
                          size ? "{\"error\":\"no memory for the download\"}" : "{\"error\":\"no recording\"}");
            return;
        }
        s_server.sendHeader("Content-Disposition", "attachment; filename=\"canrec.bin\"");
        s_server.setContentLength(size);
        s_server.send(200, "application/octet-stream", "");
        WiFiClient client = s_server.client();
        for (uint32_t at = 0; at < size && client.connected();) {
            const size_t n = size - at < 2048 ? size - at : 2048;
            if (!rawRecRead(at, buf, n) || client.write(buf, n) != n) break;
            at += n;
        }
        free(buf);
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

    /*
     * Firmware update (FwUpdate): the upload goes into the spare app slot as
     * it arrives, is checked, and the master restarts into it - and keeps it
     * only once it has run properly. Refused while moving.
     */
    s_server.on("/api/update", HTTP_GET, []() {
        FwStatus fs;
        fwStatus(fs);
        JsonDocument d;
        d["version"]     = fs.version;
        d["build"]       = fs.build;
        d["slot"]        = fs.slot;
        d["trial"]       = fs.trial;
        d["rolled_back"] = fs.rolledBack;
        d["busy"]        = fs.busy;
        d["received"]    = fs.received;
        d["mark"]        = fs.mark;
        char why[48] = "";
        d["allowed"]     = fwUpdateAllowed(why, sizeof why);
        d["why"]         = why;
        sendJson(d);
    });
    static bool s_updOk = false;
    static char s_updWhy[96] = "";
    s_server.on("/api/update", HTTP_POST, []() {
        if (!s_updOk) {
            JsonDocument d;
            d["error"] = s_updWhy[0] ? s_updWhy : "no firmware received";
            String out;
            serializeJson(d, out);
            s_server.send(409, "application/json", out);
            return;
        }
        masterBeforeRestart();
        sendOk();
        delay(300);
        ESP.restart();
    }, []() {
        // Called as the upload arrives: the whole of it, before the answer above.
        HTTPUpload &up = s_server.upload();
        switch (up.status) {
            case UPLOAD_FILE_START:
                s_updWhy[0] = '\0';
                s_updOk = fwUpdateBegin(s_updWhy, sizeof s_updWhy);
                break;
            case UPLOAD_FILE_WRITE:
                if (s_updOk) s_updOk = fwUpdateWrite(up.buf, up.currentSize, s_updWhy, sizeof s_updWhy);
                break;
            case UPLOAD_FILE_END:
                if (s_updOk) s_updOk = fwUpdateEnd(s_updWhy, sizeof s_updWhy);
                break;
            default:                                  // UPLOAD_FILE_ABORTED: the connection went
                fwUpdateAbort();
                s_updOk = false;
                snprintf(s_updWhy, sizeof s_updWhy, "the upload was cut off");
                break;
        }
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
