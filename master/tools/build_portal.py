"""Usage: python tools/build_portal.py   (run after editing tools/portal_page.html)

Fold the portal page into WebPortal.cpp, generating the metric catalogue
(name, unit, group, decimals) from MasterPacket.h plus curated short names."""
import io, os, re, sys, json

SP = os.path.dirname(os.path.abspath(__file__))
MASTER = os.path.dirname(SP)                  # master/
sys.path.insert(0, SP)
src = io.open(os.path.join(SP, "known_catalogue.py"), encoding="utf-8").read()
NAMES = eval(src[src.index("NAMES = {") + 8: src.index("\n}\n") + 2])

hdr = io.open(os.path.join(MASTER, "include", "MasterPacket.h"), encoding="utf-8").read()
units = {}
for m in re.finditer(r"#define\s+METRIC_ID_\w+\s+(0x[0-9A-Fa-f]{4})\s*/\*\*<(.*?)\*/", hdr, re.S):
    u = re.search(r"\[([^\]]*)\]\s*$", " ".join(m.group(2).split()))
    units[int(m.group(1), 16)] = u.group(1).strip() if u else ""

UNITS = {"g", "°/s", "°C", "%", "kPa", "rpm", "km/h", "°", "g/s", "V", "A", "W", "bar", "ms", "mA",
         "L/h", "s", "km", "min", "Pa", "Nm", "MPa", "λ", "/s"}
DEC = {"g": 2, "°/s": 1, "rpm": 0, "km/h": 0, "°C": 0, "%": 1, "kPa": 0, "V": 2, "λ": 3, "bar": 2, "g/s": 1,
       "°": 1, "ms": 2, "mA": 1, "A": 1, "L/h": 1, "s": 0, "km": 0, "min": 0, "Pa": 0,
       "Nm": 0, "MPa": 2, "W": 0, "/s": 0, "": 0, "AFR": 1}

def group(i, name, unit):
    if i & 0xFF00 == 0x1F00: return "System"
    if i & 0xFF00 == 0x3000: return "Custom"
    if i & 0xFF00 == 0x1200: return "Status & switches"
    if 0x1008 <= i <= 0x100E: return "Driving"
    if i in (0x0101, 0x0301, 0x0103, 0x0151, 0x110A, 0x2033, 0x2039) or i & 0xFF00 == 0x2100:
        return "Status & switches"
    if unit == "°C": return "Temperatures"
    if i & 0xFF00 == 0x1100 or re.search(r"Batt|Alternator|12V|EPS|pump A", name): return "Electrical"
    if re.search(r"Speed|Gear|Steering|Brake|Odometer|Dist|Run time|Time |Warm|SI-Drive", name):
        return "Driving"
    if re.search(r"O2|λ|Lambda|A/F|AFR|STFT|LTFT|trim|Inj|Fuel|Evap|EGR|CPC|Ethanol|WB ", name):
        return "Fuel & mixture"
    return "Engine"

meta = {}
for i, full in NAMES.items():
    name, unit = full, ""
    parts = full.rsplit(" ", 1)
    if len(parts) == 2 and parts[1] in UNITS:
        name, unit = parts
    elif units.get(i, "") in UNITS:
        unit = units[i]
    if i == 0x010C: unit = "rpm"
    if i == 0x1002: unit = "AFR"
    dec = DEC.get(unit, 2)
    if i in (0x1005, 0x0301, 0x0101, 0x0103) or i & 0xFF00 in (0x2100, 0x1200): dec = 0
    meta[i] = [name, unit, group(i, name, unit), dec]

js = "{" + ",".join(f"{i}:{json.dumps(v, ensure_ascii=False)}" for i, v in sorted(meta.items())) + "}"
page = io.open(os.path.join(SP, "portal_page.html"), encoding="utf-8").read().replace("/*META*/", js)
assert ")HTML\"" not in page
cpp = io.open(os.path.join(SP, "WebPortal.cpp.in"), encoding="utf-8").read().replace("@@PAGE@@", page)
out = os.path.join(MASTER, "src", "WebPortal.cpp")
io.open(out, "w", encoding="utf-8", newline="\n").write(cpp)
import tempfile
TMP = tempfile.gettempdir()
io.open(os.path.join(TMP, "portal_built.html"), "w", encoding="utf-8").write(page)
io.open(os.path.join(TMP, "portal_built.js"), "w", encoding="utf-8").write(
    page[page.index("<script>") + 8: page.index("</script>")])
print(f"WebPortal.cpp written: page {len(page)} bytes, {len(meta)} metrics")
