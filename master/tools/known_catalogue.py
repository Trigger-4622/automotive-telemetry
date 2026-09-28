"""Usage: python master/tools/known_catalogue.py

Regenerate the Gauge Studio's KNOWN metric catalogue in both screens so
every ID the master can publish is selectable by name. Fails if MasterPacket.h
defines an ID with no name here."""
import io, os, re, sys

# The repository root: this file is master/tools/known_catalogue.py.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
NAMES = {
0x1008:'Wheel FL km/h',0x1009:'Wheel FR km/h',0x100A:'Wheel RL km/h',0x100B:'Wheel RR km/h',
0x100C:'Lateral G g',0x100D:'Longitudinal G g',0x100E:'Yaw rate °/s',
0x100F:'Gear lever',0x1010:'Cruise set km/h',0x1011:'ATF temp °C',
0x1201:'Left turn signal',0x1202:'Right turn signal',0x1203:'High beam',0x1204:'Door open',
0x1205:'Handbrake',0x1206:'Seatbelt unfastened',0x1207:'Cruise active',0x1208:'Reverse',
0x1209:'Lever in P',0x120A:'Lever in N',0x120B:'Lever in D',0x120C:'Hazard lights',
0x120D:'Cruise main',0x120E:'Front fog',0x120F:'Rear fog',0x1210:'Door FL',0x1211:'Door FR',
0x1212:'Door RL',0x1213:'Door RR',0x1214:'Trunk open',0x1215:'Hood open',0x1216:'VDC off',
0x1217:'Passenger belt unfastened',
0x3001:'Custom 1',0x3002:'Custom 2',0x3003:'Custom 3',0x3004:'Custom 4',0x3005:'Custom 5',0x3006:'Custom 6',0x3007:'Custom 7',0x3008:'Custom 8',0x3009:'Custom 9',0x300A:'Custom 10',0x300B:'Custom 11',0x300C:'Custom 12',0x300D:'Custom 13',0x300E:'Custom 14',0x300F:'Custom 15',0x3010:'Custom 16',
0x0101:'MIL lamp',0x0301:'DTC count',0x0103:'Fuel system status',
0x0104:'Engine load %',0x0105:'Coolant °C',0x0106:'STFT B1 %',0x0107:'LTFT B1 %',
0x0108:'STFT B2 %',0x0109:'LTFT B2 %',0x010A:'Fuel press kPa',0x010B:'MAP kPa',
0x010C:'RPM',0x010D:'Speed km/h',0x010E:'Timing °',0x010F:'IAT °C',0x0110:'MAF g/s',
0x0111:'Throttle %',0x0114:'O2 B1S1 V',0x0115:'O2 B1S2 V',0x0118:'O2 B2S1 V',0x0119:'O2 B2S2 V',
0x0314:'O2 B1S1 trim %',0x0315:'O2 B1S2 trim %',0x0318:'O2 B2S1 trim %',0x0319:'O2 B2S2 trim %',
0x011F:'Run time s',0x0121:'Dist MIL km',0x0122:'Fuel rail (vac) kPa',0x0123:'Fuel rail gauge kPa',
0x0124:'WB B1S1 λ',0x0324:'WB B1S1 V',0x0128:'WB B2S1 λ',0x0328:'WB B2S1 V',
0x012C:'EGR cmd %',0x012D:'EGR error %',0x012E:'Evap purge %',0x012F:'Fuel level %',0x0130:'Warm-ups',
0x0131:'Dist cleared km',0x0132:'Evap vapor Pa',0x0133:'Barometric kPa',
0x0134:'WB B1S1 λ (I)',0x0334:'WB B1S1 mA',0x0138:'WB B2S1 λ (I)',0x0338:'WB B2S1 mA',
0x013C:'Cat temp °C',0x013D:'Cat temp B2S1 °C',0x013E:'Cat temp B1S2 °C',0x013F:'Cat temp B2S2 °C',
0x0142:'Battery V',0x0143:'Abs load %',0x0144:'Cmd λ',0x0145:'Rel throttle %',
0x0146:'Ambient °C',0x0147:'Abs throttle B %',0x0148:'Throttle C %',0x0149:'Pedal D %',
0x014A:'Pedal E %',0x014B:'Pedal F %',0x014C:'Cmd throttle %',0x014D:'Time MIL on min',
0x014E:'Time since clear min',0x0151:'Fuel type',0x0152:'Ethanol %',0x0153:'Evap abs kPa',
0x0159:'Fuel rail abs kPa',0x015A:'Relative pedal %',0x015B:'Hybrid SOC %',0x015C:'Oil temp °C',
0x015D:'Injection timing °',0x015E:'Fuel rate L/h',0x0161:'Demand torque %',
0x0162:'Actual torque %',0x0163:'Reference torque Nm',0x01A6:'Odometer km',
0x1001:'Boost bar',0x1002:'AFR',0x1003:'Oil press bar',0x1004:'EGT °C',0x1005:'Gear',
0x1006:'Steering °',0x1007:'Brake bar',
0x1101:'Batt current A',0x1102:'Batt SOC %',0x1103:'Batt SOH %',0x1104:'Batt temp °C',
0x1105:'Alternator V',0x1106:'Alternator A',0x1107:'Alternator load %',
0x1108:'12V rail V',0x1109:'Elec load W',0x110A:'Charge status',
0x2001:'Knock corr °',0x2002:'Knock corr fine °',0x2003:'A/F learn %',0x2004:'A/F corr %',
0x2005:'Inj duty %',0x2006:'Target boost bar',0x2007:'Wastegate %',0x2008:'A/F corr 2 %',
0x2009:'A/F learn 2 %',0x200A:'Front O2 #1 V',0x200B:'Rear O2 V',0x200C:'Front O2 #2 V',
0x200D:'MAF sensor V',0x200E:'TPS V',0x200F:'Atmospheric kPa',0x2010:'Manifold rel kPa',
0x2011:'Tank press kPa',0x2012:'Learned timing °',0x2013:'Accel pedal %',
0x2014:'Fuel temp °C',0x2015:'Wastegate 2 %',0x2016:'CPC duty %',0x2017:'ISC duty %',
0x2018:'A/F lean %',0x2019:'A/F heater %',0x201A:'ISC step',0x201B:'EGR steps',
0x201C:'Alternator duty %',0x201D:'Fuel pump %',0x201E:'Intake VVT R °',
0x201F:'Intake VVT L °',0x2020:'Intake OCV R %',0x2021:'Intake OCV L %',
0x2022:'A/F current mA',0x2023:'Lambda',0x2024:'Lambda 2',0x2025:'Throttle motor %',
0x2026:'Main TPS V',0x2027:'Main APS V',0x2028:'Brake boost kPa',0x2029:'Fuel HP MPa',
0x202B:'Exhaust VVT R °',0x202C:'Exhaust VVT L °',0x202D:'Rough cyl 1',0x202E:'Rough cyl 2',
0x202F:'Rough cyl 3',0x2030:'Rough cyl 4',0x2031:'Inj 2 pulse ms',0x2032:'Cold start inj ms',
0x2033:'Alternator mode',0x2034:'Fuel sender V',0x2035:'Radiator fan %',
0x2036:'Learned ign corr °',0x2037:'Boost feedback %',0x2038:'Target RPM',0x2039:'SI-Drive',
0x203A:'Odometer (SSM) km',0x203B:'EPS current A',0x203C:'Fuel pump A',0x203E:'Inj 1 pulse ms',
0x2101:'Brake switch',0x2102:'Clutch switch',0x2103:'Neutral switch',0x2104:'A/C switch',0x2105:'Idle switch',
0x2106:'Knock signal',0x2107:'Electrical load',0x2108:'Light switch',0x2109:'Radiator fan 1',
0x210A:'Radiator fan 2',0x210B:'A/C compressor',0x210C:'Fuel pump relay',0x210D:'Oil pressure switch',
0x210E:'Starter',0x210F:'Rear defogger',0x2110:'Blower',0x2111:'Wiper switch',
0x2112:'Ignition',0x2113:'Stop lamp switch',0x2114:'Knock signal 2',
0x1F01:'Night sense',0x1F02:'Master uptime s',0x1F03:'CAN frames/s',0x1F04:'CAN IDs',
0x1F05:'OBD replies/s',0x1F06:'SSM2 exch/s',
}
hdr = io.open(os.path.join(ROOT, "master", "include", "MasterPacket.h"),
              encoding="utf-8").read()
ids = sorted(set(int(m, 16) for m in re.findall(r"#define\s+METRIC_ID_\w+\s+(0x[0-9A-Fa-f]{4})", hdr)))
missing = [hex(i) for i in ids if i not in NAMES]
if missing:
    sys.exit("no name for " + ", ".join(missing))

lines, cur = [], " "
for i in ids:
    piece = "'0x%04X':'%s'," % (i, NAMES[i].replace("'", "\\'"))
    if len(cur) + len(piece) > 78:
        lines.append(cur.rstrip()); cur = " "
    cur += piece
lines.append(cur.rstrip().rstrip(","))
block = "const KNOWN = {\n" + "\n".join(lines) + "\n};"

for rel in ("screens/screen1-round/data/www/index.html",
            "screens/screen2-cluster/data/www/index.html"):
    p = os.path.join(ROOT, rel)
    s = io.open(p, encoding="utf-8").read()
    a = s.index("const KNOWN = {"); b = s.index("};", a) + 2
    io.open(p, "w", encoding="utf-8", newline="\n").write(s[:a] + block + s[b:])
    print(rel, "->", len(ids), "metrics")
