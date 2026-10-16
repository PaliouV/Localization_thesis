"""
Βρόχοι οδήγησης γύρω από τα οικοδομικά τετράγωνα του Whampoa, από το δίκτυο
δρόμων του OpenStreetMap (whampoa.osm). Κάθε βρόχος ακολουθεί ακριβώς τη γεωμετρία
των δρόμων (όχι ευθείες ανάμεσα σε διασταυρώσεις).

Πώς: οι δρόμοι γίνονται γράφος (κόμβοι = σημεία του OSM, ακμές = κομμάτια δρόμου).
Σε έναν επίπεδο γράφο, κάθε «όψη» (face) είναι ένα τετράγωνο που περικλείουν δρόμοι.
Ακολουθώντας σε κάθε κόμβο την πιο δεξιά στροφή, κάνουμε τον γύρο κάθε όψης.
Κρατάμε όψεις με περίμετρο 350–1500 m και εμβαδόν > 5000 m², κοντά στο κέντρο (Whampoa Garden).

    python matlab/whampoa_loops.py   → matlab/whampoa_loops.csv (loop, lat, lon)
"""
import csv, math, os, xml.etree.ElementTree as ET
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
CENTER = (22.3047, 114.1902)
MAX_DIST_FROM_CENTER = 250      # m — κέντρο του βρόχου κοντά στο Whampoa Garden
PERIMETER = (350, 1500)         # m
MIN_AREA = 5000                 # m² — όχι στενές λωρίδες ανάμεσα σε δύο παράλληλους δρόμους
DRIVABLE = {"primary", "secondary", "tertiary", "residential", "unclassified", "trunk",
            "primary_link", "secondary_link", "tertiary_link", "living_street"}
R = 6378137.0

def xy(lat, lon):
    return (math.radians(lon - CENTER[1]) * R * math.cos(math.radians(CENTER[0])),
            math.radians(lat - CENTER[0]) * R)

root = ET.parse(os.path.join(HERE, "whampoa.osm")).getroot()
nodes = {n.get("id"): (float(n.get("lat")), float(n.get("lon"))) for n in root.iter("node")}
P = {k: xy(*v) for k, v in nodes.items()}

adj = defaultdict(set)
for w in root.iter("way"):
    t = {x.get("k"): x.get("v") for x in w.findall("tag")}
    if t.get("highway") in DRIVABLE and t.get("area") != "yes":
        refs = [n.get("ref") for n in w.findall("nd") if n.get("ref") in nodes]
        for a, b in zip(refs, refs[1:]):
            if a != b:
                adj[a].add(b); adj[b].add(a)

def angle(a, b):
    return math.atan2(P[b][1] - P[a][1], P[b][0] - P[a][0])

# για κάθε κατευθυνόμενη ακμή (a→b), η επόμενη: στο b, η πρώτη ακμή δεξιόστροφα από την (b→a)
order = {v: sorted(adj[v], key=lambda u: angle(v, u)) for v in adj}
def next_edge(a, b):
    nb = order[b]
    i = nb.index(a)
    return b, nb[(i - 1) % len(nb)]

seen, faces = set(), []
for a in adj:
    for b in adj[a]:
        if (a, b) in seen: continue
        face, e = [], (a, b)
        while e not in seen and len(face) < 5000:
            seen.add(e); face.append(e[0]); e = next_edge(*e)
        faces.append(face)

def area_perim(face):
    pts = [P[v] for v in face]
    A = sum(x1*y2 - x2*y1 for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1])) / 2
    L = sum(math.dist(p, q) for p, q in zip(pts, pts[1:] + pts[:1]))
    cx = sum(p[0] for p in pts) / len(pts); cy = sum(p[1] for p in pts) / len(pts)
    return A, L, math.hypot(cx, cy)

loops = []
for f in faces:
    A, L, d = area_perim(f)
    if A > MIN_AREA and PERIMETER[0] <= L <= PERIMETER[1] and d <= MAX_DIST_FROM_CENTER:   # εσωτερικές όψεις (θετικό εμβαδόν)
        loops.append((L, f))
loops.sort(key=lambda x: area_perim(x[1])[2])
with open(os.path.join(HERE, "whampoa_loops.csv"), "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["loop", "lat", "lon"])
    for i, (L, f) in enumerate(loops):
        name = "ABCDEFGH"[i]
        for v in f[::-1]:                                  # οδήγηση με τη φορά των δεικτών του ρολογιού αντίστροφα
            w.writerow([name, *nodes[v]])
        print(f"βρόχος {name}: περίμετρος {L:.0f} m, {len(f)} σημεία, κέντρο {area_perim(f)[2]:.0f} m από το κέντρο")
