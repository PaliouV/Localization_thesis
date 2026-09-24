# Thesis — Project State

**Topic:** Map-based fine-grained localization of an autonomous vehicle in CARLA simulation, using a Particle Filter with camera + IMU + noisy GPS, focused on GPS-denied scenarios (urban canyons, tunnels).

**Last updated:** 24 August 2026 (session 5 — Windows home machine set up; conceptual walkthrough of the camera observation model; full literature research on urban-canyon GPS error → `GPS_RESEARCH_REPORT.md` + filled `GPS_NOISE_MODEL.md`. See Section 17.)
**Status:** filterpy + deps installed in venv. Labbe Chapter 12 partially read (still stopped before predict/update/resample). `spawn_vehicle.py` (autopilot script) reviewed — works but has 2 known unfixed issues (see Section 12). `vehicle_spawn.py` (manual-drive reference script) updated: calmer throttle on W, fixed spawn point, and a second noisy-GNSS sensor added alongside the clean one (HUD + CSV both show clean vs noisy side by side).

---

## 1. Who the user is (for tailoring help)

- **Name:** Vaios (Βάιος) Paliouras — `iis23188@uom.edu.gr`
- **School:** Applied Informatics (Εφαρμοσμένη Πληροφορική), University of Macedonia (UoM)
- **Year:** Finishing in a few months — undergraduate thesis is the last requirement
- **Career goal:** Wants to work on **autonomous vehicles** (industry)
- **Background:** Small university projects, *some* Python. Limited terminal/Linux comfort.
- **Math comfort:** **Self-described as weak.** Critical because the thesis is heavy in probability + linear algebra. He needs the math worked through, not assumed.
- **Language:** Communicates in Greek. Replies should be in Greek. Code/identifiers/commits in English.

---

## 2. Working style (read this carefully — repeat mistakes from previous sessions waste his time)

- **Mentor, not code-writer.** He explicitly said he wants to *learn how to write code* and not depend on AI chats. Explain, give exercises, review what he writes.
- **Plan-first.** Always present a short plan and wait for OK before touching files beyond a one-liner. He gets visibly annoyed otherwise.
- **Brevity first.** Lead with a direct answer in 1-3 sentences. Expand only on request. He has interrupted multi-section dumps before.
- **Explain simply.** When he says "δε κατάλαβα" (didn't understand), strip ALL jargon and use everyday-life analogies. Example that worked: rooms with tiles/wood → prior × likelihood → posterior.
- **Honesty over confidence.** Admit "I don't know" when uncertain. Don't fabricate technical details.
- **Greek language.** All explanations in Greek. Don't switch to English unless quoting code/papers/commands.
- **Don't push him into formality.** He wants direct, factual, not persuasive.
- **Don't push code before understanding.** If he says "I don't get it" while writing code, STOP and go back to whiteboard/paper. Session 2 mistake: pushed him into NumPy PF code before the intuition was solid — he pushed back and I had to backtrack to hand-computed 4-room tables. Learn from this.

---

## 3. The thesis topic (as the supervisor framed it)

From the supervisor's message (paraphrased):

> Υλοποίηση αλγορίθμου localization αυτόνομου οχήματος σε ρεαλιστικές συνθήκες, έχοντας ως γνωστό χάρτη το Google Maps ή το OpenStreetMap. Χρήση καμερών για τον εντοπισμό features που κάνουν identify στον γνωστό χάρτη (π.χ. εντοπισμός κτηρίων, data association με την κάτοψη, ενημέρωση πόζας), και των γραμμών των λωρίδων. Σκοπός: fine-grained localization που βελτιώνει το GPS (3-10 m accuracy), ώστε να ξέρουμε σε ποια λωρίδα είμαστε και την ακριβέστερη θέση μας στη λωρίδα. Επίσης αποφυγή urban canyons. Η τεχνολογία μπορεί να ακολουθήσει τη διπλωματική της Δέσποινας (Particle Filter), με major προσαρμογές.

**Supervisor:** Name not yet recorded — ask the user. Described as "πολύ καλό και βοηθητικό".

**Timeline:** ~3 months from start.

**Setting:** Simulation (CARLA), not real hardware.

---

## 4. CRITICAL DECISION #1: OSM → CARLA-native map

The user realised (29 Jun) that OSM as known map does NOT work in CARLA (scenes are fictional, no real-world map correspondence). **Pivot:** use CARLA's own scene metadata:

- `world.get_map()` → OpenDRIVE format with all lanes, junctions, road geometry
- `world.get_environment_objects()` → bounding boxes of all buildings
- Ground truth pose available for evaluation only (not at runtime)

**Status:** User already extracted `Town10HD_Opt.xodr` from CARLA. Files present in repo. Needs to be presented/confirmed with supervisor.

---

## 5. CRITICAL DECISION #2: Use existing PF library (filterpy), not custom

**The supervisor explicitly said: don't write your own PF, use an existing library.** (User confirmed this in session 2 — he had asked earlier.)

**Implication:** the thesis contribution is **NOT** the PF core (~100 lines) but everything around it:

1. **Observation model** for CARLA (lane detection + matching with OpenDRIVE)
2. **Motion model** (bicycle kinematic — can be custom or off-the-shelf)
3. **Evaluation** (ATE, RPE, lane accuracy, urban canyon scenarios)
4. **Integration** with ROS 2 sensor streams

**Library:** `filterpy` — de facto standard for Bayesian filters in Python. Roger Labbe's book https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python (Chapter 12) is the definitive reference.

---

## 6. Despoina's thesis (the reference work)

**Title:** *Development of a Methodology for Path Construction on a Directed Graph with Time Constraints and Localization Using a Particle Filter for Autonomous Vehicles*
**Author:** Despoina Christodoulou (same school as user), Feb 2026
**File:** PDF extract in scratchpad as `thesis.txt` (previous session's context)
**Context:** Real 1:10 scale vehicle for **Bosch Future Mobility Challenge** (BFMC). Team: **Vroom AuTh**.

### What she built (Particle Filter localization)

- State `(x, y, θ)`, bicycle motion model, camera-based observations (signs, lanes, markings)
- KD-tree data association, systematic resampling, ESS < threshold trigger
- Heading via `atan2(Σ wᵢ sin θᵢ, Σ wᵢ cos θᵢ)`
- Kidnapped detection via IMU pitch/yaw thresholds
- Two-metric evaluation: algorithmic (best accuracy) vs operational (real-time)

### Results (35 cm/s, normal route)

- Algorithmic: RMSE 0.246 m, MaxE 0.573 m, runtime 333 ms/iter
- Operational: RMSE 0.509 m, MaxE 1.216 m, runtime 38 ms/iter

### What's reusable

- Theoretical background chapters (Ch. 2.1.3 + 2.2.2 in Greek, citation-ready)
- PF algorithm structure
- Bicycle motion model equations
- Two-metric evaluation framework
- Bibliography — especially **Kuutti et al. [18]** localization survey

### What's different (the "major adaptations")

| Aspect | Despoina | This thesis |
|--------|----------|-------------|
| Map source | Handcrafted BFMC feature map | CARLA OpenDRIVE + scene metadata |
| Scale | 1:10, ~10 m track | Full-scale urban, km-scale |
| GPS | Not used | Used (noisy, dropouts in canyons) |
| Observations | Signs, lanes, markings | Lane lines + buildings (semantic seg) |
| Initialization | Known start | Initial from noisy GPS |
| Speeds | 25-60 cm/s | 10-50 km/h |
| Environment | Indoor controlled | CARLA urban, tall buildings, tunnels |
| **PF implementation** | **Custom** | **filterpy (approved by supervisor)** |

### Code access

**User decided NOT to ask Despoina for code.** Use her thesis text as reference only.

---

## 7. Hardware, OS, and environment (WORKING as of session 2)

**Lab desktop:**
- Dual-boot: Windows + **Ubuntu 22.04 LTS**
- CPU: Intel i5-10600KF
- RAM: 16 GB (tight for CARLA + ROS + browser)
- GPU: **RTX 2060 (6 GB VRAM)**
- **No sudo access** (lab machine)

**Software (all verified working in session 2):**
- **CARLA 0.9.16** at `/opt/carla/` — starts with `./CarlaUE4.sh --ros2 -quality-level=Low`
- **ROS 2 Humble** at `/opt/ros/humble/` — system install (someone with sudo had already done it)
- **Python venv** at `~/carla-env/` — has the `carla` Python module already installed
- **CARLA ROS 2 native integration** — verified: `/carla/hero/{gnss, imu, rgb/image, lidar/point_cloud, ...}` topics all publish correctly after running `python3 /opt/carla/PythonAPI/examples/ros2/ros2_native.py --file stack.json`

**Note on the old "ignore the laptop":** the earlier state said to forget the laptop and use only the lab desktop. That still holds for the *primary* workflow. But as of session 5 the user set up a **secondary home machine (Windows)** for doing CARLA work at home — see below.

### Home machine (Windows) — added session 5 (7 Aug 2026)

Secondary/home setup. **Lab Ubuntu desktop remains the primary environment.** Used for working at home when the lab isn't available.

- **OS:** Windows 11, user `vpaliouras` (has admin / winget available)
- **Hardware:** laptop — **RTX 3050 Ti Laptop GPU (4 GB VRAM)**, 31 GB RAM. 4 GB VRAM is tight for CARLA but runs at low quality; expect low FPS.
- **CARLA 0.9.16** at `C:\Users\vpaliouras\Downloads\CARLA_0.9.16` — the simulator is `CarlaUE4.exe` (no Python needed to launch it).
- **Python gotcha (important):** the machine's default Python is **3.14**, but CARLA's wheel is `cp312` = **Python 3.12 only**. Installed Python 3.12.10 alongside (via `winget install Python.Python.3.12`); both coexist (`py -0p` lists 3.14 default + 3.12).
- **Python venv** at `C:\Users\vpaliouras\carla-env` (3.12-based, mirrors the Ubuntu `~/carla-env`). Contains: `carla` 0.9.16 (from the local wheel at `...\CARLA_0.9.16\PythonAPI\carla\dist\carla-0.9.16-cp312-cp312-win_amd64.whl`), plus `pygame`, `numpy`, `filterpy`, `matplotlib`, `opencv-python`, `scipy`, `scikit-learn`, `shapely`. Verified `import carla` works.
- **No ROS 2 on Windows** — not set up, not needed (all current scripts use the direct `carla` Python API).

**How to run CARLA at home (two PowerShell terminals):**

Terminal 1 — start the simulator (leave running):
```powershell
C:\Users\vpaliouras\Downloads\CARLA_0.9.16\CarlaUE4.exe
```
Terminal 2 — run a client script (simulator must be up first):
```powershell
C:\Users\vpaliouras\carla-env\Scripts\python.exe C:\Users\vpaliouras\Localization_thesis\vehicle_spawn.py
```
Calling the venv's `python.exe` by full path avoids needing to "activate" the venv (which PowerShell execution policy can block). If CARLA is too heavy on the 4 GB GPU, add flags like `-windowed -ResX=800 -ResY=600` (and `-quality-level=Low` if needed for performance).

---

## 8. Installation status + what's still needed

### DONE (verified session 2)

- ROS 2 Humble system-wide
- CARLA 0.9.16 (opt/carla)
- Python venv `~/carla-env` with `carla` module
- Native ROS 2 integration in CARLA

### DONE (session 3)

- **pip installs done** — `filterpy` and `opencv-python` newly installed; `matplotlib`, `scipy`, `scikit-learn`, `shapely` were already present in the venv.
- Known side-effect: installing `filterpy` upgraded `numpy` 1.26.4 → 2.2.6, which conflicts with the unrelated `invertedai` package (wants `numpy<2.0.0`). Verified this does NOT break `filterpy` or `carla` imports. `invertedai` isn't used anywhere in this repo (only appears in an unrelated CARLA example script) — safe to ignore.

### PIP installs (reference — already done, see above)

```bash
source ~/carla-env/bin/activate
pip install filterpy matplotlib opencv-python scipy scikit-learn shapely
```

- **filterpy** — the PF library the supervisor approved
- **matplotlib** — plotting (visualizations for thesis)
- **opencv-python** — image processing / lane detection
- **scipy** — numerical utilities (splines, geometry)
- **scikit-learn** — KD-tree for data association
- **shapely** — geometric operations with OpenDRIVE lanes

### APT installs — needs sudo (give this list to whoever has admin access)

```bash
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-vision-opencv \
  ros-humble-ackermann-msgs \
  ros-humble-derived-object-msgs \
  ros-humble-tf-transformations \
  ros-humble-image-transport \
  python3-opencv
```

Some of these may already be installed as part of ros-humble-desktop — check with:
```bash
apt list --installed 2>/dev/null | grep -E "^ros-humble-(cv-bridge|vision-opencv|ackermann-msgs|derived-object-msgs|tf-transformations)"
```

- **cv-bridge / vision-opencv** — bridge between ROS Image messages and OpenCV (needed for lane detection)
- **ackermann-msgs** — control messages for CARLA vehicle
- **derived-object-msgs** — used by some CARLA components
- **tf-transformations** — utility for coordinate frame math (Python bindings)
- **image-transport** — for compressed image topics
- **python3-opencv** — system-level OpenCV (in addition to pip's opencv-python)

---

## 9. Timeline (3 months, updated for filterpy path)

Since PF core is now off the table (filterpy), the timeline shifts:

| Month | Weeks | Focus |
|-------|-------|-------|
| **1** | 1-2 | ✅ Setup (DONE). ✅ PF theory (DONE conceptually). |
| **1** | 3-4 | filterpy 1D + 2D on synthetic data. Read CARLA OpenDRIVE. Extract lane geometry. |
| **2** | 5-6 | Lane detection module (OpenCV Hough or pretrained). Motion model. GPS simulation from CARLA. |
| **2** | 7-8 | Wire everything to filterpy PF. End-to-end 2D localization pipeline. |
| **3** | 9-10 | Experiments: ATE, RPE, lane accuracy %, urban canyon dropout tests. |
| **3** | 11-12 | Writing thesis, figures, defense prep. |

### Scope decision (revised session 3 — user confirmed: BOTH are required, not staged)

Previously framed as "Option A primary / Option B stretch-if-time-permits." **User corrected this (session 3): the project is meant to deliver the full original plan — both parts are core deliverables, not one optional.**

- **Lane component:** Lane detection + OpenDRIVE matching + PF for lateral correction. Answers "which lane am I in / where exactly in the lane".
- **Building component:** Semantic segmentation of building facades + CARLA building bboxes, matched against `world.get_environment_objects()`. Addresses urban canyon GPS-denial specifically.

**Timeline risk flagged to user (not yet resolved):** doing both robustly is meaningfully more work than lane-only within the same ~3 month window. Worth explicitly confirming with the supervisor whether both need to be equally polished, or whether one can still be the primary focus with the other at a lighter/prototype level. Not settled as of session 3 — revisit.

---

## 10. Theory covered so far (sessions 2-3)

User has been walked through and understands **conceptually**:

- Bayes' rule via 4-rooms tile/wood example
- Prior / Likelihood / Posterior with numeric computation by hand
- Uniform vs non-uniform prior
- **KEY intuition:** observation is only useful if it discriminates; motion adds uncertainty, observation removes it
- Discrete → continuous state space (why particles are needed)
- The 5 PF core steps: init, predict, update, resample, estimate
- Gaussian likelihood (formula + intuition of σ as "trust in sensor")
- Effective sample size (`N_eff`) and resampling threshold
- `atan2` and why angles need special averaging (unit-circle vector trick)
- NumPy vectorized operations mapped to hand-computed Bayes table
- **(session 3)** Monte Carlo sampling — random-sampling intuition (dart-throwing area estimate → particles as "guesses")
- **(session 3)** `create_uniform_particles` vs `create_gaussian_particles` — when each is used; ties to GPS-based init (Gaussian around noisy GPS reading, not uniform)
- **(session 3)** EKF/UKF explained at a conceptual level and contrasted with PF: both assume the belief stays a single Gaussian (unimodal), which fails for ambiguous/multimodal situations (e.g. "which lane") — this is why PF was chosen for the urban-canyon case

He also **wrote and ran** the equivalent of one Bayes update step in Python (4 rooms → 10 positions → 4-particle 1D scenario), producing correct numeric output (`[0.409, 0.409, 0.091, 0.091]`).

**Currently reading:** Labbe Chapter 12, online via nbviewer, at his own pace — stopped partway through (past particle creation code, hasn't reached predict/update/resample code yet).

### Math still to cover when needed

1. Bicycle motion model equations (sin/cos/tan of steering angle)
2. Coordinate transformations: image → camera → vehicle → world
3. Multivariate Gaussian intuition (when observation model becomes 2D)

Cover these **on demand**, tied to specific implementation moments — not upfront.

---

## 11. Curated resources

| Resource | Why | Difficulty |
|----------|-----|------------|
| **Roger Labbe's book Chapter 12 (Particle Filters)** https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python | Definitive filterpy reference | Medium |
| **Cyrill Stachniss "Robot Mapping" YouTube course**, lectures 8-10 | Best free intro to PF in robotics | Medium |
| **3Blue1Brown "Essence of Linear Algebra"** (YouTube) | Visual intuition for matrices | Easy |
| **Probabilistic Robotics** (Thrun et al.), Ch. 2, 4, 8 | Textbook for Bayesian state estimation | Hard |
| **Kuutti et al. [18]** localization survey | Most-cited reference in Despoina's thesis | Survey paper |
| **CARLA docs** | Required for sensor / map / OpenDRIVE APIs | Easy-medium |

---

## 12. Immediate next steps

In rough priority:

1. ~~Install filterpy + friends in `~/carla-env`~~ — DONE (session 3).
2. ~~Ask lab admin for sudo installs~~ — **DEFERRED (session 4).** Checked what's already installed: `cv-bridge`, `image-transport`, `python3-opencv` are present; `vision-opencv`, `ackermann-msgs`, `derived-object-msgs`, `tf-transformations` are missing. But these are only needed for ROS 2 topic integration (Section 5, item 4), which is NOT on the current critical path — all work so far (`spawn_vehicle.py`, `vehicle_spawn.py`) uses the direct `carla` Python API, no ROS involved. User explicitly decided to defer this until ROS integration is actually needed. Revisit then, not before.
3. **Finish reading Labbe Chapter 12** — user stopped partway through (before predict/update/resample code). Continue from there.
4. ~~Add GPS noise simulation~~ — DONE (session 4), but only in `vehicle_spawn.py` (manual-drive reference script), not yet in `spawn_vehicle.py` (autopilot script, the one actually being developed) — worth porting over.
5. ~~Fix 2 known issues in `spawn_vehicle.py`~~ — DONE (session 4): added `None` checks (raise `RuntimeError` with a clear message) after each `try_spawn_actor` call; moved the `render()` throttle check-and-update fully inside `render_lock` so it's atomic across the GNSS/IMU callback threads.
6. **Run Labbe's 1D/2D PF example** locally with filterpy, modify parameters (N, σ), observe results.
7. **CARLA OpenDRIVE exploration:** load `Town10HD_Opt.xodr` (already in repo), walk through lane geometry extraction — ideally via `world.get_map()`/waypoints rather than the old raw-XML `parse_xodr.py` approach (see Section 4 decision). Small Python script that user writes.
8. **Wire GNSS/IMU into filterpy:** feed live sensor readings into a first filterpy PF instance, get a position estimate.

### Things to do at home (no CARLA / lab desktop needed) — added session 4

1. **Finish Labbe Chapter 12** — pick up from where you stopped (before predict/update/resample). Read online via nbviewer: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
2. **Read the urban canyon GPS sources** in `READING_LIST.md` (repo root) — six sources added 27 Jul 2026, on multipath/NLOS and DOP, with concrete error magnitudes (open sky ~3-5m, multipath up to ~30m, NLOS tens-hundreds of meters, severe canyon up to ~60m). Goal: come back able to say what GPS noise level fits a CARLA "urban canyon" test.
3. **Read through `vehicle_spawn.py`'s still-AI-generated parts** — specifically the `MultiCameraDisplay` class and `CsvLogger` class (Section 13 caveat: these haven't been read line-by-line yet, unlike the rest of the script). No CARLA needed, just reading the code and understanding what each part does — you'll need to be able to explain it in the defense.
4. *(Optional, only if you have Python available at home)* Try running Labbe's 1D/2D PF example locally with `filterpy`, tweak `N` (particle count) and `σ` (noise), and see how the results change.

### Things to CONFIRM with the user next session

- Did the supervisor sign off on the OSM → CARLA pivot? (Section 4.)
- Did the lab admin do the apt installs from Section 8?
- Did Labbe Chapter 12 make sense once he reached predict/update/resample? Any concepts still murky?
- Does the GPS noise level (~5m lat/lon, ~8m alt) feel realistic once he's watched it run, or does he want it harsher (e.g. to mimic urban canyon)?

### Rules for mentor-mode (repeated for clarity)

DO NOT, on any work session:
- Write the observation model / motion model / evaluation code for him (that's the thesis)
- Skip explaining math when he asks
- Use English unless quoting code/papers
- Give a 20-section answer when 3 sentences suffice
- Push into code when he hasn't understood the underlying concept

GOOD PATTERNS from session 2: small step → check understanding → next step. Interactive Q&A (what do you think this line does?) worked better than one-way dumps. Everyday analogies (rooms, compass) beat formulas.

---

## 13. Files currently in repo

**Note:** most of these were AI-generated in a previous attempt and should be regarded as scratch/reference, not final code. Exception: `spawn_vehicle.py` had been debugged and extended by the user himself, incrementally, with mentoring (session 3) — it was no longer just an AI stub, it was his working code.

**REVERSED session 4:** the user decided to make `vehicle_spawn.py` the primary script going forward and set `spawn_vehicle.py` aside ("το ξεχνάμε το άλλο"). Reasoning discussed: `vehicle_spawn.py` is functionally more complete (manual control, multi-camera, dual clean/noisy GPS, CSV logging), and its fixed-spawn-point + GPS-noise features were already added this session. **Caveat flagged to user (not yet acted on):** several classes in `vehicle_spawn.py` (e.g. the multi-camera display, CSV logger) are still AI-generated and not yet read/understood line-by-line by the user the way `spawn_vehicle.py` was — worth walking through before building much further, since he needs to be able to defend every part of the codebase he ships.

- `Town10HD_Opt.xodr` — OpenDRIVE map extracted from CARLA (useful, keep)
- `parse_xodr.py` — parser stub (AI-generated, review needed). Does raw XML parsing of road centerlines; per Section 4's decision, future lane work should prefer live `world.get_map()`/waypoints instead of extending this.
- `spawn_vehicle.py` — **SET ASIDE as of session 4** (see reversal note above). Was actively developed by the user in session 3: fixed the `world.blueprint_library()` bug himself, built a working autopilot + GNSS/IMU + live 2-line terminal display script. **Reviewed and fixed session 4** — the 2 issues found (no `None` check after `try_spawn_actor`, race condition in `render()`'s throttle) were both fixed, but the script itself is no longer the active development target.
- `vehicle_spawn.py` — **NOW THE PRIMARY SCRIPT as of session 4.** Bigger, originally AI-generated (pygame manual driving + multi-camera + GNSS/IMU CSV logging, ~400 lines) — parts of it (camera display class, CSV logger) are not yet read/understood line-by-line by the user, unlike `spawn_vehicle.py` was. **Updated session 4:** W-key throttle lowered from full (1.0) to a calmer value (user changed it himself); `spawn_vehicle()` now tries a fixed spawn point (index 0) first instead of a full random shuffle, falling back to a random other point only if occupied; a second GNSS sensor was added with noise attributes (`noise_lat_stddev`/`noise_lon_stddev` = 0.000045° ≈ 5m, `noise_alt_stddev` = 8m, `noise_seed` = 42) alongside the original clean one — both shown in the HUD (`GPS clean` / `GPS noisy`) and logged as separate columns in the CSV.
- `particle_filter.py` — empty stub
- `sensor_data.csv` — logged sensor data (from previous run)
- `GPS_RESEARCH_REPORT.md` — **new, session 5.** Πλήρης βιβλιογραφική αναφορά για το σφάλμα GPS σε urban canyon, γραμμένη από τον assistant κατόπιν ρητού αιτήματος του χρήστη ("διάβασέ τα όλα και δώσε μου αναφορά"). Περιέχει: επαληθευμένους αριθμούς με ακριβή απόδοση, ρητή δήλωση ποιες πηγές ΔΕΝ ήταν προσβάσιμες, τη διάκριση multipath vs NLOS, την επιβεβαίωση της ανισοτροπίας, ανάλυση του πηγαίου κώδικα του CARLA, και προτεινόμενο μοντέλο θορύβου με τεκμηρίωση κάθε παραμέτρου.
- `GPS_NOISE_MODEL.md` — **new, session 5.** Κενός πίνακας τεκμηρίωσης για το μοντέλο θορύβου GPS: 5 φαινόμενα (χρονική συσχέτιση, μεροληψία, NLOS άλματα, εξάρτηση από κτήρια/ανισοτροπία, dropouts) × παράμετροι, με στήλες «Τιμή» και «Πηγή» προς συμπλήρωση καθώς ο χρήστης διαβάζει τις έξι urban-canyon πηγές. Περιέχει επίσης τις αποφάσεις σχεδιασμού της session 5 (βλ. παρακάτω). **Ο χρήστης δεν το έχει συμπληρώσει ακόμα** — είναι το άμεσο "at home" task.
- `READING_LIST.md` — **new, session 4.** Running, dated log of articles/sources recommended for the user to read (started with urban canyon GPS error sources). Add new entries at the top with the date whenever a new source is suggested.
- `town10_grid.npy`, `town10_meta.npy`, `town10_occupancy.png` — precomputed occupancy grid

---

## 14. Session 2 summary (13 July 2026)

**Duration:** ~3 hours

**Accomplished:**

- Verified ROS 2 Humble was already installed (`/opt/ros/humble`) — install blocker RESOLVED
- Verified CARLA + ROS 2 native integration works end-to-end
  - Ran `./CarlaUE4.sh --ros2 -quality-level=Low` + `ros2_native.py`
  - Confirmed `/carla/hero/{gnss, imu, rgb/image, lidar/point_cloud}` topics publish
- **Full theoretical PF walkthrough** (Bayes → Prior/Likelihood/Posterior → Predict/Update/Resample → atan2 → NumPy vectorization)
- User understood the 5 steps conceptually
- Attempted 1D PF implementation exercise → user got stuck at NumPy Gaussian likelihood step
- Backtracked to hand-computed 4-room Bayes in Python (successful — output matched paper computation)
- **Critical decision surfaced (previously not in state file):** supervisor approved use of `filterpy`, no need to write PF core from scratch

**Session-end state:** environment ready; theory covered; user tired but productive. Deferred filterpy hands-on to a fresh session.

**Blockers / pending:**

- Some apt packages still need sudo (Section 8 list)
- User needs to try the pip installs himself
- Reading assignment: Labbe Chapter 12

**Do not repeat mistake from this session:** don't push code before intuition is solid. If he says "I don't understand", stop coding and go back to pen-and-paper.

---

## 15. Session 3 summary (13 July 2026)

**Accomplished:**

- pip installs done (`filterpy`, `opencv-python`; rest already present). Verified `filterpy`/`carla` still import fine despite a `numpy` version bump that conflicts with the unused `invertedai` package (harmless, explained to user).
- Continued Labbe Chapter 12 reading (online, at his own pace) with interactive Q&A: Monte Carlo sampling intuition, `create_uniform_particles` vs `create_gaussian_particles` (and why GPS-based init should be Gaussian, not uniform), `N_eff` recap, EKF/UKF explained and contrasted with PF (unimodal-Gaussian assumption vs particles' ability to hold multiple hypotheses — ties directly to why PF fits the urban-canyon problem).
- **First hands-on CARLA scripting session**, built incrementally by the user with mentoring, starting from the small broken `spawn_vehicle.py` stub (not the big AI-generated `vehicle_spawn.py`, deliberately):
  - User found and fixed the `world.blueprint_library()` → `get_blueprint_library()` bug himself
  - Added GNSS sensor with a `.listen()` callback — learned the callback/event-driven concept (vs polling)
  - Added IMU sensor himself, mirroring the GNSS pattern
  - Iteratively built a live 2-line terminal display: ANSI cursor-movement (`\033[2A`, `\033[K`), a shared `state` dict + `render()`, diagnosed and fixed a "reserved blank lines vs. cursor movement" mismatch bug, diagnosed and fixed a race condition (`threading.Lock`) between the GNSS/IMU background threads, added render throttling (5 Hz) and fixed-width signed number formatting for readability
  - Ran `/simplify` (4 parallel review agents) on the diff; applied the two fixes worth keeping (dedup the sensor `Transform`, `try/finally` actor cleanup on exit) and explicitly skipped others that would have undone the requested 2-line UI or pulled in unrelated `vehicle_spawn.py` machinery
  - Enabled autopilot via CARLA's Traffic Manager with calm-driving settings (`vehicle_percentage_speed_difference`, `distance_to_leading_vehicle`, `ignore_lights_percentage`/`ignore_signs_percentage` = 0)
  - Was mid-explanation of GPS noise simulation (`noise_lat_stddev` etc. on the GNSS blueprint) when the session ended — not yet applied to the file
- Explained that this chat/session history is local to this machine only; the `.md` file is the portable artifact for continuity across machines/sessions — this update is a direct result of that.

**Session-end state:** `spawn_vehicle.py` is a working, clean, thread-safe sensor-logging script with autopilot. GPS noise is the very next thing to add. Labbe Ch.12 reading not finished — stopped before predict/update/resample code.

**Good patterns confirmed again:** small step → check understanding → next step. Letting the user hit real bugs (missing `print()`, race condition, wrong transform) and diagnosing them together, rather than pre-empting them, worked well and produced strong learning moments. When the user explicitly said "κάνε τις αλλαγές/διορθώσεις," applying the already-explained fix directly was appropriate — the mentor-not-code-writer rule is about not doing his thinking for him, not about refusing to type once he's understood and decided.

**Blockers / pending:**

- apt sudo installs still not done (Section 8)
- GPS noise not yet added to `spawn_vehicle.py`
- Labbe Ch.12 unfinished (predict/update/resample sections + the full worked 2D example remain)

---

## 16. Session 4 summary (27 July 2026)

**Accomplished:**

- Reviewed `spawn_vehicle.py` (the autopilot script) at the user's request. Found and explained 2 issues, not yet fixed: (1) no `None` check after `try_spawn_actor` for vehicle/GNSS/IMU — will crash with `AttributeError` if a spawn point is occupied; (2) race condition in `render()`'s throttle check (`now - last_render_time < RENDER_INTERVAL` is checked before the lock is acquired, so both sensor callback threads can pass it before either updates the timestamp). Also noted an unused `cv2` import.
- User changed the manual-drive throttle himself in `vehicle_spawn.py` (`control.throttle = 1.0` on `K_w` was too aggressive) — a small, self-directed fix, no code written for him.
- Fixed spawn point in `vehicle_spawn.py`: walked through why a GPS lat/lon coordinate can't be used directly as a CARLA spawn `Location` (CARLA's local x/y/z vs. WGS84 degrees — the map's own geo-reference converts one to the other, only in the x/y/z → lat/lon direction via `transform_to_geolocation`). User asked for simpler alternatives twice; landed on: pick a fixed index (`0`) from `get_spawn_points()`, try it first, fall back to a random other point only if occupied.
- Added GPS noise to `vehicle_spawn.py`: a second `sensor.other.gnss` actor at the same mount point as the clean one, with `noise_lat_stddev`/`noise_lon_stddev` set to `0.000045` (≈5m, using the ~111,320 m/degree conversion) and `noise_alt_stddev` set to `8.0` (≈8m) — chosen as the midpoint of the supervisor's stated 3-10m consumer-GPS baseline. Both clean and noisy readings now show side by side in the HUD and are logged as separate CSV columns (`latitude`/`latitude_noisy` etc.).
- Recapped full project status for the user (this section's parent update).

**Session-end state:** `vehicle_spawn.py` has a working fixed spawn point and dual clean/noisy GPS. `spawn_vehicle.py` (the actively-developed script) still has the 2 open issues above and does not yet have GPS noise — porting the noise setup there, and fixing the 2 issues, are natural next steps.

**Good patterns confirmed again:** user pushed back twice on an over-engineered solution ("δεν υπαρχει πιο απλος τροπος?") before landing on the simple fixed-spawn-point approach — worth defaulting to the simplest working option first next time, rather than the more "correct"/general one, unless he asks for precision.

**Blockers / pending:**

- apt sudo installs still not done (Section 8)
- GPS noise only in `vehicle_spawn.py`, not yet ported to `spawn_vehicle.py`
- Labbe Ch.12 unfinished (predict/update/resample sections + the full worked 2D example remain)

---

## 17. Session 5 summary (24 Αυγούστου 2026)

**Accomplished:**

- **Windows home setup (νέο δεύτερο μηχάνημα).** Ο χρήστης δούλευε από το σπίτι σε Windows. Στήθηκε: Python 3.12.10 (το default ήταν 3.14, ασύμβατο με το CARLA wheel `cp312`), venv στο `C:\Users\vpaliouras\carla-env`, `carla` 0.9.16 από το τοπικό wheel + pygame/numpy/filterpy/matplotlib/opencv/scipy/sklearn/shapely. **Επιβεβαιώθηκε από τον χρήστη ότι δουλεύει** (CARLA + `vehicle_spawn.py`). Λεπτομέρειες στο Section 7.
- **Εννοιολογική συζήτηση — η καρδιά της διπλωματικής.** Ο χρήστης είπε ρητά «έχω μπερδευτεί πάρα πολύ» μετά από τεχνική συζήτηση για κάτοψη χάρτη. Έγινε πλήρες backtrack σε αναλογίες:
  - Η βασική ιδέα (χάρτης + τι βλέπει η κάμερα → πού πρέπει να στέκομαι), με αναλογία «δεμένα μάτια στη Θεσσαλονίκη».
  - Γιατί χρειάζονται **και** λωρίδες **και** κτήρια: λωρίδες → εγκάρσια θέση (πλάτος), κτήρια → κατά μήκος (μήκος). Ο χρήστης το απάντησε σωστά μόνος του («γιατί λωρίδες υπάρχουν παντού») ✅
  - **Η κάμερα δεν μετράει απόσταση, μετράει γωνία (bearing).** Το κόλπο: δεν υπολογίζεις θέση από την εικόνα — κάθε particle **προβλέπει** τι γωνίες θα έβλεπε από τη θέση του βάσει χάρτη, και συγκρίνεται με την πραγματική μέτρηση.
  - Διορθώθηκε παρανόηση: η σύγκριση είναι πάντα *πρόβλεψη particle ↔ πραγματική μέτρηση κάμερας*, ποτέ particle με particle. Και τα βάρη είναι **μαλακά** (Gaussian likelihood), όχι σκληρό κατώφλι.
  - Δηλώθηκε ρητά ότι η bearing-only προσέγγιση **δεν είναι πρωτότυπη ιδέα** — είναι κλασικό landmark-based observation model, και ότι το **data association** (ποια γωνία χάρτη αντιστοιχεί σε ποιο κτήριο) παραλείφθηκε σκόπιμα για απλότητα.
- **Βιβλιογραφική έρευνα κατόπιν ρητού αιτήματος.** Ο χρήστης ζήτησε να διαβαστούν όλες οι πηγές αναλυτικά και να ετοιμαστεί αναφορά, τονίζοντας «θέλω να είναι 100% αλήθεια, μη μου πεις κάτι για να με ικανοποιήσεις». Αποτέλεσμα: `GPS_RESEARCH_REPORT.md` + συμπληρωμένο `GPS_NOISE_MODEL.md`.

**Κύρια ευρήματα της έρευνας (όλα με πηγή στο report):**

- **Urban canyon vs ανοιχτός ουρανός: ~6-10× χειρότερα στον μέσο όρο** (3-5 m → ~31 m), **~35-50× στο χειρότερο** (έως 177.59 m). Πηγή: Wen & Hsu, Χονγκ Κονγκ, u-blox M8T.
- **Multipath ≠ NLOS** — γνήσιο multipath ~1 m (σ), NLOS δεκάδες μέτρα. Τα νούμερα «multipath» στη βιβλιογραφία είναι συνήθως NLOS (Groves 2011). **Αλλάζει τη δομή του μοντέλου: χρειάζονται δύο μηχανισμοί.**
- **Η ανισοτροπία ΕΠΙΒΕΒΑΙΩΘΗΚΕ** (είχε σημειωθεί ως «προς επαλήθευση» νωρίτερα στην ίδια session): εγκάρσιο σφάλμα χειρότερο από κατά μήκος, ADOP < CDOP. **Νέα λεπτομέρεια:** εξαρτάται από τον προσανατολισμό του δρόμου — «ελαφρώς» χειρότερο σε δρόμους Β-Ν, «ουσιωδώς» σε Α-Δ.
- **Groves 2011 περιέχει την πρόταση που δικαιολογεί τη διπλωματική:** για βαθιά urban canyons δεν υπάρχει προοπτική προσδιορισμού λωρίδας με συμβατικό GNSS. Ανοιχτό PDF, 14 σελίδες, **αξίζει πλήρη ανάγνωση**.
- **Ο θόρυβος του CARLA επαληθεύτηκε από τον πηγαίο κώδικα** (`GnssSensor.cpp`): λευκός Gaussian, νέο δείγμα κάθε tick, **καμία χρονική συσχέτιση**. Υποστηρίζει όμως `noise_lat_bias` — δεν το ξέραμε.
- **Γνωστό bug CARLA #4235:** σφάλματα εκατοντάδων μέτρων σε round-trip μετατροπές x/y ↔ lat/lon. **→ Απόφαση: δούλεψε αποκλειστικά σε μέτρα x/y, μην κάνεις καθόλου μετατροπή.** Ακυρώνει την εκκρεμότητα «lat/lon → x/y» που είχε προγραμματιστεί.
- **Κατώφλι πλήρους απώλειας στίγματος:** μέσο ύψος κτηρίων 40-45 m → λιγότεροι από 4 δορυφόροι.

**Τιμιότητα/επιφυλάξεις που δηλώθηκαν ρητά στον χρήστη:**

- **3 από τις 6 αρχικές πηγές είναι paywalled** ([S2], [S3], [S6]) — δεν διαβάστηκαν.
- **Η [S1] (USPTO patent) δεν είναι έγκυρη πηγή** — σαρωμένο δίπλωμα ευρεσιτεχνίας, μπήκε από λάθος. Σημειώθηκε ως άκυρη στο `READING_LIST.md`.
- **Οι περιγραφές που είχαν αποδοθεί στις [S2]/[S3] στο αρχικό `READING_LIST.md` («multipath έως ~30m», «RMS ~3m») δεν επαληθεύτηκαν** — διορθώθηκε ρητά στο αρχείο. Τα μεγέθη επιβεβαιώνονται από άλλες πηγές, αλλά η απόδοση σε εκείνα τα papers δεν στέκει.
- **Δεν βρέθηκε δημοσιευμένη συχνότητα εμφάνισης NLOS συμβάντων** ούτε χρονική σταθερά ειδικά για αυτοκίνητο σε urban canyon — σημειώθηκαν ως κενά/παραδοχές.

**Session-end state:** Το home setup δουλεύει. Ο χρήστης έχει καθαρή εννοιολογική εικόνα του observation model. Έχει τεκμηριωμένο μοντέλο θορύβου έτοιμο προς υλοποίηση, με πηγή σε κάθε παράμετρο.

**Εκκρεμότητες / επόμενα βήματα:**

1. **Ο χρήστης έχει να διαβάσει:** Groves 2011 (νέο, προτεραιότητα), Probabilistic Robotics Κεφ. 6 (measurement models — **ζήτησε ρητά υπενθύμιση**), Labbe Ch.12 (ακόμα ημιτελές).
2. **Ground truth στο CSV** (`vehicle.get_transform()`) — δεν έγινε ακόμα, ήταν το επόμενο βήμα πριν την εκτροπή.
3. **`gps_noise.py`** — δικό μας μοντέλο θορύβου σε μέτρα, βάσει του πίνακα στο `GPS_NOISE_MODEL.md`.
4. **Κάτοψη (minimap)** — ο χρήστης τη ζήτησε· διαπιστώθηκε ότι waypoints + ground truth **δεν** χρειάζονται μετατροπή συντεταγμένων, άρα μπορεί να γίνει άμεσα. Το `no_rendering_mode.py` του CARLA (1319 γραμμές) απορρίφθηκε ως μη υπερασπίσιμο· θα φτιαχτεί δική του μίνι έκδοση (~60 γραμμές). Το κρίσιμο κομμάτι είναι η μετατροπή μέτρα→pixels.
5. **Μέτρησε το aspect ratio (ύψος κτηρίου / πλάτος δρόμου) του Town10HD** — θα είναι πρωτότυπο νούμερο στη διπλωματική.
6. **Προθεσμία διπλωματικής: ΑΓΝΩΣΤΗ** — ρωτήθηκε ο χρήστης, δεν απάντησε. **Ξαναρώτησέ τον** — χρειάζεται για ρεαλιστικό σχεδιασμό χρόνου.

**Working-style notes που επιβεβαιώθηκαν ξανά:**

- Όταν είπε «μπερδεύτηκα», το σωστό ήταν πλήρες backtrack σε αναλογίες — **όχι** άλλος κώδικας. Δούλεψε.
- Ο έλεγχος κατανόησης με ερώτηση («γιατί δεν φτάνουν μόνο οι λωρίδες;») έδωσε σωστή απάντηση και επιβεβαίωσε ότι η έννοια κάθισε.
- Ζητάει ενεργά **τιμιότητα** για την προέλευση των ιδεών («πώς σου ήρθε αυτό;») και για την αξιοπιστία των πηγών. Το να δηλώνεται ρητά τι είναι κλασικό/δανεικό και τι δεν επαληθεύτηκε, το εκτιμά — μην το παραλείπεις.

---

## 18. Session 6 summary (24 Σεπτεμβρίου 2026) — μεταφορά σε Mac

**Νέο μηχάνημα:** ο χρήστης δουλεύει πλέον σε **Mac** (`/Users/paliouv/Localization_thesis`). Το CARLA τρέχει στο **Linux desktop** (lab Ubuntu), όχι στον Mac. Η συνομιλία της session 5 (Windows laptop) αντιγράφηκε στο `~/.claude/projects/-Users-paliouv-Localization-thesis/`.

**Ανακτήθηκαν από το ιστορικό της session 5** (δεν είχαν έρθει στον Mac): `GPS_NOISE_MODEL.md`, `GPS_RESEARCH_REPORT.md`, `READING_LIST.md`, και αυτό το αρχείο. Είναι ίδια με την τελική τους μορφή στο Windows laptop.

**Git:** ο κώδικας των sessions 3-4 ήταν στο GitHub (commit `24e39d7`, 27 Ιουλ). Ο Mac έκανε pull (fast-forward) στις 24 Σεπ. Μια παλιά, μη-committed αλλαγή του Mac στο `spawn_vehicle.py` (αρχή για RGB κάμερα, πάνω στο παλιό stub) κρατήθηκε στο `git stash` — είναι παρωχημένη σε σχέση με την έκδοση της session 3.

**Απόφαση: ζώνες θορύβου με το χέρι.** Αντί για θόρυβο υπολογισμένο από τα κτήρια (επίπεδο 2), ορίζονται χειροκίνητα ορθογώνιες ζώνες στον χάρτη (μέτρα x/y CARLA), η καθεμιά με δικά της σ. Καταγράφηκε ως §9 στο `GPS_NOISE_MODEL.md`. Μπορεί αργότερα να αντικατασταθεί από υπολογισμό από τα κτήρια χωρίς αλλαγή στον υπόλοιπο κώδικα.

**Νέα αρχεία (γράφτηκαν από τον assistant, κατόπιν ρητού αιτήματος «κάνε τα»):**
- `gps_noise.py` — το μοντέλο θορύβου (Gauss-Markov bias + λευκός θόρυβος + άλματα NLOS + απώλεια σήματος + ζώνες + ανισοτροπία). **Ο χρήστης πρέπει να το διαβάσει γραμμή-γραμμή** πριν το χρησιμοποιήσει — θα χρειαστεί να το υπερασπιστεί.
- `test_gps_noise.py` — δοκιμή χωρίς CARLA, με ψεύτικη διαδρομή· βγάζει γραφήματα.
- `export_buildings.py` — τρέχει στο Linux desktop με ανοιχτό CARLA· βγάζει θέση/μέγεθος/ύψος κάθε κτηρίου σε `town10_buildings.json`.

**Σημείωση για τον χάρτη:** το `town10_grid.npy` είναι σε συντεταγμένες OpenDRIVE· το CARLA έχει **ανάποδα τον άξονα y** (`y_carla = -y_xodr`). Δεν έχει επιβεβαιωθεί ακόμα με πραγματική θέση οχήματος.

**Εκκρεμότητες (επιπλέον αυτών της session 5):**
1. ~~Συγχρονισμός κώδικα με το GitHub~~ — έγινε (pull, 24 Σεπ).
2. Τρέξε το `export_buildings.py` στο Linux desktop → στείλε το `town10_buildings.json`.
3. Επιβεβαίωσε την αναστροφή του y: στείλε `vehicle.get_location()` + πού είναι το όχημα στον χάρτη.
4. Διάλεξε ζώνες θορύβου με βάση τα κτήρια.
5. **Χρονοδιάγραμμα (δηλώθηκε 24 Σεπ 2026):** υλοποίηση έως **15 Δεκεμβρίου 2026**, συγγραφή έως **15 Ιανουαρίου 2027**. Βλ. πλάνο παρακάτω.

### Πλάνο: υλοποίηση έως 15/12/2026, συγγραφή έως 15/1/2027 (προτάθηκε 24 Σεπ, προς επιβεβαίωση)

| Περίοδος | Στόχος |
|---|---|
| 24/9 – 7/10 | ~~Συγχρονισμός κώδικα~~ ✅. Κτήρια → ζώνες θορύβου. Ground truth στο CSV. |
| 8/10 – 21/10 | Πρώτο PF με filterpy: predict (IMU/ταχύτητα) + update μόνο με GPS, offline στο CSV. Κάτοψη (minimap). |
| 22/10 – 11/11 | Λωρίδες: ανίχνευση + ταίριασμα με OpenDRIVE → εγκάρσια διόρθωση. |
| 12/11 – 2/12 | Κτήρια: semantic segmentation + γωνίες (bearings) → διόρθωση κατά μήκος. |
| 3/12 – 15/12 | Πειράματα: RMSE, λωρίδα σωστή %, σενάρια χωρίς GPS. Figures. |
| 16/12 – 15/1 | Συγγραφή (περιλαμβάνει γιορτές — τα θεωρητικά κεφάλαια καλό να ξεκινήσουν νωρίτερα). |

**Κίνδυνος:** αν καθυστερήσουν οι λωρίδες, τα κτήρια στριμώχνονται. Ρώτα τον επιβλέποντα αν τα κτήρια μπορούν να είναι σε ελαφρύτερο/πρωτότυπο επίπεδο.

### Απόφαση (24 Σεπ): κύριο script ξανά το `spawn_vehicle.py` → μετονομάστηκε σε `BMW_spawn.py`

Ο χρήστης αποφάσισε: **ξεχνάμε το `vehicle_spawn.py`, χρησιμοποιούμε μόνο το `spawn_vehicle.py`** (μετονομάστηκε σε **`BMW_spawn.py`** για να ξεχωρίζει) (αντιστρέφει την απόφαση της session 4). Το `spawn_vehicle.py` είναι το δικό του, γραμμένο με καθοδήγηση (autopilot + GNSS + IMU + οθόνη 2 γραμμών). Δεν έχει ακόμα: καταγραφή σε CSV, ground truth, κάμερες. Ό,τι χρειάζεται (CSV, ground truth, κάτοψη, `gps_noise.py`) μπαίνει εκεί.

### Δεδομένα κτηρίων (25 Σεπ)

- **Τώρα:** κουτιά από `world.get_environment_objects(Buildings)` → `export_buildings.py` → `town10_buildings.json`. Αρκούν για τις ζώνες θορύβου.
- Το `no_rendering_mode.py` **δεν** ζωγραφίζει κτήρια (μόνο δρόμους/λωρίδες/σήματα).
- **Αργότερα (κτήρια/bearings, ~12/11):** αν τα κουτιά είναι χοντροκομμένα, ακριβή περιγράμματα από κάμερα **instance/semantic segmentation από ψηλά** (+ **depth** για ύψος). Προσοχή στην προοπτική: κάμερα πολύ ψηλά, στενό FOV, πολλές λήψεις σε πλέγμα. Δεν έχει δοκιμαστεί ακόμα.

### Καθάρισμα repo — στο τέλος (ζητήθηκε 24 Σεπ να μείνει για αργότερα)

1. Το `cache/no_rendering_mode/Town10HD_Opt_*.tga` μπήκε κατά λάθος στο git (commit `f65840c`, 688 KB). Διόρθωση: `git rm -r --cached cache` + `cache/` στο `.gitignore` (το αρχείο μένει στον δίσκο — το χρειάζεται το `no_rendering_mode.py`). Όχι αλλαγή ιστορικού.
2. Το `sensor_data.csv` ξαναγράφεται σε κάθε τρέξιμο του `vehicle_spawn.py` → καταγραφές σε ξεχωριστά αρχεία με ημερομηνία, και απόφαση τι μπαίνει στο git.
3. Παλιό `git stash` στον Mac (αρχή RGB κάμερας στο παλιό `spawn_vehicle.py`) → drop.
4. Μετονομασία `THESIS_STATE (1).md` → `THESIS_STATE.md`.
