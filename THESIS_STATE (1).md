# Thesis — Project State

**Topic:** Map-based fine-grained localization of an autonomous vehicle in CARLA simulation, using a Particle Filter with camera + IMU + noisy GPS, focused on GPS-denied scenarios (urban canyons, tunnels).

**Last updated:** 13 July 2026 (session 3 — filterpy installed, Labbe Ch.12 in progress, first working CARLA sensor script)
**Status:** filterpy + deps installed in venv. Labbe Chapter 12 partially read and understood (motivation, Monte Carlo, particle init, N_eff, EKF/UKF context). First hands-on CARLA script built and working: vehicle spawn + calm autopilot (Traffic Manager) + GNSS/IMU sensors with live terminal display. About to add GPS noise simulation.

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

**Ignore the laptop.** User explicitly said forget it, only desktop.

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

### Scope decision

**Option A (lane-only, PRIMARY):** Lane detection + OpenDRIVE matching + PF for lateral correction. Solves "which lane am I in".

**Option B (building-based, STRETCH):** Semantic segmentation of building facades + CARLA building bboxes. Solves urban canyon. Only if Option A is rock-solid by end of month 2.

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
2. **Ask lab admin for sudo installs** (see Section 8, apt install command) — still pending.
3. **Finish reading Labbe Chapter 12** — user stopped partway through (before predict/update/resample code). Continue from there.
4. **Add GPS noise simulation** to `spawn_vehicle.py` — was mid-explanation when session 3 ended (blueprint attributes `noise_lat_stddev`/`noise_lon_stddev`/`noise_alt_stddev`, suggested starting value ~0.000045° ≈ 5m via the 111,320 m/degree conversion). User had not yet applied this — pick up here next session.
5. **Run Labbe's 1D/2D PF example** locally with filterpy, modify parameters (N, σ), observe results.
6. **CARLA OpenDRIVE exploration:** load `Town10HD_Opt.xodr` (already in repo), walk through lane geometry extraction — ideally via `world.get_map()`/waypoints rather than the old raw-XML `parse_xodr.py` approach (see Section 4 decision). Small Python script that user writes.
7. **Wire GNSS/IMU into filterpy:** once noise is added, feed live sensor readings from `spawn_vehicle.py` into a first filterpy PF instance, get a position estimate.

### Things to CONFIRM with the user next session

- Did the supervisor sign off on the OSM → CARLA pivot? (Section 4.)
- Did the lab admin do the apt installs from Section 8?
- Did Labbe Chapter 12 make sense once he reached predict/update/resample? Any concepts still murky?
- Did he add the GPS noise attributes himself, or pick up from where session 3 left off?

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

**Note:** most of these were AI-generated in a previous attempt and should be regarded as scratch/reference, not final code. Exception: `spawn_vehicle.py` has since been debugged and extended by the user himself, incrementally, with mentoring (session 3) — it is no longer just an AI stub, it's his working code.

- `Town10HD_Opt.xodr` — OpenDRIVE map extracted from CARLA (useful, keep)
- `parse_xodr.py` — parser stub (AI-generated, review needed). Does raw XML parsing of road centerlines; per Section 4's decision, future lane work should prefer live `world.get_map()`/waypoints instead of extending this.
- `spawn_vehicle.py` — **actively developed by the user, session 3.** Was an 18-line broken AI stub (`world.blueprint_library()` bug — user found and fixed it himself), now a working script: spawns vehicle, enables autopilot via Traffic Manager with calm-driving settings (30% under speed limit, 5m following distance, never ignores lights/signs), attaches GNSS + IMU sensors with `.listen()` callbacks, renders a live 2-line terminal display (throttled to 5Hz, thread-safe via `threading.Lock`, fixed-width signed number formatting), and cleans up all actors on exit via `try/finally`. GPS noise not yet added (next step).
- `vehicle_spawn.py` — bigger AI-generated reference script (pygame manual driving + multi-camera + GNSS/IMU CSV logging, ~400 lines). Kept as reference only; NOT the basis for `spawn_vehicle.py`'s development — deliberately started small instead (see session 3 summary).
- `particle_filter.py` — empty stub
- `sensor_data.csv` — logged sensor data (from previous run)
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
