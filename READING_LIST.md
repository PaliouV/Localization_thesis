# Reading List

Πηγές/άρθρα που έχουν προταθεί για διάβασμα κατά τη διάρκεια της διπλωματικής, με ημερομηνία που συζητήθηκαν. Νεότερα πρώτα.

---

## 2026-08-24 — ΝΕΕΣ ΠΗΓΕΣ από τη βιβλιογραφική έρευνα (session 5)

Βρέθηκαν κατά την ανάλυση των urban-canyon πηγών. **Όλες ανοιχτές/δωρεάν.** Πλήρης ανάλυση: [`GPS_RESEARCH_REPORT.md`](GPS_RESEARCH_REPORT.md).

- ⭐ **Groves, P.D. (2011). "Shadow Matching: A New GNSS Positioning Technique for Urban Canyons." *Journal of Navigation* 64(3), 417-430** — https://discovery.ucl.ac.uk/1308009/1/1308009_JNav%20Shadow%202011.pdf
  **Η πιο πολύτιμη πηγή που βρέθηκε.** Περιέχει: (α) τη διάκριση multipath (~1 m) vs NLOS (δεκάδες μέτρα), (β) την επιβεβαίωση της ανισοτροπίας εγκάρσιου/κατά μήκος σφάλματος, (γ) πλήρη ανάλυση UERE (σύνολο 2.6 m), (δ) την πρόταση που δικαιολογεί ολόκληρη τη διπλωματική: για βαθιά urban canyons το συμβατικό GNSS δεν μπορεί να προσδιορίσει λωρίδα κυκλοφορίας. **Διάβασέ το ολόκληρο — είναι μόνο 14 σελίδες.**
- **Wen & Hsu, "3D LiDAR Aided GNSS NLOS Mitigation"** — https://arxiv.org/abs/2112.06108 *(ήταν [S4], τώρα διαβασμένο)* — πραγματικές μετρήσεις Χονγκ Κονγκ με u-blox: 31 m μέσο σφάλμα, 178 m μέγιστο.
- **"Simulation-based Analysis of Multipath Delay Distributions in Urban Canyons"** — https://arxiv.org/pdf/2006.14873 — κατώφλι: μέσο ύψος κτηρίων 40-45 m → λιγότεροι από 4 δορυφόροι → κανένα στίγμα.
- **GPS.gov, "GPS Accuracy"** — https://www.gps.gov/gps-accuracy — επίσημη βάση αναφοράς (≤3 m οριζόντια, 95%).
- **CARLA `GnssSensor.cpp`** — https://github.com/carla-simulator/carla/blob/master/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/GnssSensor.cpp — επιβεβαιώνει ότι ο θόρυβος του CARLA είναι λευκός Gaussian χωρίς χρονική συσχέτιση.
- **CARLA issue #4235** — https://github.com/carla-simulator/carla/issues/4235 — γνωστά σφάλματα εκατοντάδων μέτρων στις μετατροπές x/y ↔ lat/lon.

---

## 2026-08-24 — Observation model: πώς η κάμερα δίνει θέση (ΕΚΚΡΕΜΕΙ — ο χρήστης ζήτησε υπενθύμιση)

- **Probabilistic Robotics (Thrun), Κεφ. 6 — Measurement Models**, ειδικά η ενότητα για **landmark / feature-based models** (range + bearing). *Ο ακριβής αριθμός ενότητας δεν έχει επιβεβαιωθεί — τσέκαρε τα περιεχόμενα.* Ο χρήστης **έχει** το βιβλίο.

**Γιατί:** είναι το θεωρητικό υπόβαθρο του observation model της διπλωματικής. Συζητήθηκε στο session 5: η κάμερα δεν μετράει απόσταση, μετράει **γωνία** (bearing) — κάθε particle προβλέπει τι γωνίες θα έβλεπε από τη θέση του βάσει χάρτη, και συγκρίνεται με την πραγματική μέτρηση. Κλασικό bearing-only localization. Καλύπτει και το **data association** (ποια γωνία του χάρτη αντιστοιχεί σε ποιο κτήριο που βλέπω) — το κομμάτι που η Δέσποινα έλυσε με KD-tree.

**Σημείωση:** αυτό προστίθεται στα κεφ. 2, 4, 8 που ήδη ήταν στη λίστα. Προτεινόμενη σειρά: 4.3 (Particle Filter) → 6 (Measurement Models) → 8 (Monte Carlo Localization).

---

## 2026-07-27 — Urban canyon GPS error (για να υπολογίσεις πόσο noise να προσομοιώσεις στο CARLA)

- ~~[Urban canyon effect / DOP overview](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9857474)~~ — ❌ **ΑΚΥΡΗ ΠΗΓΗ, ΑΓΝΟΗΣΕ ΤΗΝ.** Ελέγχθηκε 24 Αυγ 2026: **δεν είναι επιστημονικό άρθρο**, είναι δίπλωμα ευρεσιτεχνίας USPTO σε σαρωμένες εικόνες. Μπήκε από λάθος στη λίστα. Αντικαταστάθηκε από το Groves 2011 παραπάνω.
- [Characterization and mitigation of urban GNSS multipath effects on smartphones (ScienceDirect)](https://www.sciencedirect.com/science/article/abs/pii/S0263224123013301) — ⚠️ **ΚΛΕΙΔΩΜΕΝΟ (paywall, HTTP 403)** — δεν επαληθεύτηκε. Η αρχική περιγραφή («multipath έως ~30m, NLOS δεκάδες-εκατοντάδες μέτρα») **δεν προέρχεται από ανάγνωση του κειμένου** — μην την αναφέρεις ως τεκμηριωμένη. Τα μεγέθη επιβεβαιώνονται ανεξάρτητα από Wen & Hsu / Groves.
- [Measuring GNSS Multipath Distributions in Urban Canyon Environments](https://ieeexplore.ieee.org/document/6873332/) — **Xie & Petovello (2015), IEEE TIM 64(2):366-377.** ⚠️ **ΚΛΕΙΔΩΜΕΝΟ** — δεν επαληθεύτηκε. Η περιγραφή «RMS ~3m» **δεν επιβεβαιώθηκε**. Το ΠΑΜΑΚ πιθανότατα έχει συνδρομή IEEE Xplore — αξίζει να το κατεβάσεις από εκεί.
- [3D LiDAR Aided GNSS NLOS Mitigation in Urban Canyons (arXiv)](https://arxiv.org/pdf/2112.06108) — ακραία σφάλματα έως ~60m (π.χ. είσοδος tunnel)
- [Statistical Multipath Model Based on Experimental GNSS Data in Static Urban Canyon (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC5948528/) — στατιστικό μοντέλο σφάλματος, πιο τυπικό από απλό Gaussian
- [Simulating GNSS multipath in urban environments using 3D ray tracing for automotive applications (ScienceDirect)](https://www.sciencedirect.com/science/article/pii/S1574119226000799) — πιο κοντά στο δικό σου use-case (automotive simulation)

**Γιατί:** να καταλάβεις το φαινόμενο urban canyon και να διαλέξεις ρεαλιστικές τιμές `noise_lat_stddev`/`noise_lon_stddev` για το GNSS blueprint στο CARLA όταν προσομοιώνεις "κακό" GPS.

---

## Παλαιότερες πηγές (από sessions 2-3, ημερομηνία σύστασης όχι ακριβώς γνωστή — προστέθηκαν εδώ αναδρομικά)

- **Roger Labbe — "Kalman and Bayesian Filters in Python", Chapter 12 (Particle Filters)** — repo: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python · διάβασέ το με εικόνες μέσω nbviewer: https://nbviewer.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb — βασική αναφορά για το filterpy, σε εξέλιξη ανάγνωσης
- **Cyrill Stachniss — "Robot Mapping" / Particle Filters (YouTube)** — Particle Filter & Monte Carlo Localization video: https://www.youtube.com/watch?v=MsYlueVDLI0 · όλα τα μαθήματα (hub): https://www.ipb.uni-bonn.de/teaching/ — καλή δωρεάν εισαγωγή σε Particle Filters στη ρομποτική
- **3Blue1Brown — "Essence of Linear Algebra" (YouTube playlist)** — https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab — οπτική διαίσθηση για πίνακες, χρήσιμο για το αδύναμο σου σημείο στα μαθηματικά
- **Probabilistic Robotics (Thrun, Burgard, Fox), Ch. 2, 4, 8** — επίσημη σελίδα: https://mitpress.mit.edu/9780262201629/probabilistic-robotics/ (πλήρες βιβλίο όχι δωρεάν — δες βιβλιοθήκη UoM) · δωρεάν σχετικό tech report του Thrun: https://www.cs.cmu.edu/~thrun/papers/thrun.probrob.pdf — βαρύ σύγγραμμα αναφοράς για Bayesian state estimation
- **Kuutti et al. — localization survey [18]** — δωρεάν PDF: https://people.computing.clemson.edu/~jmarty/projects/lowLatencyNetworking/papers/RecentEdgeML-5GMEC/SurveyofStateoftheArtlocalizationForAutonomousVehicles.pdf — το πιο αναφερόμενο paper στη διπλωματική της Δέσποινας
- **Despoina Christodoulou's thesis** (Feb 2026, ίδιο σχολείο) — δεν υπάρχει δημόσιο link· αρχείο ήδη διαθέσιμο τοπικά. PF localization για BFMC όχημα, η κύρια πηγή αναφοράς για δομή/μεθοδολογία

---

*Πρόσθεσε νέα entries πάντα στην κορυφή, με ημερομηνία, ώστε να μένει χρονολογικό.*
