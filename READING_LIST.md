# Reading List

Πηγές/άρθρα που έχουν προταθεί για διάβασμα κατά τη διάρκεια της διπλωματικής, με ημερομηνία που συζητήθηκαν. Νεότερα πρώτα.

---

## 2026-07-27 — Urban canyon GPS error (για να υπολογίσεις πόσο noise να προσομοιώσεις στο CARLA)

- [Urban canyon effect / satellite visibility & DOP overview](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9857474) — βασική εξήγηση: γιατί τα κτήρια χαλάνε τη γεωμετρία των δορυφόρων (DOP)
- [Characterization and mitigation of urban GNSS multipath effects on smartphones (ScienceDirect)](https://www.sciencedirect.com/science/article/abs/pii/S0263224123013301) — τυπικά μεγέθη σφάλματος: multipath έως ~30m, NLOS δεκάδες-εκατοντάδες μέτρα
- [Measuring GNSS Multipath Distributions in Urban Canyon Environments](https://www.researchgate.net/publication/270582695_Measuring_GNSS_Multipath_Distributions_in_Urban_Canyon_Environments) — field test, RMS ~3m σε μέτριο urban canyon
- [3D LiDAR Aided GNSS NLOS Mitigation in Urban Canyons (arXiv)](https://arxiv.org/pdf/2112.06108) — ακραία σφάλματα έως ~60m (π.χ. είσοδος tunnel)
- [Statistical Multipath Model Based on Experimental GNSS Data in Static Urban Canyon (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC5948528/) — στατιστικό μοντέλο σφάλματος, πιο τυπικό από απλό Gaussian
- [Simulating GNSS multipath in urban environments using 3D ray tracing for automotive applications (ScienceDirect)](https://www.sciencedirect.com/science/article/pii/S1574119226000799) — πιο κοντά στο δικό σου use-case (automotive simulation)

**Γιατί:** να καταλάβεις το φαινόμενο urban canyon και να διαλέξεις ρεαλιστικές τιμές `noise_lat_stddev`/`noise_lon_stddev` για το GNSS blueprint στο CARLA όταν προσομοιώνεις "κακό" GPS.

---

## Παλαιότερες πηγές (από sessions 2-3, ημερομηνία σύστασης όχι ακριβώς γνωστή — προστέθηκαν εδώ αναδρομικά)

- **Roger Labbe — "Kalman and Bayesian Filters in Python", Chapter 12 (Particle Filters)** — https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python — βασική αναφορά για το filterpy, σε εξέλιξη ανάγνωσης
- **Cyrill Stachniss — "Robot Mapping" (YouTube), lectures 8-10** — καλή δωρεάν εισαγωγή σε Particle Filters στη ρομποτική
- **3Blue1Brown — "Essence of Linear Algebra" (YouTube)** — οπτική διαίσθηση για πίνακες, χρήσιμο για το αδύναμο σου σημείο στα μαθηματικά
- **Probabilistic Robotics (Thrun, Burgard, Fox), Ch. 2, 4, 8** — βαρύ σύγγραμμα αναφοράς για Bayesian state estimation
- **Kuutti et al. — localization survey [18]** — το πιο αναφερόμενο paper στη διπλωματική της Δέσποινας
- **Despoina Christodoulou's thesis** (Feb 2026, ίδιο σχολείο) — PF localization για BFMC όχημα, η κύρια πηγή αναφοράς για δομή/μεθοδολογία

---

*Πρόσθεσε νέα entries πάντα στην κορυφή, με ημερομηνία, ώστε να μένει χρονολογικό.*
