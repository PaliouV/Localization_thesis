import argparse

import numpy as np
from filterpy.monte_carlo import multinomial_resample
from gps_noise_v2 import GpsNoise

parser = argparse.ArgumentParser()
parser.add_argument("--receiver", choices=["f9p", "m8t"], default="f9p",
                    help="ποιος δέκτης GPS: f9p (σύγχρονος) ή m8t (παλιός)")
parser.add_argument("--csv", default="drive_2026-10-21_13-54-17.csv", help="ποια καταγραφή")
parser.add_argument("--seed", type=int, default=0,
                    help="τα «ζάρια» (του φίλτρου ΚΑΙ του θορύβου GPS): ίδιο seed → ίδιο αποτέλεσμα")
# Για τη ρύθμιση (tune_pf.py): αν δοθούν, αντικαθιστούν τις τιμές του φίλτρου που είναι πιο κάτω.
parser.add_argument("--gps-sigma", type=float)
parser.add_argument("--velocity-sigma", type=float)
parser.add_argument("--turn-rate-sigma", type=float)
parser.add_argument("--show", action="store_true", help="ζωντανή οπτικοποίηση στον χάρτη (pf_view.py)")
parser.add_argument("--start", type=float, default=0,
                    help="με --show: από ποιο λεπτό της καταγραφής να αρχίσει να ζωγραφίζει")
args = parser.parse_args()

# Ρυθμίσεις του φίλτρου για κάθε δέκτη GPS. Είναι αυτό που «πιστεύει» το φίλτρο για το GPS,
# ΟΧΙ τα νούμερα του θορύβου (gps_noise_v2.py): αυτά είναι ο «πραγματικός κόσμος», που το φίλτρο δεν ξέρει.
# gps_sigma: βρέθηκε με το tune_pf.py (2 Νοε 2026, καταγραφή 21 Οκτ). Δοκιμάστηκε και σ που αλλάζει μόνο του
#   (IAE, Mohamed 1999) και βγήκε χειρότερο, γιατί το λάθος του GPS στην πόλη αλλάζει αργά (Wen et al. 2021).
# pos_sigma (πόσο απλώνονται τα particles στην αρχή): νούμερο δοκιμής, δεν ρυθμίστηκε.
SETTINGS = {
    "f9p": {"pos_sigma": 3,  "gps_sigma": 12},    # m
    "m8t": {"pos_sigma": 50, "gps_sigma": 15},    # m
}

# Τι «πιστεύει» το φίλτρο για την ταχύτητα και το γυροσκόπιο. Βγαίνουν από τους ΑΙΣΘΗΤΗΡΕΣ, όχι από ρύθμιση σε
# μια διαδρομή, γι' αυτό είναι ίδια για κάθε δέκτη και κάθε χάρτη: ο πραγματικός θόρυβος (πηγές πιο κάτω,
# στον θόρυβο του «κόσμου») επί 2, για περιθώριο ασφαλείας σε ό,τι δεν έχουμε προβλέψει.
SAFETY = 2
VELOCITY_SIGMA = SAFETY * 0.05                          # m/s   (wheel speed: Gao 2006)
TURN_RATE_SIGMA = SAFETY * np.radians(0.03 * np.sqrt(20))   # rad/s (γυροσκόπιο: MTi-10, ≈ 0.0023 → 0.0046)

np.random.seed(args.seed)   # ίδια «ζάρια» σε κάθε τρέξιμο, για να συγκρίνονται οι ρυθμίσεις δίκαια

data =np.genfromtxt(args.csv, delimiter=",", names=True)
# data = np.genfromtxt("drive_2026-09-25_15-07-08.csv", delimiter=",", names=True)
## data = np.genfromtxt("drive_2026-09-25_15-19-29.csv", delimiter=",", names=True)


N = 1000

def create_particles(gps_x, gps_y, pos_sigma, N):
    particles = np.zeros((N, 3))
    xs = np.random.normal(gps_x, pos_sigma, N)
    ys = np.random.normal(gps_y, pos_sigma, N)
    # Χωρίς πυξίδα: κάθε κατεύθυνση εξίσου πιθανή. Το φίλτρο βρίσκει τη σωστή όταν το αυτοκίνητο κινηθεί.
    yaws = np.random.uniform(0, 2 * np.pi, N)   # rad

    particles[:, 0] = xs
    particles[:,1] = ys
    particles[:, 2]=yaws
    return particles


def predict(particles, velocity, duration, velocity_sigma, turn_rate, turn_rate_sigma):
    particle_velocity = np.random.normal(velocity,  velocity_sigma, N)
    particle_turnrate = np.random.normal(turn_rate, turn_rate_sigma, N)
    
    dx = particle_velocity * duration * np.cos(particles[:, 2])
    dy = particle_velocity * duration * np.sin(particles[:, 2])

    particles[:, 0] = particles[:, 0] + dx
    particles[:, 1] = particles[:, 1] + dy
    particles[:, 2] = particles[:, 2] + particle_turnrate * duration
    return particles


def update(particles, gps_x, gps_y, gps_sigma):
    diff_x = particles[:, 0] - gps_x
    diff_y = particles[:, 1] - gps_y
    distance = np.sqrt(diff_x**2 + diff_y**2)
    weights = np.exp(-(distance**2) / (2*gps_sigma**2))
    weights = weights/np.sum(weights)
    return weights



def resample(particles, weights):
    indexes = multinomial_resample(weights)
    particles = particles[indexes]
    weights = np.ones(N)/N
    return particles, weights


def estimate(particles):
    est_x = np.mean(particles[:, 0])
    est_y = np.mean(particles[:, 1])
    mean_cos = np.mean(np.cos(particles[:, 2]))
    mean_sin = np.mean(np.sin(particles[:, 2]))
    est_yaw = np.arctan2(mean_sin, mean_cos)
    return est_x, est_y, est_yaw



pos_sigma = SETTINGS[args.receiver]["pos_sigma"]
velocity_sigma = VELOCITY_SIGMA
turn_rate_sigma = TURN_RATE_SIGMA
gps_sigma = SETTINGS[args.receiver]["gps_sigma"]
if args.gps_sigma is not None:
    gps_sigma = args.gps_sigma
if args.velocity_sigma is not None:
    velocity_sigma = args.velocity_sigma
if args.turn_rate_sigma is not None:
    turn_rate_sigma = args.turn_rate_sigma
duration = 0.05

# Θόρυβος γυροσκοπίου: ο «κόσμος», ΟΧΙ το φίλτρο (το φίλτρο δεν τα ξέρει αυτά, έχει το δικό του turn_rate_sigma).
# Xsens MTi-10 (το IMU του UrbanNav), datasheet MTi 10-series σελ. 2:
#   noise density 0.03 °/s/√Hz      → τυχαίο λάθος σε κάθε μέτρηση στα 20 Hz: 0.03·√20 ≈ 0.13 °/s
#   in-run bias stability 18 °/h    → bias, ΣΤΑΘΕΡΟ σε όλη τη διαδρομή (απλοποίηση)
# Το αρχικό bias (0.2 °/s) θεωρούμε ότι αφαιρέθηκε με βαθμονόμηση, με το αυτοκίνητο σταματημένο πριν ξεκινήσει.
gyro_noise_sigma = np.radians(0.03 * np.sqrt(20))   # rad/s
gyro_bias_sigma = np.radians(18 / 3600)             # rad/s (18 μοίρες την ώρα → ανά δευτερόλεπτο)
gyro_bias = np.random.normal(0, gyro_bias_sigma)    # μία φορά, για όλη τη διαδρομή

# Θόρυβος ταχύτητας (αισθητήρες τροχών): επίσης ο «κόσμος», ΟΧΙ το φίλτρο (το φίλτρο έχει το δικό του velocity_sigma).
# Gao, Petovello & Cannon 2006 (ION NTM), Πίνακας 2 σελ. 7: ακρίβεια wheel speed sensor 5 cm/s,
# μετρημένη σε πραγματικό αυτοκίνητο, με καταγραφή στα 20 Hz (όπως εμείς).
# Όταν το αυτοκίνητο στέκεται, ο τροχός δεν γυρίζει, άρα η μέτρηση είναι ακριβώς 0.
speed_noise_sigma = 0.05    # m/s
stopped_speed = 0.01        # m/s: κάτω από αυτό θεωρούμε ότι στέκεται (το CARLA τότε δίνει < 0.0001, όχι ακριβώς 0)

estimates = []
truths = []
gps_fixes = []

particles = None

receiver = GpsNoise(args.receiver, seed=args.seed)   # ο θόρυβος GPS έχει δικά του «ζάρια», με το ίδιο seed

view = None   # το παράθυρο του --show
if args.show:
    from pf_view import open_view, update_view
    view = open_view(args.receiver)
last_fix, last_fix_time = None, 0.0   # η τελευταία μέτρηση GPS και πότε ήρθε (για το --show)

for i in range(len(data)):
    velocity = data["vel"][i]
    gt_x = data["gt_x"][i]
    gt_y = data["gt_y"][i]
    gt_yaw = data["gt_yaw"][i]
    turn_rate = data["gyro_z"][i] + gyro_bias + np.random.normal(0, gyro_noise_sigma)   # μετρημένο, όχι τέλειο

    # Μετρημένη ταχύτητα (για το φίλτρο). Η αληθινή velocity μένει για τον θόρυβο του GPS, που είναι ο «κόσμος».
    if velocity < stopped_speed:
        measured_velocity = 0
    else:
        measured_velocity = velocity + np.random.normal(0, speed_noise_sigma)

    if i % 20 == 0:
        fix = receiver.step(gt_x, gt_y, gt_yaw, velocity)
    else:
        fix = None

    if particles is None:
        if fix is not None:
            particles = create_particles(fix[0], fix[1], pos_sigma, N)
        continue
    particles = predict(particles, measured_velocity, duration, velocity_sigma, turn_rate, turn_rate_sigma)
    if fix is not None:
        gps_fixes.append((fix[0], fix[1], gt_x, gt_y))
        weights = update(particles, fix[0], fix[1], gps_sigma)
        particles, weights = resample(particles, weights)
    est_x, est_y, est_yaw = estimate(particles)
    estimates.append((est_x, est_y, est_yaw))
    truths.append((gt_x, gt_y, np.radians(gt_yaw)))

    time = data["sim_time"][i] - data["sim_time"][0]   # s από την αρχή της καταγραφής
    if fix is not None:
        last_fix, last_fix_time = fix, time
    if view is not None and time >= args.start * 60:
        if not update_view(view, particles, (gt_x, gt_y), (est_x, est_y), last_fix, last_fix_time, time):
            view = None   # έκλεισες το παράθυρο: το φίλτρο συνεχίζει χωρίς ζωγραφική

estimates = np.array(estimates)
truths = np.array(truths)
gps_fixes = np.array(gps_fixes)

#---------filter error------------

diff_x = estimates[:, 0] - truths[:, 0]
diff_y = estimates[:, 1] - truths[:, 1]
filter_error = np.sqrt(diff_x**2 + diff_y**2)

#----------gps error---------------

diff_x = gps_fixes[:, 0] - gps_fixes[:, 2]
diff_y = gps_fixes[:, 1] - gps_fixes[:, 3]
gps_error = np.sqrt(diff_x**2 + diff_y**2)

#----------print--------------------
print("δέκτης:", args.receiver)
print("φίλτρο: μέσο", round(np.mean(filter_error), 2), "m, μέγιστο", round(np.max(filter_error), 2), "m")
print("gps: μέσο", round(np.mean(gps_error), 2), "m, μέγιστο", round(np.max(gps_error), 2), "m")


# particles = create_particles(10, 20, 3, N)
# particles = predict(particles, 5, 1, 0.5, 0.1, 0.02)
# weights = update(particles, 14, 24, 3)
# particles, weights =resample(particles, weights)
# est_x, est_y, est_yaw = estimate(particles)
# print(est_x, est_y, np.degrees(est_yaw))

