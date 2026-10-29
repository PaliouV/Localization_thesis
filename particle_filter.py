import numpy as np
from filterpy.monte_carlo import multinomial_resample
from gps_noise_v2 import GpsNoise

data = np.genfromtxt("drive_2026-10-21_13-54-17.csv", delimiter=",", names=True)
# data = np.genfromtxt("drive_2026-09-25_15-07-08.csv", delimiter=",", names=True)
## data = np.genfromtxt("drive_2026-09-25_15-19-29.csv", delimiter=",", names=True)


N = 1000

def create_particles(gps_x, gps_y, yaw_deg, pos_sigma, yaw_sigma_deg, N):
    particles = np.zeros((N, 3))
    xs = np.random.normal(gps_x, pos_sigma, N)
    ys = np.random.normal(gps_y, pos_sigma, N)
    yaws = np.random.normal(yaw_deg, yaw_sigma_deg, N)
    yaws = np.radians(yaws)

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



pos_sigma = 3
yaw_sigma_deg = 5
velocity_sigma = 0.5
turn_rate_sigma = 0.02
gps_sigma = 3
duration = 0.05

estimates = []
truths = []
gps_fixes = []

particles = None

f9p = GpsNoise("f9p", 1)
# m8t = GpsNoise("m8t", 1)

for i in range(len(data)):
    velocity = data["vel"][i]
    gt_x = data["gt_x"][i]
    gt_y = data["gt_y"][i]
    gt_yaw = data["gt_yaw"][i]
    turn_rate = data["gyro_z"][i]
    compass = data["compass"][i]

    if i % 20 == 0:
        fix = f9p.step(gt_x, gt_y, gt_yaw, velocity)
    else:
        fix = None

    if particles is None:
        if fix is not None:
            yaw_deg = np.degrees(compass) - 90
            particles = create_particles(fix[0], fix[1], yaw_deg, pos_sigma, yaw_sigma_deg, N)
        continue
    particles = predict(particles, velocity, duration, velocity_sigma, turn_rate, turn_rate_sigma)
    if fix is not None:
        weights = update(particles, fix[0], fix[1], gps_sigma)
        particles, weights = resample(particles, weights)
    est_x, est_y, est_yaw = estimate(particles)
    estimates.append((est_x, est_y, est_yaw))
    truths.append((gt_x, gt_y, np.radians(gt_yaw)))


# particles = create_particles(10, 20, 45, 3, 5, N)
# particles = predict(particles, 5, 1, 0.5, 0.1, 0.02)
# weights = update(particles, 14, 24, 3)
# particles, weights =resample(particles, weights)
# est_x, est_y, est_yaw = estimate(particles)
# print(est_x, est_y, np.degrees(est_yaw))

