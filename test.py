from time import time_ns

import virxrlru as rlru

print(rlru.__doc__)

print("Loading soccar...")
rlru.load_soccar()

print("Setting tick...")
time = 0.
times = []

for _ in range(10000):
    start = time_ns()
    rlru.tick(
        time=time,
        ball={
            "location": [
                0,
                0,
                0,
            ],
            "velocity": [
                0,
                0,
                0,
            ],
            "angular_velocity": [
                0,
                0,
                0,
            ],
        },
        car={
            "location": [
                0,
                0,
                100,
            ],
            "velocity": [
                0,
                0,
                0,
            ],
            "angular_velocity": [
                0,
                0,
                0,
            ],
            "hitbox": [
                0,
                0,
                0,
            ],
            "pitch": 0,
            "yaw": 0,
            "roll": 0,
        }
    )

    times.append(time_ns() - start)
    time += 1/120

print(f"Total test time: {round(sum(times) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times) / len(times) / 1000000, 3)}ms")

print("Calculating intercept via linear search...")
times = []

for _ in range(10000):
    start = time_ns()

    rlru.calculate_intercept([
        0, 5120, 0
    ])

    times.append(time_ns() - start)

print(f"Total test time: {round(sum(times) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times) / len(times) / 1000000, 3)}ms")

print("Calculating intercept via binary search...")
rlru.use_binary_search(True)
times = []

for _ in range(10000):
    start = time_ns()

    rlru.calculate_intercept([
        0, 5120, 0
    ])

    times.append(time_ns() - start)

print(f"Total test time: {round(sum(times) / 1000000000, 4)}s")
print(f"Avg. time of execution: {round(sum(times) / len(times) / 1000000, 3)}ms")
