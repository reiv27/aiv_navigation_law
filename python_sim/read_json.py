import json
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({
    'font.size': 18,
    'axes.titlesize': 18,
    'axes.labelsize': 20,
    'xtick.labelsize': 18,
    'ytick.labelsize': 18,
    'legend.fontsize': 12,
    'figure.titlesize': 20,
})

with open('robot_data_dynamic_2obs_noise_1.json', 'r', encoding='utf-8') as json_file:
    data = json.load(json_file)
# time_end = data["t"]
time_end = 100.0
dR = data["dR"]
zeros = [0]*10000
time = np.arange(0, 100, 0.01)

print(f"mean={np.mean(dR[1000:])}")

fig, ax = plt.subplots(figsize=(12, 4))
ax.plot(time, dR, 'black')
ax.plot(time, zeros, 'b--')
ax.set_title(r"Distance $d_R$ from the robot to the equidistant curve")
ax.set_xlabel(r"time, s")
ax.set_ylabel(r"$d_R$, m")
ax.set_xlim(0, 100)
# ax.set_ylim(, )
ax.grid(True)

info_text = (
    r"Simulation params:" "\n"
    r"--------------------" "\n"
    r"dt = 1 s" "\n"
    r"Goal $d$: 10 m" "\n"
    r"Robot velocities: $|v|=0.1$ m/s, $\omega = 0.06$ rad/s" "\n"
    r"Obstacle velocities: $v_x=-0.01$ m/s, $v_y=0.01$ m/s, $\omega = 0.001$ rad/s" "\n"
)

# fig.text(0.42, 0.65, info_text, fontsize=20, bbox=dict(facecolor='white', alpha=0.8))
plt.tight_layout()
plt.show()