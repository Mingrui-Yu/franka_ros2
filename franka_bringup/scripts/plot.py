import json
import matplotlib.pyplot as plt
import numpy as np

# Step 1: Read the JSON file
file = "/home/robotics/franka_emika_panda/ws_ros2/data.json"
with open(file, 'r') as file:
    data = json.load(file)

# Step 2: Extract the data
t_interp = data['t_interp']
q = np.array(data['q'])
q_goal = np.array(data['q_goal'])

# Step 3: Plot the data
plt.plot(t_interp, label="t_interp")
plt.plot(q[:, 6], label="q")
plt.plot(q_goal[:, 6], label="q_goal")
plt.xlabel('Time')
plt.ylabel('Value')
plt.title('Plot from JSON Data')
plt.legend()
plt.grid(True)
plt.show()