# Bayes-Filter
# 🧠 Ball Tracking using Bayes Filter

This project simulates the motion of a ball thrown from a boundary line toward a goalkeeper and estimates its **true position and velocity** using a **Bayes filter**. The simulation considers real-world challenges such as **air resistance**, **gravity**, **sensor noise**, and **missing observations**.

---

## 🎯 Objectives

- Simulate the 2D motion of a ball under the influence of gravity and air drag.
- Create noisy and missing sensor measurements of the ball's position.
- Estimate the ball’s true position and velocity using a Bayes filter.
- Visualize and compare the true trajectory, noisy observations, and filtered estimates.

---

## 🧪 Assumptions

- Motion is in 2D space: \( (x, y) \).
- Time is discrete with constant interval \( \Delta t \).
- The ball is affected by:
  - **Gravity**
  - **Linear air resistance**
- Sensor measurements are:
  - Noisy (Gaussian noise)
  - Occasionally missing
- Both motion and measurement models include Gaussian noise.

---

## 🧠 Concepts Used

- Bayes Filter (Discrete Probabilistic Estimation)
- Gaussian Noise (Process and Measurement)
- Air Drag and Gravity Physics
- Handling Missing Data
- Python Simulation and Plotting

---

## 🔢 State Vector

The state of the ball is represented by the vector:

- `x`, `y`: position (in meters)
- `vx`, `vy`: velocity components (in m/s)

---

## 🧮 Formula Summary

### 1. Motion Model (with gravity and air drag)

\[
v_x \leftarrow v_x + (-k \cdot v_x) \cdot \Delta t
\]
\[
v_y \leftarrow v_y + (-g - k \cdot v_y) \cdot \Delta t
\]
\[
x \leftarrow x + v_x \cdot \Delta t
\]
\[
y \leftarrow y + v_y \cdot \Delta t
\]

### 2. Observation Model (noisy sensor readings)

\[
z_x = x + \mathcal{N}(0, \sigma_{\text{meas}}^2)
\quad , \quad
z_y = y + \mathcal{N}(0, \sigma_{\text{meas}}^2)
\]

### 3. Process Noise (for unpredictability)

\[
v_x = v_x + \mathcal{N}(0, \sigma_{\text{process}}^2)
\quad , \quad
v_y = v_y + \mathcal{N}(0, \sigma_{\text{process}}^2)
\]

### 4. Bayes Filter Update Rule

\[
\text{Bel}(x_t) = \eta \cdot P(z_t \mid x_t) \cdot \int P(x_t \mid x_{t-1}) \cdot \text{Bel}(x_{t-1}) \, dx_{t-1}
\]

### 5. Likelihood Function (Gaussian PDF)

\[
P(z \mid x) = \frac{1}{\sqrt{2\pi\sigma^2}} \exp\left( -\frac{(z - x)^2}{2\sigma^2} \right)
\]

---

## 📁 File Structure

| File | Description |
|------|-------------|
| `bayes_filter_simulation.ipynb` | Jupyter notebook containing the simulation and filter implementation |
| `README.md` | This documentation |
| `trajectory_plot.png` | Sample output showing ball path, observations, and estimates |

---

## ⚙️ Parameters and Setup

```python
# Constants
dt = 0.1                 # Time step (s)
g = 9.81                 # Gravity (m/s^2)
k = 0.1                  # Air drag coefficient
missing_rate = 0.2       # 20% probability that a sensor value is missing

# Noise
process_noise_std = 0.5
measurement_noise_std = 2.0

# Initial state
initial_state = np.array([0.0, 0.0, 20.0, 30.0])  # [x, y, vx, vy]


▶️ How to Run
Clone the repository or open the notebook in Google Colab.

Install required libraries:

bash
Copy
Edit
pip install numpy matplotlib
Run the notebook cells in sequence:

Simulates the motion.

Generates noisy and missing observations.

Applies Bayes filter.

Plots results.

📈 Results
The notebook will generate:

True trajectory of the ball.

Sensor observations (with noise and missing values).

Estimated trajectory using Bayes filtering.

A visual comparison of the true, observed, and estimated paths shows the effectiveness of the filter even with data loss and noise.

✅ Conclusion
This project demonstrates how a Bayes filter can be applied to estimate the state of a system under uncertainty. Despite missing and noisy sensor data, the filter effectively reconstructs the ball’s trajectory by combining prior predictions with available observations. This has applications in:

Robotics and autonomous systems

Object tracking and localization

Signal processing and sensor fusion

🔍 References
Probabilistic Robotics – Sebastian Thrun, Wolfram Burgard, Dieter Fox

Wikipedia – Bayes Filter

Kalman and Bayesian Filters in Python – Roger R. Labbe Jr.

👨‍💻 Author
Anirudh pal
 Project – Ball Tracking using Bayes Filter
Department of ECE(IOT), IIIT NAGPUR











