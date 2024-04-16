import numpy as np
import matplotlib.pyplot as plt


class LinearSystem:
    def __init__(self, A, B, H, x0, Q, R, P0):
        """
        Initialize the linear system with a Kalman filter.
        A: System dynamics matrix (2x2)
        B: Control input matrix (2x2)
        C: Measurement matrix (2x2)
        x0: Initial state (2x1)
        Q: Process noise covariance matrix (2x2)
        R: Measurement noise covariance
        P0: Initial estimate error covariance matrix (2x2)
        """
        self.A = A
        self.B = B
        self.H = H
        self.state = x0
        self.Q = Q
        self.R = R
        self.P = P0  # Initial error covariance
        self.state_estimate = x0

    def evolve(self, u):
        """
        Evolve the state of the system based on the control input.
        """
        self.state = self.A @ self.state + self.B @ u

    def take_measurement(self):
        """
        Simulate a measurement based on the current state.
        """
        measurement_noise = np.random.normal(0, self.R)
        return self.H @ self.state + measurement_noise


    def estimate_state(self, measurement, u):
        """
        Update the state estimate based on the measurement.
        """
        # Step 1: Prior update/Prediction step
        state_pred = self.A @ self.state_estimate + u
        P_p = self.A @ self.P @ self.A.T + self.Q

        # Step 2: A posteriori update
        self.P = np.linalg.inv(np.linalg.inv(P_p) + self.H.T * (1/self.R) @ self.H)
        self.state_estimate = state_pred + self.P @ self.H.T * (1/self.R) @ (measurement - self.H @ state_pred)
        return self.state_estimate


# Define the system parameters
A = np.array([[1, 0.1], [0, 1]])  # System dynamics matrix
B = np.array([[0.5, 0], [0, 1]])  # Control input matrix
C = np.array([[1, 0], [0, 1]])  # Measurement matrix
x0 = np.array([[0], [0]])  # Initial state
Q = np.array([[0.00025, 0.0005], [0.0005, 0.0001]])  # Process noise covariance
R = 1  # Measurement noise covariance
P0 = 5*np.eye(2)  # Initial error covariance matrix

system = LinearSystem(A, B, C, x0, Q, R, P0)
u = np.array([[0.015], [0.1]])  # Constant control input

# Simulation loop over 1000 steps
true_states = []
measurements = []
estimates = []

for _ in range(100):
    system.evolve(u)

    measurement = system.take_measurement()
    measurements.append(measurement.flatten())

    true_states.append(system.state.flatten())

    estimate = system.estimate_state(measurement, u)
    estimates.append(estimate.flatten())


# Convert to numpy arrays for plotting
true_states = np.array(true_states)
estimates = np.array(estimates)
measurements = np.array(measurements)

# Plotting the results
plt.figure(figsize=(10, 5))
plt.plot(true_states[:, 0], label='True State x1')
plt.plot(estimates[:, 0], label='Estimated State x1')
plt.plot(measurements[:, 0], 'x', label='Measurements x1', markersize=2)
plt.title('State and Measurements over Time')
plt.xlabel('Time Step')
plt.ylabel('State Value')
plt.legend()
plt.show()
