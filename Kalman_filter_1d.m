clear all
% Simulation parameters
rng(0);
num_steps = 100;  % Number of time steps
dt = 0.1;  % Time step
true_acceleration = sin(linspace(0, 10, num_steps));  % True acceleration of the object
measurement_noise_std = 1.0;  % Standard deviation of measurement noise

% True initial state
true_state = [0; 0];

% Generate true state and noisy measurements
true_states = [true_state];
measurements = [true_state(1) + randn * measurement_noise_std];
for i = 1:num_steps
    true_state = [1 dt; 0 1] * true_state + [0.5*dt^2; dt] * true_acceleration(i);
    true_states = [true_states true_state];
    measurements = [measurements true_state(1) + randn * measurement_noise_std];
end

% Kalman Filter initialization
x = [0; 0];  % Initial state estimate: [position; velocity]
P = eye(2);  % Initial state covariance
A = [1 dt; 0 1];  % State transition matrix
H = [1 0];  % Measurement matrix
Q = diag([0.01, 0.01]);  % Process noise covariance
R = measurement_noise_std^2;  % Measurement noise covariance

% Kalman Filter loop
estimated_states = zeros(2, num_steps+1);
for i = 1:num_steps+1
    % Prediction
    x = A * x;
    P = A * P * A' + Q;

    % Measurement update
    y = measurements(i) - H * x;
    S = H * P * H' + R;
    K = P * H' / S;
    x = x + K * y;
    P = P - K * H * P;

    estimated_states(:, i) = x;
end

% Plot true position, measurements, and estimated position
figure('Position', [100, 100, 800, 400]); % [left, bottom, width, height]
plot(0:num_steps, true_states(1, :), 'b-', 'DisplayName', 'True Position', 'LineWidth', 1);
hold on;
plot(0:num_steps, measurements, 'r-', 'DisplayName', 'Measurements', 'LineWidth', 1);
plot(0:num_steps, estimated_states(1, :), 'k-', 'DisplayName', 'Estimated Position', 'LineWidth', 1);  
hold off;
xlabel('Time Step');
ylabel('Position');
legend('show');
title('True Position, Measurements, and Estimated Position');
