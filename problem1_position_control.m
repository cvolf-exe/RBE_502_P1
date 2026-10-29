clc, clear all, close all;

%% Load rigid body tree
robot = createRigidBodyTree;
robot.DataFormat = "column";
robot.Gravity = [0; 0; -9.81];

%% Controller setup
eeName = "end_effector";

%% Time Variables
dt = 0.05;
Tfinal = 5;
t = 0:dt:Tfinal;

%% The URDF has 6 movable joints: 4 arm joints + 2 gripper joints.
q = [0; 0; 0; 0];
qFull = homeConfiguration(robot)
qFull(:) = 0;

%% The Joints we can control for position control
armJointIdx = 1:4;

%% Geometric parameters
params.a0 = 0.012;
params.d1 = 0.0595;
params.a1 = 0.024;
params.d2 = 0.128;
params.a2 = 0.124;
params.ell_e = 0.12;

%% Desired Location of the end effector
x_des = [0.20; 0.20; 0.15];    
Kp = 5.0;

%% Symbolic Jacobian 
syms q1 q2 q3 q4
qSym = [q1; q2; q3; q4]; 
xSim = openManipulatorPosition(qSym, params); 
JSym = jacobian(xSim, qSym);
% Convert to a numeric function
J = matlabFunction(JSym, "Vars", {qSym});

qHist = zeros(numel(q), length(t));
qdotHist = zeros(numel(q), length(t));
xHist = zeros(3, length(t));
errHist = zeros(3, length(t));

%% Simulate differential inverse kinematics controller
figure;
for k = 1:length(t)
    %% Forward Kinematic
    x = openManipulatorPosition(q, params);
    fprintf("X: %d\n", x); 

    %% Save data
    qHist(:, k) = q;
    xHist(:, k) = x;
    errHist(:, k) = norm(x_des - x);

    %% Control Law
    qdot = controlLaw(J, Kp, x, x_des, q);
    qdotHist(:,k)= qdot;

    %% Integration of the velocities
    q = q + dt*qdot;

    %% Update joints
    qFull(armJointIdx) = q;
    show(robot, qFull, "PreservePlot", false, "Frames", "on");
    hold on;
    plot3(x_des(1), x_des(2), x_des(3), "ro", "MarkerSize", 8, "LineWidth", 2);
    hold off;
    axis([-0.1 0.4 -0.3 0.3 0 0.4]);
    grid on;
    drawnow;
end

%% Plot the required results
% 1. Angular Displacement of each joint 
figure()
plot(t, qHist)
title("Angular Displacement of Each Joint")
xlabel("Time")
ylabel("Joint Angle Displacement")
legend('q0', 'q1', 'q3', 'q4')
grid on
    
% 2. Angular Velocity of each joint 
figure()
plot(t, qdotHist)
title("Angular Velocity of Each Joint")
xlabel("Time")
ylabel("Joint Angle Velocity")
legend('q0', 'q1', 'q3', 'q4')
grid on

% 3. Evolution of the EE Cartesians with the desired position 
figure()
tiledlayout(3,1)
for i = 1:3
    nexttile;
    plot(t, xHist(i, :));
    hold on;
    yline(x_des(i), "r--");
    hold off;
    title(sprintf("EE %c Position over Time", char('x' + i - 1)));
    xlabel("Time");
    ylabel("Position");
    grid on;
end

% 4. Norm of the position error
figure()
plot(t, errHist)
title("Error Norm")
xlabel("Time")
ylabel("Error Norm")
grid on

function x = openManipulatorPosition(q, params)
    % Returns the current EE position of the manipulator given
    % the current joint position vector q

    % Shorthand definitions
    c1 = cos(q(1)); 
    c2 = cos(q(2));
    s1 = sin(q(1));
    s2 = sin(q(2));

    c23  = cos(q(2) + q(3)); 
    s23 = sin(q(2) + q(3)); 

    c234 = cos(q(2)+ q(3)+ q(4)); 
    s234 = sin(q(2)+ q(3)+ q(4)); 

    % EE Position matrix columns according to the assignment pdf
    p1 = params.a0 + c1*(params.a1*c2 + params.d2*s2 + params.a2*c23 + params.ell_e*c234); 
    p2 = s1*(params.a1*c2 + params.d2*s2+params.a2*c23+params.ell_e*c234); 
    p3 = params.d1 - params.a1*s2 + params.d2*c2 - params.a2*s23 - params.ell_e*s234;

    x = [p1; p2; p3];
    

end

function q_dot = controlLaw(J, K, x, x_des, q)
    % Evaluate the jacobian at the current joint positions
    J_at_q = J(q);
    
    % Compute the error
    error = x_des - x;

    % Return the joint velocity commands
    q_dot = pinv(J_at_q) * K * error;

end