%% KUKA iiwa 7DOF manipulation trajectory planning (improved: joint-space smoothing)
% This script demonstrates a safer approach than pointwise IK on a dense Cartesian
% interpolation: solve IK ONLY at sparse waypoints, then generate a smooth joint
% trajectory between those joint waypoints.
%
% Requirements:
%  - MATLAB + Robotics System Toolbox
%  - A KUKA iiwa7 model available either via loadrobot("kukaIiwa7") or a local URDF
%
% Outputs:
%  - data/trajectory_smooth_*.txt (Nx7 joint angles)
%  - Joint angle/velocity plots

clear; clc; close all;

%% 1) Load robot model (prefer built-in model when available)
try
    robot = loadrobot("kukaIiwa7","DataFormat","row");
catch
    % Fallback: place iiwa7.urdf in the project root or update this path
    robot = importrobot("iiwa7.urdf");
    robot.DataFormat = "row";
end
eeName = robot.BodyNames{end};

%% 2) Fixed end-effector orientation (Z-down, X-forward)
yawAlign = -pi/2;
Rfixed   = eul2rotm([yawAlign, 0, pi], 'ZYX');

%% 3) Tool/shape offsets and calibration transforms (from report; replace with your values)
shapeOffset = [12.5, -40.5, -15]/1000; 
T_tcp_shape = trvec2tform(shapeOffset);

Tec = [ 0  1 0     0;
       -1  0 0 -0.0662;
        0  0 1  0.0431;
        0  0 0     1 ];

R_ca = eul2rotm(deg2rad([-37.6880, -4.9345, 178.5475]), 'ZYX');
t_ca = [-0.019795; -0.0331015; 0.6206805];
Tca  = [R_ca, t_ca; 0 0 0 1];

qc_photo = deg2rad([55.38, 9.28, -138.54, 84.24, -6.20, -82.98, 141.00]);

T_base_tcp_c = getTransform(robot, qc_photo, eeName);
T_base_cam   = T_base_tcp_c / Tec;
T_base_marker= T_base_cam   / Tca;

%% 4) Define marker grid in base frame
mw   = 107.95/1000;
hgap = 50/1000;
dx   = 70/1000;
dy   = 73/1000;

markerOffsets = [
  -dx/2, -(mw/2 + hgap + dy/2), 0;
   dx/2, -(mw/2 + hgap + dy/2), 0;
   dx/2, -(mw/2 + hgap - dy/2), 0;
  -dx/2, -(mw/2 + hgap - dy/2), 0
];

blockThickness = abs(shapeOffset(3));
p0 = T_base_marker(1:3,4) + markerOffsets(1,:)';
tableZ = p0(3) - blockThickness;
safeZ  = tableZ + 0.12;

%% 5) Build 9 Cartesian waypoints (safe-Z approach)
pos_list = zeros(3,9);
tcpPos_photo  = T_base_tcp_c(1:3,4);
pos_list(:,1) = tcpPos_photo + shapeOffset';

k = 2;
for i = 1:4
    p_m = T_base_marker(1:3,4) + markerOffsets(i,:)';
    pos_list(:,k)   = [p_m(1:2); safeZ];
    pos_list(:,k+1) = [p_m(1:2); p_m(3)];
    k = k + 2;
end

%% 6) IK ONLY at the sparse waypoints (reduces branch-jumps)
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.1 0.1 0.1 1 1 1];

n_wp = size(pos_list,2);
q_wp = zeros(7,n_wp);
q_prev = qc_photo;

for i = 1:n_wp
    T_b_shape   = [Rfixed, pos_list(:,i); 0 0 0 1];
    T_targetTCP = T_b_shape * T_tcp_shape;

    % Try multiple initial guesses to avoid sudden branch changes
    guesses = [q_prev; qc_photo; q_prev + deg2rad(2)*randn(1,7)];
    bestQ = [];
    bestCost = inf;

    for g = 1:size(guesses,1)
        [qSol,~] = ik(eeName, T_targetTCP, weights, guesses(g,:));
        cost = norm(wrapToPi(qSol(:) - q_prev(:)));
        if cost < bestCost
            bestCost = cost;
            bestQ = qSol(:);
        end
    end

    q_wp(:,i) = bestQ;
    q_prev = bestQ.';
end

%% 7) Smooth joint-space trajectory through those joint waypoints
t_total = 20;
dt_cmd  = 0.005;
t_wp    = linspace(0, t_total, n_wp);
t_cmd   = (0:dt_cmd:t_total);

% Cubic polynomial trajectory with zero end-velocities
[q_cmd, qd_cmd, qdd_cmd] = cubicpolytraj(q_wp, t_wp, t_cmd, ...
    'VelocityBoundaryCondition', zeros(7,2));

%% 8) Export
outname = fullfile("data", "trajectory_smooth_4001x7.txt");
if ~exist("data","dir"), mkdir("data"); end
fid = fopen(outname,'w');
for i = 1:numel(t_cmd)
    fprintf(fid, '%.8f %.8f %.8f %.8f %.8f %.8f %.8f\n', q_cmd(:,i));
end
fclose(fid);
fprintf("Saved %d samples to %s\n", numel(t_cmd), outname);

%% 9) Plots
figure;
subplot(3,1,1); plot(t_cmd, q_cmd'); title("Joint Angles (smooth)"); xlabel("Time [s]"); ylabel("rad"); grid on;
subplot(3,1,2); plot(t_cmd, qd_cmd'); title("Joint Velocities (smooth)"); xlabel("Time [s]"); ylabel("rad/s"); grid on;
subplot(3,1,3); plot(t_cmd, qdd_cmd'); title("Joint Accelerations (smooth)"); xlabel("Time [s]"); ylabel("rad/s^2"); grid on;
