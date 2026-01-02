clear; clc; close all;

robot = importrobot('iiwa7.urdf');
robot.DataFormat = 'row';
eeName = robot.BodyNames{end};

yawAlign = -pi/2;
Rfixed   = eul2rotm([yawAlign, 0, pi], 'ZYX');

shapeOffset = [12.5, -40.5, -15]/1000; 
T_tcp_shape = trvec2tform(shapeOffset);

Tec = [ 0  1 0     0;
       -1  0 0 -0.0662;
        0  0 1  0.0431;
        0  0 0     1 ];

R_ca = eul2rotm(deg2rad([-37.6880, -4.9345, 178.5475]), 'ZYX');
t_ca = [-0.019795; -0.0331015; 0.6206805];
Tca  = [R_ca, t_ca; 0 0 0 1];

qc_photo     = deg2rad([55.38, 9.28, -138.54, 84.24, -6.20, -82.98, 141.00]);
T_base_tcp_c = getTransform(robot, qc_photo, eeName);
T_base_cam   = T_base_tcp_c / Tec;
T_base_marker= T_base_cam   / Tca;

disp('T_base_marker =');
disp(T_base_marker);

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

t_total = 20;                
dt_cmd  = 0.005;             
t_wp    = linspace(0, t_total, size(pos_list,2));
t_cmd   = (0:dt_cmd:t_total)';

x_cmd = interp1(t_wp, pos_list(1,:), t_cmd)';
y_cmd = interp1(t_wp, pos_list(2,:), t_cmd)';
z_cmd = interp1(t_wp, pos_list(3,:), t_cmd)';

ik      = inverseKinematics('RigidBodyTree', robot);
weights = [0.1 0.1 0.1 1 1 1];
q_cmd   = zeros(7, numel(t_cmd));
q_prev  = qc_photo;

for idx = 1:numel(t_cmd)
    T_b_shape   = [Rfixed, [x_cmd(idx); y_cmd(idx); z_cmd(idx)]; 0 0 0 1];
    T_targetTCP = T_b_shape * T_tcp_shape;
    [qSol,~]    = ik(eeName, T_targetTCP, weights, q_prev);
    q_cmd(:,idx)= qSol;
    q_prev      = qSol;
end

simSpeedFactor = 10;    
dt_frame       = 0.02;  
frameStep      = max(1, round((dt_frame * simSpeedFactor)/dt_cmd));

figure('Position',[200 200 800 600]);
ax = show(robot, qc_photo, 'PreservePlot', false);
view(120,20); grid on; hold on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

plot3( (T_base_marker(1,4) + markerOffsets(:,1)), ...
       (T_base_marker(2,4) + markerOffsets(:,2)), ...
        tableZ*ones(4,1), 'ro','LineWidth',2);

for k = 1:frameStep:length(t_cmd)
    show(robot, q_cmd(:,k)', 'Frames','off','PreservePlot', false, 'Parent', ax);
    title(sprintf('t = %.2f s (playback %.1fx)', t_cmd(k), simSpeedFactor));
    drawnow limitrate;
end

fname = 'singareddy rajeev.txt';
fid   = fopen(fname,'w');
for i = 1:numel(t_cmd)
    fprintf(fid, '%.8f %.8f %.8f %.8f %.8f %.8f %.8f\n', q_cmd(:,i));
end
fclose(fid);
fprintf('Saved %d samples to %s\n', numel(t_cmd), fname);

qd_cmd = diff(q_cmd,1,2)/dt_cmd;     
qd_cmd = [qd_cmd, zeros(7,1)];       

figure;
subplot(2,1,1);
plot(t_cmd, q_cmd');
title('Joint Angles (5 ms)');
xlabel('Time [s]'); ylabel('rad');
legend('q1','q2','q3','q4','q5','q6','q7','Location','best');
grid on;

subplot(2,1,2);
plot(t_cmd, qd_cmd');
title('Joint Velocities (5 ms)');
xlabel('Time [s]'); ylabel('rad/s');
legend('q1','q2','q3','q4','q5','q6','q7','Location','best');
grid on;
