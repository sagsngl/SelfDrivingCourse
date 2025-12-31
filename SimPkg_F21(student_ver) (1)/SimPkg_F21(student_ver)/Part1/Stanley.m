%% ================================================================
%  ROB535 Control Project - Part 1 (Tuned Stanley Controller)
%  Author: Your Name
% ================================================================

clear; close all;

load('TestTrack.mat');   % loads bl, br, cline, theta

dt = 0.01;
Tmax = 200;
N = round(Tmax/dt);

% Initial state
state = [287,5,-176,0,2,0];   % [X;u;Y;v;psi;r]

% Output control matrix
U = zeros(N,2);

%% ================================================================
% 1. Compute curvature along the centerline
% ================================================================
x = TestTrack.cline(1,:)';
y = TestTrack.cline(2,:)';

dx  = gradient(x);
dy  = gradient(y);
ddx = gradient(dx);
ddy = gradient(dy);

kappa = abs(dx .* ddy - dy .* ddx) ./ (dx.^2 + dy.^2).^(3/2);
kappa(isnan(kappa)) = 0;

%% ================================================================
% 2. Speed profile based on curvature
% ================================================================
max_speed = 35;          % top speed on straights
curv_gain = 450;         % how aggressively to slow in curves

v_des_profile = max_speed ./ sqrt(1 + curv_gain * kappa);

% Smooth speed profile
v_des_profile = smooth(v_des_profile, 10);

%% ================================================================
% 3. Controller gains
% ================================================================
k_e = 1.0;               % cross-track gain
k_h = 2.2;               % heading gain
Kp_speed = 550;          % longitudinal gain

% Steering smoothing
delta_prev = 0;

%% ================================================================
% 4. Simulation loop
% ================================================================
for k = 1:N

    X = state(k,1);
    Y = state(k,3);
    psi = state(k,5);
    u  = state(k,2);

    % ------------------------------------------------------------
    % Find nearest centerline point
    % ------------------------------------------------------------
    diffs = TestTrack.cline' - [X Y];
    [~, idx] = min(sum(diffs.^2,2));

    psi_ref = TestTrack.theta(idx);

    % ------------------------------------------------------------
    % Heading error
    % ------------------------------------------------------------
    e_h = wrapToPi(psi_ref - psi);

    % ------------------------------------------------------------
    % Cross-track error
    % ------------------------------------------------------------
    p_ref = TestTrack.cline(:,idx)';
    e_c = (-(p_ref(1)-X)*sin(psi) + (p_ref(2)-Y)*cos(psi));

    % ------------------------------------------------------------
    % Stanley steering
    % ------------------------------------------------------------
    delta_raw = k_h*e_h + atan2(k_e * e_c, u + 1e-3);

    % Smooth steering
    delta = 0.5*delta_prev + 0.5*delta_raw;
    delta_prev = delta;

    % Saturate
    delta = max(min(delta, 0.5), -0.5);

    % ------------------------------------------------------------
    % Desired speed from curvature
    % ------------------------------------------------------------
    v_des = v_des_profile(idx+2);

    % ------------------------------------------------------------
    % Speed control
    % ------------------------------------------------------------
    if v_des < u
        Fx = (Kp_speed+250) * (v_des - u);
    else
        Fx = (Kp_speed) * (v_des - u);
    end

    % Saturate
    Fx = max(min(Fx, 5000), -5000);

    % ------------------------------------------------------------
    % Store control
    % ------------------------------------------------------------
    U(k,:) = [delta, Fx];

    % ------------------------------------------------------------
    % Forward integrate one step
    % ------------------------------------------------------------
    dzdt = bike(state(k,:),U(k,:));
    sol = state(k,:) + dzdt'.*0.01;
    state(k+1,:) = sol;

    % ------------------------------------------------------------
    % Stop if finish line crossed
    % ------------------------------------------------------------
    if X > 1460
        fprintf('Track completed at t = %.2f sec\n', k*dt);
        break
    end
end

% Trim unused rows
ROB535_ControlProject_part1_input = U(1:k,:);

% Save
save('ROB535_ControlProject_part1_TeamXX_Stanley.mat', ...
     'ROB535_ControlProject_part1_input');

fprintf('Saved ROB535_ControlProject_part1_TeamXX.mat\n');
plotter(state,TestTrack);

function dzdt=bike(x,U)
%constants
Nw=2;
f=0.01;
Iz=2667;
a=1.35;
b=1.45;
By=0.27;
Cy=1.2;
Dy=0.7;
Ey=-1.6;
Shy=0;
Svy=0;
m=1400;
g=9.806;


%generate input functions
% delta_f=interp1(T,U(:,1),t,'previous','extrap');
% F_x=interp1(T,U(:,2),t,'previous','extrap');
delta_f = U(1);
F_x = U(2);

%slip angle functions in degrees
a_f=rad2deg(delta_f-atan2(x(4)+a*x(6),x(2)));
a_r=rad2deg(-atan2((x(4)-b*x(6)),x(2)));

%Nonlinear Tire Dynamics
phi_yf=(1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
phi_yr=(1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));

F_zf=b/(a+b)*m*g;
F_yf=F_zf*Dy*sin(Cy*atan(By*phi_yf))+Svy;

F_zr=a/(a+b)*m*g;
F_yr=F_zr*Dy*sin(Cy*atan(By*phi_yr))+Svy;

F_total=sqrt((Nw*F_x)^2+(F_yr^2));
F_max=0.7*m*g;

if F_total>F_max
    
    F_x=F_max/F_total*F_x;
  
    F_yr=F_max/F_total*F_yr;
end

%vehicle dynamics
dzdt= [x(2)*cos(x(5))-x(4)*sin(x(5));...
          (-f*m*g+Nw*F_x-F_yf*sin(delta_f))/m+x(4)*x(6);...
          x(2)*sin(x(5))+x(4)*cos(x(5));...
          (F_yf*cos(delta_f)+F_yr)/m-x(2)*x(6);...
          x(6);...
          (F_yf*a*cos(delta_f)-F_yr*b)/Iz];
end

function [] = plotter(sol,TestTrack)
X = sol(:,1);
Y = sol(:,3);
% targ_plot_X = targ_plot(:,1);
% targ_plot_Y = targ_plot(:,2);
psi = sol(:,5);

% Car shape (triangle)
car_length = 4;
car_width  = 2;

% Precompute car triangle in body frame
car_body = [ car_length/2,  0;
            -car_length/2,  car_width/2;
            -car_length/2, -car_width/2 ];

figure; hold on; axis equal;
title('Car Trajectory Animation');
xlabel('X (m)'); ylabel('Y (m)');

% Plot track
plot(TestTrack.bl(1,:), TestTrack.bl(2,:));
plot(TestTrack.br(1,:), TestTrack.br(2,:));
plot(TestTrack.cline(1,:), TestTrack.cline(2,:), 'r--');

% Plot full trajectory (faint)
plot(X, Y, 'b:');

% Animation settings
frame_skip = 20;     % draw every 2nd frame for speed
pause_time = 0.01;  % animation speed

car_patch = patch(NaN, NaN, 'b');  % placeholder for car shape
traj_line = plot(NaN, NaN, 'b', 'LineWidth', 2);

vel_text = text(X(1), Y(1), sprintf('Speed: %.1f m/s', sol(1)), ...
                'FontSize', 12, 'FontWeight', 'bold', 'Color', 'blue');


for k = 1:frame_skip:length(X)

    % Rotation matrix
    R = [cos(psi(k)) -sin(psi(k));
         sin(psi(k))  cos(psi(k))];

    % Transform car shape to world frame
    car_world = (R * car_body')';
    car_world(:,1) = car_world(:,1) + X(k);
    car_world(:,2) = car_world(:,2) + Y(k);

    % Update car patch
    set(car_patch, 'XData', car_world(:,1), 'YData', car_world(:,2));

    % Update trajectory line
    set(traj_line, 'XData', X(1:k), 'YData', Y(1:k));
    %set(target, 'XData', targ_plot_X(1:k), 'YData', targ_plot_Y(1:k));
    set(vel_text, 'Position', [X(k) Y(k)], ...
                  'String', sprintf('Speed: %.1f m/s', sol(k,2)));


    drawnow;
    pause(pause_time);
end

disp('Animation complete.');

end