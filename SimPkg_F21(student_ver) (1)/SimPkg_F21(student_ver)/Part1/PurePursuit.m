%% === Option A: Pure Pursuit Controller for Part 1 ===

clear; load('TestTrack.mat');

% % Plot Track
% figure()
% plot(TestTrack.cline(1,:),TestTrack.cline(2,:),'--')
% hold on
% plot(TestTrack.bl(1,:),TestTrack.bl(2,:))
% plot(TestTrack.br(1,:),TestTrack.br(2,:))

dt = 0.01;
Tmax = 200;                 % max simulation time
N = Tmax/dt;


% Initial state
state = [287,5,-176,0,2,0];   % [X;u;Y;v;psi;r]
sol = state;

% Output control matrix
U = zeros(1,2);

% Pure pursuit parameters
lookahead = 25;             % meters
L = 2.8;                    % wheelbase (a+b)

cline = TestTrack.cline';
theta = TestTrack.theta;

delx = gradient(cline(:,1));
dely = gradient(cline(:,2));
ddelx = gradient(delx);
ddely = gradient(dely);

kappa = abs(delx .* ddely - dely .* ddelx) ./ (delx.^2 + dely.^2).^(3/2);

R = [1];
s = cumsum([0; sqrt(sum(diff(cline).^2,2))]);
targ_plot = [0,0];

for k = 1:N

    X = sol(1); Y = sol(3); psi = sol(5); u = sol(2);

    % Find nearest centerline index
    d2 = sum((cline - [X Y]).^2,2);
    [~, idx] = min(d2);

    % Compute arc length
    
    s_curr = s(idx);
    s_goal = s_curr + lookahead;

    [~, idx_look] = min(abs(s - s_goal));
    target = cline(idx_look,:);
    targ_plot(k,:) = target;

    % Pure pursuit steering
    dx = target(1) - X;
    dy = target(2) - Y;
    R(k) = sqrt(dx^2 + dy^2);
    alpha = atan2(dy, dx) - psi;
    delta = atan2(2*L*sin(alpha)/R(k), 1);
    delta = max(min(delta, 0.5), -0.5);

    % Speed control
    v_des = 30*R(k)/16;
    v_des = 35 ./ sqrt(1 + 1100 * abs(kappa(idx_look)));
    v_des = max(v_des, 1);
    if v_des-u < 0
        Fx = 400*(v_des - u);
    else
        Fx = 350*(v_des - u);
    end
    Fx = max(min(Fx, 5000), -5000);

    U(k,:) = [delta, Fx];

    % Forward integrate one step
    dzdt = bike(state(k,:),U(k,:));
    sol = state(k,:) + dzdt'.*0.01;
    state(k+1,:) = sol;
    % plot(state(k+1,1), state(k+1,3), 'bo');   % car position
    % plot(target(1), target(2), 'rx');   % car position
    % drawnow

    % Stop if finish line crossed
    if X > 1460
        break
    end
end

ROB535_ControlProject_part1_input = U(1:k,:);

plotter(state,TestTrack,targ_plot)

save('ROB535_ControlProject_part1_TeamXX.mat', ...
     'ROB535_ControlProject_part1_input');

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

function [] = plotter(sol,TestTrack,targ_plot)
X = sol(:,1);
Y = sol(:,3);
targ_plot_X = targ_plot(:,1);
targ_plot_Y = targ_plot(:,2);
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
