% ========================================================
% Thesis Simulation: ωz-dependent Frequency-Domain Correction
% Author: Zhihao Yang (under supervision of control scholar)
% Fully implements your exact specifications (Mar 2026)
% ========================================================

clear; clc; close all;

%% ================== STATIC MISSILE PARAMETERS (you edit here) ==================
tau    = 0.1;          % s
zeta   = 0.7;
omega_n = 20;          % rad/s
Vc     = 1000;         % m/s
r_noise = 1e-4;        % (rad/s)^2

%% ================== SIMULATION SETTINGS ==================
Ts_seeker   = 0.01;    % s
Ts_guidance = 0.05;    % s
dt_phys     = 0.001;   % continuous integration step

omega_z_list = [5, 20, 50];                    % representative dynamic cases
omega_T_grid = logspace(-1, 2, 20);            % 20 points for frequency sweep


methods = {'PN', 'APN', 'ZMD_PNG', 'GainSchedule'};

%% ================== PRECOMPUTE 2D GAIN-SCHEDULE TABLE (offline) ==================
omega_z_grid = [5 20 50];
log10_omegaT_grid = linspace(-1,2,15);
N_eff_table = 3 * ones(length(omega_z_grid), length(log10_omegaT_grid)); % default

disp('=== Building 2D N_eff lookup table (offline, ~2 min) ===');
for i_wz = 1:length(omega_z_grid)
    wz = omega_z_grid(i_wz);
    for i_wt = 1:length(log10_omegaT_grid)
        wt = 10^log10_omegaT_grid(i_wt);
        best_N = 3; best_miss = Inf;
        for N_test = 2:0.5:8
            miss = run_single_sim(tau, zeta, omega_n, Vc, r_noise, Ts_seeker, Ts_guidance, dt_phys, wz, wt, 'PN', N_test, true); % true = silent
            if miss < best_miss
                best_miss = miss; best_N = N_test;
            end
        end
        N_eff_table(i_wz, i_wt) = best_N;
    end
end
disp('2D lookup table ready.');

%% ================== MAIN SWEEP ==================

for wz = omega_z_list
    figure('Name',sprintf('omega_z = %.1f rad/s',wz)); hold on; grid on;
    legend_str = {};
    
    for meth = methods
        miss_amp = zeros(size(omega_T_grid));
        for i = 1:length(omega_T_grid)
            wt = omega_T_grid(i);
            tf = max(50/wt, 5);                    % adaptive flight time
            miss_amp(i) = run_single_sim(tau, zeta, omega_n, Vc, r_noise, Ts_seeker, Ts_guidance, dt_phys, wz, wt, meth{1}, N_eff_table, false, tf);
        end
        semilogx(omega_T_grid, miss_amp, 'LineWidth', 2.5);
        legend_str{end+1} = meth{1};
    end
    xlabel('Target weave frequency \omega_T (rad/s)');
    ylabel('Steady-state miss-distance amplitude (m)');
    title(sprintf('Miss-distance frequency response (\\omega_z = %.1f)',wz));
    legend(legend_str, 'Location','best');
end


%% ================== CORE SIMULATION ENGINE ==================
function miss_amp = run_single_sim(tau, zeta, omega_n, Vc, r_noise, Ts_seeker, Ts_guidance, dt_phys, omega_z, omega_T, method, N_or_table, silent, tf, omega_z_grid)
if nargin < 15, tf = max(50/omega_T,5); end

% Missile SS (continuous, full with RHP zero)
[A_ap, B_ap, C_ap] = get_autopilot_ss(tau, zeta, omega_n, omega_z);
A_kin = [0 1 0 0 0; 0 0 -C_ap(1) -C_ap(2) -C_ap(3); zeros(3,2) A_ap];
B_kin = [0; 0; B_ap];
D_kin = [0; 1; zeros(3,1)];
nx = 5; x = zeros(nx,1);

% Kalman for LOS (3rd-order)
tau_E = 0.5;
A_kf = [0 1 0; 0 0 1; 0 0 -1/tau_E];
B_kf = [0;0;1/tau_E]; C_kf = [1 0 0];
Ad = expm(A_kf*Ts_seeker); Bd = integral(@(t)expm(A_kf*t)*B_kf,0,Ts_seeker,'ArrayValued',true);
P = eye(3); hat_Lambda = [0;0;0]; Qd = Bd*1*Bd'; Rd = r_noise;

t = 0; k_guid = 0; ac = 0; y_history = []; lambda_history = [];
window_len = round(5/Ts_seeker);

while t < tf
    % continuous physics (Euler)
    a_T = 50 * sin(omega_T * t);
    dx = A_kin*x + B_kin*ac + D_kin*a_T;
    x = x + dt_phys*dx;
    y = x(1); dy = x(2);
    t_go = tf - t;
    if t_go <= 0, break; end
    
    % true LOS rate (nonlinear as requested)
    r = Vc * t_go;
    lambda_dot_true = (dy * r - y * (-Vc)) / (r^2 + y^2);  % exact derivative
    
    % seeker sampling
    if mod(t, Ts_seeker) < dt_phys*0.5
        lambda_meas = lambda_dot_true + sqrt(r_noise)*randn;
        % Kalman predict+update
        hat_pred = Ad*hat_Lambda;
        P_pred = Ad*P*Ad' + Qd;
        K = P_pred*C_kf'/(C_kf*P_pred*C_kf' + Rd);
        hat_Lambda = hat_pred + K*(lambda_meas - C_kf*hat_pred);
        P = (eye(3)-K*C_kf)*P_pred;
        
        % real-time ω_T estimation via FFT
        lambda_history = [lambda_history; hat_Lambda(1)];
        if length(lambda_history) > window_len
            lambda_history(1) =[];
            [Pxx,f] = pwelch(lambda_history,[],[],[],1/Ts_seeker);
            [~,idx] = max(Pxx(2:end)); omega_T_hat = 2*pi*f(idx+1);
        else
            omega_T_hat = 0; % initial guess
        end
    end
    
    % guidance command (Ts_guidance)
    if mod(t, Ts_guidance) < dt_phys*0.5
        k_guid = k_guid + 1;
		switch method
			case 'PN'
				N = 3;
				ac = N * Vc * hat_Lambda(1);
			case 'APN'
				N = 3;
				a_T_est = hat_Lambda(2) * t_go^2 / 2;
				ac = N*Vc*hat_Lambda(1) + (N/2)*a_T_est;
			case 'ZMD_PNG'
				N = 3;
				h = [1, 0.15, 0.0225];           % designed for positive realness on static G
				lambda_eq = h * hat_Lambda;
				ac = N * Vc * lambda_eq;
			case 'GainSchedule'
				logwt = log10(max(omega_T_hat, 0.1));
				[~,iz] = min(abs(omega_z_grid-omega_z));
				[~,it] = min(abs(log10_omegaT_grid - logwt));
				N = N_or_table(iz,it);
				ac = N * Vc * hat_Lambda(1);
		end
		if abs(ac)>200
			ac=sign(ac)*200;
		end
    end
    t = t + dt_phys;
    y_history = [y_history; y];
end

% steady-state amplitude from last 10 cycles
n_last = round(10/omega_T / dt_phys);
if n_last > length(y_history)/2
    n_last = floor(length(y_history)/2);
end
y_ss = y_history(end-n_last+1:end);
Y = fft(y_ss); amp = 2*abs(Y(2:floor(end/2)))/length(Y);
miss_amp = max(amp);
if ~silent
    fprintf('ωz=%.1f, ωT=%.2f, %s → miss amp = %.3f m\n', omega_z, omega_T, method, miss_amp);
end
end

%% Autopilot state-space (companion form)
function [A,B,C] = get_autopilot_ss(tau, zeta, wn, wz)
    den = conv([tau 1], [1/wn^2, 2*zeta/wn, 1]);
    num = [-1/wz^2 0 1];
    num = num./den(1); den = den./den(1);
    A = [0 1 0; 0 0 1; -den(4) -den(3) -den(2)];
    B = [0;0;1];
    C = [num(3) num(2) num(1)];
end