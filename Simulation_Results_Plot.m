%% Nonuniform Data-driven Control for Networked Control Systems with Safe Set
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C)
% Authors: Seungyong Han <hansy@jbnu.ac.kr>
% 
% Date: October, 2, 2025
%
% This code is the intellectual property of Seungyong Han
% at Jeonbuk National University. Users are granted permission 
% to use, copy, modify, and distribute this code for academic or 
% research purposes, provided that this copyright notice is retained 
% in all copies or derivative works.
%
% By using this code, you agree that any resulting publications or 
% presentations will acknowledge its origin.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
warning('off','all')
clc

%% Data Acquisition
global A B
A = [0 1;
    0 -0.1];
B = [0;
    0.1];

h_fix = 2;
tf=100;      % Final simulation time
ti=0.01;     % Runge-Kutta sampling time 
hup =  4.5;  % Maximum sampling time
xbar = 10;   % Maximum bound of states
wbar = 0.05; % Maximum bound of disturbances
N_IC = 50;   % The number of initial states 

if exist('Collected_data.mat','file')
    % Run if the file exists
    load('Collected_data.mat')

else
    % Run if the file does not exist
    tspan=0:ti:tf;
    sample_size = size(tspan,2);
    t = 0;
    x(:,1)=[1 -1]';  % Initial state for obtaining data set
    
    d_w = [0; 0];
    ua = -1;
    ub = 1;
    u = 0;
    a = 1;
    max_sampling_time = round(h_fix/ti);
    for it=1:sample_size
    
        if (t == max_sampling_time) % tk+1-tk = h
            d = -wbar + (wbar-(-wbar))*rand(1);
            d_w = d*ones(size(A,2),1);
            u = ua + (ub-ua)*rand(1);
    
            X(:,a) = x(:,it);
            Xdot(:,a) = A*x(:,it)+B*u;
            U(:,a) = u;
            W(:,a) = d_w;
            a = a + 1;
            t = 0;
        end
    
        if (it*ti < 50 && t==0)
            max_sampling_time = round(1/ti);
        elseif (50 <= it*ti && t==0)
            max_sampling_time = round(2/ti);
        end
    
    input_temp(:,it)=u;
    sampling(:,it) = max_sampling_time*ti;
    x(:,it+1)=rk6(x(:,it), u, d_w, ti);
    t = t + 1;
    end
    save(['Collected_data.mat'])
end

%% Solve Theorem 1
K = [ -0.099797293033585  -2.065776859979098]; % Given matrix from Theorem 1.
P = [1.781761964330912  -0.078593290231972;    % Given matrix from Theorem 1.
     -0.078593290231972   0.013484540207170];

%% Initial State Selection                        % The number of initial states for the simulation
R = chol(P,'upper');                 % P = R'*R (upper triangular)

theta = 2*pi*rand(1,N_IC);           % Angle
r     = sqrt(rand(1,N_IC));          % Radius ~ U(0,1) for uniform disk
Y     = [r.*cos(theta); r.*sin(theta)];  % Points in unit disk

x0_set = R \ Y;                      % Each satisfies x' P x <= 1

%% Simulation
tspan = 0:ti:100;                 % Simulation horizon
Nt    = numel(tspan);

rng(1);                           
m = size(K,1);                    
max_sampling_time = round(hup/ti);

X_all = zeros(2, Nt+1, N_IC);
U_all = zeros(m, Nt,   N_IC);

for j = 1:N_IC
    x_r = zeros(2, Nt+1);
    u_r = zeros(m, Nt);
    x_r(:,1) = [sat(x0_set(1,j),xbar);sat(x0_set(2,j),xbar)];

    tcnt = 0;                             % Counter for (uniform) update period
    u = K*x_r(:,1);                       % Initial control

    for it = 1:Nt
        if tcnt == max_sampling_time      % Update control at t_k
            u = K*x_r(:,it);
            tcnt = 0;
        end
        u_r(:,it)   = u;
        x_r(:,it+1) = rk45([sat(x_r(1,it),xbar),sat(x_r(2,it),xbar)]', u, [0;0], ti);
        x_r(1,it+1) = sat(x_r(1,it+1),xbar);
        x_r(2,it+1) = sat(x_r(2,it+1),xbar);
        tcnt = tcnt + 1;
    end
    
    X_all(:,:,j) = x_r;
    U_all(:,:,j) = u_r;
end

load('Collected_data.mat')
x_p1 = X_all;
u_p1 = U_all;
ob_u = input_temp;
for i=1:size(x_p1,3)
    init_x(:,i) = x_p1(:,1,i);
end

%% Initial States whithin Safe Set
figure();
plot(1:50,init_x(1,:) , 'o','Color', 'magenta','LineWidth' ,1);
hold on
plot(1:50,init_x(2,:) , '*','Color', 'blue','LineWidth' ,1);
xlabel('Index of initial condition' ,'interpreter','latex', 'FontSize', 20)
ylabel('State','interpreter','latex', 'FontSize', 20)
grid on
legend('$x_{1}$','$x_{2}$','interpreter','latex', 'FontSize', 15);
axis([1 50 -xbar xbar])
set(legend,'Interpreter','latex')

%% State Trajectory from Fifty Initial States to Goal Point
figure();
P = value(P);
clf
x = sdpvar(2,1);
pro_set = [xbar, xbar, -xbar, -xbar;
            xbar, -xbar, xbar, -xbar];
x_vertex = convhull(pro_set(1,:),pro_set(2,:));
aa = fill(pro_set(1,x_vertex),pro_set(2,x_vertex), [0, 0, 0], 'linewidth',0.1,'LineStyle','-');
set(aa,'edgecolor',[0, 0, 0],'FaceColor', [1 1 0],'FaceAlpha', 1);
hold on
x_vertex = convhull(pro_set(1,:),pro_set(2,:));
aa2 = fill(pro_set(1,x_vertex),pro_set(2,x_vertex), [0, 0, 0], 'linewidth',0.1,'LineStyle','-');
set(aa2,'edgecolor',[0, 0, 0],'FaceColor', [0 0 0],'FaceAlpha', 0);
x_vertex = convhull(pro_set(1,:),pro_set(2,:));
bb=fill(pro_set(1,x_vertex),pro_set(2,x_vertex), [0, 0, 0], 'linewidth',2,'LineStyle','-');
set(bb,'edgecolor',[1, 0, 0],'FaceColor', [1 1 1],'FaceAlpha', 1);

%% Unsafe Set Generation
P2 =[1.6926   -0.0676; % Given matrix from Theorem 1 without state constraints.
   -0.0676    0.0107];
P2 = value(P2);
plot(x'*P2*x <= 1,x,[1,1,1], [],sdpsettings('plot.shade',1));
plot(x'*P*x <= 1, x,[1, 1, 0],[],sdpsettings('plot.shade',1));
for j = 1:N_IC
    p4 = plot(x_p1(1,:,j), x_p1(2,:,j), '-',  'LineWidth', 1.5, ...
              'Marker', 'o', 'MarkerIndices', 1, ...   
              'MarkerSize', 4, 'MarkerEdgeColor','k', 'MarkerFaceColor', [0 0.4470 0.7410]);
    hold on
end
p2 = plot(0,0,'g*','LineWidth' ,6)
xlabel('$x_{1}$' ,'interpreter','latex', 'FontSize', 20)
ylabel('$x_{2}$','interpreter','latex', 'FontSize', 20)
grid on
legend([aa,aa2, bb],'Safe Set','Unsafe Set','Constraint','interpreter','latex', 'FontSize', 12);
axis([-20 20 -15 20])
set(legend,'Interpreter','latex')

%% Observed Data
figure;
p1 = plot3(X(1,:),X(2,:),U(1,:), 'o','Color', 'black','LineWidth' ,1,'MarkerFaceColor',[0.855, 0.890, 0.953]);
p1.MarkerSize = 7;
xlabel('$x_{1}$' ,'interpreter','latex', 'FontSize', 20)
ylabel('$x_{2}$','interpreter','latex', 'FontSize', 20)
zlabel('$u$','interpreter','latex', 'FontSize', 20)
legend('Informative Data','interpreter','latex', 'FontSize', 15);
set(legend,'Interpreter','latex')
grid on

%% Control Input Signal
figure;
p1 = plot(tspan(:,1:end),ob_u(:,1:end), ':','Color', [0 0.4470 0.7410],'LineWidth' ,2)
hold on 
for j = 1:N_IC
p2 = plot(tspan(:,1:end),u_p1(:,1:size(ob_u,2),j), '-','LineWidth' ,2)
end
xlabel('Time' ,'interpreter','latex', 'FontSize', 20)
ylabel('Input','interpreter','latex', 'FontSize', 20)
hGroup = plot(NaN, NaN, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 2);
legend([p1, hGroup], 'Observed Input Data', 'Input Signal', 'Box', 'on','interpreter','latex', 'FontSize', 15);
set(legend,'Interpreter','latex')
grid on

%% User Defined Functions
function out = sat(u,u1)
if abs(u) <= u1
out = u;
else
out = u1*sign(u);
end
end

function dxdt = plant(x,u,d)
global A B
dxdt = A*x + B*u + d;
end

function dx = rk45(x, u, d, T)
k1 = plant(x, u, d)*T;
k2 = plant(x+k1*0.5, u, d)*T;
k3 = plant(x+k2*0.5, u, d)*T;
k4 = plant(x+k3, u, d)*T;
dx = x+((k1+k4)/6+(k2+k3)/3);
end