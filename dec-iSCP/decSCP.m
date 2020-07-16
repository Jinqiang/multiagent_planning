clc
clear all
close all
addpath(genpath('./../extras'));
addpath(genpath('./../dmpc'));
addpath(genpath('./../utils'));

name_file_results='result_deciSCP.txt';

% Time settings and variables
T = 15; % Trajectory final time
h = 0.4; % time step duration. 
tk = 0:h:T;
K = floor(T/h) + 1; % number of time steps
Ts = 0.01; % period for interpolation @ 100Hza
t = 0:Ts:T; % interpolated time vector
success = 1;

use_iSCP=true; %if false, decSCP usually doesn't work

fileID = fopen(name_file_results,'a'); 
fprintf(fileID,"++++++++++++++++++++++++++++++++++\n",h)
fprintf(fileID,"h: %0.3f\n",h)

% rmin_init = 0.75;
% Initial positions
% [po,pf] = randomTest(N,pmin,pmax,rmin_init,E1,order);

[vmax, amax, po, pf,N,order,rmin,c,E,E1,E2,pmin,pmax] = getParameters();


%% Some Precomputations
l = [];
p = [];
v = [];
a = [];
pk = [];
vk = [];
ak = [];
reached_goal = 0;
violation = 0;
error_tol = 0.01;
% Kinematic model A,b matrices
A = [1 0 0 h 0 0;
     0 1 0 0 h 0;
     0 0 1 0 0 h;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];

b = [h^2/2*eye(3);
     h*eye(3)];
 
prev_row = zeros(6,3*K); % For the first iteration of constructing matrix Ain
A_p = zeros(3*(K-1),3*K);
A_v = zeros(3*(K-1),3*K);
idx=1;
% Build matrix to convert acceleration to position
for k = 1:(K-1)
    add_b = [zeros(size(b,1),size(b,2)*(k-1)) b zeros(size(b,1),size(b,2)*(K-k))];
    new_row = A*prev_row + add_b;   
    A_p(idx:idx+2,:) = new_row(1:3,:);
    A_v(idx:idx+2,:) = new_row(4:6,:);
    prev_row = new_row;
    idx = idx+3;
end

% Empty list of obstacles
l = [];

% Maximum acceleration in m/s^2
alim = 1.0;

tic %measure the time it gets to solve the optimization problem
for i = 1:N 
    poi = po(:,:,i);
    pfi = pf(:,:,i);
    if(use_iSCP==true)
        [pi, vi, ai,success] = singleiSCP(poi,pfi,h,K,pmin,pmax,rmin,alim,l,A_p,A_v,E1,E2,order);
    else
        [pi, vi, ai,success] = singleSCP(poi,pfi,h,K,pmin,pmax,rmin,alim,l,A_p,A_v,E1,E2,order);
    end
    if ~success
        fprintf('Failed solving for vehicle %i\n',i);
        break;
    end
    l = cat(3,l,pi);
    pk(:,:,i) = pi;
    vk(:,:,i) = vi;
    ak(:,:,i) = ai;
    
    % Interpolate solution with a 100Hz sampling
    p(:,:,i) = spline(tk,pi,t);
    v(:,:,i) = spline(tk,vi,t);
    a(:,:,i) = spline(tk,ai,t);
end

if success
    reached_goal = ReachedGoal(pk,pf,K,error_tol,N);
    % Check if collision constraints were not violated
    for i = 1:N
        for j = 1:N
            if(i~=j)
                differ = E1*(p(:,:,i) - p(:,:,j));
                dist = (sum(differ.^order,1)).^(1/order);
                if min(dist) < (rmin-0.01)
                    [value,index] = min(dist);
                    violation = 1;
                    fprintf("Collision constraint violated by %.2fcm: vehicles %i and %i @ k = %i \n", (rmin -value)*100,i,j,index)
                end
            end
        end
    end
end

pass = success && reached_goal && ~violation
computation_time=toc;
fprintf("Computation time is (seconds): %0.3f\n",computation_time)
fprintf(fileID,"Computation time is (seconds): %0.3f\n",computation_time)


[t,p,v,a]=scaleInterpolateCheckCollisionPrintTime(pk,vk,ak,k,N,vmax,amax,h,E1,order,rmin,pf,fileID);

fclose(fileID);
%% Plotting
doPlots(t,N,p,pmin,pmax,po,pf,v,vmax,a,amax)
figure(6)
for i = 1:N
    for j = 1:N
        if(i~=j)
            differ = E1*(pk(:,:,i) - pk(:,:,j));
            dist = (sum(differ.^order,1)).^(1/order);
            plot(tk, dist, 'LineWidth',1.5);
            grid on;
            hold on;
            xlabel('t [s]')
            ylabel('Inter-agent distance [m]');
        end
    end
end
plot(tk,rmin*ones(length(tk),1),'--r','LineWidth',1.5);