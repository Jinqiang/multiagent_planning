clc
clear all
close all
warning('off','all')

addpath(genpath('./../../extras'));
addpath(genpath('./../../utils'));
name_file_results='result_dmpc.txt';
% Time settings and variables
max_T = 40; % Trajectory final time
h = 0.15 % time step duration /// Crashes if h=0.1
max_K = max_T/h + 1; % number of time steps
k_hor = 15; % horizon length


fileID = fopen(name_file_results,'a'); 
fprintf(fileID,"++++++++++++++++++++++++++++++++++\n",h)
fprintf(fileID,"h: %0.3f\n",h)

% Variables for ellipsoid constraint
order = 2; % choose  between 2 or 4 for the order of the super ellipsoid
rmin = 0.4; % X-Y protection radius for collisions
c = 1.0; % make this 1 for spherical constraint
E = diag([1,1,c]);
E1 = E^(-1);
E2 = E^(-order);

% Workspace boundaries
pmin = [-7.0,-7.0,-7.0];
pmax = [7.0,7.0,7.0];

% Minimum distance between vehicles in m
% rmin_init = 0.3;

% Initial positions
% [po,pf] = randomTest(N,pmin,pmax,rmin_init,E1,order);

[vmax, amax, po, pf,N] = getParameters();


%% Solving the problem
l = [];
p = [];
v = [];
a = [];
pk = [];
vk = [];
ak = [];
ak_mod = [];
vk_mod = [];
success = 0; %check if QP was feasible
at_goal = 0; %At the end of solving, makes sure every agent arrives at the goal
error_tol = 0.01; % 5cm destination tolerance
violation = 0; % checks if violations occured at end of algorithm
outbound = 0;
coll = 0;
term = -1*10^4;

% Penalty matrices when there're predicted collisions
Q = 1000;
S = 100;

% Maximum acceleration in m/s^2
alim = 1.0; %The opt problem is solved with this value, but then it is scaled so that a_max and v_max are satisfied

% Some pre computations
A = getPosMat(h,k_hor);
Aux = [1 0 0 h 0 0;
     0 1 0 0 h 0;
     0 0 1 0 0 h;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];
 
b = [h^2/2*eye(3);
     h*eye(3)];
 
prev_row = zeros(6,3*k_hor); % For the first iteration of constructing matrix Ain
A_p = [];
A_v = [];
A_initp = [];
A_init = eye(6);
for k = 1:k_hor
    add_b = [zeros(size(b,1),size(b,2)*(k-1)) b zeros(size(b,1),size(b,2)*(k_hor-k))];
    new_row = Aux*prev_row + add_b;   
    A_p = [A_p; new_row(1:3,:)];
    A_v = [A_v; new_row(4:6,:)];
    prev_row = new_row;
    A_init = Aux*A_init;
    A_initp = [A_initp; A_init(1:3,:)];  
end

Delta = getDeltaMat(k_hor); 

failed_goal = 0; %how many times the algorithm failed to reach goal
tries = 1; % how many iterations it took the DMPC to find a solution
tic % measure the time it gets to solve the optimization problem
pred = [];
moreconstr = [];
reached_goal = 0;
k = 1;
while ~reached_goal && k < max_K
    for n = 1:N
    if k==1
        poi = po(:,:,n);
        pfi = pf(:,:,n);
        [pi,vi,ai] = initDMPC(poi,pfi,h,k_hor,max_K);
        success = 1;
    else
        pok = pk(:,k-1,n);
        vok = vk(:,k-1,n);
        aok = ak(:,k-1,n);
        [pi,vi,ai,success,outbound,coll] = solveSoftDMPCbound(pok',pf(:,:,n),vok',aok',n,h,l,k_hor,rmin,pmin,pmax,alim,A,A_initp,A_p,A_v,Delta,Q,S,E1,E2,order,term); 
    end
    if (~success || outbound || coll) %problem was infeasible, exit and retry
        break;
    end
    new_l(:,:,n) = pi;
    pk(:,k,n) = pi(:,1);
    vk(:,k,n) = vi(:,1);
    ak(:,k,n) = ai(:,1);
    end
    if ~success 
        if outbound
            fprintf("Failed - problem unfeasible, vehicle couldn't stay in workspace @ k_T = %i, n = %i\n",k,n)
        elseif coll
            fprintf("Failed - collision detected @ k_T = %i by vehicle n = %i\n",k,n);
        else
            fprintf("Failed - problem unfeasible @ k_T = %i, n = %i\n",k,n)
        end
        break;
    end
    l = new_l;
    pred(:,:,:,k) = l;
    % check if the transition has been completed
    reached_goal = ReachedGoal(pk,pf,k,error_tol,N);
    k = k+1;
end

if success && reached_goal
    at_goal = 1;
elseif success && ~reached_goal %if not at goal, retry with more aggressive behaviour
    failed_goal = failed_goal + 1;
    fprintf("Failed - Did not reach goal within the maximum time of %i seconds \n", max_T)
end
passed = success && at_goal %DMPC was successful or not      
computation_time=toc;
fprintf("Computation time is (seconds): %0.3f\n",computation_time)
fprintf(fileID,"Computation time is (seconds): %0.3f\n",computation_time)

if passed   
    [t,p,v,a]=scaleInterpolateCheckCollisionPrintTime(pk,vk,ak,k,N,vmax,amax,h,E1,order,rmin,pf,fileID);
end

fclose(fileID);
%%
figure(1)
colors = distinguishable_colors(N);

% set(gcf, 'Position', get(0, 'Screensize'));
set(gcf,'currentchar',' ')
% while get(gcf,'currentchar')==' '
%    
%     for i = 1:N
%     h_line(i) = animatedline('LineWidth',2,'Color',colors(i,:),'LineStyle',':');
%     end
%     for k = 1:K
%         for i = 1:N
%             clearpoints(h_line(i));
%             addpoints(h_line(i),pred(1,:,i,k),pred(2,:,i,k),pred(3,:,i,k));     
%             hold on;
%             grid on;
%             xlim([pmin(1),pmax(1)])
%             ylim([pmin(2),pmax(2)])
%             zlim([0,pmax(3)])
%             plot3(pk(1,k,i),pk(2,k,i),pk(3,k,i),'o',...
%                 'LineWidth',2,'Color',colors(i,:));
%             plot3(po(1,1,i), po(1,2,i), po(1,3,i),'^',...
%                   'LineWidth',2,'Color',colors(i,:));
%             plot3(pf(1,1,i), pf(1,2,i), pf(1,3,i),'x',...
%                   'LineWidth',2,'Color',colors(i,:));    
%         end
%     drawnow
%     end
%     clf
%     pause(0.5)
% end

%% Plotting
doPlots(t,N,p,pmin,pmax,po,pf,v,vmax,a,amax)
%%
figure(6)
for i = 1:N
    for j = 1:N
        if(i~=j)
            differ = E1*(p(:,:,i) - p(:,:,j));
            dist = (sum(differ.^order,1)).^(1/order);
            plot(t, dist, 'LineWidth',1.5);
            grid on;
            hold on;
            xlabel('t [s]')
            ylabel('Inter-agent distance [m]');
        end
    end
end
plot(t,rmin*ones(length(t),1),'--r','LineWidth',1.5);
% legend(h_plot,h_label);