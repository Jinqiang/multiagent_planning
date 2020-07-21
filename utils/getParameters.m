function [vmax, amax, po, pf, N, order, rmin, c, E ,E1, E2, pmin, pmax]=getParameters()

N = 8; % number of vehicles

% Initial positions
po1=[4.0, 0.0, 1.0]; 
po2=[4.0, 4.0, 1.0]; 
po3=[0.0, 4.0, 1.0]; 
po4=[-4.0, 4.0, 1.0];
po5=[-4.0, 0.0, 1.0];
po6=[-4.0, -4.0, 1.0];
po7=[0.0, -4.0, 1.0];
po8=[4.0, -4.0, 1.0];
po = cat(3,po1,po2,po3,po4,po5,po6,po7,po8); %

% Final positions
pf1=[-4.0, 0.0, 1.0];
pf2=[-4.0, -4.0, 1.0];
pf3=[0.0, -4.0, 1.0];
pf4=[4.0, -4.0, 1.0];
pf5=[4.0, 0.0, 1.0];
pf6=[4.0, 4.0, 1.0];
pf7=[0.0, 4.0, 1.0];
pf8=[-4.0, 4.0, 1.0];
pf  = cat(3,pf1,pf2,pf3,pf4,pf5,pf6,pf7,pf8); %

vmax = 1.7;
amax = 6.2;

% Variables for ellipsoid constraint
order = 2; % choose between 2 or 4 for the order of the super ellipsoid
rmin = 0.3; % rmin is defined as 2*radius_drone
c = 1.0; % make this 1 for spherical constraint
E = diag([1,1,c]);
E1 = E^(-1);
E2 = E^(-order);

% Workspace boundaries
pmin = [-7.0,-7.0,-7.0];
pmax = [7.0,7.0,7.0];


end