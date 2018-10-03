clc; %clear all; 
close all;

%%
global rho        % air density
global g          % force of gravity (earth fixed)

global T          % set thrust to 0
global area       % area matrix  [nose side side]
global body_diam  % body diameter
global L          % rocket length
global dcgref     % dcg-dcgref: distance between real and referance center of gravity 
global dcg        % distance from bottom of rocket to cg

global m          % total mass
global I          % I matrix     diag([Ix Iy Iz])

global Cl         % Rolling moment coefficient
global Cm         % Pitching moment coefficient
global Cn         % Yawing moment coefficient

global CL         % Coefficient of moment due to roll rate
global CM         % Coefficient of moment due to pitch rate
global CN         % Coefficient of moment due to yaw rate

global CA         % Axial force coefficient     
global CY         % Side force coefficient
global Lap        % 
global Lay        % 
global Lda        % 
global delta      % angle of fins (?)
global noseL      % nose cone length

%% set global variables to those from the parameter datasets
load('generalParameters.mat');
load('rocketParameters.mat');

g           = generalParameters.g;
rho         = generalParameters.rho;

T           = rocketParameters.T;
area        = rocketParameters.area;
body_diam   = rocketParameters.body_diam;
body_radius = body_diam/2;
dcgref      = rocketParameters.dcgref;
dcg         = rocketParameters.dcgref;
m           = rocketParameters.m;
I           = rocketParameters.I;
L           = rocketParameters.L;

Cl          = rocketParameters.Cl;
Cm          = rocketParameters.Cm;
Cn          = rocketParameters.Cn;

CM          = rocketParameters.CM;
CN          = rocketParameters.CN;

CA          = rocketParameters.CA;     
CY          = rocketParameters.CY;
Lap         = rocketParameters.Lap;
Lay         = rocketParameters.Lay;
Lda         = rocketParameters.Lda;
delta       = rocketParameters.delta;
noseL       = rocketParameters.noseL;

%% Calculate State matrix 
states = zeros(1,12); % initialize state matrix
% position     - 1:3 - x y z
% velocity     - 4:6 - u v w
% ang position - 7:9 - roll pitch yaw
% ang velocity - 10:12

% Define initial states
states(4)  = 1;

% Define data rate
dataRate = 1; % Hz

tArray = [0:1/dataRate:2];
t = 0;      % initialize t       
for i = 2:length(tArray)
    [t_new,states_new] = ode45(@(t,y) updated_rocket_equations_of_motion(t,y),...
                               [tArray(i-1) tArray(i)],states(end,:));
    % use indexing instead
    t = vertcat(t,t_new);
    states = vertcat(states,states_new);
    
    disp((i*100)/length(tArray)); % display percent completion
end

% clean up workspace a bit
clearvars -except states t body_radius noseL dcg L dataRate

%% Animating the Rocket
% ------------------------------------------------------------------------
%% Draw Rocket
[xc, yc, zc]    = cylinder([body_radius  0] ,5);    % nose cone
[xa,  ya,  za ] = cylinder(body_radius      ,5);    % cylinder above CG
[xb,  yb,  zb ] = cylinder(body_radius      ,5);    % cylinder below CG
[x  y   z]      = cylinder(.005             ,5);    % axis bars

% Rocket body
h(1) = surface(xc,yc      , (noseL*zc+(L-noseL))        ,'FaceColor','white');
h(2) = surface(xa,ya      , ((L-noseL - dcg)*za + dcg)  ,'FaceColor','white');
h(3) = surface(xb,yb      , (dcg*zb)                    ,'FaceColor','white');

% Axis bars
h(4) = surface(z ,x       , (y + dcg)                   ,'FaceColor','blue' ,'EdgeColor', 'none');
h(5) = surface(x ,z       , (y + dcg)                   ,'FaceColor','green','EdgeColor', 'none');
h(6) = surface(x ,y       , 2*(z + dcg)                 ,'FaceColor','red'  ,'EdgeColor', 'none');

clearvars xc yc zc xa ya za xb yb zb x y z body_radius noseL L;
%% defining the axis

whitebg([0 .5 .6])
axis off

ax = axes('XLim',[-5 5],'YLim',[-5 5],'ZLim',[-10 10]);   % Sets axis limits
ax.XDir = 'normal';
ax.YDir = 'normal';
ax.ZDir = 'normal';
grid on
%axis equal

body = hgtransform('Parent',ax);
set(h,'Parent',body);                % gathers all points in h into a single "parent"
set(gcf,'Renderer','opengl');
clear h;

view([0 0]);
drawnow;

%% Animate the rocket trajectory
pos_i    = states(:,1:3);
vel_i    = states(:,4:6);
ang_i    = states(:,7:9);
angVel_i = states(:,10:12);
clear states

for i = 2:length(t)
    pause((t(i)-t(i-1))/dataRate)
    set(body,'Matrix',makehgtform('translate',pos_i(i,:)'+[0;0;dcg],...
                                  'zrotate'  ,ang_i(i,3)',...
                                  'yrotate'  ,ang_i(i,2)',...
                                  'xrotate'  ,ang_i(i,1)',...
                                  'translate',-(pos_i(i,:)'+[0;0;dcg]),...
                                  'translate',pos_i(i,:)'));
    drawnow
end





