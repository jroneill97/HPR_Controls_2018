clc; clear all; close all;
%% set global variables to those from the parameter datasets
global rho g_i T area body_diam L m I_b delta noseL body_radius Cd Cl rcp_b dcg

load('generalParameters.mat');
load('rocketParameters.mat');

g_i         = [0;0;-9.8];
rho         = 1.225;
t = 2;

T           = 0;
area        = rocketParameters.area;

rcp_b       = [0; 0; -.8];
body_diam   = rocketParameters.body_diam;
body_radius = body_diam/2;

m           = rocketParameters.m;
I_b         = rocketParameters.I;
L           = rocketParameters.L;

Cd          = .4;
Cl          = .4;

delta       = rocketParameters.delta;
noseL       = rocketParameters.noseL;
dcg         = rocketParameters.dcg;

%% Calculate State matrix 
states = zeros(1,12); % initialize state matrix
% position     - 1:3 - x y z
% velocity     - 4:6 - u v w
% ang position - 7:9 - roll pitch yaw
% ang velocity - 10:12

% Define initial states
states(6)  = 35;
states(4)  = .1;
states(1)  = 70/2;
states(3)  = 0;
states(8)  = .1;
states(11) = 0 ;
states(12) = 0 ;

% Define data rate
dataRate = 10; % Hz

tArray = [0:1/dataRate:10];
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
[x,   y,   z  ] = cylinder(.005             ,5);    % axis bars

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


width  = 70;
height = 70;
%axis equal
ax = axes('XLim',[0 width],'YLim',[-.5*width .5*width],'ZLim',[0 height]);   % Sets axis limits
grid on
% create rocket body from h
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





