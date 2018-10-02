clc; %clear all; 
close all;

global rho        % air density
global g       % force of gravity (earth fixed)
global Cd         % drag coefficient
global Cl         % lift coefficient
global area          % area matrix   [front side side]
global m          % total mass
global I          % I matrix      eye([Ix Iy Iz])
global r_cp2cg_b  % distance from cm to cg
global dcg       % distance from bottom to cg
global body_diam

g     = -9.8;
rho    = 1.225;
Cd     = 0.295;
Cl     = 0.5;
area      = 7.5;
m      = 4.0;
L      = 1.5;
body_diam = 2*0.0508;
body_radius = body_diam/2;
noseL  = .3;
I      = diag([m*(1/12)*L^2  m*(1/12)*L^2 m*body_radius^2]);
r_cp2cg_b = [0; 0; .2];
dcg = .5;



%% Values that need to be found experimentally
global Cm 
global CM 
global Cn 
global T % set thrust to 0
global CA 
global CY 
global CN 
global Lap
global Lay
global Lda
global delta
global dcgref

Cm = 0;
CM = 0;
Cn = 0;
CN = 0;
CY = 0;
T  = 0; % Thrust 
CA = 0;
Lap = 0;
Lay = 0;
Lda = 0;
delta = 0;
dcgref = dcg;
%% Draw Rocket
[xc, yc, zc]    = cylinder([body_radius  0] ,5);    % nose cone
[xa,  ya,  za ] = cylinder(body_radius      ,5);    % cylinder above CG
[xb,  yb,  zb ] = cylinder(body_radius      ,5);    % cylinder below CG
[x  y   z]      = cylinder(.005             ,5);    % axis bars
%[xs xy] = meshgrid([-2 2],[-2:2]);

h(1) = surface(xc,yc      , (noseL*zc+(L-noseL))        ,'FaceColor','white');%,'EdgeColor','none');
h(2) = surface(xa,ya      , ((L-noseL - dcg)*za + dcg)  ,'FaceColor','white');%,'EdgeColor','none');
h(3) = surface(xb,yb      , (dcg*zb)                    ,'FaceColor','white');%,'EdgeColor','none');
h(4) = surface(z ,x       , (y + dcg)                    ,'FaceColor','blue' ,'EdgeColor', 'none');
h(5) = surface(x ,z       , (y + dcg)                    ,'FaceColor','green','EdgeColor', 'none');
h(6) = surface(x ,y       , 2*(z + dcg)                     ,'FaceColor','red'  ,'EdgeColor', 'none');

h(1) = surface((noseL*xc+(L-noseL)),yc      ,zc         ,'FaceColor','white');%,'EdgeColor','none');
h(2) = surface(((L-noseL - dcg)*za + dcg),ya      , za  ,'FaceColor','white');%,'EdgeColor','none');
h(3) = surface((dcg*zb),yb      , zb                    ,'FaceColor','white');%,'EdgeColor','none');
h(4) = surface((y + dcg) ,x       , (y + dcg)                    ,'FaceColor','blue' ,'EdgeColor', 'none');
h(5) = surface((y + dcg) ,z       , (y + dcg)                    ,'FaceColor','green','EdgeColor', 'none');
h(6) = surface(2*(z + dcg) ,y       , 2*(z + dcg)                     ,'FaceColor','red'  ,'EdgeColor', 'none');
clear xc yc zc xa ya za xb yb zb x y z;

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

view([0 0]);
drawnow;

%% Calculate State matrix 
states = zeros(1,12); % initialize states
states(6) = 2;
states(7) = 0;
dataRate = 1; % Hz
tArray = [0:1/dataRate:5];
t = 0;      % initialize t       
for i = 2:length(tArray)
    [t_new,states_new] = ode45(@(t,y) updated_rocket_equations_of_motion(t,y),...
                               [tArray(i-1) tArray(i)],states(end,:));
    % use indexing instead
    t = vertcat(t,t_new);
    states = vertcat(states,states_new);
    disp((i*100)/length(tArray)); % percent completion
end

%% Animate the rocket trajectory
pos_i    = states(:,1:3);
vel_i    = states(:,4:6);
ang_i    = states(:,7:9);
angVel_i = states(:,10:12);

for i = 2:length(t)
    pause(.01)
    set(body,'Matrix',makehgtform('translate',pos_i(i,:)'+[0;0;dcg],...
                                  'zrotate'  ,ang_i(i,3)',...
                                  'yrotate'  ,ang_i(i,2)',...
                                  'xrotate'  ,ang_i(i,1)',...
                                  'translate',-(pos_i(i,:)'+[0;0;dcg]),...
                                  'translate',pos_i(i,:)'));
    %surface(xs, xy,[0 0],'FaceColor','green' ,'EdgeColor', 'none');
    drawnow
end





