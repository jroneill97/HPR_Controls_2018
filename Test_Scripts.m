clc; clear all; close all;

global rho        % air density
global g_i       % force of gravity (earth fixed)
global Cd         % drag coefficient
global Cl         % lift coefficient
global A          % area matrix   [front side side]
global m          % total mass
global I          % I matrix      [Ix Iy Iz]
global r_cp2cg_b  % distance from cm to cg

g_i     = [0; 0; -9.8];
rho    = 1.225;
Cd     = 0.295;
Cl     = 0.5;
A      = [7.5; 7.5; 0.00811];
m      = 4.0;
L      = 1.5;
radius = 0.0508;
noseL  = .3;
I      = m*[(1/12)*L^2;  (1/12)*L^2; radius^2];
r_cp2cg_b = [0; 0; .2];

%% Draw Rocket
[xc, yc, zc] = cylinder([radius  0]); %cone
[x,  y,  z ] = cylinder(radius);

h(1) = surface(x,y,((L-noseL)*z)  ,'FaceColor','blue');
h(2) = surface(xc,yc,(noseL*zc)+(L-noseL) ,'FaceColor','blue');
axis equal
xlabel x;
ylabel y;
zlabel z;

ax = axes('XLim',[-1.5 1.5],'YLim',[-1.5 1.5],'ZLim',[-1.5 1.5]);   % Sets axis limits

grid on
body = hgtransform('Parent',ax);
set(h,'Parent',body)                % gathers all points in h into a single "parent"
set(gcf,'Renderer','opengl');
drawnow;


%% Calculate State matrix
initial_states = [0 0 0 0 0 10 0 0 0 .01 0 0];
% dataRate = 32; % Hz
tArray = [0 20];
t = 0;      % initialize t
states = zeros([1 12]); % initialize states

for i = 2:length(tArray)
    [t_new,states_new] = ode45(@(t,y) rocket_equations_of_motion(t,y),...
                               [tArray(i-1) tArray(i)],initial_states);
    t = vertcat(t,t_new);
    states = vertcat(states,states_new);
end

pos_i  = states(:,1:3);
vel_i  = states(:,4:6);
ang_i  = states(:,7:9);
angV_i = states(:,10:12);
view(0,0)

for i = 2:length(t)
    % Form z-axis rotation matrix
    Transform = makehgtform('translate',pos_i(i,:)-pos_i(i-1,:));
    Rz        = makehgtform('zrotate',ang_i(i,3)-ang_i(i-1,3))
    Ry        = makehgtform('zrotate',ang_i(i,2)-ang_i(i-1,2))
    Rx        = makehgtform('zrotate',ang_i(i,1)-ang_i(i-1,1))
    Rotate    = Rz*Ry*Rx;
    % Set transforms for both transform objects
    set(body,'Matrix',Transform)
    set(body,'Matrix',Rotate)
    drawnow
end





