function AnimateRocket(tspan,states,stepSize,rocket)

%% Draw the rocket body
radius = rocket.body_diam / 2;
% Draw the rocket and axis bars
[xc, yc, zc]    = cylinder([radius  0]  ,7);    % nose cone
[xa,  ya,  za ] = cylinder( radius      ,7);    % cylinder above CG
[xb,  yb,  zb ] = cylinder( radius      ,7);    % cylinder below CG
[x,   y,   z  ] = cylinder( 0.005       ,7);    % axis bars
% Rocket body
h(1) = surface(xc,yc , (rocket.noseL*zc+(rocket.L-rocket.noseL))        ,'FaceColor','white');
h(2) = surface(xa,ya , ((rocket.L-rocket.noseL - rocket.dcg)*za + rocket.dcg)  ,'FaceColor','white');
h(3) = surface(xb,yb , (rocket.dcg*zb)                                  ,'FaceColor','white');
% Axis bars
h(4) = surface(z ,x  ,     (y + rocket.dcg) ,'FaceColor','blue' ,'EdgeColor','none');
h(5) = surface(x ,z  ,     (y + rocket.dcg) ,'FaceColor','green','EdgeColor','none');
h(6) = surface(x ,y  , 1.5*(z + rocket.dcg) ,'FaceColor','red'  ,'EdgeColor','none');


axis off
ax = axes('XLim',[min(states(:,1))-500 max(states(:,1))+500],...
          'YLim',[min(states(:,2))-500 max(states(:,2))+500],...
          'ZLim',[0                   max(states(:,3))+50]);
grid on
axis equal

% Create rocket body from h
rocketBody = hgtransform('Parent',ax);
set(h,'Parent',rocketBody);                % gathers all points in h into a single "parent"
set(gcf,'Renderer','opengl');

%% Interprit translational and angular position from states matrix
% Translational position
pos_i    = states(:,1:3);

% Euler axis angle (theta) and unit vector (u)
theta = 2*acos(states(:,10));         % principal angle
u     = states(:,7:9)./sin(theta/2);  % euler axis of rotation

%% Animate the rocket flight
view([0 0]);
camproj perspective
camlookat(h); % Set the initial camera view to the rocket

for i = 2:length(tspan) 
    pause(.1)
    set(rocketBody,'Matrix',makehgtform('translate',pos_i(i-1,:)'+[0;0;rocket.dcg],...
                                  'axisrotate',[u(i-1,1), u(i-1,2), u(i-1,3)],theta(i),...
                                  'translate',-(pos_i(i-1,:)'+[0;0;rocket.dcg]),...
                                  'translate',pos_i(i-1,:)'));  
    campos([pos_i(i-1,1) + 200, 0, pos_i(i-1,3) + 10]);
    camtarget(pos_i(i-1,:));
    drawnow
    
end