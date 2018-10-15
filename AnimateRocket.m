function AnimateRocket(t,states,axes,rocket)
radius = rocket.body_diam / 2;
% Draw the rocket and axis bars
[xc, yc, zc]    = cylinder([radius  0]  ,5);    % nose cone
[xa,  ya,  za ] = cylinder( radius      ,5);    % cylinder above CG
[xb,  yb,  zb ] = cylinder( radius      ,5);    % cylinder below CG
[x,   y,   z  ] = cylinder( 0.005                   ,5);    % axis bars

% Rocket body
h(1) = surface(xc,yc , (rocket.noseL*zc+(rocket.L-rocket.noseL))        ,'FaceColor','white');
h(2) = surface(xa,ya , ((rocket.L-rocket.noseL - rocket.dcg)*za + rocket.dcg)  ,'FaceColor','white');
h(3) = surface(xb,yb , (rocket.dcg*zb)                                  ,'FaceColor','white');
% Axis bars
h(4) = surface(z ,x  ,   (y + rocket.dcg) ,'FaceColor','blue' ,'EdgeColor','none');
h(5) = surface(x ,z  ,   (y + rocket.dcg) ,'FaceColor','green','EdgeColor','none');
h(6) = surface(x ,y  , 2*(z + rocket.dcg) ,'FaceColor','red'  ,'EdgeColor','none');

% Create rocket body from h
body = hgtransform('Parent',axes);
set(h,'Parent',body);                % gathers all points in h into a single "parent"
set(gcf,'Renderer','opengl');

% Animate the rocket trajectory
pos_i    = states(:,1:3);

[roll, pitch, yaw] = quat2angle(states(:,7:10));
angPos_i           = [roll pitch yaw]; % angular position expressed in Euler angles
axis off
grid on
for i = 1:length(t)    
    set(body,'Matrix',makehgtform('translate',pos_i(i,:)'+[0;0;rocket.dcg],...
                                  'zrotate'  ,angPos_i(i,3)',...
                                  'yrotate'  ,angPos_i(i,2)',...
                                  'xrotate'  ,angPos_i(i,1)',...
                                  'translate',-(pos_i(i,:)'+[0;0;rocket.dcg]),...
                                  'translate',pos_i(i,:)'));
    drawnow
end

