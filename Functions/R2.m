% This function generates a rotational transformation around the y (j)
% axis. Input desired angle. Default input is rad. You can specify degrees
% or radians by entering 'deg' or 'rad'

function R2 = R2(angle,degOrrad)
    switch nargin
        case 2
            if degOrrad == 'deg'
               angle = deg2rad(angle);
                 R2 = [cos(angle) ,0,-sin(angle);
                       0          ,1,          0;
                       sin(angle) ,0,cos(angle)];
                 return
                 
            elseif degOrrad == 'rad'
                 R2 = [cos(angle) ,0,-sin(angle);
                       0          ,1,          0;
                       sin(angle) ,0,cos(angle)];
                 return
                 
            else
                    error('angle type must be "deg" or "rad"');
            end
            
        case 1
                 R2 = [cos(angle) ,0,-sin(angle);
                       0          ,1,          0;
                       sin(angle) ,0,cos(angle)];
                 return
    end

    end
    