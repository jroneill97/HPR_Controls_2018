% This function generates a rotational transformation around the x (i)
% axis. Input desired angle. Default input is rad. You can specify degrees
% or radians by entering 'deg' or 'rad'

function R1 = R1(angle,degOrrad)
    switch nargin
        case 2
            if degOrrad == 'deg'
                angle = deg2rad(angle);
                R1 =[[1,           0,          0]; 
                     [0,  cos(angle), sin(angle)];
                     [0, -sin(angle), cos(angle)]];
                return
            elseif degOrrad == 'rad'
                R1 =[[1,           0,          0]; 
                     [0, cos(angle) , sin(angle)];
                     [0, -sin(angle), cos(angle)]];
                return
            else
                    error('angle type must be "deg" or "rad"');
            end
        case 1
                R1 =[[1,           0,          0]; 
                     [0, cos(angle) , sin(angle)];
                     [0, -sin(angle), cos(angle)]];
                return
    end

    end
    
    
    
