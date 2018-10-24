% This function generates a rotational transformation around the z (k)
% axis. Input desired angle. Default input is rad. You can specify degrees
% or radians by entering 'deg' or 'rad'

function R3 = R3(angle,degOrrad)
    switch nargin
        case 2
            if degOrrad == 'deg'
               angle = deg2rad(angle);
                    R3 = [[cos(angle) ,sin(angle),0];
                          [-sin(angle),cos(angle),0];
                          [0          ,0         ,1]];
                    return
            elseif degOrrad == 'rad'
                    R3 = [[cos(angle) ,sin(angle),0];
                          [-sin(angle),cos(angle),0];
                          [0          ,0         ,1]];
                    return
            else
                    error('angle type must be "deg" or "rad"');
            end
        case 1
                    R3 = [[cos(angle) ,sin(angle),0];
                          [-sin(angle),cos(angle),0];
                          [0          ,0         ,1]];
                      return
    end

    end