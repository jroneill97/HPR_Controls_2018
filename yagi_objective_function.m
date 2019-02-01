function objectivevalue = yagi_objective_function(y,controlVals,fc,BW,ang,Z0,constraints)
% YAGI_OBJECTIVE_FUNCTION returns the objective for a 6 element Yagi
% OBJECTIVE_VALUE =
% YAGI_OBJECTIVE_FUNCTION(Y,CONTROLVALS,FREQ,ANG,Z0,constraints), assigns
% the appropriate parasitic dimensions, CONTROLVALS to the Yagi antenna Y,
% and uses the frequency FREQ, angle pair,ANG, reference impedance Z0 and
% the constraints to calculate the objective function value.

% The YAGI_OBJECTIVE_FUNCTION function is used for an internal example.
% Its behavior may change in subsequent releases, so it should not be
% relied upon for programming purposes.

% Copyright 2018 The MathWorks, Inc.

y.ReflectorLength = controlVals(1);
y.DirectorLength = controlVals(2:y.NumDirectors+1);
y.ReflectorSpacing = controlVals(y.NumDirectors+2);
y.DirectorSpacing = controlVals(y.NumDirectors+3:end-2);
y.Exciter.Length = controlVals(end-1);
y.Exciter.Spacing = controlVals(end);

% Unpack constraints
Gmin = constraints.Gmin;
Gdev = constraints.Gdeviation;
FBmin = constraints.FBmin;
K = constraints.Penalty;

% Calculate antenna port and field parameters
output = analyzeAntenna(y,fc,BW,ang,Z0);

% Form objective function
output1 = output.MaxDirectivity+output.MismatchLoss;    % Directivity/Gain at zenith

% Gain constraint, e.g. G > 10
c1 = 0;
if output1<Gmin
    c1 = Gmin-output1;
end

% Gain deviation constraint, abs(G-Gmin)<0.1;
c1_dev = 0;
if abs(output1-Gmin)>Gdev
    c1_dev = -Gdev + abs(output1-Gmin);
end

% Front to Back Ratio constraint, e.g. F/B > 15
c2 = 0;
if output.FB < FBmin
    c2 = FBmin-output.FB;
end

% Form the objective + constraints
objectivevalue = -output1 + max(0,(c1+c1_dev+c2))*K;
end

function output = analyzeAntenna(ant,fc,BW,ang,Z0)
%ANALYZEANTENNA calculate the objective function
% OUTPUT = ANALYZEANTENNA(Y,FREQ,BW,ANG,Z0) performs analysis on the
% antenna ANT at the frequency, FC, and calculates the directivity at the
% angles specified by ANG and and the front-to-back ratio. The reflection
% coefficient relative to reference impedance Z0, and impedance are
% computed over the bandwidth BW around FC.

fmin = fc - (BW/2);
fmax = fc + (BW/2);
Nf = 5;
freq = unique([fc,linspace(fmin,fmax,Nf)]);
fcIdx = freq==fc;
s = sparameters(ant,freq,Z0);
Z = impedance(ant,fc);
az = ang(1,:);
el = ang(2,:);
Dmax = pattern(ant,fc,az(1),el(1));
Dback = pattern(ant,fc,az(2),el(2));

% Calculate F/B
F_by_B = Dmax-Dback;

% Compute S11 and mismatch loss
s11 = rfparam(s,1,1);
S11 = mean((20*log10(abs(s11))));
T = max(10*log10(1 - (abs(s11(fcIdx))).^2));

% Form the output structure
output.MaxDirectivity= Dmax;
output.BackLobeLevel = Dback;
output.FB = F_by_B;
output.S11 = S11;
output.MismatchLoss = T;
output.Z = Z;
end