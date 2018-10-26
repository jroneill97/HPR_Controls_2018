
var=3;
v_avg=6;
k=.002;
ap=.6126;
ct=.15;
A=.1290;
D=.0508;

h=[1:1500];
noise=sqrt(var)*rand(h(end),1); %create white noise with given variance
noise2=noise/std(noise); %standardizes noise with standard deviation
noise3=noise2'; %transposes noise vector 
velh=v_avg.*(h/80).^(1/7); %wind velocity power law as a function of height
vel=velh+noise3; %adds noise variance to velocity of height
F=k*vel.^2; %wind force equation
T=ct*A*D*vel.^2; %wind torque equation
