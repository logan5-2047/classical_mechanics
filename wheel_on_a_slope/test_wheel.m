
clear 
close all
gravity = 9.8;
%alpha = pi / 6;
alpha = 0;
psiInitial = 0.0;


start_time = 0.0;
finish_time = 5;
%finish_time = 7;
tolerance = 1.0e-4;


%
% In X
%
thetaApparent =  pi * 2/16;  

Xstart = 0.0;
Ystart = 0.0;
  
phiInitial = 0; 
thetaInitial = thetaApparent - alpha;
speed = 1.5;

wheel_radius_inner = 0.4;
wheel_radius_outer = 0.5;
wheel_radius_aSmall = wheel_radius_outer - wheel_radius_inner;
wheel_radius_aLarge = 1/2 * (wheel_radius_outer + wheel_radius_inner);

density_steel = 7860;

physicalConstants.gravity = gravity;
wheelParams.radius_aSmall = wheel_radius_aSmall;
wheelParams.radius_aLarge = wheel_radius_aLarge;
wheelParams.density = 7860;

numericalParameters.start_time = start_time;
numericalParameters.finish_time = finish_time;
numericalParameters.tolerance = tolerance;

wheel = wheel(wheelParams,alpha,Xstart,Ystart,thetaInitial,phiInitial,psiInitial,speed,physicalConstants,numericalParameters);
%wheel = wheel.applyTestField(speed);
wheel = wheel.solvewheel();
%wheel.plotwheel_time(4,'object');
%wheel.plotwheel_timeZero('object')
%wheel.plotwheel_time(0,'lab');

%wheel.animatewheel('object');
wheel.animatewheel('lab');
