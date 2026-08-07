
clear 
close all
gravity = 9.8;

alpha = pi / 15;
phiInitial = 1* pi/2; 
%phiInitial=pi/2
psiInitial = 0
thetaApparent =  0;  %
thetaApparent=0
thetaInitial = thetaApparent - alpha;
%thetaInitial = thetaApparent;
Xstart =0.0;
Ystart = 0.0;

speed = 0.0;

start_time = 0.0;
finish_time = 5;

tolerance = 1.0e-4;


wheel_radius_inner = 0.2;
wheel_radius_outer = 0.6;
wheel_radius_aSmall = wheel_radius_outer - wheel_radius_inner;
wheel_radius_aLarge = 1/2 * (wheel_radius_outer + wheel_radius_inner);

density_steel = 7860;

physicalConstants.gravity = gravity;
wheelParams.radius_aSmall = wheel_radius_aSmall;
wheelParams.radius_aLarge = wheel_radius_aLarge;
wheelParams.density = density_steel;

numericalParameters.start_time = start_time;
numericalParameters.finish_time = finish_time;
numericalParameters.tolerance = tolerance;

wheel = wheel(wheelParams,alpha,Xstart,Ystart,thetaInitial,phiInitial,psiInitial,speed,physicalConstants,numericalParameters);
wheel = wheel.solvewheel();

%wheel.animatewheel('object',3)
%movefile 'wheel.mp4' 'wheel_zoomedin.mp4'

wheel.animatewheel('lab',3);
movefile 'wheel.mp4' 'wheel_zoomedout.mp4'