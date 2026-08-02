classdef wheel
    %wheel: Calculates the motion of a rolling wheel
    %
    properties(Access=private)
        numericalParameters
        physicalConstants
        alpha
        Xstart
        Ystart
        wheelParams      
        thetaInitial
        phiInitial
        psiInitial
        thetaVelocityInitial
        phiVelocityInitial
        psiVelocityInitial
        solution
    end
    methods(Access=public)
        function obj = wheel(wheelParams,alpha,Xstart,Ystart,thetaInitial,phiInitial,psiInitial,speed,physicalConstants,numericalParameters)
            % wheel: Constructs an instance of this class
            %
            obj.numericalParameters = numericalParameters;
            obj.physicalConstants = physicalConstants;
            obj.alpha = alpha;
            obj.Xstart = Xstart;
            obj.Ystart = Ystart;
            obj.wheelParams.radius_aLarge = wheelParams.radius_aLarge;
            obj.wheelParams.radius_aSmall = wheelParams.radius_aSmall;
            obj.wheelParams.density = wheelParams.density;
            obj.wheelParams.mass = obj.get_wheelMass();
            obj.wheelParams.interiaMoment = get_interiaMoment(obj);
            obj.thetaInitial = thetaInitial;
            obj.phiInitial = phiInitial;
            obj.psiInitial = psiInitial;
            obj = obj.set_intialSpeeds(speed);
        end
        function obj = set_intialSpeeds(obj,speed)
            % Sets the intial angular speeds of the wheel.
            %   Sets the angular speed of the wheel based on the initial
            %   speed.
            %
            obj.thetaVelocityInitial = 0.0;
            obj.phiVelocityInitial = 0.0;
            aLarge = obj.wheelParams.radius_aLarge;
            aSmall = obj.wheelParams.radius_aSmall;
            cosTheta = cos(obj.thetaInitial);
            obj.psiVelocityInitial = - speed / (aLarge + cosTheta * aSmall);
        end
        function obj = applyTestField(obj,speed)
            % Apply a pseduo physical test motion to the wheel (solely to check the graphs)
            %
            startTime = 0;
            finishTime = 10;
            %
            obj.thetaVelocityInitial = 0.0;
            obj.phiVelocityInitial = 0.0;
            obj.psiVelocityInitial = speed / obj.radius;
            %
            numberOfTimes=101;
            time = linspace(startTime,finishTime,numberOfTimes);
            %
            thetaDot = ones(1,numberOfTimes) * 0.0;
            phiDot = ones(1,numberOfTimes) * 0.0;
            psiDot = ones(1,numberOfTimes) * speed / obj.radius;
            %
            theta = obj.thetaInitial + thetaDot .* time;
            phi = obj.phiInitial + phiDot .* time;
            psi = psiDot .* time;
            %
            theta    = transpose(theta);
            thetaDot = transpose(thetaDot);
            phi      = transpose(phi);
            phiDot   = transpose(phiDot);
            psi      = transpose(psi);
            psiDot   = transpose(psiDot);
            Y = [ theta thetaDot phi phiDot psi psiDot ];           
            t = transpose(time);
            %
            Ytable.time = t;
            Ytable.Y = Y;
            [xTable, yTable] = obj.get_xAndy(t,Y);
            obj.solution.Ytable = Ytable;
            obj.solution.xTable = xTable;
            obj.solution.yTable = yTable;
        end
        function obj = solvewheel(obj)
            %Solve: Solve for the motion of the wheel
            %
            % Set up the initial conditions
            %
            Y0(1) = obj.thetaVelocityInitial;
            Y0(2) = obj.phiVelocityInitial;
            Y0(3) = obj.psiVelocityInitial;
            Y0(4) = obj.thetaInitial;
            Y0(5) = obj.phiInitial;           
            Y0(6) = obj.psiInitial;   
            %
            % Now solve the equations of motion
            %
            start_time = obj.numericalParameters.start_time;
            finish_time = obj.numericalParameters.finish_time;
            tspan = [start_time finish_time];
            %
            x=1;
            %[t,Y] = ode15s(@(t,Y) obj.odefcn(t,Y,type), tspan, Y0);
            %
            % Needed to use a stiff solver because near the end, solution
            % becomes a bit unstable when wheel falls over.
            %   
            %[t,Y] = ode45(@(t,Y) obj.odefcn(t,Y), tspan, Y0);  % Non-stiff solver
            [t,Y] = ode15s(@(t,Y) obj.odefcn(t,Y), tspan, Y0);  % Stiff solver
            %
            % If there has been a colision with the ground, then we need to
            % ensure that the gemoemtical constraints are met
            %
            Y = obj.get_Y_correctedForGroundConstraints(Y);
            %
            % Store the solution.
            %
            Ytable.time = t;
            Ytable.Y = Y;
            [xTable, yTable] = obj.get_xAndy(t,Y);
            obj.solution.Ytable = Ytable;
            obj.solution.xTable = xTable;
            obj.solution.yTable = yTable;
        end
        function [xTable, yTable] = get_xAndy(obj,timeY,Y)
            % Solve for the postion of the contact point of the wheel with the ground
            %
            x0 = obj.Xstart;
            y0 = obj.Ystart;
            %
            % For some reason, these anonamous functions must contain x and
            % y for ode45 to return their solution.
            %
            start_time = timeY(1);
            finish_time = timeY(end);
            tspan = [start_time finish_time];
            %
            % Now solve the ode for x and y.
            %
            [xTable.time,xTable.distance] = ode45(@(t,x) obj.get_xDot(timeY,Y,t), tspan, x0);
            [yTable.time,yTable.distance] = ode45(@(t,x) obj.get_yDot(timeY,Y,t) , tspan, y0);
        end
        function xdot = get_xDot(obj,timeY,Y,t)
            % Get the speed of the y component of the ground contact point of the wheel
            %
            aSmall = obj.wheelParams.radius_aSmall;
            aLarge = obj.wheelParams.radius_aLarge;
            timeTable = timeY;
            thetaTable = Y(:,4);
            thetaDotTable = Y(:,1);
            phiTable = Y(:,5);
            phiDotTable = Y(:,2);
            psiDotTable = Y(:,3);
            theta_func = @(t) interp1(timeTable,thetaTable,t,'linear');
            thetaDot_func = @(t) interp1(timeTable,thetaDotTable,t,'linear');
            phi_func = @(t) interp1(timeTable,phiTable,t,'linear');
            phiDot_func = @(t) interp1(timeTable,phiDotTable,t,'linear');
            psiDot_func = @(t) interp1(timeTable,psiDotTable,t,'linear');
            theta = theta_func(t);
            phi = phi_func(t);
            
            thetaDot = thetaDot_func(t);
            phiDot = phiDot_func(t);
            psiDot = psiDot_func(t);
            
            cosPhi = cos(phi);
            sinPhi = sin(phi);
            sinTheta = sin(theta);
            cosTheta = cos(theta);
            %
            if -pi/2 + obj.numericalParameters.tolerance < theta && theta < pi/2 - obj.numericalParameters.tolerance
                xdot = cosPhi * ( aLarge + sinTheta * aSmall )* psiDot - sinTheta * cosPhi * aLarge * phiDot - sinPhi * cosTheta * aLarge * thetaDot;
            else
                %
                % The wheel is fallen over, and shouldn't move
                %
                xdot = 0.0;
            end
        end
        function ydot = get_yDot(obj,timeY,Y,t)
            % Get the speed of the y component of the ground contact point of the wheel
            %
            aSmall = obj.wheelParams.radius_aSmall;
            aLarge = obj.wheelParams.radius_aLarge;
            timeTable = timeY;
            thetaTable = Y(:,4);
            thetaDotTable = Y(:,1);
            phiTable = Y(:,5);
             phiDotTable = Y(:,2);
            psiDotTable = Y(:,3);
            theta_func = @(t) interp1(timeTable,thetaTable,t,'linear');
            thetaDot_func = @(t) interp1(timeTable,thetaDotTable,t,'linear');
            phi_func = @(t) interp1(timeTable,phiTable,t,'linear');
            phiDot_func = @(t) interp1(timeTable,phiDotTable,t,'linear');
            psiDot_func = @(t) interp1(timeTable,psiDotTable,t,'linear');
            theta = theta_func(t);
            phi = phi_func(t);
            
            thetaDot = thetaDot_func(t);
            phiDot = phiDot_func(t);
            psiDot = psiDot_func(t);
            
            cosPhi = cos(phi);
            sinPhi = sin(phi);
            sinTheta = sin(theta);
            cosTheta = cos(theta);
            %
            if -pi/2 + obj.numericalParameters.tolerance < theta && theta < pi/2 - obj.numericalParameters.tolerance
                ydot = sinPhi * (aLarge + sinTheta * aSmall) * psiDot - sinPhi * sinTheta * aLarge * phiDot + cosPhi * cosTheta * aLarge * thetaDot;

            else
                %
                % The wheel is fallen over, and shouldn't move
                %
                ydot = 0.0;
            end
        end
        function dYdt = odefcn(obj,t,Y)
            % Gets the right hand side to the ODE system: top level
            %
            rightVec = obj.get_rightVec(Y);
            Mat = obj.get_Mat(t,Y);
            Mat_inv = Mat^-1;
            vec = Mat_inv * rightVec;
            vec_lower = obj.get_vec_lower(t,Y);
            %
            theta_dot = Y(1);
            theta = Y(4);
            %
            % Add a litte dampening to disipate transiant oscillations
            %
            vec_resistence =  [ -2 * Y(1); 0; 0 ];
                        vec = vec + vec_resistence;
            fullVec = [vec; vec_lower];
            %
            % Deal with collision with the ground
            %
            if theta <= obj.numericalParameters.tolerance && theta_dot <0.0 || theta >= pi - obj.numericalParameters.tolerance && theta_dot > 0.0
                theta_dot_dot = 0.0;
                phi_dot_dot = 0.0;
                psi_dot_dot = 0.0;
                theta_dot = 0.0;
                phi_dot = 0.0;
                fullVec(1) = theta_dot_dot;
                fullVec(2) = phi_dot_dot ;
                fullVec(3) = psi_dot_dot;
                fullVec(4) = theta_dot;
                fullVec(5) = phi_dot ; 
                %
                % Leave psi_dot alone, so that the wheel keeps spinning
                %
                % fullVec(6)  we leave this one alone
                %
            end
            %
            dYdt = fullVec;
        end
        function rightVec = get_rightVec(obj,Y)
            % Gets the right hand vector for ODE: detailed implementation
            %
            % Extract the key parameters into convenient variable names to
            % ensure ease of readability.
            %
            g = obj.physicalConstants.gravity;
            thetaDot = Y(1);
            phiDot = Y(2);
            psiDot = Y(3);
            theta = Y(4);
            aSmall = obj.wheelParams.radius_aSmall;
            aLarge = obj.wheelParams.radius_aLarge;
            Mass = obj.wheelParams.mass;
            IT = obj.wheelParams.interiaMoment.I;  
            IA = obj.wheelParams.interiaMoment.III;
            cosAlpha = cos(obj.alpha);
            sinTheta = sin(theta);
            cosTheta = cos(theta);
            sinTwoTheta = sin(2 * theta);
          
            m = Mass;
            rvec_1 = (IA*phiDot^2*sinTwoTheta)/2 - (IT*phiDot^2*sinTwoTheta)/2 + (aLarge^2*m*phiDot^2*sinTwoTheta)/2 - IA*cosTheta*phiDot*psiDot + aLarge*cosAlpha*g*m*sinTheta - aLarge^2*cosTheta*m*phiDot*psiDot - (aLarge*aSmall*m*phiDot*psiDot*sinTwoTheta)/2;
            rvec_2 = (thetaDot*(- 2*m*phiDot*sinTwoTheta*aLarge^2 + aSmall*m*psiDot*sinTwoTheta*aLarge + 2*IA*cosTheta*psiDot - 2*IA*phiDot*sinTwoTheta + 2*IT*phiDot*sinTwoTheta))/2;
            rvec_3 = cosTheta*thetaDot*(IA*phiDot + 2*aLarge^2*m*phiDot - aSmall^2*m*psiDot*sinTheta - aLarge*aSmall*m*psiDot + 2*aLarge*aSmall*m*phiDot*sinTheta);
            rightVec = [ rvec_1; rvec_2; rvec_3];
        end
        function Mat = get_Mat(obj,~,Y)
            % Gets the left hand matrix that defines the ODE
            %
            % Extract the key parameters into convenient variable names to
            % ensure ease of readability.
            %
            theta = Y(4);
            Mass = obj.wheelParams.mass;
            aSmall = obj.wheelParams.radius_aSmall;
            aLarge = obj.wheelParams.radius_aLarge;
            IT = obj.wheelParams.interiaMoment.I;  
            IA = obj.wheelParams.interiaMoment.III;    
            %
            sinTheta = sin(theta);
            m = Mass;
            %
            %
            % Now populate the desired matrix.
            %
            mat11 = m*aLarge^2 + IT;
            mat21 = 0;
            mat31 = 0;
            mat12 = 0;
            mat22 = IT + IA*sinTheta^2 - IT*sinTheta^2 + aLarge^2*m*sinTheta^2;
            mat32 = -sinTheta*(m*aLarge^2 + aSmall*m*sinTheta*aLarge + IA);
            mat13 = 0;
            mat23 = -sinTheta*(m*aLarge^2 + aSmall*m*sinTheta*aLarge + IA);
            mat33 = IA + m*(aLarge + aSmall*sinTheta)^2;
            %
            Mat = [mat11,   mat12,   mat13;   
                   mat21,   mat22,   mat23;  
                   mat31,   mat32,   mat33];  
        end
        function leftVec_lower = get_vec_lower(~,~,Y)
            % Gets the lower portion of the vector that descibes the right hand side of the ODE.
            %
            theta_dot = Y(1);
            phi_dot = Y(2);
            psi_dot = Y(3);
            leftVec_lower = [theta_dot; phi_dot; psi_dot];
        end
        function Ycorrected = get_Y_correctedForGroundConstraints(~,Y)
            % Ensures solution meets the ground constraints
            %   The solution can overshoot the ground constraint. So here
            %   the ground constraint is enforced. Note that theta and phi 
            %   motions stop when the ground is impacted.
            %
            numberOfTimes = size(Y,1);
            Ycorrected = Y;
            for n = 1:numberOfTimes
                theta = Y(numberOfTimes,4);
                if theta < 0.0
                    theta = 0.0;
                    Ycorrected(numberOfTimes,4) = theta;
                elseif theta > pi
                    theta = pi;
                    Ycorrected(numberOfTimes,4) = theta;
                end
            end
        end
        function mass = get_wheelMass(obj)
            % Gets the mass of the wheel
            aSmall = obj.wheelParams.radius_aSmall;
            aLarge = obj.wheelParams.radius_aLarge;
            rOuter = aLarge + aSmall;
            rInner = aLarge - aSmall;
            density = obj.wheelParams.density;
            %
            volume = pi * ( 3 * pi * rOuter+ 4 * aSmall ) *aSmall^2 / 3;
            volume = volume + pi * 2 * aSmall * ( rOuter^2 -  rInner^2   );
            mass = density * volume;
        end
        function interiaMoment = get_interiaMoment(obj)
            % Get moment of inertia of the wheel
            %
            aSmall = obj.wheelParams.radius_aSmall;
            aLarge = obj.wheelParams.radius_aLarge;
            rOuter = aLarge + aSmall;
            rInner = aLarge - aSmall;
            density = obj.wheelParams.density;
            %
            % Use the general formula for a torus of elliptical cross
            % section
            %
            ay = aSmall;
            az = aSmall;
            h = 2 * az;
            %
            % Get moment of inertia of the wheel along a diameter
            %
            interiaMoment.I = pi * density * ay * az * ( 32 * ay^3 + 45 * pi * ay^2 * rOuter + 32 * ay * az^2 + 240 * ay * rOuter^2 + 30 * pi * az^2 * rOuter + 60 * pi * rOuter^3 ) / 120 ...
                                              - pi * density * h * ( 3 * rInner^4 - 3 * rOuter^4 + 4 * h^2 * (rInner^2 - rOuter^2)   ) / 12;
            %
            % Get moment of inertia of the axis of the wheel
            %                              
            interiaMoment.III = pi * density * ay * az * ( 32 * ay^3 + 45 * pi * ay^2 * rOuter + 240 * ay * rOuter^2 + 60 * pi * rOuter^3 ) / 60 ...
                                - pi * density * az * ( rInner^4 - rOuter^4 );                  
                            
            % temporary for debug purposes
            %interiaMoment.III = 1/2 * obj.wheelParams.mass * rOuter^2;
            %interiaMoment.I = 1/2 *interiaMoment.III;
            %
        end
        function plotwheel_time(obj,requestedTime,mode)
            % Plots the wheel at a given time
            %
            %
            % Load the tables for the interpolants
            %
            rad_wheel_aSmall = obj.wheelParams.radius_aSmall;
            rad_wheel_aSmall_large = obj.wheelParams.radius_aLarge;
            %
            %
            %
            timeTableX = obj.solution.xTable.time;
            distanceTableX = obj.solution.xTable.distance;
            timeTableY = obj.solution.yTable.time;
            distanceTableY = obj.solution.yTable.distance;
            timeTable = obj.solution.Ytable.time;
            thetaTable = obj.solution.Ytable.Y(:,4);
            phiTable = obj.solution.Ytable.Y(:,5);
            psiTable = obj.solution.Ytable.Y(:,6);
            %
            % Set up the interpolants to provide position and angle as a
            % function of time ( as calculated by the ODE solution)
            %
            x = interp1(timeTableX,distanceTableX,requestedTime,'linear');
            y = interp1(timeTableY,distanceTableY,requestedTime,'linear');
            theta = interp1(timeTable,thetaTable,requestedTime,'linear');
            phi = interp1(timeTable,phiTable,requestedTime,'linear');
            psi = interp1(timeTable,psiTable,requestedTime,'linear');             
            X = - rad_wheel_aSmall_large  * cos(theta) * cos(phi);
            Y = - rad_wheel_aSmall_large  * cos(theta) * sin(phi);
            Z = rad_wheel_aSmall_large  * sin(theta) + rad_wheel_aSmall;
            %
            % Do the plot
            %
            figureHandle = figure;
            x_centre = x + X;
            y_centre = y + Y;
            z_centre = Z;
            obj.add_plot(x_centre,y_centre,z_centre,theta,phi,psi,mode,figureHandle);   
        end
        function plotwheel_timeZero(obj,mode)
            % Plots the wheel at time zero
            %
            rad_wheel_aSmall = obj.wheelParams.radius_aSmall;
            rad_wheel_aSmall_large = obj.wheelParams.radius_aLarge;
            %
            x = obj.Xstart;
            y = obj.Ystart;
            theta = obj.thetaInitial;
            phi = obj.phiInitial;
            psi = obj.psiInitial;
            X = - rad_wheel_aSmall_large  * cos(theta) * cos(phi);
            Y = - rad_wheel_aSmall_large  * cos(theta) * sin(phi);
            Z = rad_wheel_aSmall_large  * sin(theta) + rad_wheel_aSmall;
            %
            % Do the plot
            %
            figureHandle = figure;
            x_centre = x + X;
            y_centre = y + Y;
            z_centre = Z;
            obj.add_plot(x_centre,y_centre,z_centre,theta,phi,psi,mode,figureHandle);   
        end
        function animatewheel(obj,mode)
            % Animates the solution of the wheel's motion
            %
            rad_wheel_aSmall = obj.wheelParams.radius_aSmall;
            rad_wheel_alarge = obj.wheelParams.radius_aLarge;
            %
            % Load the tables for the interpolants
            %
            timeTableX = obj.solution.xTable.time;
            distanceTableX = obj.solution.xTable.distance;
            timeTableY = obj.solution.yTable.time;
            distanceTableY = obj.solution.yTable.distance;
            timeTable = obj.solution.Ytable.time;
            thetaTable = obj.solution.Ytable.Y(:,4);
            phiTable = obj.solution.Ytable.Y(:,5);
            psiTable = obj.solution.Ytable.Y(:,6);
            %
            % Set up the interpolants to provide position and angle as a
            % function of time ( as calculated by the ODE solution)
            %
            x_func = @(requestedTime) interp1(timeTableX,distanceTableX,requestedTime,'linear');
            y_func = @(requestedTime) interp1(timeTableY,distanceTableY,requestedTime,'linear');
            theta_func = @(requestedTime) interp1(timeTable,thetaTable,requestedTime,'linear');
            phi_func = @(requestedTime) interp1(timeTable,phiTable,requestedTime,'linear');
            psi_func = @(requestedTime) interp1(timeTable,psiTable,requestedTime,'linear');
            %
            % Do the animation
            %
            timeStart = timeTable(1);
            timeFinish = timeTable(end);
            %nTime = 100;
            nTime = 20 * timeFinish;
            %timeFinish = 1
            figureHandle = figure;
            timeStep = (timeFinish - timeStart) / (nTime - 1);
            for i=1:nTime
                clf
                time = timeStart + (i - 1) * timeStep;
                theta = theta_func(time);
                phi = phi_func(time); 
                psi = psi_func(time);
                X = x_func(time);
                Y = y_func(time);

                Z = rad_wheel_alarge  * cos(theta) + rad_wheel_aSmall;
                
                x_centre = X;
                y_centre = Y;
                z_centre = Z;
                obj.add_plot(x_centre,y_centre,z_centre,theta,phi,psi,rad_wheel_alarge,rad_wheel_aSmall,mode,figureHandle);              
                drawnow
                %pause(0.2)
                flipBook(i) = getframe(figureHandle);
            end
            %
            % build the video writer
            %
            myWriter = VideoWriter('wheel','MPEG-4');
            myWriter.FrameRate = 20;
            %
            % Write the video
            %
            open(myWriter);
            writeVideo(myWriter, flipBook);
            close(myWriter);
        end
        function add_plot(obj,x,y,z,theta,phi,psi,aLarge,aSmall,mode,figureHandle)
            % Adds a plot to the current figure handle bases on requested rotation
            %
            %
            x_centerOfMass = x;
            y_centerOfMass = y;
            z_centerOfMass = z;
            tolerance = 1.0e-8;
            
            rad_sphere = 0.06;

            wheel_radius_inner = aLarge - aSmall;
            wheel_radius_outer = aLarge + aSmall;
            rad_wheel_aSmall = obj.wheelParams.radius_aSmall;
            thicknes_wheel = 2 * rad_wheel_aSmall;

            z_func = @(r) rad_wheel_aSmall * sqrt( 1 - (aLarge - r).^2 / rad_wheel_aSmall.^2 );

            thetaTemp = 0:pi/50:2*pi;
            r = linspace(wheel_radius_inner + tolerance, wheel_radius_outer - tolerance, 50); 
            z = z_func(r);
            [Z_Top,T] = meshgrid(z,thetaTemp);

            [XtorusU,YtorusU,ZtorusU] = pol2cart(T,r,Z_Top);
            XtorusD = XtorusU;
            YtorusD = YtorusU;
            ZtorusD = - ZtorusU;
            
            nTheta = 50;
            [arrayX, arrayY, arrayZ] = get_cylinder(wheel_radius_inner,aLarge,thicknes_wheel,nTheta );
            
            [x_s1,y_s1,z_s1] = sphere;
            x_s1 = rad_sphere * x_s1;
            y_s1 = rad_sphere * y_s1;
            z_s1 = 1.1 * thicknes_wheel /2 * z_s1;
            [x_s1,y_s1,z_s1] = object_translate(x_s1,y_s1,z_s1,wheel_radius_outer-rad_sphere/2,0,0);         
            %
            thetaRotation = pi/2 + theta;
            phiRotation = - pi/2 + phi;
            [x_s1,y_s1,z_s1] = object_rotate(x_s1,y_s1,z_s1,0.0,0.0,-psi);
            [x_s1,y_s1,z_s1] = object_rotate(x_s1,y_s1,z_s1,pi/2,0,0);
            [x_s1,y_s1,z_s1] = object_rotate(x_s1,y_s1,z_s1,0.0,theta,0);
            [x_s1,y_s1,z_s1] = object_rotate(x_s1,y_s1,z_s1,0.0,0.0,phi);
            
            [arrayX, arrayY, arrayZ] = object_rotate(arrayX, arrayY, arrayZ,0.0,0.0,-psi);
            [arrayX, arrayY, arrayZ] = object_rotate(arrayX, arrayY, arrayZ,0.0,thetaRotation,0);
            [arrayX, arrayY, arrayZ] = object_rotate(arrayX, arrayY, arrayZ,0.0,0.0,phiRotation);        
            
            [XtorusU,YtorusU,ZtorusU] = object_rotate(XtorusU,YtorusU,ZtorusU,0.0,0.0,-psi);
            [XtorusU,YtorusU,ZtorusU] = object_rotate(XtorusU,YtorusU,ZtorusU,0.0,thetaRotation,0);
            [XtorusU,YtorusU,ZtorusU] = object_rotate(XtorusU,YtorusU,ZtorusU,0.0,0.0,phiRotation);
            [XtorusD,YtorusD,ZtorusD] = object_rotate(XtorusD,YtorusD,ZtorusD,0.0,0.0,-psi);
            [XtorusD,YtorusD,ZtorusD] = object_rotate(XtorusD,YtorusD,ZtorusD,0.0,thetaRotation,0);
            [XtorusD,YtorusD,ZtorusD] = object_rotate(XtorusD,YtorusD,ZtorusD,0.0,0.0,phiRotation);
            %
            %wheel_radius_outer + rad_wheel_aSmall)
            %
            if strcmp(mode,'object')
                [x_s1,y_s1,z_s1] = object_translate(x_s1,y_s1,z_s1,0,0,z_centerOfMass);
                [arrayX, arrayY, arrayZ] = object_translate(arrayX, arrayY, arrayZ,0,0,z_centerOfMass);
                [XtorusU,YtorusU,ZtorusU] = object_translate(XtorusU,YtorusU,ZtorusU,0,0,z_centerOfMass);
                [XtorusD,YtorusD,ZtorusD] = object_translate(XtorusD,YtorusD,ZtorusD,0,0,z_centerOfMass);
            elseif strcmp(mode,'lab')
                [x_s1,y_s1,z_s1] = object_translate(x_s1,y_s1,z_s1,x_centerOfMass,y_centerOfMass,z_centerOfMass);
                [arrayX, arrayY, arrayZ] = object_translate(arrayX, arrayY, arrayZ,x_centerOfMass,y_centerOfMass,z_centerOfMass);
                [XtorusU,YtorusU,ZtorusU] = object_translate(XtorusU,YtorusU,ZtorusU,x_centerOfMass,y_centerOfMass,z_centerOfMass);
                [XtorusD,YtorusD,ZtorusD] = object_translate(XtorusD,YtorusD,ZtorusD,x_centerOfMass,y_centerOfMass,z_centerOfMass);
            else
                [x_s1,y_s1,z_s1] = object_translate(x_s1,y_s1,z_s1,x_centerOfMass,y_centerOfMass,z_centerOfMass);
                [arrayX, arrayY, arrayZ] = object_translate(arrayX, arrayY, arrayZ,x_centerOfMass,y_centerOfMass,z_centerOfMass);
                [XtorusU,YtorusU,ZtorusU] = object_translate(XtorusU,YtorusU,ZtorusU,x_centerOfMass,y_centerOfMass,z_centerOfMass);
                [XtorusD,YtorusD,ZtorusD] = object_translate(XtorusD,YtorusD,ZtorusD,x_centerOfMass,y_centerOfMass,z_centerOfMass);
            end
            [x_s1,y_s1,z_s1] = object_rotate(x_s1,y_s1,z_s1,0.0,obj.alpha,0.0);  
            [XtorusU,YtorusU,ZtorusU] = object_rotate(XtorusU,YtorusU,ZtorusU,0.0,obj.alpha,0.0);  
            [XtorusD,YtorusD,ZtorusD] = object_rotate(XtorusD,YtorusD,ZtorusD,0.0,obj.alpha,0.0); 
            [arrayX, arrayY, arrayZ] = object_rotate(arrayX, arrayY, arrayZ,0.0,obj.alpha,0.0); 
            %
            %[x, y] = meshgrid(-200:10:200); % Generate x and y data
            [x, y] = meshgrid(-10:1:10);
            z = zeros(size(x, 1)); % Generate z data
            
            %
            if strcmp(mode,'object')
                [x,y,z] = object_translate(x,y,z,-x_centerOfMass,-y_centerOfMass,0);
            end
            [x,y,z] = object_rotate(x,y,z,obj.alpha,0,0.0);
            %
            
            surf(x, y, z) % Plot the surface

            figure(figureHandle);

            hold on
            hidden on
            %surf(X_wheel1,Y_wheel1,Z_wheel1,'facecolor','r','LineStyle','none');
            %shading interp
            light
            %lighting phong
            
            surf(XtorusU,YtorusU,ZtorusU,'facecolor',[ 1 0 1 ],'LineStyle','none');
            surf(XtorusD,YtorusD,ZtorusD,'facecolor',[ 1 0 1 ],'LineStyle','none');
            surf(x_s1,y_s1,z_s1,'facecolor','y','LineStyle','none');
            
            %
            for n=1:nTheta - 1
               X(1:4) =  arrayX((n - 1) * 6 + 1, 1:4);
                Y(1:4) =  arrayY((n - 1) * 6 + 1, 1:4);
                Z(1:4) =  arrayZ((n - 1) * 6 + 1, 1:4);
                fill3(X,Y,Z,'r','LineStyle','none')
                X(1:4) =  arrayX((n - 1) * 6 + 1, 5:8);
                Y(1:4) =  arrayY((n - 1) * 6 + 1, 5:8);
                Z(1:4) =  arrayZ((n - 1) * 6 + 1, 5:8);
                fill3(X,Y,Z,'r','LineStyle','none')
                X(1:4) =  arrayX((n - 1) * 6 + 1, 9:12);
                Y(1:4) =  arrayY((n - 1) * 6 + 1, 9:12);
                Z(1:4) =  arrayZ((n - 1) * 6 + 1, 9:12);
                fill3(X,Y,Z,'r','LineStyle','none')
                X(1:4) =  arrayX((n - 1) * 6 + 1, 13:16);
                Y(1:4) =  arrayY((n - 1) * 6 + 1, 13:16);
                Z(1:4) =  arrayZ((n - 1) * 6 + 1, 13:16);
                fill3(X,Y,Z,'r','LineStyle','none')
            end
            %
            hold off
            axis([ -1 1    -1  1   -1  1]*5)
            grid on             
           
            %v = [-15 -15 5];
            % v = [-5 30 5];
            %[caz,cel] = view(v)
            %[caz,cel] = view([-180 40 2])
            %view([-10 1 20])
            %view([-10 30 2])
            %view([-1000 20 2])
            %view(20,0)
            %view(100,-20)
            %view([-90 40 50])
            %view([270 -200 50])
            %view([-10 40 50])
            view([30 20 ])
            pbaspect([1 1 1])
            xlabel('X') 
            ylabel('Y')
            zlabel('Z')
            grid on
            axis vis3d          
            camzoom(3)
        end
    end
end
function [X,Y,Z] = object_translate(X,Y,Z,x_trans,y_trans,z_trans)
    % Translate a coordinate array by an amount x_trans,y_trans,z_trans
    %
    X = X + x_trans;
    Y = Y + y_trans;
    Z = Z + z_trans;
end
function [X_new,Y_new,Z_new] = object_rotate(X,Y,Z,alpha,theta,phi)
    % Rotate a coodinate array in the oder: alpha(first),theta(second), and phi(third)
    %
    CA = cos(alpha);
    SA = sin(alpha);
    CT = cos(theta);
    ST = sin(theta);
    CP = cos(phi);
    SP = sin(phi);
    %
    RA = [1,    0,  0;
          0,    CA,  -SA;
          0,    SA,   CA];
      
    RT = [CT,    0,   -ST;
           0,    1,     0;
          ST,    0,   CT];  
      
    RP = [CP, - SP,     0;
          SP,   CP      0;
           0,    0,    1];     
    %  
    Rot = RP * RT * RA; 
    [n,m] = size(X);
    X_new = zeros(n,m);
    Y_new = zeros(n,m);
    Z_new = zeros(n,m);
    for j = 1:m
        for i=1:n
            invec = [X(i,j);
                     Y(i,j);
                     Z(i,j)];
            outvec = Rot * invec;
            X_new(i,j) = outvec(1);
            Y_new(i,j) = outvec(2); 
            Z_new(i,j) = outvec(3);
        end
    end  
    %xp = 
    %xlabel('X') 
    %ylabel('Y')
    %zlabel('Z')
end


