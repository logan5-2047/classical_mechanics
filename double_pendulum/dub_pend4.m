classdef dub_pend4
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    %
    properties
        Prendprops
        Physical
    end
    methods(Access = public)
        function obj = dub_pend4(Prendprops,Physical)
            %Create instance of class
            obj.Prendprops = Prendprops;
            obj.Physical = Physical;
        end
        function solve_pen(obj)
            %[t,y] = ode45(odefun,tspan,y0,options)
            tspan = [0 obj.Prendprops.interval];                            % seconds
            Y0 = obj.set_Y0(); 
            options = odeset('RelTol', 1e-5, 'AbsTol', 1e-7);
            %, options);
            [t,Y] = ode15s(@(t,Y) obj.Y_dot(t,Y),tspan,Y0, options);
            % plot the results
            step = ceil (size(t) / (obj.Prendprops.interval * 25));
            t2 = t(1:step:end);
            Y2= Y(1:step:end,:);
            % obj.plotsol(t,Y)
            obj.plotsol(t2,Y2)
        end
    end
    methods(Access = private)
        function Y0 = set_Y0(obj)
            Y0 = [obj.Prendprops.initialThetaDot1
                  obj.Prendprops.initialThetaDot2
                  obj.Prendprops.initialTheta1
                  obj.Prendprops.initialTheta2];
        end
        function Y_dot = Y_dot(obj,t,Y)
            vec = Y;
            b = obj.calcB2(vec);
            M = obj.calcM(vec);
            Minvrs =  M^(-1);
            Y_dot_top = Minvrs * b;
            Y_dot_bot = vec(1:2,1);
            Y_dot = [Y_dot_top
                     Y_dot_bot];
        end
        function b = calcB2(obj,vec)
            %Calculate b1
            thetaDot1 = vec(1,1);
            thetaDot2 = vec(2,1);   
            theta1 = vec(3,1);
            theta2 = vec(4,1);
            sinTheta1MinusTheta2 = sin(theta1 - theta2);
            a1 = obj.Prendprops.a1;
            a2 = obj.Prendprops.a2;
            m1 = obj.Prendprops.m1;
            m2 = obj.Prendprops.m2;
            g = obj.Physical.gravity;
            cosTheta1 = cos(theta1);
            sinTheta1 = sin(theta1);
            cosTheta2 = cos(theta2);
            sinTheta2 = sin(theta2);
            
          
            b1 = a1*a2*m2*sinTheta1MinusTheta2*thetaDot2*(thetaDot1 - thetaDot2) - a1*cosTheta1*g*m2 - a1*cosTheta1*g*m1 - a1*a2*m2*thetaDot1*thetaDot2*sin(theta1 - theta2);
            b2 = a1*a2*m2*sinTheta1MinusTheta2*thetaDot1*(thetaDot1 - thetaDot2) - a2*cosTheta2*g*m2 + a1*a2*m2*thetaDot1*thetaDot2*sin(theta1 - theta2);
            b = [b1
                 b2];

       end
        function M = calcM(obj,vec)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            %
            a1 = obj.Prendprops.a1;
            a2 = obj.Prendprops.a2;
            m1 = obj.Prendprops.m1;
            m2 = obj.Prendprops.m2;
            g = obj.Physical.gravity;
            theta1 = vec(3,1);
            theta2 = vec(4,1);            
            cosTheta1MinusTheta2 = cos(theta1 - theta2);
            
                      
            M11 = a1^2*(m1 + m2); 
            M21 = a1*a2*cosTheta1MinusTheta2*m2;
            M12 = a1*a2*cosTheta1MinusTheta2*m2;
            M22 = a2^2*m2;
            
            
            M = [M11 M12
                 M21 M22];
        end
        function plotsol(obj,t,Y)
            a1 = obj.Prendprops.a1;
            a2 = obj.Prendprops.a2;
            m1 = obj.Prendprops.m1;
            m2 = obj.Prendprops.m2;
            thetaDot1 = Y(:,1);
            thetaDot2 = Y(:,2);
            %v1 = a1 * thetaDot1;
            %v2 = sqrt(() * ());
            theta1 = Y(:,3);
            theta2 = Y(:,4);            
            plot(t,theta1,'r-')
            hold on
            plot(t,theta2,'b-')
            
            
            video_filename = 'my_animation.mp4';
            v = VideoWriter(video_filename, 'MPEG-4'); % 'MPEG-4' creates an MP4 file
            v.FrameRate = 25;                          % Set target frames per second
            open(v);                                   % Open the file for writing

            N=size(theta1,1);
            figure
            for i=1:N
                clf
                theta1_now = theta1(i);
                theta2_now = theta2(i);
           
            
                x1 = a1*cos(theta1_now);
                y1 = a1*sin(theta1_now);
                x2 = a2*cos(theta2_now)+x1;
                y2 = a2*sin(theta2_now)+y1;
            
                        
                LINESX = [0,x1,x2];
                LINESY = [0,y1,y2];
            
                
                %plot(LINESX,LINESY,'g-')
                %hold on
                %plot(x1,y1,'r*')
                %plot(x2,y2,'r*');
                

                plot(LINESX,LINESY, 'g-', 'LineWidth', 2);
                hold on;
                maker_size1 = ceil(50 * obj.Prendprops.m1^0.333);
                maker_size2 = ceil(50 * obj.Prendprops.m2^0.333);

                scatter(0, 0, 100, 'b', 'filled', 'MarkerFaceColor', 'b');
                scatter(LINESX(2), LINESY(2), maker_size1, 'r', 'filled', 'MarkerFaceColor', 'r');
                scatter(LINESX(3), LINESY(3), maker_size2, 'r', 'filled', 'MarkerFaceColor', 'r');
                side_length = obj.Prendprops.a1 + obj.Prendprops.a2;
                axis([-side_length side_length -side_length side_length])
                
                  drawnow;
    
                % 4. Capture the current plot frame
                frame = getframe(gcf);                 % 'gcf' gets the current figure
    
                % 5. Write the frame to the video file
                writeVideo(v, frame);
                        
            end
            
            % 6. Close the video file to finish saving
            close(v);
            disp(['Movie saved successfully as ', video_filename]);

%{
% 1. Set up the video writer object
video_filename = 'my_animation.mp4';
v = VideoWriter(video_filename, 'MPEG-4'); % 'MPEG-4' creates an MP4 file
v.FrameRate = 30;                          % Set target frames per second
open(v);                                   % Open the file for writing

% 2. Initialize the plot
x = linspace(0, 2*pi, 100);
figure;
p = plot(x, sin(x), 'LineWidth', 2);
xlim([0, 2*pi]); ylim([-1.5, 1.5]); grid on;

% 3. Loop over values
for shift = 0:0.1:10
    % Update plot data
    p.YData = sin(x + shift);
    drawnow;
    
    % 4. Capture the current plot frame
    frame = getframe(gcf);                 % 'gcf' gets the current figure
    
    % 5. Write the frame to the video file
    writeVideo(v, frame);
end

% 6. Close the video file to finish saving
close(v);
disp(['Movie saved successfully as ', video_filename]);

%}
            
        end
    end 
end

