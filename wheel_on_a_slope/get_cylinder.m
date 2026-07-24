
function [arrayX, arrayY, arrayZ] = get_cylinder(radInner,radOuter,width,nTheta)

theta=linspace(0,2*pi,nTheta);
arrayX = zeros(6 * nTheta,12);
arrayY = zeros(6 * nTheta,12);
arrayZ = zeros(6 * nTheta,12);
inDist = -width/2;
outDist = width/2;
for n=1:nTheta - 1
    angle1 = theta(n);
    angle2 = theta(n + 1);
    arrayX((n - 1) * 6 + 1, 1) = radInner * cos(angle1);
    arrayX((n - 1) * 6 + 1, 2) = radOuter * cos(angle1);
    arrayX((n - 1) * 6 + 1, 3) = radOuter * cos(angle2);
    arrayX((n - 1) * 6 + 1, 4) = radInner * cos(angle2);
    arrayY((n - 1) * 6 + 1, 1) = radInner * sin(angle1);
    arrayY((n - 1) * 6 + 1, 2) = radOuter * sin(angle1);
    arrayY((n - 1) * 6 + 1, 3) = radOuter * sin(angle2);
    arrayY((n - 1) * 6 + 1, 4) = radInner * sin(angle2);
    arrayZ((n - 1) * 6 + 1, 1) = inDist ;
    arrayZ((n - 1) * 6 + 1, 2) = inDist ;
    arrayZ((n - 1) * 6 + 1, 3) = inDist ;
    arrayZ((n - 1) * 6 + 1, 4) = inDist ;
    arrayX((n - 1) * 6 + 1, 5) = radInner * cos(angle1);
    arrayX((n - 1) * 6 + 1, 6) = radOuter * cos(angle1);
    arrayX((n - 1) * 6 + 1, 7) = radOuter * cos(angle2);
    arrayX((n - 1) * 6 + 1, 8) = radInner * cos(angle2);
    arrayY((n - 1) * 6 + 1, 5) = radInner * sin(angle1);
    arrayY((n - 1) * 6 + 1, 6) = radOuter * sin(angle1);
    arrayY((n - 1) * 6 + 1, 7) = radOuter * sin(angle2);
    arrayY((n - 1) * 6 + 1, 8) = radInner * sin(angle2);
    arrayZ((n - 1) * 6 + 1, 5) = outDist;
    arrayZ((n - 1) * 6 + 1, 6) = outDist;
    arrayZ((n - 1) * 6 + 1, 7) = outDist;
    arrayZ((n - 1) * 6 + 1, 8) = outDist;
    arrayX((n - 1) * 6 + 1, 9) = radInner * cos(angle1);
    arrayX((n - 1) * 6 + 1, 10) = radInner * cos(angle2);
    arrayX((n - 1) * 6 + 1, 11) = radInner * cos(angle2);
    arrayX((n - 1) * 6 + 1, 12) = radInner * cos(angle1);
    arrayY((n - 1) * 6 + 1,  9) = radInner * sin(angle1);
    arrayY((n - 1) * 6 + 1, 10) = radInner * sin(angle2);
    arrayY((n - 1) * 6 + 1, 11) = radInner * sin(angle2);
    arrayY((n - 1) * 6 + 1, 12) = radInner * sin(angle1);
    arrayZ((n - 1) * 6 + 1,  9) = inDist ;
    arrayZ((n - 1) * 6 + 1, 10) = inDist ;
    arrayZ((n - 1) * 6 + 1, 11) =  outDist;
    arrayZ((n - 1) * 6 + 1, 12) =  outDist;
    arrayX((n - 1) * 6 + 1, 13) = radOuter * cos(angle1);
    arrayX((n - 1) * 6 + 1, 14) = radOuter * cos(angle2);
    arrayX((n - 1) * 6 + 1, 15) = radOuter * cos(angle2);
    arrayX((n - 1) * 6 + 1, 16) = radOuter * cos(angle1);
    arrayY((n - 1) * 6 + 1, 13) = radOuter * sin(angle1);
    arrayY((n - 1) * 6 + 1, 14) = radOuter * sin(angle2);
    arrayY((n - 1) * 6 + 1, 15) = radOuter * sin(angle2);
    arrayY((n - 1) * 6 + 1, 16) = radOuter * sin(angle1);
    arrayZ((n - 1) * 6 + 1, 13) = inDist ;
    arrayZ((n - 1) * 6 + 1, 14) = inDist ;
    arrayZ((n - 1) * 6 + 1, 15) =  outDist;
    arrayZ((n - 1) * 6 + 1, 16) =  outDist;
end
%{
figure 
hold on
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
lightangle(gca,-45,30)
lighting phong
%lighting gouraud
axis equal
axis vis3d   
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])
grid on
%}
end