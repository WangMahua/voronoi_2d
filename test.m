inertialx = [ 1 3 4];
inertialy = [ 2 2 2];
mass = [ 4 2 8 ];
cv = [0 0 0;0 0 0;0 0 0];
cv(:,1) = inertialx ./mass;
cv(:,2) = inertialy ./mass;
cv
clear all;
close all;
