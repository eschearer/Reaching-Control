%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RANDOMMUSCLETARGET
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Descritpion: This script opens a text file with targets in it and plots
% the targets in 3D.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eric Schearer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created: 3 August 2015
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Updated: 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%clear all
close all

nTarg=20;

targetOrder=randperm(nTarg);

targetID = fopen('targetOrder.txt','w');
fprintf(targetID,'% d',targetOrder);
fclose(targetID);


% set up HapticMaster limits
HMtarg=load('../ID_PassiveOnly/executionCode/targets.txt');
minHM=min(HMtarg);
maxHM=max(HMtarg);
% xmax xmin ymax ymin zmax zmin
HMLimits=[maxHM(1) minHM(1) maxHM(2) minHM(2) maxHM(3) minHM(3)];


limitID = fopen('HMlimits.txt','w');
fprintf(limitID,'%f\t',HMLimits);
fclose(limitID);