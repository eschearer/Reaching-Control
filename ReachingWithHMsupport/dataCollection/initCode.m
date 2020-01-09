%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INITCODE.M
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Descritpion: This script defines variables for the xpc model
% FeedbackData.mdl
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eric Schearer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created: 25 February 2013
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Updated:8/15/18 Derek Wolf
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global Kp Ki target M actDesired idx

%%%% TIMING PARAMETERS
runTime = 5;
stimStartTime=0.1;

load('trajData.mat')

if exist('count')==1
    count=count+1
else
    count=1
end

% set idx=1 for tuning
idx=order(count);
trajectory=[data(idx).traj(:,1:3) data(idx).t(:,1:3) data(idx).forceRange];
M=data(idx).M;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fileID=fopen('../executionCode/Target.txt','w');
fprintf(fileID,'%f\t%f\t%f\n',trajectory(1,:));
fclose(fileID);

target=trajectory(1,1:3);
target

% Integrator gain
Kp=0;
Ki=0;


% Desired activation
actDesired=zeros(9,1);


rcparams = zeros(9,4);
for j = 1:9
    load(['./recruitmentData/rcparams',num2str(j),'.mat']);
    rcparams(j,:) = params;
end


% HM as Arm Support Information
% GP parameters
load('model_wristPosition')
hypx=modeldata.muscle(10).Fx.hyp.cov;
hypy=modeldata.muscle(10).Fy.hyp.cov;
hypz=modeldata.muscle(10).Fz.hyp.cov;

trainingInputs=modeldata.muscle(10).Fx.trainingInputs;
trainingOutputs_x=modeldata.muscle(10).Fx.trainingOutputs;
trainingOutputs_y=modeldata.muscle(10).Fy.trainingOutputs;
trainingOutputs_z=modeldata.muscle(10).Fz.trainingOutputs;


likx=modeldata.muscle(10).Fx.hyp.lik;
liky=modeldata.muscle(10).Fy.hyp.lik;
likz=modeldata.muscle(10).Fz.hyp.lik;


KXX=covSEard_realTime(hypx,trainingInputs,trainingInputs);
sigma2=exp(2*likx);
KXX_x=(KXX+sigma2*eye(size(KXX,1)));

KXX=covSEard_realTime(hypy,trainingInputs,trainingInputs);
sigma2=exp(2*liky);
KXX_y=(KXX+sigma2*eye(size(KXX,1)));

KXX=covSEard_realTime(hypz,trainingInputs,trainingInputs);
sigma2=exp(2*likz);
KXX_z=(KXX+sigma2*eye(size(KXX,1)));


maxF=[max(trainingOutputs_x),max(trainingOutputs_y),max(trainingOutputs_z)];
minF=[min(trainingOutputs_x),min(trainingOutputs_y),min(trainingOutputs_z)];



if exist('tg')==1
    setparam(tg,14,actDesired);
    setparam(tg,7+12,M);
    setparam(tg,9+12,trajectory);
    setparam(tg,11+12,[0;0;0]);
    setparam(tg,14+12,Ki);
    setparam(tg,15+12,Kp);
    setparam(tg,16+12,actDesired);
    setparam(tg,20+12,target');
end



dt = 1/52;
dtSlow = 1/52;
dec = 1;
markers = 18;
markerDataSize = 4;
rigidBodies = 3;
rigidBodyDataSize = 8;
hmDataSize = 15;
dataSize = hmDataSize + rigidBodies*rigidBodyDataSize + markers*markerDataSize;
uecuFreq = 1/13;


thoraxFlagIndex     = hmDataSize + 1*rigidBodyDataSize;
humerusFlagIndex    = hmDataSize + 2*rigidBodyDataSize;
forearmFlagIndex    = hmDataSize + 3*rigidBodyDataSize;
targetFlagIndex     = hmDataSize + 3*rigidBodyDataSize + markers*markerDataSize;

load('S1data')

pwmat = [0 0 0 0 0 0 0 0 0;250 23 10 20 49 62 107 22 25]; % June 2017