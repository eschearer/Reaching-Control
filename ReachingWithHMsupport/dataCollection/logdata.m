function logdata(trialNumber)
global Kp Ki target M actDesired idx 
%trialNumber = 1
fsys=xpctarget.fs('TCPIP','192.168.0.2','22222');
%fsys=xpctarget.fs('TCPIP','10.4.0.20','22222');
h = fsys.fopen('data3.dat');
data=fsys.fread(h);
fsys.fclose(h);
new_data=readxpcfile(data);
data = new_data.data;
%dt = 0.01;
%%


hmStart=4;
optoStart=19;
stimStart=115;
tauStart=133;

close all
positionMeasured     = data(:,1:3);   % position of wrist
time              = data(:,end);   % time from XPC target clock
hmMeasPosition    = data(:,hmStart:hmStart+2); % position measured by HM
hmMeasPositionCyl = data(:,hmStart+3:hmStart+5); % position measured by HM in cylindrical coordinates
hmComVelocity     = data(:,hmStart+6:hmStart+8); % velocity commanded by HM
hmComAcceleration = data(:,hmStart+9:hmStart+11); % acceleration commanded by HM
hmForce           = data(:,hmStart+12:hmStart+14); % HM end-effector force
thoraxQuaternion  = data(:,optoStart:optoStart+3); % rotation of thorax wrt Optotrak global frame
thoraxPosition    = data(:,optoStart+4:optoStart+6); % position of thorax wrt Optotrak global frame
thoraxFlag        = data(:,optoStart+7);    % Optotrak visibility flag for thorax
humerusQuaternion = data(:,optoStart+8:optoStart+11); % rotation of forearm wrt Optotrak global frame
humerusPosition   = data(:,optoStart+12:optoStart+14); % position of forearm wrt Optotrak global frame
humerusFlag       = data(:,optoStart+15);    % Optotrak visibility flag for forearm
forearmQuaternion = data(:,optoStart+16:optoStart+19);
forearmPosition   = data(:,optoStart+20:optoStart+22);
forearmFlag        = data(:,optoStart+23);
thoraxMarkers     = data(:,optoStart+24:optoStart+43);% positions and flags of thorax markers
humerusMarkers    = data(:,optoStart+44:optoStart+67);% positions and flags of forearm markers
forearmMarkers    = data(:,optoStart+68:optoStart+91);
handMarker        = data(:,optoStart+92:optoStart+95);
stimLevel        = data(:,stimStart:stimStart+8);
stimPW           = data(:,stimStart+9:stimStart+17);
totalActivation  = data(:,tauStart:tauStart+8); % Activation caluclated for totalTorque
xDes         = data(:,tauStart+9:tauStart+11); % Force from feedback
FBForce          = data(:,tauStart+12:tauStart+14); % Error for feedback
PreSatActivation = data(:,tauStart+15:tauStart+23);
forceDes=data(:,tauStart+24:tauStart+26);
passiveF=data(:,tauStart+27:tauStart+29);


dt=1/52;
plots=	1;
if plots==1
    figure(1)
    plot(time,positionMeasured)
    hold on
    plot(time,xDes,'--')

%     figure(2)
%     plot(time,xDes)
%     figure(3)
%     plot(time,FBForce)
%     figure(4)
%     plot(time,forceDes)
%     figure(5)
%     plot(time,totalActivation)
%     figure(4)
%     plot(time,totalActivation)
%     figure(5)
%     test=ForceEnd-ForceStart;
%     test=test*2/3+ForceStart;
%     plot(test,'*')
%     
%     figure(6)
%     plot(ForceEnd(:,1),ForceEnd(:,2),'k*')
%     hold on
%     for i=1:2
%         F(i,:)=forceMeasured(floor((2*i-1)/dt),:);
%     end
%     plot(F(:,1),F(:,2),'b*')
end



% make sure to not overwrite a file
    filename = ['./dataFiles/dataTrial',num2str(trialNumber),'.mat'];

%filename = 'testdata.mat';
if exist(filename,'file') == 0
	% Chicago version
	 %save(filename,'time','hmMeasPosition','hmMeasPositionCyl','hmComVelocity','hmComAcceleration','hmForce','thoraxPosition','thoraxQuaternion','thoraxFlag','humerusPosition','humerusQuaternion','humerusFlag','forearmPosition','forearmQuaternion','forearmFlag','thoraxMarkers','humerusMarkers','forearmMarkers','torques','newHmForce','jointAngles','jointAnglesEKF','handMarker')
   % Cleveland version 
	 save(filename,'positionMeasured','time','hmMeasPosition','hmMeasPositionCyl','hmComVelocity','hmComAcceleration','hmForce','thoraxQuaternion','thoraxPosition','thoraxFlag','humerusQuaternion','humerusPosition','humerusFlag','forearmQuaternion','forearmPosition','forearmFlag','thoraxMarkers','humerusMarkers','forearmMarkers','stimLevel','stimPW','totalActivation','xDes','FBForce','PreSatActivation','Ki','Kp', 'target', 'M', 'actDesired', 'idx','forceDes','passiveF')
else
    display('A logfile already exists for this trial')
end

