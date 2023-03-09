clc
% Package compares our proposed filter with AHRS filter insipred from
% Roetenberg work. This package shows the real-time behavior of the sensor
% and compares the final angular results of the two filters. 
% Steps 
% (1) Load one of the datasets .mat e.g., DATANomagwithdis.mat from the main folder
% (2) Run the Offline_IMU_DCM_COMP.m
% ------------------------------------------------------------------------------
% If you use this in published package, please cite:
%
% @article{tafrishi2021motion,
%  title={A motion estimation filter for inertial measurement unit with on-board ferromagnetic materials},
%  author={Tafrishi, Seyed Amir and Svinin, Mikhail and Yamamoto, Motoji},
% journal={IEEE Robotics and Automation Letters},
%  volume={6},
%  number={3},
%  pages={4939--4946},
%  year={2021},
%  publisher={IEEE}
%}

%% Main Code Body and Initials
fn = [ac1t(12);ac2t(12);ac3t(12)]/norm([ac1t(12);ac2t(12);ac3t(12)]); %"gravity" in n-frame
mn = [m1t(12)/1000;m2t(12)/1000;m3t(12)/1000]/(norm( [m1t(12)/1000;m2t(12)/1000;m3t(12)/1000])); %"magnetic field" in n-frame
% Read angular velocity.
%interv = 600;
%defining time
%init_time = 1;
%initial time
xa1=0;
xa2=0;
xa3=0;
ma1=0;
ma3=0;
ma2=0;
aca1=0;
aca2=0;
aca3=0;
%T=0;
theta=[1e-3;1e-3;1e-3];
bw = [0; 0; 0];
de1= [0;0;0];
de=[1e-5;1e-5;1e-5];
Nsim = size(T,1);
Euler_ = zeros(Nsim,3);
bw_ = zeros(Nsim,3);
err_ = zeros(Nsim,3);
Cbn=0.001; % Zero means you are in singular part
Cbn_hat1=0;
Cbn_hat2=0;
Cbn_hat3=0;
q_init= [1,0,0,0];
q  = q_init; %intial quatrial angles q_ref is matrix angles!!! :P
Trado1=0;
Trado2=0;
Trado3=0;
Kalm1=0;
Kalm2=0;
Kalm3=0;
Questo1=0;
Questo2=0;
Questo3=0;
%Initial Covariance matrix Kalman
P=zeros(9,9);
P(1:3,1:3)=diag([1e-4, 1e-4, 1e-4]); 
P(4:6,4:6)=diag([1e-6, 1e-6, 1e-6]); 
P(7:9,7:9)=diag([1e-6, 1e-6, 1e-6]); 
%Gyroscope bias estimate Kalman
bwtem = [0;0;0];
dwtem= [0;0;0];
Trecord=0;
%Introduce initial attitude errors Kalman
q1 = q_init;
att_err = 1e-1;
err_x = randn*sqrt(att_err);
err_y = randn*sqrt(att_err);
err_z = randn*sqrt(att_err);
qe = [1, err_x/2, err_y/2, err_z/2];
qe = qe/sqrt(qe*qe');
q1 = quat_mult( q1, qe );
m1t=m1t/1000;%Normalize unit
m2t=m2t/1000;
m3t=m3t/1000;
Cbn = eye(3);
%% PLOT BOX 
%% figure
figure;
view(3);
axis equal;
hold on; grid on;
set(gca,'Xlim',[-2 2]);
set(gca,'Ylim',[-2 2]);
set(gca,'Zlim',[-2 2]);
set(gca,'YDir','reverse');
set(gca,'ZDir','reverse');
set(gcf,'renderer','opengl');
animation_rate = 30;

%% INS Axes
ai1 = [0; 0; 1.5;];
ai2 = [0; 1.5; 0;];
ai3 = [1.5; 0; 0;];
sz =  [0; 0; 0;];
pai1 = ...
    plot3([sz(1) ai1(1)],[sz(2) ai1(2)], [sz(3) ai1(3)],'b-','linewidth',3);
pai2 = ...
    plot3([sz(1) ai2(1)],[sz(2) ai2(2)], [sz(3) ai2(3)],'g-','linewidth',3);
pai3 = ...
    plot3([sz(1) ai3(1)],[sz(2) ai3(2)], [sz(3) ai3(3)],'r-','linewidth',3);

%% Navigation frame Axes
an1 = [0; 0; 10;];
an2 = [0; 10; 0;];
an3 = [10; 0; 0;];
sz =  [0; 0; 0;];
plot3([sz(1) an1(1)],[sz(2) an1(2)], [sz(3) an1(3)],'b-','linewidth',0.1);
plot3([sz(1) an2(1)],[sz(2) an2(2)], [sz(3) an2(3)],'g-','linewidth',0.1);
plot3([sz(1) an3(1)],[sz(2) an3(2)], [sz(3) an3(3)],'r-','linewidth',0.1);
an1 = [0; 0; -10;];
an2 = [0; -10; 0;];
an3 = [-10; 0; 0;];
sz =  [0; 0; 0;];
plot3([sz(1) an1(1)],[sz(2) an1(2)], [sz(3) an1(3)],'b-','linewidth',0.1);
plot3([sz(1) an2(1)],[sz(2) an2(2)], [sz(3) an2(3)],'g-','linewidth',0.1);
plot3([sz(1) an3(1)],[sz(2) an3(2)], [sz(3) an3(3)],'r-','linewidth',0.1);

%% Cube patch for AHRS attitude animation
Vert = [0 0 0; 1 0 0; 1 1 0; 0 1 0; 0 0 1; 1 0 1; 1 1 1; 0 1 1];
Vert(:,1) = (Vert(:,1)-0.5);
Vert(:,2) = (Vert(:,2)-0.5);
Vert(:,3) = (Vert(:,3)-0.5);
Faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
ptch.Vertices = Vert;
ptch.Faces = Faces;
ptch.FaceVertexCData = copper(6);
ptch.FaceColor = 'flat';
patch_handle = patch(ptch);
 i=30;
 initiali=30;
while i<(numel(T))-1
 
  xa1=[xa1,x1t(i)];
  xa2=[xa2,x2t(i)];
  xa3=[xa3,x3t(i)];

  ma1=[ma1,m1t(i)];
  ma2=[ma2,m2t(i)];
  ma3=[ma3,m3t(i)];

  aca1=[aca1,ac1t(i)];
  aca2=[aca2,ac2t(i)];
  aca3=[aca3,ac3t(i)];
Cbn_hat1=[Cbn_hat1,Cbn_hat1(end)+trapz([(x1t(end-1))*pi/180 (x1t(end))*pi/180])];
Cbn_hat2=[Cbn_hat2,Cbn_hat2(end)+trapz([(x2t(end-1))*pi/180 (x2t(end))*pi/180])];
Cbn_hat3=[Cbn_hat3,Cbn_hat3(end)+trapz([(x3t(end-1))*pi/180 (x3t(end))*pi/180])];
    gamma1 = Cbn_hat1(end); %latest q
   gamma2 = Cbn_hat2(end);
    gamma3 = Cbn_hat3(end);
    gamma = norm(Cbn_hat1(end,:));
    lambda0 =cos(gamma/2);
    lambda1 = gamma1*sin(gamma/2)/gamma; %Quadratic transform?) done
    lambda2 = gamma2*sin(gamma/2)/gamma;
    lambda3 = gamma3*sin(gamma/2)/gamma;
 
%%  Kalamn Filter Based 
if i==1
 dt=abs(TimerReal(i));   
else
 dt=abs(TimerReal(i)-TimerReal(i-1)); 
end
 
    %% Inertial Sensors readings with noises and biases
    dwb = [(x1t(i))*pi/180;(x2t(i))*pi/180;(x3t(i))*pi/180];
    fb  = [ac1t(i);ac2t(i);ac3t(i)]/norm([ac1t(12);ac2t(12);ac3t(12)]); 
    mb  = [m1t(i);m2t(i);m3t(i)]/norm([m1t(12);m2t(12);m3t(12)]); 
    %% FerroMag_Filter_Proposed
 
    [Cbn, P, bw , de] = FeroMAg_Filter_MAIN(Cbn, de ,P, bw, dwb, fb, mb, fn, mn, dt);
 
    bw_(i,:) = bw*dt;
    [Euler_(i,1), Euler_(i,2), Euler_(i,3)] = dcm_angle(Cbn');
     Kalm1=[Kalm1,Euler_(i,1)];
     Kalm2=[Kalm2,Euler_(i,2)];
     Kalm3=[Kalm3,Euler_(i,3)]; 
 
 
      %% Animations
    if(mod(i,animation_rate) == 0)
        
        %% INS
        %axes
        %body and body axes
        %Cbnref = quat_dcm(quest)';
        a1_ = Cbn*ai1;
        a2_ = Cbn*ai2;
        a3_ = Cbn*ai3;
        s_ = [0, 0, 0];
        set(pai1,'Xdata',[s_(1) a1_(1)]);
        set(pai1,'Ydata',[s_(2) a1_(2)]);
        set(pai1,'Zdata',[s_(3) a1_(3)]);
        set(pai2,'Xdata',[s_(1) a2_(1)]);
        set(pai2,'Ydata',[s_(2) a2_(2)]);
        set(pai2,'Zdata',[s_(3) a2_(3)]);
        set(pai3,'Xdata',[s_(1) a3_(1)]);
        set(pai3,'Ydata',[s_(2) a3_(2)]);
        set(pai3,'Zdata',[s_(3) a3_(3)]);
        
        %Cube
        Vert_ = Vert;
        for j=1:size(Vert,1)
            Vert_(j,:) = (Vert(j,:)*Cbn');
        end
        set(patch_handle,'Vertices',Vert_);
        
     %   set(th,'String',...
      %      sprintf('tm %2.1f\nps   % 7.5f\ntt     % 7.5f\ngm  % 7.5f',...
      %      i*dt,Euler_(i,1)*180/pi,Euler_(i,2)*180/pi,Euler_(i,3)*180/pi));
        
        drawnow;
    end
  

  i=i+1;
end 
 %% Matlab Sensor Fusion Itself
gyroO=horzcat(x1t'*pi/180,x2t'*pi/180,x3t'*pi/180);
accO=horzcat(ac1t',ac2t',ac3t');
magNeto=horzcat(m1t',m2t',m3t');  %[m1c(i);m2c(i);m3c(i)]

 
%% AHRS
 
numSamples = size(accO,1);
decim = 2;
SampleRate=1;
FUSE = ahrsfilter('SampleRate',SampleRate,'DecimationFactor',decim);
 
%orientation = FUSE(accO,gyroO,magNeto);
FUSE.MagneticDisturbanceNoise = 10;

orientation = FUSE(accO,gyroO,magNeto);
%% Results 
        
     
 figure (2)
 title('Angles') 
orientationEulerAngles = eulerd(orientation,'ZYX','frame');
time = (0:decim:(numSamples-1))'/SampleRate;
  hold on
  plot(0:(TimerReal(:,end)/(numel(Kalm1)-1)):TimerReal(:,end),Kalm1*180/pi,'LineWidth',2.5)
  hold on
  plot(0:(TimerReal(:,end)/(numel(Kalm1)-1)):TimerReal(:,end),Kalm2*180/pi,'LineWidth',2.5)
  hold on
  plot(0:(TimerReal(:,end)/(numel(Kalm1)-1)):TimerReal(:,end),Kalm3*180/pi,'LineWidth',2.5) 
  hold on 
  initiali=12;
    plot(0:(TimerReal(:,end)/(numel(orientationEulerAngles(initiali:end,1))-1)):TimerReal(:,end),orientationEulerAngles(initiali:end,1)-orientationEulerAngles(10,1),'--m','LineWidth',2)
    plot(0:(TimerReal(:,end)/(numel(orientationEulerAngles(initiali:end,1))-1)):TimerReal(:,end),orientationEulerAngles(initiali:end,2),'--g','LineWidth',2)
    
    plot(0:(TimerReal(:,end)/(numel(orientationEulerAngles(initiali:end,1))-1)):TimerReal(:,end),orientationEulerAngles(initiali:end,3)-orientationEulerAngles(10,3),'--k','LineWidth',2)
     legend('Proposed FeroMag Filter \theta_x','Proposed FeroMag Filter \theta_y','Proposed FeroMag Filter \theta_z','AHRS Filter \theta_x','AHRS Filter \theta_y','AHRS Filter \theta_z','Reference','Reference')
     
  xlabel('Time (s)','FontSize',34,'LineWidth' , 2,'Interpreter','latex')
  ylabel('\boldmath${\theta}$ (deg)','FontSize',34,'LineWidth' , 2,'Interpreter','latex') 
  
  %---Ground Truth  
plot([0 TimerReal(:,end)],[-57 -57],'.-.r','LineWidth',1)
plot([0 TimerReal(:,end)],[69 69],'.-.r','LineWidth',1)
ylim([-64,78])  
  
 
%% Clean Up
 set(gca,'FontSize',34);
