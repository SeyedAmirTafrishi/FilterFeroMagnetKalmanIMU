function [Cbn, P, bw, de] = FeroMAg_Filter_MAIN( Cbn, de, P, bw, dwb, fb, mb, fn, mn, dt)
% The Feromagnetic Compesator Kalman-Inspired Filter 
% [Cbn, P, bw] = ahrs_dcm(Cbn, P, bw, dwb, fb, mb, fn, mn, dt)
% Implements the direction cosine matrix AHRS algorithm using 
% measurements from the three-axis gyroscope and accelerometer,
%  vector magnetometer
%   Input arguments:
%   Cbn - Direction cosine matrix [3x3]
%   P   - Kalman filter covarince matrix [6x6]
%   bw  - Gyroscopes biases vector [3x1]
%   dwb - Integral of angular rate vector over the computer cycle [3x1]
%   fb  - Acceleration vector in body frame [3x1]
%   mb  - Magnetic field vector in body frame [3x1]
%   fn  - Gravity vector in navigation frame [3x1]
%   mn  - Magetic field vector in navigation frame [3x1]
%   dt  - Computer cycle, sec.
%
%   Output arguments:
%   Cbn  - Updated direction cosine matrix [3x3]
%   P    - Updated Kalman filter covarince matrix [6x6]
%   bw   - Updated gyroscopes biases vector [3x1]
V_gt=[1e-3;1e-3;1e-3];
W_bt=[1e-3;1e-3;1e-3];
W_mt=[1e-3;1e-3;1e-3]; 
W_at=[1e-4;1e-4;1e-4];
%% Kalman predict
%System dynamics matrix F and system noise covariance matrix Q
%Continuous-time system matrix
thetak1=dcm_angle(Cbn); 
omega=skew(dwb)/dt; %OK
F= zeros(9,9);
F(1:3,1:3)=eye(3)+dt*omega+(dt^2/2)*omega^2; %theta %OK
F(1:3,4:6)=-dt*eye(3)-(dt^2/2)*omega; %b %OK
F(4:6,4:6)=eye(3); %OK
F(7:9,7:9)=eye(3); %OK
%Noise-input mapping matrix
G = zeros(9,9); %OK
G(1:3,1:3) = eye(3); %OK
G(4:6,4:6) = eye(3); %OK
G(7:9,7:9) = eye(3); %OK
%Gyro errors noise
nb = 1e-5; 
%Gyro bias noise
ngb = 1e-8;%OK 
%Electro magnetic noise
ng = 1e-5;%Very sensetive
%System noise
Qn = diag([nb, nb, nb, ngb, ngb, ngb, ng, ng, ng]);%OK
%Trapezioidal integration
Q = 1/2*(F*G*Qn*G'+G*Qn*G'*F')*dt;
% 
% %Covariance predict
P = F*P*F'+Q;
%Cbn=theta
xn=zeros(9,1);
xn(1:3,1)=dwb+bw*dt-dt*V_gt; %theta neg k-1 BAAAADD
xn(4:6,1)=bw; % b neg  k-1
xn(7:9,1)=de; % b neg  k-1
wx=zeros(9,1);
wx(4:6,1)=W_bt;
wx(7:9,1)=W_mt;
xnk1=xn;
xn=F*xn+wx;


%% Correct attitide increments  with estimated biases k?
% dwb = dwb-bw*dt+dt*V_gt; % I have to keep THESE UPDATE (8/18/2020)
% omega_nt=dwb;
% %% Attitude DCM Mechanization (need work)
% %Cbb DCM from k+1 body axes to k body axes
dwb=xn(1:3,1);
rot_norm=norm(dwb);%what is this part? 
sr_a=1-(rot_norm^2/6)+(rot_norm^4/120);
sr_b=(1/2)-(rot_norm^2/24)+(rot_norm^4/720);
Cbb=eye(3)+sr_a*skew(dwb)+sr_b*skew(dwb)*skew(dwb);
%Update Cbn for body motion
Cbn=Cbn*Cbb; % {GS}^R_nt



%% Measurements
%Estimated measurements
%"Gravity" vector estimate in navigation frame
fn_hat = Cbn*fb;
%"Magnetic field" vector estimate in navigation frame
mn_hat = Cbn*mb;

% %Measurement vector
v = zeros(9,1);
v(1:3,1) = fn_hat-fn;
v(4:6,1) = mn_hat-mn;


%% My measurement Vector and matrix
g=9.8;
%v_at=[1e-4;1e-4;1e-4];
Z_gt=Cbn(1:3,3);
a_tn=Cbn*fb;
 

v_at=[1e-3;1e-3;1e-3];
b_et=bw+W_bt;
%% Gyro And accelometer
Delta_za=cross((-Z_gt-a_tn),dwb);%+dt*cross(Z_gt,b_et) 
Delta_zb=cross(dt*(Z_gt),b_et);
c_a=.8;
v_t=(v_at+W_at+c_a*(fn_hat-fn))-cross(Z_gt,dt*V_gt);
Xg=Delta_za+Delta_zb+v_t;
%% Magneto Meter vs gyro
H_tn=Cbn*mb;
d_tn=Cbn*de;
c_d=.8;

V_mt=[1e-4;1e-4;1e-4];
Deltam_za=cross(-H_tn-d_tn,dwb);%+dt*cross(Z_gt,b_et) 
Deltam_zb=cross(dt*(H_tn),b_et);
Deltam_zc=cross(-c_d*[1;1;1],de);
vm_t=W_mt+V_mt-cross(H_tn,dt*V_gt);
Hm=H_tn-cross(d_tn,dwb)-cross(-c_d*[1;1;1],de)+W_mt+V_mt;
Hg=H_tn+cross(H_tn,dwb)-dt*cross(H_tn,b_et)+cross(H_tn,dt*V_gt);
% C=zeros(6,6);
% C(1:3,1:3)=skew(Z_gt-a_tn);%thetasss
% C(1:3,4:6)=skew(dt*Z_gt);%bwwww
% H=C;
%% MY Definitions
r=.131;
Pg=-[0;r*sin(dwb(2));r*cos(dwb(2))];
Thetapt1=dwb;
T=dt;
bet1=b_et;

%%
z = zeros(9,1);
z(1:3,1)=cross(fn_hat,dwb)+c_a*(fn_hat-fn)+W_bt+v_at;
z(4:6,1)=Delta_za+Delta_zb+v_t; % PROBLEM HERE!!!! 8/18/2020
z(7:9,1)=Deltam_za+Deltam_zb+Deltam_zc+vm_t; %mn_hat-mn;
H = zeros(9,9); 
H(1:3,1:3) = skew(fn_hat);
H(4:6,1:3)= skew((-Z_gt-a_tn));
H(4:6,4:6)= skew(dt*(Z_gt));
H(7:9,1:3) = skew(-H_tn-d_tn);
H(7:9,4:6) = skew(dt*(H_tn));
H(7:9,7:9) = skew(-c_d*[1;1;1]);

R = diag([1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4]); %Smaller better
%Gyro and distrubance last maybe

%% Kalman Update
I = eye(9);
S = H*P*H'+ R;
K = (P*H')/S;
P = (I-K*H)*P*(I-K*H)' + K*R*K';

% PC(kp)=trace(P);
% kp=kp+1;
x  = K*(z-H*xn); %x  = K*(v-H*xn); KEEP (on going OK!?)  K*(z-H*xn);

%% Correct Attitude DCM
E = eye(3)+skew(x(1:3,1));
Cbn = E*Cbn;
%Normalize Cbn matrix
delta_12 = Cbn(1,:)*Cbn(2,:)';
Cbn(1,:) = Cbn(1,:) - 1/2*delta_12*Cbn(2,:);
Cbn(2,:) = Cbn(2,:) - 1/2*delta_12*Cbn(1,:);
Cbn(3,:) = cross(Cbn(1,:),Cbn(2,:));
Cbn(1,:) = Cbn(1,:)./norm(Cbn(1,:));
Cbn(2,:) = Cbn(2,:)./norm(Cbn(2,:));
Cbn(3,:) = Cbn(3,:)./norm(Cbn(3,:));

%% Update gyro bias estimate

bw = bw+x(4:6,1);
de = de+x(7:9,1);
end
