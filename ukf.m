%Unscented Kalman Filter for orientation estimation using quaternions

%first 4 components of x is quaternion, last 3 is angular velocities

%first 3 components of z is accelerometer, next 3 gyroscopes, last 3 magnetometer

%order is always x, y, z

%(just notes for me)
%check if quaternion multiplication convention consistent, Eq. 12
%check if "small rotations" are small
%is there/should there be symmetry breaking? 

function [xpost, Ppost] = ukf(x,P,z,dT)
%process noise (fill these two in later)
Q=0.01*eye(6);

%measurement noise
R=0.1*eye(9);

S = chol(P + Q);

W = zeros(12,6);
for i=1:6
	W(i,:) = sqrt(12)*S(:,i);
	W(i+6,:) = -W(i,:);
end
%W(13,:)=zeros(1,6);
x=x(:);
x=x.';
X = addmean(W,x);

%prediction step
Y = zeros(12,7);
for i=1:12
    Y(i,:) = state2state(X(i,:),dT);
end

[xpred] = findmeanE(Y);

Wprime = subtractmean(Y,xpred);

Ppred = getcov(Wprime);

%measurement step
Z = zeros(12,9);
for i=1:12
    Z(i,:) = state2meas(Y(i,:));
end

Zcent = Z - ones([12,1])*mean(Z,1);

Pzz = getcov(Zcent);

Pxz = getcrosscov(Wprime,Zcent);

Pvv = Pzz + R;

K = Pxz/Pvv;
%K = Pxz*inv(Pvv);

inno = z - mean(Z,1);

Kinno = K*inno.';

xdelt = [vect2quat(Kinno(1:3)).';Kinno(4:6)];

xpred=xpred(:);

xpost = [quatmult(xpred(1:4),xdelt(1:4));xpred(5:end)+xdelt(5:end)];

Ppost = Ppred - K*Pvv*K.';



function [xnew] = state2state(x,dT)
%maps present state into predicted state
%convention is q_{k+1} = q_{\Delta} * q_k

dt = dT;%0.5e-3; %sampling rate, should become 500e-9

q = x(1:4);
a = dt*x(5:end);

absw = norm(a);

if absw==0
    qnew = q;
else
    qD = vect2quat(a);
    qnew = quatmult(qD,q);
end
wnew = x(5:end);

%q{k+1}=q{k}+dq/dt*t
%dq/dt = w*q{k}/2;

xnew = [qnew(:);wnew(:)]; 
end

function [z] = state2meas(x)
%maps state into expected measurement
%global gravity field
g = [0,0,-1];%?? [3,3,3];% why 3,3,3 ??

%global B field (initialize properly)
b = [0,0,0]; % why 1,1,1

q = x(1:4);

gq = [0,g];
zg = quatmult(gq,conjquat(q));
zg = quatmult(q,zg);

zg = zg(2:end);
norm(zg);

zw = x(5:end);

bq = [0,b];
zb = quatmult(bq,conjquat(q));
zb = quatmult(q,zb);
zb = zb(2:end);

z = [zg(:);zw(:);zb(:)];
end

function [ymean] = findmeanE(Y)
ymean=zeros(1,7);
wy = Y(:,5:end);
ymean(5:end)=mean(wy,1);

q = Y(:,1:4);

[v,d] = eig(q.'*q);
d=diag(d);
ind=find(d==max(d));
qmean = v(:,ind);
ymean(1:4)=qmean;
end

function [W] = subtractmean(Y,xmean)
W=zeros(12,6);
wx=xmean(5:end);
for k=1:12
	W(k,4:6) = Y(k,5:7) - wx;
end

qm = xmean(1:4);
for k=1:12
    e=quatmult(Y(k,1:4),conjquat(qm));
    ev=quat2vect(e);
    W(k,1:3) = ev;
end
end

function [P] = getcov(W)
s=size(W);
s=s(2);
P = zeros(s,s);
for k=1:12
	summand = W(k,:).'*W(k,:)/12;
	P = P + summand;
end
end

function [P] = getcrosscov(W,Z)
P = W(1,:).'*Z(1,:)/12;
for k=2:12
	summand = W(k,:).'*Z(k,:)/12;
	P = P + summand;
end
end

function [X] = addmean(W,x)
%essentially does Eq. (33) from Kraft
qx = x(1:4);
wx = x(5:7);
X=zeros(12,7);
for k=1:12
	qW = vect2quat(W(k,1:3));
	X(k,1:4) = quatmult(qW,qx);
	wW = W(k,4:6);
	X(k,5:7) = wW + wx;
end
end


%quaternion helper funcs
function [vect] = quat2vect(quat)
vect = quat(2:4)/sqrt(1-(quat(1)*quat(1)));
if abs(quat(1))==1
    vect = [0,0,0];
end
end

function [quat] = vect2quat(vect)
ang=norm(vect);
quat=[cos(ang/2),sin(ang/2)*vect(1)/ang,sin(ang/2)*vect(2)/ang,sin(ang/2)*vect(3)/ang];
if ang==0
    quat=[1,0,0,0];
end
end

function [product] = quatmult(q,p)
%get q*p
product = [q(1)*p(1) - q(2)*p(2) - q(3)*p(3) - q(4)*p(4),
    q(1)*p(2) + q(2)*p(1) + q(3)*p(4) - q(4)*p(3),
    q(1)*p(3) - q(2)*p(4) + q(3)*p(1) + q(4)*p(2),
    q(1)*p(4) + q(2)*p(3) - q(3)*p(2) + q(4)*p(1)];
end

function [qc] = conjquat(q)
%conjugates the quaternion
qc = [q(1),-q(2),-q(3),-q(4)];
end

end