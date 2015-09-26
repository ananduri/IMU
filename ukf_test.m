xpost = [1 0 0 0 0 0 0]';
Ppost = zeros(6,6);
X=[1 0 0 0 0 0 0]';
R(:,:,1)=qGetR([1 0 0 0]);
quat(:,1)=[1 0 0 0];
for i = 1:length(dt)
    [xpost, Ppost] = ukf(xpost,Ppost,[-ax(i) -ay(i) -az(i) -gx(i)*pi/180 -gy(i)*pi/180 -gz(i)*pi/180 0 0 0],dt(i));
    X=[X xpost];
    quat(:,i+1)=xpost(1:4);
    R(:,:,i+1)=qGetR(xpost(1:4));
end