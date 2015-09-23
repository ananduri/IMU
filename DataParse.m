clear all;
f=fopen('Data/Data_9_16_10_09_PM.txt');

%%
%fgetl(f);%for Roll, pitch and yaw file. Not for General
%fgetl(f);
%%

array=[];
while(~feof(f))
    temp = fgetl(f);
    if(length(temp)==65)
        array=[array; temp];
    end
end

fclose(f);
clear temp;
pctime=[];
data=[];
for i=1:size(array,1)
    temp = array(i,:);
    split=strsplit(temp);
    pctime=[pctime;split{1}];
    data=[data;split{2}];
end

pct=[];
for i=1:size(pctime,1)
    v=datevec(pctime(i,:));
    pct=[pct;v(4)*3600+v(5)*60+v(6)];    
end

pcdt= diff(pct);
data=data(1:end,:);

accx=[];
gyrox=[];
magx=[];
accy=[];
gyroy=[];
magy=[];
accz=[];
gyroz=[];
magz=[];
t=[];

for i=1:size(data,1)
    taccx=data(i,[11 12 9]);%rearrange L/MSB and throw out 0001
    taccy=data(i,[15 16 13]);
    taccz=data(i,[19 20 17]);
    tgyrox=data(i,[23 24 21 22]);%rearrange L/MSB
    tgyroy=data(i,[27 28 25 26]);
    tgyroz=data(i,[31 32 29 30]);
    tempx=dec2bin(hex2dec(data(i,33:34)));
    tmagx=[data(i,[35 36]) dec2hex(bin2dec(tempx(1:end-3)))];
    tempy=dec2bin(hex2dec(data(i,37:38)));
    tmagy=[data(i,[39 40]) dec2hex(bin2dec(tempy(1:end-3)))];
    tempz=dec2bin(hex2dec(data(i,41:42)));
    tmagz=[data(i,[43 44]) dec2hex(bin2dec(tempz(1:end-1)))];
    tempt=data(i,45:end);
    
    accx=[accx;nhex2dec(taccx,12)];
    accy=[accy;nhex2dec(taccy,12)];
    accz=[accz;nhex2dec(taccz,12)];
    gyrox=[gyrox;nhex2dec(tgyrox,16)];
    gyroy=[gyroy;nhex2dec(tgyroy,16)];
    gyroz=[gyroz;nhex2dec(tgyroz,16)];
    magx=[magx;nhex2dec(tmagx,13)];
    magy=[magy;nhex2dec(tmagy,13)];
    magz=[magz;nhex2dec(tmagz,15)];
    t=[t;hex2dec(tempt)];
end

ax=(accx)*2/2048;
ay=(accy)*2/2048;
az=(accz)*2/2048;
gx=(gyrox)*2000/32767;
gy=(gyroy)*2000/32767;
gz=(gyroz)*2000/32767;
mx=(magx)*1/16; %1uT/16 LSB
my=(magy)*1/16;
mz=(magz)*1/16;
dt=diff(t)*20e-9;
pos=find(abs(dt)<=0.02);%for 100Hz sampling
t=t(pos);
dt=dt(pos);
ax=ax(pos);
ay=ay(pos);
az=az(pos);
gx=gx(pos);
gy=gy(pos);
gz=gz(pos);
mx=mx(pos);
my=my(pos);
mz=mz(pos);