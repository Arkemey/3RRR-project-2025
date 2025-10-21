% This is a subfunction of rrrproject3.m
% Created by Gan Tao   taogan@eng.buffalo.edu
% Date: Jan.2004

function clickInverse(obj,eventdata)
global Xg Yg
global S1 S2 S3
global CInverCK
% t=1:0.01:10;
% y = sin(t);
% plot(t,y,'b--')
% axis([-1 15 -2 2])

cp=get(gca,'CurrentPoint');

Xcp = cp(1,1);
Ycp = cp(1,2);
step = 20;
Xcpinter = (Xcp-Xg)/step;
Ycpinter = (Ycp-Yg)/step;
Xg_last = Xg;
Yg_last = Yg;

if (-15 < Xcp)&&(Xcp < 15)&&(-15 < Ycp)&&(Ycp < 15)
if CInverCK
for k=1:step
    
    Xg = Xg_last+k*Xcpinter;
    Yg = Yg_last+k*Ycpinter;
    RRRdrawing(S1,S2,S3)
    pause(0.000001)
end

else
    text(-10,10,'Please click the Click to Inverse','FontSize',15,'color','r');
end
end

