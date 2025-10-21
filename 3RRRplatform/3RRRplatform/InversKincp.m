function InversKin(y)

global Xg Yg

if y==1
cp=ginput(1)

Xg = cp(1,1);
Yg = cp(1,2);

RRRdrawing(S1,S2,S3)
end