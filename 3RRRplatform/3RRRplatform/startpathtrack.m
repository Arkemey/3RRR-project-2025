% This is a subfunction of rrrproject3.m
% Created by Gan Tao   taogan@eng.buffalo.edu
% Graduate Student of SUNY Buffalo
% Date: Jan.2004
% startpathtrack.m is start paht/point tracking.

function startpathtrack(obj,eventdata)

global CC
global k
global fig
global X Y
global CInverCK
global InVS
global S1 S2 S3


cp=get(gca,'CurrentPoint');
     x=cp(1,1);
     y=cp(1,2);
if (x > -15) &  (x < 15) & (y > -15) & (y < 15)
     if CInverCK == 0 & InVS == 1
    
     CC = 1;
     k = 1;
     RRRdrawing(S1,S2,S3)
     hold on
     set(fig,'WindowButtonMotionFcn',@pathtrack);
     X = [];
     Y = [];
     elseif InVS == 1 & CInverCK == 1

     clickInverse;
     end

end

