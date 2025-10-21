% This is a subfunction of rrrproject3.m
% Created by Gan Tao        taogan@eng.buffalo.edu
% Graduate Student of SUNY Buffalo
% Date: Jan.2004
% emptycheck.m is for checking wether the variables for inverse kinematic
% are full. If there are enough for the inverse it sends 0 otherwise send
% 1.


function emck=emptycheck()

global H a1 b1 
global Yp Xp Ypb Xpb Ypc Xpc
global Xg Yg fai
global S1 S2 S3 AD


if isempty(Xg) | isempty(Yg) | isempty(fai) | isempty(H) | isempty(a1) | isempty(b1) | isempty(Yp) | isempty(Xp) | isempty(Xpb) | isempty(Ypb) | isempty(Ypc) | isempty(Xpc)...
        isempty(S1)|isempty(S2)|isempty(S3)|isempty(AD)
    

    emck = 1;
    
else 
    emck = 0;
    
end