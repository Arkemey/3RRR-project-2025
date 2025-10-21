% This is a subfunction of rrrproject3.m
% Created by Gan Tao   taogan@eng.buffalo.edu
% Graduate Student of SUNY Buffalo
% Date: Jan.2004
% angledisplay.m is to display the absolutes angle of each link and the
% relative angles. But the relative one has not been finished yet. Sorry!

function []=angledisplay(AD)

global txAD txRD
global theta11 theta12 theta21 theta22 theta31 theta32 
global diaota1 diaota2 diaota3



if AD==2

txAD(1) = text(-10,-22,['\theta_{11}=',num2str(theta11*180/pi)]);
txAD(2) = text(  0,-22,['\theta_{12}=',num2str(theta12*180/pi)]);
txAD(3) = text(-10,-24,['\theta_{21}=',num2str(theta21*180/pi)]);
txAD(4) = text(  0,-24,['\theta_{22}=',num2str(theta22*180/pi)]);
txAD(5) = text(-10,-26,['\theta_{31}=',num2str(theta31*180/pi)]);
txAD(6) = text(  0,-26,['\theta_{32}=',num2str(theta32*180/pi)]);



elseif AD == 1


else
    warning('You faced a error. Try to reopen it again, or mail to taogan@eng.buffalo.edu')
    
end

