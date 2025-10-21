% This is a subfunction of rrrproject3.m
% Created by Gan Tao   taogan@eng.buffalo.edu
% Graduate Student of SUNY Buffalo
% Date: Jan.2004
% RRRdrawing.m is the inverse kinematic and drawing angles computing
% function. One of the thing i want to mention here here is the elbowdown
% and elboup. I set some if command to settle it.

function RRRdrawing(S1,S2,S3)

global H a1 b1 
global Yp Xp Ypb Xpb Ypc Xpc
global Xg Yg fai
global theta11 theta12 theta21 theta22 theta31 theta32
global diaota1 diaota2 diaota3
global AD 
global k X Y


e  = ((3^0.5)/3)*H;
Xa = Xg-e*cos(fai+pi/6);
Ya = Yg-e*sin(fai+pi/6);

Xd =[ 1/2*(1/(-8*Xp*Xa-8*Ya*Yp+4*Ya^2+4*Yp^2+4*Xa^2+4*Xp^2)*(-4*Yp*a1^2-4*Ya^2*Yp+4*Ya^3+4*Xp^2*Yp-4*Ya*b1^2+4*Ya*a1^2+4*Ya*Xp^2+4*b1^2*Yp-4*Ya*Yp^2+4*Xa^2*Ya-8*Xp*Xa*Yp+4*Xa^2*Yp-8*Xp*Xa*Ya+4*Yp^3+4*(-2*Xa^4*Yp^2+2*Xp*Xa*b1^4+2*Xa^4*b1^2+12*Xp^2*Xa^2*a1^2+2*Xa^4*a1^2-15*Xa^4*Xp^2-2*Xa^4*Ya^2+12*Xp^2*Xa^2*b1^2-8*Xp*Xa^3*b1^2+2*Xp*Xa*a1^4+2*Xa^2*b1^2*a1^2+2*Xp^2*b1^2*a1^2-16*Xa^3*Ya*Xp*Yp-4*Xp*Xa*b1^2*a1^2+8*Yp*a1^2*Xp*Xa*Ya-4*Ya*b1^2*Xa^2*Yp+8*Ya*b1^2*Xp*Xa*Yp-4*Ya^2*b1^2*Xp*Xa-4*Ya^2*a1^2*Xp*Xa-4*b1^2*Yp^2*Xp*Xa-8*Ya*Yp^3*Xp*Xa+8*Ya^2*Xp^3*Xa-Xa^6-Xp^6-4*Yp*a1^2*Ya*Xp^2-4*Yp*a1^2*Xa^2*Ya-4*Yp^2*a1^2*Xp*Xa+12*Ya^2*Yp^2*Xp*Xa-8*Ya^3*Yp*Xp*Xa-4*Xp^2*Yp*Ya*b1^2+24*Xp^2*Yp*Xa^2*Ya-16*Xp^3*Yp*Xa*Ya-Ya^4*Xp^2-Ya^4*Xa^2-2*Xp^4*Yp^2-Xp^2*Yp^4-2*Ya^2*Xp^4-Xa^2*Yp^4+6*Xp*Xa^5+6*Xp^5*Xa+20*Xp^3*Xa^3-15*Xp^4*Xa^2-Xa^2*b1^4-Xa^2*a1^4-Xp^2*b1^4-Xp^2*a1^4+2*Xp^4*b1^2+2*Xp^4*a1^2+2*Yp^2*a1^2*Xp^2+2*Yp^2*a1^2*Xa^2-6*Ya^2*Yp^2*Xp^2+4*Ya^3*Yp*Xp^2+4*Ya^3*Yp*Xa^2-6*Ya^2*Yp^2*Xa^2+2*Ya^4*Xp*Xa+4*Xp^4*Yp*Ya+2*Xp^2*Yp^2*b1^2+4*Xp^2*Yp^3*Ya+8*Xp^3*Yp^2*Xa-12*Xp^2*Yp^2*Xa^2+2*Ya^2*b1^2*Xp^2+2*Ya^2*b1^2*Xa^2+2*Ya^2*a1^2*Xp^2+2*Ya^2*a1^2*Xa^2-12*Ya^2*Xp^2*Xa^2+2*b1^2*Yp^2*Xa^2+4*Ya*Yp^3*Xa^2+4*Xa^4*Ya*Yp+8*Xa^3*Ya^2*Xp+8*Xp*Xa^3*Yp^2+2*Xp*Xa*Yp^4-8*Xp*Xa^3*a1^2-8*Xp^3*Xa*b1^2-8*Xp^3*Xa*a1^2)^(1/2))*Yp-1/(-8*Xp*Xa-8*Ya*Yp+4*Ya^2+4*Yp^2+4*Xa^2+4*Xp^2)*(-4*Yp*a1^2-4*Ya^2*Yp+4*Ya^3+4*Xp^2*Yp-4*Ya*b1^2+4*Ya*a1^2+4*Ya*Xp^2+4*b1^2*Yp-4*Ya*Yp^2+4*Xa^2*Ya-8*Xp*Xa*Yp+4*Xa^2*Yp-8*Xp*Xa*Ya+4*Yp^3+4*(-2*Xa^4*Yp^2+2*Xp*Xa*b1^4+2*Xa^4*b1^2+12*Xp^2*Xa^2*a1^2+2*Xa^4*a1^2-15*Xa^4*Xp^2-2*Xa^4*Ya^2+12*Xp^2*Xa^2*b1^2-8*Xp*Xa^3*b1^2+2*Xp*Xa*a1^4+2*Xa^2*b1^2*a1^2+2*Xp^2*b1^2*a1^2-16*Xa^3*Ya*Xp*Yp-4*Xp*Xa*b1^2*a1^2+8*Yp*a1^2*Xp*Xa*Ya-4*Ya*b1^2*Xa^2*Yp+8*Ya*b1^2*Xp*Xa*Yp-4*Ya^2*b1^2*Xp*Xa-4*Ya^2*a1^2*Xp*Xa-4*b1^2*Yp^2*Xp*Xa-8*Ya*Yp^3*Xp*Xa+8*Ya^2*Xp^3*Xa-Xa^6-Xp^6-4*Yp*a1^2*Ya*Xp^2-4*Yp*a1^2*Xa^2*Ya-4*Yp^2*a1^2*Xp*Xa+12*Ya^2*Yp^2*Xp*Xa-8*Ya^3*Yp*Xp*Xa-4*Xp^2*Yp*Ya*b1^2+24*Xp^2*Yp*Xa^2*Ya-16*Xp^3*Yp*Xa*Ya-Ya^4*Xp^2-Ya^4*Xa^2-2*Xp^4*Yp^2-Xp^2*Yp^4-2*Ya^2*Xp^4-Xa^2*Yp^4+6*Xp*Xa^5+6*Xp^5*Xa+20*Xp^3*Xa^3-15*Xp^4*Xa^2-Xa^2*b1^4-Xa^2*a1^4-Xp^2*b1^4-Xp^2*a1^4+2*Xp^4*b1^2+2*Xp^4*a1^2+2*Yp^2*a1^2*Xp^2+2*Yp^2*a1^2*Xa^2-6*Ya^2*Yp^2*Xp^2+4*Ya^3*Yp*Xp^2+4*Ya^3*Yp*Xa^2-6*Ya^2*Yp^2*Xa^2+2*Ya^4*Xp*Xa+4*Xp^4*Yp*Ya+2*Xp^2*Yp^2*b1^2+4*Xp^2*Yp^3*Ya+8*Xp^3*Yp^2*Xa-12*Xp^2*Yp^2*Xa^2+2*Ya^2*b1^2*Xp^2+2*Ya^2*b1^2*Xa^2+2*Ya^2*a1^2*Xp^2+2*Ya^2*a1^2*Xa^2-12*Ya^2*Xp^2*Xa^2+2*b1^2*Yp^2*Xa^2+4*Ya*Yp^3*Xa^2+4*Xa^4*Ya*Yp+8*Xa^3*Ya^2*Xp+8*Xp*Xa^3*Yp^2+2*Xp*Xa*Yp^4-8*Xp*Xa^3*a1^2-8*Xp^3*Xa*b1^2-8*Xp^3*Xa*a1^2)^(1/2))*Ya+Xa^2+Ya^2-b1^2-Xp^2-Yp^2+a1^2)/(Xa-Xp);
      1/2*(1/(-8*Xp*Xa-8*Ya*Yp+4*Ya^2+4*Yp^2+4*Xa^2+4*Xp^2)*(-4*Yp*a1^2-4*Ya^2*Yp+4*Ya^3+4*Xp^2*Yp-4*Ya*b1^2+4*Ya*a1^2+4*Ya*Xp^2+4*b1^2*Yp-4*Ya*Yp^2+4*Xa^2*Ya-8*Xp*Xa*Yp+4*Xa^2*Yp-8*Xp*Xa*Ya+4*Yp^3-4*(-2*Xa^4*Yp^2+2*Xp*Xa*b1^4+2*Xa^4*b1^2+12*Xp^2*Xa^2*a1^2+2*Xa^4*a1^2-15*Xa^4*Xp^2-2*Xa^4*Ya^2+12*Xp^2*Xa^2*b1^2-8*Xp*Xa^3*b1^2+2*Xp*Xa*a1^4+2*Xa^2*b1^2*a1^2+2*Xp^2*b1^2*a1^2-16*Xa^3*Ya*Xp*Yp-4*Xp*Xa*b1^2*a1^2+8*Yp*a1^2*Xp*Xa*Ya-4*Ya*b1^2*Xa^2*Yp+8*Ya*b1^2*Xp*Xa*Yp-4*Ya^2*b1^2*Xp*Xa-4*Ya^2*a1^2*Xp*Xa-4*b1^2*Yp^2*Xp*Xa-8*Ya*Yp^3*Xp*Xa+8*Ya^2*Xp^3*Xa-Xa^6-Xp^6-4*Yp*a1^2*Ya*Xp^2-4*Yp*a1^2*Xa^2*Ya-4*Yp^2*a1^2*Xp*Xa+12*Ya^2*Yp^2*Xp*Xa-8*Ya^3*Yp*Xp*Xa-4*Xp^2*Yp*Ya*b1^2+24*Xp^2*Yp*Xa^2*Ya-16*Xp^3*Yp*Xa*Ya-Ya^4*Xp^2-Ya^4*Xa^2-2*Xp^4*Yp^2-Xp^2*Yp^4-2*Ya^2*Xp^4-Xa^2*Yp^4+6*Xp*Xa^5+6*Xp^5*Xa+20*Xp^3*Xa^3-15*Xp^4*Xa^2-Xa^2*b1^4-Xa^2*a1^4-Xp^2*b1^4-Xp^2*a1^4+2*Xp^4*b1^2+2*Xp^4*a1^2+2*Yp^2*a1^2*Xp^2+2*Yp^2*a1^2*Xa^2-6*Ya^2*Yp^2*Xp^2+4*Ya^3*Yp*Xp^2+4*Ya^3*Yp*Xa^2-6*Ya^2*Yp^2*Xa^2+2*Ya^4*Xp*Xa+4*Xp^4*Yp*Ya+2*Xp^2*Yp^2*b1^2+4*Xp^2*Yp^3*Ya+8*Xp^3*Yp^2*Xa-12*Xp^2*Yp^2*Xa^2+2*Ya^2*b1^2*Xp^2+2*Ya^2*b1^2*Xa^2+2*Ya^2*a1^2*Xp^2+2*Ya^2*a1^2*Xa^2-12*Ya^2*Xp^2*Xa^2+2*b1^2*Yp^2*Xa^2+4*Ya*Yp^3*Xa^2+4*Xa^4*Ya*Yp+8*Xa^3*Ya^2*Xp+8*Xp*Xa^3*Yp^2+2*Xp*Xa*Yp^4-8*Xp*Xa^3*a1^2-8*Xp^3*Xa*b1^2-8*Xp^3*Xa*a1^2)^(1/2))*Yp-1/(-8*Xp*Xa-8*Ya*Yp+4*Ya^2+4*Yp^2+4*Xa^2+4*Xp^2)*(-4*Yp*a1^2-4*Ya^2*Yp+4*Ya^3+4*Xp^2*Yp-4*Ya*b1^2+4*Ya*a1^2+4*Ya*Xp^2+4*b1^2*Yp-4*Ya*Yp^2+4*Xa^2*Ya-8*Xp*Xa*Yp+4*Xa^2*Yp-8*Xp*Xa*Ya+4*Yp^3-4*(-2*Xa^4*Yp^2+2*Xp*Xa*b1^4+2*Xa^4*b1^2+12*Xp^2*Xa^2*a1^2+2*Xa^4*a1^2-15*Xa^4*Xp^2-2*Xa^4*Ya^2+12*Xp^2*Xa^2*b1^2-8*Xp*Xa^3*b1^2+2*Xp*Xa*a1^4+2*Xa^2*b1^2*a1^2+2*Xp^2*b1^2*a1^2-16*Xa^3*Ya*Xp*Yp-4*Xp*Xa*b1^2*a1^2+8*Yp*a1^2*Xp*Xa*Ya-4*Ya*b1^2*Xa^2*Yp+8*Ya*b1^2*Xp*Xa*Yp-4*Ya^2*b1^2*Xp*Xa-4*Ya^2*a1^2*Xp*Xa-4*b1^2*Yp^2*Xp*Xa-8*Ya*Yp^3*Xp*Xa+8*Ya^2*Xp^3*Xa-Xa^6-Xp^6-4*Yp*a1^2*Ya*Xp^2-4*Yp*a1^2*Xa^2*Ya-4*Yp^2*a1^2*Xp*Xa+12*Ya^2*Yp^2*Xp*Xa-8*Ya^3*Yp*Xp*Xa-4*Xp^2*Yp*Ya*b1^2+24*Xp^2*Yp*Xa^2*Ya-16*Xp^3*Yp*Xa*Ya-Ya^4*Xp^2-Ya^4*Xa^2-2*Xp^4*Yp^2-Xp^2*Yp^4-2*Ya^2*Xp^4-Xa^2*Yp^4+6*Xp*Xa^5+6*Xp^5*Xa+20*Xp^3*Xa^3-15*Xp^4*Xa^2-Xa^2*b1^4-Xa^2*a1^4-Xp^2*b1^4-Xp^2*a1^4+2*Xp^4*b1^2+2*Xp^4*a1^2+2*Yp^2*a1^2*Xp^2+2*Yp^2*a1^2*Xa^2-6*Ya^2*Yp^2*Xp^2+4*Ya^3*Yp*Xp^2+4*Ya^3*Yp*Xa^2-6*Ya^2*Yp^2*Xa^2+2*Ya^4*Xp*Xa+4*Xp^4*Yp*Ya+2*Xp^2*Yp^2*b1^2+4*Xp^2*Yp^3*Ya+8*Xp^3*Yp^2*Xa-12*Xp^2*Yp^2*Xa^2+2*Ya^2*b1^2*Xp^2+2*Ya^2*b1^2*Xa^2+2*Ya^2*a1^2*Xp^2+2*Ya^2*a1^2*Xa^2-12*Ya^2*Xp^2*Xa^2+2*b1^2*Yp^2*Xa^2+4*Ya*Yp^3*Xa^2+4*Xa^4*Ya*Yp+8*Xa^3*Ya^2*Xp+8*Xp*Xa^3*Yp^2+2*Xp*Xa*Yp^4-8*Xp*Xa^3*a1^2-8*Xp^3*Xa*b1^2-8*Xp^3*Xa*a1^2)^(1/2))*Ya+Xa^2+Ya^2-b1^2-Xp^2-Yp^2+a1^2)/(Xa-Xp)];
Yd =[ 1/2/(-8*Xp*Xa-8*Ya*Yp+4*Ya^2+4*Yp^2+4*Xa^2+4*Xp^2)*(-4*Yp*a1^2-4*Ya^2*Yp+4*Ya^3+4*Xp^2*Yp-4*Ya*b1^2+4*Ya*a1^2+4*Ya*Xp^2+4*b1^2*Yp-4*Ya*Yp^2+4*Xa^2*Ya-8*Xp*Xa*Yp+4*Xa^2*Yp-8*Xp*Xa*Ya+4*Yp^3+4*(-2*Xa^4*Yp^2+2*Xp*Xa*b1^4+2*Xa^4*b1^2+12*Xp^2*Xa^2*a1^2+2*Xa^4*a1^2-15*Xa^4*Xp^2-2*Xa^4*Ya^2+12*Xp^2*Xa^2*b1^2-8*Xp*Xa^3*b1^2+2*Xp*Xa*a1^4+2*Xa^2*b1^2*a1^2+2*Xp^2*b1^2*a1^2-16*Xa^3*Ya*Xp*Yp-4*Xp*Xa*b1^2*a1^2+8*Yp*a1^2*Xp*Xa*Ya-4*Ya*b1^2*Xa^2*Yp+8*Ya*b1^2*Xp*Xa*Yp-4*Ya^2*b1^2*Xp*Xa-4*Ya^2*a1^2*Xp*Xa-4*b1^2*Yp^2*Xp*Xa-8*Ya*Yp^3*Xp*Xa+8*Ya^2*Xp^3*Xa-Xa^6-Xp^6-4*Yp*a1^2*Ya*Xp^2-4*Yp*a1^2*Xa^2*Ya-4*Yp^2*a1^2*Xp*Xa+12*Ya^2*Yp^2*Xp*Xa-8*Ya^3*Yp*Xp*Xa-4*Xp^2*Yp*Ya*b1^2+24*Xp^2*Yp*Xa^2*Ya-16*Xp^3*Yp*Xa*Ya-Ya^4*Xp^2-Ya^4*Xa^2-2*Xp^4*Yp^2-Xp^2*Yp^4-2*Ya^2*Xp^4-Xa^2*Yp^4+6*Xp*Xa^5+6*Xp^5*Xa+20*Xp^3*Xa^3-15*Xp^4*Xa^2-Xa^2*b1^4-Xa^2*a1^4-Xp^2*b1^4-Xp^2*a1^4+2*Xp^4*b1^2+2*Xp^4*a1^2+2*Yp^2*a1^2*Xp^2+2*Yp^2*a1^2*Xa^2-6*Ya^2*Yp^2*Xp^2+4*Ya^3*Yp*Xp^2+4*Ya^3*Yp*Xa^2-6*Ya^2*Yp^2*Xa^2+2*Ya^4*Xp*Xa+4*Xp^4*Yp*Ya+2*Xp^2*Yp^2*b1^2+4*Xp^2*Yp^3*Ya+8*Xp^3*Yp^2*Xa-12*Xp^2*Yp^2*Xa^2+2*Ya^2*b1^2*Xp^2+2*Ya^2*b1^2*Xa^2+2*Ya^2*a1^2*Xp^2+2*Ya^2*a1^2*Xa^2-12*Ya^2*Xp^2*Xa^2+2*b1^2*Yp^2*Xa^2+4*Ya*Yp^3*Xa^2+4*Xa^4*Ya*Yp+8*Xa^3*Ya^2*Xp+8*Xp*Xa^3*Yp^2+2*Xp*Xa*Yp^4-8*Xp*Xa^3*a1^2-8*Xp^3*Xa*b1^2-8*Xp^3*Xa*a1^2)^(1/2));
      1/2/(-8*Xp*Xa-8*Ya*Yp+4*Ya^2+4*Yp^2+4*Xa^2+4*Xp^2)*(-4*Yp*a1^2-4*Ya^2*Yp+4*Ya^3+4*Xp^2*Yp-4*Ya*b1^2+4*Ya*a1^2+4*Ya*Xp^2+4*b1^2*Yp-4*Ya*Yp^2+4*Xa^2*Ya-8*Xp*Xa*Yp+4*Xa^2*Yp-8*Xp*Xa*Ya+4*Yp^3-4*(-2*Xa^4*Yp^2+2*Xp*Xa*b1^4+2*Xa^4*b1^2+12*Xp^2*Xa^2*a1^2+2*Xa^4*a1^2-15*Xa^4*Xp^2-2*Xa^4*Ya^2+12*Xp^2*Xa^2*b1^2-8*Xp*Xa^3*b1^2+2*Xp*Xa*a1^4+2*Xa^2*b1^2*a1^2+2*Xp^2*b1^2*a1^2-16*Xa^3*Ya*Xp*Yp-4*Xp*Xa*b1^2*a1^2+8*Yp*a1^2*Xp*Xa*Ya-4*Ya*b1^2*Xa^2*Yp+8*Ya*b1^2*Xp*Xa*Yp-4*Ya^2*b1^2*Xp*Xa-4*Ya^2*a1^2*Xp*Xa-4*b1^2*Yp^2*Xp*Xa-8*Ya*Yp^3*Xp*Xa+8*Ya^2*Xp^3*Xa-Xa^6-Xp^6-4*Yp*a1^2*Ya*Xp^2-4*Yp*a1^2*Xa^2*Ya-4*Yp^2*a1^2*Xp*Xa+12*Ya^2*Yp^2*Xp*Xa-8*Ya^3*Yp*Xp*Xa-4*Xp^2*Yp*Ya*b1^2+24*Xp^2*Yp*Xa^2*Ya-16*Xp^3*Yp*Xa*Ya-Ya^4*Xp^2-Ya^4*Xa^2-2*Xp^4*Yp^2-Xp^2*Yp^4-2*Ya^2*Xp^4-Xa^2*Yp^4+6*Xp*Xa^5+6*Xp^5*Xa+20*Xp^3*Xa^3-15*Xp^4*Xa^2-Xa^2*b1^4-Xa^2*a1^4-Xp^2*b1^4-Xp^2*a1^4+2*Xp^4*b1^2+2*Xp^4*a1^2+2*Yp^2*a1^2*Xp^2+2*Yp^2*a1^2*Xa^2-6*Ya^2*Yp^2*Xp^2+4*Ya^3*Yp*Xp^2+4*Ya^3*Yp*Xa^2-6*Ya^2*Yp^2*Xa^2+2*Ya^4*Xp*Xa+4*Xp^4*Yp*Ya+2*Xp^2*Yp^2*b1^2+4*Xp^2*Yp^3*Ya+8*Xp^3*Yp^2*Xa-12*Xp^2*Yp^2*Xa^2+2*Ya^2*b1^2*Xp^2+2*Ya^2*b1^2*Xa^2+2*Ya^2*a1^2*Xp^2+2*Ya^2*a1^2*Xa^2-12*Ya^2*Xp^2*Xa^2+2*b1^2*Yp^2*Xa^2+4*Ya*Yp^3*Xa^2+4*Xa^4*Ya*Yp+8*Xa^3*Ya^2*Xp+8*Xp*Xa^3*Yp^2+2*Xp*Xa*Yp^4-8*Xp*Xa^3*a1^2-8*Xp^3*Xa*b1^2-8*Xp^3*Xa*a1^2)^(1/2))];
  


axis([-15 15 -15 15])

Xb = Xa+H*cos(fai);
Yb = Ya+H*sin(fai);


Xdb = [ -1/2*(-1/(-8*Ypb*Yb-8*Xb*Xpb+4*Ypb^2+4*Yb^2+4*Xpb^2+4*Xb^2)*(-8*Xb*Xpb*Yb+4*Ypb^3+4*Xb^2*Yb+4*Yb^3+4*Ypb*Xb^2+4*Xpb^2*Ypb-8*Xb*Xpb*Ypb+4*Ypb*b1^2+4*Xpb^2*Yb-4*Ypb*Yb^2+4*a1^2*Yb-4*Yb*b1^2-4*Ypb*a1^2-4*Ypb^2*Yb+4*(-8*Xb^3*Xpb*a1^2-8*Xb^3*Xpb*b1^2-4*Xb*Xpb*b1^2*a1^2+20*Xb^3*Xpb^3-15*Xb^2*Xpb^4+2*Xpb^2*Yb^2*a1^2+2*Xpb^2*Yb^2*b1^2-4*Xpb^2*Ypb*a1^2*Yb-4*Xpb^2*Ypb*Yb*b1^2-4*Xb*Xpb*Ypb^2*b1^2-4*Xb*Xpb*Ypb^2*a1^2+2*Ypb^2*Xb^2*b1^2+2*Ypb^2*Xb^2*a1^2+8*Xpb^3*Ypb^2*Xb+2*Xpb^2*Ypb^2*b1^2+4*Xpb^4*Ypb*Yb-6*Xpb^2*Ypb^2*Yb^2+2*Xpb^2*Ypb^2*a1^2-2*Ypb^2*Xb^4-2*Xpb^4*Ypb^2-2*Xpb^4*Yb^2+4*Xb^2*Yb^3*Ypb+2*Xb^2*Yb^2*a1^2+2*Xb^2*Yb^2*b1^2-6*Xb^2*Yb^2*Ypb^2+4*Yb^3*Xpb^2*Ypb-12*Ypb^2*Xb^2*Xpb^2+8*Ypb^2*Xb^3*Xpb-4*Xb^2*Yb*Ypb*b1^2-4*Xb^2*Yb*Ypb*a1^2-2*Xb^4*Yb^2-Xb^2*Yb^4-Yb^4*Xpb^2+2*Ypb^4*Xb*Xpb+4*Ypb^3*Xpb^2*Yb+4*Xb^4*Yb*Ypb+24*Xb^2*Xpb^2*Yb*Ypb-Ypb^4*Xpb^2+8*Xb*Xpb*Yb*Ypb*b1^2-8*Xb*Xpb*Yb^3*Ypb+8*Xb*Xpb*Yb*Ypb*a1^2-4*Xb*Xpb*Yb^2*a1^2-4*Xb*Xpb*Yb^2*b1^2+12*Xb*Xpb*Yb^2*Ypb^2-16*Xb*Xpb^3*Yb*Ypb-Xb^6-12*Xb^2*Xpb^2*Yb^2+8*Xb^3*Xpb*Yb^2+2*Xb*Xpb*Yb^4+8*Xb*Xpb^3*Yb^2+4*Ypb^3*Xb^2*Yb-Xpb^6-Ypb^4*Xb^2-8*Xb*Xpb*Yb*Ypb^3-16*Xb^3*Xpb*Yb*Ypb+2*Xb^4*b1^2+2*Xb^2*b1^2*a1^2+2*Xpb^4*a1^2+2*Xpb^4*b1^2-Xb^2*b1^4-Xb^2*a1^4+2*Xb^4*a1^2-Xpb^2*b1^4-Xpb^2*a1^4+2*Xpb^2*b1^2*a1^2-15*Xb^4*Xpb^2+6*Xb*Xpb^5+6*Xb^5*Xpb+12*Xb^2*Xpb^2*b1^2+12*Xb^2*Xpb^2*a1^2+2*Xb*Xpb*b1^4+2*Xb*Xpb*a1^4-8*Xb*Xpb^3*a1^2-8*Xb*Xpb^3*b1^2)^(1/2))*Ypb+1/(-8*Ypb*Yb-8*Xb*Xpb+4*Ypb^2+4*Yb^2+4*Xpb^2+4*Xb^2)*(-8*Xb*Xpb*Yb+4*Ypb^3+4*Xb^2*Yb+4*Yb^3+4*Ypb*Xb^2+4*Xpb^2*Ypb-8*Xb*Xpb*Ypb+4*Ypb*b1^2+4*Xpb^2*Yb-4*Ypb*Yb^2+4*a1^2*Yb-4*Yb*b1^2-4*Ypb*a1^2-4*Ypb^2*Yb+4*(-8*Xb^3*Xpb*a1^2-8*Xb^3*Xpb*b1^2-4*Xb*Xpb*b1^2*a1^2+20*Xb^3*Xpb^3-15*Xb^2*Xpb^4+2*Xpb^2*Yb^2*a1^2+2*Xpb^2*Yb^2*b1^2-4*Xpb^2*Ypb*a1^2*Yb-4*Xpb^2*Ypb*Yb*b1^2-4*Xb*Xpb*Ypb^2*b1^2-4*Xb*Xpb*Ypb^2*a1^2+2*Ypb^2*Xb^2*b1^2+2*Ypb^2*Xb^2*a1^2+8*Xpb^3*Ypb^2*Xb+2*Xpb^2*Ypb^2*b1^2+4*Xpb^4*Ypb*Yb-6*Xpb^2*Ypb^2*Yb^2+2*Xpb^2*Ypb^2*a1^2-2*Ypb^2*Xb^4-2*Xpb^4*Ypb^2-2*Xpb^4*Yb^2+4*Xb^2*Yb^3*Ypb+2*Xb^2*Yb^2*a1^2+2*Xb^2*Yb^2*b1^2-6*Xb^2*Yb^2*Ypb^2+4*Yb^3*Xpb^2*Ypb-12*Ypb^2*Xb^2*Xpb^2+8*Ypb^2*Xb^3*Xpb-4*Xb^2*Yb*Ypb*b1^2-4*Xb^2*Yb*Ypb*a1^2-2*Xb^4*Yb^2-Xb^2*Yb^4-Yb^4*Xpb^2+2*Ypb^4*Xb*Xpb+4*Ypb^3*Xpb^2*Yb+4*Xb^4*Yb*Ypb+24*Xb^2*Xpb^2*Yb*Ypb-Ypb^4*Xpb^2+8*Xb*Xpb*Yb*Ypb*b1^2-8*Xb*Xpb*Yb^3*Ypb+8*Xb*Xpb*Yb*Ypb*a1^2-4*Xb*Xpb*Yb^2*a1^2-4*Xb*Xpb*Yb^2*b1^2+12*Xb*Xpb*Yb^2*Ypb^2-16*Xb*Xpb^3*Yb*Ypb-Xb^6-12*Xb^2*Xpb^2*Yb^2+8*Xb^3*Xpb*Yb^2+2*Xb*Xpb*Yb^4+8*Xb*Xpb^3*Yb^2+4*Ypb^3*Xb^2*Yb-Xpb^6-Ypb^4*Xb^2-8*Xb*Xpb*Yb*Ypb^3-16*Xb^3*Xpb*Yb*Ypb+2*Xb^4*b1^2+2*Xb^2*b1^2*a1^2+2*Xpb^4*a1^2+2*Xpb^4*b1^2-Xb^2*b1^4-Xb^2*a1^4+2*Xb^4*a1^2-Xpb^2*b1^4-Xpb^2*a1^4+2*Xpb^2*b1^2*a1^2-15*Xb^4*Xpb^2+6*Xb*Xpb^5+6*Xb^5*Xpb+12*Xb^2*Xpb^2*b1^2+12*Xb^2*Xpb^2*a1^2+2*Xb*Xpb*b1^4+2*Xb*Xpb*a1^4-8*Xb*Xpb^3*a1^2-8*Xb*Xpb^3*b1^2)^(1/2))*Yb+Xpb^2+Ypb^2-a1^2-Xb^2-Yb^2+b1^2)/(-Xpb+Xb);
        -1/2*(-1/(-8*Ypb*Yb-8*Xb*Xpb+4*Ypb^2+4*Yb^2+4*Xpb^2+4*Xb^2)*(-8*Xb*Xpb*Yb+4*Ypb^3+4*Xb^2*Yb+4*Yb^3+4*Ypb*Xb^2+4*Xpb^2*Ypb-8*Xb*Xpb*Ypb+4*Ypb*b1^2+4*Xpb^2*Yb-4*Ypb*Yb^2+4*a1^2*Yb-4*Yb*b1^2-4*Ypb*a1^2-4*Ypb^2*Yb-4*(-8*Xb^3*Xpb*a1^2-8*Xb^3*Xpb*b1^2-4*Xb*Xpb*b1^2*a1^2+20*Xb^3*Xpb^3-15*Xb^2*Xpb^4+2*Xpb^2*Yb^2*a1^2+2*Xpb^2*Yb^2*b1^2-4*Xpb^2*Ypb*a1^2*Yb-4*Xpb^2*Ypb*Yb*b1^2-4*Xb*Xpb*Ypb^2*b1^2-4*Xb*Xpb*Ypb^2*a1^2+2*Ypb^2*Xb^2*b1^2+2*Ypb^2*Xb^2*a1^2+8*Xpb^3*Ypb^2*Xb+2*Xpb^2*Ypb^2*b1^2+4*Xpb^4*Ypb*Yb-6*Xpb^2*Ypb^2*Yb^2+2*Xpb^2*Ypb^2*a1^2-2*Ypb^2*Xb^4-2*Xpb^4*Ypb^2-2*Xpb^4*Yb^2+4*Xb^2*Yb^3*Ypb+2*Xb^2*Yb^2*a1^2+2*Xb^2*Yb^2*b1^2-6*Xb^2*Yb^2*Ypb^2+4*Yb^3*Xpb^2*Ypb-12*Ypb^2*Xb^2*Xpb^2+8*Ypb^2*Xb^3*Xpb-4*Xb^2*Yb*Ypb*b1^2-4*Xb^2*Yb*Ypb*a1^2-2*Xb^4*Yb^2-Xb^2*Yb^4-Yb^4*Xpb^2+2*Ypb^4*Xb*Xpb+4*Ypb^3*Xpb^2*Yb+4*Xb^4*Yb*Ypb+24*Xb^2*Xpb^2*Yb*Ypb-Ypb^4*Xpb^2+8*Xb*Xpb*Yb*Ypb*b1^2-8*Xb*Xpb*Yb^3*Ypb+8*Xb*Xpb*Yb*Ypb*a1^2-4*Xb*Xpb*Yb^2*a1^2-4*Xb*Xpb*Yb^2*b1^2+12*Xb*Xpb*Yb^2*Ypb^2-16*Xb*Xpb^3*Yb*Ypb-Xb^6-12*Xb^2*Xpb^2*Yb^2+8*Xb^3*Xpb*Yb^2+2*Xb*Xpb*Yb^4+8*Xb*Xpb^3*Yb^2+4*Ypb^3*Xb^2*Yb-Xpb^6-Ypb^4*Xb^2-8*Xb*Xpb*Yb*Ypb^3-16*Xb^3*Xpb*Yb*Ypb+2*Xb^4*b1^2+2*Xb^2*b1^2*a1^2+2*Xpb^4*a1^2+2*Xpb^4*b1^2-Xb^2*b1^4-Xb^2*a1^4+2*Xb^4*a1^2-Xpb^2*b1^4-Xpb^2*a1^4+2*Xpb^2*b1^2*a1^2-15*Xb^4*Xpb^2+6*Xb*Xpb^5+6*Xb^5*Xpb+12*Xb^2*Xpb^2*b1^2+12*Xb^2*Xpb^2*a1^2+2*Xb*Xpb*b1^4+2*Xb*Xpb*a1^4-8*Xb*Xpb^3*a1^2-8*Xb*Xpb^3*b1^2)^(1/2))*Ypb+1/(-8*Ypb*Yb-8*Xb*Xpb+4*Ypb^2+4*Yb^2+4*Xpb^2+4*Xb^2)*(-8*Xb*Xpb*Yb+4*Ypb^3+4*Xb^2*Yb+4*Yb^3+4*Ypb*Xb^2+4*Xpb^2*Ypb-8*Xb*Xpb*Ypb+4*Ypb*b1^2+4*Xpb^2*Yb-4*Ypb*Yb^2+4*a1^2*Yb-4*Yb*b1^2-4*Ypb*a1^2-4*Ypb^2*Yb-4*(-8*Xb^3*Xpb*a1^2-8*Xb^3*Xpb*b1^2-4*Xb*Xpb*b1^2*a1^2+20*Xb^3*Xpb^3-15*Xb^2*Xpb^4+2*Xpb^2*Yb^2*a1^2+2*Xpb^2*Yb^2*b1^2-4*Xpb^2*Ypb*a1^2*Yb-4*Xpb^2*Ypb*Yb*b1^2-4*Xb*Xpb*Ypb^2*b1^2-4*Xb*Xpb*Ypb^2*a1^2+2*Ypb^2*Xb^2*b1^2+2*Ypb^2*Xb^2*a1^2+8*Xpb^3*Ypb^2*Xb+2*Xpb^2*Ypb^2*b1^2+4*Xpb^4*Ypb*Yb-6*Xpb^2*Ypb^2*Yb^2+2*Xpb^2*Ypb^2*a1^2-2*Ypb^2*Xb^4-2*Xpb^4*Ypb^2-2*Xpb^4*Yb^2+4*Xb^2*Yb^3*Ypb+2*Xb^2*Yb^2*a1^2+2*Xb^2*Yb^2*b1^2-6*Xb^2*Yb^2*Ypb^2+4*Yb^3*Xpb^2*Ypb-12*Ypb^2*Xb^2*Xpb^2+8*Ypb^2*Xb^3*Xpb-4*Xb^2*Yb*Ypb*b1^2-4*Xb^2*Yb*Ypb*a1^2-2*Xb^4*Yb^2-Xb^2*Yb^4-Yb^4*Xpb^2+2*Ypb^4*Xb*Xpb+4*Ypb^3*Xpb^2*Yb+4*Xb^4*Yb*Ypb+24*Xb^2*Xpb^2*Yb*Ypb-Ypb^4*Xpb^2+8*Xb*Xpb*Yb*Ypb*b1^2-8*Xb*Xpb*Yb^3*Ypb+8*Xb*Xpb*Yb*Ypb*a1^2-4*Xb*Xpb*Yb^2*a1^2-4*Xb*Xpb*Yb^2*b1^2+12*Xb*Xpb*Yb^2*Ypb^2-16*Xb*Xpb^3*Yb*Ypb-Xb^6-12*Xb^2*Xpb^2*Yb^2+8*Xb^3*Xpb*Yb^2+2*Xb*Xpb*Yb^4+8*Xb*Xpb^3*Yb^2+4*Ypb^3*Xb^2*Yb-Xpb^6-Ypb^4*Xb^2-8*Xb*Xpb*Yb*Ypb^3-16*Xb^3*Xpb*Yb*Ypb+2*Xb^4*b1^2+2*Xb^2*b1^2*a1^2+2*Xpb^4*a1^2+2*Xpb^4*b1^2-Xb^2*b1^4-Xb^2*a1^4+2*Xb^4*a1^2-Xpb^2*b1^4-Xpb^2*a1^4+2*Xpb^2*b1^2*a1^2-15*Xb^4*Xpb^2+6*Xb*Xpb^5+6*Xb^5*Xpb+12*Xb^2*Xpb^2*b1^2+12*Xb^2*Xpb^2*a1^2+2*Xb*Xpb*b1^4+2*Xb*Xpb*a1^4-8*Xb*Xpb^3*a1^2-8*Xb*Xpb^3*b1^2)^(1/2))*Yb+Xpb^2+Ypb^2-a1^2-Xb^2-Yb^2+b1^2)/(-Xpb+Xb)];

Ydb = [ 1/2/(-8*Ypb*Yb-8*Xb*Xpb+4*Ypb^2+4*Yb^2+4*Xpb^2+4*Xb^2)*(-8*Xb*Xpb*Yb+4*Ypb^3+4*Xb^2*Yb+4*Yb^3+4*Ypb*Xb^2+4*Xpb^2*Ypb-8*Xb*Xpb*Ypb+4*Ypb*b1^2+4*Xpb^2*Yb-4*Ypb*Yb^2+4*a1^2*Yb-4*Yb*b1^2-4*Ypb*a1^2-4*Ypb^2*Yb+4*(-8*Xb^3*Xpb*a1^2-8*Xb^3*Xpb*b1^2-4*Xb*Xpb*b1^2*a1^2+20*Xb^3*Xpb^3-15*Xb^2*Xpb^4+2*Xpb^2*Yb^2*a1^2+2*Xpb^2*Yb^2*b1^2-4*Xpb^2*Ypb*a1^2*Yb-4*Xpb^2*Ypb*Yb*b1^2-4*Xb*Xpb*Ypb^2*b1^2-4*Xb*Xpb*Ypb^2*a1^2+2*Ypb^2*Xb^2*b1^2+2*Ypb^2*Xb^2*a1^2+8*Xpb^3*Ypb^2*Xb+2*Xpb^2*Ypb^2*b1^2+4*Xpb^4*Ypb*Yb-6*Xpb^2*Ypb^2*Yb^2+2*Xpb^2*Ypb^2*a1^2-2*Ypb^2*Xb^4-2*Xpb^4*Ypb^2-2*Xpb^4*Yb^2+4*Xb^2*Yb^3*Ypb+2*Xb^2*Yb^2*a1^2+2*Xb^2*Yb^2*b1^2-6*Xb^2*Yb^2*Ypb^2+4*Yb^3*Xpb^2*Ypb-12*Ypb^2*Xb^2*Xpb^2+8*Ypb^2*Xb^3*Xpb-4*Xb^2*Yb*Ypb*b1^2-4*Xb^2*Yb*Ypb*a1^2-2*Xb^4*Yb^2-Xb^2*Yb^4-Yb^4*Xpb^2+2*Ypb^4*Xb*Xpb+4*Ypb^3*Xpb^2*Yb+4*Xb^4*Yb*Ypb+24*Xb^2*Xpb^2*Yb*Ypb-Ypb^4*Xpb^2+8*Xb*Xpb*Yb*Ypb*b1^2-8*Xb*Xpb*Yb^3*Ypb+8*Xb*Xpb*Yb*Ypb*a1^2-4*Xb*Xpb*Yb^2*a1^2-4*Xb*Xpb*Yb^2*b1^2+12*Xb*Xpb*Yb^2*Ypb^2-16*Xb*Xpb^3*Yb*Ypb-Xb^6-12*Xb^2*Xpb^2*Yb^2+8*Xb^3*Xpb*Yb^2+2*Xb*Xpb*Yb^4+8*Xb*Xpb^3*Yb^2+4*Ypb^3*Xb^2*Yb-Xpb^6-Ypb^4*Xb^2-8*Xb*Xpb*Yb*Ypb^3-16*Xb^3*Xpb*Yb*Ypb+2*Xb^4*b1^2+2*Xb^2*b1^2*a1^2+2*Xpb^4*a1^2+2*Xpb^4*b1^2-Xb^2*b1^4-Xb^2*a1^4+2*Xb^4*a1^2-Xpb^2*b1^4-Xpb^2*a1^4+2*Xpb^2*b1^2*a1^2-15*Xb^4*Xpb^2+6*Xb*Xpb^5+6*Xb^5*Xpb+12*Xb^2*Xpb^2*b1^2+12*Xb^2*Xpb^2*a1^2+2*Xb*Xpb*b1^4+2*Xb*Xpb*a1^4-8*Xb*Xpb^3*a1^2-8*Xb*Xpb^3*b1^2)^(1/2));
        1/2/(-8*Ypb*Yb-8*Xb*Xpb+4*Ypb^2+4*Yb^2+4*Xpb^2+4*Xb^2)*(-8*Xb*Xpb*Yb+4*Ypb^3+4*Xb^2*Yb+4*Yb^3+4*Ypb*Xb^2+4*Xpb^2*Ypb-8*Xb*Xpb*Ypb+4*Ypb*b1^2+4*Xpb^2*Yb-4*Ypb*Yb^2+4*a1^2*Yb-4*Yb*b1^2-4*Ypb*a1^2-4*Ypb^2*Yb-4*(-8*Xb^3*Xpb*a1^2-8*Xb^3*Xpb*b1^2-4*Xb*Xpb*b1^2*a1^2+20*Xb^3*Xpb^3-15*Xb^2*Xpb^4+2*Xpb^2*Yb^2*a1^2+2*Xpb^2*Yb^2*b1^2-4*Xpb^2*Ypb*a1^2*Yb-4*Xpb^2*Ypb*Yb*b1^2-4*Xb*Xpb*Ypb^2*b1^2-4*Xb*Xpb*Ypb^2*a1^2+2*Ypb^2*Xb^2*b1^2+2*Ypb^2*Xb^2*a1^2+8*Xpb^3*Ypb^2*Xb+2*Xpb^2*Ypb^2*b1^2+4*Xpb^4*Ypb*Yb-6*Xpb^2*Ypb^2*Yb^2+2*Xpb^2*Ypb^2*a1^2-2*Ypb^2*Xb^4-2*Xpb^4*Ypb^2-2*Xpb^4*Yb^2+4*Xb^2*Yb^3*Ypb+2*Xb^2*Yb^2*a1^2+2*Xb^2*Yb^2*b1^2-6*Xb^2*Yb^2*Ypb^2+4*Yb^3*Xpb^2*Ypb-12*Ypb^2*Xb^2*Xpb^2+8*Ypb^2*Xb^3*Xpb-4*Xb^2*Yb*Ypb*b1^2-4*Xb^2*Yb*Ypb*a1^2-2*Xb^4*Yb^2-Xb^2*Yb^4-Yb^4*Xpb^2+2*Ypb^4*Xb*Xpb+4*Ypb^3*Xpb^2*Yb+4*Xb^4*Yb*Ypb+24*Xb^2*Xpb^2*Yb*Ypb-Ypb^4*Xpb^2+8*Xb*Xpb*Yb*Ypb*b1^2-8*Xb*Xpb*Yb^3*Ypb+8*Xb*Xpb*Yb*Ypb*a1^2-4*Xb*Xpb*Yb^2*a1^2-4*Xb*Xpb*Yb^2*b1^2+12*Xb*Xpb*Yb^2*Ypb^2-16*Xb*Xpb^3*Yb*Ypb-Xb^6-12*Xb^2*Xpb^2*Yb^2+8*Xb^3*Xpb*Yb^2+2*Xb*Xpb*Yb^4+8*Xb*Xpb^3*Yb^2+4*Ypb^3*Xb^2*Yb-Xpb^6-Ypb^4*Xb^2-8*Xb*Xpb*Yb*Ypb^3-16*Xb^3*Xpb*Yb*Ypb+2*Xb^4*b1^2+2*Xb^2*b1^2*a1^2+2*Xpb^4*a1^2+2*Xpb^4*b1^2-Xb^2*b1^4-Xb^2*a1^4+2*Xb^4*a1^2-Xpb^2*b1^4-Xpb^2*a1^4+2*Xpb^2*b1^2*a1^2-15*Xb^4*Xpb^2+6*Xb*Xpb^5+6*Xb^5*Xpb+12*Xb^2*Xpb^2*b1^2+12*Xb^2*Xpb^2*a1^2+2*Xb*Xpb*b1^4+2*Xb*Xpb*a1^4-8*Xb*Xpb^3*a1^2-8*Xb*Xpb^3*b1^2)^(1/2))];

axis([-15 15 -15 15])


Xc = Xa+H*cos(fai+pi/3);
Yc = Ya+H*sin(fai+pi/3);

Xdc = [ -1/2*(1/(-8*Yc*Ypc-8*Xpc*Xc+4*Yc^2+4*Ypc^2+4*Xc^2+4*Xpc^2)*(-4*Yc*Ypc^2+4*Yc^3-8*Xpc*Xc*Ypc+4*Ypc^3-4*Ypc*a1^2-8*Xpc*Xc*Yc+4*Yc*Xpc^2+4*Yc*a1^2+4*Xc^2*Yc-4*Yc*b1^2-4*Yc^2*Ypc+4*b1^2*Ypc+4*Xpc^2*Ypc+4*Xc^2*Ypc+4*(-6*Yc^2*Ypc^2*Xc^2+2*b1^2*Ypc^2*Xpc^2+2*b1^2*Ypc^2*Xc^2+4*Yc*Xpc^4*Ypc+2*Yc^2*a1^2*Xc^2+2*Xc^2*Yc^2*b1^2+4*Xc^4*Yc*Ypc-4*Ypc*a1^2*Xc^2*Yc-4*Xpc*Xc*Yc^2*a1^2-4*Xpc*Xc*Yc^2*b1^2-4*Yc*Xpc^2*b1^2*Ypc-4*Xc^2*Yc*b1^2*Ypc+8*Xpc*Xc*Ypc*Yc*a1^2+8*Xpc*Xc*Ypc*Yc*b1^2+2*Ypc^2*a1^2*Xpc^2+2*Ypc^2*a1^2*Xc^2-12*Xpc^2*Xc^2*Yc^2+8*Xpc^3*Xc*Yc^2+8*Xpc*Xc^3*Yc^2+2*Yc^2*Xpc^2*a1^2+2*Yc^2*Xpc^2*b1^2+24*Xpc^2*Xc^2*Ypc*Yc-16*Xpc^3*Xc*Ypc*Yc-16*Xpc*Xc^3*Ypc*Yc-4*Xpc*Xc*Ypc^2*b1^2-4*Ypc*a1^2*Yc*Xpc^2-8*Yc*Ypc^3*Xpc*Xc+12*Yc^2*Ypc^2*Xpc*Xc-8*Yc^3*Xpc*Xc*Ypc-4*Xpc*Xc*Ypc^2*a1^2-6*Yc^2*Ypc^2*Xpc^2+4*Yc*Ypc^3*Xpc^2+4*Yc*Ypc^3*Xc^2+2*Yc^4*Xpc*Xc-Xc^6-Xpc^6-Yc^4*Xpc^2-Yc^4*Xc^2-Ypc^4*Xpc^2-Ypc^4*Xc^2-2*Yc^2*Xpc^4-2*Xc^4*Yc^2-2*Xpc^4*Ypc^2-2*Xc^4*Ypc^2+6*Xpc*Xc^5+6*Xpc^5*Xc+20*Xpc^3*Xc^3-15*Xpc^2*Xc^4-15*Xpc^4*Xc^2-Xc^2*b1^4-Xc^2*a1^4+2*Xc^4*b1^2+2*Xc^4*a1^2-Xpc^2*b1^4-Xpc^2*a1^4+2*Xpc^4*b1^2+2*Xpc^4*a1^2+4*Yc^3*Xpc^2*Ypc+4*Yc^3*Xc^2*Ypc-12*Xpc^2*Xc^2*Ypc^2+2*Xpc*Xc*Ypc^4+8*Xpc^3*Xc*Ypc^2+8*Xpc*Xc^3*Ypc^2+2*Xpc*Xc*b1^4+2*Xpc*Xc*a1^4+12*Xpc^2*Xc^2*b1^2+12*Xpc^2*Xc^2*a1^2-8*Xpc*Xc^3*b1^2-8*Xpc*Xc^3*a1^2-8*Xpc^3*Xc*b1^2-8*Xpc^3*Xc*a1^2-4*Xpc*Xc*b1^2*a1^2+2*Xc^2*b1^2*a1^2+2*Xpc^2*b1^2*a1^2)^(1/2))*Yc-1/(-8*Yc*Ypc-8*Xpc*Xc+4*Yc^2+4*Ypc^2+4*Xc^2+4*Xpc^2)*(-4*Yc*Ypc^2+4*Yc^3-8*Xpc*Xc*Ypc+4*Ypc^3-4*Ypc*a1^2-8*Xpc*Xc*Yc+4*Yc*Xpc^2+4*Yc*a1^2+4*Xc^2*Yc-4*Yc*b1^2-4*Yc^2*Ypc+4*b1^2*Ypc+4*Xpc^2*Ypc+4*Xc^2*Ypc+4*(-6*Yc^2*Ypc^2*Xc^2+2*b1^2*Ypc^2*Xpc^2+2*b1^2*Ypc^2*Xc^2+4*Yc*Xpc^4*Ypc+2*Yc^2*a1^2*Xc^2+2*Xc^2*Yc^2*b1^2+4*Xc^4*Yc*Ypc-4*Ypc*a1^2*Xc^2*Yc-4*Xpc*Xc*Yc^2*a1^2-4*Xpc*Xc*Yc^2*b1^2-4*Yc*Xpc^2*b1^2*Ypc-4*Xc^2*Yc*b1^2*Ypc+8*Xpc*Xc*Ypc*Yc*a1^2+8*Xpc*Xc*Ypc*Yc*b1^2+2*Ypc^2*a1^2*Xpc^2+2*Ypc^2*a1^2*Xc^2-12*Xpc^2*Xc^2*Yc^2+8*Xpc^3*Xc*Yc^2+8*Xpc*Xc^3*Yc^2+2*Yc^2*Xpc^2*a1^2+2*Yc^2*Xpc^2*b1^2+24*Xpc^2*Xc^2*Ypc*Yc-16*Xpc^3*Xc*Ypc*Yc-16*Xpc*Xc^3*Ypc*Yc-4*Xpc*Xc*Ypc^2*b1^2-4*Ypc*a1^2*Yc*Xpc^2-8*Yc*Ypc^3*Xpc*Xc+12*Yc^2*Ypc^2*Xpc*Xc-8*Yc^3*Xpc*Xc*Ypc-4*Xpc*Xc*Ypc^2*a1^2-6*Yc^2*Ypc^2*Xpc^2+4*Yc*Ypc^3*Xpc^2+4*Yc*Ypc^3*Xc^2+2*Yc^4*Xpc*Xc-Xc^6-Xpc^6-Yc^4*Xpc^2-Yc^4*Xc^2-Ypc^4*Xpc^2-Ypc^4*Xc^2-2*Yc^2*Xpc^4-2*Xc^4*Yc^2-2*Xpc^4*Ypc^2-2*Xc^4*Ypc^2+6*Xpc*Xc^5+6*Xpc^5*Xc+20*Xpc^3*Xc^3-15*Xpc^2*Xc^4-15*Xpc^4*Xc^2-Xc^2*b1^4-Xc^2*a1^4+2*Xc^4*b1^2+2*Xc^4*a1^2-Xpc^2*b1^4-Xpc^2*a1^4+2*Xpc^4*b1^2+2*Xpc^4*a1^2+4*Yc^3*Xpc^2*Ypc+4*Yc^3*Xc^2*Ypc-12*Xpc^2*Xc^2*Ypc^2+2*Xpc*Xc*Ypc^4+8*Xpc^3*Xc*Ypc^2+8*Xpc*Xc^3*Ypc^2+2*Xpc*Xc*b1^4+2*Xpc*Xc*a1^4+12*Xpc^2*Xc^2*b1^2+12*Xpc^2*Xc^2*a1^2-8*Xpc*Xc^3*b1^2-8*Xpc*Xc^3*a1^2-8*Xpc^3*Xc*b1^2-8*Xpc^3*Xc*a1^2-4*Xpc*Xc*b1^2*a1^2+2*Xc^2*b1^2*a1^2+2*Xpc^2*b1^2*a1^2)^(1/2))*Ypc-Xc^2-Yc^2+b1^2+Xpc^2+Ypc^2-a1^2)/(Xc-Xpc);
        -1/2*(1/(-8*Yc*Ypc-8*Xpc*Xc+4*Yc^2+4*Ypc^2+4*Xc^2+4*Xpc^2)*(-4*Yc*Ypc^2+4*Yc^3-8*Xpc*Xc*Ypc+4*Ypc^3-4*Ypc*a1^2-8*Xpc*Xc*Yc+4*Yc*Xpc^2+4*Yc*a1^2+4*Xc^2*Yc-4*Yc*b1^2-4*Yc^2*Ypc+4*b1^2*Ypc+4*Xpc^2*Ypc+4*Xc^2*Ypc-4*(-6*Yc^2*Ypc^2*Xc^2+2*b1^2*Ypc^2*Xpc^2+2*b1^2*Ypc^2*Xc^2+4*Yc*Xpc^4*Ypc+2*Yc^2*a1^2*Xc^2+2*Xc^2*Yc^2*b1^2+4*Xc^4*Yc*Ypc-4*Ypc*a1^2*Xc^2*Yc-4*Xpc*Xc*Yc^2*a1^2-4*Xpc*Xc*Yc^2*b1^2-4*Yc*Xpc^2*b1^2*Ypc-4*Xc^2*Yc*b1^2*Ypc+8*Xpc*Xc*Ypc*Yc*a1^2+8*Xpc*Xc*Ypc*Yc*b1^2+2*Ypc^2*a1^2*Xpc^2+2*Ypc^2*a1^2*Xc^2-12*Xpc^2*Xc^2*Yc^2+8*Xpc^3*Xc*Yc^2+8*Xpc*Xc^3*Yc^2+2*Yc^2*Xpc^2*a1^2+2*Yc^2*Xpc^2*b1^2+24*Xpc^2*Xc^2*Ypc*Yc-16*Xpc^3*Xc*Ypc*Yc-16*Xpc*Xc^3*Ypc*Yc-4*Xpc*Xc*Ypc^2*b1^2-4*Ypc*a1^2*Yc*Xpc^2-8*Yc*Ypc^3*Xpc*Xc+12*Yc^2*Ypc^2*Xpc*Xc-8*Yc^3*Xpc*Xc*Ypc-4*Xpc*Xc*Ypc^2*a1^2-6*Yc^2*Ypc^2*Xpc^2+4*Yc*Ypc^3*Xpc^2+4*Yc*Ypc^3*Xc^2+2*Yc^4*Xpc*Xc-Xc^6-Xpc^6-Yc^4*Xpc^2-Yc^4*Xc^2-Ypc^4*Xpc^2-Ypc^4*Xc^2-2*Yc^2*Xpc^4-2*Xc^4*Yc^2-2*Xpc^4*Ypc^2-2*Xc^4*Ypc^2+6*Xpc*Xc^5+6*Xpc^5*Xc+20*Xpc^3*Xc^3-15*Xpc^2*Xc^4-15*Xpc^4*Xc^2-Xc^2*b1^4-Xc^2*a1^4+2*Xc^4*b1^2+2*Xc^4*a1^2-Xpc^2*b1^4-Xpc^2*a1^4+2*Xpc^4*b1^2+2*Xpc^4*a1^2+4*Yc^3*Xpc^2*Ypc+4*Yc^3*Xc^2*Ypc-12*Xpc^2*Xc^2*Ypc^2+2*Xpc*Xc*Ypc^4+8*Xpc^3*Xc*Ypc^2+8*Xpc*Xc^3*Ypc^2+2*Xpc*Xc*b1^4+2*Xpc*Xc*a1^4+12*Xpc^2*Xc^2*b1^2+12*Xpc^2*Xc^2*a1^2-8*Xpc*Xc^3*b1^2-8*Xpc*Xc^3*a1^2-8*Xpc^3*Xc*b1^2-8*Xpc^3*Xc*a1^2-4*Xpc*Xc*b1^2*a1^2+2*Xc^2*b1^2*a1^2+2*Xpc^2*b1^2*a1^2)^(1/2))*Yc-1/(-8*Yc*Ypc-8*Xpc*Xc+4*Yc^2+4*Ypc^2+4*Xc^2+4*Xpc^2)*(-4*Yc*Ypc^2+4*Yc^3-8*Xpc*Xc*Ypc+4*Ypc^3-4*Ypc*a1^2-8*Xpc*Xc*Yc+4*Yc*Xpc^2+4*Yc*a1^2+4*Xc^2*Yc-4*Yc*b1^2-4*Yc^2*Ypc+4*b1^2*Ypc+4*Xpc^2*Ypc+4*Xc^2*Ypc-4*(-6*Yc^2*Ypc^2*Xc^2+2*b1^2*Ypc^2*Xpc^2+2*b1^2*Ypc^2*Xc^2+4*Yc*Xpc^4*Ypc+2*Yc^2*a1^2*Xc^2+2*Xc^2*Yc^2*b1^2+4*Xc^4*Yc*Ypc-4*Ypc*a1^2*Xc^2*Yc-4*Xpc*Xc*Yc^2*a1^2-4*Xpc*Xc*Yc^2*b1^2-4*Yc*Xpc^2*b1^2*Ypc-4*Xc^2*Yc*b1^2*Ypc+8*Xpc*Xc*Ypc*Yc*a1^2+8*Xpc*Xc*Ypc*Yc*b1^2+2*Ypc^2*a1^2*Xpc^2+2*Ypc^2*a1^2*Xc^2-12*Xpc^2*Xc^2*Yc^2+8*Xpc^3*Xc*Yc^2+8*Xpc*Xc^3*Yc^2+2*Yc^2*Xpc^2*a1^2+2*Yc^2*Xpc^2*b1^2+24*Xpc^2*Xc^2*Ypc*Yc-16*Xpc^3*Xc*Ypc*Yc-16*Xpc*Xc^3*Ypc*Yc-4*Xpc*Xc*Ypc^2*b1^2-4*Ypc*a1^2*Yc*Xpc^2-8*Yc*Ypc^3*Xpc*Xc+12*Yc^2*Ypc^2*Xpc*Xc-8*Yc^3*Xpc*Xc*Ypc-4*Xpc*Xc*Ypc^2*a1^2-6*Yc^2*Ypc^2*Xpc^2+4*Yc*Ypc^3*Xpc^2+4*Yc*Ypc^3*Xc^2+2*Yc^4*Xpc*Xc-Xc^6-Xpc^6-Yc^4*Xpc^2-Yc^4*Xc^2-Ypc^4*Xpc^2-Ypc^4*Xc^2-2*Yc^2*Xpc^4-2*Xc^4*Yc^2-2*Xpc^4*Ypc^2-2*Xc^4*Ypc^2+6*Xpc*Xc^5+6*Xpc^5*Xc+20*Xpc^3*Xc^3-15*Xpc^2*Xc^4-15*Xpc^4*Xc^2-Xc^2*b1^4-Xc^2*a1^4+2*Xc^4*b1^2+2*Xc^4*a1^2-Xpc^2*b1^4-Xpc^2*a1^4+2*Xpc^4*b1^2+2*Xpc^4*a1^2+4*Yc^3*Xpc^2*Ypc+4*Yc^3*Xc^2*Ypc-12*Xpc^2*Xc^2*Ypc^2+2*Xpc*Xc*Ypc^4+8*Xpc^3*Xc*Ypc^2+8*Xpc*Xc^3*Ypc^2+2*Xpc*Xc*b1^4+2*Xpc*Xc*a1^4+12*Xpc^2*Xc^2*b1^2+12*Xpc^2*Xc^2*a1^2-8*Xpc*Xc^3*b1^2-8*Xpc*Xc^3*a1^2-8*Xpc^3*Xc*b1^2-8*Xpc^3*Xc*a1^2-4*Xpc*Xc*b1^2*a1^2+2*Xc^2*b1^2*a1^2+2*Xpc^2*b1^2*a1^2)^(1/2))*Ypc-Xc^2-Yc^2+b1^2+Xpc^2+Ypc^2-a1^2)/(Xc-Xpc)];
 
 
Ydc = [ 1/2/(-8*Yc*Ypc-8*Xpc*Xc+4*Yc^2+4*Ypc^2+4*Xc^2+4*Xpc^2)*(-4*Yc*Ypc^2+4*Yc^3-8*Xpc*Xc*Ypc+4*Ypc^3-4*Ypc*a1^2-8*Xpc*Xc*Yc+4*Yc*Xpc^2+4*Yc*a1^2+4*Xc^2*Yc-4*Yc*b1^2-4*Yc^2*Ypc+4*b1^2*Ypc+4*Xpc^2*Ypc+4*Xc^2*Ypc+4*(-6*Yc^2*Ypc^2*Xc^2+2*b1^2*Ypc^2*Xpc^2+2*b1^2*Ypc^2*Xc^2+4*Yc*Xpc^4*Ypc+2*Yc^2*a1^2*Xc^2+2*Xc^2*Yc^2*b1^2+4*Xc^4*Yc*Ypc-4*Ypc*a1^2*Xc^2*Yc-4*Xpc*Xc*Yc^2*a1^2-4*Xpc*Xc*Yc^2*b1^2-4*Yc*Xpc^2*b1^2*Ypc-4*Xc^2*Yc*b1^2*Ypc+8*Xpc*Xc*Ypc*Yc*a1^2+8*Xpc*Xc*Ypc*Yc*b1^2+2*Ypc^2*a1^2*Xpc^2+2*Ypc^2*a1^2*Xc^2-12*Xpc^2*Xc^2*Yc^2+8*Xpc^3*Xc*Yc^2+8*Xpc*Xc^3*Yc^2+2*Yc^2*Xpc^2*a1^2+2*Yc^2*Xpc^2*b1^2+24*Xpc^2*Xc^2*Ypc*Yc-16*Xpc^3*Xc*Ypc*Yc-16*Xpc*Xc^3*Ypc*Yc-4*Xpc*Xc*Ypc^2*b1^2-4*Ypc*a1^2*Yc*Xpc^2-8*Yc*Ypc^3*Xpc*Xc+12*Yc^2*Ypc^2*Xpc*Xc-8*Yc^3*Xpc*Xc*Ypc-4*Xpc*Xc*Ypc^2*a1^2-6*Yc^2*Ypc^2*Xpc^2+4*Yc*Ypc^3*Xpc^2+4*Yc*Ypc^3*Xc^2+2*Yc^4*Xpc*Xc-Xc^6-Xpc^6-Yc^4*Xpc^2-Yc^4*Xc^2-Ypc^4*Xpc^2-Ypc^4*Xc^2-2*Yc^2*Xpc^4-2*Xc^4*Yc^2-2*Xpc^4*Ypc^2-2*Xc^4*Ypc^2+6*Xpc*Xc^5+6*Xpc^5*Xc+20*Xpc^3*Xc^3-15*Xpc^2*Xc^4-15*Xpc^4*Xc^2-Xc^2*b1^4-Xc^2*a1^4+2*Xc^4*b1^2+2*Xc^4*a1^2-Xpc^2*b1^4-Xpc^2*a1^4+2*Xpc^4*b1^2+2*Xpc^4*a1^2+4*Yc^3*Xpc^2*Ypc+4*Yc^3*Xc^2*Ypc-12*Xpc^2*Xc^2*Ypc^2+2*Xpc*Xc*Ypc^4+8*Xpc^3*Xc*Ypc^2+8*Xpc*Xc^3*Ypc^2+2*Xpc*Xc*b1^4+2*Xpc*Xc*a1^4+12*Xpc^2*Xc^2*b1^2+12*Xpc^2*Xc^2*a1^2-8*Xpc*Xc^3*b1^2-8*Xpc*Xc^3*a1^2-8*Xpc^3*Xc*b1^2-8*Xpc^3*Xc*a1^2-4*Xpc*Xc*b1^2*a1^2+2*Xc^2*b1^2*a1^2+2*Xpc^2*b1^2*a1^2)^(1/2));
        1/2/(-8*Yc*Ypc-8*Xpc*Xc+4*Yc^2+4*Ypc^2+4*Xc^2+4*Xpc^2)*(-4*Yc*Ypc^2+4*Yc^3-8*Xpc*Xc*Ypc+4*Ypc^3-4*Ypc*a1^2-8*Xpc*Xc*Yc+4*Yc*Xpc^2+4*Yc*a1^2+4*Xc^2*Yc-4*Yc*b1^2-4*Yc^2*Ypc+4*b1^2*Ypc+4*Xpc^2*Ypc+4*Xc^2*Ypc-4*(-6*Yc^2*Ypc^2*Xc^2+2*b1^2*Ypc^2*Xpc^2+2*b1^2*Ypc^2*Xc^2+4*Yc*Xpc^4*Ypc+2*Yc^2*a1^2*Xc^2+2*Xc^2*Yc^2*b1^2+4*Xc^4*Yc*Ypc-4*Ypc*a1^2*Xc^2*Yc-4*Xpc*Xc*Yc^2*a1^2-4*Xpc*Xc*Yc^2*b1^2-4*Yc*Xpc^2*b1^2*Ypc-4*Xc^2*Yc*b1^2*Ypc+8*Xpc*Xc*Ypc*Yc*a1^2+8*Xpc*Xc*Ypc*Yc*b1^2+2*Ypc^2*a1^2*Xpc^2+2*Ypc^2*a1^2*Xc^2-12*Xpc^2*Xc^2*Yc^2+8*Xpc^3*Xc*Yc^2+8*Xpc*Xc^3*Yc^2+2*Yc^2*Xpc^2*a1^2+2*Yc^2*Xpc^2*b1^2+24*Xpc^2*Xc^2*Ypc*Yc-16*Xpc^3*Xc*Ypc*Yc-16*Xpc*Xc^3*Ypc*Yc-4*Xpc*Xc*Ypc^2*b1^2-4*Ypc*a1^2*Yc*Xpc^2-8*Yc*Ypc^3*Xpc*Xc+12*Yc^2*Ypc^2*Xpc*Xc-8*Yc^3*Xpc*Xc*Ypc-4*Xpc*Xc*Ypc^2*a1^2-6*Yc^2*Ypc^2*Xpc^2+4*Yc*Ypc^3*Xpc^2+4*Yc*Ypc^3*Xc^2+2*Yc^4*Xpc*Xc-Xc^6-Xpc^6-Yc^4*Xpc^2-Yc^4*Xc^2-Ypc^4*Xpc^2-Ypc^4*Xc^2-2*Yc^2*Xpc^4-2*Xc^4*Yc^2-2*Xpc^4*Ypc^2-2*Xc^4*Ypc^2+6*Xpc*Xc^5+6*Xpc^5*Xc+20*Xpc^3*Xc^3-15*Xpc^2*Xc^4-15*Xpc^4*Xc^2-Xc^2*b1^4-Xc^2*a1^4+2*Xc^4*b1^2+2*Xc^4*a1^2-Xpc^2*b1^4-Xpc^2*a1^4+2*Xpc^4*b1^2+2*Xpc^4*a1^2+4*Yc^3*Xpc^2*Ypc+4*Yc^3*Xc^2*Ypc-12*Xpc^2*Xc^2*Ypc^2+2*Xpc*Xc*Ypc^4+8*Xpc^3*Xc*Ypc^2+8*Xpc*Xc^3*Ypc^2+2*Xpc*Xc*b1^4+2*Xpc*Xc*a1^4+12*Xpc^2*Xc^2*b1^2+12*Xpc^2*Xc^2*a1^2-8*Xpc*Xc^3*b1^2-8*Xpc*Xc^3*a1^2-8*Xpc^3*Xc*b1^2-8*Xpc^3*Xc*a1^2-4*Xpc*Xc*b1^2*a1^2+2*Xc^2*b1^2*a1^2+2*Xpc^2*b1^2*a1^2)^(1/2))];

R1 = isreal(Xd);
R2 = isreal(Yd);
R3 = isreal(Xdb);
R4 = isreal(Ydb);
R5 = isreal(Xdc);
R6 = isreal(Ydc);




if R1 & R2 & R3 & R4 & R5 & R6





if (Xp > Xa)&(S1==1)                                           % solve the elbowup and elbowdown problem

elseif (Xp < Xa)&(S1==1)
    S1=2;

elseif (Xp > Xa)&(S1==2)

elseif (Xp < Xa)&(S1==2)
    S1=1;

end

if (Xpb > Xb)&(S2==1)                                           % solve the elbowup and elbowdown problem

elseif (Xpb<Xb)&(S2==1)
    S2 = 2;

elseif (Xpb>Xb)&(S2==2)

elseif (Xpb<Xb)&(S2==2)
    S2 = 1;

end

if (Xpc>Xc)&(S3==1)

elseif (Xpc<Xc)&(S3==1)
    S3=2;

elseif (Xpc>Xc)&(S3==2)
    
elseif (Xpc<Xc)&(S3==2)
    S3=1;

end
    L11 = plot([Xp Xd(S1)],[Yp Yd(S1)],'Linewidth',2,'Marker','O');
    text(Xp-1,Yp-1,'Base1','color','g');

    hold on
    L12 = plot([Xd(S1),Xa],[Yd(S1) Ya],'Linewidth',2,'Marker','O');

    hold on
    L21 = plot([Xpb Xdb(S2)],[Ypb Ydb(S2)],'Linewidth',2,'Marker','O');
    text(Xpb-1,Ypb-1,'Base2','color','r');

    hold on
    L22 = plot([Xdb(S2),Xb],[Ydb(S2) Yb],'Linewidth',2,'Marker','O');

    hold on
    L31 = plot([Xpc Xdc(S3)],[Ypc Ydc(S3)],'Linewidth',2,'Marker','O');
    text(Xpc-1,Ypc+1,'Base3','color',[0.9 0.7 0]);

    hold on 
    L32 = plot([Xdc(S3),Xc],[Ydc(S3) Yc],'Linewidth',2,'Marker','O');

    hold on

L41 = plot([Xa Xb],[Ya Yb],'color','r','Linewidth',3);
Horz= plot([Xa abs(Xb)+abs(H*sin(fai))],[Ya Ya],'color','b','LineStyle','--','color',[0 0.25 0.5]);
      text(Xa+0.7,Ya+0.5*sign(sin(fai)),'\phi','color',[0.5 0.25 0])

hold on
L42 = plot([Xb Xc],[Yb Yc],'color','y','Linewidth',3);

hold on
L43 = plot([Xc Xa],[Yc Ya],'color','g','Linewidth',3);
Center = plot(Xg,Yg,'o','color',[0.5 0 0.25]);


if (isempty(X)==0) & (isempty(Y)==0)
   for i=1:1:k-1
       hold on
       plot(X(i),Y(i),'m','marker','.')
       end
end 


axis([-15 15 -15 15])
grid on
hold off

if Yd(S1)>=Yp
theta11 = acos((Xd(S1)-Xp)/a1);
elseif Yd(S1)<Yp
    theta11 = 2*pi-acos((Xd(S1)-Xp)/a1);
end

if Ya>=Yd(S1)
theta12 = acos((Xa-Xd(S1))/b1);
elseif Ya<Yd(S1)
    theta12 = 2*pi-acos((Xa-Xd(S1))/b1);
end

if Ydb(S2)>=Ypb
theta21 = acos((Xdb(S2)-Xpb)/a1);
elseif Ydb(S2)<Ypb
    theta21 = 2*pi-acos((Xdb(S2)-Xpb)/a1);
end

if Yb>=Ydb(S2)
theta22 = acos((Xb-Xdb(S2))/b1);
elseif Yb<Ydb(S2)
    theta22 = 2*pi-acos((Xb-Xdb(S2))/b1);
end

if Ydc(S3)>=Ypc
theta31 = acos((Xdc(S3)-Xpc)/a1);
elseif Ydc(S3)<Ypc
    theta31 = 2*pi-acos((Xdc(S3)-Xpc)/a1);
end

if Yc>=Ydc(S3)
theta32 = acos((Xc-Xdc(S3))/b1);
elseif Yc<Ydc(S3)
    theta32 = 2*pi-acos((Xc-Xdc(S3))/b1);
end



if (Xd(S1)>Xa) & (Yd(S1)<Ya)
    diaota1 = theta11-theta12;
elseif (Xd(S1)<Xa) & (Yd(S1)<Ya)
    diaota1 = theta11+theta12;
elseif (Xd(S1)<Xa) & (Yd(S1)>Ya)
    diaota1 = theta12-theta11;
elseif (Xd(S1)>Xa) & (Yd(S1)>Ya)
    diaota1 = pi-(theta11-theta12);
end



if (Xdb>Xb) & (Ydb<Yb)
    diaota2 = theta21-theta22;
elseif (Xdb<Xb) & (Ydb<Yb)
    diaota2 = theta21+theta22;
elseif (Xdb<Xb) & (Ydb>Yb)
    diaota2 = theta22-theta21;
elseif (Xdb>Xb) & (Ydb>Yb)
    diaota2 = pi-(theta21-theta22);
end

if (Xdc>Xc) & (Ydc<Yc)
    diaota3 = theta31-theta32;
elseif (Xdc<Xc) & (Ydc<Yc)
    diaota3 = theta31+theta32;
elseif (Xdc<Xc) & (Ydc>Yc)
    diaota3 = theta32-theta31;
elseif (Xdc>Xc) & (Ydc>Yc)
    diaota3 = pi-(theta31-theta32);
end

angledisplay(AD)
else 
    
    warning('You facd a singularity. Please change the variables to solve it.')
    Warnd = text(-10,10,'You facd a singularity. Please change the variables to solve it.','color','b');
    set(Warnd,'visible','on')
    clc
end