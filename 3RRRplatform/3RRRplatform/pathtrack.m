% This is a subfunction of rrrproject3.m
% Created by Gan Tao   taogan@eng.buffalo.edu
% Date: Jan.2004

function pathtrack(obj,eventdata)
global CC k
global X Y
global Xg Yg
global S1 S2 S3
global InVS

if CC==1 & InVS ==1

    cp=get(gca,'CurrentPoint');
    if (cp(1,1) > -15)&(cp(1,2) > -15)&(cp(1,1) < 15)&(cp(1,2) < 15)
     X(k)=cp(1,1);
     Y(k)=cp(1,2);
     plot(X(k),Y(k),'m','Marker','.')
     axis([-15 15 -15 15])
     hold on
     k = k+1;
    end

end



    
    
    
    

    
   
    
    


    