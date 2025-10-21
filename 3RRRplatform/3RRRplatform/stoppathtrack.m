% This is a subfunction of rrrproject3.m
% Created by Gan Tao   taogan@eng.buffalo.edu
% Graduate Student of SUNY Buffalo
% Date: Jan.2004
% stoppathtrack.m is the stop the path drawing and then ask the robot to
% trace the path.

function stoppathtrack(obj,eventdata)

global CC k
global X Y
global fig
global Xg Yg
global S1 S2 S3
global InVS
global Rec_Movie
global CInverCK

     cp=get(gca,'CurrentPoint');
     x=cp(1,1);
     y=cp(1,2);
if (x > -15) &  (x < 15) & (y > -15) & (y < 15)
    
    if InVS ==1 & CInverCK == 0
       CC = 0;
       set(fig,'WindowButtonMotionFcn',@nothingfcn);
       size(X);
       size(Y);
              if Rec_Movie == 1
              set(fig,'DoubleBuffer','on');
              set(gca,'xlim',[-15 15],'ylim',[-15 15],...
                      'NextPlot','replace','Visible','off')
              mov = avifile('3RRRrobot_track_path.avi')
       end
          for i=1:1:(k-1)
              Xg = X(i);
              Yg = Y(i);
              RRRdrawing(S1,S2,S3)
              pause(0.0000000001)
              if Rec_Movie == 1
                 F = getframe(gca);
                 mov = addframe(mov,F);
                 end
        end
              if Rec_Movie == 1
                 mov = close(mov);
              end

       k = 1;
    elseif InVS == 1 & CInverCK == 1
    CC = 0;


    set(fig,'WindowButtonMotionFcn',@nothingfcn);
    end

end