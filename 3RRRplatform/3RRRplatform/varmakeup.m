% This is a subfunction of rrrproject3.m
% Created by Gan Tao   taogan@eng.buffalo.edu
% Graduate Student of SUNY Buffalo
% Date: Jan.2004
% varmakeup.m is for give the robot a fault data. if there is a data in one
% variable then it won't give fault data.

function k=varmakeup()

global H a1 b1 
global Yp Xp Ypb Xpb Ypc Xpc
global Xg Yg fai
global S1 S2 S3 
global AD
global CInverCK
global Rec_Movie
if isempty(Xg)
    Xg = 0;
    k = 1;
end

    if isempty(Yg)
        Yg = 0;
        k = k+1;
    end
        if isempty(fai)
            fai = pi/4;
            k = k+1;
        end
            if isempty(Yp)
                Yp = -8.4;
                k = k+1;
            end
                if isempty(Xp)
                    Xp = -1.5;
                    k = k+1;
                end
                    if isempty(Ypb)
                        Ypb = -1.2;
                        k = k+1;
                    end
                        if isempty(Xpb)
                            Xpb = 6.9;
                            k = k+1;
                        end
                            if isempty(Xpc)
                                Xpc = -6.6;
                                k = k+1;
                            end
                                if isempty(Ypc)
                                    Ypc = 2.1;
                                    k = k+1;
                                end
                                    if isempty(H)
                                        H = 3;
                                        k = k+1;
                                    end
                                        if isempty(a1)
                                            a1 = 5;
                                            k = k+1;
                                        end
                                            if isempty(b1)
                                                b1 = 4;
                                                k = k+1;
                                            end
                                               if isempty(S1)
                                                   S1= 1;
                                                   k = k+1;
                                               end
                                                  if isempty(S2)
                                                      S2=1;
                                                      k=k+1;
                                                  end
                                                     if isempty(S3)
                                                         S3=1;
                                                         k=k+1;
                                                     end
                                                       if isempty(AD)
                                                           AD = 1;
                                                           k=k+1;
                                                       end
                                                     
                                                           if 1
                                                           CInverCK=0;   
                                                       end 
                                                             if isempty(Rec_Movie)
                                                                 Rec_Movie = 0;
                                                             end
                                                                
                                                             