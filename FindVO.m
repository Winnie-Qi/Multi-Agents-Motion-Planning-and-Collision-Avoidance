function c,u,v = FindVO(p,r,v) % FindVO( Pos_A, Pos_B, r_A, r_B, v_Abest, v_B)

Gamma=atan2(v(2),v(1));
if Gamma < 0
    Gamma=2*pi+Gamma;% the angle of v_A -v_B
end

Alpha=atan2(p(2),p(1));
if Alpha < 0
    Alpha= 2*pi+Alpha; % the angle between agent connection line and X axis
end

if  p == r
    if Alpha >= 0 && Alpha < pi/2
        if (Gamma >=0 && Gamma < Alpha+pi/2) || (Gamma > Alpha+3*pi/2 && Gamma <2*pi)
          c=1;
        else
            c=0;
        end
    
    elseif Alpha >=pi/2 && Alpha < (3*pi/2)
        if Gamma >Alpha-pi/2 && Gamma <Alpha +pi/2
          c =1;
        else
          c=0;
        end
    else
        if (Gamma>Alpha- pi/2 && Gamma <=2*pi) || (Gamma >=0 && Gamma <Alpha-3*pi/2)
          c=1;
        else
          c=0;
        end
    end

else
    Beta=asin(r/sqrt(p(1)^2 + p(2)^2));
    
    if (Alpha-Beta) <0
       if (Gamma >= 0 && Gamma < (Alpha+Beta)) || (Gamma > (Alpha-Beta +2*pi) && Gamma <= 2*pi)
           c =1;
       else
           c=0;
       end
    elseif (Alpha+Beta)> (2*pi)
        if (Gamma >= 0 && Gamma < (Alpha+Beta-2*pi)) || (Gamma > (Alpha-Beta) && Gamma <= 2*pi)
            c = sqrt(sum((p - v).^2))-r < 0 ;
        else
            c=0;
        end
    else
        if Gamma > Alpha-Beta && Gamma < Alpha+Beta
           c = sqrt(sum((p - v).^2))-r < 0;
        else
           c=0;
        end 
    end
end
end

