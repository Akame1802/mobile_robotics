function [yvec] = trajToTimeFunc_y(tvec, points, infos, steps, isDerivative)
%Questa funzione lavora come trajToTimeFunc_x
yvec = zeros(size(tvec));
for k=1:length(tvec)
    t = tvec(k);
    if(isDerivative==0)
        for i=0:length(steps)-1
            if(t<=steps(i+1))
%             if(t>=i*step && t<=(i+1)*step)%intervallo di riferimento
                p1 = points(i+1,:);
                p2 = points(i+2,:);
                if(isempty(infos) || infos(i+1,4)=='l')
                    if(i==0)
                        yvec(k) = interp1([0, steps(i+1)],[p1(2),p2(2)],t);
                    else
                        yvec(k) = interp1([steps(i),steps(i+1)],[p1(2),p2(2)],t);
                    end
%                     yvec(k) = interp1([i*step,(i+1)*step],[p1(2),p2(2)],t);
                else
                    cx = infos(i+1,1);
                    cy = infos(i+1,2);
                    r = infos(i+1,3);
                    arg_cos0 = (p1(1)-cx)/r;
                    arg_sin0 = (p1(2)-cy)/r;
                    arg_cos1 = (p2(1)-cx)/r;
                    arg_sin1 = (p2(2)-cy)/r;
                    [alpha0,alpha1] = angles_between(arg_cos0, arg_sin0, arg_cos1, arg_sin1);
                    if(alpha0>alpha1)
                        dir = -1;
                    else
                        dir = 1;
                    end
                    angle = alpha0:0.01*dir:alpha1+0.01*dir;
                    if(angle(end)~=alpha1)
                        angle(end) = alpha1;
                    end
                    if(i==0)
                        eps = steps(i+1)/length(angle);
                        range = 0:(steps(i+1)+eps)/length(angle):steps(i+1);
                    else
                        eps = (steps(i+1)-steps(i))/length(angle);
                        range = steps(i):(steps(i+1)-steps(i)+eps)/length(angle):steps(i+1);
                    end
%                     eps = step/length(angle);
%                     range = i*step:(step+eps)/length(angle):(i+1)*step;
                    if(range(end)~=steps(i+1))
                        range(end) = steps(i+1);
                    end
                    yvec(k) = interp1(range, cy+r*sin(angle),t);
                end
                break
            end
        end
    elseif(isDerivative==1) %derivata prima
        for i=0:length(steps)-1
            if(t<=steps(i+1))%intervallo di riferimento
                p1 = points(i+1,:);
                p2 = points(i+2,:);
                if(isempty(infos) || infos(i+1,4)=='l')
                    if(i==0)
                        yvec(k) = (p2(2)-p1(2))/steps(i+1);
                    else
                        yvec(k) = (p2(2)-p1(2))/(steps(i+1)-steps(i));
                    end
%                     yvec(k) = (p2(2)-p1(2))/step;
                else
                    cx = infos(i+1,1);
                    cy = infos(i+1,2);
                    r = infos(i+1,3);
                    arg_cos0 = (p1(1)-cx)/r;
                    arg_sin0 = (p1(2)-cy)/r;
                    arg_cos1 = (p2(1)-cx)/r;
                    arg_sin1 = (p2(2)-cy)/r;
                    [alpha0,alpha1] = angles_between(arg_cos0, arg_sin0, arg_cos1, arg_sin1);
                    if(alpha0>alpha1)
                        dir = -1;
                    else
                        dir = 1;
                    end
                    angle = alpha0:0.01*dir:alpha1+0.01*dir;
                    if(angle(end)~=alpha1)
                        angle(end) = alpha1;
                    end
                    if(i==0)
                        eps = steps(i+1)/length(angle);
                        range = 0:(steps(i+1)+eps)/length(angle):steps(i+1);
                    else
                        eps = (steps(i+1)-steps(i))/length(angle);
                        range = steps(i):(steps(i+1)-steps(i)+eps)/length(angle):steps(i+1);
                    end
%                     eps = step/length(angle);
%                     range = i*step:(step+eps)/length(angle):(i+1)*step;
                    if(range(end)~=steps(i+1))
                        range(end) = steps(i+1);
                    end
                    yvec(k) = interp1(range, r*cos(angle),t);
                end
                break
            end
        end
    else %derivata seconda    
        for i=0:length(steps)-1
            if(t<=steps(i+1))%intervallo di riferimento
                p1 = points(i+1,:);
                p2 = points(i+2,:);
                if(isempty(infos) || infos(i+1,4)=='l')
                    yvec(k) = 0;
                else
                    cx = infos(i+1,1);
                    cy = infos(i+1,2);
                    r = infos(i+1,3);
                    arg_cos0 = (p1(1)-cx)/r;
                    arg_sin0 = (p1(2)-cy)/r;
                    arg_cos1 = (p2(1)-cx)/r;
                    arg_sin1 = (p2(2)-cy)/r;
                    [alpha0,alpha1] = angles_between(arg_cos0, arg_sin0, arg_cos1, arg_sin1);
                    if(alpha0>alpha1)
                        dir = -1;
                    else
                        dir = 1;
                    end
                    angle = alpha0:0.01*dir:alpha1+0.01*dir;
                    if(angle(end)~=alpha1)
                        angle(end) = alpha1;
                    end
                    if(i==0)
                        eps = steps(i+1)/length(angle);
                        range = 0:(steps(i+1)+eps)/length(angle):steps(i+1);
                    else
                        eps = (steps(i+1)-steps(i))/length(angle);
                        range = steps(i):(steps(i+1)-steps(i)+eps)/length(angle):steps(i+1);
                    end
%                     eps = step/length(angle);
%                     range = i*step:(step+eps)/length(angle):(i+1)*step;
                    if(range(end)~=steps(i+1))
                        range(end) = steps(i+1);
                    end
                    yvec(k) = interp1(range, -r*sin(angle),t);
                end
                break
            end
        end
    end
end

end