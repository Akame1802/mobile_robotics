function [alpha0,alpha1] = angles_between(arg_cos0, arg_sin0, arg_cos1, arg_sin1)
%gestione ambiguità sui valori degli angoli
%alpha0: angolo di partenza, alpha1: angolo di arrivo
if(sign(arg_cos0)>=0 && sign(arg_sin0)<=0)
    alpha0 = 2*pi-acos(arg_cos0);
elseif(sign(arg_cos0)<=0 && sign(arg_sin0)<=0)
    alpha0 = pi+(pi-acos(arg_cos0));
else
    alpha0 = acos(arg_cos0);
end
if(sign(arg_cos1)>=0 && sign(arg_sin1)<=0)
    alpha1 = 2*pi-acos(arg_cos1);
elseif(sign(arg_cos1)<=0 && sign(arg_sin1)<=0)
    alpha1 = pi+(pi-acos(arg_cos1));
else
    alpha1 = acos(arg_cos1);
end
end