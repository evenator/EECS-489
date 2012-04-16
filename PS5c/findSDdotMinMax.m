function [sDdotMin,sDdotMax,valid] = findSDdotMinMax(qDdotMax,Bvec,Mvec,sdot)
% must have -qDdotMax(i) <= Mvec(i)sddot + Bvec(i)sdot^2 <= qDdotMax(i)
% must apply to all 3 jnts 
% find min and max sDdot subject to these constraints
valid=1; %flag to indicate if there is ANY legal value of sddot
Cvec = Bvec./Mvec; %only works if no Mvec(i) term  = 0
alpha0Vec = zeros(3,1);
beta0Vec = zeros(3,1);
for i=1:3
    %same as alpha0Vec = -qDdotMax/|Mvec|
    %these limits are always alpha<beta
    alpha0Vec(i)= -qDdotMax(i)/Mvec(i); %alt: could have separate qDdotMin, instead of symmetryic +/- lims
    beta0Vec(i)=   qDdotMax(i)/Mvec(i);
    if Mvec(i)<0 %swap upper and lower bounds if divide by neg number in inequality
        alpha0Vec(i)= qDdotMax(i)/Mvec(i); %reverse inequalities if Mvec(i)<0
        beta0Vec(i)= -qDdotMax(i)/Mvec(i); %-qDdotMax is assumed qDdotMin
    end
end

    %if Mvec(i)>0, then alpha = alpha0-Cvec*sdot^2
    % and beta = beta0 -Cvec*sdot^2
    %Cvec has same influence, whether Mvec>0 or Mvec<0
    alphas = alpha0Vec-Cvec*sdot^2;
    betas = beta0Vec-Cvec*sdot^2;
sDdotMin = max(alphas); %binding constraint on min sddot
sDdotMax = min(betas); %binding constraint on max sddot
if sDdotMin>sDdotMax
    valid=0;
end 

end

