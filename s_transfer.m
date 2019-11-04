function S=s_transfer(alpha, beta,an, bn , dn,th )  

S=[                                 cos(beta)*cos(th),                                -cos(beta)*sin(th),             sin(beta),                                       an*cos(beta) + dn*sin(beta);
    cos(alpha)*sin(th) + sin(alpha)*sin(beta)*cos(th), cos(alpha)*cos(th) - sin(alpha)*sin(beta)*sin(th), -cos(beta)*sin(alpha), bn*cos(alpha) - dn*cos(beta)*sin(alpha) + an*sin(alpha)*sin(beta);
    sin(alpha)*sin(th) - cos(alpha)*sin(beta)*cos(th), sin(alpha)*cos(th) + cos(alpha)*sin(beta)*sin(th),  cos(alpha)*cos(beta), bn*sin(alpha) + dn*cos(alpha)*cos(beta) - an*cos(alpha)*sin(beta);
                                                 0,                                                 0,                     0,                                                                 1];
%   for i=1:16
%          if isa(S(i), 'double')
%          elseif S(i)<10^-5
%                S(i)=0;
%          end
%   end