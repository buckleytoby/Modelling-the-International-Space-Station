function [ RTN_triad ] = getRTNTriad( OMEGA, i, omega, nu )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
R_pqw2i = rotz(OMEGA)*rotx(i)*rotz(omega);
R_rtn2pqw = rotz(nu);
R_i2rtn = R_rtn2pqw' * R_pqw2i' ;
RTN_triad = R_i2rtn' ;

end

