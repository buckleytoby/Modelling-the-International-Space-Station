function [ fig1 ] = plotAxesTriads( fig1, S_top, mult )
%PLOTAXESTRIADS Summary of this function goes here
%   Detailed explanation goes here

figure(fig1);
P = S_top.DCM_P2B;
colors = {'red','green','blue'};
for i=1:3
  Pi = mult.*P(:,i)';
  plot3([0; Pi(1)], [0; Pi(2)], [0; Pi(3)], colors{i},'linewidth',2)
end

