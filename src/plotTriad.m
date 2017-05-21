function [ fig ] = plotTriad( fig, triad )
%PLOTTRIAD Summary of this function goes here
%   triad = [ x_hat, y_hat, z_hat], i_hat is 3x1 unit direction vector
figure(fig);hold on

colors = {'red','green','blue'};
for i=1:3
  quiver3(0,0,0, triad(1,i), triad(2,i), triad(3,i), 0, colors{i},'linewidth',3);
end
end

