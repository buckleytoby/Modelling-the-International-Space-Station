function [ x,y,z,area,surfaceCM ] = getCuboid( L, W, H)
%GETCUBE Summary of this function goes here
% counter-clockwise direction corresponds to outward-face

x = [0 0 1 1; %bottom
     0 1 1 0; %top
     0 1 1 0; %back left
     0 0 1 1; %forward right
     1 1 1 1; %forward left
     0 0 0 0; %backward right
]';

y = [0 1 1 0;
     0 0 1 1;
     0 0 0 0;
     1 1 1 1;
     0 1 1 0;
     0 0 1 1;
]';

z = [0 0 0 0;
     1 1 1 1;
     0 0 1 1;
     0 1 1 0;
     0 0 1 1;
     0 1 1 0;
]';

area = [L*W, L*W, L*H, L*H, W*H, W*H]';
surfaceCM = [0.5*L 0.5*W 0; 0.5*L 0.5*W H; 0.5*L 0 0.5*H; 0.5*L W 0.5*H ...
  ; 0 0.5*W 0.5*H; L 0.5*W 0.5*H]';

x = L.*x;
y = W.*y;
z = H.*z;

end

