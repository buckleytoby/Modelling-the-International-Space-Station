function [ surfaceAreasList, surfaceCM_bodyList, normalsList ] = S_top2Lists( S_top )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% extract surface areas, surfaceCM_body, and normals from S_top and put
% into big long list
surfaceAreasList = [];
surfaceCM_bodyList = [];
normalsList = [];
for i=1:length(S_top.children)
  chd = S_top.children{i};
  surfaceAreasList = [surfaceAreasList; chd.surfaceAreas];
  surfaceCM_bodyList = [surfaceCM_bodyList, chd.surfaceCM_body];
  normalsList = [normalsList, chd.normals];
end

