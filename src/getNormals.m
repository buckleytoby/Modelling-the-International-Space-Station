function [ n ] = getNormals( x,y,z)
%GETNORMALS Summary of this function goes here
% assumes n=size(x,2) is the number of faces
% output is 3xlen
len = size(x,2);
n = zeros(3,len);
for i=1:len
  n1 = getNormal(x(:,i), y(:,i), z(:,i));
  n(:,i) = n1(:);
end
end

