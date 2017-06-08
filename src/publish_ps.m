
s=struct;
s.format = 'html';
s.evalCode = false;
for i=1:9
  publish(['PS' num2str(i)],s)
end