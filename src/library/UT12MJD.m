function [ MJD ] = UT12MJD( calendar )
%convert calendar form [MM,DD,YYYY] to modified julian date
%montenbruck A.1.1, page 321
m = calendar(1);
d = calendar(2);
y = calendar(3);
if m<=2
    y=y-1;
    m=m+12;
end
b = y/400-y/100+y/4;
MJD = 365*y - 679004 + floor(b) + floor(30.6001*(m+1)) + d;

end

