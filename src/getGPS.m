function V = getGPS(bNoise, A_p2i, antennas_CM_P, state)
antennas_ECI = A_p2i * antennas_CM_P; % + repmat(state(1:3), [1, size(antennas_CM_P,2)]); %state is in ECI
V = antennas_ECI; %don't want to model the GPS system, so I just assume I get it from here
if bNoise
  % http://www.gps.gov/systems/gps/performance/accuracy/ 0.715 meter
  % ... seems too big .. let's guess 10cm
  stdDev = 50/1000^2; %km
  V = V + normrnd(0, stdDev*eye(size(V)));
end
end