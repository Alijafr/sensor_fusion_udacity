% Operating frequency (Hz)
fc = 77.0e9;

%Transmitted power (W)
Pt = 3e-3;

%Antenna Gain (linear)
G =  10000;

%Minimum Detectable Power
Ps = 1e-10;

%RCS of a car
RCS = 100;

%Speed of light
c = 3*10^8;

%TODO: Calculate the wavelength
A=Pt*G*RCS;
B=Ps*power(4*pi,3)*fc*fc;
R=power(A/B,0.25);

%TODO : Measure the Maximum Range a Radar can see.
disp(R);