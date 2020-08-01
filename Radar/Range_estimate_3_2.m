
Rmax=300; %meter
d_res=1; %range resolution is 1 meter
c=3e8;
% TODO : Find the Bsweep of chirp for 1 m resolution
B=c/(2*d_res);

% TODO : Calculate the chirp time based on the Radar's Max Range

Ts=5.5*2*Rmax/c;
% TODO : define the frequency shifts 

fb=[0.0,1.1e6,13e6,24e6];
% Display the calculated range
%or easier to do 
R=(Ts*c*fb)/(2*B);
disp(R);
% for i=fb
%     R=(Ts*c*i)/(2*B);
%     disp(R);
% end
