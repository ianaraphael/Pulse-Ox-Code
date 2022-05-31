% PulseOxLab 

% Code for finding heart rate and O2 saturation 

% read in sensor data from red and IR channels 

% convert frequency to capacitance
Fsample = 1000000;  % from teensie code, could change to undersample
t = 0:1/Fsample:(length(sensorRed)-1)/Fsample;


jw = tf('s'); tau=1/(2*pi*1); bpf = jw*tau/(1+jw*tau)/(1+3*jw*tau);
bpfRed = lsim(bpf, sensorRed, t);
bpfIR = lsim(bpf, sensorIR, t);

% or lowpass filter the data
lpfRed = lowpass(sensorRed,5,Fsample);
smRed = movmean(lpfRed,150);    % Moving mean smoothing filter
smIR = movmean(bpfIR,150);

% find local minimum 
minsRed = islocalmin(smRed); 
minsIR = islocalmin(smIR);

[maxpkRed,maxidxRed] = max(smRed);    % find where the max peak occurs
[maxpkIR,maxidxIR] = max(smIR);

% check that mins are lining up 
figure;
plot(t,smRed,t(mins),smRed(minsRed),'r*');


% we need to be taking the min and max from around the same areas, so
% possibly cut the data down to look at just 2-3 cycles 

maxRed = max(maxpkRed);
maxIR = max(maxpkIR);

minRed = min(minsRed);
minIR = min(minsIR); 

% attenuation coef = ln(Imax)/ln(Imin);
uRed = ln(maxRed)/ln(minRed);
uIR = ln(maxIR)/ln(minIR); 

% both mu a coeffients in one matrix, IR first 
muA = [uIR; uRed]; 

% using equation from notes on pulse oximetry Eps Hbo, Hb 
% wavelength 900 first line, mu 660 second line 
molext = [1198 761.84; 319.6 3226.56];
conc = inv(molext)*muA;     % this will be conc Hbo ; conc Hb 

% Oxygen saturation 
p02 = conc(1)/(conc(1)+conc(2));

%% get heart rate
fs = 1e6; % set sampling fx
T = 1/fs; % set sample time
L = length(smIR); % get the length of the input record - use smoothed and lpf data
t = (0:L-1)*T; % get a time vector that matches

NFFT = 2^nextpow2(L); % Next power of 2 from length of y
Y = fft(smIR-mean(smIR),NFFT)/L; % get the fft with the mean (dc voltage) subtracted off)
f = fs/2*linspace(0,1,NFFT/2+1);

powY = 2*abs(Y(1:NFFT/2+1)); % calculate power spectrum

% find index of maximum power
[~,peakIndex] = max(powY);

% get corresponding frequency
peakFrequency = f(peakIndex);

% calculate and display heart rate
disp("Heart rate: " + round(peakFrequency*60) +" bpm");