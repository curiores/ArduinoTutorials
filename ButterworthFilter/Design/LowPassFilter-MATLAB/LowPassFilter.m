%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Reference: https://github.com/curiores/ArduinoTutorials/blob/main/ButterworthFilter/Design/LowPassFilter.ipynb      %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%      Generate a test signal       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Generate a signal
samplingFreq = 1000; % sampled at 1 kHz = 1000 samples / second
tlims = [0,1]        % in seconds
signalFreq = [2,50]; % Cycles / second
signalMag = [1,0.2]; % magnitude of each sine
t = linspace(tlims(1),tlims(2),(tlims(2)-tlims(1))*samplingFreq);
y = signalMag(1)*sin(2*pi*signalFreq(1)*t) + signalMag(2)*sin(2*pi*signalFreq(2)*t);

% Plot the signal
figure(1)
plot(t,y);
ylabel('y(t)');
xlabel('t(s)');
xlim([min(t),max(t)]);

% Compute the Fourier transform
yhat = fft(y);
fcycles = fftfreq(length(t),1.0/samplingFreq); % the frequencies in cycles/s

% Plot the power spectrum

figure(2)
plot(fcycles,abs(yhat));
xlim([-100,100]);
xlabel('\omega (cycles/s)');
ylabel('$\hat{y}$','Interpreter','Latex');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Low-pass filter transfer function       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Low-pass filter
w0 = 2*pi*5; % pole frequency (rad/s)
num = w0;        % transfer function numerator coefficients
den = [1,w0];    % transfer function denominator coefficients
lowPass = tf(num,den); % Transfer function

% Generate the bode plot
w = logspace(log10(min(signalFreq)*2*pi/10), log10(max(signalFreq)*2*pi*10), 500 );
[mag, phase,w] = bode(lowPass,w);

% Returns an array and convert to dB for mag
mag = mag2db(squeeze(mag));
% Returns an array
phase = squeeze(phase);
% Magnitude plot
figure(3)
semilogx(w, mag); hold on
for i= 1 : length(signalFreq)
    sf = signalFreq(i);
    semilogx([sf*2*pi,sf*2*pi],[min(mag),max(mag)],'k:'); hold on
end
ylabel('Magnitude (dB)');
xlim([min(w),max(w)]);
ylim([min(mag),max(mag)]);

% Phase plot
figure(4)
semilogx(w, phase)  % Bode phase plot
ylabel('Phase (^\circ)')
xlabel('\omega (rad/s)')
xlim([min(w),max(w)])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Discrete transfer function       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt = 1.0/samplingFreq;
opt = c2dOptions('Method','tustin','FractDelayApproxOrder',3);
discreteLowPass = c2d(lowPass,dt,opt)

% The output like below:
%
% discreteLowPass =
% 
%  0.01547 z + 0.01547
%  -------------------
%      z - 0.9691
% 
% Sample time: 0.001 seconds

% The coefficients from the discrete form of the filter transfer function (but with a negative sign)
b = cell2mat(discreteLowPass.num);
a = -cell2mat(discreteLowPass.den);
display(['Filter coefficients b_i: ', num2str(b(1)), ', ', num2str(b(2))])
display(['Filter coefficients a_i: ', num2str(a(2))])

% Filter the signal
yfilt = zeros(length(y));
for i = 4 : length(y)
    yfilt(i) = a(2)*yfilt(i-1) + b(1)*y(i) + b(2)*y(i-1);
end   
% Plot the signal
figure(5)
plot(t,y); hold on
plot(t,yfilt); hold on
ylabel('y(t)')
xlim([min(t),max(t)]);

% Generate Fourier transform
yfilthat = fft(yfilt);
fcycles = fftfreq(length(t),1.0/samplingFreq);

figure(6)
plot(fcycles,abs(yhat)); hold on
plot(fcycles,abs(yfilthat));hold on
xlim([-100,100]);
xlabel('\omega (cycles/s)');
ylabel('$$\hat{y}$$','Interpreter','Latex');