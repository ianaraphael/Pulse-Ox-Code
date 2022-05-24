% Author: Kofi Odame
% Description: Matlab USB Serial Client
% Date: 04/8/2018

%function [s1, t1] = usbstream(s, Fs, bufsize)
global xLabel yLabel ymin ymax
numLoops=0;
maxLoops=1000;
xLabel = 'elapsed time (s)';    % x-axis label
yLabel = 'voltage (V)';         % y-axis label
% pGrid  = 'on';                  % 'off' to turn off grid
Vref = 3.3;                     % ADC full scale (in Volts)
windowSize = 3;                 % window size (in seconds) to display in plot
ymin = 0.0;                      % set y-min
ymax = Vref*1.1;                 % set y-max


%Define Function Variables
t1 = 0:1/Fs:windowSize;
sRed = zeros(size(t1));
sIR = zeros(size(t1));

%Set up Plot
fig = figure();
hold on
plotGraph = plot(t1,sIR);
while (ishandle(plotGraph) && numLoops < maxLoops) %Loop when Plot is Active

    [buf,~] = fread(s);
    lightType = buf(1);
%     lightType = 0;

    buf = buf(3:length(buf));
%     if (size(buf, 1) ~= bufsize), break, end %Make sure buf is the correct size
    
    bufDec=bin2dec([dec2bin(buf(1:2:end), 8), dec2bin(buf(2:2:end), 8)]);
    bufDec=bufDec*3.3/(2^12-1); % scale to Vref=3.3 V, 12 bits
    
    if(~isempty(bufDec)) %Make sure Data Type is Correct
        
        if lightType == 0
            plotColor = 'r'; % set plot color red for red LED
            
            sRed = [sRed(1+length(bufDec):end), bufDec'];
            
            plotGraph = plotData(t1,sRed,plotColor);
            
        else
            plotColor = 'k'; % set plot color black for IR led
            
            sIR = [sIR(1+length(bufDec):end), bufDec'];
            
            plotGraph = plotData(t1,sIR,plotColor);
          
        end
        
        drawnow
        t1 = [t1(1+length(bufDec):end), t1(end)+[1:length(bufDec)]/Fs];
        %Request more data from MCU
        %         fprintf(s, '%c', 'a');
    end
    numLoops = numLoops+1;
end

%Close Serial COM Port and Delete useless Variables
fclose(s);
clear s

disp('Session Terminated');

function plotGraph = plotData(x,y,plotColor)
global xLabel yLabel ymin ymax

% plot graph
plotGraph = plot(x,y,'-b',...
    'LineWidth',1,...
    'Color',plotColor,...
    'MarkerFaceColor','w',...
    'MarkerSize',2);
title('ADC output','FontSize',16);
xlabel(xLabel,'FontSize',13);
ylabel(yLabel,'FontSize',13);
grid on;
axis([x(1) x(end) ymin ymax])
end