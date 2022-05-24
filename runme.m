initializeSerial % initialize serial port
%[sensordata, time] = usbstream(s, Fs, bufsize); % stream data and save last few seconds

usbstream; 
% sensordata=sRed;
% time=t;
% 
% % plot last few seconds
% plot(time,sensordata,'-b',...
%     'LineWidth',1,...
%     'MarkerFaceColor','w',...
%     'MarkerSize',2);
% title('ADC output','FontSize',16);
% xlabel('elapsed time (s)','FontSize',13);
% ylabel('voltage (V)','FontSize',13);
% grid on