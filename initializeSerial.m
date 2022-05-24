% Check for previous serial connection
if ~isempty(whos('s')), fclose(s); delete(s); clear s; end

clearvars
bufsize=700;
Fs = 1e6;

% Create Serial port 's'. Specify port name
s = serial('/dev/cu.usbmodem27613001');
set(s, 'Baudrate',9600, 'StopBits', 1);
set(s, 'Terminator', 'LF', 'Parity', 'none');
set(s, 'FlowControl','none');

% Set size of receiving buffer
set(s, 'InputBufferSize', bufsize*2);
% Open connection to the sensor board.
fopen(s);

% Initiate handshake
% HandShake(s);

% Wait for data to become available
while ~get(s,'BytesAvailable'), end, return