function HandShake(s)

% disp('Ready...')
% fprintf(s, '%c', 'a');

a = 0;

disp('Waiting for ready signal...')
while ~get(s,'BytesAvailable'), end
a=fscanf(s,'%d');

while (a ~= 16383)
end

disp('Ready signal received.')
fprintf(s, '%c', 'a');
disp('Requesting data.')    
end
