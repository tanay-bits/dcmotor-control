function client(port)
%   provides a menu for accessing PIC32 motor control functions
%
%   client(port)
%
%   Input Arguments:
%       port - the name of the com port.  This should be the same as what
%               you use in screen or putty in quotes ' '
%
%   Example:
%       client('/dev/ttyUSB0') (Linux/Mac)
%       client('COM3') (PC)
%
%   For convenience, you may want to change this so that the port is hardcoded.
   
% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 120 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 230400, 'FlowControl', 'hardware','Timeout',120); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 

has_quit = false;
% menu loop
while ~has_quit
    fprintf('PIC32 MOTOR DRIVER INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf('a: Read current sensor (ADC counts)    b: Read current sensor (mA)\n');
    fprintf('c: Read encoder (counts)               d: Read encoder (deg)\n');
    fprintf('e: Reset encoder                       f: Set PWM (-100 to 100)\n');
    fprintf('g: Set current gains                   h: Get current gains\n');
    fprintf('i: Set position gains                  j: Get position gains\n');
    fprintf('k: Test current control                l: Go to angle (deg)\n');
    fprintf('m: Load step trajectory                n: Load cubic trajectory\n');
    fprintf('o: Execute trajectory                  p: Unpower the motor\n');
    fprintf('q: Quit client                         r: Get mode\n');
    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);
    
    % take the appropriate action
    switch selection
        case 'a'                         
            n = fscanf(mySerial,'%d');
            fprintf('The motor current is %d ADC counts\n\n',n);
        
        case 'b'                         
            n = fscanf(mySerial,'%d');
            fprintf('The motor current is %d mA\n\n',n);

        case 'c'                         
            n = fscanf(mySerial,'%d');   % get the encoder count from PIC32
            fprintf('The motor angle is %d counts\n\n',n);     % print it to the screen
            
        case 'd'                         
            n = fscanf(mySerial,'%d');   
            fprintf('The motor angle is %d degrees\n\n',n);
            
        case 'e'                         
            fprintf('The motor angle has been reset\n\n');  

        case 'f'
            % read the user's desired duty cycle value:                      
            duty = input('\nWhat PWM value would you like [-100 to 100]? ');           
            % send the value to PIC32:
            fprintf(mySerial,'%d\n', duty);

            if (duty < -100)
                fprintf('PWM has been set to 100%% in the negative direction.\n\n');
            elseif (duty > 100)
                fprintf('PWM has been set to 100%% in the positive direction.\n\n');                
            elseif (duty < 0)
                fprintf('PWM has been set to %d%% in the negative direction.\n\n', -duty);
            else
                fprintf('PWM has been set to %d%% in the positive direction.\n\n', duty);
            end
        
        case 'r'
            n = fscanf(mySerial,'%d');
            switch n          
                case 0
                    fprintf('The PIC32 controller mode is currently IDLE\n\n');
                case 1
                    fprintf('The PIC32 controller mode is currently PWM\n\n');
                case 2
                    fprintf('The PIC32 controller mode is currently ITEST\n\n');
                case 3
                    fprintf('The PIC32 controller mode is currently HOLD\n\n');
                case 4
                    fprintf('The PIC32 controller mode is currently TRACK\n\n');
            end
                              
            
%         case 'x'                         % example operation
%             n = input('Enter the first number: '); % get the number to send
%             m = input('Enter the second number: '); % get the number to send
%             fprintf(mySerial, '%d %d\n',[n, m]); % send the numbers
% 
%             n = fscanf(mySerial,'%d');   % get the incremented number back
%             fprintf('Read: %d\n',n);     % print it to the screen
        case 'q'
            has_quit = true;             % exit client
        otherwise
            fprintf('Invalid Selection %c\n\n', selection);
    end
end

end
