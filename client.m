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
    fprintf('\nPIC32 MOTOR DRIVER INTERFACE\n\n');
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
        % READ CURRENT SENSOR (ADC COUNTS): 
        case 'a'                         
            n = fscanf(mySerial,'%d');
            fprintf('\nThe motor current is %d ADC counts\n',n);
        
        % READ CURRENT SENSOR (mA): 
        case 'b'                         
            n = fscanf(mySerial,'%d');
            fprintf('\nThe motor current is %d mA\n',n);

        % READ ENCODER (COUNTS): 
        case 'c'                         
            n = fscanf(mySerial,'%d');  
            fprintf('\nThe motor angle is %d counts\n',n); 
           
        % READ ENCODER (DEGS): 
        case 'd'                         
            n = fscanf(mySerial,'%d');   
            fprintf('\nThe motor angle is %d degrees\n',n);
        
        % RESET ENCODER COUNTS:  
        case 'e'                         
            fprintf('\nThe motor angle has been reset\n');  

        % SET PWM:
        case 'f'
            % read the user's desired duty cycle value:                      
            duty = input('\nWhat PWM value would you like [-100 to 100]? ');           
            % send the value to PIC32:
            fprintf(mySerial,'%d\n', duty);

            if (duty < -100)
                fprintf('PWM has been set to 100%% in the negative direction.\n');
            elseif (duty > 100)
                fprintf('PWM has been set to 100%% in the positive direction.\n');                
            elseif (duty < 0)
                fprintf('PWM has been set to %d%% in the negative direction.\n', -duty);
            else
                fprintf('PWM has been set to %d%% in the positive direction.\n', duty);
            end
        
        % SET CURRENT GAINS:
        case 'g'                         
            Kp_cur = input('\nEnter your desired Kp current gain [recommended: 0.75]: ');
            Ki_cur = input('\nEnter your desired Ki current gain [recommended: 0.05]: ');
            fprintf(mySerial, '%f %f\n',[Kp_cur, Ki_cur]);
            fprintf('\nSending Kp = %.2f and Ki = %.2f to the current controller.\n',Kp_cur,Ki_cur);

        % GET CURRENT GAINS:
        case 'h'                         
            n = fscanf(mySerial,'%f\n');
            fprintf('The current controller is using Kp = %f \n', n); 
            m = fscanf(mySerial,'%f\n');  
            fprintf('The current controller is using Ki = %f \n', m);

        % SET POSITION GAINS:
        case 'i'                         
            Kp_pos = input('\nEnter your desired Kp position gain [recommended: 150]: ');
            Ki_pos = input('\nEnter your desired Ki position gain [recommended: 0]: ');
            Kd_pos = input('\nEnter your desired Kd position gain [recommended: 5000]: ');
            fprintf(mySerial, '%f %f %f\n',[Kp_pos, Ki_pos, Kd_pos]);
            fprintf('\nSending Kp = %.2f, Ki = %.2f, and Kd = %.2f to the current controller.\n',Kp_pos,Ki_pos,Kd_pos);

        % GET POSITION GAINS:
        case 'j'                         
            n = fscanf(mySerial,'%f\n');
            fprintf('The position controller is using Kp = %f \n', n); 
            m = fscanf(mySerial,'%f\n');  
            fprintf('The position controller is using Ki = %f \n', m);
            o = fscanf(mySerial,'%f\n');  
            fprintf('The position controller is using Kd = %f \n', o);

        % TEST CURRENT CONTROL:
        case 'k'                         
            read_plot_matrix(mySerial);

        % GO TO ANGLE (DEG):
        case 'l'                         
            des_ang = input('\nEnter the desired motor angle in degrees: ');
            fprintf(mySerial, '%d\n', des_ang);
            fprintf('Motor moving to %d degrees. \n', des_ang);

        % LOAD STEP TRAJECTORY:
        case 'm'                         
            des_traj = input('\nEnter step trajectory, in sec and degrees [time1, ang1; time2, ang2; ...]: ');
            step_traj = genRef(des_traj, 'step');
            num_samples = size(step_traj, 2);
            if num_samples > 2000
                fprintf('\nError: Maximum trajectory time is 10 seconds.\n'); 
            end

            fprintf(mySerial, '%d\n', num_samples);  
            for i=1:num_samples
                fprintf(mySerial, '%d\n',step_traj(i));
            end
            fprintf('Plotting the desired trajectory and sending to PIC32 ... completed.\n')

        % LOAD CUBIC TRAJECTORY:
        case 'n'                         
            des_traj = input('\nEnter cubic trajectory, in sec and degrees [time1, ang1; time2, ang2; ...]: ');
            step_traj = genRef(des_traj, 'cubic');
            num_samples = size(step_traj, 2);
            if num_samples > 2000
                fprintf('\nError: Maximum trajectory time is 10 seconds.\n'); 
            end

            fprintf(mySerial, '%d\n', num_samples);  
            for i=1:num_samples
                fprintf(mySerial, '%d\n',step_traj(i));
            end
            fprintf('Plotting the desired trajectory and sending to PIC32 ... completed.\n')

        % EXECUTE TRAJECTORY AND PLOT:
        case 'o'                         
            read_plot_matrix_pos(mySerial);

        % UNPOWER MOTOR:
        case 'p'                         
            fprintf('\n Motor is unpowered.\n')

        % EXIT CLIENT:
        case 'q'
            has_quit = true;             

        % GET MODE:
        case 'r'
            n = fscanf(mySerial,'%d');
            switch n          
                case 0
                    fprintf('The PIC32 controller mode is currently IDLE\n');
                case 1
                    fprintf('The PIC32 controller mode is currently PWM\n');
                case 2
                    fprintf('The PIC32 controller mode is currently ITEST\n');
                case 3
                    fprintf('The PIC32 controller mode is currently HOLD\n');
                case 4
                    fprintf('The PIC32 controller mode is currently TRACK\n');
            end
                              
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
end

end
