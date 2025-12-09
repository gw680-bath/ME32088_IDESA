function keyboard_control_dir2()
% Sends dual motor direction commands to the Raspberry Pi via UDP:
%   [dir1, dir2], each ∈ {-1, 0, 1}


    piIP   = '138.38.226.147';
    piPort = 55001;

    % Create UDP object
    u = udpport("Datagram","IPV4");

    % Send a stop command at start (safety)
    write(u, [0 0], "double", piIP, piPort);

    % Create a small control window to capture key events
    f = figure('Name','Dual Motor Keyboard Control', ...
               'NumberTitle','off', ...
               'MenuBar','none', ...
               'ToolBar','none', ...
               'KeyPressFcn',  @(src,event) keyDown(event,u,piIP,piPort), ...
               'KeyReleaseFcn',@(src,event) keyUp(event,u,piIP,piPort), ...
               'CloseRequestFcn',@(src,event) closeFig(src,event,u,piIP,piPort));

    uicontrol('Style','text', ...
              'String', sprintf([ ...
                  'Use ARROW KEYS to control two motors:\n' ...
                  'UP:    both forward\n' ...
                  'DOWN:  both backward\n' ...
                  'LEFT:  M1 forward, M2 backward\n' ...
                  'RIGHT: M1 backward, M2 forward\n' ...
                  'Release key: both stop\n' ...
                  'Close window to exit.']), ...
              'Units','normalized', ...
              'Position',[0.1 0.2 0.8 0.6], ...
              'FontSize',12);

    disp('Dual motor keyboard control started.');
    disp('UP/DOWN/LEFT/RIGHT arrows control the two motors; release to stop.');
    disp('Close the window to quit.');
end

function keyDown(event,u,piIP,piPort)
    switch event.Key
        case 'uparrow'
            cmd = [ 1  1];   % both forward
        case 'downarrow'
            cmd = [-1 -1];   % both backward
        case 'leftarrow'
            cmd = [ 1 -1];   % motor1 forward, motor2 backward
        case 'rightarrow'
            cmd = [-1  1];   % motor1 backward, motor2 forward
        otherwise
            return;          % ignore other keys
    end
    write(u, cmd, "double", piIP, piPort);
end

function keyUp(event,u,piIP,piPort)
    % When any arrow key is released, send stop for both motors
    switch event.Key
        case {'uparrow','downarrow','leftarrow','rightarrow'}
            cmd = [0 0];   % stop both
            write(u, cmd, "double", piIP, piPort);
        otherwise
            return;
    end
end

function closeFig(src,~,u,piIP,piPort)
    % On window close: send stop, clean up UDP, then close figure
    write(u, [0 0], "double", piIP, piPort);
    clear u;
    delete(src);
end
