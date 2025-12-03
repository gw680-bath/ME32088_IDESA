function keyboard_control_dir()
% GANTRY_KEYBOARD_CONTROL
% Sends simple direction commands to the Raspberry Pi via UDP:
%   +1 = forward, -1 = backward, 0 = stop
% Use LEFT / RIGHT arrow keys, release to stop.

    % ==== CONFIGURE THIS ====
    piIP   = '169.254.1.107';   % <-- change to your Pi's IP address
    piPort = 55001;            % <-- must match UDP Receive block in Simulink
    % =========================

    % Create UDP object
    u = udpport("Datagram","IPV4");

    % Send a stop command at start (safety)
    write(u, 0, "double", piIP, piPort);

    % Create a small control window to capture key events
    f = figure('Name','Gantry Keyboard Control', ...
               'NumberTitle','off', ...
               'MenuBar','none', ...
               'ToolBar','none', ...
               'KeyPressFcn',  @(src,event) keyDown(event,u,piIP,piPort), ...
               'KeyReleaseFcn',@(src,event) keyUp(event,u,piIP,piPort), ...
               'CloseRequestFcn',@(src,event) closeFig(src,event,u,piIP,piPort));

    uicontrol('Style','text', ...
              'String', sprintf('Use LEFT/RIGHT arrows to move.\nRelease to stop.\nClose window to exit.'), ...
              'Units','normalized', ...
              'Position',[0.1 0.3 0.8 0.4], ...
              'FontSize',12);

    disp('Gantry keyboard control started.');
    disp('RIGHT arrow = forward, LEFT arrow = backward, release = stop.');
    disp('Close the window to quit.');
end

function keyDown(event,u,piIP,piPort)
    switch event.Key
        case 'rightarrow'
            cmd = 1;   % forward
        case 'leftarrow'
            cmd = -1;  % backward
        otherwise
            return;    % ignore other keys
    end
    write(u, cmd, "double", piIP, piPort);
end

function keyUp(event,u,piIP,piPort)
    % When an arrow key is released, send stop
    switch event.Key
        case {'rightarrow','leftarrow'}
            cmd = 0;   % stop
            write(u, cmd, "double", piIP, piPort);
        otherwise
            return;
    end
end

function closeFig(src,~,u,piIP,piPort)
    % On window close: send stop, clean up UDP, then close figure
    write(u, 0, "double", piIP, piPort);
    clear u;
    delete(src);
end
