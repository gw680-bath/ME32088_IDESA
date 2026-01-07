function keyboard_control_dir3()
% KEYBOARD_CONTROL_DIR2 (Dual motor, fixed send rate + verification)
% Sends [dir1 dir2] to the Raspberry Pi at a fixed rate (e.g. 30 Hz)
% using UDP. Arrow keys set the command; releasing stops.
%
% Mapping:
%   UP:    [ 1  1]
%   DOWN:  [-1 -1]
%   LEFT:  [ 1 -1]
%   RIGHT: [-1  1]
%   Release: [0 0]

    % ==== CONFIGURE THIS ====
    piIP   = '138.38.226.147';
    piPort = 55001;

    sendRateHz = 30;         % 20–50 Hz is ideal
    period     = 1/sendRateHz;
    % =========================

    % UDP object
    u = udpport("Datagram","IPV4");

    % Shared state for command (updated by key events)
    cmd = [0 0];  % [dir1 dir2], each in {-1,0,1}

    % Verification counters
    count = 0;
    t0 = tic;

    % Send a stop command at start (safety)
    write(u, cmd, "double", piIP, piPort);

    % Create the control window
    f = figure('Name','Dual Motor Keyboard Control', ...
               'NumberTitle','off', ...
               'MenuBar','none', ...
               'ToolBar','none', ...
               'KeyPressFcn',  @(src,event) keyDown(event), ...
               'KeyReleaseFcn',@(src,event) keyUp(event), ...
               'CloseRequestFcn',@(src,event) closeFig(src));

    uicontrol('Style','text', ...
              'String', sprintf([ ...
                  'Arrow keys control two motors.\n' ...
                  'UP: [1 1], DOWN: [-1 -1], LEFT: [1 -1], RIGHT: [-1 1]\n' ...
                  'Release any arrow key to stop.\n' ...
                  'Sending at fixed rate with 1 Hz printout in MATLAB.\n' ...
                  'Close window to exit.']), ...
              'Units','normalized', ...
              'Position',[0.05 0.25 0.9 0.5], ...
              'FontSize',11);

    disp('Dual motor keyboard control started.');
    disp('Close the window to quit.');

    % Fixed-rate send loop (Method 2) — runs until window is closed
    while isvalid(f)
        tStart = tic;

        % Send current command
        write(u, cmd, "double", piIP, piPort);

        % Verification: print send rate once per second
        count = count + 1;
        if toc(t0) >= 1.0
            fprintf("Send rate: %d Hz (target %g Hz)\n", count, sendRateHz);
            count = 0;
            t0 = tic;
        end

        % Sleep remaining time to enforce send rate
        elapsed = toc(tStart);
        pause(max(0, period - elapsed));
    end

    % If figure closed unexpectedly, still try to stop
    try
        write(u, [0 0], "double", piIP, piPort);
    catch
    end
    clear u;

    % --- Nested callbacks share 'cmd' by closure ---

    function keyDown(event)
        switch event.Key
            case 'uparrow'
                cmd = [ 1  1];
            case 'downarrow'
                cmd = [-1 -1];
            case 'leftarrow'
                cmd = [ 1 -1];
            case 'rightarrow'
                cmd = [-1  1];
            otherwise
                return;
        end
    end

    function keyUp(event)
        switch event.Key
            case {'uparrow','downarrow','leftarrow','rightarrow'}
                cmd = [0 0];  % stop both on arrow release
            otherwise
                return;
        end
    end

    function closeFig(figHandle)
        % Stop motors on close
        cmd = [0 0];
        try
            write(u, cmd, "double", piIP, piPort);
        catch
        end
        delete(figHandle);
    end
end
