% Define the lengths of the robotic arm segments
ls = [1; 1];

% Define the number of iterations for the inverse kinematics algorithm
n = 10;

% Initialize figure for plotting
figure;
clf;

% Infinite loop for user interaction
while true
    % Get desired position from user
    desired = ginput(1)'; 

    % Iterate over both methods
    for mode = 0:1
        if mode == 1
            methodStr = "Newton";
        else
            methodStr = "Broyden";
        end

        % Display the selected method
        disp(['Using ', methodStr, ' method']);

        % Choose some random starting point for theta
        t = rand(2, 1);

        % Solve the inverse kinematics
        tic; % Start timing
        t = invKin2D(ls, t, desired, n, mode);
        elapsedTime = toc; % End timing

        % Display the robot and the result
        subplot(1, 2, mode+1);
        plot(desired(1), desired(2), 'r*', 'MarkerSize', 10); % Plot desired position
        hold on;
        plotRobot2D(ls, t); % Plot the robot's position
        title([methodStr, ' Method - Time: ', num2str(elapsedTime), 's']);
        hold off;
    end

    % Pause briefly to update plots
    pause(0.1);
end


% Newton's method typically converges faster than Broyden's method but requires exact derivatives or Jacobian matrices.
% Broyden's method is more computationally efficient, especially for complex functions where calculating derivatives or Jacobian matrices is resource-intensive.
% Broyden's method can be easier to implement when the derivative or Jacobian of the function is unknown or difficult to compute accurately.
% Newton's method can be more sensitive to the initial guess and may fail to converge in the presence of discontinuities or non-smooth functions.

