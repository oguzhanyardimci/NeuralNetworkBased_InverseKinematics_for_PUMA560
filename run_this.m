%% Load the neural-network model
position_values = load('position_values.mat');
theta_values = load('theta_values.mat');
position_values = position_values.position_matrix;
theta_values = theta_values.theta_values;
net = load('inverse_kinematic_network4.mat');
net = net.net;
tr = load('inverse_kinematic_network4_tr.mat');
tr = tr.tr;

%% Put inputs 
fprintf('\nNumber of iteration that has done in this training is: %d\n\n', tr.num_epochs);

%Plot the cost function
figure(1)
plot(tr.gradient);
axis([0 250 0 4])
hold on
title('Cost Function');
xlabel('Iteration');
ylabel('Cost');
pause(0.5);
hold off

fprintf('This program calculates the inverse kinematics of a 3-DOF robot arm.\n');
fprintf('There are 14445 samples in the data set.\n');
fprintf('The working range of the robot arm for given joint values are like that:\n\n');
fprintf('Min position of the robot for X-axis:%.2f m\n',min(position_values(1,:)));
fprintf('Max position of the robot for X-axis:%.2f m\n\n',max(position_values(1,:)));

fprintf('Min position of the robot for Y-axis:%.2f m\n',min(position_values(2,:)));
fprintf('Max position of the robot for Y-axis:%.2f m\n\n',max(position_values(2,:)));

fprintf('Min position of the robot for Z-axis:%.2f m\n',min(position_values(3,:)));
fprintf('Max position of the robot for Z-axis:%.2f m\n\n',max(position_values(3,:)));

selection = input('Type -data- to work with training data. Type -new- to try new data:\n', 's');

if selection == "data"
    fprintf('Please type the index that you want to calculates its inverse kinematics.\n');
    index_str = input('Index Number:','s');
    index = str2double(index_str);
    
    figure(2)
    plot3(position_values(1,:),position_values(2,:), position_values(3,:))
    hold on
    title('Work space of the robot arm');
    hold off
    
    a = net(position_values(:,index));
    fprintf('The joint values produced by network for that position will be like this:\n');
    fprintf('Theta1 = %.2f\n', a(1));
    fprintf('Theta2 = %.2f\n', a(2));
    fprintf('Theta3 = %.2f\n', a(3));
    
    fprintf('\nReal theta values for this position are like that:\n');
    fprintf('Theta1 = %.2f\n', theta_values(1,index));
    fprintf('Theta2 = %.2f\n', theta_values(2,index));
    fprintf('Theta3 = %.2f\n', theta_values(3,index));
    
    fprintf('\nReal postion for this index are like that:\n');
    fprintf('X position = %.2f\n', position_values(1,index));
    fprintf('Y position = %.2f\n', position_values(2,index));
    fprintf('Z position = %.2f\n', position_values(3,index));
    
    fprintf('\nAfter we calculate the forward kineamtics with the values produced by neural network:\n');
    forward_kinematic_for_nn = forward_kinematic_calculator(a);
    fprintf('X position = %.2f\n', forward_kinematic_for_nn(1));
    fprintf('Y position = %.2f\n', forward_kinematic_for_nn(2));
    fprintf('Z position = %.2f\n', forward_kinematic_for_nn(3));
    
    error_x = 100*((abs((forward_kinematic_for_nn(1)) - (position_values(1,index))))/ (abs(max(position_values(1,:))) + abs(min(position_values(1,:)))));
    error_y = 100*((abs((forward_kinematic_for_nn(2)) - (position_values(2,index))))/ (abs(max(position_values(2,:))) + abs(min(position_values(2,:)))));
    error_z = 100*((abs((forward_kinematic_for_nn(3)) - (position_values(3,index))))/ (abs(max(position_values(3,:))) + abs(min(position_values(3,:)))));
    
    fprintf('\nErrors between real and calculated positions:\n');
    fprintf('Error for the X position = %.2f\n', error_x);
    fprintf('Error for the Y position = %.2f\n', error_y);
    fprintf('Error for the Z position = %.2f\n', error_z);
    
else
    fprintf('\nPlease put X,Y,Z values between the values stated in the above.\n');
    x_input = input('Put x-axis position:','s');
    x_input = str2double(x_input);
    
    y_input = input('Put y-axis position:','s');
    y_input = str2double(y_input);
    
    z_input = input('Put z-axis position:','s');
    z_input = str2double(z_input);
    
    input_for_nn = [x_input; y_input; z_input];
    
    theta_output_nn = net(input_for_nn);
    fprintf('\nTheta values produced by neural network are like that:\n');
    fprintf('Theta1 = %.2f\n', theta_output_nn(1));
    fprintf('Theta2 = %.2f\n', theta_output_nn(2));
    fprintf('Theta3 = %.2f\n', theta_output_nn(3));
    
    fprintf('\nThe calculated positions will be like this:\n');
    calculated_positions = forward_kinematic_calculator(theta_output_nn);
    fprintf('Calculated x position: %.2f\n', calculated_positions(1));
    fprintf('Calculated y position: %.2f\n', calculated_positions(2));
    fprintf('Calculated z position: %.2f\n', calculated_positions(3));
    
    error_x = 100*((abs(calculated_positions(1) - x_input))/(abs(max(position_values(1,:))) + abs(min(position_values(1,:)))));
    error_y = 100*((abs(calculated_positions(2) - y_input))/(abs(max(position_values(2,:))) + abs(min(position_values(2,:)))));
    error_z = 100*((abs(calculated_positions(3) - z_input))/(abs(max(position_values(3,:))) + abs(min(position_values(3,:)))));
    
    fprintf('\nErrors in kinematic relative to workspace:\n'); 
    fprintf('Error in x-axis:%.2f\n', error_x);
    fprintf('Error in y-axis:%.2f\n', error_y);
    fprintf('Error in z-axis:%.2f\n', error_z); 
end






