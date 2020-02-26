%%
clear all
clc
%% Theta values
theta_values_1 = linspace(-1.5, 1.0, 26);
theta_values_2 = linspace(-1.5, 0.7, 23);
theta_values_3 = linspace(-0.8, 1.5, 24);

length1 = length(theta_values_1);
length2 = length(theta_values_2);
length3 = length(theta_values_3);
lenght_of_for_loop = length1*length2*length3;

n = 0;
theta_values = [];

%% for loops crate theta values

for i=1:length1
    for j=1:length2
        for k=1:length3
            adding_matrix = [theta_values_1(i), theta_values_2(j), theta_values_3(k)];
            theta_values = cat(1, theta_values, adding_matrix);
            fprintf('Step = %d of %d\n', n, lenght_of_for_loop);
            n = n+1;
        end
    end
end

theta_values = transpose(theta_values);
%% forward kinematic equations
len_theta = length(theta_values);

x_position = ones(len_theta,1);
y_position = ones(len_theta,1);
z_position = ones(len_theta,1);

position_matrix = ones(3,len_theta);

for n=1:length(theta_values)
    position_matrix(:,n) = forward_kinematic_calculator(theta_values(:,n));
end

clear x_position;
clear y_position;
clear z_position;

clear theta_values_1;
clear theta_values_2;
clear theta_values_3;
