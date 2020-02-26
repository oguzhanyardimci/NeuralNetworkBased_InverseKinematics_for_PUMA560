function postionOutput = forward_kinematic_calculator(thetaInput)

a1 = 0;
a2 = 0.4318;
a3 = 0.0203;
a4 = 0;
a5 = 0;
a6 = 0;

d1 = 0;
d2 = 0;
d3 = 0.1500;
d4 = 0.4318;
d5 = 0;
d6 = 0;

alpha1 = pi/2;
alpha2 = 0;
alpha3 = -pi/2;
alpha4 = pi/2;
alpha5 = -pi/2;
alpha6 = 0;

theta1 = thetaInput(1);
theta2 = thetaInput(2);
theta3 = thetaInput(3);

x_position = d3*sin(theta1)-d4*cos(theta1)*(cos(theta2)*sin(theta3) ...
    - cos(theta3)*sin(theta2)) + a2*cos(theta1)*cos(theta2) + ...
    a3*cos(theta1)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3));

y_position = a2*cos(theta2)*sin(theta1)-d3*cos(theta1)-d4*sin(theta1)*( ...
    cos(theta2)*sin(theta3)+cos(theta3)*sin(theta2)) + a3*sin(theta1)*( ...
    cos(theta2)*cos(theta3)-sin(theta2)*sin(theta3));

z_position = d4*cos(theta2+theta3)+a3*sin(theta2+theta3)+a2*sin(theta2);

postionOutput = [x_position; y_position; z_position];

end