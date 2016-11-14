% Set up the vectors to be plotted
t = result.t(1,:);
x = result.x(1,:);
y = result.x(2,:);
z = result.x(3,:);
roll = result.theta(1,:);
pitch = result.theta(2,:);
yaw = result.theta(3,:);

% Plot the vectors with labels and titles
plot(t,x,'DisplayName','x direction')
hold on
plot(t,y,'DisplayName','y direction')
plot(t,z,'DisplayName','Height')
title('Quadrotor Simulation Results')
xlabel('Time (sec)')
ylabel('Distance (m)')
legend('show')
grid on
hold off

% Angle plot
figure;
plot(t,roll,'DisplayName','Roll')
hold on
plot(t,pitch,'DisplayName','Pitch')
plot(t,yaw,'DisplayName','Yaw')
xlabel('Time (sec)')
ylabel('Angle (radians)')
legend('show')
grid on
hold off

% 3D plot of quadrotor movement
figure;
plot3(x,y,z)
xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
grid on

