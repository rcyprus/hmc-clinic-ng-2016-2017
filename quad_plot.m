% Set up the vectors to be plotted
t = result.t(1,:);
x = result.x(1,:);
y = result.x(2,:);
z = result.x(3,:);
roll = result.theta(1,:);
pitch = result.theta(2,:);
yaw = result.theta(3,:);
x_start = x(1);
y_start = y(1);
z_start = z(1);

% Setup the subplot 
figure; 
plots = [subplot(3, 2, 1:3), subplot(3, 2, 5), subplot(3, 2, 6)];

% Plot the vectors with labels and titles
subplot(plots(2))
plot(t,x,'DisplayName','x direction')
hold on
plot(t,y,'DisplayName','y direction')
plot(t,z,'DisplayName','Height')
title('Height Data')
xlabel('Time (sec)')
ylabel('Distance (m)')
legend('show', 'Location', 'southwest')
grid on
hold off

% Angle plot
subplot(plots(3))
plot(t,roll,'DisplayName','Roll')
hold on
plot(t,pitch,'DisplayName','Pitch')
plot(t,yaw,'DisplayName','Yaw')
title('Angle Data')
xlabel('Time (sec)')
ylabel('Angle (radians)')
legend('show', 'Location', 'southwest')
grid on
hold off

% 3D plot of quadrotor movement
subplot(plots(1))
plot3(x_start,y_start,z_start,'ro','LineWidth', 2)
hold on
plot3(x,y,z,'LineWidth', 2)
xlabel('x','FontWeight','bold')
ylabel('y','FontWeight','bold')
zlabel('z','FontWeight','bold')
axis([0 6 0 6 0 15])
legend('Starting Point','Location','northwest')
title('Quadrotor Simulation Results')
grid on

