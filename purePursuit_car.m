path=[ 0.00 0.00
       0.50 0.50
       1.00 1.00
       1.50 1.50
       2.00 2.00
       2.50 2.50
       3.00 3.00
       3.50 3.50
       4.00 4.00]
   
% Set the current location and the goal location of the robot as defined by the path
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
initialSteer=0;
robotCurrentPose = [robotCurrentLocation initialOrientation initialSteer];

%differential robot simulator
robotRadius = 0.4;
dt = 0.05;
robot = ExampleHelperCarBot(robotCurrentPose, dt);
robot.showTrajectory(true);
robot.setCarPose(robotCurrentPose);

%visualize the trajectory
plot(path(:,1), path(:,2),'k--d')
xlim([0 13]);
ylim([0 13]);

%controller
controller = robotics.PurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.2;

%driving the bot
goalRadius = 0.1;
distanceToGoal = norm(robotCurrentLocation - robotGoal);
controlRate = robotics.Rate(10);
while( distanceToGoal > goalRadius )

    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robot.getCarPose);

    % Simulate the robot using the controller outputs.
    drive(robot, v, omega);

    % Extract current location information ([X,Y]) from the current pose of the
    % robot
    robotCurrentPose = robot.getCarPose;

    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);

    waitfor(controlRate);

end