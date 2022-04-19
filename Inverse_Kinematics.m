% Importing and Displaying the Robot
robot = loadrobot('frankaEmikaPanda');

% Adding an end effector (in both cases, coinciding with the origin of the last considered frame)
% In Case 1, the end effector is in at the origin of frame 7
% In Case 2, the end effector is at the origin of frame 2
newPoint = robotics.RigidBody('end_effector');
newPoint.Mass = 0;
newPoint.Inertia = [0 0 0 0 0 0];
setFixedTransform(newPoint.Joint,trvec2tform([0 0 0]));
addBody(robot,newPoint,'panda_hand');   % Case 1
%addBody(robot,newPoint,'panda_link2'); % Case 2

% Using the inverse Kinemtaics Solver to get the joint parameters for the
% desired end effector position
point = [0.088 0 1.033];    % Case 1
%point = [0 0 0.33];        % Case 2
ik = inverseKinematics('RigidBodyTree',robot);
% We use weights of 0 (first three) for orientation & 1 (last three) for position since we are more
% concerned with the robot reaching the desired position
weights = [0 0 0 1 1 1];
end_effector = 'end_effector';
transform = trvec2tform(point);
disp(transform)
initialGuess = robot.homeConfiguration;
[solution, solinfo] = ik(end_effector,transform,weights,initialGuess);
show(robot,solution,'PreservePlot',false,'Frames','off')

% Displaying the obtained Joint Parameters
disp(struct2table(solution));