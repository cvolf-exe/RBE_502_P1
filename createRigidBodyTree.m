function [robot,homeConfig] = createRigidBodyTree
% Creates a robotics.RigidBodyTree object to be used by the
% forward and inverse kinematics blocks.

% Copyright 2017-2018 The MathWorks, Inc.

% Import the robot description from the URDF file
try
    robot = importrobot('/Users/colette/Documents/Courses/RBE 502/RBE_502_P1/open_manipulator_x.urdf');
    
    % Add gravity
    gravityVec = [0 0 -9.80665];
    robot.Gravity = gravityVec;
        
    % Add another massless coordinate frame for the end effector
    eeOffset = 0.12;
    eeBody = robotics.RigidBody('end_effector');
    eeBody.Mass = 0;
    eeBody.Inertia = [0 0 0 0 0 0];
    setFixedTransform(eeBody.Joint,trvec2tform([eeOffset 0 0]));
    addBody(robot,eeBody,'link5');
    
% If this fails, load the preloaded rigid body tree
catch err
    warning('Error importing URDF file (%s). Loading presaved object.', err.message);
    load openManipulatorDescription robot
end

% Return its home configuration
homeConfig = robot.homeConfiguration;

end
