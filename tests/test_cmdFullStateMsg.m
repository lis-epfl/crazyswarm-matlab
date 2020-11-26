% test_cmdFullStateMsg
%
% This test clocks the assignment of a message cmdFullStateMsg in two
% different ways.
%
% To execute this test you need to have the crazyflie_driver messages
% installed. To do so, please execute:
%   rosgenmsg("custom_msgs")
% or, for fast run (Matlab >= 2020b), execute:
%   rosgenmsg("custom_msgs",'BuildConfiguration','fasterruns')
% Then, run this test.
%

cmdFullStateMsg = rosmessage("crazyflie_driver/FullState");
pos = zeros(3, 1);
vel = ones(3, 1);
acc = ones(3, 1);
yaw = 1;
omega = zeros(3, 1);
tic
for i = 1:1000
    quat = eul2quat([0, 0, yaw], 'xyz');
    cmdFullStateMsg.Pose.Position.X    = pos(1);
    cmdFullStateMsg.Pose.Position.Y    = pos(2);
    cmdFullStateMsg.Pose.Position.Z    = pos(3);
    cmdFullStateMsg.Twist.Linear.X     = vel(1);
    cmdFullStateMsg.Twist.Linear.Y     = vel(2);
    cmdFullStateMsg.Twist.Linear.Z     = vel(3);
    cmdFullStateMsg.Acc.X              = acc(1);
    cmdFullStateMsg.Acc.Y              = acc(2);
    cmdFullStateMsg.Acc.Z              = acc(3);
    cmdFullStateMsg.Pose.Orientation.W = quat(1);
    cmdFullStateMsg.Pose.Orientation.X = quat(2);
    cmdFullStateMsg.Pose.Orientation.Y = quat(3);
    cmdFullStateMsg.Pose.Orientation.Z = quat(4);
    cmdFullStateMsg.Twist.Angular.X    = omega(1);
    cmdFullStateMsg.Twist.Angular.Y    = omega(2);
    cmdFullStateMsg.Twist.Angular.Z    = omega(3);
end
time = toc / 1000;
fprintf( 'Average Time per message: %d s \n' , time);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cmdFullStateMsg = rosmessage("crazyflie_driver/FullState");
pose = cmdFullStateMsg.Pose;
posePos = pose.Position;
poseOrient = pose.Orientation;
twist = cmdFullStateMsg.Twist;
twistLin = twist.Linear;
twistAng = twist.Angular;
accel = cmdFullStateMsg.Acc;

pos = zeros(3, 1);
vel = ones(3, 1);
acc = ones(3, 1);
yaw = 1;
omega = zeros(3, 1);

tic
for i = 1:1000
    quat = eul2quat([0, 0, yaw], 'xyz');
    posePos.X    = pos(1);
    posePos.Y    = pos(2);
    posePos.Z    = pos(3);
    twistLin.X   = vel(1);
    twistLin.Y   = vel(2);
    twistLin.Z   = vel(3);
    accel.X      = acc(1);
    accel.Y      = acc(2);
    accel.Z      = acc(3);
    poseOrient.W = quat(1);
    poseOrient.X = quat(2);
    poseOrient.Y = quat(3);
    poseOrient.Z = quat(4);
    twistAng.X   = omega(1);
    twistAng.Y   = omega(2);
    twistAng.Z   = omega(3);
end

time = toc / 1000;
fprintf( 'Average Time per message with prebuilt: %d s \n' , time);
