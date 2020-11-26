%% Example script to demonstrate the use of the crazyswarm-matlab API

clear
clc

% Parameters
Z = 0.5;
r = 0.75;
w = 2 * pi / 8;
t = 0;
dt = 0.1;
rate = rosrate(1 / dt);

% Initialize log variables
state_history = [];
cmd_history = [];
t_history = [];

swarm = Crazyswarm("crazyflies.yaml");
time = ros.internal.Time;  % Faster than calling rostime("now")

% Takeoff
swarm.takeoff(Z, 1.0+Z)
pause(1.5 + Z)

% Move to start of circle
for cf = swarm.crazyflies
    pos = [r * cos(w * t), r * sin(w * t), Z];
    cf{1}.goTo(cf{1}.initialPosition + pos, 0, 2)
end
pause(2)


% Fly in circle
reset(rate)
t_id = 1;
tic
while t < 8
    ros_time = time.CurrentTime;
    t_history(t_id) = ros_time.Sec + ros_time.Nsec / 1e9;
    
    state_history(:, :, t_id) = swarm.state();
    
    pos = [r * cos(w * t), r * sin(w * t), Z];
    vel = [-r * w * sin(w * t), r * w * cos(w * t), 0];
    acc = [-r * w * w * cos(w * t), -r * w * w * sin(w * t), 0];
    yaw = w * t;
    omega = [0, 0, w];
    
    for i = 1:length(swarm.crazyflies)
        cf = swarm.crazyflies{i};
        cf.cmdFullState(cf.initialPosition + pos, vel, acc, yaw, omega)
        cmd_history(:, i, t_id) = [cf.initialPosition + pos, vel]';
    end
    
    t = t + dt;
    t_id = t_id + 1;
    waitfor(rate);
end
toc

% Switch to high-level mode and go back to initial position
for cf = swarm.crazyflies
    cf{1}.notifySetpointsStop(100)
    cf{1}.goTo(cf{1}.initialPosition + [0, 0, Z], 0, 2)
end
pause(2)

% Land
swarm.land(0.02, 2)
pause(2)

plotting(t_history, state_history, cmd_history)