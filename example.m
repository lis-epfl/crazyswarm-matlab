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

swarm = Crazyswarm("crazyflies.yaml");

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
while t < 8
    pos = [r * cos(w * t), r * sin(w * t), Z];
    vel = [-r * w * sin(w * t), r * w * cos(w * t), 0];
    acc = [-r * w * w * cos(w * t), -r * w * w * sin(w * t), 0];
    yaw = w * t;
    omega = [0, 0, w];

    for cf = swarm.crazyflies
        cf{1}.cmdFullState(cf{1}.initialPosition + pos, vel, acc, yaw, omega)
    end
    t = t + dt;
    waitfor(rate);
end

% Switch to high-level mode and go back to initial position
for cf = swarm.crazyflies
    cf{1}.notifySetpointsStop(100)
    cf{1}.goTo(cf{1}.initialPosition + [0, 0, Z], 0, 2)
end
pause(2)

% Land
swarm.land(0.02, 2)
pause(2)