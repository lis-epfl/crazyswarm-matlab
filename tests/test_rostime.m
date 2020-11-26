% test_rostime
%
% This test clocks the call to the ros time function in two
% different ways.
%

rosinit

tic
for i = 1:1000
    time = rostime('now');
end
rostime_time = toc / 1000;
fprintf( 'Average Time for rostime: %d s \n' , rostime_time);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic
t = ros.internal.Time([]);
for i = 1:1000
    currentTime = t.CurrentTime;
end
rostime_time = toc / 1000;
fprintf( 'Average Time for ros.internal: %d s \n' , rostime_time);

rosshutdown
