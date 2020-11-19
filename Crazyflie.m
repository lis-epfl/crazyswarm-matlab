classdef Crazyflie < handle
    %CRAZYFLIE Class representing a single crazyflie.
    
    properties(GetAccess='public', SetAccess='private')
        % Properties
        id
        prefix
        tfTree
        initialPosition

        % Publishers
        cmdPositionPublisher
        cmdVelocityWorldPublisher
        cmdFullStatePublisher
         
        % Services
        setGroupMaskService
        takeoffService
        landService
        goToService
        notifySetpointsStopService
        % TODO: Implement updateParamsService
    end
    
    methods
        function obj = Crazyflie(id, initialPosition, tfTtree)
            %CRAZYFLIE Constructor for a Crazyflie
            obj.id = id;
            prefix = "cf" + num2str(id);
            obj.prefix = prefix;
            obj.initialPosition = initialPosition;
            obj.tfTree = tfTtree;
            
            obj.cmdPositionPublisher = rospublisher(prefix + "/cmd_position", ...
                                                    "crazyflie_driver/Position");
                                        
            obj.cmdVelocityWorldPublisher = rospublisher(prefix + "/cmd_velocity_world", ...
                                                         "crazyflie_driver/VelocityWorld");
            
            obj.cmdFullStatePublisher = rospublisher(prefix + "/cmd_full_state", ...
                                                     "crazyflie_driver/FullState");
                                               
                                               
            obj.setGroupMaskService = rossvcclient(prefix + "/set_group_mask");
            obj.takeoffService = rossvcclient(prefix + "/takeoff");
            obj.landService = rossvcclient(prefix + "/land");
            obj.goToService = rossvcclient(prefix + "/go_to");
            obj.notifySetpointsStopService = rossvcclient(prefix + "/notify_setpoints_stop");
        end
        
       function delete(obj)
            %DELETE Delete the Crazyflie
            clear obj.cmdPositionPublisher
            clear obj.cmdVelocityWorldPublisher
            clear obj.cmdFullStatePublisher
            clear obj.setGroupMaskService
            clear obj.takeoffService
            clear obj.landService
            clear obj.goToService
            clear obj.notifySetpointsStopService
        end
        
        function pos = position(obj)
            %POSITION Get position of crazyflie.
            % Latest known transformation
            tf = getTransform(obj.tfTree, "/world", obj.prefix, "Timeout", 10);
            % t = tf.Header.Stamp.Sec + tf.Header.Stamp.Nsec / 1e9;
            transl = tf.Transform.Translation;
            pos = [transl.X; transl.Y; transl.Z];
        end

        function vel = velocity(obj)
            %VELOCITY Get velocity of crazyflie.
            % Alternative code if velocity is calculated in in python
            % vel_msg = obj.VelSubscriber.LatestMessage;
            % vel = [vel_msg.Vector.X; vel_msg.Vector.Y; vel_msg.Vector.Z];
            dt = 0.1;
            timePrev = rostime("now") - dt;
            tfNow = getTransform(obj.tfTree, "/world", obj.prefix, "Timeout", 10);
            tfPrev = getTransform(obj.tfTree, "/world", obj.prefix, timePrev);
            
            translNow = tfNow.Transform.Translation;
            translPrev = tfPrev.Transform.Translation;
            
            posNow = [translNow.X; translNow.Y; translNow.Z];
            posPrev = [translPrev.X; translPrev.Y; translPrev.Z];
            
            vel = (posNow - posPrev) / dt;
        end

        function cmdPosition(obj, pos, yaw)
            %CMDPOSITION Sends a streaming command of absolute position and yaw setpoint.
            msg = rosmessage("crazyflie_driver/Position");
            msg.Header.Stamp   = rostime("now");
            msg.Header.FrameId = "/world";
            msg.X              = pos(1);
            msg.Y              = pos(2);
            msg.Z              = pos(3);
            msg.Yaw            = yaw;
            
            send(obj.cmdPositionPublisher, msg);
        end
        
        function cmdVelocityWorld(obj, vel, yawRate)
            %CMDVELOCITYWORLD Sends a streaming velocity-world controller setpoint command.
            msg = rosmessage("crazyflie_driver/VelocityWorld");
            msg.Header.Stamp   = rostime("now");
            msg.Header.FrameId = "/world";
            msg.Vel            = vel(1);
            msg.Vel            = vel(2);
            msg.Vel            = vel(3);
            msg.YawRate        = yawRate;
            
            send(obj.cmdVelocityWorldPublisher, msg);
        end
        
        function cmdFullState(obj, pos, vel, acc, yaw, omega)
            %CMDFULLSTATE Sends a streaming full-state controller setpoint command.
            quat = eul2quat([0, 0, yaw], 'xyz');
            msg = rosmessage('crazyflie_driver/FullState');
            msg.Header.Stamp       = rostime("now");
            msg.Header.FrameId     = "/world";
            msg.Pose.Position.X    = pos(1);
            msg.Pose.Position.Y    = pos(2);
            msg.Pose.Position.Z    = pos(3);
            msg.Twist.Linear.X     = vel(1);
            msg.Twist.Linear.Y     = vel(2);
            msg.Twist.Linear.Z     = vel(3);
            msg.Acc.X              = acc(1);
            msg.Acc.Y              = acc(2);
            msg.Acc.Z              = acc(3);
            msg.Pose.Orientation.W = quat(1);
            msg.Pose.Orientation.X = quat(2);
            msg.Pose.Orientation.Y = quat(3);
            msg.Pose.Orientation.Z = quat(4);
            msg.Twist.Angular.X    = omega(1);
            msg.Twist.Angular.Y    = omega(2);
            msg.Twist.Angular.Z    = omega(3);
            
            send(obj.cmdFullStatePublisher, msg);
        end
        
        function setGroupMask(obj, groupMask)
            %SETGROUPMASK Sets the group mask bits for this robot.
            request = rosmessage(obj.setGroupMaskService);
            request.GroupMask = groupMask;
            
            call(obj.setGroupMaskService, request);
        end

        function takeoff(obj, targetHeight, duration, groupMask)
            %TAKEOFF Execute a takeoff - fly straight up, then hover indefinitely.
            if nargin < 4  || isempty(groupMask)
                groupMask = 0;
            end
            
            request = rosmessage(obj.takeoffService);
            request.Height    = targetHeight;
            request.Duration  = rosduration(duration);
            request.GroupMask = groupMask;
            
            call(obj.takeoffService, request);
        end
        
        function land(obj, targetHeight, duration, groupMask)
            %LAND Execute a landing - fly straight down. User must cut power after.
            if nargin < 4  || isempty(groupMask)
                groupMask = 0;
            end
            
            request = rosmessage(obj.landService);
            request.Height    = targetHeight;
            request.Duration  = rosduration(duration);
            request.GroupMask = groupMask;
            
            call(obj.landService, request);
        end
        
        function goTo(obj, goal, yaw, duration, relative, groupMask)
            %GOTO Move smoothly to the goal, then hover indefinitely.
            if nargin < 5  || isempty(relative)
                relative = false;
            end
            if nargin < 6  || isempty(groupMask)
                groupMask = 0;
            end
            request = rosmessage(obj.goToService);
            request.Goal.X    = goal(1);
            request.Goal.Y    = goal(2);
            request.Goal.Z    = goal(3);
            request.Yaw       = yaw;  % In degrees
            request.Duration  = rosduration(duration);
            request.Relative  = relative;
            request.GroupMask = groupMask;
            
            call(obj.goToService, request);
        end

        function notifySetpointsStop(obj, remainValidMillisecs, groupMask)
            %NOTIFYSETPOINTSSTOP Informs that streaming low-level setpoint packets are about to stop.
            if nargin < 2 || isempty(remainValidMillisecs)
                remainValidMillisecs = 100;
            end
            if nargin < 3  || isempty(groupMask)
                groupMask = 0;
            end
            
            request = rosmessage(obj.notifySetpointsStopService);
            request.RemainValidMillisecs = remainValidMillisecs;
            request.GroupMask            = groupMask;
            
            call(obj.notifySetpointsStopService, request);
        end
    end
end

