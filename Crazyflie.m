classdef Crazyflie < handle
    %CRAZYFLIE Class representing a single crazyflie.
    
    properties (GetAccess = public, SetAccess = private)
        id
        prefix
        initialPosition
    end
    
    properties (GetAccess = private, SetAccess = private)
        tfTree

        % Publishers
        cmdPositionPublisher
        cmdVelocityWorldPublisher
        cmdFullStatePublisher
        
        % Subscribers
        posSubscriber
        velSubscriber
         
        % Services
        setGroupMaskService
        takeoffService
        landService
        goToService
        notifySetpointsStopService
        updateParamsService
        
        % Messages
        cmdPositionMsg
        cmdVelocityWorldMsg
        cmdFullStateMsg
        
        % Service messages
        setGroupMaskServiceMsg
        takeoffServiceMsg
        landServiceMsg
        goToServiceMsg
        notifySetpointsStopServiceMsg
        updateParamsServiceMsg
        
        time
    end
     
    methods
        function obj = Crazyflie(id, initialPosition, tfTtree)
            %CRAZYFLIE Constructor for a Crazyflie
            
            % Hack because rostime() is too slow
            % Use ros.internal.Time for versions >= R2019b
            % Use robotics.ros.internal.Time([]) for versions < R2019b
            try
                obj.time = ros.internal.Time;
            catch ex
                if contains(ex.message, "Unable to resolve the name")
                    obj.time = robotics.ros.internal.Time([]);
                end
            end
            
            obj.id = id;
            prefix = "cf" + num2str(id);
            obj.prefix = prefix;
            obj.initialPosition = initialPosition;
            obj.tfTree = tfTtree;
            
            obj.cmdPositionPublisher = rospublisher(prefix + "/cmd_position", ...
                                                    "crazyflie_driver/Position");
            obj.cmdPositionMsg = rosmessage("crazyflie_driver/Position");
            obj.cmdPositionMsg.Header.FrameId     = "/world";
                   
            obj.cmdVelocityWorldPublisher = rospublisher(prefix + "/cmd_velocity_world", ...
                                                         "crazyflie_driver/VelocityWorld");
            obj.cmdVelocityWorldMsg = rosmessage("crazyflie_driver/VelocityWorld");
            obj.cmdVelocityWorldMsg.Header.FrameId     = "/world";

            obj.cmdFullStatePublisher = rospublisher(prefix + "/cmd_full_state", ...
                                                     "crazyflie_driver/FullState");
            obj.cmdFullStateMsg = rosmessage("crazyflie_driver/FullState");
            obj.cmdFullStateMsg.Header.FrameId     = "/world";
            
            % NOTE: Workaround code. See Crazyflie.position() and 
            % Crazyflie.velocity() for more details.
            obj.posSubscriber = rossubscriber(prefix + "/pos");
            obj.velSubscriber = rossubscriber(prefix + "/vel");
            
            obj.setGroupMaskService = rossvcclient(prefix + "/set_group_mask");
            obj.takeoffService = rossvcclient(prefix + "/takeoff");
            obj.landService = rossvcclient(prefix + "/land");
            obj.goToService = rossvcclient(prefix + "/go_to");
            obj.notifySetpointsStopService = rossvcclient(prefix + "/notify_setpoints_stop");
            obj.updateParamsService = rossvcclient(prefix + "/update_params");
            
            obj.setGroupMaskServiceMsg = rosmessage(obj.setGroupMaskService);
            obj.takeoffServiceMsg = rosmessage(obj.takeoffService);
            obj.landServiceMsg = rosmessage(obj.landService);
            obj.goToServiceMsg = rosmessage(obj.goToService);
            obj.notifySetpointsStopServiceMsg = rosmessage(obj.notifySetpointsStopService);
            obj.updateParamsServiceMsg = rosmessage(obj.updateParamsService);
        end
        
        function delete(obj)
            %DELETE Delete the Crazyflie
            clear obj.cmdPositionPublisher
            clear obj.cmdVelocityWorldPublisher
            clear obj.cmdFullStatePublisher
            clear obj.posSubscriber
            clear obj.velSubscriber
            clear obj.setGroupMaskService
            clear obj.takeoffService
            clear obj.landService
            clear obj.goToService
            clear obj.notifySetpointsStopService
            clear obj.cmdPositionMsg
            clear obj.cmdVelocityWorldMsg
            clear obj.cmdFullStateMsg
            clear obj.setGroupMaskServiceMsg
            clear obj.takeoffServiceMsg
            clear obj.landServiceMsg
            clear obj.goToServiceMsg
            clear obj.notifySetpointsStopServiceMsg
            clear obj.updateParamsServiceMsg
            clear obj.time
        end
        
        function pos = position(obj)
            %POSITION Get position of crazyflie.
            %
            % NOTE: The commented code below is the equivalent of the code
            % in pycrazyswarm. Unfortunately, getTransform is currently too 
            % slow to be usable in real time with many drones. For this
            % reason we use an additional Python node to publish position
            % and velocity of each crazyflie on topics called 'cf#/pos'
            % and 'cf#/vel'.
            
%             % Latest known transformation
%             tf = getTransform(obj.tfTree, "/world", obj.prefix, "Timeout", 10);
%             % t = tf.Header.Stamp.Sec + tf.Header.Stamp.Nsec / 1e9;
%             transl = tf.Transform.Translation;
%             pos = [transl.X; transl.Y; transl.Z];
            
            % Workaround (requires python node that publishes position)
            msg = obj.posSubscriber.LatestMessage;
            pos = [msg.X, msg.Y, msg.Z];
        end

        function vel = velocity(obj)
            %VELOCITY Get velocity of crazyflie.
            %
            % NOTE: The commented code below is the equivalent of the code
            % in pycrazyswarm. Unfortunately, getTransform is currently too 
            % slow to be usable in real time with many drones. For this
            % reason we use an additional Python node to publish position
            % and velocity of each crazyflie on topics called 'cf#/pos'
            % and 'cf#/vel'.
            
%             dt = 0.1;
%             timePrev = obj.time.CurrentTime - dt;
%             tfNow = getTransform(obj.tfTree, "/world", obj.prefix, "Timeout", 10);
%             tfPrev = getTransform(obj.tfTree, "/world", obj.prefix, timePrev);
%             
%             translNow = tfNow.Transform.Translation;
%             translPrev = tfPrev.Transform.Translation;
%             
%             posNow = [translNow.X; translNow.Y; translNow.Z];
%             posPrev = [translPrev.X; translPrev.Y; translPrev.Z];
%             
%             vel = (posNow - posPrev) / dt;
            
            % Workaround (requires python node that publishes velocity)
            msg = obj.velSubscriber.LatestMessage;
            vel = [msg.X, msg.Y, msg.Z];
        end

        function cmdPosition(obj, pos, yaw)
            %CMDPOSITION Sends a streaming command of absolute position and yaw setpoint.
            obj.cmdPositionMsg.Header.Stamp   = obj.time.CurrentTime;
            obj.cmdPositionMsg.X              = pos(1);
            obj.cmdPositionMsg.Y              = pos(2);
            obj.cmdPositionMsg.Z              = pos(3);
            obj.cmdPositionMsg.Yaw            = yaw;
            
            send(obj.cmdPositionPublisher, obj.cmdPositionMsg);
        end
        
        function cmdVelocityWorld(obj, vel, yawRate)
            %CMDVELOCITYWORLD Sends a streaming velocity-world controller setpoint command.
            obj.cmdVelocityWorldMsg.Header.Stamp   = obj.time.CurrentTime;
            obj.cmdVelocityWorldMsg.Vel            = vel(1);
            obj.cmdVelocityWorldMsg.Vel            = vel(F2);
            obj.cmdVelocityWorldMsg.Vel            = vel(3);
            obj.cmdVelocityWorldMsg.YawRate        = yawRate;
            
            send(obj.cmdVelocityWorldPublisher, obj.cmdVelocityWorldMsg);
        end
        
        function cmdFullState(obj, pos, vel, acc, yaw, omega)
            %CMDFULLSTATE Sends a streaming full-state controller setpoint command.
            quat = eul2quat([0, 0, yaw], 'xyz');
            obj.cmdFullStateMsg.Header.Stamp       = obj.time.CurrentTime;
            obj.cmdFullStateMsg.Pose.Position.X    = pos(1);
            obj.cmdFullStateMsg.Pose.Position.Y    = pos(2);
            obj.cmdFullStateMsg.Pose.Position.Z    = pos(3);
            obj.cmdFullStateMsg.Twist.Linear.X     = vel(1);
            obj.cmdFullStateMsg.Twist.Linear.Y     = vel(2);
            obj.cmdFullStateMsg.Twist.Linear.Z     = vel(3);
            obj.cmdFullStateMsg.Acc.X              = acc(1);
            obj.cmdFullStateMsg.Acc.Y              = acc(2);
            obj.cmdFullStateMsg.Acc.Z              = acc(3);
            obj.cmdFullStateMsg.Pose.Orientation.W = quat(1);
            obj.cmdFullStateMsg.Pose.Orientation.X = quat(2);
            obj.cmdFullStateMsg.Pose.Orientation.Y = quat(3);
            obj.cmdFullStateMsg.Pose.Orientation.Z = quat(4);
            obj.cmdFullStateMsg.Twist.Angular.X    = omega(1);
            obj.cmdFullStateMsg.Twist.Angular.Y    = omega(2);
            obj.cmdFullStateMsg.Twist.Angular.Z    = omega(3);
            
            send(obj.cmdFullStatePublisher, obj.cmdFullStateMsg);
        end
        
        function setGroupMask(obj, groupMask)
            %SETGROUPMASK Sets the group mask bits for this robot.
            obj.setGroupMaskServiceMsg.GroupMask = groupMask;
            
            call(obj.setGroupMaskService, obj.setGroupMaskServiceMsg);
        end

        function takeoff(obj, targetHeight, duration, groupMask)
            %TAKEOFF Execute a takeoff - fly straight up, then hover indefinitely.
            if nargin < 4  || isempty(groupMask)
                groupMask = 0;
            end
            
            obj.takeoffServiceMsg.Height    = targetHeight;
            obj.takeoffServiceMsg.Duration  = rosduration(duration);
            obj.takeoffServiceMsg.GroupMask = groupMask;
            
            call(obj.takeoffService, obj.takeoffServiceMsg);
        end
        
        function land(obj, targetHeight, duration, groupMask)
            %LAND Execute a landing - fly straight down. User must cut power after.
            if nargin < 4  || isempty(groupMask)
                groupMask = 0;
            end
            
            obj.landServiceMsg.Height    = targetHeight;
            obj.landServiceMsg.Duration  = rosduration(duration);
            obj.landServiceMsg.GroupMask = groupMask;
            
            call(obj.landService, obj.landServiceMsg);
        end
        
        function goTo(obj, goal, yaw, duration, relative, groupMask)
            %GOTO Move smoothly to the goal, then hover indefinitely.
            if nargin < 5  || isempty(relative)
                relative = false;
            end
            if nargin < 6  || isempty(groupMask)
                groupMask = 0;
            end
            
            obj.goToServiceMsg.Goal.X    = goal(1);
            obj.goToServiceMsg.Goal.Y    = goal(2);
            obj.goToServiceMsg.Goal.Z    = goal(3);
            obj.goToServiceMsg.Yaw       = yaw;  % In degrees
            obj.goToServiceMsg.Duration  = rosduration(duration);
            obj.goToServiceMsg.Relative  = relative;
            obj.goToServiceMsg.GroupMask = groupMask;
            
            call(obj.goToService, obj.goToServiceMsg);
        end

        function notifySetpointsStop(obj, remainValidMillisecs, groupMask)
            %NOTIFYSETPOINTSSTOP Informs that streaming low-level setpoint packets are about to stop.
            if nargin < 2 || isempty(remainValidMillisecs)
                remainValidMillisecs = 100;
            end
            if nargin < 3  || isempty(groupMask)
                groupMask = 0;
            end
            
            obj.notifySetpointsStopServiceMsg.RemainValidMillisecs = remainValidMillisecs;
            obj.notifySetpointsStopServiceMsg.GroupMask            = groupMask;
            
            call(obj.notifySetpointsStopService, obj.notifySetpointsStopServiceMsg);
        end
        
        function setParam(obj, name, value)
            %SETPARAM Changes the value of the given parameter.
            rosparam('set', obj.prefix + "/" + name, value);
            obj.updateParamsServiceMsg.Params = {convertStringsToChars(name)};
            call(obj.updateParamsService, obj.updateParamsServiceMsg);
        end
        
        function setParams(obj, params)
            %SETPARAMS Changes the value of several parameters at once.
            % params: struct
            rosparam('set', obj.prefix, params);  % Set entire struct at once
            obj.updateParamsServiceMsg.Params = fieldnames(params);
            call(obj.updateParamsService, obj.updateParamsServiceMsg);
        end  
        
        function setLEDColor(obj, r, g, b)
            %SETLEDCOLOR Sets the color of the LED ring deck.

            % PRECONDITION: The param "ring/effect" must be set to 7 (solid color)
            % for this command to have any effect. The default mode uses the ring
            % color to indicate radio connection quality.

            % This is a blocking command, so it may cause stability problems for
            % large swarms and/or high-frequency changes.
            
            obj.setParam("ring/solidRed", floor(r * 255))
            obj.setParam("ring/solidGreen", floor(g * 255))
            obj.setParam("ring/solidBlue", floor(b * 255))
        end
    end
end

