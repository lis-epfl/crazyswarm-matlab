classdef Crazyswarm < handle
    %CRAZYSWARM Container for all crazyflies in a swarm. Allows to send
    %broadcasting commands to all crazyflies at once.
    
    properties (GetAccess='public', SetAccess='private')
        % Properties
        crazyflies
                
        % Services
        takeoffService
        landService
        goToService
        updateParamsService
        
        % Service messages
        takeoffServiceMsg
        landServiceMsg
        goToServiceMsg
        updateParamServiceMsg
    end
    
    methods
        function obj = Crazyswarm(crazyflies_yaml)
            %CRAZYSWARM Construct an instance of this class            
            obj.takeoffService = rossvcclient("/takeoff");
            obj.landService = rossvcclient("/land");
            obj.goToService = rossvcclient("/go_to");
            obj.updateParamsService = rossvcclient("/update_params");
            
            obj.takeoffServiceMsg = rosmessage(obj.takeoffService);
            obj.landServiceMsg = rosmessage(obj.landService);
            obj.goToServiceMsg = rosmessage(obj.goToService);
            obj.updateParamServiceMsg = rosmessage(obj.updateParamsService);
 
            cfg = yaml.ReadYaml(convertStringsToChars(crazyflies_yaml));
            tfTree = rostf;

            obj.crazyflies = {};
            
            for crazyflie = cfg.crazyflies
                id = crazyflie{1}.id;
                initialPosition = cell2mat(crazyflie{1}.initialPosition);
                obj.crazyflies{end+1} = Crazyflie(id, initialPosition, tfTree);
            end            
        end
        
       function delete(obj)
            %DELETE Delete the swarm object and all contained crazyflies.
            clear obj.takeoffService
            clear obj.landService
            clear obj.goToService
            clear obj.updateParamsService
            clear obj.takeoffServiceMsg
            clear obj.landServiceMsg
            clear obj.goToServiceMsg
            clear obj.updateParamServiceMsg
            
            for cf = obj.crazyflies
                cf{1}.delete();
                clear cf
            end
            
            clear obj.crazyflies
        end
        
        function state = state(obj)
            %STATE Get state matrix of crazyswarm.
            n_cfs = length(obj.crazyflies);
            state = zeros(6, n_cfs);  % Prealocate state matrix
            for i = 1:n_cfs
                cf = obj.crazyflies{i};
                state(1:3, i) = cf.position;
                state(4:6, i) = cf.velocity;
            end
        end
        
        
        function cmdPosition(obj, posVec, yawVec)
            %CMDPOSITION Sends a streaming command of absolute position and
            %yaw setpoint for each crazyflie in the swarm.
            n_cfs = length(obj.crazyflies);
            for i = 1:n_cfs
                pos = posVec(:, i);
                yaw = yawVec(i);
                
                obj.crazyflies{i}.cmdPosition(pos, yaw);
            end
        end
        
        function cmdVelocityWorld(obj, velVec, yawRateVec)
            %CMDVELOCITYWORLD Sends a streaming velocity-world controller
            %setpoint command for all crazyflies in the swarm.
            n_cfs = length(obj.crazyflies);
            for i = 1:n_cfs
                vel = velVec(:, i);
                yawRate = yawRateVec(i);
                
                obj.crazyflies{i}.cmdVelocityWorld(vel, yawRate);
            end
        end
        
        function cmdFullState(obj, posVec, velVec, accVec, yawVec, omegaVec)
            %CMDFULLSTATE Sends a streaming full-state controller setpoint
            %command for each crazyflie in the swarm.
            n_cfs = length(obj.crazyflies);
            for i = 1:n_cfs
                pos = posVec(:, i);
                vel = velVec(:, i);
                acc = accVec(:, i);
                yaw = yawVec(i);
                omega = omegaVec(:, i);
                
                obj.crazyflies{i}.cmdFullState(pos, vel, acc, yaw, omega);
            end
        end
               
        function takeoff(obj, targetHeight, duration, groupMask)
            %TAKEOFF Broadcasted takeoff - fly straight up, then hover indefinitely.
            if nargin < 4  || isempty(groupMask)
                groupMask = 0;
            end
            
            obj.takeoffServiceMsg.Height    = targetHeight;
            obj.takeoffServiceMsg.Duration  = rosduration(duration);
            obj.takeoffServiceMsg.GroupMask = groupMask;
            call(obj.takeoffService, obj.takeoffServiceMsg);
        end
        
        function land(obj, targetHeight, duration, groupMask)
            %LAND Broadcasted landing - fly straight down. User must cut power after.
            if nargin < 4  || isempty(groupMask)
                groupMask = 0;
            end
            
            obj.landServiceMsg.Height    = targetHeight;
            obj.landServiceMsg.Duration  = rosduration(duration);
            obj.landServiceMsg.GroupMask = groupMask;
            
            call(obj.landService, obj.landServiceMsg);
        end
        
        function goTo(obj, goal, yaw, duration, groupMask)
            %GOTO Broadcasted goTo - Move smoothly to the goal, then hover indefinitely.
            %Always in relative coordinates because absolute goTo would
            %cause collisions.
            if nargin < 5  || isempty(groupMask)
                groupMask = 0;
            end
            obj.goToServiceMsg.Goal.X    = goal(1);
            obj.goToServiceMsg.Goal.Y    = goal(2);
            obj.goToServiceMsg.Goal.Z    = goal(3);
            obj.goToServiceMsg.Yaw       = yaw;  % In degrees
            obj.goToServiceMsg.Duration  = rosduration(duration);
            obj.goToServiceMsg.Relative  = true;
            obj.goToServiceMsg.GroupMask = groupMask;
            
            call(obj.goToService, obj.goToServiceMsg);
        end
        
        function setParam(obj, name, value)
            %SETPARAM Broadcasted setParam. See Crazyflie.setParam() for details
            rosparam('set', "/allcfs/" + name, value);
            obj.updateParamServiceMsg.Params = {name};
            call(obj.updateParamsService, obj.updateParamServiceMsg);
        end

    end
end

