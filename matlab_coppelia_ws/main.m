clc
clear
close all

%% MQTT connection
% MQTT parameters
client_id = "matlabClient";
broker_address = "tcp://192.168.4.1";
port = 1883;

% Connect to mqtt
mq_client = mqttclient(broker_address, Port=port, ClientID=client_id);

% Check mqtt connection
if (mq_client.Connected == false)
    return;
end

%% Robot 1 mqtt topics
% Publish
topic_robot_1_pos_x = "ROBOT_1/POSITION_X";
topic_robot_1_pos_y = "ROBOT_1/POSITION_Y";
topic_robot_1_pos_des_x = "ROBOT_1/POSITION_DES_X";
topic_robot_1_pos_des_y = "ROBOT_1/POSITION_DES_Y";
% Subscribe 
topic_robot_1_vel = "ROBOT_1/VELOCITY";
topic_robot_1_omega = "ROBOT_1/OMEGA";
subscribe(mq_client, topic_robot_1_vel);
subscribe(mq_client, topic_robot_1_omega);

%% Robot 2 mqtt topics
% Publish 
topic_robot_2_pos_x = "ROBOT_2/POSITION_X";
topic_robot_2_pos_y = "ROBOT_2/POSITION_Y";
topic_robot_2_pos_des_x = "ROBOT_2/POSITION_DES_X";
topic_robot_2_pos_des_y = "ROBOT_2/POSITION_DES_Y";
% Subscribe
topic_robot_2_vel = "ROBOT_2/VELOCITY";
topic_robot_2_omega = "ROBOT_2/OMEGA";
subscribe(mq_client, topic_robot_2_vel);
subscribe(mq_client, topic_robot_2_omega);

%% Robot 1 control variables
robot_1_velocity = 0.0;
robot_1_omega = 0.0;

%% Robot 2 control variables
robot_2_velocity = 0.0;
robot_2_omega = 0.0;

%% Coppelia connection
% Setup remote connection
sim=remApi('remoteApi');
sim.simxFinish(-1);
clientID=sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

if (clientID>-1)
    disp('Coppelia connected');

    % TODO configure pioneer parameters
    % Configuración de los Handles

    %% Robot 1
    [~, Robot_block_1] = sim.simxGetObjectHandle(clientID,...
        'Pioneer_p3dx', sim.simx_opmode_blocking);
    % Preparar Motor izquierdo
    [~, left_robot_1]=sim.simxGetObjectHandle(clientID,...      
        'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking); 
    % Preparar Motor Derecho
    [~, right_robot_1]=sim.simxGetObjectHandle(clientID,...   
        'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);
    
    %% Robot 2
    [~, Robot_block_2] = sim.simxGetObjectHandle(clientID,...
        'Pioneer_p3dx', sim.simx_opmode_blocking);
    % Preparar Motor izquierdo
    [~, left_robot_2]=sim.simxGetObjectHandle(clientID,...     
        'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking); 
    % Preparar Motor Derecho
    [~, right_robot_2]=sim.simxGetObjectHandle(clientID,...    
        'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);

    % Inicialización para obtener la posición Robot 1
    [~, Position_Robot_1] = sim.simxGetObjectPosition(clientID,Robot_block_1,-1,...
        sim.simx_opmode_streaming);
    % Inicialización para obtener la posición Robot 2
    [~, Position_Robot_2] = sim.simxGetObjectPosition(clientID,Robot_block_2,-1,...
        sim.simx_opmode_streaming);

    % Inicialización para obtener la orientación Robot 1
    [~, Orientation_Robot_1] = sim.simxGetObjectOrientation(clientID,Robot_block_1,-1,...
        sim.simx_opmode_streaming);
    % Inicialización para obtener la orientación Robot 2
    [~, Orientation_Robot_2] = sim.simxGetObjectOrientation(clientID,Robot_block_2,-1,...
        sim.simx_opmode_streaming);

% Example publish to topic
% msg = "70.0";
% write(mq_client, topic_robot_1_pos_x, msg);
% 
% msg = "90.0";
% write(mq_client, topic_robot_1_pos_y, msg);

    % Inicializacion de los motores en cada robot
    % Robot 1
    % Motor izquierdo
    [~] = sim.simxSetJointTargetVelocity(clientID,left_robot_1,0,... 
        sim.simx_opmode_blocking);
    % Motor derecho
    [~] = sim.simxSetJointTargetVelocity(clientID,right_robot_1,0,... 
        sim.simx_opmode_blocking);

    % Robot 2
    % Motor izquierdo
    [~] = sim.simxSetJointTargetVelocity(clientID,left_robot_2,0,... 
        sim.simx_opmode_blocking);
    % Motor derecho
    [~] = sim.simxSetJointTargetVelocity(clientID,right_robot_2,0,... 
        sim.simx_opmode_blocking);

    %% Simulation
    while true
        % Robot 1 messages
        robot_1_vel_msg = read(mq_client, Topic = topic_robot_1_vel);
        if check_message(robot_1_vel_msg)
            robot_1_velocity = str2double(robot_1_vel_msg.Data(1));
        end
        robot_1_omega_msg = read(mq_client, Topic = topic_robot_1_omega);
        if check_message(robot_1_omega_msg)
            robot_1_omega = str2double(robot_1_omega_msg.Data(1));
        end
    
        % Robot 2 messages
        robot_2_vel_msg = read(mq_client, Topic = topic_robot_2_vel);
        if check_message(robot_2_vel_msg)
            robot_2_velocity = str2double(robot_2_vel_msg.Data(1));
        end
        robot_2_omega_msg = read(mq_client, Topic = topic_robot_2_omega);
        if check_message(robot_2_omega_msg)
            robot_2_omega = str2double(robot_2_omega_msg.Data(1));
        end
    
        % TODO send the velocity to coppelia (Vl, Vr)
        %%%%%% Cambiar los nombres de la velocidad de cada motor
            %% Robot 1
        % Motor izquierdo
        [~] = sim.simxSetJointTargetVelocity(clientID,left_robot_1,0,... 
            sim.simx_opmode_blocking);
        % Motor derecho
        [~] = sim.simxSetJointTargetVelocity(clientID,right_robot_1,0,... 
            sim.simx_opmode_blocking);
    
            %% Robot 2
        % Motor izquierdo
        [~] = sim.simxSetJointTargetVelocity(clientID,left_robot_2,robot1_vr,... 
            sim.simx_opmode_blocking);
        % Motor derecho
        [~] = sim.simxSetJointTargetVelocity(clientID,right_robot_2,0,... 
            sim.simx_opmode_blocking);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Obtner la posición del robot 1
        [~, Position_Robot_1]=sim.simxGetObjectPosition(clientID,Robot_block_1,-1,...
            sim.simx_opmode_buffer);
        msg = string(Position_Robot_1(1));
        write(mq_client, topic_robot_1_pos_x, msg);  % Mandar la posición en X del robot 1
        msg = string(Position_Robot_1(2));
        write(mq_client, topic_robot_1_pos_y, msg);  % Mandar la posición en Y del robot 1

        % Obtner la posición del robot 2
        [~, Position_Robot_2]=sim.simxGetObjectPosition(clientID,Robot_block_2,-1,...
            sim.simx_opmode_buffer);
        msg = string(Position_Robot_2(1));
        write(mq_client, topic_robot_2_pos_x, msg);  % Mandar la posición en X del robot 2
        msg = string(Position_Robot_2(2));
        write(mq_client, topic_robot_2_pos_y, msg);  % Mandar la posición en Y del robot 2

        % Obtener la orientación del robot 1
        [~,Orientation_Robot_1] = sim.simxGetObjectOrientation(clientID,Robot_block_1,-1,...
            sim.simx_opmode_buffer);
        msg = string(Orientation_Robot_1(3));
        write(mq_client, topic_robot_1_orientation, msg); % Mandar la oreintación del robot 1

        % Obtener la orientación del robot 2
        [~,Orientation_Robot_2] = sim.simxGetObjectOrientation(clientID,Robot_block_2,-1,...
            sim.simx_opmode_buffer);
        msg = string(Orientation_Robot_2(3));
        write(mq_client, topic_robot_2_orientation, msg); % Mandar la orientación del robot 2
    
    %     disp("Robot 1");
    %     disp("Vel: " + robot_1_velocity);
    %     disp("Omega: " + robot_1_omega);
    % 
    %     disp("Robot 2");
    %     disp("Vel: " + robot_2_velocity);
    %     disp("Omega: " + robot_2_omega);


        % Obtner la posición de cada uno de los robots
        

    
        pause(1); % Remove on real program
    
        % TODO trajectory generation (POSITION_DESIRED)
        % TODO send current position from COPPELIA (POS)
    
    end
end

sim.delete();
clear mq_client;