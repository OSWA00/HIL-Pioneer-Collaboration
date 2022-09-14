clc
clear
close all
%% MQTT connection
mqttClient = mqttclient("tcp://192.168.4.1", ...
    Port=1883, ...
    ClientID="matlab", ...
    KeepAliveDuration=minutes(5));

% Check mqtt connection
if (mqttClient.Connected == false)
    return;
end

disp("MQTT connected");

%% Robot 1 mqtt topics
% Publish
topic_robot_1_pos_x = "ROBOT_1/POSITION_X";
topic_robot_1_pos_y = "ROBOT_1/POSITION_Y";
topic_robot_1_pos_des_x = "ROBOT_1/POSITION_DES_X";
topic_robot_1_pos_des_y = "ROBOT_1/POSITION_DES_Y";
topic_robot_1_orientation = "ROBOT_1/THETA";

% Subscribe 
topic_robot_1_vel_l = "ROBOT_1/VEL_L";
topic_robot_1_vel_r = "ROBOT_1/VEL_R";
subscribe(mqttClient, topic_robot_1_vel_l);
subscribe(mqttClient, topic_robot_1_vel_r);
disp("Robot 1 topics subscribed");

%% Robot 2 mqtt topics
% Publish 
topic_robot_2_pos_x = "ROBOT_2/POSITION_X";
topic_robot_2_pos_y = "ROBOT_2/POSITION_Y";
topic_robot_2_pos_des_x = "ROBOT_2/POSITION_DES_X";
topic_robot_2_pos_des_y = "ROBOT_2/POSITION_DES_Y";
topic_robot_2_orientation = "ROBOT_2/THETA";
% Subscribe
topic_robot_2_vel_l = "ROBOT_2/VEL_L";
topic_robot_2_vel_r = "ROBOT_2/VEL_R";
subscribe(mqttClient, topic_robot_2_vel_l);
subscribe(mqttClient, topic_robot_2_vel_r);
disp("Robot 2 topics subscribed");

%% Robot 1 control variables
robot_1_vel_l = 0.0;
robot_1_vel_r = 0.0;

%% Robot 2 control variables
robot_2_vel_l = 0.0;
robot_2_vel_r = 0.0;

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
        'Pioneer_p3dx#0', sim.simx_opmode_blocking);
    % Preparar Motor izquierdo
    [~, left_robot_2]=sim.simxGetObjectHandle(clientID,...     
        'Pioneer_p3dx_leftMotor#0',sim.simx_opmode_blocking); 
    % Preparar Motor Derecho
    [~, right_robot_2]=sim.simxGetObjectHandle(clientID,...    
        'Pioneer_p3dx_rightMotor#0',sim.simx_opmode_blocking);
    
    [~, bar]=sim.simxGetObjectHandle(clientID,...
        'Cuboid1',sim.simx_opmode_blocking);

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

    %CIRCULO
%------------------------------------------------------------------------
% [~] = sim.simxSetObjectPosition(clientID,Robot_block_1,-1,[0,0.7,1.3879e-01],sim.simx_opmode_oneshot);
% [~] = sim.simxSetObjectPosition(clientID,Robot_block_2,-1,[0,1.3,1.3879e-01],sim.simx_opmode_oneshot);
% [~] = sim.simxSetObjectPosition(clientID,bar,-1,[0,1,3],sim.simx_opmode_oneshot);
%------------------------------------------------------------------------

%SENO
%------------------------------------------------------------------------
[~] = sim.simxSetObjectPosition(clientID,Robot_block_1,-1,[-0.19,1.78,1.3879e-01],sim.simx_opmode_oneshot);
[~] = sim.simxSetObjectOrientation(clientID,Robot_block_1,-1,[0,0,-pi/4],sim.simx_opmode_oneshot);

[~] = sim.simxSetObjectPosition(clientID,Robot_block_2,-1,[0.22,2.2,1.3879e-01],sim.simx_opmode_oneshot);
[~] = sim.simxSetObjectOrientation(clientID,Robot_block_2,-1,[0,0,-pi/4],sim.simx_opmode_oneshot);

[~] = sim.simxSetObjectPosition(clientID,bar,-1,[0,2,3],sim.simx_opmode_oneshot);
[~] = sim.simxSetObjectOrientation(clientID,bar,-1,[0,0,-pi/4],sim.simx_opmode_oneshot);
%------------------------------------------------------------------------


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


    %CIRCULO
    %------------------------------------------------------------------------
    % t0=0;
    % x0=sin(0.30*(t0));
    % y0=cos(0.30*(t0));
    %----------------------------------------------------------------------
    
    %SENO
    %----------------------------------------------------------------------
    t0=0.001;
    x0=sin(0.2*t0);
    y0=2-t0*0.2;
    %----------------------------------------------------------------------

    disp("Starting simulation")
    %% Simulation
    for i = 1:200
        %% MQTT messages
        % Robot 1 messages
        robot_1_vel_l_msg = read(mqttClient, Topic = topic_robot_1_vel_l);
        if check_message(robot_1_vel_l_msg)
            robot_1_vel_l = str2double(robot_1_vel_l_msg.Data(1));
        end
        robot_1_vel_r_msg = read(mqttClient, Topic = topic_robot_1_vel_r);
        if check_message(robot_1_vel_r_msg)
            robot_1_vel_r= str2double(robot_1_vel_r_msg.Data(1));
        end
    
        % Robot 2 messages
        robot_2_vel_l_msg = read(mqttClient, Topic = topic_robot_2_vel_l);
        if check_message(robot_2_vel_l_msg)
            robot_2_vel_l = str2double(robot_2_vel_l_msg.Data(1));
        end
        robot_2_vel_r_msg = read(mqttClient, Topic = topic_robot_2_vel_r);
        if check_message(robot_2_vel_r_msg)
            robot_2_vel_r = str2double(robot_2_vel_r_msg.Data(1));
        end
        
        if i==1
            robot_1_vel_r= 0.1;
            robot_1_vel_l = 0.1;
        
            robot_2_vel_l = 0.1;
            robot_2_vel_r = 0.1;
        end
        

        disp("MQTT messages received");
        %% Robot 1 velocities
        % Left motor
        [~] = sim.simxSetJointTargetVelocity(clientID, ...
            left_robot_1, robot_1_vel_l, ... 
            sim.simx_opmode_blocking);
        % Right motor
        [~] = sim.simxSetJointTargetVelocity(clientID, ...
            right_robot_1, robot_1_vel_r,... 
            sim.simx_opmode_blocking);
    
        %% Robot 2 velocities
        % Left motor
        [~] = sim.simxSetJointTargetVelocity(clientID, ...
            left_robot_2, robot_2_vel_l,... 
            sim.simx_opmode_blocking);
        % Right motor
        [~] = sim.simxSetJointTargetVelocity(clientID, ...
            right_robot_2, robot_2_vel_r,... 
            sim.simx_opmode_blocking);
        disp("Sent velocities to Coppelia");
        %% Trajectory
        %CIRCULO
        %------------------------------------------------------------------------
        % t=0.1*(i-1); 
        % xd=sin(0.3*(t)); %x va de -2m a 2m de forma sinusoidal
        % yd=cos(0.3*(t)); %y va de 2m a -2m aumentando en 0.1
        %------------------------------------------------------------------------

        %SENO
        %------------------------------------------------------------------------
        t=0.1*(i)*2; 
        xd=sin(0.2*t); %x va de -2m a 2m de forma sinusoidal
        yd=2-t*0.2; %y va de 2m a -2m aumentando en 1.1
        %------------------------------------------------------------------------

        %% Robot 1 pose
        [~, Position_Robot_1]=sim.simxGetObjectPosition(clientID, ...
            Robot_block_1, -1,...
            sim.simx_opmode_buffer);
        % X pose
        msg = string(Position_Robot_1(1));
        write(mqttClient, topic_robot_1_pos_x, msg);
        
        % Y pose
        msg = string(Position_Robot_1(2));
        write(mqttClient, topic_robot_1_pos_y, msg);
        

        % Theta pose
        [~,Orientation_Robot_1] = sim.simxGetObjectOrientation( ...
            clientID, ...
            Robot_block_1, -1,...
            sim.simx_opmode_buffer);
        msg = string(Orientation_Robot_1(3));
        write(mqttClient, topic_robot_1_orientation, msg);
        disp("Sent pose to robot1")

        %% Robot 2 pose
        [~, Position_Robot_2]=sim.simxGetObjectPosition(clientID, ...
            Robot_block_2, -1,...
            sim.simx_opmode_buffer);
        % X pose
        msg = string(Position_Robot_2(1));
        write(mqttClient, topic_robot_2_pos_x, msg);
        disp(Position_Robot_2)

        % Y pose
        msg = string(Position_Robot_2(2));
        write(mqttClient, topic_robot_2_pos_y, msg);

        % Theta pose
        [~,Orientation_Robot_2] = sim.simxGetObjectOrientation( ...
            clientID, ...
            Robot_block_2, -1,...
            sim.simx_opmode_buffer);
        msg = string(Orientation_Robot_2(3));
        write(mqttClient, topic_robot_2_orientation, msg);
        disp("Sent pose to robot2")
        
        %% Collaborative trajectory generation 
        m = ( yd - y0 ) / ( xd - x0 );
        mp = -1 / m;
        d = 0.3;
        ang = atan2( -( xd - x0 ), ( yd - y0 ) );
        dy = d * sin(ang);
        dx = d * cos(ang);
        
        %%%%%%%%%%%%%%%%%%%
        %Esto es lo que hay que mandar
        % Robot 1 desired pose 
        xp1 = xd + dx;
        write(mqttClient, topic_robot_1_pos_des_x, string(xp1));

        yp1 = mp * ( xp1 - xd) + yd;
        write(mqttClient, topic_robot_1_pos_des_y, string(yp1));
        
        % Robot 2 desired pose
        xp2=xd-dx; 
        write(mqttClient, topic_robot_2_pos_des_x, string(xp2));

        yp2=mp*(xp2-xd)+yd;
        write(mqttClient, topic_robot_2_pos_des_y, string(yp2));
       
        
        x0=xd;
        y0=yd;
        
        %% Robot 1
        % Left motor
        [~] = sim.simxSetJointTargetVelocity(clientID, ...
            left_robot_1, robot_1_vel_l, ... 
            sim.simx_opmode_blocking);
        disp(robot_1_vel_l);
        % Right motor
        [~] = sim.simxSetJointTargetVelocity(clientID, ...
            right_robot_1, robot_1_vel_r,... 
            sim.simx_opmode_blocking);
    
        %% Robot 2
        % Left motor
        [~] = sim.simxSetJointTargetVelocity(clientID, ...
            left_robot_2, robot_2_vel_l,... 
            sim.simx_opmode_blocking);
        disp(robot_2_vel_l);
        % Right motor
        [~] = sim.simxSetJointTargetVelocity(clientID, ...
            right_robot_2, robot_2_vel_r,... 
            sim.simx_opmode_blocking);

        figure(1)
        scatter(Position_Robot_1(1),Position_Robot_1(2),'o','g')  %Gráfica de la posición del robot
        hold on
        scatter(Position_Robot_2(1),Position_Robot_2(2),'o','r')  %Gráfica de la posición del robot
        scatter(xd,yd,'.','b')  %Gráfica de la posición del robot
        scatter(xp1,yp1,'.','g')  %Gráfica de la posición del robot
        scatter(xp2,yp2,'.','r')  %Gráfica de la posición del robot
        hold on
        %pause(0.5);
    end
end

sim.delete();
clear mqttClient;