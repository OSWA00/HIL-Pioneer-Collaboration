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

%% Simulation

% Example publish to topic
% msg = "70.0";
% write(mq_client, topic_robot_1_pos_x, msg);
% 
% msg = "90.0";
% write(mq_client, topic_robot_1_pos_y, msg);

while true
    %% Robot 1 messages
    robot_1_vel_msg = read(mq_client, Topic = topic_robot_1_vel);
    if check_message(robot_1_vel_msg)
        robot_1_velocity = str2double(robot_1_vel_msg.Data(1));
    end
    robot_1_omega_msg = read(mq_client, Topic = topic_robot_1_omega);
    if check_message(robot_1_omega_msg)
        robot_1_omega = str2double(robot_1_omega_msg.Data(1));
    end

    %% Robot 2 messages
    robot_2_vel_msg = read(mq_client, Topic = topic_robot_2_vel);
    if check_message(robot_2_vel_msg)
        robot_2_velocity = str2double(robot_2_vel_msg.Data(1));
    end
    robot_2_omega_msg = read(mq_client, Topic = topic_robot_2_omega);
    if check_message(robot_2_omega_msg)
        robot_2_omega = str2double(robot_2_omega_msg.Data(1));
    end

    % TODO send to coppelia

%     disp("Robot 1");
%     disp("Vel: " + robot_1_velocity);
%     disp("Omega: " + robot_1_omega);
% 
%     disp("Robot 2");
%     disp("Vel: " + robot_2_velocity);
%     disp("Omega: " + robot_2_omega);

    pause(1); % Remove on real program

    % TODO trajectory generation (POSITION_DESIRED)
    % TODO send current position from COPPELIA (POS)

end


clear mq_client;