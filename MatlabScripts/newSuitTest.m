clear; close all;
%%

COM_PORTS = serialportlist("available")'
%%
arduinoPort = "COM4";
dynamixelPort = "COM3";

%%
labels = ["Time", "Desired stiffness", "Actual Stiffness","Init angle","Desired position","Actual position","Position offset","Control output","Loadcell reading","Servo position"];

arduinoObj = serialport(arduinoPort,115200);

configureTerminator(arduinoObj,"\n");

flush(arduinoObj);

arduinoObj.experimentData = struct("Data",[],"Count",10);

configureCallback(arduinoObj,"terminator",@readSerialData);

%%
enableDynamixel
%%


while 1
%     if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
%         break;
%     end
    
    goalPosition = arduinoObj.experimentData.Data(end,9);

    % Write goal position
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, typecast(int32(goalPosition), 'uint32'));
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end

    while 1
        % Read present position
        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end

        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', DXL_ID, goalPosition, typecast(uint32(dxl_present_position), 'int32'));

        if ~(abs(goalPosition - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end
    end

%     % Change goal position
%     if index == 1
%         index = 2;
%     else
%         index = 1;
%     end
end



% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;

