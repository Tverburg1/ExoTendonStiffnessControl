close all; clear;

%% Load data
addpath("Raw Data\")

fileName = "TVerburgE7_6Processed.csv";
arduinoFileName = "FinalE7_6.txt";

data1 = readmatrix(fileName);

inputData = textread(arduinoFileName,'%s','delimiter','\n');
arduinoData = zeros(length(inputData), 10);

for i = 1:length(inputData)
    line = strsplit(string(inputData(i)),{': ', ', '},'CollapseDelimiters',true);
    if(i == 1)
        labels = line(1:2:end);
    end
    arduinoData(i, :) = line(2:2:end);
end

filteredData = fillmissing(data1,'previous', 1);

%% Unpack data

t = filteredData(:,1)/200;

EL = filteredData(:, 12:14);
angleL = asin((abs(EL(:,1) - EL(1,1)))/0.088);

motorPosGradient = abs(gradient(arduinoData(:, 6)));
markerAngleGradient = abs(gradient(angleL));
arduinoStartIdx = 1;
markerStartIdx = 1;

for n = 1:length(motorPosGradient)
    if (and(motorPosGradient(n) > 1, arduinoStartIdx == 1))
        arduinoStartIdx = n;
    end
end

for n = 1:length(markerAngleGradient)
    if (and(markerAngleGradient(n) > (1.5*10^(-3)), markerStartIdx == 1))
        markerStartIdx = n;

%         markerStartIdx = markerStartIdx + 75; %% OFFSET TO FIX BAD AUTOMATIC ALIGNMENT <---------------
    end
end

t_a = ((arduinoData(:,1)-arduinoData(1,1))/1000);


if ((t_a(arduinoStartIdx) - t(markerStartIdx)) >= 0)
    tStartIdx = find(t_a >= (t_a(arduinoStartIdx) - t(markerStartIdx)));
    tStartIdx = [tStartIdx; length(t)];
    tEndIdx = find(t >= (t_a(end) - t_a(tStartIdx(1))));

    arduinoDataClipped = arduinoData(tStartIdx(1):end,:);
    filteredData = filteredData(1:(tEndIdx(1)),:);  
else
    tStartIdx = find(t >= (t(markerStartIdx) - t_a(arduinoStartIdx)));
    tStartIdx = [tStartIdx; 1];
    tEndIdx = find(t >= (t_a(end) + t(tStartIdx(1))));

    arduinoDataClipped = arduinoData(:,:);
    filteredData = filteredData(tStartIdx(1):(tEndIdx(1)),:);  
end



t = (filteredData(:,1)-filteredData(1,1))/200;
t_a = ((arduinoDataClipped(:,1)-arduinoDataClipped(1,1))/1000);
ssArduinoData = interp1(t_a, arduinoDataClipped, (filteredData(:,1) - filteredData(1,1))/200);
ssArduinoData(1,:) = ssArduinoData(2,:);
t_a = ((ssArduinoData(:,1)-ssArduinoData(1,1))/1000);


springDelta = (filteredData(:,30:32) - filteredData(:,33:end))-(filteredData(1,30:32) - filteredData(1,33:end));
springExtension = vecnorm(springDelta, 1,2);

EL = filteredData(:, 12:14);
ER = filteredData(:, 15:17);

angleL = asin((EL(:,1) - EL(1,1))/0.088);
angleR = asin((abs(ER(:,1) - ER(1,1)))/0.088);



SRdz = filteredData(:, 35)-(filteredData(1, 35));
SLdz = filteredData(:, 32)-(filteredData(1, 32));

targetMotorPosition = ssArduinoData(:, 5);
actualMotorPosition = ssArduinoData(:, 6);

targetAngle = 2*pi.*targetMotorPosition/(4096*(0.068/0.0313));
actualAngle = 2*pi.*actualMotorPosition/(4096*(0.068/0.0313));

Ft = 2*tan(abs(actualAngle-actualAngle(1))).*144.*springExtension;

%% Calculation of actuation range

maxSRdz = SRdz(islocalmax(SRdz,'MinSeparation',3000));
minSRdz = SRdz(islocalmin(SRdz,'MinSeparation',3000));

dx = maxSRdz - minSRdz(1:length(maxSRdz));
    
minIdx = find(islocalmin(SRdz,'MinSeparation',3000) == 1);
base_value = dx(1).*ones(length(SRdz),1);

for i = 1:length(dx)
    base_value((minIdx(i)+1):end) = dx(i);
end

figure
hold on
plot(t, Ft./ base_value)
plot(t_a, ssArduinoData(:, 2))
ylim([0,6000])
%%
figure
plot(t, springExtension)


%%

t_Arduino = (arduinoData(:,1)- arduinoData(1,1))/1000;
time = 483.87;

cutIdxMarker = find(t  == time)
cutIdxArduino = find(t_Arduino <= (time - 5));
cutIdxArduino(end)
arduinoData(cutIdxArduino(end),1)

