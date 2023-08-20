close all; clear;

%% Load data
addpath("Raw Data\")

meanDx = [];
stdDx = [];
stiffnesses = ones(40,6)*nan;
stiffnesses(:,1) =  linspace(50, 50+39*50, 40);

stiffnessesTension = ones(40,6)*nan;
stiffnessesTension(:,1) =  linspace(50, 50+39*50, 40);

initialMarkerPositions = zeros(5, 35);

for cn = 2:15
    arduinoFileName = sprintf("TVerburgE21_%d.csv", cn);

    data1 = readmatrix(sprintf("TVerburgE21_%dProcessed.csv", cn));
    
    arduinoData = readmatrix(arduinoFileName);
    arduinoData(1, :) = [];

    filteredData = fillmissing(data1,'previous', 1);

    t = filteredData(:,1)/200;

    EL = filteredData(:, 12:14);
    angleL = asin((abs(EL(:,1) - EL(1,1)))/0.088);
    
    motorPosGradient = abs(gradient(arduinoData(:, 6)));
    markerAngleGradient = abs(gradient(angleL));
    arduinoStartIdx = 1;
    markerStartIdx = 1;
    
    for n = 1:length(motorPosGradient)
        if (and(motorPosGradient(n) > 10, arduinoStartIdx == 1))
            arduinoStartIdx = n;
        end
    end
    
    for n = 1:length(markerAngleGradient)
        if (and(markerAngleGradient(n) > (1.5*10^(-3)), markerStartIdx == 1))
            markerStartIdx = n;
    
%             markerStartIdx = markerStartIdx + 75; %% OFFSET TO FIX BAD AUTOMATIC ALIGNMENT <---------------
        end
    end
    
    t_a = ((arduinoData(:,1)-arduinoData(1,1))/1000);
    
    if ((t_a(arduinoStartIdx) - t(markerStartIdx)) >= 0)
        tStartIdx = find(t_a >= (t_a(arduinoStartIdx) - t(markerStartIdx)));
        tStartIdx = [tStartIdx; 1];
        tEndIdx = find(t >= (t_a(end) - t(tStartIdx(1))));

        if (isempty(tEndIdx))
            tEndIdx = find((t_a - t(tStartIdx(1))) >= t(end));
            
            arduinoDataClipped = arduinoData(tStartIdx(1):end,:);
            filteredData = filteredData(:,:);
        else
            arduinoDataClipped = arduinoData(tStartIdx(1):end,:);
            filteredData = filteredData(1:(tEndIdx(1)),:);
        end   
    else
        tStartIdx = find(t >= (t(markerStartIdx) - t_a(arduinoStartIdx)));
        tStartIdx = [tStartIdx; 1];
        tEndIdx = find(t >= (t_a(end) + t(tStartIdx(1))));

        if (isempty(tEndIdx))
            tEndIdx = find((t_a - t(tStartIdx(1))) >= t(end));
            
            arduinoDataClipped = arduinoData(tStartIdx(1):tEndIdx(1),:);
            filteredData = filteredData(:,:);
        else
            arduinoDataClipped = arduinoData(tStartIdx(1):end,:);
            filteredData = filteredData(1:(tEndIdx(1)),:);
        end  
    end
    
    
    t = (filteredData(:,1)-filteredData(1,1))/200;
    t_a = ((arduinoDataClipped(:,1)-arduinoDataClipped(1,1))/1000);
    ssArduinoData = interp1(t_a, arduinoDataClipped, (filteredData(:,1) - filteredData(1,1))/200);
    ssArduinoData(1,:) = ssArduinoData(2,:);
    t_a = ((ssArduinoData(:,1)-ssArduinoData(1,1))/1000);
    
    initialMarkerPositions(cn, :) = filteredData(1,:);
    
    springDelta = (filteredData(:,30:32) - filteredData(:,33:end))-(filteredData(1,30:32) - filteredData(1,33:end));
    springExtension = vecnorm(springDelta, 1,2);
    
    EL = filteredData(:, 12:14);
    ER = filteredData(:, 15:17);
    
    angleL = asin((EL(:,1) - EL(1,1))/0.088);
    angleR = asin((abs(ER(:,1) - ER(1,1)))/0.088);
    
%     Ft = 2*tan(angleR).*144.*springExtension;
    
    SRdz = filteredData(:, 35)-(filteredData(1, 35));
%     SRdz = filteredData(:, 35)-min((filteredData(:, 35)));
    SLdz = filteredData(:, 32)-(filteredData(1, 32));
    
    targetMotorPosition = ssArduinoData(:, 5);
    actualMotorPosition = ssArduinoData(:, 6);
    
    targetAngle = 2*pi.*targetMotorPosition/(4096*(0.068/0.0313));
    actualAngle = 2*pi.*actualMotorPosition/(4096*(0.068/0.0313));
    
    Ft = 2*tan(abs(actualAngle-actualAngle(1))).*144.*springExtension;

    
    maxSRdz = SRdz(islocalmax(SRdz,'MinSeparation',1000, 'MinProminence',0.02));
    maxSRdz = maxSRdz(maxSRdz > 0.005);
    

    minIdx = find(islocalmin(SRdz,'MinSeparation',1200, 'MinProminence',0.02) == 1);
    maxIdx = find(islocalmax(SRdz,'MinSeparation',1000, 'MinProminence',0.02) == 1);
    minIdx = [1; minIdx; length(SRdz)];
    
    minSRdz = SRdz(minIdx);

    dx = maxSRdz - minSRdz(1:length(maxSRdz));
    base_value = dx(1).*ones(length(SRdz),1);
    
    for i = 1:length(dx)
        base_value((minIdx(i)+1):end) = dx(i);
    end
    
    stiffnessMotorAngleFt = Ft./base_value;
    maxStiffnessIdx = islocalmax(stiffnessMotorAngleFt,'MinSeparation',1000, 'MinProminence',30);
    maxStiffnessForce = stiffnessMotorAngleFt(maxStiffnessIdx);
    maxStiffnessForce = maxStiffnessForce(maxStiffnessForce > 1);

    stiffnesses(1:length(maxStiffnessForce), cn) = maxStiffnessForce;

    stiffnessTension = (ssArduinoData(:,9)-ssArduinoData(1,9))./ base_value;
    maxTensionStiffIdx = islocalmax(stiffnessTension,'MinSeparation',1000, 'MinProminence',30);
    maxTensionStiffnessForce = stiffnessTension(maxTensionStiffIdx);
    maxTensionStiffnessForce = maxTensionStiffnessForce(maxTensionStiffnessForce > 1);

    stiffnessesTension(1:length(maxTensionStiffnessForce), cn) = maxTensionStiffnessForce;

    meanDx = [meanDx mean(base_value)];
    stdDx = [stdDx std(base_value(1:(end-1000)))];

    figure(69+cn)
    
    tiles = tiledlayout(4,1, 'Padding', 'none', 'TileSpacing', 'compact'); 
    title(tiles, cn)

    ax1 = nexttile;
    hold on
%     markerBasedStiffness = Ft./ (SRdz);
    markerBasedStiffness = Ft./ (mean(base_value));

%     outlierIdx = find(or(markerBasedStiffness > (1.6*ssArduinoData(:, 2)), markerBasedStiffness < (0.4*ssArduinoData(:, 2))));
%     markerBasedStiffness(outlierIdx) = nan;
%     markerBasedStiffness = fillmissing(markerBasedStiffness,'previous', 1);

    plot(t, markerBasedStiffness)
    plot(t_a, ssArduinoData(:, 2))
    ylim([0,2100])

    ax2 = nexttile;
    hold on
    plot(t, Ft./base_value)
    plot(t_a, ssArduinoData(:, 2))
    ylim([0,2100])

    
% %     plot(t_a, (ssArduinoData(:,9))./ (pi*0.02*(180- ssArduinoData(:,10))/360))
%     plot(t_a, (ssArduinoData(:,9))./ (0.024191))
%     plot(t_a, ssArduinoData(:, 2))
%     ylim([0,800])
% 
%     linkaxes([ax1 ax2], 'x')

    ax3 = nexttile;
    hold on
    plot(t_a, (ssArduinoData(:,9))./ (mean(base_value)))
    plot(t_a, ssArduinoData(:, 2))
    ylim([0,2100])



    ax4 = nexttile;
    hold on
    plot(t_a, (ssArduinoData(:,9)-ssArduinoData(1,9))./ base_value)
    plot(t_a, ssArduinoData(:, 2))
    ylim([0,2100])

%     figure(42+cn)
%     hold on
%     plot(t_a, (ssArduinoData(:,9)-ssArduinoData(1,9))./ base_value)
%     plot(t_a, ssArduinoData(:, 2))
%     ylim([0,2100])


end

%% Stiffness Accuracy

stiffData = readmatrix("E1Data.xlsx");
stiffData(:,2) = [];
stiffData(:,7:8) = [];

stiffData = stiffnessesTension;
backStiffnesses = stiffnesses;
stiffData(:,5) = [];


stiffError = stiffData(:, 2:end) - stiffData(:,1);
%%
figure
tiledlayout(2,1, 'Padding', 'none', 'TileSpacing', 'compact'); 


p1 = nexttile;
hold on
plot(stiffData(1:40,1), stiffData(1:40,2:size(stiffData,2)))
plot(stiffData(1:40,1), stiffData(1:40,1), "r--")
lgd = legend({'e1', 'e2','e3', 'e4', 'e5', 'e6','e7','e8','e9','e10','e11','e12','e13', 'Reference'},'Location','southeast', 'Orientation','horizontal');
xlim([25, 2100])
% fontsize(lgd,'decrease')


set(gca,'XTickLabel',[])

x_label = '';
y_label = '$k_{m} (N/m)$';
processPlot

p2 = nexttile;
hold on
plot(stiffData(1:40,1), stiffError(1:40,2:size(stiffError,2)))
plot(stiffData(1:40,1), stiffData(1:40,1) - stiffData(1:40,1), "r--")
% legend({'e1', 'e2','e3', 'e4', 'e5'},'Location','southwest')

x_label = '$k_{o} (N/m)$';
y_label = '$k_{e} (N/m)$';
xlim([25, 2100])

processPlot

linkaxes([p1 p2], 'x')

% stiffData(:,2) = [];

% stiffnessesError = stiffnesses(:, 2:end) - stiffnesses(:,1);

figure
tiledlayout(2,1, 'Padding', 'none', 'TileSpacing', 'compact'); 


p1 = nexttile;
hold on
plot(stiffnesses(1:40,1), (stiffnesses(1:40,2:size(stiffnesses,2))))
plot(stiffnesses(1:40,1), stiffnesses(1:40,1), "r--")
lgd = legend({'e1', 'e2','e3', 'e4', 'e5', 'e6','e7','e8','e9','e10','e11','e12','e13', 'Reference'},'Location','southeast', 'Orientation','horizontal');
xlim([25, 2100])
% fontsize(lgd,'decrease')


set(gca,'XTickLabel',[])

x_label = '';
y_label = '$k_{m} (N/m)$';
processPlot

p2 = nexttile;
hold on
plot(stiffnesses(1:40,1), (stiffnesses(1:40,2:size(stiffnesses,2)) - stiffnesses(1:40,1)))
plot(stiffnesses(1:40,1), stiffnesses(1:40,1) - stiffnesses(1:40,1), "r--")
% legend({'e1', 'e2','e3', 'e4', 'e5'},'Location','southwest')

x_label = '$k_{o} (N/m)$';
y_label = '$k_{e} (N/m)$';
xlim([25, 2100])

processPlot

linkaxes([p1 p2], 'x')

% figure
% plot_distribution(stiffnesses(1:20,1), (stiffnesses(1:20,2:end) - stiffnesses(1:20,1))')

%%
figure
tiledlayout(2,1, 'Padding', 'none', 'TileSpacing', 'compact'); 

rb1 = nexttile;
hold on
plot(stiffData(1:40,1), stiffData(1:40,1), "r--")
plot_distribution(stiffData(1:40,1), stiffData(1:40,2:size(stiffData,2))', 'Color', [0 0.4470 0.7410]	)


yticks([50  1000  2000])
set(gca,'XTickLabel',[])

x_label = '';
y_label = '$k_{m} (N/m)$';
xlim([25, 2000])
processPlot

rb2 = nexttile;
hold on
plot(stiffData(1:40,1), stiffData(1:40,1) - stiffData(1:40,1), "r--")
plot_distribution(stiffData(1:40,1), stiffError(1:40,2:size(stiffError,2))', 'Color', [0 0.4470 0.7410]	)

x_label = '$k_{o} (N/m)$';
y_label = '$k_{e} (N/m)$';
xlim([25, 2000])

processPlot
linkaxes([rb1 rb2], 'x')




figure
tiledlayout(2,1, 'Padding', 'none', 'TileSpacing', 'compact'); 

rb1 = nexttile;
hold on
plot(stiffnesses(1:40,1), stiffnesses(1:40,1), "r--")
plot_distribution(stiffnesses(1:40,1), (stiffnesses(1:40,2:size(stiffnesses,2)))', 'Color', [0 0.4470 0.7410]	)


yticks([50  1000  2000])
set(gca,'XTickLabel',[])

x_label = '';
y_label = '$k_{m} (N/m)$';
xlim([25, 2000])
processPlot

rb2 = nexttile;
hold on
plot(stiffnesses(1:40,1), stiffnesses(1:40,1) - stiffnesses(1:40,1), "r--")
plot_distribution(stiffnesses(1:40,1), (stiffnesses(1:40,2:size(stiffnesses,2)) - stiffnesses(1:40,1))', 'Color', [0 0.4470 0.7410]	)

x_label = '$k_{o} (N/m)$';
y_label = '$k_{e} (N/m)$';
xlim([25, 2000])

processPlot
linkaxes([rb1 rb2], 'x')


%% Initial position analysis

figure
hold on
scatter(initialMarkerPositions(:, 3:3:end), initialMarkerPositions(:, 5:3:end))


figure
hold on
plot(initialMarkerPositions(:,15)- initialMarkerPositions(1,15))
plot(initialMarkerPositions(:,12)- initialMarkerPositions(1,12))
