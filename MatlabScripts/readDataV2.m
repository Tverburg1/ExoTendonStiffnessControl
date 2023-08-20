close all; clear;

%% Load data
addpath("Raw Data\")

fileName = "TVerburgE22_7Processed.csv";
arduinoFileName = "TVerburgE22_7.csv";

data1 = readmatrix(fileName);

arduinoData = readmatrix(arduinoFileName);
arduinoData(1,:) = [];


filteredData = fillmissing(data1,'previous', 1);

%% Plot data for all markers

markers = ["AL", 'AR', 'B', 'EL', 'ER', 'FX', 'FY', 'FZ', 'O', 'SL', 'SR'];

for i = 1:((size(filteredData, 2)-2)/3)
    figure
    tiledlayout(3,1, 'Padding', 'none', 'TileSpacing', 'compact'); 

    nexttile
    plot(filteredData(:,3*i))
    title(sprintf("Marker %s X", markers(i)))

    nexttile
    plot(filteredData(:,3*(i)+1))
    title(sprintf("Marker %s Y", markers(i)))

    nexttile
    plot(filteredData(:,3*(i)+2))
    title(sprintf("Marker %s Z", markers(i)))
end

%% Top-Down Scatter plot

figure
scatter(filteredData(:, 3:3:end), filteredData(:, 5:3:end))
legend(markers,'Location','southwest')
%% Animated Scatter plot

figure
h = scatter(filteredData(1, 3:3:end), filteredData(1, 4:3:end),'Or') ;

for i = 1:length(filteredData)
    set(h,'XData',filteredData(i, 3:3:end),'YData',filteredData(i, 5:3:end)) ;
    drawnow
    pause(0.0001)
end

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


%% The error between the measured angle and desired angle

figure
hold on
plot(t, angleL)
plot(t, angleR)
plot(t_a, abs(targetAngle-targetAngle(1)))
plot(t_a, abs(actualAngle-actualAngle(1)))
x_label =  '$t (s)$';
y_label = '$\theta (rad)$';
title("Angle Error")
legend({'Angle L','Angle R', 'Desired Angle', 'Actual Angle'},'Location','northwest')

processPlot

figure
hold on
plot(t, angleL - abs(targetAngle-targetAngle(1)))
plot(t, angleR- abs(targetAngle-targetAngle(1)))
x_label =  '$t (s)$';
y_label = '$\theta (rad)$';
title("Angle Error")
legend({'Angle L Error','Angle R Error'},'Location','northwest')

processPlot

%% Displacement of spring in Z direction (forward)
figure
hold on
plot(t, SRdz)
plot(t, SLdz)

figure
hold on
plot(t, gradient(SRdz))
plot(t, gradient(SLdz))

%% Extension of spring

figure
plot(springExtension)

%% Calculation of actuation range

maxSRdz = SRdz(islocalmax(SRdz,'MinSeparation',1500));
minSRdz = SRdz(islocalmin(SRdz,'MinSeparation',1500));

dx = maxSRdz - minSRdz(1:length(maxSRdz));
    
minIdx = find(islocalmin(SRdz,'MinSeparation',1500) == 1);
base_value = dx(1).*ones(length(SRdz),1);

for i = 1:length(dx)
    base_value((minIdx(i)+1):end) = dx(i);
end

%% Measured stiffness vs desired stiffness

% displacementGradient = gradient(SRdz);
% b_value = 0;
% 
% for i = 1:length(SRdz)
%   if(and(displacementGradient(i) > 1*10^(-4), SRdz(i) < 0.0005))
%     b_value = SRdz(i);
%   end    
%   base_value(i) = b_value;
% end
% 
% measured_displacement = SRdz - base_value;

% figure
% plot(measured_displacement)

figure
hold on
plot(t, Ft./((SRdz + SLdz)/2))
plot(t_a, ssArduinoData(:, 2))
ylim([0,6000])


%% Bending Analysis E6

measuredTension = ssArduinoData(:,9).*0.00981;
displacementER = vecnorm((ER - ER(1,:)),1,2);

[maxDeflectionER, maxDisERIdx] = max(displacementER);

figure
hold on
scatter(filteredData(1, 3:3:end), filteredData(1, 5:3:end))
scatter(filteredData(maxDisERIdx, 3:3:end), filteredData(maxDisERIdx, 5:3:end))
% legend(markers,'Location','southwest')


figure
tiledlayout(4,1, 'Padding', 'none', 'TileSpacing', 'compact'); 

plt1 = nexttile;
plot(t_a, 5111)

x_label =  '';
y_label = '$F (N)$';
set(gca,'XTickLabel',[])
processPlot

plt2 = nexttile;
plot(t, displacementER)

x_label =  '';
y_label = '$d (m)$';
set(gca,'XTickLabel',[])
processPlot

plt3 = nexttile;
plot(t_a, measuredTension./displacementER)

x_label =  '';
y_label = '$k_{m} (N/m)$';
set(gca,'XTickLabel',[])
processPlot

plt4 = nexttile;
hold on
plot(t_a, ssArduinoData(:,5))
plot(t_a, ssArduinoData(:,6))
legend(["$\phi_{d}$" "$\phi_{m}$"],'Location','southeast')

x_label =  't (s)';
y_label = '$\phi (steps)$';
processPlot

linkaxes([plt1 plt2 plt3 plt4], 'x')
xlim([13 28])

%%

dERSection = abs(displacementER(1500:end, :) - displacementER(1500, :));
figure
plot(dERSection)
maxdERSectionDeflection = max(dERSection);

%% Peformance Metrics E2

EL = filteredData(:, 12:14);
ER = filteredData(:, 15:17);

angleL = asin((EL(:,1) - EL(1,1))/0.088);
angleR = asin((abs(ER(:,1) - ER(1,1)))/0.088);

figure
plot(t,angleR)


%% Constant Force E5
measuredTension = ssArduinoData(:,9);
displacementER = vecnorm((ER - ER(1,:)),1,2);


subSecMeasuredTension = measuredTension(1602:10002) - measuredTension(1);
meanSubSec = mean(subSecMeasuredTension);
stdSubSec = std(subSecMeasuredTension);

figure
tiledlayout(3,1, 'Padding', 'none', 'TileSpacing', 'compact'); 

grt1 = nexttile;
hold on
plot(t_a, measuredTension)
xlim([8 50])
x_label =  '';
y_label = '$F (N)$';
set(gca,'XTickLabel',[])
processPlot

% figure
% hold on
% plot(t_a(1935:10000), ssArduinoData(1935:10000, 2))
% plot(t(1935:10000), Ft(1935:10000)./(SRdz(1935:10000) +0.05))

grt2 = nexttile;
hold on
plot(t, Ft./(0.022 + SRdz))
plot(t_a, ssArduinoData(:, 2))
ylim([0 420])

xlim([8 50])
legend(["$k_{m}$" "$k_{d}$"],'Location','southwest')
x_label =  '';
y_label = '$k (N/m)$';
set(gca,'XTickLabel',[])
processPlot


grt3 = nexttile;
plot(t, (ssArduinoData(:, 2)) - (Ft./(SRdz +0.022)))


x_label =  't (s)';
y_label = '$k_{e} (N/m)$';
processPlot


linkaxes([grt1 grt2 grt3], 'x')
xlim([8 50])


%% Constant Displacement E3

figure
tiledlayout(2,1, 'Padding', 'none', 'TileSpacing', 'compact'); 

blt1 = nexttile;
hold on
plot(t, Ft./ ((SRdz + SLdz)/2 +0.03))
plot(t_a, ssArduinoData(:, 2))
ylim([0,10000])

legend(["$k_{m}$" "$k_{d}$"],'Location','northwest')
x_label =  '';
y_label = '$k (N/m)$';
set(gca,'XTickLabel',[])

processPlot

blt2 = nexttile;
plot(t, (ssArduinoData(:, 2) - Ft./ ((SRdz + SLdz)/2 +0.03)))

x_label =  '$t (s)$';
y_label = '$k_{e} (N/m)$';
ylim([0 1000])

processPlot

linkaxes([blt1 blt2], 'x')

figure
hold on
plot(t_a, ssArduinoData(:,9)*0.00981)
plot(t, Ft)

max(ssArduinoData(:,9)*0.00981)
max(Ft)

%% 

maxSRdz = SRdz(islocalmax(SRdz,'MinSeparation',3000));
minSRdz = SRdz(islocalmin(SRdz,'MinSeparation',3000));

dx = maxSRdz - minSRdz(1:length(maxSRdz));
    
minIdx = find(islocalmin(SRdz,'MinSeparation',3000) == 1);
base_value = dx(1).*ones(length(SRdz),1);

for i = 1:length(dx)
    base_value((minIdx(i)+1):end) = dx(i);
end

figure
tiledlayout(2,1, 'Padding', 'none', 'TileSpacing', 'compact'); 

blt1 = nexttile;
hold on
plot(t, Ft./ base_value)
plot(t_a, ssArduinoData(:, 2))
ylim([0,5000])

legend(["$k_{m}$" "$k_{d}$"],'Location','northwest')
x_label =  '';
y_label = '$k (N/m)$';
set(gca,'XTickLabel',[])

processPlot

blt2 = nexttile;
plot(t, (ssArduinoData(:, 2) - Ft./ base_value))

x_label =  '$t (s)$';
y_label = '$k_{e} (N/m)$';
ylim([0 1000])

processPlot

linkaxes([blt1 blt2], 'x')

figure
hold on
plot(t_a, ssArduinoData(:,9)*0.00981)
plot(t, Ft)

max(ssArduinoData(:,9)*0.00981)
max(Ft)


%% E10 Search for max


maxSRdz = SRdz(islocalmax(SRdz,'MinSeparation',3000));
minSRdz = SRdz(islocalmin(SRdz,'MinSeparation',3000));

dx = maxSRdz - minSRdz(1:length(maxSRdz));
    
minIdx = find(islocalmin(SRdz,'MinSeparation',3000) == 1);
base_value = dx(1).*ones(length(SRdz),1);

for i = 1:length(dx)
    base_value((minIdx(i)+1):end) = dx(i);
end

figure
tiledlayout(3,1, 'Padding', 'none', 'TileSpacing', 'compact'); 

blt1 = nexttile;
hold on
plot(t, Ft./ base_value)
plot(t_a, ssArduinoData(:, 2))
ylim([0,2800])

legend(["$k_{m}$" "$k_{d}$"],'Location','northwest')
x_label =  false;
y_label = '$k (N/m)$';
set(gca,'XTickLabel',[])

processPlot

blt2 = nexttile;
plot(t_a, ssArduinoData(:,9))

x_label =  false;
y_label = '$F (N)$';
set(gca,'XTickLabel',[])


processPlot

blt3 = nexttile;

plot(t, SRdz)

x_label =  '$t (s)$';
y_label = '$\Delta x$';
processPlot


linkaxes([blt1 blt2 blt3], 'x')
xlim([27 523])



max(ssArduinoData(:,9)*0.00981)
max(Ft)

