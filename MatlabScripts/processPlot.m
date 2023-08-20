%%

% Open your .fig file
% for example by using: h = openfig('your_figure.fig');

% Get the current axes
ax = gca;

% Set the font size
set(ax, 'FontSize', 24)
% Set the interpreter for the tick labels
set(ax, 'TickLabelInterpreter', 'latex')

% Get the x and y labels and set their interpreter
if (not(x_label == false))
    xl = xlabel(ax, string(x_label));
    set(xl, 'Interpreter', 'latex', 'FontSize', 28)
end

yl = ylabel(ax, string(y_label));
set(yl, 'Interpreter', 'latex', 'FontSize', 28)

% Get all line objects and modify their size and width
lines = findobj(gcf, 'Type', 'line');
for i = 1:length(lines)
    set(lines(i), 'markersize', 10, 'linewidth', 2);
end

% Modify the figure size
set(gcf, 'Position', [303 495 1115 331])

% Get the legend and set its interpreter
l = findobj(gcf, 'Type', 'legend');
set(l, 'Interpreter', 'latex')

% Turn on grid and box
grid on
box on
