function [x_target,y_target,q1_final,q2_final,q3_final,L_gripper] = selectGripperTarget1(q1_0,q2_0,q3_0,p)

% SETTINGS
L_gripper = p.Gripper; % m
q3_Gripper = deg2rad(0); % Gripper angle (rad)
elbow = -1; % Relative elbow angle sign: 1 or -1

% PLOT OUTPUTS
showTargetPlot = p.plotOutputs == 1 && p.plot.targetSelection == 1;

% REACHABLE GRIPPER-TIP POSITIONS FOR THE FINAL GRIPPER ANGLE
% Geometric reach only; joint limits and collisions are not included.
Rmax = p.L1 + p.L2 + L_gripper;
Rmin = max(0,2*max([p.L1 p.L2 L_gripper])-Rmax);
Rwmax = p.L1 + p.L2;
Rwmin = abs(p.L1-p.L2);
phi = linspace(0,2*pi,500);
tol = 1e-10*max(1,Rmax);

% INITIAL ARM POSITION (all angles are absolute)
x1 = p.L1*cos(q1_0);
y1 = p.L1*sin(q1_0);
x2 = x1 + p.L2*cos(q2_0);
y2 = y1 + p.L2*sin(q2_0);
x3 = x2 + L_gripper*cos(q3_0);
y3 = y2 + L_gripper*sin(q3_0);

% TARGET SELECTION PLOT
if showTargetPlot

fig = figure('Name','Select Gripper Target','NumberTitle','off');
ax = axes('Parent',fig);
hold(ax,'on')

fill(ax,Rmax*cos(phi),Rmax*sin(phi),[0.85 0.85 0.85], ...
    'FaceAlpha',0.35,'EdgeColor','k','LineStyle','--','LineWidth',1.5)

if Rmin > 0
    fill(ax,Rmin*cos(phi),Rmin*sin(phi),'w','EdgeColor','k', ...
        'LineStyle','--','LineWidth',1.5)
end

plot(ax,[0 x1 x2],[0 y1 y2],'-o','LineWidth',3)
plot(ax,[x2 x3],[y2 y3],'-o','LineWidth',3,'Color',[0.2 0.6 0.2])
plot(ax,0,0,'s','MarkerSize',10,'MarkerFaceColor','k','MarkerEdgeColor','k')

axis(ax,'equal')
grid(ax,'on')
xlabel(ax,'X position relative to arm base (m)')
ylabel(ax,'Y position relative to arm base (m)')
title(ax,{'Select the GRIPPER TIP position'})

margin = 0.1*Rmax;
xlim(ax,[-Rmax-margin Rmax+margin])
ylim(ax,[-Rmax-margin Rmax+margin])

method = menu('Select target position','Click on graph','Enter X and Y');

else

% Graph disabled - use manual X/Y input
method = 2;

end

if method == 0
    if showTargetPlot && isgraphics(fig), close(fig), end
    error('Arm:TargetCancelled','Target selection cancelled.')
end

% SELECT TARGET POSITION
while true

    if showTargetPlot && ~isgraphics(fig)
        error('Arm:TargetCancelled','Target selection cancelled.')
    end

    if method == 1
        figure(fig)
        axes(ax)

        try
            [x_target,y_target,button] = ginput(1);
        catch ME
            if ~isgraphics(fig)
                error('Arm:TargetCancelled','Target selection cancelled.')
            end
            rethrow(ME)
        end

        if isempty(button) || button == 27
            close(fig)
            error('Arm:TargetCancelled','Target selection cancelled.')
        end

        if button ~= 1
            continue
        end

    else
        answer = inputdlg({'X position relative to base (m):','Y position relative to base (m):'}, ...
            'Gripper Tip Target',[1 35],{'0.5','0.4'});

        if isempty(answer)
            if showTargetPlot && isgraphics(fig), close(fig), end
            error('Arm:TargetCancelled','Target selection cancelled.')
        end

        x_target = str2double(answer{1});
        y_target = str2double(answer{2});

        if ~isreal([x_target y_target]) || any(~isfinite([x_target y_target]))
            uiwait(warndlg('Enter finite, real numbers for X and Y.','Invalid Target'))
            continue
        end
    end

    % CHECK REACHABLE WORKSPACE
    d = hypot(x_target,y_target);

    if d > Rmax + tol || d < Rmin - tol
        uiwait(warndlg('Target is outside the reachable region. Select another position.','Unreachable Target'))
        continue
    end

    % KEEP PREFERRED GRIPPER ANGLE WHEN REACHABLE
    q3_final = q3_Gripper;
    xw = x_target - L_gripper*cos(q3_final);
    yw = y_target - L_gripper*sin(q3_final);
    r = hypot(xw,yw);

    if r > Rwmax || r < Rwmin

        % FIND CLOSEST FEASIBLE GRIPPER ORIENTATION
        if d > tol
            theta = atan2(y_target,x_target);
            r_allowed = max(Rwmin,min(Rwmax,r));
            c = (d^2 + L_gripper^2 - r_allowed^2)/(2*d*L_gripper);
            alpha = acos(max(-1,min(1,c)));
            candidates = [theta+alpha theta-alpha];
            difference = atan2(sin(candidates-q3_Gripper),cos(candidates-q3_Gripper));
            [~,index] = min(abs(difference));
            q3_final = candidates(index);
        end

        xw = x_target - L_gripper*cos(q3_final);
        yw = y_target - L_gripper*sin(q3_final);
        r = hypot(xw,yw);
    end

    % INVERSE KINEMATICS FOR THE TWO LINKS
    c2 = (xw^2 + yw^2 - p.L1^2 - p.L2^2)/(2*p.L1*p.L2);
    c2 = max(-1,min(1,c2));
    q2_relative = elbow*acos(c2);

    if r <= tol && Rwmin <= tol
        q1_final = q1_0;
    else
        q1_final = atan2(yw,xw) - atan2(p.L2*sin(q2_relative),p.L1+p.L2*cos(q2_relative));
    end

    q2_final = q1_final + q2_relative;

    % EQUIVALENT ANGLES NEAREST INITIAL POSITION
    q1_final = q1_0 + atan2(sin(q1_final-q1_0),cos(q1_final-q1_0));
    q2_final = q2_0 + atan2(sin(q2_final-q2_0),cos(q2_final-q2_0));
    q3_final = q3_0 + atan2(sin(q3_final-q3_0),cos(q3_final-q3_0));

    break
end

% SHOW SELECTED FINAL ARM POSITION
if showTargetPlot && isgraphics(fig)

xf1 = p.L1*cos(q1_final);
yf1 = p.L1*sin(q1_final);
xf2 = xf1 + p.L2*cos(q2_final);
yf2 = yf1 + p.L2*sin(q2_final);

plot(ax,[0 xf1 xf2],[0 yf1 yf2],'k-o','LineWidth',2)
plot(ax,[xf2 x_target],[yf2 y_target],'g-o','LineWidth',2)
plot(ax,x_target,y_target,'gx','MarkerSize',14,'LineWidth',3)

text(ax,x_target+0.02,y_target+0.02,sprintf('(%.2f, %.2f) m',x_target,y_target))

title(ax,sprintf('Target: X = %.3f m, Y = %.3f m; gripper angle = %.1f deg', ...
    x_target,y_target,rad2deg(q3_final)))

drawnow

end
end
