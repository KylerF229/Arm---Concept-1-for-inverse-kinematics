function api = arm_length_study(p,targets,applyCallback)
% A target getter keeps an open study synchronized with new recorded targets.
if nargin<3, applyCallback=[]; end
targetSource=targets;
if isa(targetSource,'function_handle'), targets=targetSource(); end
assert(height(targets)>0,'Record one or more targets in the viewer first.');
result=[];
f=figure('Name','Optimize arm lengths for recorded targets','NumberTitle','off', ...
    'Color','w','Position',[100 80 1100 760]);
heading=textUI([.04 .92 .92 .05],sprintf('%d recorded targets | gripper stays %.3f m | lengths in metres',height(targets),p.viewer.gripperLength));
labels={'L1 minimum','L1 maximum','L2 minimum','L2 maximum','Step'};
defaults={'.20','.60','.20','.60','.01'}; ed=gobjects(5,1);
for k=1:5
    x=.04+(k-1)*.135; textUI([x .865 .13 .03],labels{k});
    ed(k)=uicontrol(f,'Style','edit','Units','normalized','Position',[x .825 .12 .04],'String',defaults{k});
end
objective=uicontrol(f,'Style','popupmenu','Units','normalized','Position',[.73 .825 .23 .045], ...
    'String',{'Shortest total length','Lowest peak holding torque'});
clearance=uicontrol(f,'Style','checkbox','Units','normalized','Position',[.04 .765 .43 .04], ...
    'String','Check link clearance from ground and rover','Value',1,'BackgroundColor','w');
runButton=uicontrol(f,'Style','pushbutton','Units','normalized','Position',[.51 .765 .20 .045], ...
    'String','Run length study','Callback',@runStudy);
applyButton=uicontrol(f,'Style','pushbutton','Units','normalized','Position',[.75 .765 .21 .045], ...
    'String','Apply best to viewer','Enable','off','Callback',@applyBest);
expand=uicontrol(f,'Style','checkbox','Units','normalized','Position',[.04 .715 .46 .035], ...
    'String','Auto-expand for unreachable targets','Value',1,'BackgroundColor','w');
textUI([.51 .715 .28 .03],'Maximum per link (m)');
ceiling=uicontrol(f,'Style','edit','Units','normalized','Position',[.80 .715 .16 .035],'String','1.00');
summary=textUI([.04 .645 .92 .065],'Run reads the latest recorded targets, including unreachable ones.');
textUI([.04 .61 .92 .035],'Feasible candidates, ranked by the chosen objective (first 100 shown)');
rankTable=uitable(f,'Units','normalized','Position',[.04 .335 .92 .275]);
textUI([.04 .29 .92 .035],'Best design: signed holding torques at each target');
targetTable=uitable(f,'Units','normalized','Position',[.04 .085 .92 .20]);
textUI([.04 .018 .92 .055],'Best on the sampled grid. Static gravity only; no drawer/payload/self-collision or path checks.');
api=struct('figure',f,'run',@runStudy,'getResult',@getResult,'apply',@applyBest);
 function h=textUI(pos,str)
    h=uicontrol(f,'Style','text','Units','normalized','Position',pos,'String',str,'HorizontalAlignment','left','BackgroundColor','w');
 end
 function r=getResult(), r=result; end
 function runStudy(varargin)
    result=[]; set(applyButton,'Enable','off');
    values=arrayfun(@(h)str2double(get(h,'String')),ed);
    if any(~isfinite(values)) || any(values<=0) || values(2)<values(1) || values(4)<values(3)
        set(summary,'String','Enter positive finite bounds and step, with maximum >= minimum.'); return
    end
    o=struct('L1Values',values(1):values(5):values(2),'L2Values',values(3):values(5):values(4), ...
        'objective','shortest','checkClearance',logical(get(clearance,'Value')), ...
        'autoExpand',logical(get(expand,'Value')),'maxLinkLength',str2double(get(ceiling,'String')));
    if get(objective,'Value')==2, o.objective='torque'; end
    set(runButton,'Enable','off'); set(applyButton,'Enable','off');
    set(summary,'String','Checking lengths, both elbow branches, masses and torques...'); drawnow;
    try
        if isa(targetSource,'function_handle'), targets=targetSource(); end
        set(heading,'String',sprintf('%d latest recorded targets | gripper %.3f m | lengths in metres',height(targets),p.viewer.gripperLength));
        result=arm_optimize_recorded_targets(p,targets,o);
        assignin('base','armLengthStudy',result);
        fields={'L1_m','L2_m','Total_m','TargetsReached','PeakJ1_Nm','PeakJ2_Nm','PeakJ3_Nm'};
        top=result.ranked(1:min(100,height(result.ranked)),fields);
        set(rankTable,'Data',table2cell(top),'ColumnName',fields,'ColumnWidth','auto');
        if isempty(result.bestP)
            set(targetTable,'Data',{}); set(summary,'String',result.message);
        else
            show={'TargetX_m','TargetZ_m','TargetAngle_deg','Payload_kg','J1_Nm','J2_Nm','J3_Nm'};
            set(targetTable,'Data',table2cell(result.targetResults(:,show)),'ColumnName',show,'ColumnWidth','auto');
            set(summary,'String',sprintf('%s\nOriginal total %.3f m; selected total %.3f m. %d/%d candidates feasible.\nResults saved as armLengthStudy. Apply updates this viewer and publishes pOptimized.', ...
                result.message,p.L1+p.L2,result.bestP.L1+result.bestP.L2,height(result.ranked),height(result.candidates)));
            if ~isempty(applyCallback), set(applyButton,'Enable','on'); end
        end
    catch err
        result=[]; set(summary,'String',err.message);
    end
    set(runButton,'Enable','on');
 end
 function applyBest(varargin)
    if isempty(result) || isempty(result.bestP) || isempty(applyCallback), return; end
    try
        if isa(targetSource,'function_handle')
            latest=targetSource();
            fields={'TargetX_m','TargetY_m','TargetZ_m','TargetAngle_deg','Payload_kg'};
            if ~isequaln(latest(:,fields),result.targets(:,fields))
                runStudy();
                if isempty(result) || isempty(result.bestP), return; end
            end
        end
        applyCallback(result);
        set(summary,'String',sprintf('%s\nApplied to the viewer. Use p = pOptimized for subsequent calculations.',result.message));
    catch err
        set(summary,'String',err.message);
    end
 end
end
