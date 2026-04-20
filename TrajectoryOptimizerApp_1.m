classdef TrajectoryOptimizerApp_1 < matlab.apps.AppBase

    % =====================================================================
    % UI COMPONENT DECLARATIONS
    % =====================================================================
    properties (Access = public)
        UIFigure        matlab.ui.Figure
        TabGroup        matlab.ui.container.TabGroup

        % --- TABS ---
        DynamicsTab     matlab.ui.container.Tab
        ConstraintsTab  matlab.ui.container.Tab
        ObjectiveTab    matlab.ui.container.Tab
        SolverTab       matlab.ui.container.Tab
        ComputeTab      matlab.ui.container.Tab
        ExampleTab      matlab.ui.container.Tab

        % --- DYNAMICS TAB ---
        DynLabel_n          matlab.ui.control.Label
        DynField_n          matlab.ui.control.NumericEditField
        DynLabel_m          matlab.ui.control.Label
        DynField_m          matlab.ui.control.NumericEditField
        DynLabel_nc         matlab.ui.control.Label
        DynField_nc         matlab.ui.control.NumericEditField
        DynTimeLabel        matlab.ui.control.Label
        DynTimeSwitch       matlab.ui.control.Switch
        DynGenerateButton   matlab.ui.control.Button
        DynConstLabel       matlab.ui.control.Label
        DynConstantsTable   matlab.ui.control.Table
        DynEOMLabel         matlab.ui.control.Label
        DynEOMTable         matlab.ui.control.Table
        DynConfirmButton    matlab.ui.control.Button
        DynStatusLabel      matlab.ui.control.Label
        DynInitialStateTable matlab.ui.control.Table

        % --- CONSTRAINTS TAB ---
        Con_InputLabel      matlab.ui.control.Label
        Con_InputTable      matlab.ui.control.Table
        Con_StateLabel      matlab.ui.control.Label
        Con_StateTable      matlab.ui.control.Table
        Con_TermCheck       matlab.ui.control.CheckBox
        Con_TermTable       matlab.ui.control.Table
        Con_ConfirmButton   matlab.ui.control.Button
        Con_StatusLabel     matlab.ui.control.Label

        % --- OBJECTIVE TAB ---
        Obj_EnableQ         matlab.ui.control.CheckBox
        Obj_QLabel          matlab.ui.control.Label
        Obj_QTable          matlab.ui.control.Table
        Obj_EnableQf        matlab.ui.control.CheckBox
        Obj_QfLabel         matlab.ui.control.Label
        Obj_QfTable         matlab.ui.control.Table
        Obj_QfNote          matlab.ui.control.Label
        Obj_EnableR         matlab.ui.control.CheckBox
        Obj_RLabel          matlab.ui.control.Label
        Obj_RTable          matlab.ui.control.Table
        Obj_ConfirmButton   matlab.ui.control.Button
        Obj_StatusLabel     matlab.ui.control.Label

        % --- SOLVER TAB ---
        Sol_TfLabel         matlab.ui.control.Label
        Sol_TfField         matlab.ui.control.NumericEditField
        Sol_ResLabel        matlab.ui.control.Label
        Sol_ResGroup        matlab.ui.container.ButtonGroup
        % Sol_LowBtn          matlab.ui.control.RadioButton
        % Sol_MedBtn          matlab.ui.control.RadioButton
        % Sol_HighBtn         matlab.ui.control.RadioButton
        Sol_HorizonField    matlab.ui.control.NumericEditField
        % Sol_AdvCheck        matlab.ui.control.CheckBox
        % Sol_dtLabel         matlab.ui.control.Label
        % Sol_dtField         matlab.ui.control.NumericEditField
        % Sol_FreeTfCheck     matlab.ui.control.CheckBox
        Sol_ExportButton    matlab.ui.control.Button
        Sol_StatusLabel     matlab.ui.control.Label

        % --- COMPUTE TAB ---
        Comp_RunButton      matlab.ui.control.Button
        Comp_StatusLabel    matlab.ui.control.Label
        Comp_TextArea       matlab.ui.control.TextArea
        Comp_UIAxes_X       matlab.ui.control.UIAxes
        Comp_UIAxes_U       matlab.ui.control.UIAxes

        % --- EXAMPLE TAB ---
        Ex_UIAxes           matlab.ui.control.UIAxes
        Ex_RunButton        matlab.ui.control.Button
        %Ex_ReplayButton     matlab.ui.control.Button
        Ex_Frames struct = struct('cdata', {}, 'colormap', {})
        Ex_Validation       matlab.ui.control.Button
        Ex_ValidationLabel  matlab.ui.control.Label
    end

    % =====================================================================
    % INTERNAL DATA (not UI)
    % =====================================================================
    properties (Access = private)
        %problem definition
        n   % number of state variables
        m   % number of control inputs
        nc  % number of constants

        %NMPC execution
        NMPCFuture
        ProgressQueue
        Xsol %optimized trajectory
        Usol %optimized input sequence

        %Example Validation
        Xexample
        Uexample

        dynamicsConfirmed    logical = false
        constraintsConfirmed logical = false
        objectiveConfirmed   logical = false
    end

    % =====================================================================
    % BUILD THE UI
    % =====================================================================
    methods (Access = private)

        function createComponents(app)

            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 800 620];
            app.UIFigure.Name = 'Trajectory Optimizer';

            app.TabGroup = uitabgroup(app.UIFigure);
            app.TabGroup.Position = [10 10 780 600];

            app.DynamicsTab    = uitab(app.TabGroup, 'Title', 'Dynamics');
            app.ConstraintsTab = uitab(app.TabGroup, 'Title', 'Constraints');
            app.ObjectiveTab   = uitab(app.TabGroup, 'Title', 'Objective');
            app.SolverTab      = uitab(app.TabGroup, 'Title', 'Solver');

            % =============================================================
            % COMPUTE TAB
            % =============================================================
            app.ComputeTab = uitab(app.TabGroup, 'Title', 'Compute');
            p = app.ComputeTab;
            
            app.Comp_RunButton = uibutton(p,'push',...
                'Text','Run NMPC',...
                'Position',[30 540 150 40],...
                'FontWeight','bold',...
                'ButtonPushedFcn', @(~,~) app.onRunNMPC());
            
            app.Comp_StatusLabel = uilabel(p,...
                'Text','Idle.',...
                'Position',[200 550 500 22]);
            
            app.Comp_TextArea = uitextarea(p,...
                'Position',[30 300 740 220],...
                'Editable','off');
            
            app.Comp_UIAxes_X = uiaxes(p,...
                'Position',[30 30 350 240]);
            title(app.Comp_UIAxes_X,'State Trajectory')
            
            app.Comp_UIAxes_U = uiaxes(p,...
                'Position',[420 30 350 240]);
            title(app.Comp_UIAxes_U,'Control Inputs')

            % =============================================================
            % DYNAMICS TAB
            % =============================================================
            p = app.DynamicsTab;

            app.DynLabel_n = uilabel(p, 'Text', '# State Variables (n):', ...
                'Position', [20 530 155 22], 'FontWeight', 'bold');
            app.DynField_n = uieditfield(p, 'numeric', ...
                'Position', [180 530 55 22], 'Value', 2, ...
                'Limits', [1 50], 'RoundFractionalValues', true);

            app.DynLabel_m = uilabel(p, 'Text', '# Control Inputs (m):', ...
                'Position', [265 530 150 22], 'FontWeight', 'bold');
            app.DynField_m = uieditfield(p, 'numeric', ...
                'Position', [420 530 55 22], 'Value', 1, ...
                'Limits', [1 50], 'RoundFractionalValues', true);

            app.DynLabel_nc = uilabel(p, 'Text', '# Constants:', ...
                'Position', [505 530 90 22], 'FontWeight', 'bold');
            app.DynField_nc = uieditfield(p, 'numeric', ...
                'Position', [600 530 55 22], 'Value', 0, ...
                'Limits', [0 50], 'RoundFractionalValues', true);

            app.DynTimeLabel = uilabel(p, 'Text', 'Time Type:', ...
                'Position', [20 493 110 22], 'FontWeight', 'bold');
            app.DynTimeSwitch = uiswitch(p, 'slider', ...
                'Items', {'Continuous', 'Discrete'}, ...
                'Position', [160 490 110 26], 'Value', 'Continuous');

            app.DynGenerateButton = uibutton(p, 'push', ...
                'Text', 'Generate Tables', ...
                'Position', [620 488 130 28], 'FontWeight', 'bold', ...
                'ButtonPushedFcn', @(~,~) app.onGenerateTables());

            app.DynConstLabel = uilabel(p, 'Text', 'Constants', ...
                'Position', [20 458 200 22], 'FontWeight', 'bold', 'FontSize', 13);

            app.DynConstantsTable = uitable(p, ...
                'Position', [20 355 400 100], ...
                'ColumnName', {'Symbol', 'Name', 'Value'}, ...
                'ColumnWidth', {70, 150, 150}, ...
                'ColumnEditable', [false true true], ...
                'Data', {});

            app.DynEOMLabel = uilabel(p, 'Text', 'Equations of Motion', ...
                'Position', [20 320 300 22], 'FontWeight', 'bold', 'FontSize', 13);

            app.DynEOMTable = uitable(p, ...
                'Position', [20 175 740 140], ...
                'ColumnName', {'Variable', 'Equation  (use x1,x2,...  u1,u2,...  and your constant names)'}, ...
                'ColumnWidth', {75, 635}, ...
                'ColumnEditable', [false true], ...
                'Data', {});

            app.DynConfirmButton = uibutton(p, 'push', ...
                'Text', 'Confirm Dynamics', ...
                'Position', [620 60 140 30], 'FontWeight', 'bold', ...
                'BackgroundColor', [0.18 0.55 0.27], 'FontColor', [1 1 1], ...
                'ButtonPushedFcn', @(~,~) app.onConfirmDynamics());

            app.DynStatusLabel = uilabel(p, ...
                'Text', 'Enter values above and click Generate Tables.', ...
                'Position', [20 130 570 22], 'FontColor', [0.5 0.5 0.5]);

            app.DynInitialStateTable = uitable(p, ...
                'Position', [20 40 400 90], ...
                'ColumnName', {'State', 'Initial Value'}, ...
                'ColumnWidth', {80, 150}, ...
                'ColumnEditable', [false true], ...
                'Data', {});

            % =============================================================
            % CONSTRAINTS TAB
            % =============================================================
            p = app.ConstraintsTab;

            app.Con_InputLabel = uilabel(p, 'Text', 'Input Bounds', ...
                'Position', [20 530 200 22], 'FontWeight', 'bold', 'FontSize', 13);
            uilabel(p, 'Position', [20 508 650 18], 'FontColor', [0.5 0.5 0.5], ...
                'Text', 'Limits on control inputs at every timestep. Leave as -Inf / Inf if unconstrained.');

            app.Con_InputTable = uitable(p, ...
                'Position', [20 405 500 100], ...
                'ColumnName', {'Input', 'Min', 'Max'}, ...
                'ColumnWidth', {80, 200, 200}, ...
                'ColumnEditable', [false true true], 'Data', {});

            app.Con_StateLabel = uilabel(p, 'Text', 'State Bounds', ...
                'Position', [20 373 200 22], 'FontWeight', 'bold', 'FontSize', 13);
            uilabel(p, 'Position', [20 351 650 18], 'FontColor', [0.5 0.5 0.5], ...
                'Text', 'Limits on states at every timestep. Leave as -Inf / Inf if unconstrained.');

            app.Con_StateTable = uitable(p, ...
                'Position', [20 248 500 100], ...
                'ColumnName', {'State', 'Min', 'Max'}, ...
                'ColumnWidth', {80, 200, 200}, ...
                'ColumnEditable', [false true true], 'Data', {});

            app.Con_TermCheck = uicheckbox(p, ...
                'Text', 'Enable Terminal State Constraint', ...
                'Position', [20 215 280 22], 'FontWeight', 'bold', 'FontSize', 13, ...
                'ValueChangedFcn', @(~,~) app.onTerminalStateToggled());
            uilabel(p, 'Position', [20 193 680 18], 'FontColor', [0.5 0.5 0.5], ...
                'Text', 'Forces trajectory to end at exact values. When active, Qf in Objective tab is disabled.');

            app.Con_TermTable = uitable(p, ...
                'Position', [20 90 400 100], ...
                'ColumnName', {'State', 'Required Final Value'}, ...
                'ColumnWidth', {80, 290}, ...
                'ColumnEditable', [false true], 'Data', {}, 'Visible', 'off');

            app.Con_ConfirmButton = uibutton(p, 'push', ...
                'Text', 'Confirm Constraints', ...
                'Position', [620 45 140 30], 'FontWeight', 'bold', ...
                'BackgroundColor', [0.18 0.55 0.27], 'FontColor', [1 1 1], ...
                'ButtonPushedFcn', @(~,~) app.onConfirmConstraints());

            app.Con_StatusLabel = uilabel(p, ...
                'Text', 'Confirm Dynamics first to populate these tables.', ...
                'Position', [20 45 570 22], 'FontColor', [0.5 0.5 0.5]);

            % =============================================================
            % OBJECTIVE TAB
            % =============================================================
            p = app.ObjectiveTab;

            app.Obj_EnableQ = uicheckbox(p, ...
                'Text', 'Enable Running State Cost  Q  (n x n)', ...
                'Position', [20 530 280 22], 'FontWeight', 'bold', 'FontSize', 13, 'Value', true, ...
                'ValueChangedFcn', @(src,~) app.onMatrixToggle(src, app.Obj_QTable));
            app.Obj_QLabel = uilabel(p, 'Position', [20 508 680 18], 'FontColor', [0.5 0.5 0.5], ...
                'Text', 'Penalizes state deviation from zero throughout the trajectory. Diagonal weights only.');
            app.Obj_QTable = uitable(p, ...
                'Position', [20 400 450 105], ...
                'ColumnName', {'State', 'Diagonal Weight'}, ...
                'ColumnWidth', {80, 270}, ...
                'ColumnEditable', [false true], 'Data', {});

            app.Obj_EnableQf = uicheckbox(p, ...
                'Text', 'Enable Terminal Cost  Qf  (n x n)', ...
                'Position', [20 368 280 22], 'FontWeight', 'bold', 'FontSize', 13, 'Value', true, ...
                'ValueChangedFcn', @(src,~) app.onMatrixToggle(src, app.Obj_QfTable));
            app.Obj_QfLabel = uilabel(p, 'Position', [20 346 500 18], 'FontColor', [0.5 0.5 0.5], ...
                'Text', 'Penalizes state deviation from zero at the final time only.');
            app.Obj_QfNote = uilabel(p, ...
                'Text', 'Disabled: terminal state constraint is active.', ...
                'Position', [310 368 380 22], 'FontColor', [0.85 0.5 0.1], 'Visible', 'off');
            app.Obj_QfTable = uitable(p, ...
                'Position', [20 238 450 105], ...
                'ColumnName', {'State', 'Diagonal Weight'}, ...
                'ColumnWidth', {80, 270}, ...
                'ColumnEditable', [false true], 'Data', {});

            app.Obj_EnableR = uicheckbox(p, ...
                'Text', 'Enable Control Cost  R  (m x m)', ...
                'Position', [20 205 280 22], 'FontWeight', 'bold', 'FontSize', 13, 'Value', true, ...
                'ValueChangedFcn', @(src,~) app.onMatrixToggle(src, app.Obj_RTable));
            app.Obj_RLabel = uilabel(p, 'Position', [20 183 680 18], 'FontColor', [0.5 0.5 0.5], ...
                'Text', 'Penalizes large control inputs. Keep > 0 to avoid numerical blow-up.');
            app.Obj_RTable = uitable(p, ...
                'Position', [20 75 450 105], ...
                'ColumnName', {'Input', 'Diagonal Weight'}, ...
                'ColumnWidth', {80, 270}, ...
                'ColumnEditable', [false true], 'Data', {});

            app.Obj_ConfirmButton = uibutton(p, 'push', ...
                'Text', 'Confirm Objective', ...
                'Position', [620 40 140 30], 'FontWeight', 'bold', ...
                'BackgroundColor', [0.18 0.55 0.27], 'FontColor', [1 1 1], ...
                'ButtonPushedFcn', @(~,~) app.onConfirmObjective());

            app.Obj_StatusLabel = uilabel(p, ...
                'Text', 'Confirm Dynamics first to populate these tables.', ...
                'Position', [20 40 570 22], 'FontColor', [0.5 0.5 0.5]);

            % =============================================================
            % SOLVER TAB
            % =============================================================
            p = app.SolverTab;

            app.Sol_TfLabel = uilabel(p, 'Text', 'Final Time  T  (s):', ...
                'Position', [20 530 140 22], 'FontWeight', 'bold');
            app.Sol_TfField = uieditfield(p, 'numeric', ...
                'Position', [165 530 80 22], 'Value', 10);

            app.Sol_HorizonField = uieditfield(p,'numeric',...
                'Position',[180 490 50 22],'Value',50);

            app.Sol_ResLabel = uilabel(p, 'Text', 'Resolution  (N timesteps):', ...
                'Position', [20 490 190 22], 'FontWeight', 'bold');
            app.Sol_ResGroup = uibuttongroup(p, ...
                'Position', [20 455 420 35], 'BorderType', 'none');
            % app.Sol_LowBtn  = uiradiobutton(app.Sol_ResGroup, ...
            %     'Text', 'Low  (N=20)',    'Position', [5   8 120 22]);
            % app.Sol_MedBtn  = uiradiobutton(app.Sol_ResGroup, ...
            %     'Text', 'Medium  (N=50)', 'Position', [140 8 140 22]);
            % app.Sol_HighBtn = uiradiobutton(app.Sol_ResGroup, ...
            %     'Text', 'High  (N=100)', 'Position', [295 8 120 22]);
            % app.Sol_MedBtn.Value = true;

            % app.Sol_AdvCheck = uicheckbox(p, ...
            %     'Text', 'Advanced Mode', ...
            %     'Position', [20 410 160 22], 'FontWeight', 'bold', 'FontSize', 12, ...
            %     'ValueChangedFcn', @(~,~) app.onAdvancedToggled());

            % app.Sol_dtLabel = uilabel(p, 'Text', 'Fixed  dt  (s):', ...
            %     'Position', [40 378 110 22], 'Visible', 'off');
            % app.Sol_dtField = uieditfield(p, 'numeric', ...
            %     'Position', [155 378 80 22], 'Value', 0.1, 'Visible', 'off');
            % app.Sol_FreeTfCheck = uicheckbox(p, ...
            %     'Text', 'Free Final Time  (solver finds T)', ...
            %     'Position', [40 348 260 22], 'Visible', 'off');

            app.Sol_ExportButton = uibutton(p, 'push', ...
                'Text', 'EXPORT PROBLEM TO WORKSPACE', ...
                'Position', [210 250 340 45], ...
                'FontWeight', 'bold', 'FontSize', 14, ...
                'BackgroundColor', [0.15 0.40 0.70], 'FontColor', [1 1 1], ...
                'ButtonPushedFcn', @(~,~) app.onExport());

            uilabel(p, 'Position', [20 218 720 18], 'FontColor', [0.5 0.5 0.5], ...
                'HorizontalAlignment', 'center', ...
                'Text', 'Saves everything as  TrajOptProblem  in the MATLAB workspace for the solver script.');

            app.Sol_StatusLabel = uilabel(p, ...
                'Text', 'Status: Complete all tabs then export.', ...
                'Position', [20 185 720 22], ...
                'FontColor', [0.5 0.5 0.5], 'HorizontalAlignment', 'center');

            app.UIFigure.Visible = 'on';

            % =============================================================
            % EXAMPLE TAB
            % =============================================================
            app.ExampleTab = uitab(app.TabGroup,'Title','Example');
            p = app.ExampleTab;
            
            app.Ex_RunButton = uibutton(p,'push',...
                'Text','Run Animation',...
                'Position',[30 540 150 40],...
                'ButtonPushedFcn', @(~,~) app.runExampleAnimation());

            app.Ex_Validation = uibutton(p,'push',...
                'Text','Evaluate Example Check Case',...
                'Position',[250 540 200 40],...
                'ButtonPushedFcn',@(~,~) app.runCheckCase());

            app.Ex_ValidationLabel = uilabel(p,...
                'Text','Not evaluated',...
                'Position',[470 550 250 22],...
                'FontWeight','bold');
            
            % app.Ex_ReplayButton = uibutton(p,'push',...
            %     'Text','Replay',...
            %     'Position',[200 540 150 40],...
            %     'ButtonPushedFcn', @(~,~) app.replayAnimation());
            
            app.Ex_UIAxes = uiaxes(p,...
                'Position',[30 50 720 460]);
            
            axis(app.Ex_UIAxes,'equal')
            hold(app.Ex_UIAxes,'on')
        end

    end

    % =====================================================================
    % CALLBACKS
    % =====================================================================
    methods (Access = private)

        % -----------------------------------------------------------------
        % DYNAMICS: Generate Tables
        % -----------------------------------------------------------------
        function onGenerateTables(app)
            app.n  = app.DynField_n.Value;
            app.m  = app.DynField_m.Value;
            app.nc = app.DynField_nc.Value;

            if app.nc > 0
                syms = arrayfun(@(i) sprintf('c%d',i), 1:app.nc, 'UniformOutput', false)';
                app.DynConstantsTable.Data = [syms, repmat({''},app.nc,1), repmat({''},app.nc,1)];
            else
                app.DynConstantsTable.Data = {};
            end

            varLabels = arrayfun(@(i) sprintf('x%ddot',i), 1:app.n, 'UniformOutput', false)';
            app.DynEOMTable.Data = [varLabels, repmat({''},app.n,1)];

            stateLabels = arrayfun(@(i) sprintf('x%d', i), 1:app.n, 'UniformOutput', false)';
            app.DynInitialStateTable.Data = [stateLabels, num2cell(zeros(app.n,1))];

            app.dynamicsConfirmed = false;
            app.setDynStatus('Tables generated. Fill in equations then click Confirm Dynamics.', 'warn');
        end

        % -----------------------------------------------------------------
        % DYNAMICS: Confirm
        % -----------------------------------------------------------------
        function onConfirmDynamics(app)
            if isempty(app.DynEOMTable.Data)
                app.setDynStatus('Error: Click Generate Tables first.', 'error'); return
            end

            eomData  = app.DynEOMTable.Data;
            eq_cells = eomData(:, 2);
            if any(cellfun(@(s) isempty(strtrim(s)), eq_cells))
                app.setDynStatus('Error: All EOM rows must be filled in.', 'error'); return
            end

            const_names = {};
            const_vals  = {};
            if app.nc > 0
                constData   = app.DynConstantsTable.Data;
                const_names = constData(:, 2);
                const_vals  = constData(:, 3);
                if any(cellfun(@(s) isempty(strtrim(s)), const_names))
                    app.setDynStatus('Error: All constant Name fields must be filled in.', 'error'); return
                end
                if any(cellfun(@(s) isempty(strtrim(s)), const_vals))
                    app.setDynStatus('Error: All constant Value fields must be filled in.', 'error'); return
                end
            end

            initData = app.DynInitialStateTable.Data;
            init_x0 = cell2mat(initData(:,2));
            
            assignin('base','dyn_x0',init_x0);
            assignin('base', 'dyn_eq_cells',    eq_cells);
            assignin('base', 'dyn_const_names', const_names);
            assignin('base', 'dyn_const_vals',  const_vals);
            assignin('base', 'dyn_n',           app.n);
            assignin('base', 'dyn_m',           app.m);
            assignin('base', 'dyn_nc',          app.nc);
            assignin('base', 'dyn_timeType',    app.DynTimeSwitch.Value);

            app.dynamicsConfirmed = true;
            app.populateConstraintsTables();
            app.populateObjectiveTables();
            app.setDynStatus(sprintf( ...
                'Dynamics confirmed and saved.  n=%d  m=%d  — Constraints and Objective tabs are now populated.', ...
                app.n, app.m), 'ok');
        end

        % -----------------------------------------------------------------
        % Populate Constraints tab
        % -----------------------------------------------------------------
        function populateConstraintsTables(app)
            inputLabels = arrayfun(@(i) sprintf('u%d',i), 1:app.m, 'UniformOutput', false)';
            stateLabels = arrayfun(@(i) sprintf('x%d',i), 1:app.n, 'UniformOutput', false)';

            app.Con_InputTable.Data = [inputLabels, repmat({'-Inf'},app.m,1), repmat({'Inf'},app.m,1)];
            app.Con_StateTable.Data = [stateLabels, repmat({'-Inf'},app.n,1), repmat({'Inf'},app.n,1)];
            app.Con_TermTable.Data  = [stateLabels, repmat({''},app.n,1)];

            app.Con_StatusLabel.Text = 'Fill in bounds then click Confirm Constraints.';
            app.Con_StatusLabel.FontColor = [0.8 0.75 0.2];
        end

        % -----------------------------------------------------------------
        % Populate Objective tab
        % -----------------------------------------------------------------
        function populateObjectiveTables(app)
            stateLabels = arrayfun(@(i) sprintf('x%d',i), 1:app.n, 'UniformOutput', false)';
            inputLabels = arrayfun(@(i) sprintf('u%d',i), 1:app.m, 'UniformOutput', false)';

            app.Obj_QTable.Data  = [stateLabels, num2cell(ones(app.n,1))];
            app.Obj_QfTable.Data = [stateLabels, num2cell(ones(app.n,1))];
            app.Obj_RTable.Data  = [inputLabels, num2cell(ones(app.m,1))];

            app.Obj_StatusLabel.Text = 'Set weights then click Confirm Objective.';
            app.Obj_StatusLabel.FontColor = [0.8 0.75 0.2];
        end

        % -----------------------------------------------------------------
        % CONSTRAINTS: Terminal state toggle
        % -----------------------------------------------------------------
        function onTerminalStateToggled(app)
            isOn = app.Con_TermCheck.Value;
            if isOn
                app.Con_TermTable.Visible   = 'on';
                app.Obj_EnableQf.Value      = false;
                app.Obj_EnableQf.Enable     = 'off';
                app.Obj_QfTable.Enable      = 'off';
                app.Obj_QfNote.Visible      = 'on';
            else
                app.Con_TermTable.Visible   = 'off';
                app.Obj_EnableQf.Enable     = 'on';
                app.Obj_QfTable.Enable      = 'on';
                app.Obj_QfNote.Visible      = 'off';
            end
        end

        % -----------------------------------------------------------------
        % CONSTRAINTS: Confirm
        % -----------------------------------------------------------------
        function onConfirmConstraints(app)
            if ~app.dynamicsConfirmed
                app.Con_StatusLabel.Text = 'Error: Confirm Dynamics first.';
                app.Con_StatusLabel.FontColor = [0.8 0.2 0.2]; return
            end

            iBnd  = app.Con_InputTable.Data;
            sBnd  = app.Con_StateTable.Data;

            useTerminal = app.Con_TermCheck.Value;
            x_terminal  = {};
            if useTerminal
                tData      = app.Con_TermTable.Data;
                x_terminal = tData(:,2);
                if any(cellfun(@(s) isempty(strtrim(s)), x_terminal))
                    app.Con_StatusLabel.Text = ...
                        'Error: Fill in all terminal state values, or uncheck terminal state.';
                    app.Con_StatusLabel.FontColor = [0.8 0.2 0.2]; return
                end
            end

            assignin('base', 'con_u_min',       iBnd(:,2));
            assignin('base', 'con_u_max',       iBnd(:,3));
            assignin('base', 'con_x_min',       sBnd(:,2));
            assignin('base', 'con_x_max',       sBnd(:,3));
            assignin('base', 'con_useTerminal', useTerminal);
            assignin('base', 'con_x_terminal',  x_terminal);

            app.constraintsConfirmed = true;
            app.Con_StatusLabel.Text = 'Constraints confirmed and saved to workspace.';
            app.Con_StatusLabel.FontColor = [0.2 0.6 0.3];
        end

        % -----------------------------------------------------------------
        % OBJECTIVE: Matrix enable toggle
        % -----------------------------------------------------------------
        function onMatrixToggle(~, checkbox, table)
            if checkbox.Value
                table.Enable = 'on';
            else
                table.Enable = 'off';
            end
        end

        % -----------------------------------------------------------------
        % OBJECTIVE: Confirm
        % -----------------------------------------------------------------
        function onConfirmObjective(app)
            if ~app.dynamicsConfirmed
                app.Obj_StatusLabel.Text = 'Error: Confirm Dynamics first.';
                app.Obj_StatusLabel.FontColor = [0.8 0.2 0.2]; return
            end

            useQ  = app.Obj_EnableQ.Value;
            useQf = app.Obj_EnableQf.Value;
            useR  = app.Obj_EnableR.Value;

            if useQ
                Q_diag = cell2mat(app.Obj_QTable.Data(:,2));
            else
                Q_diag = zeros(app.n,1);
            end

            if useQf
                Qf_diag = cell2mat(app.Obj_QfTable.Data(:,2));
            else
                Qf_diag = zeros(app.n,1);
            end

            if useR
                R_diag = cell2mat(app.Obj_RTable.Data(:,2));
            else
                R_diag = zeros(app.m,1);
            end

            assignin('base', 'obj_useQ',    useQ);
            assignin('base', 'obj_useQf',   useQf);
            assignin('base', 'obj_useR',    useR);
            assignin('base', 'obj_Q_diag',  Q_diag);
            assignin('base', 'obj_Qf_diag', Qf_diag);
            assignin('base', 'obj_R_diag',  R_diag);

            app.objectiveConfirmed = true;
            app.Obj_StatusLabel.Text = 'Objective confirmed and saved to workspace.';
            app.Obj_StatusLabel.FontColor = [0.2 0.6 0.3];
        end

        % -----------------------------------------------------------------
        % SOLVER: Advanced mode toggle
        % -----------------------------------------------------------------
        % function onAdvancedToggled(app)
        %     if app.Sol_AdvCheck.Value, vis = 'on'; else, vis = 'off'; end
        %     app.Sol_dtLabel.Visible     = vis;
        %     app.Sol_dtField.Visible     = vis;
        %     app.Sol_FreeTfCheck.Visible = vis;
        % end

        % -----------------------------------------------------------------
        % SOLVER: Export everything as TrajOptProblem struct
        % -----------------------------------------------------------------
        function onExport(app)
            warnings = {};
            if ~app.dynamicsConfirmed,    warnings{end+1} = 'Dynamics not confirmed.';    end
            if ~app.constraintsConfirmed, warnings{end+1} = 'Constraints not confirmed.'; end
            if ~app.objectiveConfirmed,   warnings{end+1} = 'Objective not confirmed.';   end

            if ~isempty(warnings)
                app.Sol_StatusLabel.Text = ['Warning: ' strjoin(warnings,' | ') ...
                    '  Exporting with current data.'];
                app.Sol_StatusLabel.FontColor = [0.85 0.5 0.1];
            end

            try
                prob = struct();

                % Dynamics
                prob.n           = evalin('base','dyn_n');
                prob.m           = evalin('base','dyn_m');
                prob.nc          = evalin('base','dyn_nc');
                prob.eq_cells    = evalin('base','dyn_eq_cells');
                prob.const_names = evalin('base','dyn_const_names');
                prob.const_vals  = evalin('base','dyn_const_vals');
                prob.timeType    = evalin('base','dyn_timeType');
                prob.x0          = evalin('base','dyn_x0');

                % Constraints
                prob.u_min       = evalin('base','con_u_min');
                prob.u_max       = evalin('base','con_u_max');
                prob.x_min       = evalin('base','con_x_min');
                prob.x_max       = evalin('base','con_x_max');
                prob.useTerminal = evalin('base','con_useTerminal');
                prob.x_terminal  = evalin('base','con_x_terminal');

                % Objective
                prob.useQ    = evalin('base','obj_useQ');
                prob.useQf   = evalin('base','obj_useQf');
                prob.useR    = evalin('base','obj_useR');
                prob.Q_diag  = evalin('base','obj_Q_diag');
                prob.Qf_diag = evalin('base','obj_Qf_diag');
                prob.R_diag  = evalin('base','obj_R_diag');

                % Solver settings
                prob.T_final      = app.Sol_TfField.Value;
                % prob.advancedMode = app.Sol_AdvCheck.Value;
                % prob.freeFinalTime = app.Sol_FreeTfCheck.Value;

                % if app.Sol_LowBtn.Value
                %     prob.N = 20;
                % elseif app.Sol_MedBtn.Value
                %     prob.N = 50;
                % else
                %     prob.N = 100;
                % end
                prob.N = app.Sol_HorizonField.Value;

                % if app.Sol_AdvCheck.Value
                %     prob.dt = app.Sol_dtField.Value;
                % else
                %     prob.dt = prob.T_final / prob.N;
                % end
                prob.dt = prob.T_final/prob.N;

                assignin('base', 'TrajOptProblem', prob);

                if isempty(warnings)
                    app.Sol_StatusLabel.Text = ...
                        'Success: TrajOptProblem exported to workspace. Run your solver script now.';
                    app.Sol_StatusLabel.FontColor = [0.2 0.6 0.3];
                end

            catch ME
                app.Sol_StatusLabel.Text = ['Export error: ' ME.message];
                app.Sol_StatusLabel.FontColor = [0.8 0.2 0.2];
            end
        end

        % -----------------------------------------------------------------
        % HELPER: Set dynamics status label
        % -----------------------------------------------------------------
        function setDynStatus(app, msg, type)
            app.DynStatusLabel.Text = msg;
            switch type
                case 'ok',    app.DynStatusLabel.FontColor = [0.2 0.6 0.3];
                case 'error', app.DynStatusLabel.FontColor = [0.8 0.2 0.2];
                case 'warn',  app.DynStatusLabel.FontColor = [0.8 0.75 0.2];
            end
        end

        % --------------------------
        % Animation
        % ----------------
        function runExampleAnimation(app)
            try
                try
                    app.Xsol;
                catch
                    uialert(app.UIFigure,'Run NMPC first','Error');
                    return
                end
           
                if isempty(app.Ex_UIAxes) || ~isvalid(app.Ex_UIAxes)
                    error('Ex_UIAxes not valid');
                end
            
                x = app.Xsol(:,1);
                theta = app.Xsol(:,3);
            
                if any(isnan(x)) || any(isnan(theta))
                    error('Xsol contains NaNs');
                end
            
                l = 1;
                Nplus1 = height(x);
            
                pend_x = x - l*sin(theta);
                pend_y = l*cos(theta);
            
                ax = app.Ex_UIAxes;
                cla(ax)
                hold(ax,'on')
                axis(ax,'equal')
            
                xlim(ax,[min(x)-l-1 , max(x)+l+1])
                ylim(ax,[-l-0.5 , l+0.5])
            
                plot(ax,[min(x)-l-2 max(x)+l+2],[0 0],'k','LineWidth',2)
            
                cart_width = 0.4;
                cart_height = 0.2;
            
                cart = rectangle('Parent', ax, ...
                    'Position',[x(1)-cart_width/2,-cart_height/2,cart_width,cart_height],...
                    'FaceColor',[0.2 0.6 0.8]);
            
                rod = plot(ax,[x(1) pend_x(1)],[0 pend_y(1)],'k','LineWidth',2);
                bob = plot(ax,pend_x(1),pend_y(1),'ro','MarkerSize',10,'MarkerFaceColor','r');
            
                drawnow
            
                frames = repmat(struct('cdata',[],'colormap',[]), Nplus1, 1);
            
                for i = 1:Nplus1
            
                    if ~isvalid(cart) || ~isvalid(rod) || ~isvalid(bob)
                        break;
                    end
            
                    set(cart,'Position',[x(i)-cart_width/2,-cart_height/2,cart_width,cart_height]);
                    set(rod,'XData',[x(i) pend_x(i)],'YData',[0 pend_y(i)]);
                    set(bob,'XData',pend_x(i),'YData',pend_y(i));
            
                    drawnow limitrate
            
                    %frames(i) = getframe(app.UIFigure);
                    pause(.1);
                end
            
                app.Ex_Frames = frames;
            catch ME
                disp(ME)
                disp(ME.stack)
                uialert(app.UIFigure, ME.message, 'Animation Error');
            end
        end
        % function replayAnimation(app)
        % 
        %     if isempty(app.Ex_Frames)
        %         uialert(app.UIFigure,'Run animation first','Error');
        %         return
        %     end
        % 
        %     movie(app.Ex_UIAxes, app.Ex_Frames, 1, 30);
        % 
        % end

    %----------------------------
    % - Validation Check Case -
    %-----------------------------
        function runCheckCase(app)
            try
                app.Xsol;
                app.Usol;
            catch
                uialert(app.UIFigure,'Run NMPC first','Error');
                return
            end
            errTolerance = 1e-5;
            err = norm(app.Xsol - app.Xexample) + norm(app.Usol - app.Uexample);
            if err>errTolerance
                %Pass = false;
                color = [1 0 0];
                resultText = sprintf('FAIL | Error = %.6f',err);
            else
                %Pass = true;
                color = [0 1 0];
                resultText = sprintf('PASS | Error = %.6f',err);
            end

            app.Ex_ValidationLabel.Text = resultText;
            app.Ex_ValidationLabel.FontColor = color;
        end
    end

    % =====================================================================
    % STARTUP / SHUTDOWN
    % =====================================================================
    methods (Access = public)

        function app = TrajectoryOptimizerApp_1
            createComponents(app);

            % Load example data AFTER UI is fully built
            data = load('example_Data.mat');
            app.Xexample = data.example_Data.X;
            app.Uexample = data.example_Data.U;
        
            % Register last
            registerApp(app, app.UIFigure);
        
            if nargout == 0
                clear app
            end
        end

        function delete(app)
            delete(app.UIFigure)
        end

        % -------- RUN NMPC --------
        function onRunNMPC(app)
        
            app.Comp_StatusLabel.Text = 'Running NMPC...';
            app.Comp_TextArea.Value = {'Starting solver...'};
        
            try
                prob = evalin('base','TrajOptProblem');
            catch
                app.Comp_StatusLabel.Text = 'Error: Export problem first.';
                return
            end
        
            app.ProgressQueue = parallel.pool.DataQueue;
            afterEach(app.ProgressQueue, @(msg) app.appendLog(msg));
        
            f = parfeval(@multipleNMPC, 2, prob, app.ProgressQueue);
            app.NMPCFuture = f;
        
            t = timer;
            t.ExecutionMode = 'fixedSpacing';
            t.Period = 0.5;
            t.TimerFcn = @(~,~) app.checkNMPC(t);
            start(t);
        end
        
        % -------- CHECK COMPLETION --------
        function checkNMPC(app, t)

            if isempty(app) || ~isvalid(app) || ~isvalid(app.UIFigure)
                stop(t); delete(t);
                return;
            end
        
            f = app.NMPCFuture;
        
            if strcmp(f.State,'finished')
        
                stop(t); delete(t);
        
                [U, X] = fetchOutputs(f);
                % X is (N+1)xn and U is Nxm
                app.Comp_StatusLabel.Text = 'Completed';
                app.Xsol=X; app.Usol=U;
        
                N = size(X,1)-1;
                prob = evalin('base','TrajOptProblem');
                tvecState = linspace(0, prob.T_final, N+1);
                tvec = linspace(0,prob.T_final*(1-1/N),N);
                
                cla(app.Comp_UIAxes_X)
                plot(app.Comp_UIAxes_X, tvecState, X)
                title(app.Comp_UIAxes_X,'States')
                xlabel(app.Comp_UIAxes_X,'Time [s]')
                stateLabels = arrayfun(@(i) sprintf('x%d', i), 1:size(X,2), 'UniformOutput', false);
                legend(app.Comp_UIAxes_X, 'off')
                legend(app.Comp_UIAxes_X, stateLabels, 'Location','best')
        
                cla(app.Comp_UIAxes_U)
                stairs(app.Comp_UIAxes_U, tvec, U)
                title(app.Comp_UIAxes_U,'Inputs')
                xlabel(app.Comp_UIAxes_U,'Time [s]')
                uLabels = arrayfun(@(i) sprintf('u%d',i), 1:size(U,2), 'UniformOutput', false);
                legend(app.Comp_UIAxes_U,'off')
                legend(app.Comp_UIAxes_U, uLabels,'Location','best')
        
                app.appendLog('NMPC finished.');
        
            elseif strcmp(f.State,'failed')
        
                stop(t); delete(t);
                app.Comp_StatusLabel.Text = 'Solver failed.';
            end
        end
        
        % -------- APPEND LOG --------
        function appendLog(app, msg)
        
            if isempty(app.Comp_TextArea.Value)
                app.Comp_TextArea.Value = {msg};
            else
                app.Comp_TextArea.Value{end+1} = msg;
            end
        
            drawnow limitrate
        end

    end

end

function [Usol, Xsol] = multipleNMPC(prob, q)
    try
        N = prob.N;
        n = prob.n;          % number of states
        m = prob.m;          % number of controls
        X0 = prob.x0;

        % Dynamics setup
        FullEOM = prob.eq_cells;
        [FullEOM_sym, stateVec, uVec, constVec] = makeEOMsymbolic(FullEOM, n, m, prob.const_names);
        FullEOM_sym = subs(FullEOM_sym, constVec, str2double(prob.const_vals));

        if strcmp(prob.timeType, 'Continuous')
            [symDiscreteAk, symDiscreteBk, fullEOMdiscrete] = discreteLinearization(FullEOM_sym, stateVec, uVec, 'ForwardEuler', prob.T_final/N);
        else
            fullEOMdiscrete = FullEOM_sym;
            symDiscreteAk = jacobian(fullEOMdiscrete, stateVec);
            symDiscreteBk = jacobian(fullEOMdiscrete, uVec);
        end

        eval_fullEOMdiscrete = matlabFunction(fullEOMdiscrete, 'Vars', {stateVec, uVec});
        eval_discreteAk = matlabFunction(symDiscreteAk, 'Vars', {stateVec, uVec});
        eval_discreteBk = matlabFunction(symDiscreteBk, 'Vars', {stateVec, uVec});

        % Objective
        Q = zeros(n,n);
        if prob.useQ
            for i = 1:n
                Q(i,i) = prob.Q_diag(i,1);
            end
        end
        if prob.useQf
            Qf = zeros(n,n);
            for i = 1:n
                Qf(i,i) = prob.Qf_diag(i,1);
            end
        else
            Qf = 10*Q;
        end
        R = zeros(m,m);
        if prob.useR
            for i = 1:m
                R(i,i) = prob.R_diag(i,1);
            end
        end

        [Qh, Rh] = bigWeighting(Q, R, Qf, N);
        H = multipleShootingH(n, Qh, Rh);
        H = H + 1e-2 * eye(size(H));   % regularization

        TerminalState = [];
        if prob.useTerminal
            TerminalState = str2double(prob.x_terminal);
        end
        StateMin = str2double(prob.x_min);
        StateMax = str2double(prob.x_max);
        InputMin = str2double(prob.u_min);
        InputMax = str2double(prob.u_max);
        % === BUILD LINEAR INEQUALITY CONSTRAINTS G z <= h ===
        % Decision vector Z = [x0; x1; ...; xN; u0; ...; u_{N-1}]
        % Total variables: n*(N+1) + m*N

        G = [];
        h = [];

        % 1. State bounds (applied to x1, x2, ..., xN — x0 is fixed by equality)
        if any(isfinite(StateMin)) || any(isfinite(StateMax))
            numStateIneq = sum(isfinite(StateMin)) + sum(isfinite(StateMax));
            if numStateIneq > 0
                G_states = zeros(numStateIneq, n*(N+1) + m*N);
                h_states = zeros(numStateIneq, 1);
                row = 0;

                for k = 1:N   % for each node x1 to xN
                    x_start_col = (k * n) + 1;   % column index of x_k in Z

                    for i = 1:n   % for each state variable
                        % Lower bound:  -x_{k,i}  <= -StateMin(i)   =>   G row = -e_i
                        if isfinite(StateMin(i))
                            row = row + 1;
                            G_states(row, x_start_col + i - 1) = -1;
                            h_states(row,1) = -StateMin(i);
                        end

                        % Upper bound:   x_{k,i}  <=  StateMax(i)
                        if isfinite(StateMax(i))
                            row = row + 1;
                            G_states(row, x_start_col + i - 1) = 1;
                            h_states(row,1) = StateMax(i);
                        end
                    end
                end
                G = [G; G_states];
                h = [h; h_states];
            end
        end

        % 2. Input bounds (applied to u0, u1, ..., u_{N-1})
        if any(isfinite(InputMin)) || any(isfinite(InputMax))
            numInputIneq = sum(isfinite(InputMin)) + sum(isfinite(InputMax));
            if numInputIneq > 0
                G_inputs = zeros(numInputIneq, n*(N+1) + m*N);
                h_inputs = zeros(numInputIneq, 1);
                row = 0;

                for k = 0:N-1   % for each control u_k
                    u_start_col = n*(N+1) + (k * m) + 1;   % column index of u_k in Z

                    for j = 1:m
                        % Lower bound:  -u_{k,j}  <= -InputMin(j)
                        if isfinite(InputMin(j))
                            row = row + 1;
                            G_inputs(row, u_start_col + j - 1) = -1;
                            h_inputs(row) = -InputMin(j);
                        end

                        % Upper bound:   u_{k,j}  <=  InputMax(j)
                        if isfinite(InputMax(j))
                            row = row + 1;
                            G_inputs(row, u_start_col + j - 1) = 1;
                            h_inputs(row) = InputMax(j);
                        end
                    end
                end
                G = [G; G_inputs];
                h = [h; h_inputs];
            end
        end

        % LQR initial guess (your version)
        x_eq = zeros(n,1);
        u_eq = zeros(m,1);
        A0 = eval_discreteAk(x_eq, u_eq);
        B0 = eval_discreteBk(x_eq, u_eq);
        K = DiscreteFiniteHorizonLQR(A0,B0,Q,R,Qf,N);
        send(q, sprintf('LQR gain computed successfully (norm(K) = %.2f)', norm(K,'fro')));
        Zguess = sim_horizonK_on_fd(eval_fullEOMdiscrete, K, N, X0);
        ZguessOld = Zguess;

        % Merit
        mu = 10;
        [CurrentObjCost, sum_gaps, ~] = evaluateNonlinearGaps(Zguess, H, eval_fullEOMdiscrete, X0, prob, N, n, m, TerminalState);
        CurrentMeritCost = CurrentObjCost + mu * sum_gaps;

        Options = optimoptions('quadprog', 'Display', 'off');

        tol_grad = 1e-6;
        tol_eq   = 1e-6;
        tol_step = 1e-6;
        MAXiterations = 1000;

        for numSQPsteps = 1:MAXiterations
            % Linearize
            allA = zeros(n,n,N);
            allB = zeros(n,m,N);
            allCk = zeros(n,N);
            for k = 1:N
                xk = Zguess((k-1)*n + 1 : k*n);
                uk = Zguess(n*(N+1) + (k-1)*m + 1 : n*(N+1) + k*m);
                allA(:,:,k) = eval_discreteAk(xk, uk);
                allB(:,:,k) = eval_discreteBk(xk, uk);
                allCk(:,k) = eval_fullEOMdiscrete(xk, uk) - allA(:,:,k)*xk - allB(:,:,k)*uk;
            end

            [AeqDynamics, beqDynamics] = multipleShootingDynamicConstraints(allA, allB, allCk, X0, N);
            if prob.useTerminal
                [Atc, btc] = multipleTerminalConstraint(TerminalState, N, m);
                Aeq = [AeqDynamics; Atc];
                beq = [beqDynamics; btc];
            else
                Aeq = AeqDynamics;
                beq = beqDynamics;
            end

            % QP
            [Zstar, ~, exitFlag, ~, lambda] = quadprog(H, [], G, h, Aeq, beq, [], [], Zguess, Options);

            if exitFlag <= 0
                if nargin > 1 && ~isempty(q)
                    send(q, sprintf('QP failed (exitFlag = %d)', exitFlag));
                end
                break;
            end

            max_lambda_eq = max(norm(lambda.eqlin, 'inf'));
            if ~isempty(lambda.ineqlin)
                max_lambda_ineq = max(norm(lambda.ineqlin,'inf'));
            else
                max_lambda_ineq = 0;
            end
            max_lambda = max([max_lambda_ineq,max_lambda_eq]);
            mu = max(mu, max_lambda + 0.1);

            % === IMPROVED LINE SEARCH - prevents huge steps ===
            learningRate = 1.0;
            betterMerit = false;
            for ls = 1:20   % more attempts
                Ztrial = Zguess + learningRate * (Zstar - Zguess);
                [trialObj, trial_sum_gaps, trial_max_gap] = evaluateNonlinearGaps(Ztrial, H, eval_fullEOMdiscrete, X0, prob, N, n, m, TerminalState);
                NewMeritCost = trialObj + mu * trial_sum_gaps;

                if NewMeritCost < CurrentMeritCost - 1e-4 * learningRate * abs(CurrentMeritCost)
                    Zguess = Ztrial;
                    CurrentMeritCost = NewMeritCost;
                    eq_res = trial_max_gap;
                    betterMerit = true;
                    break;
                end
                learningRate = learningRate * 0.5;
            end

            if ~betterMerit
                % Take a small safe step instead of rejecting completely
                learningRate = 0.1;   % conservative step when line search fails
                Zguess = Zguess + learningRate * (Zstar - Zguess);
                [~, trial_sum_gaps, trial_max_gap] = evaluateNonlinearGaps(Zguess, H, eval_fullEOMdiscrete, X0, prob, N, n, m, TerminalState);
                CurrentMeritCost = 0.5*Zguess'*H*Zguess + mu * trial_sum_gaps;
                eq_res = trial_max_gap;
                if nargin > 1 && ~isempty(q)
                    send(q, sprintf('Line search failed - taking small safe step (alpha=0.1), mu=%.2e', mu));
                end
            end

            % Convergence
            if isempty(G)
                gradL_res = norm(H*Zguess + Aeq'*lambda.eqlin, 'inf');
            else
                gradL_res = norm(H*Zguess + Aeq'*lambda.eqlin + G'*lambda.ineqlin, 'inf');
            end
            step_res = norm(Zstar - ZguessOld, 'inf');   % use attempted step size
            ZguessOld = Zguess;

            if nargin > 1 && ~isempty(q)
                if mod(numSQPsteps,10)==0 || numSQPsteps <= 5
                    send(q, sprintf('SQP %d: opt->%.2f eq->%.2f delta->%.2f | abs_grad=%.2e abs_eq=%.2e', ...
                        numSQPsteps, gradL_res/tol_grad, eq_res/tol_eq, step_res/tol_step, gradL_res, eq_res));
                end
            end

            if gradL_res < tol_grad && eq_res < tol_eq && step_res < tol_step
                if nargin > 1 && ~isempty(q)
                    send(q, 'Converged successfully!');
                end
                break;
            end
        end

        % Extract results
        Usol = reshape(Zguess(n*(N+1)+1:end), m, N)';
        Xsol = reshape(Zguess(1:n*(N+1)), n, N+1)';

    catch ME
        if nargin > 1 && ~isempty(q)
            send(q, sprintf('ERROR in multipleNMPC: %s', ME.message));
        end
        disp(ME.message);
        disp(ME.stack);
    end
end
%% 
function [objCost, sum_gaps, max_gap] = evaluateNonlinearGaps(Z, H, eval_fullEOMdiscrete, X0, prob, N, NumStateVars, NumControlVars, TerminalState)
    objCost = 0.5 * Z' * H * Z;
    X = Z(1:NumStateVars*(N+1));
    U = Z(NumStateVars*(N+1)+1:end);
    sum_gaps = 0;
    max_gap = 0;
    gap0 = X(1:NumStateVars) - X0;
    sum_gaps = sum_gaps + sum(abs(gap0));
    max_gap = max(max_gap, max(abs(gap0)));
    for k = 1:N
        x_k = X((k-1)*NumStateVars+1 : k*NumStateVars);
        u_k = U((k-1)*NumControlVars+1 : k*NumControlVars);
        x_kp1 = X(k*NumStateVars+1 : (k+1)*NumStateVars);
        gap_k = eval_fullEOMdiscrete(x_k, u_k) - x_kp1;
        sum_gaps = sum_gaps + sum(abs(gap_k));
        max_gap = max(max_gap, max(abs(gap_k)));
    end
    if prob.useTerminal
        gapT = TerminalState - X(end-NumStateVars+1:end);
        sum_gaps = sum_gaps + sum(abs(gapT));
        max_gap = max(max_gap, max(abs(gapT)));
    end
end
%%
function [f_sym, xvec, uvec, constvec] = makeEOMsymbolic(eq_cells, numstatevars, numcontrolvars, const_cells)
    % makeEOMsymbolic makes eq_cells symbolic.
    % 
    % Inputs:
    %   eq_cells       - nx1 cell array of chars or string array containing EOMs
    %   numstatevars   - Integer number of state variables (x)
    %   numcontrolvars - Integer number of control variables (u)
    %
    % Outputs:
    %   f_sym - The symbolic column vector of the dynamics
    %   xvec  - Symbolic state vector [x1; x2; ...]
    %   uvec  - Symbolic control vector [u1; u2; ...]

    % 1. Create symbolic state and control vectors
    xvec = sym('x', [numstatevars, 1]);
    uvec = sym('u', [numcontrolvars, 1]);
    
    % 2. Create the symbolic constants vector
    % The sym() function can take a cell array of chars/strings and 
    % instantly convert it into a vector of symbolic variables.
    if nargin > 3 && ~isempty(const_cells)
        constvec = sym(const_cells);
        constvec = constvec(:); % Ensure it's a column vector
    else
        constvec = sym([]); % Empty if no constants provided
    end

    % 3. Convert the cell array of strings into symbolic expressions
    % str2sym will automatically match 'm', 'g', etc. to the variables in constvec
    f_sym = str2sym(eq_cells);
    f_sym = f_sym(:);

end
%%
function [symDiscreteAk, symDiscreteBk,f_discrete] = discreteLinearization(fContinuous_sym, xVec, uVec, IntegratorType, dt)
    % numericalLinearization Computes the discrete-time numerical Jacobians
    %
    % Inputs:
    %   fContinuous_sym - Symbolically defined vector expression for continuous dynamics (xdot = f(x,u))
    %   xVec            - Symbolic vector with the names of each state variable
    %   uVec            - Symbolic vector with the names of each control variable
    %   Xbar            - Numeric column vector defining the state linearization point
    %   Ubar            - Numeric column vector defining the control linearization point
    %   IntegratorType  - 'ForwardEuler', 'RK2', or 'RK4'
    %   dt              - Numeric time step for the discretization
    %
    % Outputs:
    %   symDiscreteAk      - symbolic discrete-time state Jacobian (A matrix)
    %   symDiscreteBk      - symbolic discrete-time input Jacobian (B matrix)
    
    % 1. Formulate the explicit discrete-time nonlinear dynamics symbolically
    % x_{k+1} = f_discrete(x_k, u_k)
    switch IntegratorType
        case 'ForwardEuler'
            f_discrete = xVec + fContinuous_sym * dt;
            
        case 'RK2' % Also known as Midpoint or Heun's method
            k1 = fContinuous_sym;
            
            % Evaluate continuous dynamics at (x_k + k1*dt/2)
            k2 = subs(fContinuous_sym, xVec, xVec + k1 * (dt / 2));
            
            f_discrete = xVec + k2 * dt;
            
        case 'RK4' % Standard 4th Order Runge-Kutta
            k1 = fContinuous_sym;
            k2 = subs(fContinuous_sym, xVec, xVec + k1 * (dt / 2));
            k3 = subs(fContinuous_sym, xVec, xVec + k2 * (dt / 2));
            k4 = subs(fContinuous_sym, xVec, xVec + k3 * dt);
            
            f_discrete = xVec + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
            
        otherwise
            error('Invalid IntegratorType. Choose ''ForwardEuler'', ''RK2'', or ''RK4''.');
    end
    
    % 2. Compute the exact symbolic Jacobians of the DISCRETE dynamics
    symDiscreteAk = jacobian(f_discrete, xVec);
    symDiscreteBk = jacobian(f_discrete, uVec);
end
%%
function [Qh,Rh] = bigWeighting(Q,R,Qf,N)
%%%%% Notes %%%%%%
% bigWeighting concatenates weighting matrices over the horizon
% for an objective J=X'QhX+U'RhU using single shooting, REQUIRES Q

%%%% Inputs %%%%%
% Q is nxn state error weighting matrix applied to x_(k+1):x_(k+N-1)
% R is mxm control effort weighting matrix applied through horizon u_k:u_(k+N-1)
% Qf is nxn terminal state error weighting matrix applied to x_(k+N)
% N is the scalar horizon

%%%% Outputs %%%%%%
% Qh is (N*n)x(N*n) the concatenated state weighting matrix over trajectory
% Rh is (N*m)x(N*m) the concatenated control effort weighting matrix over trajectory

n = size(Q,1); % number of state variables
m = size(R,1); % number of inputs
Qh = zeros(N*n,N*n);
Rh = zeros(N*m,N*m);
for r = 1:N
    for c = 1:N
        if c==r
            Rh((r-1)*m+1:r*m,(c-1)*m+1:c*m) = R;
            if c == N
                Qh((r-1)*n+1:r*n,(c-1)*n+1:c*n) = Qf;
            else
                Qh((r-1)*n+1:r*n,(c-1)*n+1:c*n) = Q;
            end
        end
    end
end
end
%%
function H = multipleShootingH(NumStateVars, Qh, Rh)
    % MULTIPLESHOOTINGH Creates the Hessian matrix for Multiple Shooting MPC
    %
    % Inputs:
    % NumStateVars - (n) Number of state variables
    % Qh           - (n*N x n*N) Concatenated state weighting matrix (x1 to xN)
    % Rh           - (m*N x m*N) Concatenated control weighting matrix
    % Q            - (unused - kept for backward compatibility)
    %
    % Output:
    % H - The large block-diagonal Hessian matrix
    %     [ 0     0    0 ]
    %     [ 0    Qh    0 ]
    %     [ 0     0   Rh ]
    H_initial = zeros(NumStateVars, NumStateVars);  % x0 is fixed -> no cost on x0
    H = 2 * blkdiag(H_initial, Qh, Rh);
    % Optional: sparse for large N (N=50 is still tiny)
    % H = sparse(H);
end
%%
function K = DiscreteFiniteHorizonLQR(A,B,Q,R,Qf,N)
%Inputs:
% A and B contain the already linearized and discretized
% state matrices
% Dimensions: A is nxn, B is nxm, Q is nxn, R is mxm, Qf is nxn, N is scalar

%Output:
% K is a gain matrix with each row corresponding to a control input, and
% each column corresponding to a state variable, with the 3rd dimension representing time step. 
% Dimensions: K is m x NumStates x N
% The output: K(:,:,1) is the gain matrix at time 0, and K(:,:,end) for control at time Ts*(N-1)

P_now = Qf; %define terminal optimal weighting matrix contraint
%avoid unnecessary repeated computation by defining A',B'
Atranspose = A';
Btranspose = B';
NumStates = height(A);
NumInputs = height(R);
K = zeros(NumInputs,NumStates);
for i=1:N %Apply backwards Riccati Recursion repeatedly until P_0
    K(:,:,i) = (R+Btranspose*P_now*B)\Btranspose*P_now*A;
    P_before = Q + Atranspose*P_now*A-(Atranspose*P_now*B)*K(:,:,i);
    P_now = P_before;
end
K = flip(K,3);
end
%%
function Zguess = sim_horizonK_on_fd(eval_fullEOMdiscrete,K,N,X0)
    % SIM_U_ON_FK Predicts the state trajectory given a control sequence
    %
    % Inputs:
    %   eval_fullEOMdiscrete - Function handle for discrete dynamics x(k+1) = f(x,u)
    %   K                    - mxnxN proportial gain matrix for initial control guess
    %   N                    - Scalar horizon length
    %   X0                   - Initial state column vector [n x 1]
    %
    % Output:
    % Zguess containing [Xguess;Uguess]
    %   Xguess               - Concatenated state vector [x0; x1; ...; xN] 
    %                          Size: (n * (N+1)) x 1
    
    % Determine system dimensions
    m = size(K,1);
    n = size(K,2);
    
    % Preallocate the full state trajectory vector
    Xguess = zeros(n * (N + 1), 1);
    Uguess = zeros(m*N,1);
    
    % Set the first state (x0) and control u0
    current_X = X0;
    Xguess(1:n,1) = current_X;
    current_U = -K(:,:,1)*X0;
    Uguess(1:m,1) = current_U;
    
    % Roll the dynamics forward
    for k = 1:N
        % Step the dynamics forward: x(k+1) = f(current_x, current_u)
        current_X = eval_fullEOMdiscrete(current_X, current_U);
        
        % Store the resulting state in the trajectory
        x_next_idx = k*n + 1 : (k+1)*n;
        Xguess(x_next_idx) = current_X;
        
        %find next U but don't need u_{k+N}, only up to u_{k+N-1}
        if k<N
            currentK = K(:,:,k+1);
            current_U = -currentK*current_X;
            %store current U 
            Uguess(k*m+1:(k+1)*m,1) = current_U;
        end 
    end
    
    % Convert to double in case the function handle returns symbolic results
    Xguess = double(Xguess);
    Uguess = double(Uguess);
    Zguess = [Xguess;Uguess];
end
%%
function [Aeq, beq] = multipleShootingDynamicConstraints(allA, allB, allCk, xinitial, N)
    % Inputs:
    % allA    - (n x n x N) 3D array of linearizations
    % allB    - (n x m x N) 3D array of linearizations
    % allCk   - (n x N) Constant offset terms from linearization
    % xinitial - (n x 1) The current measured state of the plant
    % N       - Scalar horizon length
    
    n = size(allA, 1);
    m = size(allB, 2);
    
    % Decision vector Z is [x0; x1; ...; xN; u0; ...; uN-1]
    % Total variables: n*(N+1) + m*N
    numVars = n*(N+1) + m*N;
    % Total equations: n*(N+1)  (One for x0 constraint, N for dynamics)
    numEqs = n*(N+1);
    
    Aeq = zeros(numEqs, numVars);
    beq = zeros(numEqs, 1);
    
    % 1. Initial State Constraint: x0 = xinitial
    % Corresponds to: [I 0 0 ... 0] * Z = xinitial
    Aeq(1:n, 1:n) = eye(n);
    beq(1:n) = xinitial;
    
    % 2. Dynamics Constraints: -A_k*x_k + I*x_{k+1} - B_k*u_k = c_k
    for k = 1:N
        rowIdx = (k*n) + 1 : (k+1)*n;
        
        % Indices for x_k and x_{k+1}
        xk_colIdx = ((k-1)*n) + 1 : k*n;
        xnext_colIdx = (k*n) + 1 : (k+1)*n;
        
        % Indices for u_k
        % Note: u_k starts after all (N+1) states
        uk_colIdx = (n*(N+1)) + (k-1)*m + 1 : (n*(N+1)) + k*m;
        
        % Fill Aeq
        Aeq(rowIdx, xk_colIdx) = -allA(:,:,k);      % -Ak
        Aeq(rowIdx, xnext_colIdx) = eye(n);         %  I
        Aeq(rowIdx, uk_colIdx) = -allB(:,:,k);      % -Bk
        
        % Fill beq
        beq(rowIdx) = allCk(:, k);
    end
end
%%
function [Atc,btc] = multipleTerminalConstraint(TerminalState,N,m)
% Inputs
% TerminalState is nx1
% N is horizon length
% m is number of control variables

%Outputs
%Atc = equality A matrix in QP constraint formulation in multiple shooting
%btc is simply the terminal state

btc = TerminalState;

n = size(TerminalState,1); %num state variables
Atc = zeros(n,(N+1)*n+N*m);
I = eye(n); %identify matrix so X_N=X_f
Atc(:,N*n+1:(N+1)*n) = I;
end