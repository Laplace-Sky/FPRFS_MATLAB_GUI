

function varargout = FPRFS_GUI_multiboards(varargin)
%FPRFS_GUI_multiboards MATLAB code file for FPRFS_GUI_multiboards.fig
%      FPRFS_GUI_multiboards, by itself, creates a new FPRFS_GUI_multiboards or raises the existing
%      singleton*.
%
%      H = FPRFS_GUI_multiboards returns the handle to a new FPRFS_GUI_multiboards or the handle to
%      the existing singleton*.
%
%      FPRFS_GUI_multiboards('Property','Value',...) creates a new FPRFS_GUI_multiboards using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to FPRFS_GUI_multiboards_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      FPRFS_GUI_multiboards('CALLBACK') and FPRFS_GUI_multiboards('CALLBACK',hObject,...) call the
%      local function named CALLBACK in FPRFS_GUI_multiboards.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help FPRFS_GUI_multiboards

% Last Modified by GUIDE v2.5 05-Feb-2023 01:22:44

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @FPRFS_GUI_multiboards_OpeningFcn, ...
                   'gui_OutputFcn',  @FPRFS_GUI_multiboards_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
   gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before FPRFS_GUI_multiboards is made visible.
function FPRFS_GUI_multiboards_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)
    global RF_pattern;
    global board_pattern;
    
    RF_pattern = ['0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0'];
              
    board_pattern = {};  %this is to define board_pattern to be a cell array type to get rid of "Conversion to double from cell is not possible." error
    board_pattern(1:4) = {['0' '0' '0' '0' '0' '0' '0' '0';...
                           '0' '0' '0' '0' '0' '0' '0' '0';...
                           '0' '0' '0' '0' '0' '0' '0' '0';...
                           '0' '0' '0' '0' '0' '0' '0' '0';...
                           '0' '0' '0' '0' '0' '0' '0' '0';...
                           '0' '0' '0' '0' '0' '0' '0' '0';...
                           '0' '0' '0' '0' '0' '0' '0' '0';...
                           '0' '0' '0' '0' '0' '0' '0' '0']};
              
    global timeout;
    global t_delay;
    global len;
    timeout = 5;
    t_delay = str2double(get(handles.time_out, 'string'));
    len = str2double(get(handles.num_sweep_point, 'string'));
               
    global max_board_num;
    global region_num;
    global data_num;
    global stub_num;
    max_board_num = 1;
    region_num = 8;
    data_num = max_board_num * region_num;
    stub_num = data_num * 60;
    

    global application_type_dec;
    application_type_dec = 1;
    global patch_type_dec;
    patch_type_dec = 1;
    
    global multifreq_sweep
    multifreq_sweep = get(handles.enable_multifreq, 'value');
    global marker_recover;
    marker_recover = get(handles.marker_to_recover, 'value');
    global num_target_freq;
    num_target_freq = get(handles.number_freq, 'value');
    global single_target_freq;
    single_target_freq = str2double(get(handles.freq_target1, 'string')) * 1e9;
    global top_num_save;
    top_num_save = get(handles.top_number_save, 'value');
    handles.enable_weight.Enable = 'on';
    if handles.enable_weight.Enable == 'on'
        handles.freq_target2.Enable = 'on';
        handles.freq_target3.Enable = 'on';
        handles.freq_target4.Enable = 'on';
        handles.freq_target5.Enable = 'on';
        handles.freq_target6.Enable = 'on';
        handles.freq_target7.Enable = 'on';
        handles.freq_target8.Enable = 'on';
    else
        handles.freq_target2.Enable = 'off';
        handles.freq_target3.Enable = 'off';
        handles.freq_target4.Enable = 'off';
        handles.freq_target5.Enable = 'off';
        handles.freq_target6.Enable = 'off';
        handles.freq_target7.Enable = 'off';
        handles.freq_target8.Enable = 'off';
    end
    handles.weight1.Enable = 'off';
    handles.weight2.Enable = 'off';
    handles.weight3.Enable = 'off';
    handles.weight4.Enable = 'off';
    handles.weight5.Enable = 'off';
    handles.weight6.Enable = 'off';
    handles.weight7.Enable = 'off';
    handles.weight8.Enable = 'off';
    handles.marker_to_recover.Enable = 'on';
    handles.number_freq.Enable = 'on';
    
    global S_opti_min_max_selection
    S_opti_min_max_selection = 1;
    
    global application_for_opti
    handles.application_for_opti.Enable = 'off';
    
    global num_board_selection;
    global antenna_board_selection;
    global IMN_board_selection;
    global filter_board_selection;
    num_board_selection = 1;
    antenna_board_selection = 0;
    IMN_board_selection = 0;
    filter_board_selection = 0;
    handles.current_board_selection.Enable = 'off';
    handles.antenna_board_selection.Enable = 'off';
    handles.IMN_board_selection.Enable = 'off';
    handles.filter_board_selection.Enable = 'off';
    
    global num_board_selection_IP;
    global current_board_selection_IP;
    num_board_selection_IP = 1;
    current_board_selection_IP = 1;
    if get(handles.configure_method, 'value') == 1 % wifi
        handles.ESP32_COM.Enable = 'off';
        
        handles.IP_address.Enable = 'on';
        handles.IP_address_2.Enable = 'on';
    else % serial
        handles.ESP32_COM.Enable = 'on';
        
        handles.IP_address.Enable = 'off';
        handles.IP_address_2.Enable = 'off';
    end
    handles.current_board_selection_IP.Enable = 'off';
    handles.IP_address_2.Enable = 'off';
    handles.ConfigBoth.Enable = 'off';
    
    global view_sweeping;
    global view_sweeping_interval;
    view_sweeping = get(handles.view_sweeping, 'value');

    set(handles.feedback_source, 'value', 1);

    global enable_selfadaptive;
    enable_selfadaptive = 0;
    if get(handles.feedback_source, 'value') == 3
        handles.enable_self_adaptive.Enable = 'on';
        handles.self_adaptive_tolerance.Enable = 'on';
        handles.self_adapt_base.Enable = 'on';
        handles.rfmeter_COM_port.Enable = 'on';
        handles.check_VDC.Enable = 'on';
        handles.initialize_RFmeter.Enable = 'on';


        handles.view_sweeping.Enable = 'on';
        handles.view_sweep_interval.Enable = 'on';
        
        handles.enable_multifreq.Enable = 'off';
        handles.enable_weight.Enable = 'off';
        handles.marker_to_recover.Enable = 'off';
        handles.number_freq.Enable = 'off';
        handles.top_number_save.Enable = 'off';
        handles.num_localmin.Enable = 'off';
        handles.smooth_index.Enable = 'off';

        
        set(handles.save_opti_pattern,'value',0);
        set(handles.save_sweep_pattern_file,'value',0);
        set(handles.save_sweep_S_file,'value',0);
        set(handles.save_opti_S,'value',0);
        handles.save_sweep_S_file.Enable = 'off';
        handles.save_sweep_pattern_file.Enable = 'off';
        handles.save_opti_S.Enable = 'off';
        handles.save_opti_pattern.Enable = 'off';
        handles.save_sweep_filename.Enable = 'off';
        
        
        handles.freq_target1.Enable = 'off';
        handles.freq_target2.Enable = 'off';
        handles.freq_target3.Enable = 'off';
        handles.freq_target4.Enable = 'off';
        handles.freq_target5.Enable = 'off';
        handles.freq_target6.Enable = 'off';
        handles.freq_target7.Enable = 'off';
        handles.freq_target8.Enable = 'off';
        
        handles.weight1.Enable = 'off';
        handles.weight2.Enable = 'off';
        handles.weight3.Enable = 'off';
        handles.weight4.Enable = 'off';
        handles.weight5.Enable = 'off';
        handles.weight6.Enable = 'off';
        handles.weight7.Enable = 'off';
        handles.weight8.Enable = 'off';
        
        handles.enable_other_filterload.Enable = 'off';
        handles.directly_coupled.Enable = 'off';
        handles.filter_load_name.Enable = 'off';
    else
        
        handles.rfmeter_COM_port.Enable = 'off';
        handles.check_VDC.Enable = 'off';
        handles.initialize_RFmeter.Enable = 'off';
    end

    view_sweeping_interval = str2double(get(handles.view_sweep_interval, 'string'));
    
    global enable_other_filterload;
    enable_other_filterload = get(handles.enable_other_filterload, 'value');
    handles.filter_load_name.Enable = 'off';
    
    global directly_coupled;
    directly_coupled = get(handles.directly_coupled, 'value');
    
    
    global s_ESP32;
    global ESP32_configured;
    ESP32_configured = 0;
    global ESP32_2_configured;
    ESP32_2_configured = 0;
    
    global s_MCU_DC;
    %s_MCU_DC = get_serial_port_rfmeter(hObject, eventdata, handles);
    global RFmeter_initialized;
    RFmeter_initialized = 0;
    global rfmeter_debug;
    rfmeter_debug = 0;

    global s_nanoVNA;
    global nanoVNA_initialized;
    nanoVNA_initialized = 0;
    
    %self adaptive
    global selfadapt_error;
    selfadapt_error = 1;
    global selfadapt_base;
    selfadapt_base = get(handles.self_adapt_base, 'value');
    global hide_sweeping_process;
    hide_sweeping_process = get(handles.hide_sweeping, 'value');
    global sweeping_inprogress;
    sweeping_inprogress = 0;
    
    if str2num(get(handles.num_sweep_point, 'string')) == 11
        set(handles.enable_multifreq, 'value', 0);
        multifreq_sweep = 0;
    else
        set(handles.enable_multifreq, 'value', 1);
        multifreq_sweep = 1;
    end
    
    global ADC_extremum;
    global nanoVNA_extremum;
    ADC_extremum = 0;
    nanoVNA_extremum = 0;
    
    global w_window;
    global h_window;
    global linecolor;
    global linewidth;
    linecolor = [0 0 0];
    linewidth = 2;
    w_window = 1200;
    h_window = 600;
  

% Choose default command line output for FPRFS_GUI_multiboards
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes FPRFS_GUI_multiboards wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = FPRFS_GUI_multiboards_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

    

function [serial_port] = get_serial_port_rfmeter(hObject, eventdata, handles)
    %% start the serial port for nanoVNA

    s_port = get(handles.rfmeter_COM_port, 'string');
    baudrate = 115200;

%     %make sure the serial port is available
%     if(~isempty(instrfind))        
%         fclose(instrfind);
%         delete(instrfind);
%     end
% 
%     serial_port = serial(s_port);
%     rfmeter_port_global = serial_port;
%     set(serial_port, 'DataBits', 8);
%     set(serial_port, 'StopBit', 1);
%     set(serial_port, 'BaudRate', baudrate);

    serial_port = serialport(s_port, baudrate);
    configureTerminator(serial_port, 'CR/LF');

    if(~ismember(s_port, seriallist))
        fprintf('%s port is not available! Check the COM port and retry.\n\n', s_port);
    end
    
    
function [serial_port] = get_serial_port_ESP32(hObject, eventdata, handles)

    s_port = get(handles.ESP32_COM, 'string');
    baudrate = 115200;

    serial_port = serialport(s_port, baudrate);
    %configureTerminator(serial_port, 'CR/LF');
    %fprintf('%s port is configured.\n\n', s_port);

    if(~ismember(s_port, seriallist))
        fprintf('%s port is not available! Check the COM port and retry.\n\n', s_port);
    end
    
    

function [serial_port] = get_serial_port_nanoVNA(hObject, eventdata, handles)
    %% start the serial port for nanoVNA
    s_port = get(handles.COM_port, 'string');
    baudrate = 115200;

%     serial_port = serialport(s_port, baudrate);
    
    %make sure the serial port is available
    if(~isempty(instrfind))        
        fclose(instrfind);
        delete(instrfind);
    end

    serial_port = serial(s_port);
    set(serial_port, 'DataBits', 8);
    set(serial_port, 'StopBit', 1);
    set(serial_port, 'BaudRate', baudrate);

    if(~ismember(s_port, seriallist))
        fprintf('%s port is not available! Check the COM port and retry.\n\n', s_port);
    end
 
    
function send_ESP32_serial(bitstream, hObject, eventdata, handles)
    global ESP32_configured;
    global s_ESP32;
    
    if ~ESP32_configured
        s_ESP32 = get_serial_port_ESP32(hObject, eventdata, handles);
        ESP32_configured = 1;
    end
    
    bitstream
    %flush(s_ESP32, 'output');
    write(s_ESP32, bitstream, 'uint8');
    
%     delete(s_ESP32);
%     ESP32_configured = 0;
    
    pause(str2num(get(handles.pattern_interval, 'string'))*0.001);
    
    
    
function send_ESP32_port(bitstream, timeout, hObject, eventdata, handles)
    %% start the WiFi port for configuration  
    global ESP32_wifi;
    global port;
    global ESP32_configured;
    global address;
    
    if ~ESP32_configured
        address = get(handles.IP_address, 'string');
        port = 80;
        ESP32_wifi = tcpclient(address, port, "Timeout", timeout, "ConnectTimeout", timeout);   
        ESP32_configured = 1;
    end
    %ESP32_wifi = tcpclient(address, port, "Timeout", timeout, "ConnectTimeout", timeout); 
    write(ESP32_wifi, bitstream, 'uint8');
    %pause(0.3);  % wait for ESP32 terminal to received and shiftout before
    %reading the VDC from RF meter
        
function send_ESP32_port_2(bitstream, timeout, hObject, eventdata, handles)
    %% start the WiFi port for configuration  
    global ESP32_wifi;
    global port;
    global ESP32_2_configured;
    global address_2;
    
    if ~ESP32_2_configured
        address_2 = get(handles.IP_address_2, 'string');
        port = 80;
        ESP32_wifi = tcpclient(address_2, port, "Timeout", timeout, "ConnectTimeout", timeout);   
        ESP32_2_configured = 1;
    end
    write(ESP32_wifi, bitstream, 'uint8');

function get_nanoVNA_parameters()
    global S21_50ohm_nanoVNA;
    global insertion_loss_nanoVNA;
    global coupling_nanoVNA;
    global isolation_nanoVNA;
    i = sqrt(-1);
    
    T = csvread(['S21_50ohm', '_R_nanoVNA', '.csv']);  % real part
    S21_50ohm_R_nanoVNA = T(:, 1);
    T = csvread(['S21_50ohm', '_X_nanoVNA', '.csv']);   %img part
    S21_50ohm_X_nanoVNA = T(:, 1);
    S21_50ohm_nanoVNA = S21_50ohm_R_nanoVNA + i .* S21_50ohm_X_nanoVNA;

    T = csvread(['insertion_loss_R_nanoVNA', '.csv']);
    insertion_loss_R_nanoVNA = T(:, 1);
    T = csvread(['insertion_loss_X_nanoVNA', '.csv']);
    insertion_loss_X_nanoVNA = T(:, 1);
    insertion_loss_nanoVNA = insertion_loss_R_nanoVNA + i .* insertion_loss_X_nanoVNA;

    T = csvread(['coupling_R_nanoVNA', '.csv']);
    coupling_R_nanoVNA = T(:, 1);
    T = csvread(['coupling_X_nanoVNA', '.csv']);
    coupling_X_nanoVNA = T(:, 1);
    coupling_nanoVNA = coupling_R_nanoVNA + i .* coupling_X_nanoVNA;
    
    T = csvread(['isolation_R_nanoVNA', '.csv']);
    isolation_R_nanoVNA = T(:, 1);
    T = csvread(['isolation_X_nanoVNA', '.csv']);
    isolation_X_nanoVNA = T(:, 1);
    isolation_nanoVNA = isolation_R_nanoVNA + i .* isolation_X_nanoVNA;
    
    assignin('base', 'S21_50ohm_nanoVNA', S21_50ohm_nanoVNA);
    assignin('base', 'insertion_loss_nanoVNA', insertion_loss_nanoVNA);
    assignin('base', 'coupling_nanoVNA', coupling_nanoVNA);
    assignin('base', 'isolation_nanoVNA', isolation_nanoVNA);
    
    

function [H_nanoVNA] = read_nanoVNA(S_measure, nanoVNA_port, hObject, eventdata, handles)
    global S21_50ohm_nanoVNA;
    global insertion_loss_nanoVNA;
    global coupling_nanoVNA;
    global isolation_nanoVNA;
    
    global single_target_freq;
    
    global enable_other_filterload;
    global filter_load_name;
    
    global nanoVNA_initialized;
    
    if ~nanoVNA_initialized
        nanoVNA_port = get_serial_port_nanoVNA(hObject, eventdata, handles);
        fopen(nanoVNA_port);

        get_nanoVNA_parameters();

        nanoVNA_initialized = 1;
    end
    

    str_S21 = nanoCommand(nanoVNA_port, 'data 1');                %% S21 with DUT connected, real and imaginary parts are in data 1
    str_freq = nanoCommand(nanoVNA_port, 'frequencies');
    
    fclose(nanoVNA_port);
    delete(nanoVNA_port);
    nanoVNA_initialized = 0;
    
    len_new = numel(str_freq);
 
    freq_nanoVNA = zeros(len_new, 1);


    S21_nanoVNA = zeros(len_new, 1);
    re = zeros(len_new, 1);
    im = zeros(len_new, 1);
    for m = 1:len_new
        freq_nanoVNA(m) = str2double(str_freq{m});
        vals = strsplit(str_S21{m});
        re(m) = str2double(vals(1));
        im(m) = str2double(vals(2));
        S21_nanoVNA(m) = complex(re(m), im(m));
    end
    
    if enable_other_filterload
        filter_load_name = get(handles.filter_load_name, 'string');
        S11_load = zeros(len_new, 1);
        T = csvread([filter_load_name, '_R', '.csv'], 3, 0);
        re_load = T(:, 2);
        T = csvread([filter_load_name, '_X', '.csv'], 3, 0);
        im_load = T(:, 2);
        for m = 1:length(re_load)
            S11_load(m) = complex(re_load(m), im_load(m));
        end
        
        assignin('base', 'S11_load', S11_load);
    end

    assignin('base', 'S21_nanoVNA', S21_nanoVNA);
%     assignin('base', 'S21_50ohm_nanoVNA', S21_50ohm_nanoVNA);
%     assignin('base', 'insertion_loss_nanoVNA', insertion_loss_nanoVNA);
%     assignin('base', 'coupling_nanoVNA', coupling_nanoVNA);

    index = round((single_target_freq / 1e9 - 0.1) / ((3 - 0.1) / 200)) + 1;
    if S_measure == 1   %measure S11 for antenna IMN reflected power
        %display('Measuring reflected power...');           
        H_nanoVNA = (S21_nanoVNA(5) - S21_50ohm_nanoVNA(index)) / (insertion_loss_nanoVNA(index) * coupling_nanoVNA(index)); 
    elseif S_measure == 2   %measure S21 for filter power transfered
        %display('Measuring transfered power (50ohm load)...');
        H_nanoVNA = S21_nanoVNA(5) / coupling_nanoVNA(index);
    elseif S_measure == 3    %measure S21 for filter power transfered using measured load
        %display('Measuring transfered power (other load)...');
        H_nanoVNA = S21_nanoVNA(5) / (coupling_nanoVNA(index) + isolation_nanoVNA(index) * insertion_loss_nanoVNA(index) * S11_load(index));
    elseif S_measure == 4
        %display('Measuring transfered power (directly coupled)...');
        H_nanoVNA = S21_nanoVNA(5);
    end

    
    
   
function [H_nanoVNA] = save_S21(S_measure, nanoVNA_port, spectrum_sweep, hObject, eventdata, handles)
    global S21_50ohm_nanoVNA;
    global insertion_loss_nanoVNA;
    global coupling_nanoVNA;
    global isolation_nanoVNA;
    
    global len;
    global freq_nanoVNA;
    
    global single_target_freq;
    
    global enable_other_filterload;
    global filter_load_name;
    
    
    %check the smallest S11 value and save it

    str_S21 = nanoCommand(nanoVNA_port, 'data 1');                %% S21 with DUT connected, real and imaginary parts are in data 1
    str_freq = nanoCommand(nanoVNA_port, 'frequencies');
    
    if spectrum_sweep
        len_new = len;
    else
        len_new = numel(str_freq);
    end
    
    freq_nanoVNA = zeros(len_new, 1);


    S21_nanoVNA = zeros(len_new, 1);
    re = zeros(len_new, 1);
    im = zeros(len_new, 1);
    for m = 1:len_new
        freq_nanoVNA(m) = str2double(str_freq{m});
        vals = strsplit(str_S21{m});
        re(m) = str2double(vals(1));
        im(m) = str2double(vals(2));
        S21_nanoVNA(m) = complex(re(m), im(m));
    end
    
    if enable_other_filterload
        filter_load_name = get(handles.filter_load_name, 'string');
        S11_load = zeros(len_new, 1);
        T = csvread([filter_load_name, '_R', '.csv'], 3, 0);
        re_load = T(:, 2);
        T = csvread([filter_load_name, '_X', '.csv'], 3, 0);
        im_load = T(:, 2);
        for m = 1:length(re_load)
            S11_load(m) = complex(re_load(m), im_load(m));
        end
        
        assignin('base', 'S11_load', S11_load);
    end

    assignin('base', 'S21_nanoVNA', S21_nanoVNA);
%     assignin('base', 'S21_50ohm_nanoVNA', S21_50ohm_nanoVNA);
%     assignin('base', 'insertion_loss_nanoVNA', insertion_loss_nanoVNA);
%     assignin('base', 'coupling_nanoVNA', coupling_nanoVNA);

    if spectrum_sweep
        
        if S_measure == 1   %measure S11 for antenna IMN reflected power
            %display('Measuring reflected power...');
            H_nanoVNA = (S21_nanoVNA - S21_50ohm_nanoVNA) ./ (insertion_loss_nanoVNA .* coupling_nanoVNA); 
        elseif S_measure == 2   %measure S21 for filter power transfered using 50ohm load
            %display('Measuring transfered power (50ohm load)...');
            H_nanoVNA = S21_nanoVNA ./ coupling_nanoVNA;
        elseif S_measure == 3    %measure S21 for filter power transfered using measured load
            %display('Measuring transfered power (other load)...');
            H_nanoVNA = S21_nanoVNA ./ (coupling_nanoVNA + isolation_nanoVNA .* insertion_loss_nanoVNA .* S11_load);
        elseif S_measure == 4
            %display('Measuring transfered power (directly coupled)...');
            H_nanoVNA = S21_nanoVNA;
        end

    else
        index = round((single_target_freq / 1e9 - 0.1) / ((3 - 0.1) / 200)) + 1;
        assignin('base', 'single_target_freq', single_target_freq);
        if S_measure == 1   %measure S11 for antenna IMN reflected power
            %display('Measuring reflected power...');           
            H_nanoVNA = (S21_nanoVNA(5) - S21_50ohm_nanoVNA(index)) / (insertion_loss_nanoVNA(index) * coupling_nanoVNA(index)); 
        elseif S_measure == 2   %measure S21 for filter power transfered
            %display('Measuring transfered power (50ohm load)...');
            H_nanoVNA = S21_nanoVNA(5) / coupling_nanoVNA(index);
        elseif S_measure == 3    %measure S21 for filter power transfered using measured load
            %display('Measuring transfered power (other load)...');
            H_nanoVNA = S21_nanoVNA(5) / (coupling_nanoVNA(index) + isolation_nanoVNA(index) * insertion_loss_nanoVNA(index) * S11_load(index));
        elseif S_measure == 4
            %display('Measuring transfered power (directly coupled)...');
            H_nanoVNA = S21_nanoVNA(5)
        end
    end
    
    
function [H_nanoVNA] = save_S21_raw(nanoVNA_port)
    
    global len;
    global freq_nanoVNA;
    
    %check the smallest S11 value and save it

    str_S21 = nanoCommand(nanoVNA_port, 'data 1');                %% S21 with DUT connected, real and imaginary parts are in data 1
    str_freq = nanoCommand(nanoVNA_port, 'frequencies');
    
    len = numel(str_freq);
    freq_nanoVNA = zeros(len, 1);
    for m = 1:len
        freq_nanoVNA(m) = str2double(str_freq{m});
    end

    S21_nanoVNA = zeros(len, 1);
    re = zeros(len, 1);
    im = zeros(len, 1);
    for m = 1:len
        freq_nanoVNA(m) = str2double(str_freq{m});
        vals = strsplit(str_S21{m});
        re(m) = str2double(vals(1));
        im(m) = str2double(vals(2));
        S21_nanoVNA(m) = complex(re(m), im(m));
    end

    assignin('base', 'S21_nanoVNA', S21_nanoVNA);
%     assignin('base', 'S21_50ohm_nanoVNA', S21_50ohm_nanoVNA);
%     assignin('base', 'insertion_loss_nanoVNA', insertion_loss_nanoVNA);
%     assignin('base', 'coupling_nanoVNA', coupling_nanoVNA);
    H_nanoVNA = S21_nanoVNA;
    
    
function [VDC_rfmeter] = save_VDC_rfmeter(hObject, eventdata, handles)
    global s_MCU_DC;
    global RFmeter_initialized;

    if ~RFmeter_initialized
        fprintf("Initializing the RF meter COM port...\n");
        s_MCU_DC = get_serial_port_rfmeter(hObject, eventdata, handles);
        RFmeter_initialized = 1;
        fprintf("The RF meter COM port is initilialized!\n");
        VDC_rfmeter = -1;
    end
    

%         %original reading scheme (with some problem)
%         flush(s_MCU_DC, 'input');
%         write(s_MCU_DC, '1', 'char');
%         VDC_rfmeter = 4096 - str2num(readline(s_MCU_DC));

    %read_available = 0;

    %fprintf('1: %d     ',s_MCU_DC.NumBytesAvailable);
    flush(s_MCU_DC, 'input');
    %fprintf('2: %d     ',s_MCU_DC.NumBytesAvailable);
    write(s_MCU_DC, '1', 'char');
    %fprintf('3: %d     ',s_MCU_DC.NumBytesAvailable);

    pause(0.05);
%     while (~read_available)
%         if s_MCU_DC.NumBytesAvailable > 0
%             read_available = 1;
%             fprintf('4: %d     ',s_MCU_DC.NumBytesAvailable);
%             VDC_rfmeter = 4096 - str2num(readline(s_MCU_DC));
%             fprintf('5: %d\n',s_MCU_DC.NumBytesAvailable);
%         end
%     end

    %fprintf('4: %d     ',s_MCU_DC.NumBytesAvailable);
    VDC_rfmeter = 4096 - str2num(readline(s_MCU_DC));
    %fprintf('5: %d\n',s_MCU_DC.NumBytesAvailable);
    %fprintf('%d\n', VDC_rfmeter);



    

% --- Executes on button press in check_VDC.
function check_VDC_Callback(hObject, eventdata, handles)
% hObject    handle to check_VDC (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global s_MCU_DC;
    global RFmeter_initialized;
    global enable_selfadaptive;
    global ADC_rfmeter_current;
    
    global ADC_extremum;
    
    if ~RFmeter_initialized
        fprintf("Initializing the RF meter COM port...\n");
        s_MCU_DC = get_serial_port_rfmeter(hObject, eventdata, handles);
        RFmeter_initialized = 1;
        fprintf("The RF meter COM port is initilialized!\n");
    end
    
    %read_available = 0;
    
    fprintf('1: %d     ',s_MCU_DC.NumBytesAvailable);
    flush(s_MCU_DC, 'input');
    fprintf('2: %d     ',s_MCU_DC.NumBytesAvailable);
    write(s_MCU_DC, '1', 'char');
    fprintf('3: %d     ',s_MCU_DC.NumBytesAvailable);
    
    pause(0.05);
%     while (~read_available)
%         if s_MCU_DC.NumBytesAvailable == 6
%             read_available = 1;
%             ADC_rfmeter_current = 4096 - str2num(readline(s_MCU_DC));
%             %s_MCU_DC.NumBytesAvailable
%         end
%     end

    fprintf('4: %d     ',s_MCU_DC.NumBytesAvailable);
    ADC_rfmeter_current = 4096 - str2num(readline(s_MCU_DC))
    ADC_extremum = ADC_rfmeter_current;
    fprintf('5: %d\n',s_MCU_DC.NumBytesAvailable);

        
    if ~enable_selfadaptive
        fprintf("%d\n", ADC_rfmeter_current);
        assignin('base', 'ADC_rfmeter', ADC_rfmeter_current);
    end

    
    
% --- Executes on button press in initialize_RFmeter.
function initialize_RFmeter_Callback(hObject, eventdata, handles)
% hObject    handle to initialize_RFmeter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global s_MCU_DC;
    global RFmeter_initialized;
    
    
    if ~RFmeter_initialized
        fprintf("Initializing the RF meter COM port...\n");
        s_MCU_DC = get_serial_port_rfmeter(hObject, eventdata, handles);
        RFmeter_initialized = 1;
        fprintf("The RF meter COM port is initilialized!\n");
    else
        fprintf("The RF meter COM port is already initilialized.\n");
    end
    


% --- Executes on button press in check_pattern.
function clear_RFmeter_port_Callback(hObject, eventdata, handles)
% hObject    handle to clear_RFmeter_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global s_MCU_DC;
    global RFmeter_initialized;
    global ESP32_configured;
    global ESP32_2_configured;
    global s_ESP32;
    global nanoVNA_initialized;
    global s_nanoVNA;
    
    ESP32_configured = 0;
    ESP32_2_configured = 0;
    
    RFmeter_initialized = 0;
    nanoVNA_initialized = 0;
    
    delete(s_MCU_DC);
    
    delete(s_ESP32);
 
    delete(s_nanoVNA);
    %s = get_serial_port_nanoVNA(hObject, eventdata, handles);
    %fclose(s);
    
    fprintf("COM ports are cleared!\n\n");  
    
    
    
    
  
function [bitstream] = set_pattern_and_config(pattern, ~, eventdata, handles)       % pattern is whatever displayed on the GUI and it is a num array, configured is the whole pattern for all boards and they are char arrays
    global num_board_selection;    
    global region_num;
    global RF_pattern;
    global board_pattern;
    global data_num;
    
    global hide_sweeping_process;
    global sweeping_inprogress;
    
    bitstream = zeros(1, data_num);
    region = cell(num_board_selection, region_num);
    
    %set all the button value accordingly
    %if get(handles.patch_type_selection, 'Value');
    for i = 1:region_num
        for j = 1:7
            %set(handles.(sprintf('togglebutton%d%d', i, j)), 'Value', pattern(i, j));
            if(pattern(i, j))
                if ~(hide_sweeping_process && sweeping_inprogress)
                    set(handles.(sprintf('togglebutton%d%d', i, j)), 'Value', pattern(i, j));
                    set(handles.(sprintf('togglebutton%d%d', i, j)), 'BackgroundColor', 'r');
                end
                RF_pattern(i, j) = '1';
            else
                if ~(hide_sweeping_process && sweeping_inprogress)
                    set(handles.(sprintf('togglebutton%d%d', i, j)), 'Value', pattern(i, j));
                    set(handles.(sprintf('togglebutton%d%d', i, j)), 'BackgroundColor', [0.94 0.94 0.94]);
                end
                RF_pattern(i, j) = '0';
            end
        end
        if(rem(i, 2))
            %set(handles.(sprintf('togglebutton%d%d', i, 8)), 'Value', pattern(i, 8));
            if(pattern(i, 8))
                if ~(hide_sweeping_process && sweeping_inprogress)
                    set(handles.(sprintf('togglebutton%d%d', i, 8)), 'Value', pattern(i, 8));
                    set(handles.(sprintf('togglebutton%d%d', i, 8)), 'BackgroundColor', 'r');
                end
                RF_pattern(i, 8) = '1';
            else
                if ~(hide_sweeping_process && sweeping_inprogress)
                    set(handles.(sprintf('togglebutton%d%d', i, 8)), 'Value', pattern(i, 8));
                    set(handles.(sprintf('togglebutton%d%d', i, 8)), 'BackgroundColor', [0.94 0.94 0.94]);
                end
                RF_pattern(i, 8) = '0';
            end
        end
    end
    
    if num_board_selection == 1
        board_pattern{1} = RF_pattern;
    end
        
    for i = 1:num_board_selection
        bitstream(1) = num_board_selection;  % send the header with number of boards activated
        for j = 1:region_num
            region(i, j) = {strrep(board_pattern{i}(j, :), ' ', '')};
            bitstream((i - 1) * region_num + j + 1) = bin2dec(region(i, j));       % shift all bits to the right by 1 byte to make room for the first byte
        end   
    end
    
    assignin('base', 'region', region); 
    
                 
function pattern_mapping()
    global pattern_array;
    global pattern_mapped;
    
    % pattern mapping
    pattern_array(1, 1) = pattern_mapped(3, 2);
    pattern_array(1, 2) = pattern_mapped(3, 1);
    pattern_array(1, 3) = pattern_mapped(1, 1);
    pattern_array(1, 4) = pattern_mapped(2, 2);
    pattern_array(1, 5) = pattern_mapped(2, 1);
    pattern_array(1, 6) = pattern_mapped(1, 2);
    pattern_array(1, 7) = pattern_mapped(4, 2);
    pattern_array(1, 8) = pattern_mapped(4, 1);

    pattern_array(2, 1) = pattern_mapped(6, 3);
    pattern_array(2, 2) = pattern_mapped(3, 3);
    pattern_array(2, 3) = pattern_mapped(1, 3);
    pattern_array(2, 4) = pattern_mapped(6, 2);
    pattern_array(2, 5) = pattern_mapped(6, 1);
    pattern_array(2, 6) = pattern_mapped(1, 4);
    pattern_array(2, 7) = pattern_mapped(3, 4);

    pattern_array(3, 1) = pattern_mapped(8, 2);
    pattern_array(3, 2) = pattern_mapped(8, 1);
    pattern_array(3, 3) = pattern_mapped(1, 5);
    pattern_array(3, 4) = pattern_mapped(10, 1);
    pattern_array(3, 5) = pattern_mapped(1, 6);
    pattern_array(3, 6) = pattern_mapped(10, 2);
    pattern_array(3, 7) = pattern_mapped(3, 5);
    pattern_array(3, 8) = pattern_mapped(3, 6);

    pattern_array(4, 1) = pattern_mapped(5, 4);
    pattern_array(4, 2) = pattern_mapped(8, 3);
    pattern_array(4, 3) = pattern_mapped(10, 3);
    pattern_array(4, 4) = pattern_mapped(5, 5);
    pattern_array(4, 5) = pattern_mapped(5, 6);
    pattern_array(4, 6) = pattern_mapped(10, 4);
    pattern_array(4, 7) = pattern_mapped(8, 4);

    pattern_array(5, 1) = pattern_mapped(7, 5);
    pattern_array(5, 2) = pattern_mapped(7, 6);
    pattern_array(5, 3) = pattern_mapped(9, 6);
    pattern_array(5, 4) = pattern_mapped(10, 5);
    pattern_array(5, 5) = pattern_mapped(10, 6);
    pattern_array(5, 6) = pattern_mapped(9, 5);
    pattern_array(5, 7) = pattern_mapped(8, 5);
    pattern_array(5, 8) = pattern_mapped(8, 6);

    pattern_array(6, 1) = pattern_mapped(6, 4);
    pattern_array(6, 2) = pattern_mapped(7, 4);
    pattern_array(6, 3) = pattern_mapped(9, 4);
    pattern_array(6, 4) = pattern_mapped(6, 5);
    pattern_array(6, 5) = pattern_mapped(6, 6);
    pattern_array(6, 6) = pattern_mapped(9, 3);
    pattern_array(6, 7) = pattern_mapped(7, 3);

    pattern_array(7, 1) = pattern_mapped(4, 5);
    pattern_array(7, 2) = pattern_mapped(4, 6);
    pattern_array(7, 3) = pattern_mapped(2, 6);
    pattern_array(7, 4) = pattern_mapped(9, 2);
    pattern_array(7, 5) = pattern_mapped(9, 1);
    pattern_array(7, 6) = pattern_mapped(2, 5);
    pattern_array(7, 7) = pattern_mapped(7, 2);
    pattern_array(7, 8) = pattern_mapped(7, 1);

    pattern_array(8, 1) = pattern_mapped(5, 3);
    pattern_array(8, 2) = pattern_mapped(4, 4);
    pattern_array(8, 3) = pattern_mapped(2, 4);
    pattern_array(8, 4) = pattern_mapped(5, 1);
    pattern_array(8, 5) = pattern_mapped(5, 2);
    pattern_array(8, 6) = pattern_mapped(2, 3);
    pattern_array(8, 7) = pattern_mapped(4, 3);
    
    
function recover_pattern(S_param, pattern_num, pattern_saved, H_antenna_nanoVNA_saved, freq_target_round, hObject, eventdata, handles)
    global timeout;
    global freq_nanoVNA;
    global multifreq_sweep;
    
    global ESP32_configured;
    global s_ESP32;
    
    global w_window;
    global h_window;
    global linecolor;
    global linewidth;
    
    
	%% recover the RF surface pattern as IMN
    localmin_index = str2double(get(handles.num_localmin, 'string'));
    smooth_index = str2double(get(handles.smooth_index, 'string'));
    
    %set all the button value accordingly
    bitstream = set_pattern_and_config(pattern_saved, hObject, eventdata, handles);
    if get(handles.configure_method, 'value') == 1
        send_ESP32_port(bitstream, timeout, hObject, eventdata, handles);
    else
        send_ESP32_serial(bitstream, hObject, eventdata, handles);
        delete(s_ESP32);
        ESP32_configured = 0;
    end
    %% end
    
    H_antenna_smooth = smooth(H_antenna_nanoVNA_saved, smooth_index);
    H_antenna_smooth_log = 20 * log10(abs(H_antenna_smooth));
    
    if multifreq_sweep
        S_table = [freq_nanoVNA H_antenna_nanoVNA_saved H_antenna_smooth];
        assignin('base', 'S_param_plot', S_table);
    end
    
%     freq_localmin = nonzeros(freq_nanoVNA .* (islocalmin(H_antenna_smooth_log)));
%     H_antenna_localmin = nonzeros(H_antenna_smooth_log .* (islocalmin(H_antenna_smooth_log)));
%     size_localmin = length(freq_localmin);
%     
%     num_localmin = min(localmin_index, size_localmin);       %setting the number of smallest localmin
%     
%     S11_localmin_table = sortrows([freq_localmin H_antenna_localmin], 2, 'ascend');    %sort the table based on the reflection coefficient
%     S11_localmin_table((num_localmin + 1):size_localmin, :) = [];
%     %assignin('base', 'freq_localmin', freq_localmin);
%     %assignin('base', 'H_antenna_localmin', H_antenna_localmin);
%     assignin('base', 'S11_min', S11_localmin_table);
%     
%     S11_localmin_table = sortrows(S11_localmin_table, 1, 'ascend');    %sort the table based on the frequency
    
    if get(handles.enable_self_adaptive, 'value')
        figure('Name', 'Recovered');
        set(gcf,'position',[[200, 200], w_window, h_window]);
        plot(freq_nanoVNA, 20 * log10(abs(H_antenna_nanoVNA_saved)), 'Color', linecolor, 'LineWidth', linewidth);
        hold on;
        plot(freq_nanoVNA, 20 * log10(abs(H_antenna_smooth)), 'Color', linecolor, 'LineWidth', linewidth, 'Linestyle', ':');

        grid on
        set(gca, 'FontSize', 18);
        title('S-parameter recovered', 'FontSize', 20);
        xlabel('Frequency (GHz)', 'FontSize', 18);
        ylabel('S-paramete (dB)', 'FontSize', 18);
        ylim([-inf 0]);

        x_local = xline(freq_target_round, '--', 'Linewidth', linewidth, 'Fontsize', 15);
        x_local.Label = [num2str(freq_target_round / 1e9), ' GHz'];
        x_local.LabelVerticalAlignment = 'bottom';
        x_local.LabelHorizontalAlignment = 'left';
    end
    
    fprintf('Minimum S11/Maximum S21 observed at %.2fGHz is %.2fdB with the %dth RF surface pattern configured as recovered.\n\n', freq_target_round / 1e9, S_param, pattern_num);
     
    
function recover_pattern_rfmeter(pattern_num, pattern_saved, hObject, eventdata, handles)
    global timeout;
    
    global ESP32_configured;
    global s_ESP32;
    
    %set all the button value accordingly
    bitstream = set_pattern_and_config(pattern_saved, hObject, eventdata, handles);
    
    if get(handles.configure_method, 'value') == 1
        send_ESP32_port(bitstream, timeout, hObject, eventdata, handles);
    else
        send_ESP32_serial(bitstream, hObject, eventdata, handles);
        delete(s_ESP32);
        ESP32_configured = 0;
    end
    %% end
    
    fprintf('Minimum S11/Maximum S21 is observed at the %dth RF surface pattern as recovered.\n\n', pattern_num);    
    
    
function [char_array] = numarray2chararray(num_array)
    char_array = ['0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0'];

    for i = 1:8
        char_array(i, :) = strrep(num2str(num_array(i, :)), ' ', '');
    end

    
function [S_extremum, pattern_num, pattern_saved, H_antenna_nanoVNA_saved] = pattern_config_and_measure(Enable_mapping, read_from_nanoVNA, function_type, S_measure, nanoVNA_port, freq_target_index, ...
          S_extremum, pattern_num, pattern_saved, H_antenna_nanoVNA_saved, pattern_count, hObject, eventdata, handles)            %function_type: 0 for normal; 1 for antenna sweeping; 2 for IMN sweeping; 3 for filter
    global timeout;                                                                                                            %S_measure: 1 for collect reflected power from coupler; 2 for collect transfered power from coupler
    global t_delay;
    global pattern_array;         % pattern_array is a num array for displaying stubs on GUI
    global len
    
    global board_pattern;         % board_pattern is a series of char arrays for constructing bitstream for configuring
    global num_board_selection;
    global antenna_board_selection
    global IMN_board_selection
    global filter_board_selection
    
    global region_num;
    
    global multifreq_sweep;
    global num_target_freq;
    
    global view_sweeping;
    global view_sweeping_interval;
    view_sweeping_interval = str2double(get(handles.view_sweep_interval, 'string'));   
    
    global application_for_opti;
    global S_opti_min_max_selection;
    
    global rfmeter_debug;
    
    global ESP32_configured;
    
    if Enable_mapping
        pattern_mapping();
    end
    
    pattern_array_char = numarray2chararray(pattern_array);
    if num_board_selection == 1
        board_pattern{1} = pattern_array_char;
    else
        if application_for_opti == 2 % antenna
            board_pattern{antenna_board_selection} = pattern_array_char;
        elseif application_for_opti == 3 % IMN
            board_pattern{IMN_board_selection} = pattern_array_char;
        elseif application_for_opti == 4 % filter
            board_pattern{filter_board_selection} = pattern_array_char;
        end
    end
    
    %assignin('base', 'board_pattern', board_pattern);
    
    bitstream = set_pattern_and_config(pattern_array, hObject, eventdata, handles);
        
    if ~view_sweeping
        if rfmeter_debug
            fprintf('\n\nConfiguring pattern No.%d:\n', pattern_count);
        end
        
        if get(handles.configure_method, 'value') == 1
            send_ESP32_port(bitstream, timeout, hObject, eventdata, handles);
        else
            send_ESP32_serial(bitstream, hObject, eventdata, handles);
            fprintf('Device 1 pattern configured through Serial port!\n\n');
        end

        if read_from_nanoVNA == 1
            pause(t_delay);

            if get(handles.save_sweep_S_file,'value') == 1
                %save the S11 response

                str_S11 = nanoCommand(nanoVNA_port, 'data 0');

                for t = 1:len
                    freq_nanoVNA(t) = str2double(str_freq{t});
                    vals = strsplit(str_S11{t});
                    re_save(t) = str2double(vals(1));
                    im_save(t) = str2double(vals(2));
                    S11_antenna_nanoVNA_save(t) = 20 * log10(sqrt(re_save(t)^2 + im_save(t)^2));
                end

                num_start = str2num(get(handles.num_start, 'string'));
                filename = get(handles.save_sweep_filename, 'string');
                filename = [filename, '_S11_', num2str(pattern_count + num_start), '.csv'];
                csvwrite(filename, [freq_nanoVNA, S11_antenna_nanoVNA_save', re_save', im_save']); 
            end

            if get(handles.save_sweep_pattern_file,'value') == 1
                %save the S11 response

                pattern_save = zeros(8, 8);

                %save all the button value into array
                for i = 1:region_num
                    for j = 1:7
                        pattern_save(i, j) = get(handles.(sprintf('togglebutton%d%d', i, j)), 'Value');
                    end
                    if(rem(i, 2))
                        pattern_save(i, 8) = get(handles.(sprintf('togglebutton%d%d', i, 8)), 'Value');
                    end
                end

                num_start = str2num(get(handles.num_start, 'string'));
                filename = get(handles.save_sweep_filename, 'string');
                filename = [filename, '_pattern_', num2str(pattern_count + num_start - 1), '.csv'];
                csvwrite(filename, pattern_save); 
            end

            %check the smallest S11 value and save it
            fprintf('Reading S data for pattern No.%d from nanoVNA...\n', pattern_count);
            
            H_antenna_nanoVNA = save_S21(S_measure, nanoVNA_port, multifreq_sweep, hObject, eventdata, handles);

            current_index = size(S_extremum, 1) + 1;

            for j = 1:num_target_freq

                S_extremum(current_index, j) = 20 * log10(abs(H_antenna_nanoVNA(freq_target_index(j))));
                pattern_saved(:, :, current_index) = pattern_array;
                H_antenna_nanoVNA_saved(:, current_index) = H_antenna_nanoVNA;

                %fprintf('\ncurrent at marker %d: %f', j, 20 * log10(abs(H_antenna_nanoVNA(freq_target_index(j)))));
            end
            fprintf('\n\n');

            for j = 1:num_target_freq
                if S_measure == 1
                    %fprintf('S11_min at marker %d: %f\n', j, min(S_extremum(:, j))); 
                else
                    if S_opti_min_max_selection == 1
                        fprintf('S11_min at marker %d: %f\n', j, min(S_extremum(:, j))); 
                    else
                        fprintf('S21_max at marker %d: %f\n', j, max(S_extremum(:, j))); 
                    end
                end
            end
            
        elseif read_from_nanoVNA == 0   % read from RF meter
            if get(handles.save_sweep_pattern_file,'value') == 1
                %save the S11 response

                pattern_save = zeros(8, 8);

                %save all the button value into array
                for i = 1:region_num
                    for j = 1:7
                        pattern_save(i, j) = get(handles.(sprintf('togglebutton%d%d', i, j)), 'Value');
                    end
                    if(rem(i, 2))
                        pattern_save(i, 8) = get(handles.(sprintf('togglebutton%d%d', i, 8)), 'Value');
                    end
                end

                num_start = str2num(get(handles.num_start, 'string'));
                filename = get(handles.save_sweep_filename, 'string');
                filename = [filename, '_pattern_', num2str(pattern_count + num_start - 1), '.csv'];
                csvwrite(filename, pattern_save); 
            end

            %check the smallest S11 value and save it
            
            if rfmeter_debug
                fprintf('Reading DC voltage for pattern No.%d from RF meter...\n', pattern_count);
            end
            
            H_antenna_nanoVNA = save_VDC_rfmeter();

            current_index = length(S_extremum) + 1;

            S_extremum(current_index) = H_antenna_nanoVNA;
            pattern_saved(:, :, current_index) = pattern_array;

            if rfmeter_debug
                fprintf('\ncurrent reading from RF meter: %d', H_antenna_nanoVNA);
                fprintf('\n\n');
            
                if S_measure == 1
                    fprintf('S11_min: %d\n', min(S_extremum)); 
                else
                    if S_opti_min_max_selection == 1
                        fprintf('S11_min: %d\n', min(S_extremum)); 
                    else
                        fprintf('S21_max: %d\n', max(S_extremum)); 
                    end
                end
            end
            
        end
    else
        pause(view_sweeping_interval);
    end

   
function [S21_max, pattern_num, pattern_saved, H_antenna_nanoVNA_saved] = pattern_config_and_measure_gain(Enable_mapping, read_from_nanoVNA, function_type, nanoVNA_port, freq_target_index, ...
          S21_max, pattern_num, pattern_saved, H_antenna_nanoVNA_saved, pattern_count, hObject, eventdata, handles)            %function_type: 0 for normal; 1 for antenna sweeping; 2 for IMN sweeping
    global timeout;
    global t_delay;
    global pattern_array;         % pattern_array is a num array for displaying stubs on GUI
    global len
    
    global board_pattern;         % board_pattern is a series of char arrays for constructing bitstream for configuring
    global num_board_selection;
    global antenna_board_selection
    global IMN_board_selection
    global filter_board_selection
    
    global region_num;
    
    global num_target_freq;
    
    global S_opti_min_max_selection;
    global rfmeter_debug;
    
    global ESP32_configured;
   
    if Enable_mapping
        pattern_mapping();
        pattern_array_char = numarray2chararray(pattern_array);
        if function_type == 1
            if num_board_selection ~= 1
                board_pattern{antenna_board_selection} = pattern_array_char;
            else
                board_pattern{1} = pattern_array_char;
            end
        elseif function_type == 2
            if num_board_selection ~= 1
                board_pattern{IMN_board_selection} = pattern_array_char;
            else
                board_pattern{1} = pattern_array_char;
            end
        elseif function_type == 3
            if num_board_selection ~= 1
                board_pattern{filter_board_selection} = pattern_array_char;
            else
                board_pattern{1} = pattern_array_char;
            end
        end
    end

    bitstream = set_pattern_and_config(pattern_array, hObject, eventdata, handles);
    if rfmeter_debug
        fprintf('\n\nConfiguring pattern No.%d:\n', pattern_count);
    end
    
    if get(handles.configure_method, 'value') == 1
        send_ESP32_port(bitstream, timeout, hObject, eventdata, handles);
    else
        send_ESP32_serial(bitstream, hObject, eventdata, handles);
        fprintf('Device 1 pattern configured through Serial port!\n\n');     
    end

    if read_from_nanoVNA == 1
        pause(t_delay);

        if get(handles.save_sweep_S_file,'value') == 1
            %save the S11 response

            str_freq = nanoCommand(nanoVNA_port, 'frequencies');
            str_S11 = nanoCommand(nanoVNA_port, 'data 0');

            for t = 1:len
                freq_nanoVNA(t) = str2double(str_freq{t});
                vals = strsplit(str_S11{t});
                re_save(t) = str2double(vals(1));
                im_save(t) = str2double(vals(2));
                S11_antenna_nanoVNA_save(t) = 20 * log10(sqrt(re_save(t)^2 + im_save(t)^2));
            end

            num_start = str2num(get(handles.num_start, 'string'));
            filename = get(handles.save_sweep_filename, 'string');
            filename = [filename, '_S11_', num2str(pattern_count + num_start), '.csv'];
            csvwrite(filename, [freq_nanoVNA', S11_antenna_nanoVNA_save', re_save', im_save']); 
        end

        if get(handles.save_sweep_pattern_file,'value') == 1
            %save the S11 response

            pattern_save = zeros(8, 8);

            %save all the button value into array
            for i = 1:region_num
                for j = 1:7
                    pattern_save(i, j) = get(handles.(sprintf('togglebutton%d%d', i, j)), 'Value');
                end
                if(rem(i, 2))
                    pattern_save(i, 8) = get(handles.(sprintf('togglebutton%d%d', i, 8)), 'Value');
                end
            end

            num_start = str2num(get(handles.num_start, 'string'));
            filename = get(handles.save_sweep_filename, 'string');
            filename = [filename, '_pattern_', num2str(pattern_count + num_start - 1), '.csv'];
            csvwrite(filename, pattern_save); 
        end

        %check the extrenum S21 value and save it
        fprintf('Reading S21 data for pattern No.%d from nanoVNA...\n', pattern_count);


        H_antenna_nanoVNA = save_S21_raw(nanoVNA_port);


        current_index = size(S21_max, 1) + 1;

        for j = 1:num_target_freq

            S21_max(current_index, j) = 20 * log10(abs(H_antenna_nanoVNA(freq_target_index(j))));
            pattern_saved(:, :, current_index) = pattern_array;
            H_antenna_nanoVNA_saved(:, current_index) = H_antenna_nanoVNA;

            fprintf('\ncurrent at marker %d: %f', j, 20 * log10(abs(H_antenna_nanoVNA(freq_target_index(j)))));
        end
        fprintf('\n\n');
        for j = 1:num_target_freq
            if S_opti_min_max_selection == 2
                fprintf('S21 max at marker %d: %f\n', j, max(S21_max(:, j))); 
            elseif S_opti_min_max_selection == 1
                fprintf('S21 min at marker %d: %f\n', j, min(S21_max(:, j))); 
            end
        end
        
    elseif read_from_nanoVNA == 0   % read from RF meter
        if get(handles.save_sweep_pattern_file,'value') == 1
            %save the S11 response

            pattern_save = zeros(8, 8);

            %save all the button value into array
            for i = 1:region_num
                for j = 1:7
                    pattern_save(i, j) = get(handles.(sprintf('togglebutton%d%d', i, j)), 'Value');
                end
                if(rem(i, 2))
                    pattern_save(i, 8) = get(handles.(sprintf('togglebutton%d%d', i, 8)), 'Value');
                end
            end

            num_start = str2num(get(handles.num_start, 'string'));
            filename = get(handles.save_sweep_filename, 'string');
            filename = [filename, '_pattern_', num2str(pattern_count + num_start - 1), '.csv'];
            csvwrite(filename, pattern_save); 
        end

        %check the smallest S11 value and save it
        if rfmeter_debug
            fprintf('Reading DC voltage for pattern No.%d from RF meter...\n', pattern_count);
        end

        H_antenna_nanoVNA = save_VDC_rfmeter();

        %length(S21_max)
        current_index = length(S21_max) + 1;

        S21_max(current_index) = H_antenna_nanoVNA;
        pattern_saved(:, :, current_index) = pattern_array;
        
        if rfmeter_debug
            fprintf('\ncurrent reading from RF meter: %d', H_antenna_nanoVNA);
            fprintf('\n\n');
            
            if S_opti_min_max_selection == 2
                fprintf('S21 max: %d\n', max(S21_max)); 
            elseif S_opti_min_max_selection == 1
                fprintf('S21 min: %d\n', min(S21_max)); 
            end
        end
            
    end



% --- Executes on button press in nanoVNA_initialization.
function nanoVNA_initialization_Callback(hObject, eventdata, handles)
% hObject    handle to nanoVNA_initialization (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global multifreq_sweep;
    
    nanoVNA_initialization(multifreq_sweep, hObject, eventdata, handles);

    
    
function nanoVNA_initialization(spectrum_sweep, hObject, eventdata, handles)
    global single_target_freq;
    global len;
    
    global nanoVNA_initialized;
    global s_nanoVNA;
    
    len = str2double(get(handles.num_sweep_point, 'string'));
    num_points = len;
    
    if ~nanoVNA_initialized
        s_nanoVNA = get_serial_port_nanoVNA(hObject, eventdata, handles);
        fopen(s_nanoVNA);
        nanoVNA_initialized = 1;
    end
    
    recall_num = get(handles.VNA_recall, 'string');
    
    
    %['recall ', recall_num]
    %nanoCommand(s, ['recall ', recall_num]);      %recall calibration setting
    
    if spectrum_sweep
        
        nanoCommand(s_nanoVNA, ['scan 100000000 3000000000 ', int2str(len)]);

        nanoCommand(s_nanoVNA, 'trace 0 logmag'); 
        nanoCommand(s_nanoVNA, 'trace 0 channel 0'); 
        nanoCommand(s_nanoVNA, 'trace 1 smith'); 
        nanoCommand(s_nanoVNA, 'trace 1 channel 0'); 
        nanoCommand(s_nanoVNA, 'trace 2 channel 1');                  %0 for S11 and 1 for S21
        nanoCommand(s_nanoVNA, 'trace 2 logmag'); 
        nanoCommand(s_nanoVNA, 'trace 3 channel 1'); 
        nanoCommand(s_nanoVNA, 'trace 3 off'); 

        freq_target(1) = 1.37 * 1e9;
        freq_target(2) = 1.45 * 1e9;
        freq_target(3) = 2.1 * 1e9;
        freq_target(4) = 2.25 * 1e9;
        freq_target(5) = 2.45 * 1e9;

        for j = 1:5
            freq_target_index(j) = round((freq_target(j) - 100e6) / ((3e9 - 100e6) / num_points));
            marker_command = sprintf('marker %d %d', j, freq_target_index(j));
            nanoCommand(s_nanoVNA, marker_command);
        end
    else
        single_target_freq = str2double(get(handles.freq_target1, 'string')) * 1e9;
        
        target_freq_low = single_target_freq - 0.05 * 1e9; 
        target_freq_high = single_target_freq + 0.05 * 1e9; 
        
        nanoCommand(s_nanoVNA, ['scan ' num2str(target_freq_low) ' ' num2str(target_freq_high) ' ' num2str(11)]);
        marker_command = sprintf('marker %d %d', 1, 5);
        nanoCommand(s_nanoVNA, marker_command);
        
        for j = 2:4 
            marker_command = sprintf('marker %d off', j);
            nanoCommand(s_nanoVNA, marker_command);
        end
    end
         
    %fclose(s);
    clear s_nanoVNA;
    nanoVNA_initialized = 0;
    
    fprintf('nanoVNA configured!\n\n');
    
% --- Executes on selection change in patch_type_selection.
function patch_type_selection_Callback(hObject, eventdata, handles)
% hObject    handle to patch_type_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns patch_type_selection contents as cell array
%        contents{get(hObject,'Value')} returns selected item from patch_type_selection
    global patch_type_dec;
    patch_type_dec = get(handles.patch_type_selection, 'Value');
    
    
% --- Executes on selection change in num_board_selection.
function num_board_selection_Callback(hObject, eventdata, handles)
% hObject    handle to num_board_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns num_board_selection contents as cell array
%        contents{get(hObject,'Value')} returns selected item from num_board_selection   
    global num_board_selection;
    num_board_selection = get(handles.num_board_selection, 'Value');
    if num_board_selection == 1
        handles.current_board_selection.Enable = 'off';
        handles.antenna_board_selection.Enable = 'off';
        handles.IMN_board_selection.Enable = 'off';
        handles.filter_board_selection.Enable = 'off';
        handles.application_for_opti.Enable = 'off';
    else
        handles.current_board_selection.Enable = 'on';
        handles.antenna_board_selection.Enable = 'on';
        handles.IMN_board_selection.Enable = 'on';
        handles.filter_board_selection.Enable = 'on';
        handles.application_for_opti.Enable = 'on';
    end


% --- Executes on selection change in IMN_board_selection.
function IMN_board_selection_Callback(hObject, eventdata, handles)
% hObject    handle to IMN_board_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns IMN_board_selection contents as cell array
%        contents{get(hObject,'Value')} returns selected item from IMN_board_selection
    global IMN_board_selection;  
    IMN_board_selection = get(handles.IMN_board_selection, 'Value') - 1;
    

% --- Executes on selection change in antenna_board_selection.
function antenna_board_selection_Callback(hObject, eventdata, handles)
% hObject    handle to antenna_board_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns antenna_board_selection contents as cell array
%        contents{get(hObject,'Value')} returns selected item from antenna_board_selection
    global antenna_board_selection;
    antenna_board_selection = get(handles.antenna_board_selection, 'Value') - 1;
    
    
% --- Executes on selection change in filter_board_selection.
function filter_board_selection_Callback(hObject, eventdata, handles)
% hObject    handle to filter_board_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns filter_board_selection contents as cell array
%        contents{get(hObject,'Value')} returns selected item from filter_board_selection
    global filter_board_selection;
    filter_board_selection = get(handles.filter_board_selection, 'Value') - 1;


% --- Executes on button press in save_config_for_board.
function save_config_for_board_Callback(hObject, eventdata, handles)
% hObject    handle to save_config_for_board (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global region_num;
    global RF_pattern;
    global board_pattern;
    
    current_board = get(handles.current_board_selection, 'Value');
    
    board_pattern{current_board} = RF_pattern;
    
    % clear the pattern after saving
    RF_pattern = ['0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0'];
    
    %clear all the button value
    for i = 1:region_num
        for j = 1:7
            set(handles.(sprintf('togglebutton%d%d', i, j)), 'Value', 0);
            set(handles.(sprintf('togglebutton%d%d', i, j)), 'BackgroundColor', [0.94 0.94 0.94]);
        end
        if(rem(i, 2))
            set(handles.(sprintf('togglebutton%d%d', i, 8)), 'Value', 0);
            set(handles.(sprintf('togglebutton%d%d', i, 8)), 'BackgroundColor', [0.94 0.94 0.94]);
        end
    end
    
    %assignin('base', fprintf('pattern_board_d%', current_board), current_board);
    fprintf('Pattern for board No.%d is saved!\n\n', current_board);
    
    
% --- Executes on button press in check_pattern.
function check_pattern_Callback(hObject, eventdata, handles)
% hObject    handle to check_pattern (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global region_num;
    global RF_pattern;
    global board_pattern;
    
    current_board = get(handles.current_board_selection, 'Value');
    
    RF_pattern = board_pattern{current_board};
    
    for i = 1:region_num
        for j = 1:7
            set(handles.(sprintf('togglebutton%d%d', i, j)), 'Value', str2num(RF_pattern(i, j)));
            if(str2num(RF_pattern(i, j)))
                set(handles.(sprintf('togglebutton%d%d', i, j)), 'BackgroundColor', 'r');
                RF_pattern(i, j) = '1';
            else
                set(handles.(sprintf('togglebutton%d%d', i, j)), 'BackgroundColor', [0.94 0.94 0.94]);
                RF_pattern(i, j) = '0';
            end
        end
        if(rem(i, 2))
            set(handles.(sprintf('togglebutton%d%d', i, 8)), 'Value', str2num(RF_pattern(i, 8)));
            if(str2num(RF_pattern(i, 8)))
                set(handles.(sprintf('togglebutton%d%d', i, 8)), 'BackgroundColor', 'r');
                RF_pattern(i, 8) = '1';
            else
                set(handles.(sprintf('togglebutton%d%d', i, 8)), 'BackgroundColor', [0.94 0.94 0.94]);
                RF_pattern(i, 8) = '0';
            end
        end
    end
    
    fprintf('Pattern for board No.%d is restored!\n\n', current_board);



% --- Executes on button press in S_optimize.
function S_optimize_Callback(hObject, eventdata, handles)
% hObject    handle to S_optimize (see GCBO)f
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    for j = 1:length(findobj('type','figure'))
        close(figure(j));
    end

    global len;
    global pattern_array;
    global pattern_mapped;

    global patch_type_dec;
    patch_type_dec = get(handles.patch_type_selection, 'value');

    global multifreq_sweep;
    global marker_recover;
    
    global feedback_source;
    feedback_source = get(handles.feedback_source, 'value');
    
    global application_type_dec;
    application_type_dec = get(handles.application_type, 'value');
    
    global num_target_freq;
    global freq_nanoVNA;
    global top_num_save;
    
    global w_window;
    global h_window;
    global linecolor;
    global linewidth;
    
    global view_sweeping;
    
    global S_opti_min_max_selection;
    
    global s_MCU_DC;
    global RFmeter_initialized;
    global enable_selfadaptive;
    global ADC_extremum;
    
    global nanoVNA_initialized;
    global nanoVNA_extremum;
    
    global sweeping_inprogress;
    sweeping_inprogress = 1;
    
    global ESP32_configured;
    global s_ESP32;
    
    global s_nanoVNA;
    global nanoVNA_initialized;
    
    num_points = len;
    
    if multifreq_sweep
        num_target_freq = get(handles.number_freq, 'value');
        en_weight = get(handles.enable_weight, 'value');
        marker_recover = get(handles.marker_to_recover, 'value');
    
        freq_target(1) = str2double(get(handles.freq_target1, 'string')) * 1e9;
        freq_target(2) = str2double(get(handles.freq_target2, 'string')) * 1e9;
        freq_target(3) = str2double(get(handles.freq_target3, 'string')) * 1e9;
        freq_target(4) = str2double(get(handles.freq_target4, 'string')) * 1e9;
        freq_target(5) = str2double(get(handles.freq_target5, 'string')) * 1e9;
        freq_target(6) = str2double(get(handles.freq_target6, 'string')) * 1e9;
        freq_target(7) = str2double(get(handles.freq_target7, 'string')) * 1e9;
        freq_target(8) = str2double(get(handles.freq_target8, 'string')) * 1e9;
          
        if en_weight
            weight(1) = str2double(get(handles.weight1, 'string'));
            weight(2) = str2double(get(handles.weight2, 'string'));
            weight(3) = str2double(get(handles.weight3, 'string'));
            weight(4) = str2double(get(handles.weight4, 'string'));
            weight(5) = str2double(get(handles.weight5, 'string'));
            weight(6) = str2double(get(handles.weight6, 'string'));
            weight(7) = str2double(get(handles.weight7, 'string'));
            weight(8) = str2double(get(handles.weight8, 'string'));
        end

        for j = 1:num_target_freq
            freq_target_index(j) = round((freq_target(j) - 100e6) / ((3e9 - 100e6) / num_points));
            freq_target_round(j) = 100e6 + freq_target_index(j) * (3e9 - 100e6) / num_points;
            S11_min = [];
            S21_min = [];
            S21_max = [];
            pattern_num(j) = 0;
            S_param_measured = [];
            pattern_saved = [];
        end
    else
        num_target_freq = 1;
        marker_recover = 1;
        freq_target(1) = str2double(get(handles.freq_target1, 'string')) * 1e9;
        freq_target_index(1) = 1;
        freq_target_round(1) = freq_target(1);
        S11_min = [];
        S21_min = [];
        S21_max = [];
        pattern_num(1) = 0;
        S_param_measured = [];
        pattern_saved = [];
    end
    
    %reconstruct the pattern array
    pattern_array = zeros(8, 8);
    pattern_mapped = zeros(10, 6);   

    if feedback_source == 1

            %% start the serial port for nanoVNA
        if ~view_sweeping
            if ~nanoVNA_initialized
                s_nanoVNA = get_serial_port_nanoVNA(hObject, eventdata, handles);
                fopen(s_nanoVNA);
                nanoVNA_initialized = 1;
            end

            %nanoCommand(s, ['scan 100000000 3000000000 ', int2str(len)]);
        else
            s_nanoVNA = '';
        end

        if application_type_dec == 1 % Sweep based on S11 for antenna application
            if ~view_sweeping
                get_nanoVNA_parameters();
            end

            % sweeping algorithm for antenna patterns
            if patch_type_dec == 1                            % rect patch antenna with solid patterns
                [S11_min, pattern_num, pattern_saved, S_param_measured] = rect_sweep_algorithm(1, 0, s_nanoVNA, freq_target_index, S11_min, pattern_num, ...
                 pattern_saved, S_param_measured, hObject, eventdata, handles);
            elseif patch_type_dec == 2                            % fork patch antenna 
                [S11_min, pattern_num, pattern_saved, S_param_measured] = preset_sweep_algorithm(1, 0, s_nanoVNA, freq_target_index, S11_min, pattern_num, ...
                 pattern_saved, S_param_measured, hObject, eventdata, handles);
            elseif patch_type_dec == 3                            % preset antenna 
                [S11_min, pattern_num, pattern_saved, S_param_measured] = preset_sweep_algorithm(1, 0, s_nanoVNA, freq_target_index, S11_min, pattern_num, ...
                 pattern_saved, S_param_measured, hObject, eventdata, handles);
            end

            if ~view_sweeping
                fclose(s_nanoVNA);
                nanoVNA_initialized = 0;
                fprintf('\nAntenna pattern sweeping finished!\n\n');
            end
  
        elseif application_type_dec == 2  % IMN sweeping based on S11
            if ~view_sweeping
                get_nanoVNA_parameters();
            end   
            
            [S11_min, pattern_num, pattern_saved, S_param_measured] = IMN_sweep_algorithm(1, 0, s_nanoVNA, freq_target_index, S11_min, pattern_num, ...
             pattern_saved, S_param_measured, hObject, eventdata, handles);
            
            if ~view_sweeping
                fclose(s_nanoVNA);
                nanoVNA_initialized = 0;
                fprintf('\nIMN pattern sweeping finished!\n\n');
            end
             
        elseif application_type_dec == 3  % filter sweeping based on S21
            if ~view_sweeping
                get_nanoVNA_parameters();
            end
            
            if S_opti_min_max_selection == 1
%                 [S11_min, pattern_num, pattern_saved, S_param_measured] = preset_sweep_algorithm(1, 1, s_nanoVNA, freq_target_index, S11_min, pattern_num, ...
%                  pattern_saved, S_param_measured, hObject, eventdata, handles);
                [S21_min, pattern_num, pattern_saved, S_param_measured] = preset_sweep_algorithm(1, 1, s_nanoVNA, freq_target_index, S21_min, pattern_num, ...
                 pattern_saved, S_param_measured, hObject, eventdata, handles);
            elseif S_opti_min_max_selection == 2
                [S21_max, pattern_num, pattern_saved, S_param_measured] = preset_sweep_algorithm(1, 1, s_nanoVNA, freq_target_index, S21_max, pattern_num, ...
                 pattern_saved, S_param_measured, hObject, eventdata, handles);
            end
            
            if ~view_sweeping
                fclose(s_nanoVNA);
                nanoVNA_initialized = 0;
                fprintf('\nRF filter pattern sweeping finished!\n\n');
            end
             
        end
        
        if ~view_sweeping
            for j = 1:num_target_freq
                if application_type_dec == 3  % filter sweeping based on S21
                    if S_opti_min_max_selection == 1
                        %[S11_min_sorted(:, j), sorted_index(:, j)] = sort(S11_min(:, j), 'ascend');
                        [S21_min_sorted(:, j), sorted_index(:, j)] = sort(S21_min(:, j), 'ascend');
                    elseif S_opti_min_max_selection == 2
                        [S21_max_sorted(:, j), sorted_index(:, j)] = sort(S21_max(:, j), 'descend');
                    end
                else
                    [S11_min_sorted(:, j), sorted_index(:, j)] = sort(S11_min(:, j), 'ascend');
                end

            end
        
            if application_type_dec == 3  % filter sweeping based on S21
                if S_opti_min_max_selection == 1
                    assignin('base', 'S21_min', S21_min);
                    assignin('base', 'S21_min_sorted', S21_min_sorted);
                    assignin('base', 'sorted_index', sorted_index);
                    sweeping_inprogress = 0;
                    recover_pattern(S11_min_sorted(1, marker_recover), sorted_index(1, marker_recover), pattern_saved(:, :, sorted_index(1, marker_recover)), S_param_measured(:, sorted_index(1, marker_recover)), freq_target_round(marker_recover), hObject, eventdata, handles);
                    
                    nanoVNA_extremum = S21_min_sorted(1, marker_recover);
                elseif S_opti_min_max_selection == 2
                    assignin('base', 'S21_max', S21_max);
                    assignin('base', 'S21_max_sorted', S21_max_sorted);
                    assignin('base', 'sorted_index', sorted_index);
                    sweeping_inprogress = 0;
                    recover_pattern(S21_max_sorted(1, marker_recover), sorted_index(1, marker_recover), pattern_saved(:, :, sorted_index(1, marker_recover)), S_param_measured(:, sorted_index(1, marker_recover)), freq_target_round(marker_recover), hObject, eventdata, handles);
                
                    nanoVNA_extremum = S21_max_sorted(1, marker_recover);
                end
            else
                assignin('base', 'S11_min', S11_min);
                assignin('base', 'S11_min_sorted', S11_min_sorted);
                assignin('base', 'sorted_index', sorted_index);
                sweeping_inprogress = 0;

                recover_pattern(S11_min_sorted(1, marker_recover), sorted_index(1, marker_recover), pattern_saved(:, :, sorted_index(1, marker_recover)), S_param_measured(:, sorted_index(1, marker_recover)), freq_target_round(marker_recover), hObject, eventdata, handles);
                nanoVNA_extremum = S11_min_sorted(1, marker_recover);
            end

            assignin('base', 'pattern_saved', pattern_saved);
            assignin('base', 'S_param_measured', S_param_measured);

            if get(handles.save_opti_S, 'value')    % save S curve
                for j = 1:num_target_freq
                    figure;
                    set(gcf,'position',[[200, 200], w_window, h_window]);

                    x_local = xline(freq_target_round(j), '--', 'Linewidth', linewidth, 'Fontsize', 15);
                    x_local.Label = [num2str(freq_target_round(j) / 1e9), ' GHz'];
                    x_local.LabelVerticalAlignment = 'bottom';
                    x_local.LabelHorizontalAlignment = 'left';
                    grid on
                    set(gca, 'FontSize', 18);
                    title('S-parameters measured', 'FontSize', 20);
                    xlabel('Frequency (GHz)', 'FontSize', 18);
                    ylabel('S-parameter (dB)', 'FontSize', 18);
                    ylim([-inf 0]);

                    dropdown_menu = get(handles.application_type, 'string'); 

                    mkdir([pwd '\recent_saved']);
                    for k = 1:top_num_save
                        plot(freq_nanoVNA, 20 * log10(abs(S_param_measured(:, sorted_index(k, j)))), 'Color', linecolor, 'LineWidth', linewidth);
                        filename = sprintf('%s_S-param_%.2fGHz_top%d.fig', char(dropdown_menu(get(handles.application_type, 'value'))), freq_target(j)/1e9, k);
                        saveas(gcf, fullfile([pwd '\recent_saved'], filename));
                    end

                end
            end
        end
    elseif feedback_source == 2  % source from desktop VNA
        % remain to be finished
    elseif feedback_source == 3  % source from AC-DC converter
        if ~RFmeter_initialized
            s_MCU_DC = get_serial_port_rfmeter(hObject, eventdata, handles);
            RFmeter_initialized = 1;
        end
        
        if view_sweeping
            s_MCU_DC = '';
        end
        
        if application_type_dec == 1 % Sweep based on S11 for antenna application

            % sweeping algorithm for antenna patterns
            if patch_type_dec == 1                            % rect patch antenna with solid patterns
                [S11_min, pattern_num, pattern_saved, S_param_measured] = rect_sweep_algorithm(0, 0, s_MCU_DC, freq_target_index, S11_min, pattern_num, ...
                 pattern_saved, S_param_measured, hObject, eventdata, handles);
            elseif patch_type_dec == 2                            % fork patch antenna 
                [S11_min, pattern_num, pattern_saved, S_param_measured] = preset_sweep_algorithm(0, 0, s_MCU_DC, freq_target_index, S11_min, pattern_num, ...
                 pattern_saved, S_param_measured, hObject, eventdata, handles);
            elseif patch_type_dec == 3                            % preset antenna 
                [S11_min, pattern_num, pattern_saved, S_param_measured] = preset_sweep_algorithm(0, 0, s_MCU_DC, freq_target_index, S11_min, pattern_num, ...
                 pattern_saved, S_param_measured, hObject, eventdata, handles);
            end

            if ~enable_selfadaptive
                delete(s_MCU_DC);
                RFmeter_initialized = 0;
            end
            %fprintf('\nAntenna pattern sweeping finished!\n\n');

            fprintf('\nAntenna pattern sweeping finished!\n\n');
  
        elseif application_type_dec == 2  % IMN sweeping based on S11
            
            [S11_min, pattern_num, pattern_saved, S_param_measured] = IMN_sweep_algorithm(0, 0, s_MCU_DC, freq_target_index, S11_min, pattern_num, ...
             pattern_saved, S_param_measured, hObject, eventdata, handles);
            
            if ~enable_selfadaptive
                delete(s_MCU_DC);
                RFmeter_initialized = 0;
            end
            fprintf('\nIMN pattern sweeping finished!\n\n');
             
        elseif application_type_dec == 3  % filter sweeping based on S21
 
            [S21_max, pattern_num, pattern_saved, S_param_measured] = preset_sweep_algorithm(0, 1, s_MCU_DC, freq_target_index, S21_max, pattern_num, ...
             pattern_saved, S_param_measured, hObject, eventdata, handles);
            

            if ~enable_selfadaptive
                delete(s_MCU_DC);
                RFmeter_initialized = 0;
            end
            fprintf('\nRF filter pattern sweeping finished!\n\n');
             
        end
        
        if ~view_sweeping
            
            if application_type_dec == 3  % filter sweeping based on S21
                if S_opti_min_max_selection == 1
                    [S21_min_sorted, S21_sorted_index] = sort(S21_max, 'ascend');

                    assignin('base', 'S21', S21_max);
                    assignin('base', 'S21_min_sorted', S21_min_sorted);
                    assignin('base', 'S21_min_sortindex', S21_sorted_index);

                    assignin('base', 'pattern_saved', pattern_saved);
                    assignin('base', 'S_param_measured', S_param_measured);
                    
                    sweeping_inprogress = 0;
                    recover_pattern_rfmeter(S21_sorted_index(1), pattern_saved(:, :, S21_sorted_index(1)), hObject, eventdata, handles);
                    
                    ADC_extremum = S21_min_sorted(1);
                elseif S_opti_min_max_selection == 2
                    [S21_max_sorted, S21_sorted_index] = sort(S21_max, 'descend');

                    assignin('base', 'S21', S21_max);
                    assignin('base', 'S21_max_sorted', S21_max_sorted);
                    assignin('base', 'S21_max_sortindex', S21_sorted_index);

                    assignin('base', 'pattern_saved', pattern_saved);
                    assignin('base', 'S_param_measured', S_param_measured);

                    sweeping_inprogress = 0;
                    recover_pattern_rfmeter(S21_sorted_index(1), pattern_saved(:, :, S21_sorted_index(1)), hObject, eventdata, handles);
                    
                    ADC_extremum = S21_max_sorted(1);
                end
                
            else
                [S11_min_sorted, S11_sorted_index] = sort(S11_min, 'ascend');

                assignin('base', 'S11', S11_min);
                assignin('base', 'S11_min_sorted', S11_min_sorted);
                assignin('base', 'S11_min_sortindex', S11_sorted_index);

                assignin('base', 'pattern_saved', pattern_saved);
                assignin('base', 'S_param_measured', S_param_measured);
                
                sweeping_inprogress = 0;
                recover_pattern_rfmeter(S11_sorted_index(1), pattern_saved(:, :, S11_sorted_index(1)), hObject, eventdata, handles);
                
                ADC_extremum = S11_min_sorted(1);
            end
        else
            s_MCU_DC = '';
        end
        
    end
    
    if ~view_sweeping
        if get(handles.save_opti_pattern, 'value')    % save FPRFS pattern
            if feedback_source == 1
                for j = 1:num_target_freq
                    dropdown_menu = get(handles.application_type, 'string'); 

                    mkdir([pwd '\recent_saved']);
                    for k = 1:top_num_save
                        filename = sprintf('%s_pattern_%.2fGHz_top%d.csv', char(dropdown_menu(get(handles.application_type, 'value'))), freq_target(j)/1e9, k);
                        csvwrite(fullfile([pwd '\recent_saved'], filename), pattern_saved(:, :, sorted_index(k, j)));
                    end
                end
                save(sprintf('%s_pattern.mat', char(dropdown_menu(get(handles.application_type, 'value')))));
            elseif feedback_source == 3
                dropdown_menu = get(handles.application_type, 'string'); 
                mkdir([pwd '\recent_saved_rfmeter']);
                
                filename = sprintf('%s_pattern.csv', char(dropdown_menu(get(handles.application_type, 'value'))));
                if application_type_dec == 3  % filter sweeping based on S21
                    csvwrite(fullfile([pwd '\recent_saved_rfmeter'], filename), pattern_saved(:, :, S21_sorted_index(1)));
                else
                    csvwrite(fullfile([pwd '\recent_saved_rfmeter'], filename), pattern_saved(:, :, S11_sorted_index(1)));
                end
            end
        end
    else
        s_MCU_DC = '';
    end
    
    delete(s_ESP32);
    ESP32_configured = 0;
    
    
    
% --- Executes on button press in gain_optimize.
function gain_optimize_Callback(hObject, eventdata, handles)
% hObject    handle to gain_optimize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    for j = 1:length(findobj('type','figure'))
        close(figure(j));
    end

    global len;
    global pattern_array;
    global pattern_mapped;

    global patch_type_dec;
    patch_type_dec = get(handles.patch_type_selection, 'value');

    global multifreq_sweep;
    global marker_recover;
    
    global feedback_source;
    feedback_source = get(handles.feedback_source, 'value');
    
    global application_type_dec;
    application_type_dec = get(handles.application_type, 'value');
    
    global num_target_freq;
    global freq_nanoVNA;
    global top_num_save;
    
    global w_window;
    global h_window;
    global linecolor;
    global linewidth;
    
    global s_MCU_DC;
    global RFmeter_initialized;
    global enable_selfadaptive;
    global S_opti_min_max_selection;
    
    global ADC_extremum;
    
    global nanoVNA_initialized;
    global nanoVNA_extremum;
    
    global sweeping_inprogress;
    sweeping_inprogress = 1;
    
    global ESP32_configured;
    global s_ESP32;
    
    global s_nanoVNA;
    global nanoVNA_initialized;
    
    num_points = len;
    
    if multifreq_sweep
        num_target_freq = get(handles.number_freq, 'value');
        en_weight = get(handles.enable_weight, 'value');
        marker_recover = get(handles.marker_to_recover, 'value');
    
        freq_target(1) = str2double(get(handles.freq_target1, 'string')) * 1e9;
        freq_target(2) = str2double(get(handles.freq_target2, 'string')) * 1e9;
        freq_target(3) = str2double(get(handles.freq_target3, 'string')) * 1e9;
        freq_target(4) = str2double(get(handles.freq_target4, 'string')) * 1e9;
        freq_target(5) = str2double(get(handles.freq_target5, 'string')) * 1e9;
        freq_target(6) = str2double(get(handles.freq_target6, 'string')) * 1e9;
        freq_target(7) = str2double(get(handles.freq_target7, 'string')) * 1e9;
        freq_target(8) = str2double(get(handles.freq_target8, 'string')) * 1e9;       
    
        if en_weight
            weight(1) = str2double(get(handles.weight1, 'string'));
            weight(2) = str2double(get(handles.weight2, 'string'));
            weight(3) = str2double(get(handles.weight3, 'string'));
            weight(4) = str2double(get(handles.weight4, 'string'));
            weight(5) = str2double(get(handles.weight5, 'string'));
            weight(6) = str2double(get(handles.weight6, 'string'));
            weight(7) = str2double(get(handles.weight7, 'string'));
            weight(8) = str2double(get(handles.weight8, 'string'));
        end

        for j = 1:num_target_freq
            
            freq_target_index(j) = round((freq_target(j) - 100e6) / ((3e9 - 100e6) / num_points));
            freq_target_round(j) = 100e6 + freq_target_index(j) * (3e9 - 100e6) / num_points;
            S21_max = [];
            S21_min = [];
            pattern_num(j) = 0;
            S_param_measured = [];
            pattern_saved = [];
        end
    else
        num_target_freq = 1;
        marker_recover = 1;
        freq_target(1) = str2double(get(handles.freq_target1, 'string')) * 1e9;
        freq_target_index(1) = 1;
        freq_target_round(1) = freq_target(1);
        S21_max = [];
        S21_min = [];
        pattern_num(1) = 0;
        S_param_measured = [];
        pattern_saved = [];
    end
    
    %reconstruct the pattern array
    pattern_array = zeros(8, 8);
    pattern_mapped = zeros(10, 6);
    
    if feedback_source == 1
        %% start the serial port for nanoVNA
        if ~nanoVNA_initialized
            s_nanoVNA = get_serial_port_nanoVNA(hObject, eventdata, handles);
            fopen(s_nanoVNA);
            nanoVNA_initialized = 1;
        end

        %nanoCommand(s, ['scan 100000000 3000000000 ', int2str(len)]);

        if application_type_dec == 1 % Sweep based on S11 for antenna application

            % sweeping algorithm for antenna patterns

            if patch_type_dec == 1                            % rect patch antenna with solid patterns
                [S21_max, pattern_num, pattern_saved, S_param_measured] = rect_sweep_algorithm(1, 2, s_nanoVNA, freq_target_index, S21_max, pattern_num, ...
                 pattern_saved, S_param_measured, hObject, eventdata, handles);
            elseif patch_type_dec == 2   % fork patch antenna 
                [S21_max, pattern_num, pattern_saved, S_param_measured] = preset_sweep_algorithm(1, 2, s_nanoVNA, freq_target_index, S21_max, pattern_num, ...
                 pattern_saved, S_param_measured, hObject, eventdata, handles);
            elseif patch_type_dec == 3                            % preset antenna 
                [S21_max, pattern_num, pattern_saved, S_param_measured] = preset_sweep_algorithm(1, 2, s_nanoVNA, freq_target_index, S21_max, pattern_num, ...
                 pattern_saved, S_param_measured, hObject, eventdata, handles);
            end

            fclose(s_nanoVNA);
            nanoVNA_initialized = 0;
            fprintf('\nAntenna pattern sweeping finished!\n\n');
 
        elseif application_type_dec == 2  % IMN sweeping based on S11
            
            [S21_max, pattern_num, pattern_saved, S_param_measured] = IMN_sweep_algorithm(1, 2, s_nanoVNA, freq_target_index, S21_max, pattern_num, ...
             pattern_saved, S_param_measured, hObject, eventdata, handles);
    
            fclose(s_nanoVNA);
            nanoVNA_initialized = 0;
            fprintf('\nIMN pattern sweeping finished!\n\n');
            
        elseif application_type_dec == 3  % filter sweeping based on S21
            
            [S21_max, pattern_num, pattern_saved, S_param_measured] = preset_sweep_algorithm(1, 2, s_nanoVNA, freq_target_index, S21_max, pattern_num, ...
             pattern_saved, S_param_measured, hObject, eventdata, handles);
            
            fclose(s_nanoVNA);
            nanoVNA_initialized = 0;
            fprintf('\nRF filter pattern sweeping finished!\n\n');
            
        end
        
        if (application_type_dec == 3) && (S_opti_min_max_selection == 1) %min
            for j = 1:num_target_freq
                [S21_min_sorted(:, j), sorted_index(:, j)] = sort(S21_min(:, j), 'ascend');
            end

            assignin('base', 'S21_min_sorted', S21_min_sorted);
            assignin('base', 'sorted_index', sorted_index);

            sweeping_inprogress = 0;
            recover_pattern(S21_min_sorted(1, marker_recover), sorted_index(1, marker_recover), pattern_saved(:, :, sorted_index(1, marker_recover)), S_param_measured(:, sorted_index(1, marker_recover)), freq_target_round(marker_recover), hObject, eventdata, handles);
        
            nanoVNA_extremum = S21_min_sorted(1, marker_recover);
        else
            for j = 1:num_target_freq
                [S21_max_sorted(:, j), sorted_index(:, j)] = sort(S21_max(:, j), 'descend');
            end

            assignin('base', 'S21_max_sorted', S21_max_sorted);
            assignin('base', 'sorted_index', sorted_index);

            sweeping_inprogress = 0;
            recover_pattern(S21_max_sorted(1, marker_recover), sorted_index(1, marker_recover), pattern_saved(:, :, sorted_index(1, marker_recover)), S_param_measured(:, sorted_index(1, marker_recover)), freq_target_round(marker_recover), hObject, eventdata, handles);
        
            nanoVNA_extremum = S21_max_sorted(1, marker_recover);
        end
            
            
        assignin('base', 'S21', S21_max);
        assignin('base', 'pattern_saved', pattern_saved);
        assignin('base', 'S_param_measured', S_param_measured);
        
        if get(handles.save_opti_S, 'value')    % save S curve
            for j = 1:num_target_freq
                figure;
                set(gcf,'position',[[200, 200], w_window, h_window]);
                
                x_local = xline(freq_target_round(j), '--', 'Linewidth', linewidth, 'Fontsize', 15);
                x_local.Label = [num2str(freq_target_round(j) / 1e9), ' GHz'];
                x_local.LabelVerticalAlignment = 'bottom';
                x_local.LabelHorizontalAlignment = 'left';
                grid on
                set(gca, 'FontSize', 18);
                title('S21 measured at far-field', 'FontSize', 20);
                xlabel('Frequency (GHz)', 'FontSize', 18);
                ylabel('Far-field S21 (dB)', 'FontSize', 18);
                ylim([-inf 0]);
                dropdown_menu = get(handles.application_type, 'string'); 
                
                mkdir([pwd '\recent_saved']);
                for k = 1:top_num_save
                    plot(freq_nanoVNA, 20 * log10(abs(S_param_measured(:, sorted_index(k, j)))), 'Color', linecolor, 'LineWidth', linewidth);
                    filename = sprintf('%s_S-param_%.2fGHz_top%d.fig', char(dropdown_menu(get(handles.application_type, 'value'))), freq_target(j)/1e9, k);
                    saveas(gcf, fullfile([pwd '\recent_saved'], filename));
                end

            end
        end
    elseif feedback_source == 2  % source from desktop VNA
        % remain to be finished
    elseif feedback_source == 3  % source from AC-DC converter
        if ~RFmeter_initialized
            s_MCU_DC = get_serial_port_rfmeter(hObject, eventdata, handles);
            RFmeter_initialized = 1;
        end

        
        if application_type_dec == 1 % Sweep based on S11 for antenna application

            % sweeping algorithm for antenna patterns
            if patch_type_dec == 1                            % rect patch antenna with solid patterns
                [S21_max, pattern_num, pattern_saved, S_param_measured] = rect_sweep_algorithm(0, 2, s_MCU_DC, freq_target_index, S21_max, pattern_num, ...
                 pattern_saved, S_param_measured, hObject, eventdata, handles);
            elseif patch_type_dec == 2                            % fork patch antenna 
                [S21_max, pattern_num, pattern_saved, S_param_measured] = preset_sweep_algorithm(0, 2, s_MCU_DC, freq_target_index, S21_max, pattern_num, ...
                 pattern_saved, S_param_measured, hObject, eventdata, handles);
            elseif patch_type_dec == 3                            % preset antenna 
                [S21_max, pattern_num, pattern_saved, S_param_measured] = preset_sweep_algorithm(0, 2, s_MCU_DC, freq_target_index, S21_max, pattern_num, ...
                 pattern_saved, S_param_measured, hObject, eventdata, handles);
            end

            if ~enable_selfadaptive
                delete(s_MCU_DC);
                RFmeter_initialized = 0;
            end
            fprintf('\nAntenna pattern sweeping finished!\n\n');
  
        elseif application_type_dec == 2  % IMN sweeping based on S11
            
            [S21_max, pattern_num, pattern_saved, S_param_measured] = IMN_sweep_algorithm(0, 2, s_MCU_DC, freq_target_index, S21_max, pattern_num, ...
             pattern_saved, S_param_measured, hObject, eventdata, handles);
            
            if ~enable_selfadaptive
                delete(s_MCU_DC);
                RFmeter_initialized = 0;
            end
            fprintf('\nIMN pattern sweeping finished!\n\n');
             
        elseif application_type_dec == 3  % filter sweeping based on S21
            
            [S21_max, pattern_num, pattern_saved, S_param_measured] = preset_sweep_algorithm(0, 2, s_MCU_DC, freq_target_index, S21_max, pattern_num, ...
             pattern_saved, S_param_measured, hObject, eventdata, handles);
            
            if ~enable_selfadaptive
                delete(s_MCU_DC);
                RFmeter_initialized = 0;
            end
            fprintf('\nRF filter pattern sweeping finished!\n\n');
             
        end

        if S_opti_min_max_selection == 1 %min
            [S21_min_sorted, S21_sorted_index] = sort(S21_max, 'ascend');

            assignin('base', 'S21', S21_max);
            assignin('base', 'S21_min_sorted', S21_min_sorted);
            assignin('base', 'S21_min_sortindex', S21_sorted_index);

            assignin('base', 'pattern_saved', pattern_saved);
            assignin('base', 'S_param_measured', S_param_measured);

            sweeping_inprogress = 0;
            recover_pattern_rfmeter(S21_sorted_index(1), pattern_saved(:, :, S21_sorted_index(1)), hObject, eventdata, handles);
            
            ADC_extremum = S21_min_sorted(1);
        elseif S_opti_min_max_selection == 2
            [S21_max_sorted, S21_sorted_index] = sort(S21_max, 'descend');

            assignin('base', 'S21', S21_max);
            assignin('base', 'S21_max_sorted', S21_max_sorted);
            assignin('base', 'S21_max_sortindex', S21_sorted_index);

            assignin('base', 'pattern_saved', pattern_saved);
            assignin('base', 'S_param_measured', S_param_measured);

            sweeping_inprogress = 0;
            recover_pattern_rfmeter(S21_sorted_index(1), pattern_saved(:, :, S21_sorted_index(1)), hObject, eventdata, handles);
            
            ADC_extremum = S21_max_sorted(1);
        end
    end
    
    if get(handles.save_opti_pattern, 'value')    % save FPRFS pattern
        if feedback_source == 1
            for j = 1:num_target_freq
                dropdown_menu = get(handles.application_type, 'string'); 

                mkdir([pwd '\recent_saved']);
                for k = 1:top_num_save
                    filename = sprintf('%s_pattern_%.2fGHz_top%d.csv', char(dropdown_menu(get(handles.application_type, 'value'))), freq_target(j)/1e9, k);
                    csvwrite(fullfile([pwd '\recent_saved'], filename), pattern_saved(:, :, sorted_index(k, j)));
                end
            end
            save(sprintf('%s_pattern.mat', char(dropdown_menu(get(handles.application_type, 'value')))));
        elseif feedback_source == 3
            dropdown_menu = get(handles.application_type, 'string'); 
            mkdir([pwd '\recent_saved_rfmeter']);

            filename = sprintf('%s_pattern.csv', char(dropdown_menu(get(handles.application_type, 'value'))));
            csvwrite(fullfile([pwd '\recent_saved_rfmeter'], filename), pattern_saved(:, :, S21_sorted_index(1)));
        end
    end
    
    delete(s_ESP32);
    ESP32_configured = 0;
    
    
    

% --- Executes on button press in config_FPRFS.
function config_FPRFS_Callback(hObject, eventdata, handles)
% hObject    handle to config_FPRFS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global num_board_selection;
    global num_board_selection_IP; 
    global current_board_selection_IP; 
    global region_num;
    global RF_pattern;
    global board_pattern;
    global data_num;
    global timeout;
    
    global s_ESP32;
    global ESP32_configured;
    
    bitstream = zeros(1, data_num);
    region = cell(num_board_selection, region_num);
    
    if num_board_selection == 1
        board_pattern{1} = RF_pattern;
    end
        
    for i = 1:num_board_selection
        bitstream(1) = num_board_selection;  % send the header with number of boards activated
        for j = 1:region_num
            region(i, j) = {strrep(board_pattern{i}(j, :), ' ', '')};
            bitstream((i - 1) * region_num + j + 1) = bin2dec(region(i, j));       % shift all bits to the right by 1 byte to make room for the first byte
        end   
    end
    
    %restore pattern on GUI display with board pattern 1
    for i = 1:region_num
        for j = 1:7
            set(handles.(sprintf('togglebutton%d%d', i, j)), 'Value', str2num(board_pattern{1}(i, j)));
            if(str2num(board_pattern{1}(i, j)))
                set(handles.(sprintf('togglebutton%d%d', i, j)), 'BackgroundColor', 'r');
                RF_pattern(i, j) = '1';
            else
                set(handles.(sprintf('togglebutton%d%d', i, j)), 'BackgroundColor', [0.94 0.94 0.94]);
                RF_pattern(i, j) = '0';
            end
        end
        if(rem(i, 2))
            set(handles.(sprintf('togglebutton%d%d', i, 8)), 'Value', str2num(board_pattern{1}(i, 8)));
            if(str2num(board_pattern{1}(i, 8)))
                set(handles.(sprintf('togglebutton%d%d', i, 8)), 'BackgroundColor', 'r');
                RF_pattern(i, 8) = '1';
            else
                set(handles.(sprintf('togglebutton%d%d', i, 8)), 'BackgroundColor', [0.94 0.94 0.94]);
                RF_pattern(i, 8) = '0';
            end
        end
    end
    
    bitstream;
    
    if get(handles.configure_method, 'value') == 1
        if handles.ConfigBoth.Value
            send_ESP32_port(bitstream, timeout, hObject, eventdata, handles);
            fprintf('Device 1 pattern configured through WiFi!\n\n');
            send_ESP32_port_2(bitstream, timeout, hObject, eventdata, handles);
            fprintf('Device 2 pattern configured through WiFi!\n\n');
        else
            if num_board_selection_IP == 1
                send_ESP32_port(bitstream, timeout, hObject, eventdata, handles);
                fprintf('Device 1 pattern configured through WiFi!\n\n');
            elseif num_board_selection_IP == 2
                if current_board_selection_IP == 1
                    send_ESP32_port(bitstream, timeout, hObject, eventdata, handles);
                    fprintf('Device 1 pattern configured through WiFi!\n\n');
                elseif current_board_selection_IP == 2
                    send_ESP32_port_2(bitstream, timeout, hObject, eventdata, handles);
                    fprintf('Device 2 pattern configured through WiFi!\n\n');
                end
            end
        end
    else
        send_ESP32_serial(bitstream, hObject, eventdata, handles);
        fprintf('Device 1 pattern configured through Serial port!\n\n');
        delete(s_ESP32);
        ESP32_configured = 0;
    end
    

% --- Executes on button press in config_FPRFS_readnanoVNA.
function config_FPRFS_readnanoVNA_Callback(hObject, eventdata, handles)
% hObject    handle to config_FPRFS_readnanoVNA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global t_delay
    global freq_nanoVNA;
    global num_board_selection;    
    global region_num;
    global RF_pattern;
    global board_pattern;
    global data_num;
    global timeout;
    
    global multifreq_sweep;
    
    global s_ESP32;
    global ESP32_configured;
    
    global s_nanoVNA;
    global nanoVNA_initialized;
    
    global w_window;
    global h_window;
    global linecolor;
    global linewidth;
    
    for j = 1:length(findobj('type','figure'))
        close(figure(j));
    end
    
    %nanoVNA_initialization(1, hObject, eventdata, handles);
    
    bitstream = zeros(1, data_num);
    region = cell(num_board_selection, region_num);
    
    if num_board_selection == 1
        board_pattern{1} = RF_pattern;
    end
        
    for i = 1:num_board_selection
        bitstream(1) = num_board_selection;  % send the header with number of boards activated
        for j = 1:region_num
            region(i, j) = {strrep(board_pattern{i}(j, :), ' ', '')};
            bitstream((i - 1) * region_num + j + 1) = bin2dec(region(i, j));       % shift all bits to the right by 1 byte to make room for the first byte
        end   
    end
    
    %restore pattern on GUI display with board pattern 1
    for i = 1:region_num
        for j = 1:7
            set(handles.(sprintf('togglebutton%d%d', i, j)), 'Value', str2num(board_pattern{1}(i, j)));
            if(str2num(board_pattern{1}(i, j)))
                set(handles.(sprintf('togglebutton%d%d', i, j)), 'BackgroundColor', 'r');
                RF_pattern(i, j) = '1';
            else
                set(handles.(sprintf('togglebutton%d%d', i, j)), 'BackgroundColor', [0.94 0.94 0.94]);
                RF_pattern(i, j) = '0';
            end
        end
        if(rem(i, 2))
            set(handles.(sprintf('togglebutton%d%d', i, 8)), 'Value', str2num(board_pattern{1}(i, 8)));
            if(str2num(board_pattern{1}(i, 8)))
                set(handles.(sprintf('togglebutton%d%d', i, 8)), 'BackgroundColor', 'r');
                RF_pattern(i, 8) = '1';
            else
                set(handles.(sprintf('togglebutton%d%d', i, 8)), 'BackgroundColor', [0.94 0.94 0.94]);
                RF_pattern(i, 8) = '0';
            end
        end
    end
    
    bitstream;
    
    if get(handles.configure_method, 'value') == 1
        send_ESP32_port(bitstream, timeout, hObject, eventdata, handles);
        fprintf('Pattern configured through WiFi!\n\n');
    else
        send_ESP32_serial(bitstream, hObject, eventdata, handles);
        fprintf('Device 1 pattern configured through Serial port!\n\n');
        delete(s_ESP32);
        ESP32_configured = 0;
    end
    
    pause(t_delay);
    
    %% for displaying the S11 measured from the nanoVNA
    localmin_index = str2double(get(handles.num_localmin, 'string'));
    smooth_index = str2double(get(handles.smooth_index, 'string'));
    
    fprintf('\nReading S11 data from nanoVNA...\n');
    get_nanoVNA_parameters();

    if ~nanoVNA_initialized
        s_nanoVNA = get_serial_port_nanoVNA(hObject, eventdata, handles);
        fopen(s_nanoVNA);
        nanoVNA_initialized = 1;
    end
    
    H_antenna_nanoVNA = save_S21(1, s_nanoVNA, multifreq_sweep, hObject, eventdata, handles);

    fclose(s_nanoVNA);
    nanoVNA_initialized = 0;
    
    H_antenna_smooth = smooth(H_antenna_nanoVNA, smooth_index);
    H_antenna_smooth_log = 20 * log10(abs(H_antenna_smooth));
    
    if multifreq_sweep
        S11_table = [freq_nanoVNA H_antenna_nanoVNA H_antenna_smooth];
        assignin('base', 'S11min_plot', S11_table);

        freq_localmin = nonzeros(freq_nanoVNA .* (islocalmin(H_antenna_smooth_log)));
        H_antenna_localmin = nonzeros(H_antenna_smooth_log .* (islocalmin(H_antenna_smooth_log)));
        size_localmin = length(freq_localmin);

        num_localmin = min(localmin_index, size_localmin);       %setting the number of smallest localmin

        S11_localmin_table = sortrows([freq_localmin H_antenna_localmin], 2, 'ascend');    %sort the table based on the reflection coefficient
        S11_localmin_table((num_localmin + 1):size_localmin, :) = [];
        %assignin('base', 'freq_localmin', freq_localmin);
        %assignin('base', 'H_antenna_localmin', H_antenna_localmin);

        S11_localmin_table = sortrows(S11_localmin_table, 1, 'ascend');    %sort the table based on the frequency

        figure;
        set(gcf,'position',[[200, 200], w_window, h_window]);
        plot(freq_nanoVNA, 20 * log10(abs(H_antenna_nanoVNA)), 'Color', linecolor, 'LineWidth', linewidth);
        hold on;
        plot(freq_nanoVNA, 20 * log10(abs(H_antenna_smooth)), 'Color', linecolor, 'LineWidth', linewidth, 'Linestyle', ':');

        grid on
        set(gca, 'FontSize', 18);
        title('S11 of configured FPRFS pattern', 'FontSize', 20);
        xlabel('Frequency (GHz)', 'FontSize', 18);
        ylabel('Reflection Coefficient S11 (dB)', 'FontSize', 18);
        ylim([-inf 0]);

        for k = 1:num_localmin
            x_localmin = xline(S11_localmin_table(k, 1), '--', 'Linewidth', linewidth, 'Fontsize', 15);
            x_localmin.Label = [num2str(freq_localmin(k) / 1e9), ' GHz'];
            x_localmin.LabelVerticalAlignment = 'bottom';
            x_localmin.LabelHorizontalAlignment = 'left';
        end
    else
       fprintf('Measured S at single frequency: %.2fdB\n', 20 * log10(abs(H_antenna_nanoVNA)));
    end
    fprintf('S11 of configured pattern is displayed.\n\n');
    %% end of S11 measuring
    
    
    
function save_file_name_Callback(hObject, eventdata, handles)
% hObject    handle to save_file_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of save_file_name as text
%        str2double(get(hObject,'String')) returns contents of save_file_name as a double


% --- Executes during object creation, after setting all properties.
function save_file_name_CreateFcn(hObject, eventdata, handles)
% hObject    handle to save_file_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


    % --- Executes on button press in clear.
function clear_Callback(hObject, eventdata, handles)
% hObject    handle to clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)  
    global region_num;
    global RF_pattern;
    
    RF_pattern = ['0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0';...
                  '0' '0' '0' '0' '0' '0' '0' '0'];
    
    %clear all the button value
    for i = 1:region_num
        for j = 1:7
            set(handles.(sprintf('togglebutton%d%d', i, j)), 'Value', 0);
            set(handles.(sprintf('togglebutton%d%d', i, j)), 'BackgroundColor', [0.94 0.94 0.94]);
        end
        if(rem(i, 2))
            set(handles.(sprintf('togglebutton%d%d', i, 8)), 'Value', 0);
            set(handles.(sprintf('togglebutton%d%d', i, 8)), 'BackgroundColor', [0.94 0.94 0.94]);
        end
    end

    
% --- Executes on button press in set_all.
function set_all_Callback(hObject, eventdata, handles)
% hObject    handle to set_all (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    global region_num;
    global RF_pattern;
    
    RF_pattern = ['1' '1' '1' '1' '1' '1' '1' '1';...
                  '1' '1' '1' '1' '1' '1' '1' '0';...
                  '1' '1' '1' '1' '1' '1' '1' '1';...
                  '1' '1' '1' '1' '1' '1' '1' '0';...
                  '1' '1' '1' '1' '1' '1' '1' '1';...
                  '1' '1' '1' '1' '1' '1' '1' '0';...
                  '1' '1' '1' '1' '1' '1' '1' '1';...
                  '1' '1' '1' '1' '1' '1' '1' '0'];
    
    %set all the button value to 1
    for i = 1:region_num
        for j = 1:7
            set(handles.(sprintf('togglebutton%d%d', i, j)), 'Value', 1);
            set(handles.(sprintf('togglebutton%d%d', i, j)), 'BackgroundColor', 'r');
        end
        if(rem(i, 2))
            set(handles.(sprintf('togglebutton%d%d', i, 8)), 'Value', 1);
            set(handles.(sprintf('togglebutton%d%d', i, 8)), 'BackgroundColor', 'r');
        end
    end


% --- Executes on button press in save_data.
function save_data_Callback(hObject, eventdata, handles)
% hObject    handle to save_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    filename = get(handles.save_file_name, 'string');
    filename = [filename, '.csv'];

    fprintf('\nReading S11 data from nanoVNA...\n\n');
    
    s = get_serial_port_nanoVNA(hObject, eventdata, handles);

    fopen(s);

    %save the S11 response
    str_freq = nanoCommand(s, 'frequencies');
    str_S11 = nanoCommand(s, 'data 0');

    len = length(str_S11);

    for t = 1:len
        freq_nanoVNA(t) = str2double(str_freq{t});
        vals = strsplit(str_S11{t});
        re_save(t) = str2double(vals(1));
        im_save(t) = str2double(vals(2));
        S11_antenna_nanoVNA_save(t) = 20 * log10(sqrt(re_save(t)^2 + im_save(t)^2));
    end

    csvwrite(filename, [freq_nanoVNA', S11_antenna_nanoVNA_save', re_save', im_save']); 
    fprintf('File "%s" saved!\n\n', filename);
    

% --- Executes on button press in save_pattern.
function save_pattern_Callback(hObject, eventdata, handles)
% hObject    handle to save_pattern (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    global region_num;
    
    pattern_save = zeros(8, 8);
    
    %save all the button value into array
    for i = 1:region_num
        for j = 1:7
            pattern_save(i, j) = get(handles.(sprintf('togglebutton%d%d', i, j)), 'Value');
        end
        if(rem(i, 2))
            pattern_save(i, 8) = get(handles.(sprintf('togglebutton%d%d', i, 8)), 'Value');
        end
    end

    filename = get(handles.save_file_name, 'string');
    filename = [filename, '.csv'];
    csvwrite(filename, pattern_save);

    
    % --- Executes on button press in browse_file.
function browse_file_Callback(hObject, eventdata, handles)
% hObject    handle to browse_file (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global load_filename;
    global path;

    [load_filename, path] = uigetfile('*.csv');
    if load_filename == 0
        % user pressed cancel
        return
    end

    global region_num;    
    global RF_pattern;

    pattern_load = csvread(fullfile(path, load_filename))

    %set all the button value accordingly
    for i = 1:region_num
        for j = 1:7
            set(handles.(sprintf('togglebutton%d%d', i, j)), 'Value', pattern_load(i, j));
            if(pattern_load(i, j))
                set(handles.(sprintf('togglebutton%d%d', i, j)), 'BackgroundColor', 'r');
                RF_pattern(i, j) = '1';
            else
                set(handles.(sprintf('togglebutton%d%d', i, j)), 'BackgroundColor', [0.94 0.94 0.94]);
                RF_pattern(i, j) = '0';
            end
        end
        if(rem(i, 2))
            set(handles.(sprintf('togglebutton%d%d', i, 8)), 'Value', pattern_load(i, 8));
            if(pattern_load(i, 8))
                set(handles.(sprintf('togglebutton%d%d', i, 8)), 'BackgroundColor', 'r');
                RF_pattern(i, 8) = '1';
            else
                set(handles.(sprintf('togglebutton%d%d', i, 8)), 'BackgroundColor', [0.94 0.94 0.94]);
                RF_pattern(i, 8) = '0';
            end
        end
    end

    

% --- Executes on selection change in current_board_selection.
function current_board_selection_Callback(hObject, eventdata, handles)
% hObject    handle to current_board_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns current_board_selection contents as cell array
%        contents{get(hObject,'Value')} returns selected item from current_board_selection 
    


% --- Executes on button press in togglebutton11.
function togglebutton11_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton11
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(1, 1) = '1';
    else 
        RF_pattern(1, 1) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton12.
function togglebutton12_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton12
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(1, 2) = '1';
    else 
        RF_pattern(1, 2) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton13.
function togglebutton13_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton13
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(1, 3) = '1';
    else 
        RF_pattern(1, 3) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton14.
function togglebutton14_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton14
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(1, 4) = '1';
    else 
        RF_pattern(1, 4) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton15.
function togglebutton15_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton15
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(1, 5) = '1';
    else 
        RF_pattern(1, 5) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton16.
function togglebutton16_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton16
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(1, 6) = '1';
    else 
        RF_pattern(1, 6) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton17.
function togglebutton17_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton17
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(1, 7) = '1';
    else 
        RF_pattern(1, 7) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton18.
function togglebutton18_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton18
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(1, 8) = '1';
    else 
        RF_pattern(1, 8) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton21.
function togglebutton21_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton21
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(2, 1) = '1';
    else 
        RF_pattern(2, 1) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton54.
function togglebutton22_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton54 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton54
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(2, 2) = '1';
    else 
        RF_pattern(2, 2) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton23.
function togglebutton23_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton23
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(2, 3) = '1';
    else 
        RF_pattern(2, 3) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

    
% --- Executes on button press in togglebutton24.
function togglebutton24_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton24
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(2, 4) = '1';
    else 
        RF_pattern(2, 4) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton25.
function togglebutton25_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton25
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(2, 5) = '1';
    else 
        RF_pattern(2, 5) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end   
    
% --- Executes on button press in togglebutton26.
function togglebutton26_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton26
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(2, 6) = '1';
    else 
        RF_pattern(2, 6) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton27.
function togglebutton27_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton27
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(2, 7) = '1';
    else 
        RF_pattern(2, 7) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton31.
function togglebutton31_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton31
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(3, 1) = '1';
    else 
        RF_pattern(3, 1) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end    

% --- Executes on button press in togglebutton52.
function togglebutton32_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton52 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton52
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(3, 2) = '1';
    else 
        RF_pattern(3, 2) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton46.
function togglebutton33_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton33
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(3, 3) = '1';
    else 
        RF_pattern(3, 3) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton34.
function togglebutton34_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton34 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton34
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(3, 4) = '1';
    else 
        RF_pattern(3, 4) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton35.
function togglebutton35_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton35 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton35
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(3, 5) = '1';
    else 
        RF_pattern(3, 5) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
 
% --- Executes on button press in togglebutton36.
function togglebutton36_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton36 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton36
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(3, 6) = '1';
    else 
        RF_pattern(3, 6) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton37.
function togglebutton37_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton37 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton37
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(3, 7) = '1';
    else 
        RF_pattern(3, 7) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton38.
function togglebutton38_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton38 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton38
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(3, 8) = '1';
    else 
        RF_pattern(3, 8) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton41.
function togglebutton41_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton41
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(4, 1) = '1';
    else 
        RF_pattern(4, 1) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton42.
function togglebutton42_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton42
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(4, 2) = '1';
    else 
        RF_pattern(4, 2) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton44.
function togglebutton43_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton44 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton43
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(4, 3) = '1';
    else 
        RF_pattern(4, 3) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton44.
function togglebutton44_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton44 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton44
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(4, 4) = '1';
    else 
        RF_pattern(4, 4) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton45.
function togglebutton45_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton45 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton45
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(4, 5) = '1';
    else 
        RF_pattern(4, 5) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton47.
function togglebutton46_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton46 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton46
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(4, 6) = '1';
    else 
        RF_pattern(4, 6) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton47.
function togglebutton47_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton47 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton47
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(4, 7) = '1';
    else 
        RF_pattern(4, 7) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton51.
function togglebutton51_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton51 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton51
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(5, 1) = '1';
    else 
        RF_pattern(5, 1) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton52.
function togglebutton52_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton52 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton52
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(5, 2) = '1';
    else 
        RF_pattern(5, 2) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton53.
function togglebutton53_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton53 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton53
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(5, 3) = '1';
    else 
        RF_pattern(5, 3) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
 
% --- Executes on button press in togglebutton54.
function togglebutton54_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton54 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton54
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(5, 4) = '1';
    else 
        RF_pattern(5, 4) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton55.
function togglebutton55_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton55 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton55
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(5, 5) = '1';
    else 
        RF_pattern(5, 5) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton56.
function togglebutton56_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton56 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton56
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(5, 6) = '1';
    else 
        RF_pattern(5, 6) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton57.
function togglebutton57_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton57 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton57
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(5, 7) = '1';
    else 
        RF_pattern(5, 7) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton58.
function togglebutton58_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton58 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton58
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(5, 8) = '1';
    else 
        RF_pattern(5, 8) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton61.
function togglebutton61_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton61 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton61
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(6, 1) = '1';
    else 
        RF_pattern(6, 1) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton62.
function togglebutton62_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton62 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton62
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(6, 2) = '1';
    else 
        RF_pattern(6, 2) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton63.
function togglebutton63_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton63 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton63
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(6, 3) = '1';
    else 
        RF_pattern(6, 3) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton64.
function togglebutton64_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton64 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton64
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(6, 4) = '1';
    else 
        RF_pattern(6, 4) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton65.
function togglebutton65_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton65 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton65
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(6, 5) = '1';
    else 
        RF_pattern(6, 5) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton66.
function togglebutton66_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton66 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton66
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(6, 6) = '1';
    else 
        RF_pattern(6, 6) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton64.
function togglebutton67_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton64 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton64
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(6, 7) = '1';
    else 
        RF_pattern(6, 7) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton71.
function togglebutton71_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton71 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton71
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(7, 1) = '1';
    else 
        RF_pattern(7, 1) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
  
% --- Executes on button press in togglebutton72.
function togglebutton72_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton72 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton72
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(7, 2) = '1';
    else 
        RF_pattern(7, 2) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton73.
function togglebutton73_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton73 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton73
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(7, 3) = '1';
    else 
        RF_pattern(7, 3) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton74.
function togglebutton74_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton74 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton74
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(7, 4) = '1';
    else 
        RF_pattern(7, 4) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton75.
function togglebutton75_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton75 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton75
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(7, 5) = '1';
    else 
        RF_pattern(7, 5) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton76.
function togglebutton76_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton76 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton76
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(7, 6) = '1';
    else 
        RF_pattern(7, 6) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton77.
function togglebutton77_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton77 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton77
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(7, 7) = '1';
    else 
        RF_pattern(7, 7) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end
    
% --- Executes on button press in togglebutton78.
function togglebutton78_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton78 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton78
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(7, 8) = '1';
    else 
        RF_pattern(7, 8) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton81.
function togglebutton81_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton81 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton81
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(8, 1) = '1';
    else 
        RF_pattern(8, 1) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton82.
function togglebutton82_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton82 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton82
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(8, 2) = '1';
    else 
        RF_pattern(8, 2) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton83.
function togglebutton83_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton83 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton83
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(8, 3) = '1';
    else 
        RF_pattern(8, 3) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton84.
function togglebutton84_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton84 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton84
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(8, 4) = '1';
    else 
        RF_pattern(8, 4) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton85.
function togglebutton85_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton85 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton85
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(8, 5) = '1';
    else 
        RF_pattern(8, 5) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton86.
function togglebutton86_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton86 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton86
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(8, 6) = '1';
    else 
        RF_pattern(8, 6) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

% --- Executes on button press in togglebutton87.
function togglebutton87_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton87 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton87
    global RF_pattern;
    
    if(get(hObject,'Value') == 1)
        set(hObject,'BackgroundColor','r');
        RF_pattern(8, 7) = '1';
    else 
        RF_pattern(8, 7) = '0';
        set(hObject,'BackgroundColor',[0.94 0.94 0.94]);
    end

    


    

function IP_address_Callback(hObject, eventdata, handles)
% hObject    handle to IP_address (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of IP_address as text
%        str2double(get(hObject,'String')) returns contents of IP_address as a double


% --- Executes during object creation, after setting all properties.
function IP_address_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IP_address (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function freq_target1_Callback(hObject, eventdata, handles)
% hObject    handle to freq_target1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of freq_target1 as text
%        str2double(get(hObject,'String')) returns contents of freq_target1 as a double


% --- Executes during object creation, after setting all properties.
function freq_target1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to freq_target1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function COM_port_Callback(hObject, eventdata, handles)
% hObject    handle to COM_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of COM_port as text
%        str2double(get(hObject,'String')) returns contents of COM_port as a double


% --- Executes during object creation, after setting all properties.
function COM_port_CreateFcn(hObject, eventdata, handles)
% hObject    handle to COM_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function num_localmin_Callback(hObject, eventdata, handles)
% hObject    handle to num_localmin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of num_localmin as text
%        str2double(get(hObject,'String')) returns contents of num_localmin as a double


% --- Executes during object creation, after setting all properties.
function num_localmin_CreateFcn(hObject, eventdata, handles)
% hObject    handle to num_localmin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function smooth_index_Callback(hObject, eventdata, handles)
% hObject    handle tosmooth_index (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of num_localmin as text
%        str2double(get(hObject,'String')) returns contents of num_localmin as a double


% --- Executes during object creation, after setting all properties.
function smooth_index_CreateFcn(hObject, eventdata, handles)
% hObject    handle to smooth_index (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function input_port_Callback(hObject, eventdata, handles)
% hObject    handle to input_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of input_port as text
%        str2double(get(hObject,'String')) returns contents of input_port as a double


% --- Executes during object creation, after setting all properties.
function input_port_CreateFcn(hObject, eventdata, handles)
% hObject    handle to input_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function output_port_Callback(hObject, eventdata, handles)
% hObject    handle to output_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of output_port as text
%        str2double(get(hObject,'String')) returns contents of output_port as a double


% --- Executes during object creation, after setting all properties.
function output_port_CreateFcn(hObject, eventdata, handles)
% hObject    handle to output_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function patch_type_selection_CreateFcn(hObject, eventdata, handles)
% hObject    handle to patch_type_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in save_sweep_S_file.
function save_sweep_S_file_Callback(hObject, eventdata, handles)
% hObject    handle to save_sweep_S_file (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of save_sweep_S_file


% --- Executes on button press in save_sweep_pattern_file.
function save_sweep_pattern_file_Callback(hObject, eventdata, handles)
% hObject    handle to save_sweep_pattern_file (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of save_sweep_pattern_file


function save_sweep_filename_Callback(hObject, eventdata, handles)
% hObject    handle to save_sweep_filename (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of save_sweep_filename as text
%        str2double(get(hObject,'String')) returns contents of save_sweep_filename as a double


% --- Executes during object creation, after setting all properties.
function save_sweep_filename_CreateFcn(hObject, eventdata, handles)
% hObject    handle to save_sweep_filename (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes during object creation, after setting all properties.
function num_board_selection_CreateFcn(hObject, eventdata, handles)
% hObject    handle to num_board_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function current_board_selection_CreateFcn(hObject, eventdata, handles)
% hObject    handle to current_board_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes during object creation, after setting all properties.
function antenna_board_selection_CreateFcn(hObject, eventdata, handles)
% hObject    handle to antenna_board_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function IMN_board_selection_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IMN_board_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function filter_board_selection_CreateFcn(hObject, eventdata, handles)
% hObject    handle to filter_board_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function num_start_Callback(hObject, eventdata, handles)
% hObject    handle to num_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of num_start as text
%        str2double(get(hObject,'String')) returns contents of num_start as a double


% --- Executes during object creation, after setting all properties.
function num_start_CreateFcn(hObject, eventdata, handles)
% hObject    handle to num_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function freq_target2_Callback(hObject, eventdata, handles)
% hObject    handle to freq_target2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of freq_target2 as text
%        str2double(get(hObject,'String')) returns contents of freq_target2 as a double


% --- Executes during object creation, after setting all properties.
function freq_target2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to freq_target2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function freq_target3_Callback(hObject, eventdata, handles)
% hObject    handle to freq_target3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of freq_target3 as text
%        str2double(get(hObject,'String')) returns contents of freq_target3 as a double


% --- Executes during object creation, after setting all properties.
function freq_target3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to freq_target3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function freq_target4_Callback(hObject, eventdata, handles)
% hObject    handle to freq_target4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of freq_target4 as text
%        str2double(get(hObject,'String')) returns contents of freq_target4 as a double


% --- Executes during object creation, after setting all properties.
function freq_target4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to freq_target4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function freq_target5_Callback(hObject, eventdata, handles)
% hObject    handle to freq_target5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of freq_target5 as text
%        str2double(get(hObject,'String')) returns contents of freq_target5 as a double


% --- Executes during object creation, after setting all properties.
function freq_target5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to freq_target5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in marker_to_recover.
function marker_to_recover_Callback(hObject, eventdata, handles)
% hObject    handle to marker_to_recover (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns marker_to_recover contents as cell array
%        contents{get(hObject,'Value')} returns selected item from marker_to_recover
    global marker_receover;
    marker_receover = get(handles.marker_to_recover, 'Value');

% --- Executes during object creation, after setting all properties.
function marker_to_recover_CreateFcn(hObject, eventdata, handles)
% hObject    handle to marker_to_recover (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




function freq_target6_Callback(hObject, eventdata, handles)
% hObject    handle to freq_target6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of freq_target6 as text
%        str2double(get(hObject,'String')) returns contents of freq_target6 as a double


% --- Executes during object creation, after setting all properties.
function freq_target6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to freq_target6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function freq_target7_Callback(hObject, eventdata, handles)
% hObject    handle to freq_target7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of freq_target7 as text
%        str2double(get(hObject,'String')) returns contents of freq_target7 as a double


% --- Executes during object creation, after setting all properties.
function freq_target7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to freq_target7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function freq_target8_Callback(hObject, eventdata, handles)
% hObject    handle to freq_target8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of freq_target8 as text
%        str2double(get(hObject,'String')) returns contents of freq_target8 as a double


% --- Executes during object creation, after setting all properties.
function freq_target8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to freq_target8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function weight6_Callback(hObject, eventdata, handles)
% hObject    handle to weight6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of weight6 as text
%        str2double(get(hObject,'String')) returns contents of weight6 as a double


% --- Executes during object creation, after setting all properties.
function weight6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to weight6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function weight7_Callback(hObject, eventdata, handles)
% hObject    handle to weight7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of weight7 as text
%        str2double(get(hObject,'String')) returns contents of weight7 as a double


% --- Executes during object creation, after setting all properties.
function weight7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to weight7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function weight8_Callback(hObject, eventdata, handles)
% hObject    handle to weight8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of weight8 as text
%        str2double(get(hObject,'String')) returns contents of weight8 as a double


% --- Executes during object creation, after setting all properties.
function weight8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to weight8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function weight1_Callback(hObject, eventdata, handles)
% hObject    handle to weight1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of weight1 as text
%        str2double(get(hObject,'String')) returns contents of weight1 as a double


% --- Executes during object creation, after setting all properties.
function weight1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to weight1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function weight2_Callback(hObject, eventdata, handles)
% hObject    handle to weight2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of weight2 as text
%        str2double(get(hObject,'String')) returns contents of weight2 as a double


% --- Executes during object creation, after setting all properties.
function weight2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to weight2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function weight3_Callback(hObject, eventdata, handles)
% hObject    handle to weight3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of weight3 as text
%        str2double(get(hObject,'String')) returns contents of weight3 as a double


% --- Executes during object creation, after setting all properties.
function weight3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to weight3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function weight4_Callback(hObject, eventdata, handles)
% hObject    handle to weight4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of weight4 as text
%        str2double(get(hObject,'String')) returns contents of weight4 as a double


% --- Executes during object creation, after setting all properties.
function weight4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to weight4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function weight5_Callback(hObject, eventdata, handles)
% hObject    handle to weight5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of weight5 as text
%        str2double(get(hObject,'String')) returns contents of weight5 as a double


% --- Executes during object creation, after setting all properties.
function weight5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to weight5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function IP_address_2_Callback(hObject, eventdata, handles)
% hObject    handle to IP_address_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of IP_address_2 as text
%        str2double(get(hObject,'String')) returns contents of IP_address_2 as a double


% --- Executes during object creation, after setting all properties.
function IP_address_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IP_address_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in num_board_selection_IP.
function num_board_selection_IP_Callback(hObject, eventdata, handles)
% hObject    handle to num_board_selection_IP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns num_board_selection_IP contents as cell array
%        contents{get(hObject,'Value')} returns selected item from num_board_selection_IP
    global num_board_selection_IP;
    num_board_selection_IP = get(handles.num_board_selection_IP, 'Value');
    if num_board_selection_IP == 1
        handles.current_board_selection_IP.Enable = 'off';
        handles.IP_address_2.Enable = 'off';
        handles.ConfigBoth.Enable = 'off';
        handles.ConfigBoth.Value = 0;
    else
        handles.current_board_selection_IP.Enable = 'on';
        handles.IP_address_2.Enable = 'on';
        handles.ConfigBoth.Enable = 'on';
    end


% --- Executes during object creation, after setting all properties.
function num_board_selection_IP_CreateFcn(hObject, eventdata, handles)
% hObject    handle to num_board_selection_IP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in current_board_selection_IP.
function current_board_selection_IP_Callback(hObject, eventdata, handles)
% hObject    handle to current_board_selection_IP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns current_board_selection_IP contents as cell array
%        contents{get(hObject,'Value')} returns selected item from current_board_selection_IP
    global current_board_selection_IP;
    current_board_selection_IP = get(handles.current_board_selection_IP, 'Value');

% --- Executes during object creation, after setting all properties.
function current_board_selection_IP_CreateFcn(hObject, eventdata, handles)
% hObject    handle to current_board_selection_IP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in ConfigBoth.
function ConfigBoth_Callback(hObject, eventdata, handles)
% hObject    handle to ConfigBoth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of ConfigBoth


% --- Executes on button press in self_adapting.
function self_adapting_Callback(hObject, eventdata, handles)
% hObject    handle to self_adapting (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in application_type.
function application_type_Callback(hObject, eventdata, handles)
% hObject    handle to application_type (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns application_type contents as cell array
%        contents{get(hObject,'Value')} returns selected item from application_type


% --- Executes during object creation, after setting all properties.
function application_type_CreateFcn(hObject, eventdata, handles)
% hObject    handle to application_type (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function VNA_recall_Callback(hObject, eventdata, handles)
% hObject    handle to VNA_recall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of VNA_recall as text
%        str2double(get(hObject,'String')) returns contents of VNA_recall as a double


% --- Executes during object creation, after setting all properties.
function VNA_recall_CreateFcn(hObject, eventdata, handles)
% hObject    handle to VNA_recall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function num_sweep_point_Callback(hObject, eventdata, handles)
% hObject    handle to num_sweep_point (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of num_sweep_point as text
%        str2double(get(hObject,'String')) returns contents of num_sweep_point as a double
    global multifreq_sweep;

    if str2num(get(handles.num_sweep_point, 'string')) == 11
        set(handles.enable_multifreq, 'value', 0);
        multifreq_sweep = 0;
    else
        set(handles.enable_multifreq, 'value', 1);
        multifreq_sweep = 1;
    end


% --- Executes during object creation, after setting all properties.
function num_sweep_point_CreateFcn(hObject, eventdata, handles)
% hObject    handle to num_sweep_point (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in save_opti_S.
function save_opti_S_Callback(hObject, eventdata, handles)
% hObject    handle to save_opti_S (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of save_opti_S


% --- Executes on button press in save_opti_pattern.
function save_opti_pattern_Callback(hObject, eventdata, handles)
% hObject    handle to save_opti_pattern (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of save_opti_pattern



function self_adaptive_tolerance_Callback(hObject, eventdata, handles)
% hObject    handle to self_adaptive_tolerance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of self_adaptive_tolerance as text
%        str2double(get(hObject,'String')) returns contents of self_adaptive_tolerance as a double
    global selfadapt_error;
    selfadapt_error = 1 - get(handles.self_adaptive_tolerance, 'value');
    
    if get(handles.feedback_source, 'Value') == 3 %rfmeter
        set(handles.sensitivity, 'String', num2str(selfadapt_error * 300));
    elseif get(handles.feedback_source, 'Value') == 1 %nanoVNA
        set(handles.sensitivity, 'String', num2str(selfadapt_error * 20));
    end


% --- Executes during object creation, after setting all properties.
function self_adaptive_tolerance_CreateFcn(hObject, eventdata, handles)
% hObject    handle to self_adaptive_tolerance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in enable_weight.
function enable_weight_Callback(hObject, eventdata, handles)
% hObject    handle to enable_weight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable_weight

    global num_target_freq;
    enable_weight = get(handles.enable_weight, 'value');
    
    tar_freq_index = {handles.weight1, handles.weight2, handles.weight3, handles.weight4, handles.weight5, ...
            handles.weight6, handles.weight7, handles.weight8};
    
    if enable_weight
        for i = 1:num_target_freq
            tar_freq_index{i}.Enable = 'on';
        end
        
    else
        handles.weight1.Enable = 'off';
        handles.weight2.Enable = 'off';
        handles.weight3.Enable = 'off';
        handles.weight4.Enable = 'off';
        handles.weight5.Enable = 'off';
        handles.weight6.Enable = 'off';
        handles.weight7.Enable = 'off';
        handles.weight8.Enable = 'off';
    end
    
    

% --- Executes on selection change in feedback_source.
function feedback_source_Callback(hObject, eventdata, handles)
% hObject    handle to feedback_source (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns feedback_source contents as cell array
%        contents{get(hObject,'Value')} returns selected item from feedback_source
    global selfadapt_error;

    clear_RFmeter_port_Callback(hObject, eventdata, handles);
    
    if get(handles.feedback_source, 'Value') == 3 %rfmeter
        set(handles.sensitivity, 'String', num2str(selfadapt_error * 300));
    elseif get(handles.feedback_source, 'Value') == 1 %nanoVNA
        set(handles.sensitivity, 'String', num2str(selfadapt_error * 20));
    end

    tar_freq_index = {handles.freq_target1, handles.freq_target2, handles.freq_target3, handles.freq_target4, handles.freq_target5, ...
            handles.freq_target6, handles.freq_target7, handles.freq_target8};
    
    tar_freq_weight_index = {handles.weight1, handles.weight2, handles.weight3, handles.weight4, handles.weight5, ...
            handles.weight6, handles.weight7, handles.weight8};
    
    if get(handles.feedback_source, 'value') == 3
        handles.view_sweeping.Enable = 'on';
        handles.view_sweep_interval.Enable = 'on';
        
        handles.enable_multifreq.Enable = 'off';
        handles.enable_weight.Enable = 'off';
        handles.marker_to_recover.Enable = 'off';
        handles.number_freq.Enable = 'off';
        handles.top_number_save.Enable = 'off';
        handles.num_localmin.Enable = 'off';
        handles.smooth_index.Enable = 'off';
        
        handles.freq_target1.Enable = 'off';
        handles.freq_target2.Enable = 'off';
        handles.freq_target3.Enable = 'off';
        handles.freq_target4.Enable = 'off';
        handles.freq_target5.Enable = 'off';
        handles.freq_target6.Enable = 'off';
        handles.freq_target7.Enable = 'off';
        handles.freq_target8.Enable = 'off';
        
        handles.weight1.Enable = 'off';
        handles.weight2.Enable = 'off';
        handles.weight3.Enable = 'off';
        handles.weight4.Enable = 'off';
        handles.weight5.Enable = 'off';
        handles.weight6.Enable = 'off';
        handles.weight7.Enable = 'off';
        handles.weight8.Enable = 'off';
        
        handles.save_sweep_S_file.Enable = 'off';
        handles.save_sweep_pattern_file.Enable = 'off';
        handles.save_opti_S.Enable = 'off';
        handles.save_opti_pattern.Enable = 'off';
        handles.save_sweep_filename.Enable = 'off';     

        set(handles.save_opti_pattern, 'value', 0);
        
        handles.enable_other_filterload.Enable = 'off';
        handles.directly_coupled.Enable = 'off';
        handles.filter_load_name.Enable = 'off';
        
        % self adaptive panel
        handles.rfmeter_COM_port.Enable = 'on';
        handles.check_VDC.Enable = 'on';
        handles.initialize_RFmeter.Enable = 'on';
    else
        handles.view_sweeping.Enable = 'on';
        handles.view_sweep_interval.Enable = 'on';
        
        handles.enable_multifreq.Enable = 'on';
        handles.enable_weight.Enable = 'on';
        handles.marker_to_recover.Enable = 'on';
        handles.number_freq.Enable = 'on';
        handles.top_number_save.Enable = 'on';
        handles.num_localmin.Enable = 'on';
        handles.smooth_index.Enable = 'on';
        
        for i = 1:get(handles.number_freq, 'value')
            tar_freq_index{i}.Enable = 'on';
        end
        
        if get(handles.enable_weight, 'value')
            for i = 1:get(handles.number_freq, 'value')
                tar_freq_weight_index{i}.Enable = 'on';
                end
        end
        
        
        handles.save_sweep_S_file.Enable = 'on';
        handles.save_sweep_pattern_file.Enable = 'on';
        handles.save_opti_S.Enable = 'on';
        handles.save_opti_pattern.Enable = 'on';
        handles.save_sweep_filename.Enable = 'on';
        
        set(handles.save_opti_pattern, 'value', 1);
        
        handles.enable_other_filterload.Enable = 'on';
        handles.directly_coupled.Enable = 'on';
        handles.filter_load_name.Enable = 'on';
        
        % self adaptive panel
        handles.rfmeter_COM_port.Enable = 'off';
        handles.check_VDC.Enable = 'off';
        handles.initialize_RFmeter.Enable = 'off';
    end


% --- Executes during object creation, after setting all properties.
function feedback_source_CreateFcn(hObject, eventdata, handles)
% hObject    handle to feedback_source (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in configure_method.
function configure_method_Callback(hObject, eventdata, handles)
% hObject    handle to configure_method (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns configure_method contents as cell array
%        contents{get(hObject,'Value')} returns selected item from configure_method
    global ESP32_configured;
    
    ESP32_configured = 0;

    if get(handles.configure_method, 'value') == 1 % wifi
        handles.ESP32_COM.Enable = 'off';
        
        handles.IP_address.Enable = 'on';
        handles.IP_address_2.Enable = 'on';
    else % serial
        handles.ESP32_COM.Enable = 'on';
        
        handles.IP_address.Enable = 'off';
        handles.IP_address_2.Enable = 'off';
    end
        



% --- Executes during object creation, after setting all properties.
function configure_method_CreateFcn(hObject, eventdata, handles)
% hObject    handle to configure_method (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in enable_multifreq.
function enable_multifreq_Callback(hObject, eventdata, handles)
% hObject    handle to enable_multifreq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable_multifreq
    
    global num_target_freq
    global multifreq_sweep
    global marker_recover;

    multifreq_sweep = get(handles.enable_multifreq, 'value');
    
    if multifreq_sweep

        num_target_freq = get(handles.number_freq, 'value');
        tar_freq_index = {handles.freq_target1, handles.freq_target2, handles.freq_target3, handles.freq_target4, handles.freq_target5, ...
            handles.freq_target6, handles.freq_target7, handles.freq_target8};
        
        handles.number_freq.Enable = 'on';
        handles.enable_weight.Enable = 'on';
        
        for j = 1:num_target_freq
            tar_freq_index{j}.Enable = 'on';
        end
        
        handles.marker_to_recover.Enable = 'on';
    
        marker_recover = get(handles.marker_to_recover, 'value');
        num_target_freq = get(handles.number_freq, 'value');
    else
        
        num_target_freq = 1;
        handles.enable_weight.Enable = 'off';
        handles.number_freq.Enable = 'off';
        
        handles.freq_target2.Enable = 'off';
        handles.freq_target3.Enable = 'off';
        handles.freq_target4.Enable = 'off';
        handles.freq_target5.Enable = 'off';
        handles.freq_target6.Enable = 'off';
        handles.freq_target7.Enable = 'off';
        handles.freq_target8.Enable = 'off';
        handles.marker_to_recover.Enable = 'off';
    end

    % --- Executes on selection change in number_freq.
function number_freq_Callback(hObject, eventdata, handles)
% hObject    handle to number_freq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns number_freq contents as cell array
%        contents{get(hObject,'Value')} returns selected item from number_freq
    global multifreq_sweep
    global num_target_freq;

    if multifreq_sweep
        num_target_freq = get(handles.number_freq, 'value');
        tar_freq_index = {handles.freq_target1, handles.freq_target2, handles.freq_target3, handles.freq_target4, handles.freq_target5, ...
            handles.freq_target6, handles.freq_target7, handles.freq_target8};

        handles.enable_weight.Enable = 'on';

        for j = 1:num_target_freq
            tar_freq_index{j}.Enable = 'on';
        end

        for j = num_target_freq + 1:8
            tar_freq_index{j}.Enable = 'off';
        end

        handles.marker_to_recover.Enable = 'on';

        marker_recover = get(handles.marker_to_recover, 'value');
    end


% --- Executes during object creation, after setting all properties.
function number_freq_CreateFcn(hObject, eventdata, handles)
% hObject    handle to number_freq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on selection change in top_number_save.
function top_number_save_Callback(hObject, eventdata, handles)
% hObject    handle to top_number_save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns top_number_save contents as cell array
%        contents{get(hObject,'Value')} returns selected item from top_number_save
    global top_num_save
    top_num_save = get(handles.top_number_save, 'value');


% --- Executes during object creation, after setting all properties.
function top_number_save_CreateFcn(hObject, eventdata, handles)
% hObject    handle to top_number_save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function time_out_Callback(hObject, eventdata, handles)
% hObject    handle to time_out (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of time_out as text
%        str2double(get(hObject,'String')) returns contents of time_out as a double
     global t_delay;
     t_delay = str2double(get(handles.time_out, 'string'));


% --- Executes during object creation, after setting all properties.
function time_out_CreateFcn(hObject, eventdata, handles)
% hObject    handle to time_out (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in view_sweeping.
function view_sweeping_Callback(hObject, eventdata, handles)
% hObject    handle to view_sweeping (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of view_sweeping
    global view_sweeping;
    view_sweeping = get(handles.view_sweeping, 'value');


function view_sweep_interval_Callback(hObject, eventdata, handles)
% hObject    handle to view_sweep_interval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of view_sweep_interval as text
%        str2double(get(hObject,'String')) returns contents of view_sweep_interval as a double


% --- Executes during object creation, after setting all properties.
function view_sweep_interval_CreateFcn(hObject, eventdata, handles)
% hObject    handle to view_sweep_interval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




function [S_extremum, pattern_num, pattern_saved, S_param_measured] = preset_sweep_algorithm(serial_port_type, S_extremnum_selection, nanoVNA_port, freq_target_index, S_extremum, pattern_num, ...
          pattern_saved, S_param_measured, hObject, eventdata, handles)
    global pattern_array
    global len
    
    global enable_other_filterload
    global directly_coupled;

    %% Get information about what's inside folder.
    path_current = pwd;
    
    if (get(handles.application_type, 'value') == 1) && (get(handles.patch_type_selection, 'value') == 2)   % antenna & fork
        path = [path_current, '\fork_preset'];
        filenames = {dir(path).name};
    elseif (get(handles.application_type, 'value') == 1) && (get(handles.patch_type_selection, 'value') == 3)   % antenna & other preset
        path = [path_current, '\other_antenna_preset'];
        filenames = {dir(path).name};
    elseif get(handles.application_type, 'value') == 3  % filter
        path = [path_current, '\filter_preset'];
        filenames = {dir(path).name};
    end

    csvfiles = filenames(endsWith(filenames,'.csv'));
    csvfiles = natsortfiles(csvfiles);
    num_files = length(csvfiles);
    num_points = len;
    
    num_pattern = 1;

    if S_extremnum_selection == 0 % based on reflected S-param
        for k = 1:num_files
            pattern_array = csvread([path '\' char(csvfiles(k))]);
            [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(0, serial_port_type, 0, 1, nanoVNA_port, freq_target_index, S_extremum, pattern_num, ...
             pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
            num_pattern = num_pattern + 1;
        end
    elseif S_extremnum_selection == 1 % based on transferred S-param       
        for k = 1:num_files
            if enable_other_filterload
                pattern_array = csvread([path '\' char(csvfiles(k))]);
                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(0, serial_port_type, 3, 3, nanoVNA_port, freq_target_index, S_extremum, pattern_num, ...
                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                num_pattern = num_pattern + 1;
            elseif directly_coupled
                pattern_array = csvread([path '\' char(csvfiles(k))]);
                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(0, serial_port_type, 3, 4, nanoVNA_port, freq_target_index, S_extremum, pattern_num, ...
                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                num_pattern = num_pattern + 1;
            else
                pattern_array = csvread([path '\' char(csvfiles(k))]);
                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(0, serial_port_type, 3, 2, nanoVNA_port, freq_target_index, S_extremum, pattern_num, ...
                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                num_pattern = num_pattern + 1;
            end
        end
    else %==2  based on far-field gain
        for k = 1:num_files
            path
            pattern_array = csvread([path '\' char(csvfiles(k))]);
            [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(0, serial_port_type, 0, nanoVNA_port, freq_target_index, S_extremum, pattern_num, ...
             pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
            num_pattern = num_pattern + 1;
        end
    end
    
    %% end of S11 measuring

    


function [S_extremum, pattern_num, pattern_saved, S_param_measured] = rect_sweep_algorithm(serial_port_type, S_extremnum_selection, serial_port, freq_target_index, S_extremum, pattern_num, ...
          pattern_saved, S_param_measured, hObject, eventdata, handles)                   % serial_port_type: 0 for DC-AC, 1 for nano VNA, 2 for desktop VNA
    global pattern_mapped
    
    % finding the input port and output port setting
    input_port = cell2mat(split(get(handles.input_port, 'string'), '-'));
    in_port_1 = str2num(input_port(1));
    in_port_2 = str2num(input_port(2));
    pattern_mapped(in_port_1, in_port_2) = 1;

    num_pattern = 1;
    % input port locates at left side
    if (in_port_2 == 1) && (mod(in_port_1, 2) == 1)                    
        for p = 1:4                                                          % sweeping based on width of the patch antenna (number of squares)
            for q = max(1, in_port_1 - 2 * p):2:min(in_port_1, 9 - 2 * p)    % sweeping based on the outmost outline position (from small number side)
                for m = 1:4                                                  % sweeping based on length of feed line (number of segments including the input port)
                    for n = 1:5 - m                                          % sweeping based on length of the patch antenna pattern
                        pattern_mapped(in_port_1, 1:m) = 1;                             % the feed line

                        pattern_mapped(2*m : 2 : 2*(m+n-1), ((q+1)/2+1) : ((q+1)/2+1)+p-1) = 1;         % the bottom edge perpendicular to feed line
                        pattern_mapped(2*m+2*n, ((q+1)/2+1) : ((q+1)/2+1)+p-1) = 1;     % the top edge perpendicular to feed line
                        pattern_mapped(q : 2 : q+2*(p-1), m+1 : m+n) = 1;               % the left edge parallel to feed line
                        pattern_mapped(q+2*p, m+1 : m+n) = 1;                           % the right edge parallel to feed line

                        if S_extremnum_selection == 0
                            [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 1, 1, serial_port, freq_target_index, S_extremum, pattern_num, ...
                             pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                        else
                            [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 1, serial_port, freq_target_index, S_extremum, pattern_num, ...
                             pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                        end

                        pattern_mapped = zeros(10, 6);
                        num_pattern = num_pattern + 1;
                    end
                end
            end
        end
    end

    % input port locates at right side
    if (in_port_2 == 6) && (mod(in_port_1, 2) == 1)
        for p = 1:4                                                          % sweeping based on width of the patch antenna (number of squares)
            for q = max(1, in_port_1 - 2 * p):2:min(in_port_1, 9 - 2 * p)    % sweeping based on the outmost outline position (from small number side)
                for m = 1:4                                                  % sweeping based on length of feed line (number of segments including the input port)
                    for n = 1:5 - m                                          % sweeping based on length of the patch antenna pattern
                        pattern_mapped(in_port_1, 6:-1:6-m+1) = 1;                             % the feed line

                        pattern_mapped(2*(6-m) : -2 : 2*(7-m-n), ((q+1)/2+1) : ((q+1)/2+1)+p-1) = 1;         % the bottom edge perpendicular to feed line
                        pattern_mapped(2*(6-m)-2*n, ((q+1)/2+1) : ((q+1)/2+1)+p-1) = 1;     % the top edge perpendicular to feed line
                        pattern_mapped(q : 2 : q+2*(p-1), 6-m : -1 : 7-m-n) = 1;               % the right edge parallel to feed line
                        pattern_mapped(q+2*p, 6-m : -1 : 7-m-n) = 1;                           % the left edge parallel to feed line

                        if S_extremnum_selection == 0
                            [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 1, 1, serial_port, freq_target_index, S_extremum, pattern_num, ...
                             pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                        else
                            [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 1, serial_port, freq_target_index, S_extremum, pattern_num, ...
                             pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                        end

                        pattern_mapped = zeros(10, 6);
                        num_pattern = num_pattern + 1;
                    end
                end
            end
        end
    end


    % input port locates at top side
    if (in_port_2 == 1) && (mod(in_port_1, 2) == 0)                    
        for p = 1:4                                                          % sweeping based on width of the patch antenna (number of squares)
            for q = max(2, in_port_1 - 2 * p):2:min(in_port_1, 10 - 2 * p)    % sweeping based on the outmost outline position (from small number side)
                for m = 1:4                                                  % sweeping based on length of feed line (number of segments including the input port)
                    for n = 1:5 - m                                          % sweeping based on length of the patch antenna pattern
                        pattern_mapped(in_port_1, 1:m) = 1;                             % the feed line

                        pattern_mapped(2*m-1 : 2 : 2*(m+n)-3, (q/2+1) : (q/2+1)+p-1) = 1;         % the bottom edge perpendicular to feed line
                        pattern_mapped(2*m+2*n-1, (q/2+1) : (q/2+1)+p-1) = 1;     % the top edge perpendicular to feed line
                        pattern_mapped(q : 2 : q+2*(p-1), m+1 : m+n) = 1;               % the right edge parallel to feed line
                        pattern_mapped(q+2*p, m+1 : m+n) = 1;                           % the left edge parallel to feed line

                        if S_extremnum_selection == 0
                            [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 1, 1, serial_port, freq_target_index, S_extremum, pattern_num, ...
                             pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                        else
                            [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 1, serial_port, freq_target_index, S_extremum, pattern_num, ...
                             pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                        end

                        pattern_mapped = zeros(10, 6);
                        num_pattern = num_pattern + 1;
                    end
                end
            end
        end
    end


    % input port locates at bottom side
    if (in_port_2 == 6) && (mod(in_port_1, 2) == 0)                    
        for p = 1:4                                                          % sweeping based on width of the patch antenna (number of squares)
            for q = max(2, in_port_1 - 2 * p):2:min(in_port_1, 10 - 2 * p)    % sweeping based on the outmost outline position (from small number side)
                for m = 1:4                                                  % sweeping based on length of feed line (number of segments including the input port)
                    for n = 1:5 - m                                          % sweeping based on length of the patch antenna pattern
                        pattern_mapped(in_port_1, 6:-1:6-m+1) = 1;                             % the feed line

                        pattern_mapped(11-2*m : -2 : 13-2*m-2*n, (q/2+1) : (q/2+1)+p-1) = 1;      % the bottom edge perpendicular to feed line
                        pattern_mapped(11-2*m-2*n, (q/2+1) : (q/2+1)+p-1) = 1;     % the top edge perpendicular to feed line
                        pattern_mapped(q : 2 : q+2*(p-1), 6-m : -1 : 7-m-n) = 1;               % the right edge parallel to feed line
                        pattern_mapped(q+2*p, 6-m : -1 : 7-m-n) = 1;                           % the left edge parallel to feed line

                        if S_extremnum_selection == 0
                            [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 1, 1, serial_port, freq_target_index, S_extremum, pattern_num, ...
                             pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                        else
                            [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 1, serial_port, freq_target_index, S_extremum, pattern_num, ...
                             pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                        end
                        pattern_mapped = zeros(10, 6);
                        num_pattern = num_pattern + 1;
                    end
                end
            end
        end
    end
    
    
function [S_extremum, pattern_num, pattern_saved, S_param_measured] = IMN_sweep_algorithm(serial_port_type, S_extremnum_selection, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
          pattern_saved, S_param_measured, hObject, eventdata, handles)                   % serial_port_type: 0 for DC-AC, 1 for nano VNA, 2 for desktop VNA
    global pattern_mapped
    global patch_type_dec
      
    % finding the input port and output port setting
    input_port = cell2mat(split(get(handles.input_port, 'string'), '-'));
    in_port_1 = str2num(input_port(1));
    in_port_2 = str2num(input_port(2));
    pattern_mapped(in_port_1, in_port_2) = 1;
    
    output_port = cell2mat(split(get(handles.output_port, 'string'), '-'));
    out_port_1 = str2num(output_port(1));
    out_port_2 = str2num(output_port(2));
    pattern_mapped(out_port_1, out_port_2) = 1;
      
	IMN_stub_num = dec2bin(patch_type_dec, 2);
    num_pattern = 1;        % count pattern number
            
    % sweeping algorithm for IMN patterns
    if (mod(in_port_1, 2) == 1) && (mod(out_port_1, 2) == 1) && (in_port_2 < out_port_2)        % if first argument of IN and OUT are both odd, they are both on horizontal direction, and from left to right
        for m = 2:5
            pattern_mapped(in_port_1, m) = 1;            %define the horizontal part of the main line, the main line go horizontal to the last vertical segments and then go vertical
        end

        %define the vertical part of the main line, depending on the relative position of IN and OUT
        if in_port_1 == out_port_1                       % IN position is equal to OUT position 
            % sweep the branch stub combinations
            if str2num(IMN_stub_num(2))                  % one stub matching case
                if in_port_1 >= 5
                    for m = 0:2:10              % m = 0 for no any branch stub
                        if m > 2
                            pattern_mapped(m - 2, :) = 0;          % this is to clear the previous stub
                        end
                        for n = (in_port_1 + 1) / 2 : -1 : 1
                            if(m ~= 0)
                                pattern_mapped(m, n) = 1;
                                num_pattern = num_pattern + 1; 
                            end
                            
                            if S_extremnum_selection == 0
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 2, 1, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            else
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 2, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            end
                            
                            if m == 0
                                break
                            end
                        end                  
                    end
                else
                    for m = 0:2:10              % m = 0 for no any branch stub
                        if m > 2
                            pattern_mapped(m - 2, :) = 0;          % this is to clear the previous stub
                        end
                        for n = (in_port_1 + 1) / 2 + 1 : 1 : 6
                            if(m ~= 0)
                                pattern_mapped(m, n) = 1;
                                num_pattern = num_pattern + 1; 
                            end

                            if S_extremnum_selection == 0
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 2, 1, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            else
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 2, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            end

                            if m == 0
                                break
                            end
                        end                 
                    end
                end
                pattern_mapped(m, :) = 0;          % this is to clear the previous stub
            end
            if str2num(IMN_stub_num(1))                  % two stub matching case
                if in_port_1 >= 5
                    for m1 = 2:2:8              % m = 0 for no any branch stub
                        for n1 = (in_port_1 + 1) / 2 : -1 : 1
                            pattern_mapped(m1, n1) = 1;
                            for m2 = m1 + 2 : 2 : 10
                                for n2 = (in_port_1 + 1) / 2 : -1 : 1
                                    pattern_mapped(m2, n2) = 1;
                                    num_pattern = num_pattern + 1; 

                                    if S_extremnum_selection == 0
                                        [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 2, 1, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                         pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                                    else
                                        [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 2, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                         pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                                    end
                                    
                                end
                                pattern_mapped(m2, :) = 0;          % this is to clear the previous stub
                            end  
                        end 
                        pattern_mapped(m1, :) = 0;          % this is to clear the previous stub
                    end
                else
                    for m1 = 2:2:8              % m = 0 for no any branch stub
                        for n1 = (in_port_1 + 1) / 2 + 1: 1 : 6
                            pattern_mapped(m1, n1) = 1;
                            for m2 = m1 + 2 : 2 : 10
                                for n2 = (in_port_1 + 1) / 2 + 1 : 1 : 6
                                    pattern_mapped(m2, n2) = 1;
                                    num_pattern = num_pattern + 1; 

                                    if S_extremnum_selection == 0
                                        [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 2, 1, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                         pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                                    else
                                        [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 2, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                         pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                                    end
                                    
                                end
                                pattern_mapped(m2, :) = 0;          % this is to clear the previous stub
                            end  
                        end 
                        pattern_mapped(m1, :) = 0;          % this is to clear the previous stub
                    end
                end
            end
        end

        if in_port_1 < out_port_1                        % IN position is higher than OUT position 
            for m = in_port_1:2:out_port_1 - 2
                pattern_mapped(10, (m + 1) / 2 + 1) = 1;
            end

            % sweep the branch stub combinations
            if str2num(IMN_stub_num(2))                  % single stub matching case
                if in_port_1 >= 5               % branch stubs are above
                    for m = 0:2:10              % m = 0 for no any branch stub, this is for vertical branches
                        for n = (in_port_1 + 1) / 2 : -1 : 1
                            if(m ~= 0)
                                pattern_mapped(m, n) = 1;
                                num_pattern = num_pattern + 1; 
                            end

                            if S_extremnum_selection == 0
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 2, 1, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            else
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 2, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            end

                            if m == 0
                                break
                            end
                        end  
                        if m ~= 0
                            if m == 10
                                pattern_mapped(m, ((in_port_1 + 1) / 2: -1 :1)) = 0;
                            else
                                pattern_mapped(m, :) = 0;          % this is to clear the previous stub
                            end
                        end
                    end

                    for m = in_port_1 + 2 : 2 : out_port_1 % this is for horizontal branches
                        for n = 5:-1:1
                            pattern_mapped(m, n) = 1;
                            num_pattern = num_pattern + 1; 

                            if S_extremnum_selection == 0
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 2, 1, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            else
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 2, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            end

                            if m == 0
                                break
                            end
                        end
                        pattern_mapped(m, (1:5)) = 0;          % this is to clear the previous stub
                    end
                else                          % branch stubs are below
                    for m = 0:2:8              % m = 0 for no any branch stub
                        for n = (in_port_1 + 1) / 2 + 1 : 1 : 6
                            if(m ~= 0)
                                pattern_mapped(m, n) = 1;
                                num_pattern = num_pattern + 1;  
                            end

                            if S_extremnum_selection == 0
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 2, 1, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            else
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 2, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            end

                            if m == 0
                                break
                            end
                        end
                        if m ~= 0
                            pattern_mapped(m, :) = 0;          % this is to clear the previous stub
                        end
                    end
                    for n = (in_port_1 + 1) / 2 : -1 : 1
                        pattern_mapped(10, n) = 1;
                        num_pattern = num_pattern + 1; 

                        if S_extremnum_selection == 0
                            [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 2, 1, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                             pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                        else
                            [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 2, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                             pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                        end
                        
                        if m == 0
                            break
                        end
                    end
                    pattern_mapped(10, ((in_port_1 + 1) / 2: -1 :1)) = 0;
                    for m = in_port_1 + 2 : 2 : out_port_1 % this is for horizontal branches
                        for n = 5:-1:1
                            pattern_mapped(m, n) = 1;
                            num_pattern = num_pattern + 1; 

                            if S_extremnum_selection == 0
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 2, 1, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            else
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 2, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            end

                            if m == 0
                                break
                            end
                        end
                        pattern_mapped(m, (1:5)) = 0;          % this is to clear the previous stub
                    end
                end
            end
            if str2num(IMN_stub_num(1))                  % two stub matching case
                fprintf('Two stub matching for non-aligned input and output ports are not available.\n\n');
            end
        end

        if in_port_1 > out_port_1                        % IN position is lower than OUT position 
            for m = in_port_1:-2:out_port_1 + 2
                pattern_mapped(10, (m + 1) / 2) = 1;
            end

            % sweep the branch stub combinations
            if str2num(IMN_stub_num(2))                  % single stub matching case
                if in_port_1 <= 5               % branch stubs are below
                    for m = 0:2:10              % m = 0 for no any branch stub, this is for vertical branches
                        for n = (in_port_1 + 1) / 2 + 1: 1 : 6
                            if(m ~= 0)
                                pattern_mapped(m, n) = 1;
                                num_pattern = num_pattern + 1; 
                            end

                            if S_extremnum_selection == 0
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 2, 1, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            else
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 2, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            end

                            if m == 0
                                break
                            end
                        end  
                        if m ~= 0
                            if m == 10
                                pattern_mapped(m, ((in_port_1 + 1) / 2 + 1: 1 :6)) = 0;
                            else
                                pattern_mapped(m, :) = 0;          % this is to clear the previous stub
                            end
                        end
                    end

                    for m = in_port_1 - 2 : -2 : out_port_1 % this is for horizontal branches
                        for n = 5:-1:1
                            pattern_mapped(m, n) = 1;
                            num_pattern = num_pattern + 1; 

                            if S_extremnum_selection == 0
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 2, 1, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            else
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 2, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            end

                            if m == 0
                                break
                            end
                        end
                        pattern_mapped(m, (1:5)) = 0;          % this is to clear the previous stub
                    end
                else                          % branch stubs are above
                    for m = 0:2:8              % m = 0 for no any branch stub
                        for n = (in_port_1 + 1) / 2 : -1 : 1
                            if(m ~= 0)
                                pattern_mapped(m, n) = 1;
                                num_pattern = num_pattern + 1; 
                            end

                            if S_extremnum_selection == 0
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 2, 1, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            else
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 2, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            end

                            if m == 0
                                break
                            end
                        end
                        if m ~= 0
                            pattern_mapped(m, :) = 0;          % this is to clear the previous stub
                        end
                    end
                    for n = (in_port_1 + 1) / 2 + 1: 1 : 6
                        pattern_mapped(10, n) = 1;
                        num_pattern = num_pattern + 1; 

                        if S_extremnum_selection == 0
                            [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 2, 1, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                             pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                        else
                            [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 2, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                             pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                        end

                        if m == 0
                            break
                        end
                    end
                    pattern_mapped(10, ((in_port_1 + 1) / 2 + 1: 1 :6)) = 0;
                    for m = in_port_1 - 2 : -2 : out_port_1 % this is for horizontal branches
                        for n = 5:-1:1
                            pattern_mapped(m, n) = 1;
                            num_pattern = num_pattern + 1; 

                            if S_extremnum_selection == 0
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure(1, serial_port_type, 2, 1, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            else
                                [S_extremum, pattern_num, pattern_saved, S_param_measured] = pattern_config_and_measure_gain(1, serial_port_type, 2, serialVNA_port, freq_target_index, S_extremum, pattern_num, ...
                                 pattern_saved, S_param_measured, num_pattern, hObject, eventdata, handles);
                            end

                            if m == 0
                                break
                            end
                        end
                        pattern_mapped(m, (1:5)) = 0;          % this is to clear the previous stub
                    end
                end
            end
            if str2num(IMN_stub_num(1))                  % two stub matching case
                fprintf('Two stub matching for non-aligned input and output ports are not available.\n\n');
            end
        end
    end    


% --- Executes on selection change in S_min_max_selection.
function S_min_max_selection_Callback(hObject, eventdata, handles)
% hObject    handle to S_min_max_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns S_min_max_selection contents as cell array
%        contents{get(hObject,'Value')} returns selected item from S_min_max_selection
    global S_opti_min_max_selection
    S_opti_min_max_selection = get(handles.S_min_max_selection, 'Value');


% --- Executes during object creation, after setting all properties.
function S_min_max_selection_CreateFcn(hObject, eventdata, handles)
% hObject    handle to S_min_max_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in application_for_opti.
function application_for_opti_Callback(hObject, eventdata, handles)
% hObject    handle to application_for_opti (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns application_for_opti contents as cell array
%        contents{get(hObject,'Value')} returns selected item from application_for_opti
    global application_for_opti
    application_for_opti = get(handles.application_for_opti, 'Value');
    

% --- Executes during object creation, after setting all properties.
function application_for_opti_CreateFcn(hObject, eventdata, handles)
% hObject    handle to application_for_opti (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function filter_load_name_Callback(hObject, eventdata, handles)
% hObject    handle to filter_load_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of filter_load_name as text
%        str2double(get(hObject,'String')) returns contents of filter_load_name as a double


% --- Executes during object creation, after setting all properties.
function filter_load_name_CreateFcn(hObject, eventdata, handles)
% hObject    handle to filter_load_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in enable_other_filterload.
function enable_other_filterload_Callback(hObject, eventdata, handles)
% hObject    handle to enable_other_filterload (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable_other_filterload
    global enable_other_filterload;
    enable_other_filterload = get(handles.enable_other_filterload, 'value');
    
    if get(handles.enable_other_filterload, 'Value')
        handles.filter_load_name.Enable = 'on';
    else
        handles.filter_load_name.Enable = 'off';
    end



function rfmeter_COM_port_Callback(hObject, eventdata, handles)
% hObject    handle to rfmeter_COM_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rfmeter_COM_port as text
%        str2double(get(hObject,'String')) returns contents of rfmeter_COM_port as a double


% --- Executes during object creation, after setting all properties.
function rfmeter_COM_port_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rfmeter_COM_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in directly_coupled.
function directly_coupled_Callback(hObject, eventdata, handles)
% hObject    handle to directly_coupled (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of directly_coupled
    global directly_coupled;
    directly_coupled = get(handles.directly_coupled, 'value');


% --- Executes on button press in debug_mode.
function debug_mode_Callback(hObject, eventdata, handles)
% hObject    handle to debug_mode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of debug_mode
    global rfmeter_debug;
    
    rfmeter_debug = get(handles.debug_mode, 'value');


% --- Executes on selection change in self_adapt_base.
function self_adapt_base_Callback(hObject, eventdata, handles)
% hObject    handle to self_adapt_base (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns self_adapt_base contents as cell array
%        contents{get(hObject,'Value')} returns selected item from self_adapt_base
    global selfadapt_base;
    selfadapt_base = get(handles.self_adapt_base, 'value');


% --- Executes during object creation, after setting all properties.
function self_adapt_base_CreateFcn(hObject, eventdata, handles)
% hObject    handle to self_adapt_base (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in hide_sweeping.
function hide_sweeping_Callback(hObject, eventdata, handles)
% hObject    handle to hide_sweeping (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of hide_sweeping
    global hide_sweeping_process;
    hide_sweeping_process = get(handles.hide_sweeping, 'value');
    
    
function ESP32_COM_Callback(hObject, eventdata, handles)
% hObject    handle to ESP32_COM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ESP32_COM as text
%        str2double(get(hObject,'String')) returns contents of ESP32_COM as a double


% --- Executes during object creation, after setting all properties.
function ESP32_COM_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ESP32_COM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pattern_interval_Callback(hObject, eventdata, handles)
% hObject    handle to pattern_interval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pattern_interval as text
%        str2double(get(hObject,'String')) returns contents of pattern_interval as a double


% --- Executes during object creation, after setting all properties.
function pattern_interval_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pattern_interval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in enable_self_adaptive.
function enable_self_adaptive_Callback(hObject, eventdata, handles)
% hObject    handle to enable_self_adaptive (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable_self_adaptive

    global enable_selfadaptive;
    enable_selfadaptive = get(handles.enable_self_adaptive, 'value')
    global selfadapt_base;
    selfadapt_base = get(handles.self_adapt_base, 'value');

    global selfadapt_error;
    selfadapt_error = 1 - get(handles.self_adaptive_tolerance, 'value');
    
    global s_MCU_DC;
    %global RFmeter_initialized;
    global ADC_rfmeter_current;
    global ADC_extremum;
    
    global nanoVNA_initialized;
    global s_nanoVNA;
    global nanoVNA_extremum;
    
    global application_type_dec;
    global S_opti_min_max_selection;
    global enable_other_filterload
    global directly_coupled;
    
    global multifreq_sweep;
    multifreq_sweep = get(handles.enable_multifreq, 'value');


    tar_freq_index = {handles.freq_target1, handles.freq_target2, handles.freq_target3, handles.freq_target4, handles.freq_target5, ...
            handles.freq_target6, handles.freq_target7, handles.freq_target8};
    
    tar_freq_weight_index = {handles.weight1, handles.weight2, handles.weight3, handles.weight4, handles.weight5, ...
            handles.weight6, handles.weight7, handles.weight8};

    if ~get(handles.enable_self_adaptive, 'Value')
        
        
        handles.self_adaptive_tolerance.Enable = 'on';
            
        handles.S_optimize.Enable = 'on';
        handles.gain_optimize.Enable = 'on';
        handles.view_sweeping.Enable = 'on';
        handles.view_sweep_interval.Enable = 'on';
        
        handles.enable_multifreq.Enable = 'on';
        handles.enable_weight.Enable = 'on';
        handles.marker_to_recover.Enable = 'on';
        handles.number_freq.Enable = 'on';
        handles.top_number_save.Enable = 'on';
        handles.num_localmin.Enable = 'on';
        handles.smooth_index.Enable = 'on';

        handles.save_sweep_S_file.Enable = 'on';
        handles.save_sweep_pattern_file.Enable = 'on';
        handles.save_opti_S.Enable = 'on';
        handles.save_opti_pattern.Enable = 'on';
        handles.save_sweep_filename.Enable = 'on';
        
        
        for i = 1:get(handles.number_freq, 'value')
            tar_freq_index{i}.Enable = 'on';
        end
        
        if get(handles.enable_weight, 'value')
            for i = 1:get(handles.number_freq, 'value')
                tar_freq_weight_index{i}.Enable = 'on';
            end
        end
        
        handles.enable_other_filterload.Enable = 'on';
        handles.directly_coupled.Enable = 'on';
        handles.filter_load_name.Enable = 'on';

    else
        handles.self_adaptive_tolerance.Enable = 'on';

        handles.S_optimize.Enable = 'off';
        handles.gain_optimize.Enable = 'off';
        handles.view_sweeping.Enable = 'off';
        handles.view_sweep_interval.Enable = 'off';
        
        handles.enable_multifreq.Enable = 'off';
        handles.enable_weight.Enable = 'off';
        handles.marker_to_recover.Enable = 'off';
        handles.number_freq.Enable = 'off';
        handles.top_number_save.Enable = 'off';
        handles.num_localmin.Enable = 'off';
        handles.smooth_index.Enable = 'off';

        
        set(handles.save_opti_pattern,'value',0);
        set(handles.save_sweep_pattern_file,'value',0);
        set(handles.save_sweep_S_file,'value',0);
        set(handles.save_opti_S,'value',0);
        handles.save_sweep_S_file.Enable = 'off';
        handles.save_sweep_pattern_file.Enable = 'off';
        handles.save_opti_S.Enable = 'off';
        handles.save_opti_pattern.Enable = 'off';
        handles.save_sweep_filename.Enable = 'off';
       
        
        handles.freq_target1.Enable = 'off';
        handles.freq_target2.Enable = 'off';
        handles.freq_target3.Enable = 'off';
        handles.freq_target4.Enable = 'off';
        handles.freq_target5.Enable = 'off';
        handles.freq_target6.Enable = 'off';
        handles.freq_target7.Enable = 'off';
        handles.freq_target8.Enable = 'off';
        
        handles.weight1.Enable = 'off';
        handles.weight2.Enable = 'off';
        handles.weight3.Enable = 'off';
        handles.weight4.Enable = 'off';
        handles.weight5.Enable = 'off';
        handles.weight6.Enable = 'off';
        handles.weight7.Enable = 'off';
        handles.weight8.Enable = 'off';
        
        handles.enable_other_filterload.Enable = 'off';
        handles.directly_coupled.Enable = 'off';
        handles.filter_load_name.Enable = 'off';
        

        %start sweeping and updating process
        if selfadapt_base == 1 %s-param based
            %1. run the S-param opti for the first time and save the extrenum and pattern; update the pattern
            S_optimize_Callback(hObject, eventdata, handles);
            
            if ~((application_type_dec == 3) && (S_opti_min_max_selection == 2)) % based on min
                rfmeter_current = save_VDC_rfmeter(hObject, eventdata, handles);
                if ADC_extremum < rfmeter_current
                    ADC_extremum = rfmeter_current;
                end
            end
                
            if get(handles.feedback_source, 'Value') == 3
                ADC_extremum;
                set(handles.saved_extremum, 'String', num2str(ADC_extremum));
            else
                nanoVNA_extremum;
                set(handles.saved_extremum, 'String', num2str(nanoVNA_extremum));
            end
            
            while enable_selfadaptive
                if get(handles.feedback_source, 'Value') == 3
                    %pause(0.1);
                    %2. keep reading from the RF meter and compare it with the error tolerance
                    rfmeter_current = save_VDC_rfmeter(hObject, eventdata, handles);
                    set(handles.current_reading, 'String', num2str(rfmeter_current));
                    %ADC_extremum - rfmeter_current

                    %ADC_rfmeter_current - ADC_extremum
                    %3. if the error is larger, redo the sweep and update
                    if (application_type_dec == 3) && (S_opti_min_max_selection == 2) % based on max
                        if abs(rfmeter_current - ADC_extremum) > selfadapt_error * 300
                            fprintf("\nUpdating FPRFS pattern due to disturbance...");
                            S_optimize_Callback(hObject, eventdata, handles);
                            set(handles.saved_extremum, 'String', num2str(ADC_extremum));
                            %ADC_extremum
                        end
                    else                                                 % based on min
                        if abs(ADC_extremum - rfmeter_current) > selfadapt_error * 300
                            fprintf("\nUpdating FPRFS pattern due to disturbance...");
                            S_optimize_Callback(hObject, eventdata, handles);
                            
                            if ADC_extremum < rfmeter_current
                                ADC_extremum = rfmeter_current;
                            end
                            
                            set(handles.saved_extremum, 'String', num2str(ADC_extremum));
                        end
                    end
                elseif get(handles.feedback_source, 'Value') == 1
                    fprintf("in loop\n");
                    
                    if (application_type_dec == 1) || (application_type_dec == 2)
                        S_measure = 1;
                    else
                        if enable_other_filterload
                            S_measure = 3;
                        elseif directly_coupled
                            S_measure = 4;
                        else
                            S_measure = 2;
                        end
                    end
                    
                    if ~nanoVNA_initialized
                        s_nanoVNA = get_serial_port_nanoVNA(hObject, eventdata, handles);
                        fopen(s_nanoVNA);
                        
                        get_nanoVNA_parameters();
                        
                        nanoVNA_initialized = 1;
                    end

                    nanoVNA_current = 20 * log10(abs(save_S21(S_measure, s_nanoVNA, multifreq_sweep, hObject, eventdata, handles)));
                    fclose(s_nanoVNA);
                    nanoVNA_initialized = 0;

                    set(handles.current_reading, 'String', num2str(nanoVNA_current));
                    
                    if abs(application_type_dec == 3) && (S_opti_min_max_selection == 2) % based on max
                        if abs(nanoVNA_current - nanoVNA_extremum) > selfadapt_error * 20 %db
                            fprintf("\nUpdating FPRFS pattern due to disturbance...");
                            S_optimize_Callback(hObject, eventdata, handles);
                            set(handles.saved_extremum, 'String', num2str(nanoVNA_extremum));
                        end
                    else
                        if abs(nanoVNA_extremum - nanoVNA_current) > selfadapt_error * 20 %db
                            fprintf("\nUpdating FPRFS pattern due to disturbance...");
                            S_optimize_Callback(hObject, eventdata, handles);
                            set(handles.saved_extremum, 'String', num2str(nanoVNA_extremum));
                        end
                    end                
                end
            end
        elseif selfadapt_base == 2 %gain based
            %1. run the S-param opti for the first time and save the extrenum and pattern; update the pattern
            gain_optimize_Callback(hObject, eventdata, handles);
            if get(handles.feedback_source, 'Value') == 3
                ADC_extremum;
                set(handles.saved_extremum, 'String', num2str(ADC_extremum));
            else
                nanoVNA_extremum;
                set(handles.saved_extremum, 'String', num2str(nanoVNA_extremum));
            end
            
            while enable_selfadaptive
                
                rfmeter_current = save_VDC_rfmeter(hObject, eventdata, handles);
                set(handles.current_reading, 'String', num2str(rfmeter_current));
                
                if get(handles.feedback_source, 'Value') == 3 
                
                    %2. keep reading from the RF meter and compare it with the error tolerance
                    rfmeter_current = save_VDC_rfmeter(hObject, eventdata, handles);

                    %3. if the error is larger, redo the sweep and update
                    if S_opti_min_max_selection == 2 % based on max
                        if abs(rfmeter_current - ADC_extremum) > selfadapt_error * 300
                            fprintf("\nUpdating FPRFS pattern due to disturbance...");
                            gain_optimize_Callback(hObject, eventdata, handles);
                            set(handles.saved_extremum, 'String', num2str(ADC_extremum));
                        end
                    else                             % based on min
                        if abs(ADC_extremum - rfmeter_current) > selfadapt_error * 300
                            fprintf("\nUpdating FPRFS pattern due to disturbance...");
                            gain_optimize_Callback(hObject, eventdata, handles);
                            set(handles.saved_extremum, 'String', num2str(ADC_extremum));
                        end
                    end
                elseif get(handles.feedback_source, 'Value') == 1
                    if ~nanoVNA_initialized
                        s_nanoVNA = get_serial_port_nanoVNA(hObject, eventdata, handles);
                        
                        get_nanoVNA_parameters();
                        
                        nanoVNA_initialized = 1;
                    end
                    
                    nanoVNA_current = save_S21_raw(s_nanoVNA);
                    
                    if abs(application_type_dec == 3) && (S_opti_min_max_selection == 2) % based on max
                        if abs(nanoVNA_current - nanoVNA_extremum) > selfadapt_error * 20 %db
                            fprintf("\nUpdating FPRFS pattern due to disturbance...");
                            S_optimize_Callback(hObject, eventdata, handles);
                        end
                    else
                        if abs(nanoVNA_extremum - nanoVNA_current) > selfadapt_error * 20 %db
                            fprintf("\nUpdating FPRFS pattern due to disturbance...");
                            S_optimize_Callback(hObject, eventdata, handles);
                        end
                    end                
                end
            end
        end     
    end
