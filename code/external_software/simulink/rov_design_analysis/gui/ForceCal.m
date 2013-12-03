function varargout = ForceCal(varargin)
%FORCECAL M-file for ForceCal.fig
%      FORCECAL, by itself, creates a new FORCECAL or raises the existing
%      singleton*.
%
%      H = FORCECAL returns the handle to a new FORCECAL or the handle to
%      the existing singleton*.
%
%      FORCECAL('Property','Value',...) creates a new FORCECAL using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to ForceCal_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      FORCECAL('CALLBACK') and FORCECAL('CALLBACK',hObject,...) call the
%      local function named CALLBACK in FORCECAL.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ForceCal

% Last Modified by GUIDE v2.5 08-May-2006 11:22:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ForceCal_OpeningFcn, ...
                   'gui_OutputFcn',  @ForceCal_OutputFcn, ...
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


% --- Executes just before ForceCal is made visible.
function ForceCal_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for ForceCal
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ForceCal wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ForceCal_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[filename, pathname] = uigetfile( ...
    {'*.mat', 'All MAT-Files (*.mat)'; ...
        '*.*','All Files (*.*)'}, ...
    'Select Address Book');
% If "Cancel" is selected then return
if isequal([filename,pathname],[0,0])
    return
    % Otherwise construct the fullfilename and Check and load the file.
else
    File = fullfile(pathname,filename);
    % if the MAT-file is not valid, do not save the name
    if Check_And_Load(File,handles)
        handles.LastFIle = File;
        guidata(h,handles)
    end
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)


% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)

% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)




% --- Executes on button press in calculatetether.
function calculatetether_Callback(hObject, eventdata, handles)
% hObject    handle to calculatetether (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

sol=cable3dbvp([4 5 300],300);

%strFx=sprintf('%d, ',floor(sol));
str2=sprintf('%1.2f %1.2f %1.2f  ',sol.parameters(1),sol.parameters(2),sol.parameters(3))

set(handles.tetherforce2,'String',str2);



function tether_menu_Callback(hObject, eventdata, handles)
% hObject    handle to tether_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

str1=sprintf('%s \n ','The tether is divided into n equal length elements and the result ',...
    'from one element is propagated into next till it reaches the final ',...
    'end point at the ROV CG. The inertial reference frame (X, Y, Z)',...
    'is defined at surface of waterline and the first cables element is ',...
    'attached to the launch boat. The tether forces in three-dimensional ',...
    '(3D) are computed by the method in Sagatun that uses Catenary ',...
    'equations with end forces estimates.',...
     '------------------------------------------------------------- ',...
    'The initial guess for the forces in N is [4 5 180]^T ',... 
'cable length is 300m, cable weight is Wc = 1N/m',... 
'diameter dc = 0.014m, modulus of elasticity Ec=200x10^9 N/m2',...
'axial stiffness EA=3 x10^4 N and density p=662.2kg/m3.',...
'The launch boat positions (in X and Y directions) are assumed',... 
'to move by 0 to 20 m due to the wave that causes the boat to ',...
'drift from its initial position. Note that even if the initial ',...
'guess for the estimated force deviated from the correct value ',...
'the routine managed to find a good estimate.' );

set(handles.abtforce_txt,'String',str1);
% --------------------------------------------------------------------
function abt_force_Callback(hObject, eventdata, handles)
% hObject    handle to abt_force (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


str1=sprintf('%s \n ','ROV Design and Analysis (RDA) provides the necessary resources',...
    'for rapid implementation of mathematical models of ROV systems',...
    'with focus on ROV modeling, control system design and analysis.',...
    'The platform adopted for the development of RDA is MATLAB/SIMULINK.',...
    'This allows a modular simulator structure using the SIMULINK toolbox',...
    'library to be used and it enables a systematic reuse of knowledge and',...
    'results in efficient tools for research and education. ');

set(handles.abtforce_txt,'String',str1);



function tetherforce2_Callback(hObject, eventdata, handles)
% hObject    handle to tetherforce2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tetherforce2 as text
%        str2double(get(hObject,'String')) returns contents of tetherforce2 as a double


% --- Executes during object creation, after setting all properties.
function tetherforce2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tetherforce2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

function addedmass_Callback(hObject, eventdata, handles)
% hObject    handle to tetherforce2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tetherforce2 as text
%        str2double(get(hObject,'String')) returns contents of tetherforce2 as a double


% --- Executes during object creation, after setting all properties.
function addedmass_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tetherforce2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

function dragforce_Callback(hObject, eventdata, handles)
% hObject    handle to tetherforce2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tetherforce2 as text
%        str2double(get(hObject,'String')) returns contents of tetherforce2 as a double


% --- Executes during object creation, after setting all properties.
function dragforce_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tetherforce2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on button press in pushbutton_tetherfig.
function pushbutton_tetherfig_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_tetherfig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

W=[0 0 -1]'; %Constant distributed force
Fguess=[4 5 180]'; %Guess for end force EndP (We know this guess is wrong)
E=200e9; %Modulus of elasticity
D=0.014; %Cable diameter
A=pi*(D/2)^2; %Cable cross-sectional area
L=300;


sol=cable3dbvp([4 5 300],300);
axes(handles.figforce);
plot(sol.y(1,:),sol.y(3,:),'k--')
xlabel('x(m)')
ylabel('z(m)')
title('Cable bvp')


% --- Executes on button press in cal_disturb.
function cal_disturb_Callback(hObject, eventdata, handles)
% hObject    handle to cal_disturb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

warndlg('It will take a few minutes.......','Message','modal')
[Fx,Fy]=cablesolxy;