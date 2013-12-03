function varargout = rov_property2(varargin)
% ROV_PROPERTY2 M-file for rov_property2.fig
%      ROV_PROPERTY2, by itself, creates a new ROV_PROPERTY2 or raises the existing
%      singleton*.
%
%      H = ROV_PROPERTY2 returns the handle to a new ROV_PROPERTY2 or the handle to
%      the existing singleton*.
%
%      ROV_PROPERTY2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROV_PROPERTY2.M with the given input arguments.
%
%      ROV_PROPERTY2('Property','Value',...) creates a new ROV_PROPERTY2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before rov_property2_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to rov_property2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Copyright 2002-2003 The MathWorks, Inc.

% Edit the above text to modify the response to help rov_property2

% Last Modified by GUIDE v2.5 30-Aug-2007 09:10:50

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @rov_property2_OpeningFcn, ...
                   'gui_OutputFcn',  @rov_property2_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
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


% --- Executes just before rov_property2 is made visible.
function rov_property2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to rov_property2 (see VARARGIN)

% Choose default command line output for rov_property2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes rov_property2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = rov_property2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in loadrov.
function loadrov_Callback(hObject, eventdata, handles)
% hObject    handle to loadrov (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clc
cd c:\Matlab7\toolbox\rov_design_analysis\hybrid
open rrcrov2_para

% --- Executes on button press in showdynamic.
function showdynamic_Callback(hObject, eventdata, handles)
% hObject    handle to showdynamic (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cd c:\Matlab7\toolbox\rov_design_analysis\m_file
load rrcrovf
run Analy_ss

% --- Executes on button press in conditionnumber.
function conditionnumber_Callback(hObject, eventdata, handles)
% hObject    handle to conditionnumber (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc
load rrcrovf
sys=ss(Ao,Bo,Co,Do);

GT=tf([0.97],[0.02 1]);
G44=[GT 0 0 0; 0 GT 0 0; 0 0 GT 0; 0 0 0 GT];
G44_thruster=G44*pinv(T);

sys_thruster=canon(G44_thruster,'modal');
[AT2,BT2,CT2,DT2]=ssdata(sys_thruster);

sys_tot=sys*sys_thruster;
[At,Bt,Ct,Dt]=ssdata(sys_tot);
cond(At)

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clc
load rrcrovf
sys=ss(Ao,Bo,Co,Do);
GT=tf([0.97],[0.02 1]);
G44=[GT 0 0 0; 0 GT 0 0; 0 0 GT 0; 0 0 0 GT];
G44_thruster=G44*pinv(T);

sys_thruster=canon(G44_thruster,'modal');
[AT2,BT2,CT2,DT2]=ssdata(sys_thruster);

sys_tot=sys*sys_thruster;
[At,Bt,Ct,Dt]=ssdata(sys_tot);

tzero(sys_tot)

% --- Executes on button press in contrb_obsvr.
function contrb_obsvr_Callback(hObject, eventdata, handles)
% hObject    handle to contrb_obsvr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc
load rrcrovf

rank(ctrb(Ao,Bo))    
rank(obsv(Ao,Co))

% --- Executes on button press in bandwidth.
function bandwidth_Callback(hObject, eventdata, handles)
% hObject    handle to bandwidth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cd c:\Matlab7\toolbox\rov_design_analysis\m_file ;
load rrcrovf
run Analy_ss
clc
run Analy_band   

% --- Executes on button press in coupling.
function coupling_Callback(hObject, eventdata, handles)
% hObject    handle to coupling (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cd c:\Matlab7\toolbox\rov_design_analysis\m_file ;
load rrcrovf
run Analy_ss
clc
run Analy_ger 

% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clc
load rrcrovf
run Analy_ss
clc
run numerical


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc


% --- Executes on button press in loadrov_nonlinear.
function loadrov_nonlinear_Callback(hObject, eventdata, handles)
% hObject    handle to loadrov_nonlinear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc
load rrcrovf

% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cd c:\Matlab7\toolbox\rov_design_analysis\hybrid ;
open controllable_property

% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cd c:\Matlab7\toolbox\rov_design_analysis\hybrid ;
open thruster_openloop

% --- Executes on button press in nonlin_coupling.
function nonlin_coupling_Callback(hObject, eventdata, handles)
% hObject    handle to nonlin_coupling (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cd c:\Matlab7\toolbox\rov_design_analysis\m_file ;
run state_coupling

% --- Executes on button press in nonlin_uniqueness.
function nonlin_uniqueness_Callback(hObject, eventdata, handles)
% hObject    handle to nonlin_uniqueness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cd c:\Matlab7\toolbox\rov_design_analysis\m_file ;
open nonlin_unique

% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cd c:\Matlab7\toolbox\rov_design_analysis\hybrid ;
open nonlin_unique

% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cd c:\Matlab7\toolbox\rov_design_analysis\hybrid ;
run rov_property


function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double
cd c:\Matlab7\toolbox\rov_design_analysis\m_file ;
run rov_property

% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on button press in nonlin_stabilizability.
function nonlin_stabilizability_Callback(hObject, eventdata, handles)
% hObject    handle to nonlin_stabilizability (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cd c:\Matlab7\toolbox\rov_design_analysis\hybrid ;
open controllable_property

% --------------------------------------------------------------------
function File_menu_Callback(hObject, eventdata, handles)
% hObject    handle to File_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  

       


% --------------------------------------------------------------------
function Help_menu_Callback(hObject, eventdata, handles)
% hObject    handle to Help_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)




% --------------------------------------------------------------------
function Open_menu_Callback(hObject, eventdata, handles)
% hObject    handle to Open_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end


% --------------------------------------------------------------------
function Print_menu_Callback(hObject, eventdata, handles)
% hObject    handle to Print_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)


% --------------------------------------------------------------------
function Close_menu_Callback(hObject, eventdata, handles)
% hObject    handle to Close_menu (see GCBO)
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
function helpmenu_Callback(hObject, eventdata, handles)
% hObject    handle to helpmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

str1=sprintf('%s','RDA-Analysis Section ver 1.0');
helpdlg('Copyright 2007, NTU. All rights reserved. GUI Designed by Cheng Siong Chin','RDA-Analysis Section ver 1.0')




% --- Executes on button press in checkbox3.
function checkbox3_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox3
clear all

