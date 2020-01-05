function varargout = trackBall(varargin)
% TRACKBALL MATLAB code for trackBall.fig
%      TRACKBALL, by itself, creates a new TRACKBALL or raises the existing
%      singleton*.
%
%      H = TRACKBALL returns the handle to a new TRACKBALL or the handle to
%      the existing singleton*.
%
%      TRACKBALL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TRACKBALL.M with the given input arguments.
%
%      TRACKBALL('Property','Value',...) creates a new TRACKBALL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before trackBall_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to trackBall_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help trackBall

% Last Modified by GUIDE v2.5 03-Jan-2020 20:15:23

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @trackBall_OpeningFcn, ...
                   'gui_OutputFcn',  @trackBall_OutputFcn, ...
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


% --- Executes just before trackBall is made visible.
function trackBall_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to trackBall (see VARARGIN)


set(hObject,'WindowButtonDownFcn',{@my_MouseClickFcn,handles.axes1});
set(hObject,'WindowButtonUpFcn',{@my_MouseReleaseFcn,handles.axes1});
axes(handles.axes1);


handles.Cube=DrawCube(eye(3));
set(handles.axes1,'CameraPosition',...
    [0 0 5],'CameraTarget',...
    [0 0 -5],'CameraUpVector',...
    [0 1 0],'DataAspectRatio',...
    [1 1 1]);

set(handles.axes1,'xlim',[-3 3],'ylim',[-3 3],'visible','off','color','none');

% Choose default command line output for trackBall
handles.output = hObject;

%Global var initialitzation
global old_quat;
old_quat = [1;0;0;0];
SetVariableGlobal_old_rot([1;0;0;0]);

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes trackBall wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = trackBall_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function my_MouseClickFcn(obj,event,hObject)

handles=guidata(obj);
xlim = get(handles.axes1,'xlim');
ylim = get(handles.axes1,'ylim');
mousepos=get(handles.axes1,'CurrentPoint');
xmouse = mousepos(1,1);
ymouse = mousepos(1,2);

if xmouse > xlim(1) && xmouse < xlim(2) && ymouse > ylim(1) && ymouse < ylim(2)

    set(handles.figure1,'WindowButtonMotionFcn',{@my_MouseMoveFcn,hObject});
    
    x_click = xmouse;
    SetVariableGlobal_xclick(x_click);
    y_click = ymouse;
    SetVariableGlobal_yclick(y_click);

end
guidata(hObject,handles)

function my_MouseReleaseFcn(obj,event,hObject)
handles=guidata(hObject);
set(handles.figure1,'WindowButtonMotionFcn','');

global old_quat;
global old_rot;

 %rota_quat * old_rota_quat
 q = old_quat;
 p = old_rot;
    
 r_quat = [ (q(1)*p(1)) - (transpose(q(2:4))*p(2:4)); (q(1)*p(2:4)) + (p(1) * q(2:4)) + (cross(q(2:4), p(2:4)))];
    
SetVariableGlobal_old_rot(r_quat);

guidata(hObject,handles);

function my_MouseMoveFcn(obj,event,hObject)

handles=guidata(obj);
xlim = get(handles.axes1,'xlim');
ylim = get(handles.axes1,'ylim');
mousepos=get(handles.axes1,'CurrentPoint');
xmouse = mousepos(1,1);
ymouse = mousepos(1,2);

% -----
i_vect = zeros(3, 1);
f_vect = zeros(3, 1);

if xmouse > xlim(1) && xmouse < xlim(2) && ymouse > ylim(1) && ymouse < ylim(2)
    
    r = sqrt(3);
    % Get the previously stored mouse position x-y
    m_x = GetVariableGlobal_xclick;
    m_y = GetVariableGlobal_yclick;
    
    %Start calculating the next rotation-------
    global old_quat;
    global old_rot;
    
    %Old pos
     if((m_x^2 + m_y^2) < 0.5 * r^2)
        i_vect = [m_x; m_y; sqrt(r^2 - m_x^2 - m_y^2)];
     else
        i_vect= [m_x; m_y; (r^2) / (2 * sqrt(m_x^2 + m_y^2))]; % Apply formula
        i_vect= (r * i_vect) / norm(i_vect); % Make sure this vector is normalized!!
    end
    
    %New pos
    if xmouse^2 + ymouse^2 < 0.5 * power(r, 2)
        f_vect = [xmouse; ymouse; sqrt(r^2 - xmouse^2 - ymouse^2)];
    else 
        vecModule = norm([xmouse, ymouse, (r^2 / (2*sqrt(xmouse^2 + ymouse^2)))]);
        f_vect = (r * [xmouse; ymouse; (r^2 / (2*sqrt(xmouse^2 + ymouse^2)))]) / vecModule;
    end

    r_axis = cross(i_vect, f_vect) / norm(cross(i_vect, f_vect)); % Get rotation axis
    r_angle = acos((transpose(f_vect) * i_vect)/(norm(f_vect) * norm(i_vect)))

    rotation_quaternion = [cos(r_angle/2);sin(r_angle/2)* r_axis];
    
    %rota_quat * old_rota_quat
    q = rotation_quaternion;
    p = old_rot;
    
    r_quat = [ (q(1)*p(1)) - (transpose(q(2:4))*p(2:4)); (q(1)*p(2:4)) + (p(1) * q(2:4)) + (cross(q(2:4), p(2:4)))];
    

    %R = rotQua2M(r_quat);
    R = rotQua2M(r_quat);
    handles.Cube = RedrawCube(R,handles);
    old_quat =  rotation_quaternion;
end
guidata(hObject,handles);

function h = DrawCube(R)

M0 = [    -1  -1 1;   %Node 1
    -1   1 1;   %Node 2
    1   1 1;   %Node 3
    1  -1 1;   %Node 4
    -1  -1 -1;  %Node 5
    -1   1 -1;  %Node 6
    1   1 -1;  %Node 7
    1  -1 -1]; %Node 8

M = (R*M0')';


x = M(:,1);
y = M(:,2);
z = M(:,3);


con = [1 2 3 4;
    5 6 7 8;
    4 3 7 8;
    1 2 6 5;
    1 4 8 5;
    2 3 7 6]';

x = reshape(x(con(:)),[4,6]);
y = reshape(y(con(:)),[4,6]);
z = reshape(z(con(:)),[4,6]);

c = 1/255*[255 248 88;
    0 0 0;
    57 183 225;
    57 183 0;
    255 178 0;
    255 0 0];

h = fill3(x,y,z, 1:6);

for q = 1:length(c)
    h(q).FaceColor = c(q,:);
end

function h = RedrawCube(R,hin)

h = hin.Cube;
c = 1/255*[255 248 88;
    0 0 0;
    57 183 225;
    57 183 0;
    255 178 0;
    255 0 0];

M0 = [    -1  -1 1;   %Node 1
    -1   1 1;   %Node 2
    1   1 1;   %Node 3
    1  -1 1;   %Node 4
    -1  -1 -1;  %Node 5
    -1   1 -1;  %Node 6
    1   1 -1;  %Node 7
    1  -1 -1]; %Node 8

M = (R*M0')';


x = M(:,1);
y = M(:,2);
z = M(:,3);


con = [1 2 3 4;
    5 6 7 8;
    4 3 7 8;
    1 2 6 5;
    1 4 8 5;
    2 3 7 6]';

x = reshape(x(con(:)),[4,6]);
y = reshape(y(con(:)),[4,6]);
z = reshape(z(con(:)),[4,6]);

for q = 1:6
    h(q).Vertices = [x(:,q) y(:,q) z(:,q)];
    h(q).FaceColor = c(q,:);
end

UpdateRotMat(hin,R);
UpdateEA(hin,R);
UpdateRotVec(hin,R);
UpdateEAA(hin,R);
UpdateQuaternion(hin,R);



function q_0_Callback(hObject, eventdata, handles)
% hObject    handle to q_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q_0 as text
%        str2double(get(hObject,'String')) returns contents of q_0 as a double


% --- Executes during object creation, after setting all properties.
function q_0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q_1_Callback(hObject, eventdata, handles)
% hObject    handle to q_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q_1 as text
%        str2double(get(hObject,'String')) returns contents of q_1 as a double


% --- Executes during object creation, after setting all properties.
function q_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q_2_Callback(hObject, eventdata, handles)
% hObject    handle to q_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q_2 as text
%        str2double(get(hObject,'String')) returns contents of q_2 as a double


% --- Executes during object creation, after setting all properties.
function q_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q_3_Callback(hObject, eventdata, handles)
% hObject    handle to q_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q_3 as text
%        str2double(get(hObject,'String')) returns contents of q_3 as a double


% --- Executes during object creation, after setting all properties.
function q_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in q_rot.
function q_rot_Callback(hObject, eventdata, handles)
% hObject    handle to q_rot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q=zeros(1,4);
q(1)=str2double(get(handles.q_0,'String'));
q(2)=str2double(get(handles.q_1,'String'));
q(3)=str2double(get(handles.q_2,'String'));
q(4)=str2double(get(handles.q_3,'String'));

SetVariableGlobal_old_rot(q');

R= rotQua2M(q);
handles.Cube=RedrawCube(R,handles);




function eu_angle_Callback(hObject, eventdata, handles)
% hObject    handle to eu_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a= str2double(get(handles.eu_angle,'String'));
u=zeros(3,1);
u(1)= str2double(get(handles.eu_x,'String'));
u(2)= str2double(get(handles.eu_y,'String'));
u(3)= str2double(get(handles.eu_z,'String'));

R=Eaa2rotMat(a,u);
SetVariableGlobal_old_rot(rotM2Quat(R));
handles.Cube=RedrawCube(R,handles);


% Hints: get(hObject,'String') returns contents of eu_angle as text
%        str2double(get(hObject,'String')) returns contents of eu_angle as a double


% --- Executes during object creation, after setting all properties.
function eu_angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to eu_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function eu_x_Callback(hObject, eventdata, handles)
% hObject    handle to eu_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of eu_x as text
%        str2double(get(hObject,'String')) returns contents of eu_x as a double


% --- Executes during object creation, after setting all properties.
function eu_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to eu_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function eu_y_Callback(hObject, eventdata, handles)
% hObject    handle to eu_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of eu_y as text
%        str2double(get(hObject,'String')) returns contents of eu_y as a double


% --- Executes during object creation, after setting all properties.
function eu_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to eu_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function eu_z_Callback(hObject, eventdata, handles)
% hObject    handle to eu_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of eu_z as text
%        str2double(get(hObject,'String')) returns contents of eu_z as a double


% --- Executes during object creation, after setting all properties.
function eu_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to eu_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in eu_rot.
function eu_rot_Callback(hObject, eventdata, handles)
% hObject    handle to eu_rot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)






% --- Executes on button press in mat_rot.
function mat_rot_Callback(hObject, eventdata, handles)
% hObject    handle to mat_rot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)




function vec_x_Callback(hObject, eventdata, handles)
% hObject    handle to vec_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vec_x as text
%        str2double(get(hObject,'String')) returns contents of vec_x as a double


% --- Executes during object creation, after setting all properties.
function vec_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vec_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function vec_y_Callback(hObject, eventdata, handles)
% hObject    handle to vec_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vec_y as text
%        str2double(get(hObject,'String')) returns contents of vec_y as a double


% --- Executes during object creation, after setting all properties.
function vec_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vec_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function vec_z_Callback(hObject, eventdata, handles)
% hObject    handle to vec_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vec_z as text
%        str2double(get(hObject,'String')) returns contents of vec_z as a double


% --- Executes during object creation, after setting all properties.
function vec_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vec_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in vec_rot.
function vec_rot_Callback(hObject, eventdata, handles)
% hObject    handle to vec_rot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

vec=zeros(3,1);
vec(1)=str2double(get(handles.vec_x,'String'));
vec(2)=str2double(get(handles.vec_y,'String'));
vec(3)=str2double(get(handles.vec_z,'String'));

R=rotVec2rotMat(vec);

SetVariableGlobal_old_rot(rotM2Quat(R));

handles.Cube=RedrawCube(R,handles);





function yaw_Callback(hObject, eventdata, handles)
% hObject    handle to yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw as text
%        str2double(get(hObject,'String')) returns contents of yaw as a double


% --- Executes during object creation, after setting all properties.
function yaw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pitch_Callback(hObject, eventdata, handles)
% hObject    handle to pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pitch as text
%        str2double(get(hObject,'String')) returns contents of pitch as a double


% --- Executes during object creation, after setting all properties.
function pitch_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function roll_Callback(hObject, eventdata, handles)
% hObject    handle to roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of roll as text
%        str2double(get(hObject,'String')) returns contents of roll as a double


% --- Executes during object creation, after setting all properties.
function roll_CreateFcn(hObject, eventdata, handles)
% hObject    handle to roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in angles_rot.
function angles_rot_Callback(hObject, eventdata, handles)
% hObject    handle to angles_rot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
yaw=str2double(get(handles.yaw,'String'));
pitch=str2double(get(handles.pitch,'String'));
roll=str2double(get(handles.roll,'String'));

R=eAngles2rotM(yaw,pitch,roll);
SetVariableGlobal_old_rot(rotM2Quat(R));
handles.Cube=RedrawCube(R,handles);
    


%---------------------Reset & Update Functions------------------

function UpdateRotMat(handles,rot_mat)
set(handles.mat_11,'String',rot_mat(1,1));
set(handles.mat_12,'String',rot_mat(1,2));
set(handles.mat_13,'String',rot_mat(1,3));
set(handles.mat_21,'String',rot_mat(2,1));
set(handles.mat_22,'String',rot_mat(2,2));
set(handles.mat_23,'String',rot_mat(2,3));
set(handles.mat_31,'String',rot_mat(3,1));
set(handles.mat_32,'String',rot_mat(3,2));
set(handles.mat_33,'String',rot_mat(3,3));

function UpdateQuaternion(handles,rot_mat)
quaternion=rotM2Quat(rot_mat);
set(handles.q_0,'String',quaternion(1));
set(handles.q_1,'String',quaternion(2));
set(handles.q_2,'String',quaternion(3));
set(handles.q_3,'String',quaternion(4));

function UpdateEAA(handles,rot_mat)
[euler_angle,euler_axis]=rotMat2Eaa(rot_mat);
set(handles.eu_angle,'String',euler_angle);
set(handles.eu_x,'String',euler_axis(1));
set(handles.eu_y,'String',euler_axis(2));
set(handles.eu_z,'String',euler_axis(3));

function UpdateRotVec(handles,rot_mat)
r_vec=rotM2rotVec(rot_mat);
set(handles.vec_x,'String',r_vec(1));
set(handles.vec_y,'String',r_vec(2));
set(handles.vec_z,'String',r_vec(3));

function UpdateEA(handles,rot_mat)
[y,p,r]=rotM2eAngles(rot_mat);
set(handles.yaw,'String',y);
set(handles.pitch,'String',p);
set(handles.roll,'String',r);


% --- Executes on button press in reset.
function reset_Callback(hObject, eventdata, handles)
% hObject    handle to reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
R=eye(3);

global old_quat;
old_quat = [1;0;0;0];
SetVariableGlobal_old_rot([1;0;0;0]);

handles.Cube=RedrawCube(R,handles);


% GLOBAL VAR UTILS
function SetVariableGlobal_xclick(variable)
    global xclick;
    xclick = variable;
    
function r = GetVariableGlobal_xclick
    global xclick;
    r = xclick;

function SetVariableGlobal_yclick(variable)
    global yclick;
    yclick = variable;
    
function r = GetVariableGlobal_yclick
    global yclick;
    r = yclick;
  
function SetVariableGlobal_old_rot(variable)
    global old_rot;
    old_rot = variable;
    
function r = GetVariableGlobal_old_rot
    global old_rot;
    r = old_rot;
    
    
    
    
