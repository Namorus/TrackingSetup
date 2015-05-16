%% DATA IMPORT

%% tabula rasa
clear all
close all
clc

%% load data
importdata('datarecording_1426593724.csv')

%% save to appropriate variables

timestamp = ans.data(:,1);

panPosition = ans.data(:,2);
tiltPosition = ans.data(:,3);

panCtrlType = ans.data(:,4);
panSetValue = ans.data(:,5);

tiltCtrlType = ans.data(:,6);
tiltSetValue = ans.data(:,7);

local_lat = ans.data(:,8);
local_lon = ans.data(:,9);
local_elev = ans.data(:,10);

object_lat = ans.data(:,11);
object_lon = ans.data(:,12);
object_elev = ans.data(:,13);

object_x = ans.data(:,14);
object_y = ans.data(:,15);
object_z = ans.data(:,16);

estObject_x = ans.data(:,17);
estObject_y = ans.data(:,18);
estObject_z = ans.data(:,19);

estObject_lat = ans.data(:,20);
estObject_lon = ans.data(:,21);
estObject_elev = ans.data(:,22);

%% plots

time_vector=timestamp-timestamp(1);

% flight path

figure('color','w')
title('Flight Path')
%set(gcf,'units','normalized','outerposition',[0 0 1 1])
hold on; grid on; box on;axis equal;
 
h1=plot3(object_x,object_y,object_z,'-b');
h2=plot3(estObject_x,estObject_y,estObject_z,'-r');

xlabel('East(y) (m)')
ylabel('North(x) (m)')
zlabel('Up(z) (m)')
legend([h1,h2],'Linear Connection GPS Points','Antenna Estimation')

% antenna angles

panPosition_deg=epos2deg(panPosition);
tiltPosition_deg=epos2deg(tiltPosition);

figure('color','w')
title('Antenna Angles')
%set(gcf,'units','normalized','outerposition',[0 0 1 1])

ax1=subplot(2,1,1);
hold on; grid on; box on;
h3=plot(time_vector,mod(panSetValue,360),'-b');
h4=plot(time_vector,mod(panPosition_deg,360),'-r');
xlabel('Time (s)')
ylabel('Pan Angle [degree]')
legend([h3,h4],'SetValue','truePosition')

ax2=subplot(2,1,2);
hold on; grid on; box on;
h5=plot(time_vector,tiltSetValue,'-b');
h6=plot(time_vector,tiltPosition_deg,'-r');
xlabel('Time (s)')
ylabel('Tilt Angle [degree]')
legend([h5,h6],'SetValue','truePosition')

linkaxes([ax1,ax2],'x')
