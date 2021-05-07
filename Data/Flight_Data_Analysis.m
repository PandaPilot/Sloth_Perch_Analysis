%% ROS-RC Flight Data Analysis
folders=dir('hover_drop');
n_folder=size(folders,1);
if ~exist('n_data')
    k=0;
    for i=1:n_folder
        dum_name =join([folders(i).folder,'\',folders(i).name,'\_slash_Flight_Data.csv']);
        dum_name1=join([folders(i).folder,'\',folders(i).name,'\_slash_mavros_slash_imu_slash_data.csv']);
        dum_name2=join([folders(i).folder,'\',folders(i).name,'\_slash_dynamixel_workbench_slash_dynamixel_state.csv']);
        
        if exist(dum_name) && contains(folders(i).name,'05-05') && ~contains(folders(i).name,'05-05-14')
            Data_raw(i-k).name=folders(i).name;
            Data_raw(i-k).FD=readtable(dum_name);
            Data_raw(i-k).IMU=readtable(dum_name1);
            Data_raw(i-k).Ser=readtable(dum_name2);
        else
            k=k+1;
        end
    end
    clear n_folder folders dum_name i
    n_data=size(Data_raw,2);
    disp('Data acquired')
else
    disp('Skip data acquisition')
end

if ~exist('processed')
    Data=Data_raw; % create a copy, DO NOT CLEAR Data_raw OR k, else data acquisition will run.
    wc=40; % butterworth cutoff freq
    for i=1:n_data
        Data(i).t_raw=Data(i).FD.secs+Data(i).FD.nsecs*1e-9;
        index_t0=find(all(cell2mat(Data(i).FD.Status)=='"Disarmed"',2),1,'first');  % find the index of first "Disarmed"
        Data(i).delay=str2num(Data(i).name(6:9));
        t=Data(i).IMU.secs+Data(i).IMU.nsecs*1e-9-Data(i).t_raw(index_t0);
        Data(i).ax_imu=interp1(t,Data(i).IMU.x_2,Data(i).t_raw-Data(i).t_raw(index_t0));
        Data(i).ay_imu=interp1(t,Data(i).IMU.y_2,Data(i).t_raw-Data(i).t_raw(index_t0));
        Data(i).az_imu=interp1(t,Data(i).IMU.z_2,Data(i).t_raw-Data(i).t_raw(index_t0));
        
        t=Data(i).Ser.Var1*1e-9-Data(i).t_raw(index_t0);
        Data(i).servo_pos=interp1(t,Data(i).Ser.Var5,Data(i).t_raw-Data(i).t_raw(index_t0));
        
        %Data(i).t_raw=Data(i).t_raw-Data(i).t_raw(1);
        Data(i).dt=Data(i).t_raw(2:end)-Data(i).t_raw(1:end-1);
        Data(i).Dt=mean(Data(i).dt); % find mean timestep
        Data(i).t=((0:length(Data(i).FD.rosbagTimestamp)-1)-index_t0)'*Data(i).Dt;
        
        %% interpolate position and velocity with mean timestep
        t=Data(i).t_raw-Data(i).t_raw(index_t0);
        Data(i).x=interp1(t,Data(i).FD.x,Data(i).t);
        Data(i).y=interp1(t,Data(i).FD.y,Data(i).t);
        Data(i).z=interp1(t,Data(i).FD.z,Data(i).t);
        Data(i).vx=interp1(t,Data(i).FD.vx,Data(i).t);
        Data(i).vy=interp1(t,Data(i).FD.vy,Data(i).t);
        Data(i).vz=interp1(t,Data(i).FD.vz,Data(i).t);
        Data(i).ax_imu=interp1(t,Data(i).ax_imu,Data(i).t);
        Data(i).ay_imu=interp1(t,Data(i).ay_imu,Data(i).t);
        Data(i).az_imu=interp1(t,Data(i).az_imu,Data(i).t)-9.80665;
        Data(i).servo_pos=interp1(t,Data(i).servo_pos,Data(i).t);
        
        %% remove NaN and post perch data
        while(isnan(Data(i).x(1))||isnan(Data(i).y(1))||isnan(Data(i).z(1)) ...
                ||isnan(Data(i).vx(1))||isnan(Data(i).vy(1))||isnan(Data(i).vz(1)) ...
                ||isnan(Data(i).ax_imu(1))||isnan(Data(i).ay_imu(1))||isnan(Data(i).az_imu(1))) % remove NaN
            Data(i).t(1)=[];
            Data(i).x(1)=[];
            Data(i).y(1)=[];
            Data(i).z(1)=[];
            Data(i).vx(1)=[];
            Data(i).vy(1)=[];
            Data(i).vz(1)=[];
            Data(i).ax_imu(1)=[];
            Data(i).ay_imu(1)=[];
            Data(i).az_imu(1)=[];
            Data(i).servo_pos(1)=[];
        end
        while(isnan(Data(i).x(end))||isnan(Data(i).y(end))||isnan(Data(i).z(end)) ...
                ||isnan(Data(i).vx(end))||isnan(Data(i).vy(end))||isnan(Data(i).vz(end)) ...
                ||isnan(Data(i).ax_imu(end))||isnan(Data(i).ay_imu(end))||isnan(Data(i).az_imu(end))) % remove NaN
            Data(i).t(end)=[];
            Data(i).x(end)=[];
            Data(i).y(end)=[];
            Data(i).z(end)=[];
            Data(i).vx(end)=[];
            Data(i).vy(end)=[];
            Data(i).vz(end)=[];
            Data(i).ax_imu(end)=[];
            Data(i).ay_imu(end)=[];
            Data(i).az_imu(end)=[];
            Data(i).servo_pos(end)=[];
        end
        
        %% calculate vicon accel using central difference
        Data(i).ax=(Data(i).vx(3:end)-Data(i).vx(1:end-2))/(2*Data(i).Dt);
        Data(i).ay=(Data(i).vy(3:end)-Data(i).vy(1:end-2))/(2*Data(i).Dt);
        Data(i).az=(Data(i).vz(3:end)-Data(i).vz(1:end-2))/(2*Data(i).Dt);
        
        Data(i).t(end)=[];
        Data(i).x(end)=[];
        Data(i).y(end)=[];
        Data(i).z(end)=[];
        Data(i).vx(end)=[];
        Data(i).vy(end)=[];
        Data(i).vz(end)=[];
        Data(i).ax_imu(end)=[];
        Data(i).ay_imu(end)=[];
        Data(i).az_imu(end)=[];
        Data(i).servo_pos(end)=[];
        Data(i).t(end)=[]; % repeat delete end to simplify t0=0
        Data(i).x(1)=[];
        Data(i).y(1)=[];
        Data(i).z(1)=[];
        Data(i).vx(1)=[];
        Data(i).vy(1)=[];
        Data(i).vz(1)=[];
        Data(i).ax_imu(1)=[];
        Data(i).ay_imu(1)=[];
        Data(i).az_imu(1)=[];
        Data(i).servo_pos(1)=[];
        
        %% remove post release data
        j=length(Data(i).servo_pos);
        while Data(i).servo_pos(j)<max(Data(i).servo_pos)
            Data(i).t(end)=[];
            Data(i).x(end)=[];
            Data(i).y(end)=[];
            Data(i).z(end)=[];
            Data(i).vx(end)=[];
            Data(i).vy(end)=[];
            Data(i).vz(end)=[];
            Data(i).ax_imu(end)=[];
            Data(i).ay_imu(end)=[];
            Data(i).az_imu(end)=[];
            Data(i).ax(end)=[];
            Data(i).ay(end)=[];
            Data(i).az(end)=[];
            Data(i).servo_pos(end)=[];
            j=j-1;
        end
        %% accel filtering
        Data(i).ax_filter(1:3,1)=Data(i).ax(1:3);
        Data(i).ay_filter(1:3,1)=Data(i).ay(1:3);
        Data(i).az_filter(1:3,1)=Data(i).az(1:3);
        Data(i).ax_imufilter(1:3,1)=Data(i).ax_imu(1:3);
        Data(i).ay_imufilter(1:3,1)=Data(i).ay_imu(1:3);
        Data(i).az_imufilter(1:3,1)=Data(i).az_imu(1:3);
        for j=4:length(Data(i).t)
            Data(i).ax_filter(j)=(1+4*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2+Data(i).Dt^3*wc^3)^-1 ...
                *(Data(i).Dt^3*wc^3*Data(i).ax(j)+(3+10*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2)*Data(i).ax_filter(j-1)-(3+8*Data(i).Dt*wc)*Data(i).ax_filter(j-2)+(1+2*Data(i).Dt*wc)*Data(i).ax_filter(j-3));
            Data(i).ay_filter(j)=(1+4*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2+Data(i).Dt^3*wc^3)^-1 ...
                *(Data(i).Dt^3*wc^3*Data(i).ay(j)+(3+10*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2)*Data(i).ay_filter(j-1)-(3+8*Data(i).Dt*wc)*Data(i).ay_filter(j-2)+(1+2*Data(i).Dt*wc)*Data(i).ay_filter(j-3));
            Data(i).az_filter(j)=(1+4*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2+Data(i).Dt^3*wc^3)^-1 ...
                *(Data(i).Dt^3*wc^3*Data(i).az(j)+(3+10*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2)*Data(i).az_filter(j-1)-(3+8*Data(i).Dt*wc)*Data(i).az_filter(j-2)+(1+2*Data(i).Dt*wc)*Data(i).az_filter(j-3));
            
            Data(i).ax_imufilter(j)=(1+4*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2+Data(i).Dt^3*wc^3)^-1 ...
                *(Data(i).Dt^3*wc^3*Data(i).ax_imu(j)+(3+10*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2)*Data(i).ax_imufilter(j-1)-(3+8*Data(i).Dt*wc)*Data(i).ax_imufilter(j-2)+(1+2*Data(i).Dt*wc)*Data(i).ax_imufilter(j-3));
            Data(i).ay_imufilter(j)=(1+4*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2+Data(i).Dt^3*wc^3)^-1 ...
                *(Data(i).Dt^3*wc^3*Data(i).ay_imu(j)+(3+10*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2)*Data(i).ay_imufilter(j-1)-(3+8*Data(i).Dt*wc)*Data(i).ay_imufilter(j-2)+(1+2*Data(i).Dt*wc)*Data(i).ay_imufilter(j-3));
            Data(i).az_imufilter(j)=(1+4*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2+Data(i).Dt^3*wc^3)^-1 ...
                *(Data(i).Dt^3*wc^3*Data(i).az_imu(j)+(3+10*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2)*Data(i).az_imufilter(j-1)-(3+8*Data(i).Dt*wc)*Data(i).az_imufilter(j-2)+(1+2*Data(i).Dt*wc)*Data(i).az_imufilter(j-3));
        end
        
        %% perch detection
        if Data(i).z(end)>0.4
            Data(i).perch=1;
        else
            Data(i).perch=0;
        end
        
        %% isolate data to time during perch
        index_perch=Data(i).t>=-1 & Data(i).t<=1.5;
        Data(i).t(~index_perch)=[];
        Data(i).x(~index_perch)=[];
        Data(i).y(~index_perch)=[];
        Data(i).z(~index_perch)=[];
        Data(i).vx(~index_perch)=[];
        Data(i).vy(~index_perch)=[];
        Data(i).vz(~index_perch)=[];
        Data(i).ax_imu(~index_perch)=[];
        Data(i).ay_imu(~index_perch)=[];
        Data(i).az_imu(~index_perch)=[];
        Data(i).ax(~index_perch)=[];
        Data(i).ay(~index_perch)=[];
        Data(i).az(~index_perch)=[];
        Data(i).ax_imufilter(~index_perch)=[];
        Data(i).ay_imufilter(~index_perch)=[];
        Data(i).az_imufilter(~index_perch)=[];
        Data(i).ax_filter(~index_perch)=[];
        Data(i).ay_filter(~index_perch)=[];
        Data(i).az_filter(~index_perch)=[];
        Data(i).servo_pos(~index_perch)=[];
        Data(i).jz=zeros(length(Data(i).t),1);
        
        %% use max jerk-z as impact
        Data(i).jz(2:end)=(Data(i).az_filter(2:end)-Data(i).az_filter(1:end-1))/Data(i).Dt;
        Data(i).index_impact=Data(i).jz==max(Data(i).jz);
        
        processed=1;
    end
else
    disp('Skip processing')
end

close all
% figure % end z position
% for i=1:n_data
%     plot(i,Data(i).z(end),'*');
%     hold on
% end
% grid on

figure % t-z graph
for i=1:n_data
    if Data(i).perch
        plot(Data(i).t,Data(i).z);
    else
        plot(Data(i).t,Data(i).z,'--');
    end
    hold on
end
xlim([-0.5 1.5])
grid on

figure % 3D xyz
hold on
for i=1:n_data
    index_t=Data(i).t>=-1 & Data(i).t<=1.5;
    if Data(i).perch
        plot3(Data(i).x(index_t),Data(i).y(index_t),Data(i).z(index_t),'LineWidth',2);
    else
        plot3(Data(i).x(index_t),Data(i).y(index_t),Data(i).z(index_t),'--');
    end
    plot3(Data(i).x(Data(i).index_impact),Data(i).y(Data(i).index_impact),Data(i).z(Data(i).index_impact),'o');
end
axis equal
grid on

figure % xyz at impact
for i=1:n_data
    if Data(i).perch
        plot3(Data(i).x(Data(i).index_impact),Data(i).y(Data(i).index_impact),Data(i).z(Data(i).index_impact),'o');
    else
        plot3(Data(i).x(Data(i).index_impact),Data(i).y(Data(i).index_impact),Data(i).z(Data(i).index_impact),'x');
    end
    hold on
end
axis equal
grid on

figure % analytics
plts=5;
for i=1:n_data
    subplot(plts,1,1);
    scatter(Data(i).perch,Data(i).x(Data(i).index_impact));
    ylabel('x')
    hold on
    grid on
    subplot(plts,1,2);
    scatter(Data(i).perch,Data(i).vz(Data(i).index_impact));
    ylabel('vz')
    hold on
    grid on
    subplot(plts,1,3);
    scatter(Data(i).perch,Data(i).servo_pos(Data(i).index_impact));
    ylabel('servo pos')
    hold on
    grid on
    subplot(plts,1,4);
    if Data(i).perch
        scatter(Data(i).delay,Data(i).z(Data(i).t==0));
    end
    xlabel('delay')
    ylabel('z')
    hold on
    grid on
    subplot(plts,1,5);
    if Data(i).perch
        scatter(Data(i).delay,Data(i).servo_pos(Data(i).index_impact));
    end
    xlabel('delay')
    ylabel('servo pos')
    hold on
    grid on
end


%
% figure % need to plot impact velocity
% for i=1:n_data
%     if Data(i).perch
%        plot(1,Data(i).vz(Data(i).t==0),'*')
%     else
%         plot(0,Data(i).vz(Data(i).t==0),'*')
%     end
%     hold on
% end