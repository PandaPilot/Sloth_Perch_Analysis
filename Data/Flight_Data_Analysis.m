%% ROS-RC Flight Data Analysis
folders=dir('hover_drop');
n_folder=size(folders,1);
if ~exist('n_data')
    
    k=0;
    for i=1:n_folder
        dum_name =join([folders(i).folder,'\',folders(i).name,'\_slash_Flight_Data.csv']);
        dum_name1=join([folders(i).folder,'\',folders(i).name,'\_slash_mavros_slash_imu_slash_data.csv']);
        dum_name2=join([folders(i).folder,'\',folders(i).name,'\_slash_dynamixel_workbench_slash_dynamixel_state.csv']);
        
        if exist(dum_name) && contains(folders(i).name,'05-05') && ~contains(folders(i).name,'05-05-14');
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
        
        t=Data(i).IMU.secs+Data(i).IMU.nsecs*1e-9;
        Data(i).ax_imu=interp1(t,Data(i).IMU.x_2,Data(i).t_raw);
        Data(i).ay_imu=interp1(t,Data(i).IMU.y_2,Data(i).t_raw);
        Data(i).az_imu=interp1(t,Data(i).IMU.z_2,Data(i).t_raw);
        
        t=Data(i).Ser.Var1*1e-9;
        Data(i).servo_pos=interp1(t,Data(i).Ser.Var5,Data(i).t_raw);
        
        %Data(i).t_raw=Data(i).t_raw-Data(i).t_raw(1);
        Data(i).dt=Data(i).t_raw(2:end)-Data(i).t_raw(1:end-1);
        Data(i).Dt=mean(Data(i).dt); % find mean timestep
        Data(i).t=(1:length(Data(i).FD.rosbagTimestamp))'*Data(i).Dt;
        
        %% interpolate position and velocity with mean timestep
        Data(i).x=interp1(Data(i).t_raw-Data(i).t_raw(1),Data(i).FD.x,Data(i).t);
        Data(i).y=interp1(Data(i).t_raw-Data(i).t_raw(1),Data(i).FD.y,Data(i).t);
        Data(i).z=interp1(Data(i).t_raw-Data(i).t_raw(1),Data(i).FD.z,Data(i).t);
        Data(i).vx=interp1(Data(i).t_raw-Data(i).t_raw(1),Data(i).FD.vx,Data(i).t);
        Data(i).vy=interp1(Data(i).t_raw-Data(i).t_raw(1),Data(i).FD.vy,Data(i).t);
        Data(i).vz=interp1(Data(i).t_raw-Data(i).t_raw(1),Data(i).FD.vz,Data(i).t);
        Data(i).ax_imu=interp1(Data(i).t_raw-Data(i).t_raw(1),Data(i).ax_imu,Data(i).t);
        Data(i).ay_imu=interp1(Data(i).t_raw-Data(i).t_raw(1),Data(i).ay_imu,Data(i).t);
        Data(i).az_imu=interp1(Data(i).t_raw-Data(i).t_raw(1),Data(i).az_imu,Data(i).t)-9.80665;
        Data(i).servo_pos=interp1(Data(i).t_raw-Data(i).t_raw(1),Data(i).servo_pos,Data(i).t);
        
        %% remove NaN and post perch data
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
%         Data(i).servo_pos(max(Data(i).servo_pos)
        %% accel filtering
        Data(i).ax_filter(1:3)=Data(i).ax(1:3);
        Data(i).ay_filter(1:3)=Data(i).ay(1:3);
        Data(i).az_filter(1:3)=Data(i).az(1:3);
        Data(i).ax_imufilter(1:3)=Data(i).ax_imu(1:3);
        Data(i).ay_imufilter(1:3)=Data(i).ay_imu(1:3);
        Data(i).az_imufilter(1:3)=Data(i).az_imu(1:3);
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
        processed=1;
    end
else
    disp('Skip processing')
end
for i=1:n_data
    plot(i,Data(i).z(end),'*');
    hold on
    disp(Data(i).z(end));
end