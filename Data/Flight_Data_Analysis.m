%% ROS-RC Flight Data Analysis
folders=dir('hover_drop');
n_folder=size(folders,1);
if ~exist('k')
    k=0;
    for i=1:n_folder
        dum_name=join([folders(i).folder,'\',folders(i).name,'\_slash_Flight_Data.csv']);
        if exist(dum_name)
            Data_raw(i-k).data=readtable(dum_name);
        else
            k=k+1;
        end
    end
    clear n_folder folders dum_name i
    n_data=size(Data,2);
    disp('Data acquired')
else
    disp('Skip data acquisition')
end
Data=Data_raw; % create a copy, DO NOT CLEAR Data_raw OR k, else data acquisition will run.
wc=40; % butterworth cutoff freq

for i=1:n_data
    Data(i).t_raw=(Data(i).data.rosbagTimestamp-Data(i).data.rosbagTimestamp(1))*1e-9;
    Data(i).dt=Data(i).t_raw(2:end)-Data(i).t_raw(1:end-1);
    Data(i).Dt=mean(Data(i).dt); % find mean timestep
    Data(i).t=(1:length(Data(i).data.rosbagTimestamp))'*Data(i).Dt;
    
    %% interpolate position and velocity with mean timestep
    Data(i).x=interp1(Data(i).t_raw,Data(i).data.x,Data(i).t);
    Data(i).y=interp1(Data(i).t_raw,Data(i).data.y,Data(i).t);
    Data(i).z=interp1(Data(i).t_raw,Data(i).data.z,Data(i).t);
    Data(i).vx=interp1(Data(i).t_raw,Data(i).data.vx,Data(i).t);
    Data(i).vy=interp1(Data(i).t_raw,Data(i).data.vy,Data(i).t);
    Data(i).vz=interp1(Data(i).t_raw,Data(i).data.vz,Data(i).t);
    
    while(isnan(Data(i).x(end))||isnan(Data(i).y(end))||isnan(Data(i).z(end))||isnan(Data(i).vx(end))||isnan(Data(i).vy(end))||isnan(Data(i).vz(end))) % remove NaN
        Data(i).t(end)=[];
        Data(i).x(end)=[];
        Data(i).y(end)=[];
        Data(i).z(end)=[];
        Data(i).vx(end)=[];
        Data(i).vy(end)=[];
        Data(i).vz(end)=[];
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
    Data(i).t(end)=[]; % repeat delete end to simplify t0=0
    Data(i).x(1)=[];
    Data(i).y(1)=[];
    Data(i).z(1)=[];
    Data(i).vx(1)=[];
    Data(i).vy(1)=[];
    Data(i).vz(1)=[];
    %% accel filtering 
    Data(i).ax_filter(1:3)=Data(i).ax(1:3);
    Data(i).ay_filter(1:3)=Data(i).ay(1:3);
    Data(i).az_filter(1:3)=Data(i).az(1:3);
    for j=4:length(Data(i).t)
        Data(i).ax_filter(j)=(1+4*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2+Data(i).Dt^3*wc^3)^-1 ...
            *(Data(i).Dt^3*wc^3*Data(i).ax(j)+(3+10*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2)*Data(i).ax_filter(j-1)-(3+8*Data(i).Dt*wc)*Data(i).ax_filter(j-2)+(1+2*Data(i).Dt*wc)*Data(i).ax_filter(j-3));
        Data(i).ay_filter(j)=(1+4*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2+Data(i).Dt^3*wc^3)^-1 ...
            *(Data(i).Dt^3*wc^3*Data(i).ay(j)+(3+10*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2)*Data(i).ay_filter(j-1)-(3+8*Data(i).Dt*wc)*Data(i).ay_filter(j-2)+(1+2*Data(i).Dt*wc)*Data(i).ay_filter(j-3));
        Data(i).az_filter(j)=(1+4*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2+Data(i).Dt^3*wc^3)^-1 ...
            *(Data(i).Dt^3*wc^3*Data(i).az(j)+(3+10*Data(i).Dt*wc+2*Data(i).Dt^2*wc^2)*Data(i).az_filter(j-1)-(3+8*Data(i).Dt*wc)*Data(i).az_filter(j-2)+(1+2*Data(i).Dt*wc)*Data(i).az_filter(j-3));
    end
end