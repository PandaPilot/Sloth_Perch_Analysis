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

if ~exist('processed')||~processed
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
        tmp=find(Data(i).z<0.7,1,'first');
        tmp1=find(Data(i).z>0.5,1,'last');
        %tmp=find(Data(i).jz==max(Data(i).jz));
        Data(i).index_impact=Data(i).vz==min(Data(i).vz(tmp:tmp1));
        Data(i).index_impact=find(Data(i).index_impact);
        
        Data(i).servo_impact=Data(i).servo_pos(Data(i).index_impact);
    end
    [tmp2,I]=sort([Data.servo_impact]);
    Data=Data(I);
    global ig
    ig=1; % grouping index
    ng=1; % variable row index
    tmp2=[Data.perch];
    grouping=[0,0];
    while ig<=n_data
        grouping(ng,1)=Count(tmp2);
        grouping(ng,2)=ig-1;
        ig=ig+1;
        ng=ng+1;
    end
    grouping=grouping(grouping(:,1)~=0,:); % (1,:) how many in a row, (2,:) position of largest value/end of row
    processed=1;
else
    disp('Skip processing')
end

close all

%% x and vz at impact

% figure
% hold on
perch_servo_pos=zeros(size(Data));
perch_success=perch_servo_pos;
for i=1:n_data
    perch_servo_pos(i)=Data(i).servo_pos(Data(i).index_impact);
    if Data(i).perch
        perch_success(i)=true;
        %         a=plot(Data(i).servo_pos(Data(i).index_impact),Data(i).vz(Data(i).index_impact),'o');
        %         a.MarkerEdgeColor=[0.4660 0.6740 0.1880];
        %         a.MarkerFaceColor=[0.4660 0.6740 0.1880];
    end
end
logical(perch_success);
% for i=1:n_data
%     if ~Data(i).perch
%         if Data(i).servo_pos(Data(i).index_impact)<=max(perch_servo_pos) && Data(i).servo_pos(Data(i).index_impact)>=min(perch_servo_pos(perch_servo_pos>0))
%             b=plot(Data(i).servo_pos(Data(i).index_impact),Data(i).vz(Data(i).index_impact),'x');
%             b.MarkerEdgeColor=[0.6350 0.0780 0.1840];
%             b.MarkerSize=8;
%             b.LineWidth=1;
%         end
%     end
% end
% xlabel('servo pos')
% ylabel('v_z')
% %axis equal
% grid on
%

%% Perch branch size and arm angle
servo_flat=1700; % XH430 0.088 deg per resolution
r_spool=6; % spool radius
l_t=10; % distance between joint and tendon hole
perch_ang_arm=zeros(size(perch_servo_pos));

perch_ang_arm=real(180-2*asind((l_t-(perch_servo_pos-servo_flat)*0.088/180*pi*r_spool/2)/l_t)); % servo->spool angle->tendon retraction->half internal angle->external angle
[arm_xs,arm_ys]=pol2cart(perch_ang_arm(find(perch_success))/180*pi+pi,100);
[arm_xf,arm_yf]=pol2cart(perch_ang_arm(find(~perch_success))/180*pi+pi,100);

d_branch=125;
r_branch=d_branch/2;
x=meshgrid([-110:.1:110]*5); % graph resolution
y=x';
zone=zeros(size(x));
z_zone=[100,75,50,10;5^2,25^2,50^2,70^2];
drone=zone+z_zone(2);
drone_h=72; % drone body height
drone_w=60; % drone body half width
m0=(-2*drone_w^2*r_branch)/((r_branch^2+drone_w^2)*(2*(-drone_w)*r_branch^2/(drone_w^2+r_branch^2)+drone_w)); % arm gradient
optimal_arm_ang=atan2d(m0,1);
drone=[-drone_w,-drone_w,drone_w,drone_w,-drone_w*.9,-drone_w*.9, drone_w*.9,drone_w*.9,-drone_w*.9,-drone_w*.9,-drone_w;drone_h,0,0,drone_h,drone_h,drone_h*.9,drone_h*.9,drone_h*.1,drone_h*.1,drone_h,drone_h];
% drone(x>-10&x<10&y>0&y<10)=100;
% drone(x>-8&x<8&y>1.5&y<8.5)=0;
% [drn_idx_x,drn_idx_y]=find(drone~=0);

% % % for radial zone
% % for i=size(z_zone,2):-1:1
% %     z(((x+10).^2+(y).^2)<=z_zone(2,i))=z_zone(1,i);
% % end
i=find(max(grouping(:,1))==(grouping(:,1)));
pct_group=[1 .8 .6 .4];
high_s=(perch_ang_arm>=optimal_arm_ang)&[Data.perch]; % high side successful perch
low_s=(perch_ang_arm<optimal_arm_ang)&[Data.perch];

high_s(1:find(perch_ang_arm>=optimal_arm_ang)-1)=[]; % remove zeros below optimal perch angle
low_s(find(perch_ang_arm<optimal_arm_ang,1,'last')+1:end)=[];

prob_h=[];
for j=1:numel(high_s)
    prob_h(j)=sum(high_s(1:j))/j;
end
prob_l=[];
for j=1:numel(low_s)
    prob_l(j)=sum(low_s((numel(low_s)-j+1):end))/j;
end
zone_r=100*((1-.5)+.5*[0:1/(numel(pct_group)):1]);
gap=1.2;
for i=1:numel(pct_group)
    tmp3=abs(prob_h-pct_group(i));
    tmp4=abs(prob_l-pct_group(i));
    pct_i(1,i)= find(tmp3==min(tmp3),1,'last')-1;% percentage index (optimal +- ...) (1,:)=high (2,:)=low
    pct_i(2,i)= find(tmp4==min(tmp4),1,'last')-1;
    zone(atan2d(y,x+drone_w)<perch_ang_arm(find(perch_ang_arm>=optimal_arm_ang,1)+pct_i(1,i))-180 & atan2d(y,x+drone_w)>optimal_arm_ang-180  & (((x+drone_w).^2+(y).^2)>(zone_r(i)+gap)^2) & (((x+drone_w).^2+(y).^2)<(zone_r(i+1)-gap)^2))=prob_h(pct_i(1,i)+1); % high side
    zone(atan2d(y,x+drone_w)>perch_ang_arm(find(perch_ang_arm<optimal_arm_ang,1,'last')-pct_i(2,i))-180 & atan2d(y,x+drone_w)<optimal_arm_ang-180 & (((x+drone_w).^2+(y).^2)>(zone_r(i)+gap)^2) & (((x+drone_w).^2+(y).^2)<(zone_r(i+1)-gap)^2))=prob_l(pct_i(2,i)+1); % low side
end
zone(((x+drone_w).^2+(y).^2)>100^2)=0;
% zone(atan2d(y,x+drone_w)< 0 & atan2d(y,x+drone_w)>-180)=z_zone(1,4);
% zone(atan2d(y,x+drone_w)<perch_ang_arm(grouping(i+2,2))-180 & atan2d(y,x+drone_w)>perch_ang_arm(grouping(i-1,2)-grouping(i-1,1)+1)-180)=z_zone(1,3);
% zone(atan2d(y,x+drone_w)<perch_ang_arm(grouping(i,2))-180 & atan2d(y,x+drone_w)>perch_ang_arm(grouping(i,2)-grouping(i,1)+1)-180)=z_zone(1,2);

[tx_posx,tx_posy]=pol2cart(([90,75,60,45,30,15,-15,-30,-45,-60,-75,-90]+optimal_arm_ang)/180*pi+pi,100+15);
tx=["+90","+75","+60","+45","+30","+15","-15","-30","-45","-60","-75","-90"];


% zone(x>-drone_w)=0;
% zone(y>0)=0;


close
g1=figure;

contourf(x,y,zone)
hold on


b=0:.1:1;%[0 0 0 0 0 0 0 0 0 0 0]+1;
g=0:.1:1;%[0 0 0 0 0 0 0 0 0 0 0]+1;
r=[0 0 0 0 0 0 0 0 0 0 0]+1;

map = [ 1/.8    1/.8     1/.8
    r(1) g(1)  b(1)
    r(2) g(2)  b(2)
    r(3) g(3)  b(3)
    r(4) g(4)  b(4)
    r(5) g(5)  b(5)
    r(6) g(6)  b(6)
    r(7) g(7)  b(7)
    r(8) g(8)  b(8)
    r(9) g(9)  b(9)
    r(10) g(10) b(10)
    r(11) g(11) b(11)]*.8;


%fill(drone(1,:),drone(2,:),'b') % drone body fill
colormap(g1,map)
%fimplicit(@(x,y) x^2+(y+d_branch/2)^2-(d_branch/2)^2,'k','LineWidth',3);

l_arm=35*3;
y2=-m0*l_arm/(1+m0^2)^.5;
x2=-l_arm/(1+m0^2)^.5-drone_w;
plot([-drone_w x2],[0 y2],'k','LineWidth',4)
plot(arm_xs-drone_w,arm_ys,'go')
plot(arm_xf-drone_w,arm_yf,'ro');

text(tx_posx-drone_w,tx_posy,tx,'HorizontalAlignment','center')
for i=1:numel(tx_posx)
    plot([-drone_w,tx_posx(i)-drone_w],[0,tx_posy(i)],'--','Color',[0.5 0.5 0.5])
end

axis equal
axis off

J = imread('drone_fig/sloth_body.png');
Jtp=ones(size(J,1),size(J,2));
Jtp=Jtp-all(J==255,3);
image('CData',J,'XData',[-drone_w drone_w],'YData',[0 drone_h],'AlphaData',Jtp)

K = imread('drone_fig/sloth_arm.png');
Ktp=ones(size(K,1),size(K,2));
Ktp=Ktp-all(K==255,3);
K = imrotate(K,-optimal_arm_ang);
Ktp = imrotate(Ktp,-optimal_arm_ang);
image('CData',K,'AlphaData',Ktp)


%imshow(J);
disp("Done")

%% t-z graph
% figure
% for i=1:n_data
%     if Data(i).perch
%         plot(Data(i).t,Data(i).z);
%     else
%         plot(Data(i).t,Data(i).z,'--');
%     end
%     hold on
% end
% xlim([-0.5 1.5])
% grid on
%
% figure % 3D xyz
% hold
% for i=1:n_data
%     index_t=Data(i).t>=-1 & Data(i).t<=1.5;
%     if Data(i).perch
%         plot3(Data(i).x(index_t),Data(i).y(index_t),Data(i).z(index_t),'LineWidth',2);
%     else
%         plot3(Data(i).x(index_t),Data(i).y(index_t),Data(i).z(index_t),'--');
%     end
%     plot3(Data(i).x(Data(i).index_impact),Data(i).y(Data(i).index_impact),Data(i).z(Data(i).index_impact),'o');
% end
% axis equal
% grid on

% figure % end z position
% for i=1:n_data
%     plot(i,Data(i).z(end),'*');
%     hold on
% end
% grid on
%
% figure % xyz at impact
% for i=1:n_data
%     if Data(i).perch
%         plot3(Data(i).x(Data(i).index_impact),Data(i).y(Data(i).index_impact),Data(i).z(Data(i).index_impact),'o');
%     else
%         plot3(Data(i).x(Data(i).index_impact),Data(i).y(Data(i).index_impact),Data(i).z(Data(i).index_impact),'x');
%     end
%     hold on
% end
% xlabel('x')
% ylabel('y')
% axis equal
% grid on
%
% k=1;
% k2=1;
% figure % analytics
% plts=5;
% for i=1:n_data
%     subplot(1,plts,1);
%     scatter(Data(i).perch,Data(i).x(Data(i).index_impact));
%     ylabel('x')
%     hold on
%     grid on
%     subplot(1,plts,2);
%     scatter(Data(i).perch,Data(i).vz(Data(i).index_impact));
%     if(Data(i).perch)
%         vz_good(k)=Data(i).vz(Data(i).index_impact);
%         k=k+1;
%     else
%         vz_bad(k2)=Data(i).vz(Data(i).index_impact);
%         k2=k2+1;
%     end
%     ylabel('vz')
%     hold on
%     grid on
%     subplot(1,plts,3);
%     scatter(Data(i).perch,Data(i).servo_pos(Data(i).index_impact));
%     ylabel('servo pos')
%     hold on
%     grid on
%     subplot(1,plts,4);
%     if Data(i).perch
%         scatter(Data(i).delay,Data(i).z(Data(i).t==0));
%     end
%     xlabel('delay')
%     ylabel('z')
%     hold on
%     grid on
%     subplot(1,plts,5);
%     if Data(i).perch
%         scatter(Data(i).delay,Data(i).servo_pos(Data(i).index_impact));
%     end
%     xlabel('delay')
%     ylabel('servo pos')
%     hold on
%     grid on
% end

% figure
% plot(vz_good,'o')
% hold on
% plot(vz_bad,'*')

%% functions
function out = Count(in)
global ig
out=0;
if in(ig) == 0
    out = 0;
else
    ig=ig+1;
    out = out + 1 + Count(in);
end
end