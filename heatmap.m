if(~exist("./no_foot/Results") || ~exist(Results))
else
    Results=readtable('./no_foot/Results.csv')
end

coarseness=unique(Results.Coarseness);
od=unique(Results.OD);
angles=[0:5:180];
heat=zeros(length(angles),length(od));
figure(1)
figure(2)
for i=1:length(coarseness)
    if coarseness(i)=='c'||coarseness(i)=='f'
        data=Results(Results.Coarseness==coarseness(i),:);
%         data=table2array(Results(Results.Coarseness==coarseness(i),[2,3,6])); % with columns OD roll_i I_f
        data.roll_i=abs(data.roll_i); % negative roll = positive roll
        data.roll_i(data.roll_i>180)=360-data.roll_i(data.roll_i>180);
        data.I_f(data.I_f<1e-4)=0; % discard negative drop current
        data=sortrows(data,'roll_i'); % sort by angle
        data=sortrows(data,'OD'); % sort by pipe size
        figure(1)
        subplot(2,1,i)
        for j=1:length(od)
            r=data.roll_i(data.OD==od(j));
            I=data.I_f(data.OD==od(j));
            heat(:,j) = interp1(r,I,angles,'linear');
            heat(1,j)=0;
            plot(r,I)
            hold on
        end
        figure(2)
        subplot(2,1,i)
        temp=sortrows(data,2);
        angle=temp(:,2);
        [A,B]=meshgrid(od,angles);
        contourf(A,B,heat)
        xlabel('Diameter (mm)')
        ylabel('Angle (deg)')
        ylim([0 135])
    end
end
hold off

