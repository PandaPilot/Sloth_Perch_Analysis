if(exist("Results"))
else
    open('Results.csv')
    input('Import then hit Enter')
end

coarseness=unique(Results.Coarseness);
od=unique(Results.OD);
angles=[0:10:180];
heat=zeros(length(angles),length(od));
figure(1)
figure(2)
for i=1:length(coarseness)
    if coarseness(i)=='c'||coarseness(i)=='f'
        data=table2array(Results(Results.Coarseness==coarseness(i),[2,3,6])); % with columns OD roll_i I_f
        data(:,2)=abs(data(:,2)); % negative roll = positive roll
        data(data(:,2)>180,2)=360-data(data(:,2)>180,2);
        data(data(:,3)<1e-4,3)=0; % discard negative drop current
        data=sortrows(data,2); % sort by angle
        data=sortrows(data,1); % sort by pipe size
        figure(1)
        subplot(length(coarseness),1,i)
        for j=1:length(od)
            X=data(data(:,1)==od(j),2);
            Y=data(data(:,1)==od(j),3);
            heat(:,j) = interp1(X,Y,angles);
            plot(X,Y)
            hold on
        end
        figure(2)
        subplot(length(coarseness),1,i)
        temp=sortrows(data,2);
        angle=temp(:,2);
        [A,B]=meshgrid(od,angles);
        surf(A,B,heat)
        xlabel('Diameter (mm)')
        ylabel('Angle (deg)')
    end
end
hold off

