function doPlots(t,N,p,pmin,pmax,po,pf,v,vmax,a,amax)

L = length(t);
colors = distinguishable_colors(N);
% figure(1)
% set(gcf,'currentchar',' ')
% while get(gcf,'currentchar')==' '
%     for k = 1:K
%         for i = 1:N
%             plot3(pk(1,k,i),pk(2,k,i),pk(3,k,i),'o', ...
%                   'LineWidth',2, 'Color',colors(i,:));
%             hold on;
%             grid on;
%             xlim([-4,4])
%             ylim([-4,4])
%             zlim([0,3.5])
%             plot3(po(1,1,i), po(1,2,i), po(1,3,i),'^',...
%                   'LineWidth',2,'Color',colors(i,:));
%             plot3(pf(1,1,i), pf(1,2,i), pf(1,3,i),'x',...
%                   'LineWidth',2,'Color',colors(i,:));    
%         end
%         drawnow
%     end
%     pause(1)
%     clf
% end

L = length(t);
colors = distinguishable_colors(N);
       
for i = 1:N
    figure(1);
    h_plot(i) = plot3(p(1,:,i), p(2,:,i), p(3,:,i), 'LineWidth',1.5,...
                'Color',colors(i,:));
    h_label{i} = ['Vehicle #' num2str(i)];
    hold on;
    grid on;
    xlim([pmin(1),pmax(1)])
    ylim([pmin(2),pmax(2)])
    zlim([0,pmax(3)])
    xlabel('x[m]')
    ylabel('y[m]');
    zlabel('z[m]')
    plot3(po(1,1,i), po(1,2,i), po(1,3,i),'x',...
                  'LineWidth',3,'Color',colors(i,:));
%     plot3(pf(1,1,i), pf(1,2,i), pf(1,3,i),'x',...
%                   'LineWidth',5,'Color',colors(i,:)); 
    
    figure(2)
    diff = p(:,:,i) - repmat(pf(:,:,i),length(t),1)';
    dist = sqrt(sum(diff.^2,1));
    plot(t, dist, 'LineWidth',1.5);
    grid on;
    hold on;
    xlabel('t [s]')
    ylabel('Distance to target [m]');
    
    
    figure(3)
    subplot(3,1,1)
    plot(t,p(1,:,i),'LineWidth',1.5);
    plot(t,pmin(1)*ones(length(t),1),'--r','LineWidth',1.5);
    plot(t,pmax(1)*ones(length(t),1),'--r','LineWidth',1.5);
    ylabel('x [m]')
    xlabel ('t [s]')
    grid on;
    hold on;

    subplot(3,1,2)
    plot(t,p(2,:,i),'LineWidth',1.5);
    plot(t,pmin(2)*ones(length(t),1),'--r','LineWidth',1.5);
    plot(t,pmax(2)*ones(length(t),1),'--r','LineWidth',1.5);
    ylabel('y [m]')
    xlabel ('t [s]')
    grid on;
    hold on;

    subplot(3,1,3)
    plot(t,p(3,:,i),'LineWidth',1.5);
    plot(t,pmin(3)*ones(length(t),1),'--r','LineWidth',1.5);
    plot(t,pmax(3)*ones(length(t),1),'--r','LineWidth',1.5);
    ylabel('z [m]')
    xlabel ('t [s]')
    grid on;
    hold on;

    figure(4)
    subplot(3,1,1)
    plot(t,v(1,:,i),'LineWidth',1.5);
    plot(t,vmax*ones(length(t),1),'--r','LineWidth',1.5);
    plot(t,-vmax*ones(length(t),1),'--r','LineWidth',1.5);
    ylabel('vx [m/s]')
    xlabel ('t [s]')
    grid on;
    hold on;

    subplot(3,1,2)
    plot(t,v(2,:,i),'LineWidth',1.5);
    plot(t,vmax*ones(length(t),1),'--r','LineWidth',1.5);
    plot(t,-vmax*ones(length(t),1),'--r','LineWidth',1.5);
    ylabel('vy [m/s]')
    xlabel ('t [s]')
    grid on;
    hold on;

    subplot(3,1,3)
    plot(t,v(3,:,i),'LineWidth',1.5);
    plot(t,vmax*ones(length(t),1),'--r','LineWidth',1.5);
    plot(t,-vmax*ones(length(t),1),'--r','LineWidth',1.5);
    ylabel('vz [m/s]')
    xlabel ('t [s]')
    grid on;
    hold on;

    figure(5)
    subplot(3,1,1)
    plot(t,a(1,:,i),'LineWidth',1.5);
    plot(t,amax*ones(length(t),1),'--r','LineWidth',1.5);
    plot(t,-amax*ones(length(t),1),'--r','LineWidth',1.5);
    ylabel('ax [m/s]')
    xlabel ('t [s]')
    grid on;
    hold on;

    subplot(3,1,2)
    plot(t,a(2,:,i),'LineWidth',1.5);
    plot(t,amax*ones(length(t),1),'--r','LineWidth',1.5);
    plot(t,-amax*ones(length(t),1),'--r','LineWidth',1.5);
    ylabel('ay [m/s]')
    xlabel ('t [s]')
    grid on;
    hold on;

    subplot(3,1,3)
    plot(t,a(3,:,i),'LineWidth',1.5);
    plot(t,amax*ones(length(t),1),'--r','LineWidth',1.5);
    plot(t,-amax*ones(length(t),1),'--r','LineWidth',1.5);
    ylabel('az [m/s]')
    xlabel ('t [s]')
    grid on;
    hold on;
   
end


legend(h_plot,h_label);

end