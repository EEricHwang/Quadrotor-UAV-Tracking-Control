function UAV_Tracking_Plot(T_atti,T_posi,UAV_Result,mode)

    %% Quadrotor UAV States

    x_atti     =  UAV_Result.x_atti;                                        % [-] Real attitude
    xhat_atti  =  UAV_Result.xhat_atti;                                     % [-] Estimated attitude
    
    x_alti     =  UAV_Result.x_alti;                                        % [-] Real altitude
    xhat_alti  =  UAV_Result.xhat_alti;                                     % [-] Estimated altitude
    
    x_posi     =  UAV_Result.x_posi;                                        % [-] Real position
    xhat_posi  =  UAV_Result.xhat_posi;                                     % [-] Estimated position
    
    xr_atti    =  UAV_Result.xr_atti;
    xr_alti    =  UAV_Result.xr_alti;
    xr_posi    =  UAV_Result.xr_posi;

    Quadrotor  =  UAV_Result.Quadrotor;                                                       
        
    % figure(1)
    % plot(T_posi,member_w1,'LineWidth',2.5); 
    % hold on; 
    % plot(T_posi,member_w2,'LineWidth',2.5); 
    % hold on; grid on;
    % legend('$w_1(k)$','$w_2(k)$','interpreter','latex')
    
    % figure(2)
    % plot(T_atti,xhat_atti(1,:)*180/pi,'LineWidth',2.5); 
    % hold on; 
    % plot(T_atti,xhat_atti(3,:)*180/pi,'LineWidth',2.5); 
    % hold on; 
    % plot(T_atti,xhat_atti(5,:)*180/pi,'LineWidth',2.5); 
    % hold on;
    % plot(T_atti,xr_atti(1,:)*180/pi,'-.','LineWidth',2.5); 
    % hold on; 
    % plot(T_atti,xr_atti(3,:)*180/pi,'-.','LineWidth',2.5); 
    % hold on; 
    % plot(T_atti,xr_atti(5,:)*180/pi,'-.','LineWidth',2.5); 
    % hold on; grid on;
    % legend('$\hat{\phi}(k)$','$\hat{\theta}(k)$','$\hat{\psi}(k)$','$\phi_r(k)$','$\theta_r(k)$','$\psi_r(k)$','interpreter','latex','Fontsize',12)
    % xlabel('Time [s]')
    % ylabel('Angle [degree]')
    % 
    % figure(3)
    % plot(T_posi,xr_alti(1,:),'Color',"#0072BD",'LineWidth',2.5); 
    % hold on; 
    % plot(T_posi,xhat_alti(1,:),'-.','LineWidth',2.5); 
    % hold on; grid on;
    % 
    % figure(4)
    % plot3(xhat_posi(1,:),xhat_posi(2,:),xhat_alti(1,:),'LineWidth',2.5); hold on;
    % plot3(xr_posi(1,:),xr_posi(2,:),xr_alti(1,:),'-.','LineWidth',2.5); hold on; grid on;
    % xlim([-1 15])
    % ylim([-1 15])
    % zlim([0 50])
    % xlabel('X Position [m]')
    % ylabel('Y Position [m]')
    % zlabel('Z Position [m]')


    if mode == 0
    elseif mode == 1
    
        drone1      =  Quadrotor{1};
        drone2      =  Quadrotor{2};
        drone3      =  Quadrotor{3};
        drone4      =  Quadrotor{4};

        x_init      =  0;
        y_init      =  0;
        z_init      =  0;

        X_Position  =  xhat_posi(1,:);
        Y_Position  =  xhat_posi(2,:);
        Z_Position  =  xhat_alti(1,:);

        AXIS        =  30;

        for j = 1 : 1 : 300

            figure(1)
            set(gcf,'Position',[200, 200, 900, 600],'Color','w')

            plot3(xr_posi(1,:),xr_posi(2,:),xr_alti(1,:),'-.','LineWidth',3); hold on
            plot3(x_posi(1,:),x_posi(2,:),x_alti(1,:),'LineWidth',2); hold on
            xlabel('X Position [m]','fontsize',15,'fontname','Times New Roman')
            ylabel('Y Position [m]','fontsize',15,'fontname','Times New Roman')
            zlabel('Z Position [m]','fontsize',15,'fontname','Times New Roman')

            % Center
            scatter3(X_Position(j), Y_Position(j), Z_Position(j), 'MarkerEdgeColor', [0 0 0], 'LineWidth', 1.5); hold on
            axis([-AXIS, x_init+AXIS, -AXIS, y_init+AXIS, 0, z_init+AXIS]); grid on;
    
            % Rotor 1
            scatter3(drone1(1,j), drone1(2,j), drone1(3,j), 'MarkerEdgeColor', [0 0 0], 'LineWidth', 1.5); hold on
            plot3([X_Position(j) drone1(1,j)], [Y_Position(j) drone1(2,j)], [Z_Position(j) drone1(3,j)],'LineWidth', 1.5, 'Color', 'red'); hold on
            %textscatter3(real([drone1(1,j) drone1(2,j) 0.2+drone1(3,j)]), string(1), 'MarkerSize',12);
    
            % Rotor 4
            scatter3(drone2(1,j), drone2(2,j), drone2(3,j), 'MarkerEdgeColor', [0 0 0], 'LineWidth', 1.5); hold on
            plot3([X_Position(j) drone2(1,j)], [Y_Position(j) drone2(2,j)], [Z_Position(j) drone2(3,j)],'LineWidth', 1.5, 'Color', 'blue'); hold on
            %textscatter3(real([drone2(1,j) drone2(2,j) 0.2+drone2(3,j)]), string(4), 'MarkerSize',12);
    
            % Rotor 3
            scatter3(drone3(1,j), drone3(2,j), drone3(3,j),'MarkerEdgeColor', [0 0 0], 'LineWidth', 1.5); hold on
            plot3([X_Position(j) drone3(1,j)], [Y_Position(j) drone3(2,j)], [Z_Position(j) drone3(3,j)],'LineWidth', 1.5, 'Color', 'red'); hold on
            %textscatter3(real([drone3(1,j) drone3(2,j) 0.2+drone3(3,j)]), string(3), 'MarkerSize',12);
    
            % Rotor 2
            scatter3(drone4(1,j), drone4(2,j), drone4(3,j), 'MarkerEdgeColor', [0 0 0], 'LineWidth', 1.5); hold on
            plot3([X_Position(j) drone4(1,j)], [Y_Position(j) drone4(2,j)], [Z_Position(j) drone4(3,j)],'LineWidth', 1.5, 'Color', 'blue'); hold on
            %textscatter3(real([drone4(1,j) drone4(2,j) 0.2+drone4(3,j)]), string(2), 'MarkerSize',12);
    
            hold on
            axis([-10, 20, -10, 20, 0, z_init+AXIS]);
            grid on; 
            hold off

            j

        end
    end

end