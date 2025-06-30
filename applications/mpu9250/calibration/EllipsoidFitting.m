function EllipsoidFitting(raw)
%   only selet the x,y,z values for acceleration or magnet
    format long
    raw = table2array(raw);
    tmp = size(raw);
    psi = [raw(:,2).^2 raw(:,3).^2 raw(:,1) raw(:,2) raw(:,3) ones(tmp(1,1),1)];
    Y = -(raw(:,1).^2);
    P = inv(transpose(psi)*psi)
    theta = P*transpose(psi)*Y
    x_c = theta(3,1)/-2;
    y_c = theta(4,1)/(-2*theta(1,1));
    z_c = theta(5,1)/(-2*theta(2,1));
    x_r = sqrt(x_c^2+theta(1,1)*y_c^2+theta(2,1)*z_c^2-theta(6,1));
    y_r = sqrt(x_r^2/theta(1,1));
    z_r = sqrt(x_r^2/theta(2,1));
    ret = [x_c;y_c;z_c;x_r;y_r;z_r]
%   draw the ellipsoid and the raw data points
    figure(1)
    
    subplot(2,2,1)
    [x,y,z] = ellipsoid(x_c,y_c,z_c,x_r,y_r,z_r);
    surf(x, y, z)
    title('磁力计原始数据');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    hold on
    alpha 0.2
    plot3(raw(:,1),raw(:,2),raw(:,3),'.')
    axis equal vis3d;   
    view(3);
    
    subplot(2,2,2)
    scatter(raw(:,1),raw(:,2))
    xlabel('X'); ylabel('Y');
    axis equal;        % 等比例显示 + 紧凑边界
    title('XY平面视图');
    grid on;
    
    subplot(2,2,3)
    scatter(raw(:,1),raw(:,3))
    xlabel('X'); ylabel('Z');
    axis equal;        % 等比例显示 + 紧凑边界
    title('XZ平面视图');
    grid on;
    
    subplot(2,2,4)
    scatter(raw(:,2),raw(:,3))
    xlabel('Y'); ylabel('Z');
    axis equal;        % 等比例显示 + 紧凑边界
    title('YZ平面视图');
    grid on;
    
    set(gcf, 'Color', 'w'); % 设置背景为白色
    
    figure(2)
    
    subplot(2,2,1)
    [x_uint,y_uint,z_uint] = ellipsoid(0,0,0,1,1,1);
    surf(x_uint,y_uint,z_uint)
    title('磁力计校准后数据');
    hold on
    alpha 0.2
    x_calib = 1/x_r*(raw(:,1)-x_c);
    y_calib = 1/y_r*(raw(:,2)-y_c);
    z_calib = 1/z_r*(raw(:,3)-z_c);
    plot3(x_calib,y_calib,z_calib,'.')
    axis equal vis3d;   
    view(3);
    
    subplot(2,2,2)
    scatter(x_calib,y_calib)
    xlabel('X'); ylabel('Y');
    axis equal;        % 等比例显示 + 紧凑边界
    title('XY平面视图');
    grid on;
    
    subplot(2,2,3)
    scatter(x_calib,z_calib)
    xlabel('X'); ylabel('Z');
    axis equal;        % 等比例显示 + 紧凑边界
    title('XZ平面视图');
    grid on;
    
    subplot(2,2,4)
    scatter(y_calib,z_calib)
    xlabel('Y'); ylabel('Z');
    axis equal;        % 等比例显示 + 紧凑边界
    title('YZ平面视图');
    grid on;
    
    set(gcf, 'Color', 'w'); % 设置背景为白色
    
end