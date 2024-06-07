function [ par, dervs ] = AcftConstDynDervs( dt, u, w, par0, varargin )
%AircraftConstDyn performs one step of one motion under simple aircraft 
%dynamics with constant controls. 
%   dt - time interval (raw matrix), v0 - initial velocity, phi0 - initial course angle,
%   u - lateral acceleration, w - tangential acceleration. x0, z0 - initial position. 
%   x1, z1 - final position, v1 - final course angle, phi1 - final
%   velocity. x-axis is directed to north, z-axis is directed to east, phi
%   angle is counted clockwise. It is suggested
%   then inital states and controls are scalar values but "dt" is not. 

    % Надо переделать на counterclockwise
    
    % Защита от дурака, то есть меня. 
    if size(dt, 1) > 1 && size(dt, 2) == 1; dt = dt'; end; 
    
    % Переводим удобный снаружи вектор в удобные внутри переменные. 
    [ x0, z0, phi0, v0 ] = unvec(par0); 
    n = numel(dt); 

    % Новая скорость. 
    v1 = v0 + w * dt;
    % Принудительно ограничиваем скорость. 
    sel = v1 < 1e-3; v1(sel) = 1e-3; 
    % Выбираем элементы с ненулевым продольным ускорением. 
    if abs(w) > 1e-8
        % Интегрирование угла для ненулевого продольного ускорения. 
        dphi_du = (1 / w) * reallog(v1 / v0);
    else
        % Интегрирование угла для нулевого продольного ускорения.
        dphi_du = (1 / v0) * dt;
    end
    phi1 = phi0 + dphi_du * u;
    % 
    k = u.^2 + 4 * w.^2;
    a = nan; b = nan; 
    s_phi1 = sin(phi1); c_phi1 = cos(phi1);
    s_phi0 = sin(phi0); c_phi0 = cos(phi0);
    v1_2 = v1.^2; v0_2 = v0.^2; 
    if k > 1e-16
        % Ненулевое ускорение хоть по какой-то оси. 
        a = u / k; b = 2 * w / k;
        % Случай осей xz и phi по часовой стрелке. 
        dx = b * (v1_2 .* c_phi1 - v0_2 * c_phi0) + a * (v1_2 .* s_phi1 - v0_2 * s_phi0);
        dz = b * (v1_2 .* s_phi1 - v0_2 * s_phi0) - a * (v1_2 .* c_phi1 - v0_2 * c_phi0);
        % Случай осей xz и phi против часовой стрелки. 
%         dx =  b * (v1_2 .* c_phi1 - v0_2 * c_phi0) - a * (v1_2 .* s_phi1 - v0_2 * s_phi0);
%         dz = -b * (v1_2 .* s_phi1 - v0_2 * s_phi0) - a * (v1_2 .* c_phi1 - v0_2 * c_phi0);
    else
        % Прямолинейное равномерное движение. 
        dx = c_phi0 * v0 * dt;
        dz = s_phi0 * v0 * dt;
    end
    x1 = x0 + dx; z1 = z0 + dz; 
    
    [ par ] = [ x1; z1; phi1; v1 ]; 
    dervs = [];
    
    % Смотрим, надо ли вычислять производные. 
    if numel(varargin) > 0 && ~isempty(strfind(varargin{1}, 'der'))
        % 
        dv_dw = dt; dv_dt = w * ones(1, n); 
        % 
        dphi_dv0 = (- u / v0) * dt ./ v1;
%         dphi_du = dphi / u;
        if abs(w) > 1e-8
            dphi_dw = (u / w) * (dt ./ v1 - dphi_du);
        else
            dphi_dw = (-0.5 * u / v0_2) * dt.^2; 
        end
        % 
        dx_dphi0 = -dz; 
        dz_dphi0 = dx; 
        if k > 1e-16
            dx_dv0 = ...
                b * (2 * v1 .* c_phi1 - 2 * v0 * c_phi0 - v1_2 .* s_phi1 .* dphi_dv0) + ...
                a * (2 * v1 .* s_phi1 - 2 * v0 * s_phi0 + v1_2 .* c_phi1 .* dphi_dv0);
            dz_dv0 = ...
                b * (2 * v1 .* s_phi1 - 2 * v0 * s_phi0 + v1_2 .* c_phi1 .* dphi_dv0) - ...
                a * (2 * v1 .* c_phi1 - 2 * v0 * c_phi0 - v1_2 .* s_phi1 .* dphi_dv0);
%             dx_dv0 = ...
%                 b * (  2 * v1 .* c_phi1 - 2 * v0 * c_phi0  + v1_2 .* s_phi1 .* dphi_dv0) + ...
%                 a * (-(2 * v1 .* s_phi1 - 2 * v0 * s_phi0) + v1_2 .* c_phi1 .* dphi_dv0);
%             dz_dv0 = ...
%                 b * (-(2 * v1 .* s_phi1 - 2 * v0 * s_phi0) - v1_2 .* c_phi1 .* dphi_dv0) - ...
%                 a * (  2 * v1 .* c_phi1 - 2 * v0 * c_phi0  - v1_2 .* s_phi1 .* dphi_dv0);
            %
            da_du = b^2 - a^2; da_dw = -4 * a * b; 
            db_du = 0.5 * da_dw; db_dw = -2 * da_du;
            %
            dx_du = ...
                db_du * (v1_2 .* c_phi1 - v0_2 * c_phi0) + ...
                da_du * (v1_2 .* s_phi1 - v0_2 * s_phi0) + ...
                b * (-v1_2 .* s_phi1 .* dphi_du) + ...
                a * ( v1_2 .* c_phi1 .* dphi_du);
            dz_du = ...
                db_du * (v1_2 .* s_phi1 - v0_2 * s_phi0) - ...
                da_du * (v1_2 .* c_phi1 - v0_2 * c_phi0) + ...
                b * ( v1_2 .* c_phi1 .* dphi_du) - ...
                a * (-v1_2 .* s_phi1 .* dphi_du);
            dx_dw = ...
                db_dw * (v1_2 .* c_phi1 - v0_2 * c_phi0) + ...
                da_dw * (v1_2 .* s_phi1 - v0_2 * s_phi0) + ...
                b * (2 * v1 .* dt .* c_phi1 - v1_2 .* s_phi1 .* dphi_dw) + ...
                a * (2 * v1 .* dt .* s_phi1 + v1_2 .* c_phi1 .* dphi_dw);
            dz_dw = ...
                db_dw * (v1_2 .* s_phi1 - v0_2 * s_phi0) - ...
                da_dw * (v1_2 .* c_phi1 - v0_2 * c_phi0) + ...
                b * (2 * v1 .* dt .* s_phi1 + v1_2 .* c_phi1 .* dphi_dw) - ...
                a * (2 * v1 .* dt .* c_phi1 - v1_2 .* s_phi1 .* dphi_dw);
        else
            dx_dv0 = dt .* c_phi1; 
            dz_dv0 = dt .* s_phi1; 
            %
            dx_du = -0.5 * s_phi0 * dt.^2; 
            dz_du =  0.5 * c_phi0 * dt.^2; 
            dx_dw =  0.5 * c_phi0 * dt.^2; 
            dz_dw =  0.5 * s_phi0 * dt.^2; 
        end
        % 
        dx_dt = v1 .* c_phi1; dz_dt = v1 .* s_phi1;
        dphi_dt = u ./ v1; % dv_dt = w * ones(1, n); 
        % 
        dervs.dpar_dpar0 = cell(1, n);
        for i = 1 : n
            dervs.dpar_dpar0{i} = eye(4); 
            dervs.dpar_dpar0{i}(3, 4) = dphi_dv0(i); 
            dervs.dpar_dpar0{i}(2, 4) = dz_dv0(i); 
            dervs.dpar_dpar0{i}(1, 4) = dx_dv0(i); 
            dervs.dpar_dpar0{i}(2, 3) = dz_dphi0(i); 
            dervs.dpar_dpar0{i}(1, 3) = dx_dphi0(i); 
        end
        % 
        dervs.dpar_du = [dx_du; dz_du; dphi_du; zeros(1, n)]; 
        dervs.dpar_dw = [dx_dw; dz_dw; dphi_dw; dv_dw]; 
        dervs.dpar_dt = [dx_dt; dz_dt; dphi_dt; dv_dt];
    end

end

function [ x, z, phi, v ] = unvec(par)
    x = par(1); z = par(2); phi = par(3); v = par(4);
end
