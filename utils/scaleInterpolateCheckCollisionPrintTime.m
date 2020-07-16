function [t,p,v,a]=scaleInterpolateCheckCollisionPrintTime(pk,vk,ak,k,N,vmax,amax,h,E1,order,rmin,pf,fileID)
%Scale to satisfy amax and vmax
    for i=1:N
        ak_mod(:,i) = amax./sqrt(sum(ak(:,:,i).^2,1));
        vk_mod(:,i) = vmax./sqrt(sum(vk(:,:,i).^2,1));
    end
    r_factor = min([min(min(ak_mod)), min(min(vk_mod))]);
    h_scaled = h/sqrt(r_factor);

    % Time settings and variables
    T = (k-2)*h_scaled; % Trajectory final time
    tk = 0:h_scaled:T;
    Ts = 0.01; % period for interpolation @ 100Hz
    t = 0:Ts:T; % interpolated time vector
    K = T/h_scaled + 1;

    % Compute new velocity and acceleration profiles
    for i = 1:N
        for k = 1:size(pk,2)-1
            ak(:,k,i) = ak(:,k,i)*r_factor;
            vk(:,k+1,i) = vk(:,k,i) + h_scaled*ak(:,k,i);
            pk(:,k+1,i) = pk(:,k,i) + h_scaled*vk(:,k,i) + h_scaled^2/2*ak(:,k,i);
        end
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Interpolate solution with a 100Hz sampling
for i = 1:N
    p(:,:,i) = spline(tk,pk(:,:,i),t);
    v(:,:,i) = spline(tk,vk(:,:,i),t);
    a(:,:,i) = spline(tk,ak(:,:,i),t); 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    violation=0;
    safety_ratio=1e10;
    % Check if collision constraints were not violated
    for i = 1:N
        for j = 1:N
            if(i~=j)
                differ = E1*(p(:,:,i) - p(:,:,j));
                dist = (sum(differ.^order,1)).^(1/order);
                safety_ratio=min(min(dist)/rmin,safety_ratio);
                if min(dist) < (rmin) %was "<(rmin-0.05)" in the original code, but that doesn't prevent a collision
                    [value,index] = min(dist);
                    violation = 1;
                    fprintf("Collision constraint violated after interpolation by %.2fcm: vehicles %i and %i @ k = %i \n", (rmin -value)*100,i,j,index)
                end
            end
        end
    end

    % Calculate how much time is required to complete the transition
    % within a 5cm margin of the goal

    for i = 1:N
        differ = p(:,:,i) - repmat(pf(:,:,i),length(t),1)';
        dist = sqrt(sum(differ.^2,1));
        hola = find(dist >= 0.05,1,'last');
        if isempty(hola)
            time_index(i) = 0;
        else
            time_index(i) = hola + 1;
        end
    end
    max_time_index = max(time_index);
    fprintf("Total flight time(seconds): %.2f \n",max_time_index*Ts);
    fprintf(fileID,"Total flight time(seconds): %.2f \n",max_time_index*Ts)
    totdist_dmpc = sum(sum(sqrt(diff(p(1,:,:)).^2+diff(p(2,:,:)).^2+diff(p(3,:,:)).^2)));
    fprintf("Total Flight Distance (m): %.2f\n",totdist_dmpc);
    fprintf(fileID,"Total Flight Distance (m): %.2f\n",totdist_dmpc)
    
    if ~violation
        fprintf("No collisions found, safety ratio: %.3f \n",safety_ratio)
        fprintf(fileID,"No collisions found, safety ratio: %.3f \n",safety_ratio)
    else
        fprintf("Collisions found, safety ratio: %.3f \n",safety_ratio)
        fprintf(fileID,"Collisions found, safety ratio: %.3f \n",safety_ratio)
    end
    
end