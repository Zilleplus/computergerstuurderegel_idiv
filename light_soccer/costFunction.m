function [ J ] = costFunction(  goalkeeper,attacker )
    [ xl, xr,~,~,range_penality] = cover( goalkeeper,attacker );
%     if range_penality>0        
%         disp(num2str(range_penality))
%     end
    J = xl^2 + xr^2 + range_penality;
end