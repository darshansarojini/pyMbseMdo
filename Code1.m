function [Aircraft1]= Code1()
model = systemcomposer.openModel("Product");
temp = lookup(model,Path="Product/Aircraft/Taxi-Based");
abc = temp.getParameterNames;
l=length(abc);
Aircraft1=zeros(l,1);
for i = 1:l
    abc1 = abc(i);
    [paramValue,paramUnits,isDefault] = temp.getParameterValue(abc(i));
    para=str2double(paramValue);
    Aircraft1(i,1) = para;
end
% for k=1:l
%     assignin('base',abc(k),abc2(k)); 
% end
end