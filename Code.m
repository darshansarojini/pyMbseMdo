function [Aircraft]= Code()
model = systemcomposer.openModel("Product");
temp = lookup(model,Path="Product/Aircraft");
abc = temp.getParameterNames;
l=length(abc);
Aircraft=zeros(l,1);
for i = 1:l
    abc1 = abc(i);
    [paramValue,paramUnits,isDefault] = temp.getParameterValue(abc(i));
    para=str2double(paramValue);
    Aircraft(i,1) = para;
end
% for k=1:l
%     assignin('base',abc(k),abc2(k)); 
% end
end