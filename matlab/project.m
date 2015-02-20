function [ D ] = project(P,C)
    Cok=[C(:,1:3) ones(size(C,1),1)]';
    Dbad=(P*Cok)';
    D=Dbad(:,1:3);
end