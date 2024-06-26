function [q,A] = deterministicAttitude(M,V)
%DETERMINISTIC_ATTITUDE Function that uses a determinitstic attitude
%approach to find the attitude matrix
% INPUT: M: measurement matrix 3xn in principal frame
%        V: reference matrix according to on-board model (inertial frame)
%        3xn
% OUTPUT: A is the DCM from inertia to principal 3x3

if size(M,2)<2 | size(V,2)~=size(M,2)
    fprintf('Error in matrix size');
    return
elseif size(M,2)==2
    P1_tilde=(M(:,1)+M(:,2))/2;
    P2_tilde=(M(:,1)-M(:,2))/2;
    m1=P1_tilde;
    m2=cross(P1_tilde,P2_tilde)/(norm(cross(P1_tilde,P2_tilde)));
    m3=cross(m1,m2);
    M=[m1,m2,m3];
    V1_tilde=(V(:,1)+V(:,2))/2;
    V2_tilde=(V(:,1)-V(:,2))/2;    
    v1=V1_tilde;
    v2=cross(V1_tilde,V2_tilde)/(norm(cross(V1_tilde,V2_tilde)));
    v3=cross(v1,v2);
    V=[v1,v2,v3];
end

A=M*V'*inv(V*V');
q=dcm2quat(A);
if q(1)<0
    q=q*(-1);
end




