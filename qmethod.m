function [q,A] = qmethod(M,V,w)
%QMETHOD Function to implement the q_method
% INPUT: M: measurement matrix 3xn in principal frame
%        V: reference matrix according to on-board model (inertial frame)
%        3xn
%        W: vector with measurement weights 
% OUTPUT: A is the DCM from inertia to principal 3x3

if isrow(w)
    w=w';
end

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
    w=[(w(1)+w(2))/2; (w(1)+w(2))/2; (w(1)+w(2))/2];
end

w = repmat(w,1,size(M,1))';
w=sqrt(w);
U=w.*V;
W=w.*M;
B=W*U';
S=B+B';
Z=[B(2,3)-B(3,2);B(3,1)-B(1,3);B(1,2)-B(2,1)];
sigma=trace(B);
K=[S-eye(size(S,1)), Z; Z',sigma];
[V, D] = eig(K);
% Extract the maximum eigenvalue and its corresponding eigenvector
[maxEigenvalue, maxEigenvalueIndex] = max(diag(D));
q= V(:, maxEigenvalueIndex);
q=[q(4);q(1:3)];
q=q'/norm(q);
if q(1)<0
    q=q*(-1);
end
A=quat2dcm(q);
