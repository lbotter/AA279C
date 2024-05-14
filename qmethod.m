function [A] = qmethod(M,V,w)
%QMETHOD Function to implement the q_method
% INPUT: M: measurement matrix 3xn in principal frame
%        V: reference matrix according to on-board model (inertial frame)
%        3xn
%        W: vector with measurement weights 
% OUTPUT: A is the DCM from inertia to principal 3x3

if isrow(w)
    w=w';
end

w=sqrt(w);
w = repmat(w,1,size(M,2));
U=w.*V;
W=w.*M;
B=W*U';
S=B+B';
Z=B(2,3)-B(3,2);B(3,1)-B(1,3);B(1,2)-B(2,1);
sigma=trace(B);
K=[S-eye(size(S,1)), Z; Z',sigma];
[V, D] = eig(K);
% Extract the maximum eigenvalue and its corresponding eigenvector
[maxEigenvalue, maxEigenvalueIndex] = max(diag(D));
q= V(:, maxEigenvalueIndex);
A=quat2dcm(q);