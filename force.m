function [f_contact, k, d_contact] = force(dipl_node,coord_find,...
    pos_dipl_node,Inv_Kg)
%force - Find node where force applies and calculate force vector
%
% Syntax: [f_contact, k, d_contact] = force(dipl_node,coord_find,...
%                                    pos_dipl_node,Inv_Kg)
%
% Inputs:
%    dipl_node - coordinater with the position where force applies
%    coord_find - matrix with nodes coordinates we used into find function
%    pos_dipl_node - the position where the user want to move the node
%    Inv_Kg- inverse global stiffness matrix
%
% Outputs:
%    f_contact - force vector
%    k - the node which deformed
%    d_contact - the diplacement vector
%
%
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
%
% Author: Dimitris Dounas
% Work address: none
% email: jdounas1992@gmail.com
% Website: none
% May 2015; Last revision: none

%------------- BEGIN CODE --------------

% find the node which will deform

k = find(coord_find(:,1)==dipl_node(:,1));
l = find(coord_find(:,2)==dipl_node(:,2));
m = find(coord_find(:,3)==dipl_node(:,3));

% check if node exists
if (k~=l) & (k~=m)
    disp('error with the point')
end
% the position of degrees of freedom in global stiffness matrix

c=k*3;
b=c-1;
a=b-1;

% calculate K_bb which we use to calculate f_contact
K_bb=  [Inv_Kg(a,a) Inv_Kg(a,b) Inv_Kg(a,c);
    Inv_Kg(b,a) Inv_Kg(b,b) Inv_Kg(b,c);
    Inv_Kg(c,a) Inv_Kg(c,b) Inv_Kg(c,c)];

Inv_K_bb=inv(K_bb);
% calculate d_contact
d_contact = [pos_dipl_node(:,1)-dipl_node(:,1)...
    pos_dipl_node(:,2)-dipl_node(:,2) pos_dipl_node(:,3)-dipl_node(:,3)]';
%calculate force vector
f_contact = Inv_K_bb* d_contact;


end %function

%------------- END OF CODE --------------
