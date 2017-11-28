%--------------------------------------------------------------------------
%Buckingham's PI-Theorem
%T. Tresch (20.02.2013)
%--------------------------------------------------------------------------
clear; clc;

%% Define dimensional variables with fundamental (SI) units

%Units Exponents:           [Length   Time   Mass   Temperature ... ....]
% Input variables with units
% Var={ 'v',          [1 -1 0 0], ...
%       'L',          [1 0 0 0], ...
%       '\rho',       [-3 0 1 0], ...
%       '\eta',       [-1 -1 1 0], ...
%       'f',          [0 -1 0 0]};


%add variables here
%script is extremely sensitive to changes in ordering of the variables


%[Length   Time   Mass   Temperature]
%                   [L Time M Temp]
Var={ 
      'length',     [1 0 0 0], ...          
      'area',       [2 0 0 0], ... 
      'volume',     [3 0 0 0], ...
      'velocity',   [1 -1 0 0], ...
      'soundspeed'  [1 -1 0 0], ...      
      'density',    [-3 0 1 0], ...
      'accel',      [1 -2 0 0], ...
      'pressure',   [-1 -2 1 0], ...
      'mass flow',  [0 -1 1 0], ... 
      'force',      [1 -2 1 0], ...      
      'temp',       [0 0 0 1], ...
      'viscosity',  [-1 -1 1 0], ...              
      'holeperimeter', [1 0 0 0], ...
      'holearea',   [2 0 0 0], ...
      };
%'height',     [1 0 0 0], ...      
      
%'dissipation', [0 -4 1 0], ...  
%'perimeter',  [1 0 0 0], ...
            
 
      
      
%% Calculate the PI-Groups

%Number of dimensional variables
N_dim=round(length(Var)/2);

%Construct the Units-Matrix
A=zeros(length(Var{2}),N_dim);
for k=1:N_dim
    A(:,k)=Var{2*k}';
end

%kernel (null-space) of A
N=null(A,'r');  

%Number of Pi-groups
[a,b]=size(N);
No_groups=b;




%% Write Latex code 

for n=1:No_groups
    %Find non-zero variables
    NZ=find(N(:,n)~=0);
    PI{n}=[];
    for k=1:length(NZ)
        PI{n}=[PI{n} Var{2*NZ(k)-1} ' ^{' num2str(N(NZ(k),n)) '} '];
    end
end

for n=1:No_groups
    %Find non-zero and negative exponents
    NZ=find(N(:,n)~=0);
    Neg=find(N(:,n)<0);
        if ~isempty(Neg)
            PI{n}=['\pi(' num2str(n) ')=\frac{'];     
        else
            PI{n}=['\pi(' num2str(n) ')={'];
        end
        
    for k=1:length(NZ)
        if N(NZ(k),n)>0
            PI{n}=[PI{n} Var{2*NZ(k)-1} '^{' num2str(N(NZ(k),n)) '}\cdot '];
        end
    end
    if ~isempty(Neg); PI{n}=[PI{n} '}{']; end
    
    for k=1:length(NZ)
        if N(NZ(k),n)<0
            PI{n}=[PI{n} Var{2*NZ(k)-1} ' ^{' num2str(-N(NZ(k),n)) '}\cdot '];
        end
    end
    PI{n}=[PI{n} '}'];
    PI{n}=['\begin{equation}' PI{n} '\end{equation}' ];
end
N
%replace: '^{1}' ---> ''  and  '\cdot }}' ----> '}'
for n=1:No_groups
    PI_Latex{n} = strrep(PI{n}, '^{1}', '');
    PI_Latex{n} = strrep(PI_Latex{n}, '\cdot }', '}');
    PI_Latex{n}
end

%% Write tex-file
fid = fopen('C:\Users\Jared\Documents\MATLAB\bar_rocket_drake\pi_groups.tex', 'w');
Tex_Header='\documentclass[11pt,a4paper]{scrartcl}\usepackage{amssymb,bm}\usepackage{amsmath}\begin{document}';
fprintf(fid,'%s\n',Tex_Header);

for n=1:No_groups
    fprintf(fid,'%s\n',PI_Latex{n});
end
fprintf(fid,'%s\n','\end{document}');
fclose(fid);

%--------------------------------------------------------------------------




