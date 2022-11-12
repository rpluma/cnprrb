% #7 Tipos de datos - Numéricos
a=2;
x=-35.2;
x=2.23e-3;
a=2.3+8j;
X=7;

clear X x;
clear;

x=pi;
y=NaN;
z=Inf;
t=0*z;
u=z*0;
v=2^8;
clear;

% #8 Tipos de datos - Operaciones

x=exp(1);
y=log(x);
z=log2(256);
t=log10(1000);
u=sqrt(2);
clear;

x=sin(pi/4);
y=cos(pi/2);
z=tan(pi/4);
u=acos(0);
v=atan(-1);
% w=atan2(-1); no funciona
clear;

x=abs(-2);
y=sign(-2);
z=int8(257);
t=int16(257);
u=round(3.49,1);
clear;

z=3+2j;
x=real(z);
y=imag(z);
ab=abs(z);
an=angle(z);
cn=conj(z);
clear;

% #9 Tipos de datos - Definición de vectores
v=[2 3 4];
w=-10:1:5;
l=length(w);
x=linspace(-5, 10, 30);
y=logspace(-5, 10, 50);
clear;

% #10 Tipos de datos - Definición de matrices
M=[3 4 5;6 7 8; 1 -1 10];
M=[];
M=zeros(2, 4);
M=ones(2, 4);
M=eye(2, 4);
M=rand(2, 4);
M=[zeros(2, 2) ones(2, 3)];
X=zeros(2,2); Y=ones(3, 2); M=[X; Y]; % M=[zeros(2, 2); ones(2 2)]; no funciona

X=M(:,1);
X=M(1,:);

%11 Acceso a elementos
M;
M(1:2,1:2);
M(3,:);
M(5:-2:1, : ); % filas 5, 3 y 1
M(3:end, :);
M(3:end-1, :);
M(2,:)=[2, 2];
M(:,2)=M(:,1)+M(:,2);
clear;

%12 Tipos de datos - texto
t='texto de prueba';
t(2:4);
fprintf('teto formateado %0.1f',2.333);
% error('muestra y termina')



