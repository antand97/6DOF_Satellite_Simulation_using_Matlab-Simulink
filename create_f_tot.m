function f_tot_sym = create_f_tot(x,y,z)
%CREATE_F_TOT
%    F_TOT_SYM = CREATE_F_TOT(X,Y,Z)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    28-Nov-2021 23:23:38

t2 = y.^2;
t3 = z.^2;
t4 = x+1.0e+2;
t5 = t4.^2;
t6 = t2+t3+t5;
t7 = 1.0./sqrt(t6);
t8 = t7.^3;
t9 = t7-2.0e+1./3.0;
f_tot_sym = [x.*-3.0+t4.*t8.*t9.*1.0e+4-6.0;y.*-3.0+t8.*t9.*y.*1.0e+4;z.*-3.0+t8.*t9.*z.*1.0e+4];
