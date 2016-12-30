function [Ad, Bd, Cd] = linearize_quadrotor(x, u, constants)
    
    % Quadrotor inputs (from Andrew Gibiansky's model)
    m = constants.m;
    L = constants.L;
    k = constants.k;
    b = constants.b;
    I = constants.I;
    kd = constants.kd;
    g = constants.g;
    dt = constants.dt;
    
    % Calculations
    Ixx = I(1); Ixx1 = 1/Ixx;
    Iyy = I(5); Iyy1 = 1/Iyy;
    Izz = I(9); Izz1 = 1/Izz;

    syms x3 x4 x5 x6 x7 x8 x9 x10 x11 x12
    syms u1 u2 u3 u4

    dfdx = jacobian([x6, ...
                     -(kd/m)*x4, ...
                     -(kd/m)*x5, ...
                     -(kd/m)*x6, ...
                     x10 + (sin(x8)*cos(x8))/(cos(x8)*sin(x7)*(cos(x8) + sin(x7)))*x11 + (sin(x8)*sin(x7))/(cos(x8)*sin(x7)*(cos(x8) + sin(x7)))*x12, ...
                     (1/(cos(x8) + sin(x7)))*x11 - (1/(cos(x8) + sin(x7)))*x12, ...
                     (sin(x7)/(cos(x8)*sin(x7)*(cos(x8) + sin(x7))))*x11 + (cos(x7)/(cos(x8)*sin(x7)*(cos(x8) + sin(x7))))*x12, ...
                     ((Iyy - Izz)/Ixx)*x11*x12, ...
                     ((Izz - Ixx)/Iyy)*x10*x12, ...
                     ((Iyy - Izz)/Ixx)*x10*x11], ...
                     [x3, x4, x5, x6, x7, x8, x9, x10, x11, x12]);

    dfdu = jacobian([0, ...
                     (1/m)*sin(x8)*sin(x9)*(u1 + u2 + u3 + u4), ...
                    -(1/m)*cos(x9)*sin(x8)*(u1 + u2 + u3 + u4), ...
                    -g + (1/m)*cos(x8)*(u1 + u2 + u3 + u4), ...
                     0, 0, 0, ...
                     Ixx1*L*k*(u1 - u3), ...
                     Iyy1*L*k*(u2 - u4), ...
                     Izz1*b*(u1 - u2 + u3 - u4)], ...
                     [u1, u2, u3, u4]);

    x = num2cell(x);
    [x3, x4, x5, x6, x7, x8, x9, x10, x11, x12] = deal(x{:});
    
    u = num2cell(u);
    [u1, u2, u3, u4] = deal(u{:});
    
    A = double(subs(dfdx));
    B = double(subs(dfdu));
    C = eye(10);
    
    % determine digital state space equations
    sysd = ss(A,B,C,0,dt);
    Ad = sysd.a;
    Bd = sysd.b;
    Cd = sysd.c;
    
end


