Matlab code used to obtain analytical Taylor Series Jacobians

```
clear; clc;
syms tht L_ tau_stall_ free_speed_ M_ r_w_ x y vl vr volt_l volt_r
m_dim_ = 2;
n_dim_ = 5;
J = [cos(tht), cos(tht);
    sin(tht), sin(tht);
    -1 / L_, 1 / L_];

% base states to wheel states Jacobian, J_plus
J_plus = [cos(tht), sin(tht), -L_;
          cos(tht), sin(tht), L_];

dJ_plus = [-1 * sin(tht), cos(tht), 0;
           -1 * sin(tht), cos(tht), 0];

up_right = J;
low_right = dJ_plus * J - J_plus * inv(M_) * J_plus.' * ...
            tau_stall_ / ((r_w_^2) * free_speed_);
lr_dim = size(low_right);
ur_dim = size(up_right);
up_left = zeros(n_dim_ - lr_dim(1), n_dim_ - ur_dim(2));
low_left = zeros(n_dim_ - ur_dim(1), n_dim_ - lr_dim(2));

H = [up_left, up_right;
     low_left, low_right];

C = [low_left.';
     J_plus * inv(M_) * J_plus.' * tau_stall_ / ((r_w_^2) * free_speed_)];

x_state = [x, y, tht, vl, vr].';
u_state = [volt_l, volt_r].';


F = H * x_state + C * u_state;

dF_dx = jacobian(F, x_state)
dF_du = jacobian(F, u_state);
```