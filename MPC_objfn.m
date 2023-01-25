

function objfn = MPC_objfn( Ukf)
global  P H Q R x_r_change u_r_change xk_error 
    
x_hat=P*xk_error+H*Ukf;
delta_ukf=Ukf-u_r_change;
delta_x=x_hat-x_r_change;
objfn= delta_ukf'*R*delta_ukf + delta_x'*Q*delta_x;

end

