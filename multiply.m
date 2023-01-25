function u = multiply(k,j,l) %k defines the instant prediction is made
global N_pred                % k+j is te starting point for multiplication
global A                     % K+N_pred-l is the last element
  u=eye(3);
for n=3*(k+j)-2:3:3*(k+N_pred-l)-2   
    if 3*(k+N_pred-l)>length(A)
            break
        end     
    u=u*A(1:3,n:n+2);
           
 end
return
end