function tau= controller(H, DH, q_des, q_dot_des, q_2dot_des, q, q_dot)   
    kp=[
        20 0;
        0 100];
    kv=kp / 10;
    tau = kp * (q_des - q) + kv * (q_dot_des - q_dot) + gravitytorques(DH);