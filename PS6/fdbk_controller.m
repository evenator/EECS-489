function tau= fdbk_controller(H, DH, q_des, q_dot_des, q_2dot_des, q, q_dot)   
    kp=[
        2000 0;
        0 10000];
    kv=kp / 10;
    tau = kp * (q_des - q) + kv * (q_dot_des - q_dot);