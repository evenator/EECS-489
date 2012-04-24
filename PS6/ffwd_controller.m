function tau= ffwd_controller(H, DH, q_des, q_dot_des, q_2dot_des, q, q_dot)   
    kp=[
        0 0;
        0 0];
    kv=[
        0 0;
        0 0];
    tau = gravitytorques(DH) + H * q_2dot_des;