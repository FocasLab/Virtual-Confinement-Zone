function save_data(D, R, Controller, Zn_check, J_check, n_obstacle, n_target,reach_state, type)
    if strcmp(type, "Mono")
        filename_controller = "10_10_D"+string(D)+"_R"+string(R)+"_Controller_Mono";
        filename_objects = "10_10_D"+string(D)+"_R"+string(R)+"_Objects_Mono";
    else
        filename_controller = "10_10_D"+string(D)+"_R"+string(R)+"_Controller";
        filename_objects = "10_10_D"+string(D)+"_R"+string(R)+"_Objects";
    end
    save(filename_controller, "Controller", "Zn_check", "J_check","-v7.3");
    save(filename_objects, "n_obstacle", "n_target","reach_state","-v7.3");
end