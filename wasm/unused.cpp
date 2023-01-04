//void mypr(std::string s, Eigen::Matrix4d T){
//    std::cout <<"Matrix " << s << ": " << std::endl;
//    std::cout << T << std::endl;
//}
//
//void printKine(Kine kine){
//    auto el = kine.rootLink;
//    std::cout << "***** "<< kine.chainName << " *****" << std::endl;
//    while(kine.treeMap.count(el->name)){
//        std::cout << el->name << std::endl;
//        std::cout << el->absTransform << std::endl;
//        if(el->child_links.empty()) break;
//        el = el->child_links[0];
//    }
//}

//Trajectory rotTraj;
//
//while(t > ((loops_done_rot + 1) * rotTraj.getTotalTime())){
//++loops_done_rot;
//rotTraj.currentSegmentNumber = 0;
//}
//
//auto timeRot = t - ( loops_done_rot * rotTraj.getTotalTime() );
//auto [xr, yr, zr] = rotTraj.getStates(timeRot);

//Right arm trajectory
//    Eigen::MatrixXd m(4, 7); //points (x;y;z) for trajectory generation
//    m << 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
//         0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
//         0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
//         1.000, 1.000, 1.000, 1.000, 1.000, 1.000;
//
//    double vel = 0.01;
//
//    traj.computeMultiSegment(m, vel);

//    double t = 4.001;
//    auto [x1, y1, z1] = traj.getStates(t);
//    t += 0.001;
//    auto [x2, y2, z2] = traj.getStates(t);
//    t -= 0.002;
//    auto [x0, y0, z0] = traj.getStates(t);

//    auto x_diff = ( x2-x0 ) * 500.0;
//    std::cout << "x1: " << x1 << std::endl;
//    std::cout << "x2: " << x2 << std::endl;
//    std::cout << "x_diff: " << x_diff << std::endl;
