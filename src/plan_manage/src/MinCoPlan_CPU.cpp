#include<plan_manage/se3_planner.h>
#include<se3gcopter/se3gcopter_cpu.hpp>

void MavGlobalPlanner::openData(std::string fileToOpen,
                                std::vector<MatrixXd>& hPolys)
{

  // the inspiration for creating this function was drawn from here (I did NOT copy and paste the code)
  // https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix

  // the input is the file: "fileToOpen.csv":
  // a,b,c
  // d,e,f
  // This function converts input file data into the Eigen matrix format



  // the matrix entries are stored in this variable row-wise. For example if we have the matrix:
  // M=[a b c
  //    d e f]
  // the entries are stored as matrixEntries=[a,b,c,d,e,f], that is the variable "matrixEntries" is a row vector
  // later on, this vector is mapped into the Eigen matrix format
  vector<double> matrixEntries;

  // in this object we store the data from the matrix
  ifstream matrixDataFile(fileToOpen);

  // this variable is used to store the row of the matrix that contains commas
  string matrixRowString;

  // this variable is used to store the matrix entry;
  string matrixEntry;

  // this variable is used to track the number of rows
  int matrixRowNumber = 0;


  while (getline(matrixDataFile, matrixRowString)) // here we read a row by row of matrixDataFile and store every line into the string variable matrixRowString
  {
    if (matrixRowString != "") {
      stringstream matrixRowStringStream(matrixRowString); //convert matrixRowString that is a string to a stream variable.

      while (getline(matrixRowStringStream, matrixEntry, ',')) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
      {
        matrixEntries.push_back(stod(matrixEntry));   //here we convert the string to double and fill in the row vector storing all the matrix entries
      }
      matrixRowNumber++; //update the column numbers
    }
    else  {
      // here we convet the vector variable into the matrix and return the resulting object,
      // note that matrixEntries.data() is the pointer to the first memory location at which the entries of the vector matrixEntries are stored;
      hPolys.push_back(Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber));
      matrixRowNumber = 0;
      matrixEntries.clear();
    }
  }
}

void saveSegments(const std::string filename,
                  const std::vector<std::pair<Vec3f, Vec3f>>& segments)
{
  std::cout << "START SAVING SEGMENTS" << std::endl;

  // Open the file for writing.
  std::ofstream seg_file (filename + "-seg.txt");
  if (!seg_file.is_open()) {
    std::cerr << "Unable to write file" << "\n";
    return;
  }


  std::stringstream ss_seg_coords;;

  for (const auto seg : segments) {
    // Create octant
    const Eigen::Vector3f start_coord(seg.first.x(), seg.first.y(), seg.first.z());

    ss_seg_coords << start_coord.x() << " "
                  << start_coord.y() << " "
                  << start_coord.z() << std::endl;

    const Eigen::Vector3f goal_coord(seg.second.x(), seg.second.y(), seg.second.z());

    ss_seg_coords << goal_coord.x() << " "
                  << goal_coord.y() << " "
                  << goal_coord.z() << std::endl;
  }
  std::cout << "FINISHED SAVING SEGMENTS" << std::endl;

  seg_file << ss_seg_coords.str();

  seg_file.close();

  return;
}

void MavGlobalPlanner::plan(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,vector<Eigen::Vector3d>* wp_list){
    std::cout << "has_map =                   " << (has_map ? "true" : "false") << std::endl;
    std::cout << "jps_pathfinder.has_map_() = " << (jps_pathfinder.has_map_() ? "true" : "false") << std::endl;
    if(!has_map||!jps_pathfinder.has_map_()) return;
    Vector3d zeroVec(0.0,0.0,0.0);
    Vector3d start_pt;
    Vector3d end_pt;
    start_pt = iniState.col(0);
    end_pt  = finState.col(0);
    vec_Vec3f path;

    std::string map_file;
    nh.param("map/file", map_file, std::string("/home/nils/workspace_/projects_/supereight-2-srl/test/planner/zhangjiajie"));

    jps_pathfinder.printFrames(Vecf<3>(start_pt[0],start_pt[1],start_pt[2]),Vecf<3>(end_pt[0],end_pt[1],end_pt[2]));

//     std::chrono::duration<double, std::milli> dur_a_star(0);
//     if(!wp_list){
//       auto start_a_star = std::chrono::high_resolution_clock::now();
//       jps_pathfinder.plan(Vecf<3>(start_pt[0],start_pt[1],start_pt[2]),Vecf<3>(end_pt[0],end_pt[1],end_pt[2]),1,false);
//       auto stop_a_star = std::chrono::high_resolution_clock::now();
//       path = jps_pathfinder.getSamplePath();
//       jps_pathfinder.publishAll();
//       dur_a_star = stop_a_star - start_a_star;
//     }
//     else{
//       std::cout << "START A* COMPUTATION" << std::endl;
//       auto start_a_star = std::chrono::high_resolution_clock::now();
//       vec_Vec3f path_0;
//       jps_pathfinder.plan(Vecf<3>(start_pt[0],start_pt[1],start_pt[2]),Vecf<3>((*wp_list)[0][0],(*wp_list)[0][1],(*wp_list)[0][2]),1,false);
//       path_0 = jps_pathfinder.getSamplePath();
//       auto stop_a_star = std::chrono::high_resolution_clock::now();
//       dur_a_star = stop_a_star - start_a_star;
//       for(int i = 0;i< wp_list->size()-1;i++){
//         start_a_star = std::chrono::high_resolution_clock::now();
//         vec_Vec3f tmp_path;
//         jps_pathfinder.plan(Vecf<3>((*wp_list)[i][0],(*wp_list)[i][1],(*wp_list)[i][2]),Vecf<3>((*wp_list)[i+1][0],(*wp_list)[i+1][1],(*wp_list)[i+1][2]),1,false);
//         tmp_path = jps_pathfinder.getSamplePath();
//         path_0.pop_back();
//         path_0.insert(path_0.end(),tmp_path.begin(),tmp_path.end());
//         stop_a_star = std::chrono::high_resolution_clock::now();
//         dur_a_star += stop_a_star - start_a_star;
//       }
//       start_a_star = std::chrono::high_resolution_clock::now();
//       vec_Vec3f end_path;
//       jps_pathfinder.plan(Vecf<3>((*wp_list)[wp_list->size()-1][0],(*wp_list)[wp_list->size()-1][1],(*wp_list)[wp_list->size()-1][2]),
//       Vecf<3>(end_pt[0],end_pt[1],end_pt[2]),1,false);
//       end_path = jps_pathfinder.getSamplePath();
//       path_0.pop_back();
//       path_0.insert(path_0.end(),end_path.begin(),end_path.end());
//       path = path_0;
//       stop_a_star = std::chrono::high_resolution_clock::now();
//       dur_a_star += stop_a_star - start_a_star;
//     }
//
//     std::cout << "Duration A* computation = " << dur_a_star.count() << " ms" << std::endl;
//
////     jps_pathfinder.saveMap(map_file, map_util, (*obs_pointer));
//
//     std::cout << "START ELLIPSOID EXPANSION" << std::endl;
//     auto start_poly = std::chrono::high_resolution_clock::now();
//
//     float max_segment_length;
//     nh.param("jps/max_segment_length", max_segment_length, 4.f);
//     EllipsoidDecomp3D decomp_util;
//     decomp_util.set_obs(*obs_pointer);
//     decomp_util.set_local_bbox(Eigen::Vector3d(config.polyhedronBox(0),
//                                                config.polyhedronBox(1),
//                                                config.polyhedronBox(2)));
//     vec_E<Polyhedron3D> decompPolys;
//     vec_E<Ellipsoid3D> ellips;
//     int i_prev = 0;
//     std::vector<std::pair<Vec3f, Vec3f>> segments;
//     for(int i = 0;i<path.size()-1;){
//         //find the farest unblocked point
//         int k;
//         for(k = i+1;k<path.size();k++){
//             if(map_util->isBlocked(path[i],path[k])||((path[i]-path[k]).norm()>=max_segment_length)){
//                 k--;
//                 break;
//             }
//         }
//         if(k<i+1){
//             k = i+1;
//         }
//         if(k>=path.size()) k = path.size()-1;
//         vec_Vec3f line;
//         line.push_back(path[i]);
//         line.push_back(path[k]);
//
//         Vec3f start_m = path[i];
//         Vec3f start_v = map_util->floatToFloat(start_m);
//         Vec3f goal_m = path[k];
//         Vec3f goal_v = map_util->floatToFloat(goal_m);
//
//         segments.push_back(std::make_pair(start_v, goal_v));
//
//         decomp_util.dilate(line);
//         Polyhedron3D poly = decomp_util.get_polyhedrons()[0];
//         Ellipsoid3D ellip = decomp_util.get_ellipsoids()[0];
//         decompPolys.push_back(poly);
//         ellips.push_back(ellip);
//         //find the nearest one to the boundry of poly.
//         int j;
//         for(j=k;j<path.size();j++){
//             Vec3f pt;
//             pt[0] = path[j][0];
//             pt[1] = path[j][1];
//             pt[2] = path[j][2];
//             if(!poly.inside(pt)){
//                 break;
//             }
//         }
//         j--;
//         if(j>=path.size()-1){
//             break;
//         }
//         int wp;
//         wp = round((1*i+4*j)/5);
//         i = wp;
//
//         if (i == i_prev) {
//           i++;
//         }
//         i_prev = i;
//     }
//
////     saveSegments(map_file, segments);
//
//     auto stop_poly = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double, std::milli> dur_poly = stop_poly - start_poly;
//
//     std::cout << "END ELLIPSOID EXPANSION" << std::endl;
//     std::cout << "Duration polynomial computation = " << dur_poly.count() << " ms" << std::endl;
//
////     float floor_height = -0.15f;
////     float ceiling_height = -1.35f + 4.f - 0.5f;
////     float wall_left_pos = -16.f;
////     float wall_right_pos = 16.f;
//
//    float floor_height = 0.0f;
//    float ceiling_height = 3.5f;
//
//    std::cout << "floor height   = " << floor_height << std::endl;
//    std::cout << "ceiling height = " << ceiling_height << std::endl;
////    std::cout << "wall left pos  = " << wall_left_pos << std::endl;
////    std::cout << "wall right pos = " << wall_right_pos << std::endl;
//
//     std::cout << "Number of polyhedra from map: " << decompPolys.size() << std::endl;
//     for (size_t i = 0; i < decompPolys.size(); i++)
//     {
//         decompPolys[i].add(Hyperplane3D(Eigen::Vector3d(0.0, 0.0, ceiling_height),
//                                           Eigen::Vector3d(0.0, 0.0, 1.0)));
//         decompPolys[i].add(Hyperplane3D(Eigen::Vector3d(0.0, 0.0, floor_height),
//                                           Eigen::Vector3d(0.0, 0.0, -1.0)));
////         decompPolys[i].add(Hyperplane3D(Eigen::Vector3d(wall_left_pos, 0.0, 0.0),
////                                         Eigen::Vector3d(-1.0, 0.0, 0.0)));
////         decompPolys[i].add(Hyperplane3D(Eigen::Vector3d(wall_right_pos, 0.0, 0.0),
////                                         Eigen::Vector3d(1.0, 0.0, 0.0)));
//     }
//     visualization.visualizePolyH(decompPolys);
//     vector<MatrixXd> hPolys;
//     Eigen::MatrixXd current_poly;
//     for (uint i = 0; i < decompPolys.size(); i++)
//     {
//         vec_E<Hyperplane3D> current_hyperplanes = decompPolys[i].hyperplanes();
//         current_poly.resize(6, current_hyperplanes.size());
//         for (uint j = 0; j < current_hyperplanes.size(); j++)
//         {
//             current_poly.col(j) << current_hyperplanes[j].n_, current_hyperplanes[j].p_;
//             //outside
//         }
//         hPolys.push_back(current_poly);
//     }
//     std::cout << "Number of polyhedra from map: " << hPolys.size() << std::endl;
//     std::cout<<"----------------------------------------------\n";
//     for(int i = 0;i<hPolys.size();i++){
//         std::cout<<"poly id: "<<i<<endl;
//         std::cout<<hPolys[i]<<std::endl;
//     }
//     std::cout<<"-------------------------------------------\n";



   std::cout << "LOAD HYPERPLANES" << std::endl;
   std::string fileToOpen = map_file + ".csv";
   vector<MatrixXd> hPolysRead;
   openData(fileToOpen, hPolysRead);

   std::cout << "hPolysRead.size() = " << hPolysRead.size() << std::endl;

//   float floor_height = 0.f;
//   float ceiling_height = 3.5f;
    float floor_height = -0.15f;
    float ceiling_height = -1.35f + 4.f - 0.5f;
    float wall_left_pos = -16.f;
    float wall_right_pos = 16.f;

   // std::cout << "floor height   = " << floor_height << std::endl;
   // std::cout << "ceiling height = " << ceiling_height << std::endl;
   // std::cout << "wall left pos  = " << wall_left_pos << std::endl;
   // std::cout << "wall right pos = " << wall_right_pos << std::endl;

   vec_E<Polyhedron3D> decompPolys;
   for (size_t i = 0; i < hPolysRead.size(); i++) {
     Polyhedron3D poly;
     for (size_t j = 0; j < hPolysRead[i].cols(); j++) {
       Eigen::Vector3d p = Eigen::Vector3d(hPolysRead[i].col(j)[3], hPolysRead[i].col(j)[4], hPolysRead[i].col(j)[5]);
       Eigen::Vector3d n = Eigen::Vector3d(hPolysRead[i].col(j)[0], hPolysRead[i].col(j)[1], hPolysRead[i].col(j)[2]);
       poly.add(Hyperplane3D(p, n));
     }
     Eigen::Vector3d floor_p = Eigen::Vector3d(0.0, 0.0, floor_height);
     Eigen::Vector3d floor_n = Eigen::Vector3d(0.0, 0.0, -1.0);
     poly.add(Hyperplane3D(floor_p, floor_n));
     Eigen::Vector3d ceiling_p = Eigen::Vector3d(0.0, 0.0, ceiling_height);
     Eigen::Vector3d ceiling_n = Eigen::Vector3d(0.0, 0.0, 1.0);
     poly.add(Hyperplane3D(ceiling_p, ceiling_n));

      Eigen::Vector3d wall_left_p = Eigen::Vector3d(wall_left_pos, 0.0, 0.0);
      Eigen::Vector3d wall_left_n = Eigen::Vector3d(-1.0, 0.0, 0.0);
      poly.add(Hyperplane3D(wall_left_p, wall_left_n));
      Eigen::Vector3d wall_right_p = Eigen::Vector3d(wall_right_pos, 0.0, 0.0);
      Eigen::Vector3d wall_right_n = Eigen::Vector3d(1.0, 0.0, 0.0);
      poly.add(Hyperplane3D(wall_right_p, wall_right_n));

     decompPolys.push_back(poly);
   }
   std::cout << "VISUALISE " << decompPolys.size() << " POLYHEDRA" << std::endl;
   visualization.visualizePolyH(decompPolys);

   vector<MatrixXd> hPolys;
   Eigen::MatrixXd current_poly;
   for (uint i = 0; i < decompPolys.size(); i++)
   {
       vec_E<Hyperplane3D> current_hyperplanes = decompPolys[i].hyperplanes();
       current_poly.resize(6, current_hyperplanes.size());
       for (uint j = 0; j < current_hyperplanes.size(); j++)
       {
           current_poly.col(j) << current_hyperplanes[j].n_, current_hyperplanes[j].p_;
           //outside
       }
       hPolys.push_back(current_poly);
   }

   std::chrono::duration<double, std::milli> dur_a_star(0);
   std::chrono::duration<double, std::milli> dur_poly(0);
   ///----------



    auto start_traj = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
    SE3GCOPTER nonlinOpt;
    Trajectory traj;
    ROS_INFO("Begin to optimize the traj~");
    if (!nonlinOpt.setup(config.rho, config.totalT, iniState, finState, hPolys, INFINITY,
                            config.qdIntervals, config.horizHalfLen, config.vertHalfLen,
                            config.safeMargin, config.velMax, config.thrustAccMin, config.thrustAccMax,
                            config.bodyRateMax, config.gravAcc, config.penaltyPVTB, config.useC2Diffeo))
    {
        return;
    }
    double finalObj = nonlinOpt.optimize(traj, config.optRelTol);
    std::chrono::high_resolution_clock::time_point toc = std::chrono::high_resolution_clock::now();
    double compTime = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1.0e-3;

    auto stop_traj = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> dur_traj = stop_traj - start_traj;

    std::cout << "Duration trajectory optimisation = " << dur_traj.count() << " ms" << std::endl;

    printf("finished!!!\n");
    std::cout << "Optimization time usage: " << compTime << " ms" << std::endl;
    std::cout << "Final cost: " << finalObj << std::endl;
    std::cout << "Maximum Vel: " << traj.getMaxVelRate() << std::endl;
    std::cout << "Maximum Acc: " << traj.getMaxAccRate() << std::endl;
    std::cout << "Total Duation: " << traj.getTotalDuration() << std::endl;

    std::cout << "A* [ms]\t" << "Poly comp [ms]\t" << "Traj opti [ms]\t" << "Poly #\t" << "Traj cost\t" << "Traj dur [s]" << std::endl;
    std::cout << dur_a_star.count() << "\t\t" << dur_poly.count() << "\t\t" << compTime << "\t\t" << decompPolys.size() << "\t" << finalObj << "\t\t" << traj.getTotalDuration() << std::endl;

    if (traj.getPieceNum() > 0)
    {
        lastIniStamp = ros::Time::now();

        visualization.visualize(traj);
        visualization.visualizeEllipsoid(traj,450);
        // visualization.visualizeQuadrotor(traj, 70);
    }
    quadrotor_msgs::PolynomialTrajectory traj_msg;
    traj_msg = traj2msg(traj);
    _traj_pub.publish(traj_msg);
}
