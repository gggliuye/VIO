#include "ScaleEstimator.h"

double Estimator::pose_estimation_SVD(
	         const std::vector<Eigen::Vector3d>& pts1,
	         const std::vector<Eigen::Vector3d>& pts2,
	         Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    // calculate p1 p2
    Eigen::Vector3d p1(0,0,0), p2(0,0,0);
    int N = pts1.size();
    for (int i = 0; i<N; i++){
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = (p1) / N;
    p2 = (p2) / N;
    std::cout << p1.transpose() << " " <<  p2.transpose()<< "\n\n";

    std::vector<Eigen::Vector3d> q1(N), q2(N); 
    for (int i = 0; i<N; i++){
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }
 
    // 2. W = sum(q1*q2^T)
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i<N; i++){
        W += q1[i] * q2[i].transpose();
    }
    //std::cout << W << std::endl;

    // 3. SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    //std::cout << U << std::endl;
    //std::cout << V << std::endl;    

    // 4. R = U*V^T
    R = U * (V.transpose());

    // 5. scale
    double scale = 0;
    for(int i = 0 ; i < N ; i ++){
        Eigen::Vector3d tmp = R * q2[i];
        for(int k = 0 ; k < 3 ; k++){
            scale += q1[i](k)/tmp(k);
        }
    }
    scale /= (3*N);

    // 6. t
    t = p1 / scale - R * p2;
 
    return scale;
}


double Estimator::ScaleEstimator(Ulocal::LocalizationLY *pLocalizationLY, 
           const std::string &arcore_traj_file, const std::string &arcore_image_path, 
           double focus_length, int num_images)
{
    bool show_arcore = true;

    std::cout << "\n Test image from : " << arcore_image_path << std::endl;
    std::cout << "   image focus length : " << focus_length << std::endl;
    std::cout << "  " << num_images << " images to test.\n";

    ///////////////////// read pose estimations //////////////////////////
    std::cout << " Read Arcore pose estimations from file : " << arcore_traj_file << std::endl;
    std::ifstream fin_pose(arcore_traj_file, std::ios_base::in );
    if ( !fin_pose.is_open ( ) ){
        std::cout << " Cannot open arcore trajectory file. " << std::endl;
    }
    
    std::vector<Eigen::Vector4d> vQvecs;
    std::vector<Eigen::Vector3d> vTvecs;

    Eigen::Vector4d qvec_t;
    Eigen::Vector3d tvec_t;
    std::string str;
    char ch;
    int index = 0;
    while (!fin_pose.eof( )){
        fin_pose.get(ch);
        if( ch != ' ' && ch != '\t' && ch != '\n' ){
            str.push_back(ch);
        } else if (ch == '\n') {
            qvec_t(3) = atof(str.c_str());
            //std::cout << qvec_t(0) << " " << qvec_t(1) << " " << qvec_t(2) << " " << qvec_t(3) << " "
            //          << tvec_t(0) << " " << tvec_t(1) << " " << tvec_t(2) << "\n";
            vQvecs.push_back(qvec_t);
            vTvecs.push_back(tvec_t);
            str.clear();
            index = 0;
        } else if (ch == ' '){
            if(index == 0){
                //std::cout << " index : " << atoi(str.c_str()) << " ";
            } else if(index <= 3){
                tvec_t(index-1) = atof(str.c_str());
            } else {
                qvec_t(index-4) = atof(str.c_str());
            }
            str.clear();
            index++;
        } else {
            str.clear();
        }
    }


    std::string test_images_path = arcore_image_path;

    std::vector<Eigen::Vector3d> vColmap_Trajectory;
    std::vector<Eigen::Vector3d> vArcore_Trajectory;
    vColmap_Trajectory.reserve(num_images);
    vArcore_Trajectory.reserve(num_images);
    int total_image =  vQvecs.size()-1;
    int interval = std::max(1, total_image/num_images);
    int success_image_count = 0;
    for(int i = 1; i < total_image + 1 ; i=i+interval){
        std::string pathimg = test_images_path + std::to_string(i) + ".png";
        Eigen::Vector4d qvec;
        Eigen::Vector3d tvec;
        cv::Mat image = cv::imread(pathimg);
        if(pLocalizationLY->LocalizeImage(image, focus_length, qvec, tvec)){
            //std::cout << i << " " << tvec(0) << " " << tvec(1) << " " << tvec(2) << " "
            //          << qvec(0) << " " << qvec(1) << " " << qvec(2) << " " << qvec(3) << std::endl;
            Eigen::Quaterniond q_colmap(qvec(0), qvec(1), qvec(2), qvec(3));
            Eigen::Vector3d pose_colmap = - (q_colmap.inverse() * tvec);
            vColmap_Trajectory.push_back(pose_colmap);

            Eigen::Vector3d arcore_tvec(vTvecs[i-1](0), -vTvecs[i-1](1), vTvecs[i-1](2));
            //Eigen::Quaterniond arcore_qvec(vQvecs[i-1](0), -vQvecs[i-1](1), vQvecs[i-1](2), -vQvecs[i-1](3));
            vArcore_Trajectory.push_back(arcore_tvec);
            //pLocalizationLY->pViewerLY->SetArcorePose(arcore_qvec, arcore_tvec);
            std::cout << " --- [ " << i << "/" << num_images << " ] --- \n";
            success_image_count += 1;
        }
    }

    Eigen::Matrix3d relative_R;
    Eigen::Vector3d relative_t;
    double scale = 1.0;
    relative_R.setIdentity();

    if(success_image_count > 3){
        scale = pose_estimation_SVD(vColmap_Trajectory, vArcore_Trajectory,relative_R, relative_t);
    }

    Eigen::Quaterniond relative_q(relative_R); 

    if(success_image_count > 3){
        std::cout << " relative pose is : \n";
        std::cout << relative_t.transpose() << " " << relative_q.x() << " " 
                  << relative_q.y() << " " << relative_q.z()
                  << " " << relative_q.w() << std::endl;
        std::cout << " scale is : " << scale << std::endl;
    }

    // show projected arcore trajectory
    if(show_arcore && pLocalizationLY->bViewer){
        for(size_t i = 0; i < vQvecs.size()-1; i++){
            Eigen::Vector3d arcore_tvec(vTvecs[i](0), -vTvecs[i](1), vTvecs[i](2));
            Eigen::Quaterniond arcore_qvec(vQvecs[i](0), -vQvecs[i](1), vQvecs[i](2), -vQvecs[i-1](3));
            if(success_image_count > 3){
                Eigen::Vector3d transformed_tvec = scale * (relative_R * arcore_tvec + relative_t);
                Eigen::Quaterniond transformed_qvec = relative_q * arcore_qvec;
                pLocalizationLY->pViewerLY->SetArcorePose(transformed_qvec, transformed_tvec);   
            } else {
                pLocalizationLY->pViewerLY->SetArcorePose(arcore_qvec, arcore_tvec); 
            } 
        }  
    }

    return scale;
}
