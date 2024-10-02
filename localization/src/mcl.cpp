#include "mcl.h"

mcl::mcl(){
    m_sync_count = 0;
    gen.seed(rd());

    gridMap_show = cv::imread("/home/pyo/erodedMap.png", cv::IMREAD_GRAYSCALE);
    gridMap_use = cv::imread("/home/pyo/erodedMap.png", cv::IMREAD_GRAYSCALE);

    numOfParticle = 1; // 2500
    minOdomDistance = 0.1; //[m]
    minOdomAngle = 30; // [deg]
    repropagateCountNeeded = 1; // [num]
    odomCovariance[0] = 0.02; // Rotation to Rotation
    odomCovariance[1] = 0.02; // translation to Rotation
    odomCovariance[2] = 0.02; // translation to translation
    odomCovariance[3] = 0.02; // Rotation to translation
    odomCovariance[4] = 0.02; // X
    odomCovariance[5] = 0.02; // Y

    imageResolution = 0.1; // [m] per [pixel]
    roll_rad = -90* M_PI / 180.0;
    pitch_rad = 0;
    yaw_rad = -90* M_PI / 180.0;
    rotation = tool::get_rotation_matrix(roll_rad, pitch_rad, yaw_rad);
    tf_pc2robot.block<3,3>(0,0) << rotation;
    tf_pc2robot(0,3) = 0;
    tf_pc2robot(1,3) = 0;
    tf_pc2robot(2,3) = 0;
    tf_pc2robot(3,3) = 1;

    mapCenterX = 0; //[m]
    mapCenterY = 0; //[m]
    is1stPose = true;
    predictionCounter = 0;
    
    initializeParticles();
    showInMap();
}

mcl::~mcl(){

}

void mcl::initializeParticles(){
    particles.clear();
    /*
    * !!! 맵의 가로 크기를 4로 나눔 -> 입자 초기화 범위를 전체 맵의 중앙에 분포하도록. <- 계산 트릭인듯
    * 맵 전체에 분포되도록 하려면 2로 나눌 것!!!
    */
    // std::uniform_real_distribution<float> x_pos(mapCenterX - gridMap_use.cols * imageResolution / 8.0,
    //             mapCenterX + gridMap_use.cols * imageResolution / 8.0);
    // std::uniform_real_distribution<float> y_pos(mapCenterY - gridMap_use.rows * imageResolution / 8.0,
    //             mapCenterY + gridMap_use.rows * imageResolution / 8.0);
    // std::uniform_real_distribution<float> theta_pos(-M_PI, M_PI); // -180~180 [deg]
    for(int i=0; i<numOfParticle; ++i){
        particle currParticle;
        // float randomX = x_pos(gen);
        // float randomY = y_pos(gen);
        // float randomTheta = theta_pos(gen);
        // currParticle.pose = tool::xyzrpy2eigen(randomX, randomY, 0,0,0, randomTheta);
        Eigen::VectorXf initPose = tool::eigen2xyzrpy(odomBefore);
        currParticle.pose = tool::xyzrpy2eigen(initPose(0), initPose(1), 0,0,0, initPose(5)); // !!!
        currParticle.score = 1/(double)numOfParticle;
        particles.emplace_back(currParticle);
    }
    // showInMap();
}

void mcl::prediction(Eigen::Matrix4f diffPose){
    std::cout << "Predicting..." << m_sync_count << std::endl;
    Eigen::VectorXf diff_xyzrpy = tool::eigen2xyzrpy(diffPose); //{x,y,z,R,P,Y} (z, R, P -> assume to 0)
    

    // Using Odometry model to motion model
    double delta_trans = sqrt(pow(diff_xyzrpy(0), 2) + pow(diff_xyzrpy(1), 2));
    double delta_rot1 = atan2(diff_xyzrpy(1), diff_xyzrpy(0)); // [rad] 
    double delta_rot2 = diff_xyzrpy(5) - delta_rot1; // [rad] 목표 위치로 이동 후 추가로 회전한 각도
    

    std::default_random_engine generator;
    if(delta_rot1  > M_PI)
            delta_rot1 -= (2*M_PI);
    if(delta_rot1  < -M_PI)
            delta_rot1 += (2*M_PI);
    if(delta_rot2  > M_PI)
            delta_rot2 -= (2*M_PI);
    if(delta_rot2  < -M_PI)
            delta_rot2 += (2*M_PI);
    //// Add noises to trans/rot1/rot2
    // double trans_noise_coeff = odomCovariance[2]*fabs(delta_trans) + odomCovariance[3]*fabs(delta_rot1+delta_rot2);
    // double rot1_noise_coeff = odomCovariance[0]*fabs(delta_rot1) + odomCovariance[1]*fabs(delta_trans);
    // double rot2_noise_coeff = odomCovariance[0]*fabs(delta_rot2) + odomCovariance[1]*fabs(delta_trans);

    float scoreSum = 0;
    for(int i=0; i<particles.size(); ++i){
        // std::normal_distribution<double> gaussian_distribution(0,1);

        // delta_trans = delta_trans + gaussian_distribution(gen) * trans_noise_coeff;
        // delta_rot1 = delta_rot1 + gaussian_distribution(gen) * rot1_noise_coeff;
        // delta_rot2 = delta_rot2 + gaussian_distribution(gen) * rot2_noise_coeff;

        // double x = delta_trans * cos(delta_rot1) + gaussian_distribution(gen) * odomCovariance[4]; // 책에서는 뺀다
        // double y = delta_trans * sin(delta_rot1) + gaussian_distribution(gen) * odomCovariance[5]; // 책에서는 뺀다
        // double theta = delta_rot1 + delta_rot2 + (gaussian_distribution(gen) * odomCovariance[0]*(M_PI/180.0)); // rad 값으로 더하기 위함, 책에서는 뺀다
        double x = delta_trans * cos(delta_rot1); // !!!
        double y = delta_trans * sin(delta_rot1); // delta_rot1으로 해야 해.
        double theta = delta_rot1 + delta_rot2; // !!!

        Eigen::Matrix4f diff_odom_w_noise = tool::xyzrpy2eigen(x, y, 0, 0, 0, theta);
        Eigen::Matrix4f pose_t_plus_1 = particles.at(i).pose * diff_odom_w_noise;

        scoreSum = scoreSum + particles.at(i).score;
        particles.at(i).pose = pose_t_plus_1;
    }

    for(int i=0; i<particles.size(); ++i){
        particles.at(i).score = particles.at(i).score/scoreSum; // normalize the score
    }

    showInMap();
}

void mcl::weightning(Eigen::Matrix4Xf scan){
    float maxScore = 0;
    float scoreSum = 0;

    for(int i=0; i<particles.size(); ++i){
        Eigen::Matrix4Xf transScan = particles.at(i).pose * tf_pc2robot * scan;
        float calcedWeight = 0;

        for(int j=0; j<transScan.cols(); ++j){
            int ptX_px = static_cast<int>((transScan(0,j) - mapCenterX) / imageResolution + (gridMap_use.cols / 2.0));
            int ptY_px = static_cast<int>((transScan(1,j) - mapCenterY) / imageResolution + (gridMap_use.rows / 2.0));
            
            if(gridMap_use.cols <= ptX_px || ptX_px < 0 || gridMap_use.rows <= ptY_px || ptY_px < 0) continue;
            else{
                double img_val = gridMap_use.at<uchar>(ptX_px, ptY_px)/(double)255; // calc the score
                calcedWeight += img_val; // sum up the score
            }
        }
        particles.at(i).score = particles.at(i).score + (calcedWeight / transScan.cols()); // adding score to particles
        scoreSum += particles.at(i).score;
    
        if(maxScore < particles.at(i).score){
            maxProbParticle = particles.at(i);
            maxProbParticle.scan = scan;
            maxScore = particles.at(i).score;
        }
    }

    for(int i=0; i<particles.size(); ++i)
        particles.at(i).score = particles.at(i).score/scoreSum; //normalize the score
}

void mcl::resampling(){
    std::cout << "Resampling..."<< m_sync_count << std::endl;

    std::vector<double> particleScores;
    std::vector<particle> particleSampled;
    double scoreBaseline = 0; 
    
    for(int i=0; i<particles.size(); ++i){
        scoreBaseline += particles.at(i).score;
        particleScores.emplace_back(scoreBaseline);
    }

    std::uniform_real_distribution<double> dart(0, scoreBaseline);

    for(int i=0; i<particles.size(); ++i){
        double darted = dart(gen); // darted number (0~ maximum Scores)
        auto lowerBound = std::lower_bound(particleScores.begin(), particleScores.end(), darted);
        int particleIdx = lowerBound - particleScores.begin(); // index of particle in particles ??

        particle selectedParticle = particles.at(particleIdx);
        particleSampled.emplace_back(selectedParticle);
    }
    particles = particleSampled;
}

void mcl::showInMap(){
    cv::Mat showMap;
    cv::cvtColor(gridMap_show, showMap, cv::COLOR_GRAY2BGR);

    // draw particles in blue dots
    for(int i=0; i<numOfParticle; ++i){
        int poseX_px = static_cast<int>((particles.at(i).pose(0,3) - mapCenterX) / imageResolution + (gridMap_use.cols / 2.0));
        int poseY_px = static_cast<int>((particles.at(i).pose(1,3) - mapCenterY) / imageResolution + (gridMap_use.rows / 2.0));
        std::cout << poseX_px << " | " << poseY_px << std::endl;
        cv::circle(showMap, cv::Point(poseX_px, poseY_px), 5, cv::Scalar(255,0,0), -1); // 1
    }

    // if(maxProbParticle.score > 0){
    // // if(true){
    //     float x_all = 0;
    //     float y_all = 0;
        
    //     for(int i=0; i<particles.size(); ++i){
    //         // x_all = x_all + particles.at(i).pose(0,3) * particles.at(i).score;
    //         // y_all = y_all + particles.at(i).pose(1,3) * particles.at(i).score;
    //         x_all = x_all + particles.at(i).pose(0,3);
    //         y_all = y_all + particles.at(i).pose(1,3);
    //     }
    //     int poseX_px_all = static_cast<int>((x_all - mapCenterX) / imageResolution + (gridMap_use.cols / 2.0));
    //     int poseY_px_all = static_cast<int>((y_all - mapCenterY) / imageResolution + (gridMap_use.rows / 2.0));

    //     cv::circle(showMap, cv::Point(poseX_px_all, poseY_px_all), 2, cv::Scalar(0,0,255), -1);
    //     Eigen::Matrix4Xf transScan = maxProbParticle.pose * tf_pc2robot * maxProbParticle.scan;

    //     for(int i=0; i<transScan.cols(); ++i){
    //         int scanX_px = static_cast<int>((transScan(0,i) - mapCenterX) / imageResolution + (gridMap_use.cols / 2.0));
    //         int scanY_px = static_cast<int>((transScan(1,i) - mapCenterY) / imageResolution + (gridMap_use.rows / 2.0));

    //         cv::circle(showMap, cv::Point(scanX_px, scanY_px), 1, cv::Scalar(0,255,255), -1);
    //     }
    // }
    
    cv::imshow("MCL2", showMap);
    cv::waitKey(1);
}

void mcl::updateData(Eigen::Matrix4f pose, Eigen::Matrix4Xf scan, double t1, double t2){
    if(is1stPose){
        odomBefore = pose;
        // mapCenterX = pose(0,3);
        // mapCenterY = pose(1,3);
        is1stPose = false;
        // mapCenterZ = pose(2,3);
    }

    Eigen::Matrix4f diffOdom = odomBefore.inverse()*pose; // odom After = odom New * diffOdom
    Eigen::VectorXf diffxyzrpy = tool::eigen2xyzrpy(diffOdom);
    float diffDistance = sqrt(pow(diffxyzrpy(0), 2) + pow(diffxyzrpy(1), 2)); //[m]
    float diffAngle = fabs(diffxyzrpy(5) * 180.0/3.141592); // [deg]
    // std:: cout << "diffDistance: " << diffDistance << " | diffAngle: " << diffAngle << std::endl;
    prediction(diffOdom); // !!!!
    odomBefore = pose; // !!!!
    // if(diffDistance > minOdomDistance || diffAngle > minOdomAngle){
    //     // prediction(diffOdom); // !!!! 밖으로 꺼내 
    //     // weightning(scan);

    //     predictionCounter++;
    //     if(predictionCounter == repropagateCountNeeded){
    //         // resampling();
    //         predictionCounter = 0;
    //     }

    //     m_sync_count = m_sync_count +1;
    //     odomBefore = pose;
    // }
}