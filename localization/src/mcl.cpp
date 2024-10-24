#include "mcl.h"

mcl::mcl(){
    m_sync_count = 0;
    gen.seed(rd());

    gridMap_show = cv::imread("/home/pyo/map_door.png", cv::IMREAD_GRAYSCALE);

    gridMap_use = tool::cvMaptoMCLMap(gridMap_show.clone(), 2);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(gridMap_use, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    epsilon = 10.0;  // 허용 오차 값 (이미지 크기에 따라 조정 필요)
    std::vector<std::vector<cv::Point>> simplifiedContours;

    for (const auto& contour : contours) {
        std::vector<cv::Point> simplified = tool::getPolygonMap(contour, epsilon);
        simplifiedContours.push_back(simplified);
    }

    mapCorners = simplifiedContours[0];

    cornerXmin = mapCorners[0].x;
    cornerXmax = mapCorners[0].x;
    cornerYmin = mapCorners[0].y;
    cornerYmin = mapCorners[0].y;
    for(const auto& corner : mapCorners) {
        if (corner.x < cornerXmin) {
            cornerXmin = corner.x;
        }if (corner.x > cornerXmax) {
            cornerXmax = corner.x;
        }if (corner.y < cornerYmin) {
            cornerYmin = corner.y;
        }if (corner.y > cornerYmax) {
            cornerYmax = corner.y;
        }
    }
    std::cout<< "[px] X min " << cornerXmin << "| Y min "<< cornerYmin << "| X max "<< cornerXmax << " | Y max "<<cornerYmax << std::endl;

    double sigma_hit = 1.0;
    likelihoodField = createLikelihoodField(gridMap_show, sigma_hit);

    numOfParticle = 2000; // 2500
    minOdomDistance = 0.01; //[m]
    minOdomAngle = 5; // [deg]
    repropagateCountNeeded = 5; // [num]
    odomCovariance[0] = 0.02; // Rotation to Rotation
    odomCovariance[1] = 0.02; // translation to Rotation
    odomCovariance[2] = 0.02; // translation to translation
    odomCovariance[3] = 0.02; // Rotation to translation
    odomCovariance[4] = 0.02; // X
    odomCovariance[5] = 0.02; // Y

    imageResolution = 0.05; // [m] per [pixel]
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
    
    // initializeParticles();
    // showInMap();
}

mcl::~mcl(){

}

mcl::particle mcl::createRandomParticle(){
    float px_Xmin = (cornerXmax + cornerXmin)/2.0 - ((cornerXmax - cornerXmin) /2.0); // [px]
    float px_Xmax = (cornerXmax + cornerXmin)/2.0 + ((cornerXmax - cornerXmin) /2.0); // [px]
    float px_Ymin = (cornerYmax + cornerYmin)/2.0 - ((cornerYmax - cornerYmin) /2.0); // [px]
    float px_Ymax = (cornerYmax + cornerYmin)/2.0 + ((cornerYmax - cornerYmin) /2.0); // [px]
    int px_randomX, px_randomY;

    std::uniform_real_distribution<float> x_pos((px_Xmin - (gridMap_use.cols/2.0)) * imageResolution + mapCenterX, 
                                        (px_Xmax - (gridMap_use.cols/2.0)) * imageResolution + mapCenterX); // [m]
    std::uniform_real_distribution<float> y_pos((px_Ymin - (gridMap_use.rows/2.0)) * imageResolution + mapCenterY, 
                                        (px_Ymax - (gridMap_use.rows/2.0)) * imageResolution + mapCenterY); // [m]
    std::uniform_real_distribution<float> theta_pos(-M_PI, M_PI); // -180~180 [deg]

    float randomX, randomY, randomTheta;
    particle retval;
    do{
        randomX = x_pos(gen); // [m]
        randomY = y_pos(gen); // [m]
        randomTheta = theta_pos(gen); // [deg]

        px_randomX = static_cast<int>((randomX - mapCenterX) / imageResolution + (gridMap_use.cols / 2.0)); // [px]
        px_randomY = static_cast<int>((randomY - mapCenterY) / imageResolution + (gridMap_use.rows / 2.0)); // [px]
    }
    while(!tool::isInside(mapCorners, (double)(px_randomX), (double)(px_randomY),
                        (double)px_Xmin, (double)px_Ymin, (double)px_Xmax, (double)px_Ymax)); // [px]
    std::cout << "createRandom- " << px_randomX << " | "<< px_randomY << std::endl;
    
    Eigen::VectorXf initPose = tool::eigen2xyzrpy(odomBefore);
    retval.pose = tool::xyzrpy2eigen(randomX, randomY, 0, 0, 0, randomTheta); // [m]
    // retval.pose = tool::xyzrpy2eigen(randomX, randomY, 0, 0, 0, initPose(5)); // [m]
    retval.score = 1/(double)numOfParticle;

    return retval;
}

void mcl::initializeParticles(){
    particles.clear();
    /*
    * !!! 맵의 가로 크기를 4로 나눔 -> 입자 초기화 범위를 전체 맵의 중앙에 분포하도록. <- 계산 트릭인듯
    * 맵 전체에 분포되도록 하려면 2로 나눌 것!!!
    */
    
    for(int i=0; i<numOfParticle; ++i){
        Eigen::VectorXf initPose = tool::eigen2xyzrpy(odomBefore);
        particle currParticle;
        
        currParticle = createRandomParticle();
        // currParticle.pose = tool::xyzrpy2eigen(initPose(0), initPose(1), 0,0,0, initPose(5));
        currParticle.score = 1/(double)numOfParticle;
        particles.emplace_back(currParticle);

        std::cout << "init- " << (int)currParticle.pose(0,3)/imageResolution << " | "<< (int)currParticle.pose(1,3)/imageResolution << std::endl;
    }
    
    showInMap();
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
    double trans_noise_coeff = odomCovariance[2]*fabs(delta_trans) + odomCovariance[3]*fabs(delta_rot1+delta_rot2);
    double rot1_noise_coeff = odomCovariance[0]*fabs(delta_rot1) + odomCovariance[1]*fabs(delta_trans);
    double rot2_noise_coeff = odomCovariance[0]*fabs(delta_rot2) + odomCovariance[1]*fabs(delta_trans);

    float scoreSum = 0;
    for(int i=0; i<particles.size(); ++i){
        std::normal_distribution<double> gaussian_distribution(0,1);

        delta_trans = delta_trans + gaussian_distribution(gen) * trans_noise_coeff; // 책에서는 뺀다
        delta_rot1 = delta_rot1 + gaussian_distribution(gen) * rot1_noise_coeff; // 책에서는 뺀다
        delta_rot2 = delta_rot2 + gaussian_distribution(gen) * rot2_noise_coeff; // 책에서는 뺀다

        double x = delta_trans * cos(delta_rot1) + gaussian_distribution(gen) * odomCovariance[4]; // [m]
        double y = delta_trans * sin(delta_rot1) + gaussian_distribution(gen) * odomCovariance[5]; // [m]
        double theta = delta_rot1 + delta_rot2 + (gaussian_distribution(gen) * odomCovariance[0]*(M_PI/180.0)); // rad 값으로 더하기 위함

        Eigen::Matrix4f diff_odom_w_noise = tool::xyzrpy2eigen(x, y, 0, 0, 0, theta);
        Eigen::Matrix4f pose_t_plus_1 = particles.at(i).pose * diff_odom_w_noise;
        
        // motion model with map
        int px_X = static_cast<int>((pose_t_plus_1(0,3) - mapCenterX) / imageResolution + (gridMap_use.cols / 2.0)); // [px]
        int px_Y = static_cast<int>((pose_t_plus_1(1,3) - mapCenterY) / imageResolution + (gridMap_use.rows / 2.0)); // [px]
        if(tool::isInside(mapCorners, px_X, px_Y, cornerXmin, cornerYmin, cornerXmax, cornerYmax)){
            particles.at(i).pose = pose_t_plus_1;
        }else{
            particles.at(i).pose = createRandomParticle().pose; // 혹은 근처이면서 맵 내부인 위치 만들기
            particles.at(i).score = 1/(double)particles.size(); // 혹은 근처이면서 맵 내부인 위치 만들기
        }

        scoreSum = scoreSum + particles.at(i).score;
    }
    for(int i=0; i<particles.size(); ++i){
        particles.at(i).score = particles.at(i).score/scoreSum; // normalize the score
    }

    showInMap();
}

void mcl::weightning(Eigen::Matrix4Xf scan){
    double maxScore = 0;
    double scoreSum = 0;

    for(int i=0; i<particles.size(); ++i){
        double calcedWeight = calculateScanLikelihood(scan, particles.at(i).pose);
        particles.at(i).score = calcedWeight;
        scoreSum += calcedWeight;

        if(maxScore < calcedWeight){
            maxProbParticle = particles.at(i);
            maxProbParticle.scan = scan;
            maxScore = calcedWeight;
        }
    }

    // 가중치의 합이 너무 작을 때 정규화
    if(scoreSum > 0){
        for(int i=0; i<particles.size(); ++i)
            particles.at(i).score = particles.at(i).score/scoreSum; //normalize the score
    }else{
        for(int i=0; i<particles.size(); ++i)
            particles.at(i).score = 1.0 / particles.size(); //normalize the score
    }
    
}

void mcl::resampling(){
    // std::cout << "Resampling..."<< m_sync_count << std::endl;

    std::vector<double> particleScores;
    std::vector<particle> particleSampled;
    double scoreBaseline = 0; 
    double min_weight = 1e-5;
    for(int i=0; i<particles.size(); ++i){
        // scoreBaseline += particles.at(i).score;
        scoreBaseline += std::max(particles.at(i).score, min_weight);
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
    // cv::Mat showMap = likelihoodField.clone();
    // cv::cvtColor(showMap, showMap, cv::COLOR_GRAY2BGR);

    // draw particles in blue dots
    for(int i=0; i<numOfParticle; ++i){
        int poseX_px = static_cast<int>((particles.at(i).pose(0,3) - mapCenterX) / imageResolution + (showMap.cols / 2.0)); // [px]
        int poseY_px = static_cast<int>((particles.at(i).pose(1,3) - mapCenterY) / imageResolution + (showMap.rows / 2.0)); // [px]

        // std::cout << "draw- " << poseX_px << " | "<< poseY_px << std::endl;
        cv::circle(showMap, cv::Point(poseX_px, poseY_px), 1, cv::Scalar(255,0,0), -1); // 1
    }

    if(maxProbParticle.score > 0){
        float x_all = 0;
        float y_all = 0;
        
        for(int i=0; i<particles.size(); ++i){
            x_all = x_all + particles.at(i).pose(0,3) * particles.at(i).score;
            y_all = y_all + particles.at(i).pose(1,3) * particles.at(i).score;
        }
        int poseX_px_all = static_cast<int>((x_all - mapCenterX) / imageResolution + (showMap.cols / 2.0));
        int poseY_px_all = static_cast<int>((y_all - mapCenterY) / imageResolution + (showMap.rows / 2.0));

        cv::circle(showMap, cv::Point(poseX_px_all, poseY_px_all), 2, cv::Scalar(0,0,255), -1);
        Eigen::Matrix4Xf transScan = maxProbParticle.pose * tf_pc2robot * maxProbParticle.scan;

        for(int i=0; i<transScan.cols(); ++i){
            int scanX_px = static_cast<int>((transScan(0,i) - mapCenterX) / imageResolution + (showMap.cols / 2.0));
            int scanY_px = static_cast<int>((transScan(1,i) - mapCenterY) / imageResolution + (showMap.rows / 2.0));

            cv::circle(showMap, cv::Point(scanX_px, scanY_px), 1, cv::Scalar(0,255,255), -1);
        }
    }
    
    cv::imshow("MCL2", showMap);
    cv::waitKey(1);
}

cv::Mat mcl::createLikelihoodField(const cv::Mat& obstacleMap, double sigma_hit){
    cv::Mat distanceMap;
    cv::distanceTransform(obstacleMap, distanceMap, cv::DIST_L2, 5);

    cv::Mat likelihoodField(distanceMap.size(), CV_32F);
    for(int i = 0; i<distanceMap.rows; ++i){
        for(int j = 0; j<distanceMap.cols; ++j){
            float dist = distanceMap.at<float>(i,j);
            likelihoodField.at<float>(i,j) = exp(-0.5 * (dist*dist) / (sigma_hit *sigma_hit));
        }
    }
    return likelihoodField;
}

double mcl::calculateScanLikelihood(const Eigen::Matrix4Xf scan, const Eigen::Matrix4f pose){
    Eigen::Matrix4Xf transScan = pose * tf_pc2robot * scan; // [m]

    // float q = 1.0; 
    float q_log = 0.0; 

    for(int i=0; i<transScan.cols(); ++i){
        int ptX_px = static_cast<int>((transScan(0,i) - mapCenterX) / imageResolution + (likelihoodField.cols / 2.0)); // [px]
        int ptY_px = static_cast<int>((transScan(1,i) - mapCenterY) / imageResolution + (likelihoodField.rows / 2.0)); // [px]
        
        bool isRoughlyInside = (cornerXmin < ptX_px && ptX_px < cornerXmax) && (cornerYmin < ptY_px && ptY_px < cornerYmax);
        if (isRoughlyInside){
        // if (tool::isInside(mapCorners, ptX_px, ptY_px, cornerXmin, cornerYmin, cornerXmax, cornerYmax)){
            float likelihood = likelihoodField.at<float>(ptY_px, ptX_px);
            // q = q * likelihood;
            q_log = q_log + log(likelihood);
        }else{
            // q = q * 0.01;
            q_log = q_log + log(0.01);
        }
    }

    // return q;
    return exp(q_log);
}

void mcl::updateData(Eigen::Matrix4f pose, Eigen::Matrix4Xf scan, double t1, double t2){
    if(is1stPose){
        odomBefore = pose;
        mapCenterX = pose(0,3);
        mapCenterY = pose(1,3);
        mapCenterZ = pose(2,3);

        initializeParticles();
        is1stPose = false;
    }
    // std::cout <<"UPDATE-DATA Pose_t: \n"<< pose << std::endl; // ?!?

    Eigen::Matrix4f diffOdom = odomBefore.inverse()*pose; // odom After = odom New * diffOdom
    Eigen::VectorXf diffxyzrpy = tool::eigen2xyzrpy(diffOdom);
    
    float diffDistance = sqrt(pow(diffxyzrpy(0), 2) + pow(diffxyzrpy(1), 2)); //[m]
    float diffAngle = fabs(diffxyzrpy(5) * 180.0/3.141592); // [deg]
    
    if(diffDistance > minOdomDistance || diffAngle > minOdomAngle){
        prediction(diffOdom); 
        weightning(scan);

        predictionCounter++;
        if(predictionCounter == repropagateCountNeeded){
            resampling();
            predictionCounter = 0;
        }

        m_sync_count = m_sync_count +1;
        odomBefore = pose;
    }
}