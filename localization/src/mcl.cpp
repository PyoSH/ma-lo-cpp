#include "mcl.h"

mcl::mcl(){
    m_sync_count = 0;
    gen.seed(rd());

    gridMap_show = cv::imread("/home/kriso/map_new.png", cv::IMREAD_GRAYSCALE);
    pxCenterX = 200/2;
    pxCenterY = gridMap_show.rows - 200/2 - 200;

    gridMap_use = tool::cvMaptoMCLMap(gridMap_show.clone(), 2);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(gridMap_use, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    mapCorners = contours[0];

    cornerXmin = gridMap_show.cols/2;
    cornerXmax = gridMap_show.cols/2;
    cornerYmin = gridMap_show.rows/2;
    cornerYmin = gridMap_show.rows/2;
    
    for (int i = 0; i < gridMap_show.cols; i++) {
        for (int y = 0; y < gridMap_show.rows; y++) {
            
            uchar currPx = gridMap_show.at<uchar>(y, i);
            if (currPx == 0 || currPx >=250) {
                if (cornerXmin == -1 || i < cornerXmin) cornerXmin = i;
                if (cornerXmax == -1 || i > cornerXmax) cornerXmax = i;
                if (cornerYmin == -1 || y < cornerYmin) cornerYmin = y;
                if (cornerYmax == -1 || y > cornerYmax) cornerYmax = y;
            }
        }
    }
    std::cout<< "[px] X min " << cornerXmin << "| Y min "<< cornerYmin << "| X max "<< cornerXmax << " | Y max "<<cornerYmax << std::endl;

    double sigma_hit = 5.0;
    likelihoodField = createLikelihoodField(gridMap_show, sigma_hit);

    numOfParticle = 700; // 2500
    minOdomDistance = 0.05; //[m]
    minOdomAngle = 5; // [deg]
    repropagateCountNeeded = 20; // [num]
    odomCovariance[0] = 0.0002; // Rotation to Rotation 0.02
    odomCovariance[1] = 0.02; // translation to Rotation
    odomCovariance[2] = 0.0002; // translation to translation
    odomCovariance[3] = 0.0002; // Rotation to translation
    odomCovariance[4] = 0.002; // X
    odomCovariance[5] = 0.002; // Y

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

    canScanWrite = true;
    isPubOngoing = false;

}

mcl::~mcl(){

}

mcl::particle mcl::createRandomParticle(){
    float px_Xmin = (cornerXmax + cornerXmin)/2.0 - ((cornerXmax - cornerXmin) /2.0); // [px]
    float px_Xmax = (cornerXmax + cornerXmin)/2.0 + ((cornerXmax - cornerXmin) /2.0); // [px]
    float px_Ymin = (cornerYmax + cornerYmin)/2.0 - ((cornerYmax - cornerYmin) /2.0); // [px]
    float px_Ymax = (cornerYmax + cornerYmin)/2.0 + ((cornerYmax - cornerYmin) /2.0); // [px]
    int px_randomX, px_randomY;

    std::uniform_real_distribution<float> x_pos((px_Xmin - pxCenterX) * imageResolution + mapCenterX, 
                                    (px_Xmax - pxCenterX) * imageResolution + mapCenterX); // [m]
    std::uniform_real_distribution<float> y_pos((px_Ymin - pxCenterY) * imageResolution + mapCenterY, 
                                        (px_Ymax - pxCenterY) * imageResolution + mapCenterY); // [m]
    std::uniform_real_distribution<float> theta_pos(-M_PI, M_PI); // -180~180 [deg]

    float randomX, randomY, randomTheta;
    particle retval;
    do{
        randomX = x_pos(gen); // [m]
        randomY = y_pos(gen); // [m]
        randomTheta = theta_pos(gen); // [deg]

        px_randomX = static_cast<int>((randomX - mapCenterX) / imageResolution + pxCenterX); // [px]
        px_randomY = static_cast<int>((randomY - mapCenterY) / imageResolution + pxCenterY); // [px]
    }
    while(!(isInside_OGM(gridMap_show, (double)(px_randomX), (double)(px_randomY)) == "free")); // [px]
    
    Eigen::VectorXf initPose = tool::eigen2xyzrpy(odomBefore);
    retval.pose = tool::xyzrpy2eigen(randomX, randomY, 0, 0, 0, randomTheta); // [m]
    retval.score = 1/(double)numOfParticle;

    return retval;
}

void mcl::initializeParticles(){
    particles.clear();
    
    for(int i=0; i<numOfParticle; ++i){
        particle currParticle;
        currParticle.pose = createRandomParticle().pose;
        currParticle.score = 1/(double)numOfParticle;
        particles.emplace_back(currParticle);
    }
    
    // showInMap();
}

void mcl::prediction(Eigen::Matrix4f diffPose){
    // std::cout << "Predicting..." << m_sync_count << std::endl;
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
    std::normal_distribution<double> gaussian_distribution(0,1);
    std::normal_distribution<double> gaussian_distribution_x(0, 1); // x 방향 가우시안
    std::normal_distribution<double> gaussian_distribution_y(0, 1); // y 방향 가우시안
    std::uniform_real_distribution<float> theta_pos(-M_PI, M_PI); // -180~180 [deg]

    for(int i=0; i<particles.size(); ++i){

        delta_trans = delta_trans + gaussian_distribution(gen) * trans_noise_coeff; // 책에서는 뺀다
        delta_rot1 = delta_rot1 + gaussian_distribution(gen) * rot1_noise_coeff; // 책에서는 뺀다
        delta_rot2 = delta_rot2 + gaussian_distribution(gen) * rot2_noise_coeff; // 책에서는 뺀다

        double x = delta_trans * cos(delta_rot1) + gaussian_distribution(gen) * odomCovariance[4]; // [m]
        double y = delta_trans * sin(delta_rot1) + gaussian_distribution(gen) * odomCovariance[5]; // [m]
        double theta = delta_rot1 + delta_rot2 + (gaussian_distribution(gen) * odomCovariance[0]*(M_PI/180.0)); // rad 값으로 더하기 위함

        Eigen::Matrix4f diff_odom_w_noise = tool::xyzrpy2eigen(x, y, 0, 0, 0, theta);
        Eigen::Matrix4f pose_t_plus_1 = particles.at(i).pose * diff_odom_w_noise;
        
        // motion model with map
        int px_X = static_cast<int>((pose_t_plus_1(0,3) - mapCenterX) / imageResolution + pxCenterX); // [px]
        int px_Y = static_cast<int>((pose_t_plus_1(1,3) - mapCenterY) / imageResolution + pxCenterY); // [px]
        if(isInside_OGM(gridMap_show, px_X, px_Y) == "free"){
            particles.at(i).pose = pose_t_plus_1;
        }else{

            bool valid_position_found = false;
            int attempts = 0; // 무한 루프 방지를 위한 샘플링 시도 횟수 제한

            while(!valid_position_found && attempts < 10) { // 최대 10번 시도
                // 가우시안 분포를 사용해 새 위치를 샘플링
                double sampled_x = pose_t_plus_1(0,3) + gaussian_distribution_x(gen);
                double sampled_y = pose_t_plus_1(1,3) + gaussian_distribution_y(gen);
                double sampledTheta = theta_pos(gen); // [deg]

                // 샘플링한 위치의 맵 좌표 계산
                int new_px_X = static_cast<int>((sampled_x-mapCenterX)/imageResolution + pxCenterX);
                int new_px_Y = static_cast<int>((sampled_y-mapCenterY)/imageResolution + pxCenterY);

                // 맵 내에서 자유 공간인지 확인
                if(isInside_OGM(gridMap_show, new_px_X, new_px_Y) == "free") {
                    // 자유 공간이라면 위치 적용
                    particles.at(i).pose = tool::xyzrpy2eigen(sampled_x, sampled_y, 0, 0, 0, sampledTheta);
                    valid_position_found = true;
                }
                attempts++;
            }

            // 제한 횟수 초과 시 랜덤하게 맵 내부의 자유 공간에 파티클 생성
            if(!valid_position_found) {
                particles.at(i).pose = createRandomParticle().pose;
                particles.at(i).score = 1.0 / static_cast<double>(particles.size());
            }
        }

        scoreSum = scoreSum + particles.at(i).score;
    }
    for(int i=0; i<particles.size(); ++i){
        particles.at(i).score = particles.at(i).score/scoreSum; // normalize the score
    }
    // std::cout << "prediction - scoreSum " << scoreSum << std::endl;

    showInMap();
}

void mcl::weightning(Eigen::Matrix4Xf scan){
    double maxScore = 0.0;
    double scoreSum = 0.0;
    double minLikelihood = 1e-6;

    for(int i=0; i<particles.size(); ++i){
        double calcedWeight = calculateScanLikelihood(scan, particles.at(i).pose);

        calcedWeight = std::max(calcedWeight, minLikelihood);

        particles.at(i).score = calcedWeight;
        scoreSum += calcedWeight;

        if(maxScore < calcedWeight){
            maxProbParticle = particles.at(i);
            maxProbParticle.scan = scan;
            maxScore = calcedWeight;
        }
    }
    
    // std::cout << "weightning - scoreSum " << scoreSum << std::endl;

    for(int i=0; i<particles.size(); ++i)
        particles.at(i).score /= scoreSum;
}

void mcl::resampling(){
    // std::cout << "!!!!!Resampling!!!!!"<< m_sync_count << std::endl;

    std::vector<double> particleScores;
    std::vector<particle> particleSampled;
    double scoreBaseline = 0; 
    double min_weight = 1e-5;
    double resample_ratio = 0.1;
    double lowScoreThreshold = 0.001;

    for(int i=0; i<particles.size(); ++i){
        // scoreBaseline += particles.at(i).score;
        scoreBaseline += std::max(particles.at(i).score, min_weight);
        particleScores.emplace_back(scoreBaseline);
    }

    std::uniform_real_distribution<double> dart(0, scoreBaseline);
    std::uniform_real_distribution<double> random_dist(0, 1);

    for(int i=0; i<particles.size(); ++i){
        double darted = dart(gen); // darted number (0~ maximum Scores)
        auto lowerBound = std::lower_bound(particleScores.begin(), particleScores.end(), darted);
        int particleIdx = lowerBound - particleScores.begin(); // index of particle in particles ??

        particle selectedParticle = particles.at(particleIdx);

        particleSampled.emplace_back(selectedParticle);
        
    }
    particles = particleSampled;

    // 정규화
    for (auto& p : particles) {
        p.score = 1/numOfParticle;
    }
}

void mcl::showInMap(){
    cv::Mat showMap;
    cv::cvtColor(gridMap_show.clone(), showMap, cv::COLOR_GRAY2BGR);
    
    // draw particles in blue dots
    for(int i=0; i<numOfParticle; ++i){
        int poseX_px = static_cast<int>((particles.at(i).pose(0,3) - mapCenterX) / imageResolution + pxCenterX); // [px]
        int poseY_px = static_cast<int>((particles.at(i).pose(1,3) - mapCenterY) / imageResolution + pxCenterY); // [px]

        cv::circle(showMap, cv::Point(poseX_px, poseY_px), 1, cv::Scalar(255,0,0), -1); // 1
    }

    Eigen::VectorXf odomxyzrpy = tool::eigen2xyzrpy(odomBefore);
    int odomX_px = static_cast<int>((odomxyzrpy(0)) / imageResolution + pxCenterX); // [px]
    int odomY_px = static_cast<int>((odomxyzrpy(1)) / imageResolution + pxCenterY); // [px]
    cv::circle(showMap, cv::Point(odomX_px, odomY_px), 5, cv::Scalar(255,0,255), -1); // 1

    if(maxProbParticle.score > 0){
        float x_all = 0;
        float y_all = 0;
        
        for(int i=0; i<particles.size(); ++i){
            x_all = x_all + particles.at(i).pose(0,3) * particles.at(i).score;
            y_all = y_all + particles.at(i).pose(1,3) * particles.at(i).score;
        }
        int poseX_px_all = static_cast<int>((x_all - mapCenterX) / imageResolution + pxCenterX);
        int poseY_px_all = static_cast<int>((y_all - mapCenterY) / imageResolution + pxCenterY);

        cv::circle(showMap, cv::Point(poseX_px_all, poseY_px_all), 2, cv::Scalar(0,0,255), -1);
        Eigen::Matrix4Xf transScan = maxProbParticle.pose * tf_pc2robot * maxProbParticle.scan;

        for(int i=0; i<transScan.cols(); ++i){
            int scanX_px = static_cast<int>((transScan(0,i) - mapCenterX) / imageResolution + pxCenterX);
            int scanY_px = static_cast<int>((transScan(1,i) - mapCenterY) / imageResolution + pxCenterY);

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

    float sumTerm =0.0;
    float maxTerm = 0.0;
    for(int i = 0; i<distanceMap.rows; ++i){
        for(int j = 0; j<distanceMap.cols; ++j){
            float dist = distanceMap.at<float>(i,j);
            // likelihoodField.at<float>(i,j) =  exp(-0.5 * (dist*dist) / (sigma_hit *sigma_hit))/sqrt(2*M_PI*pow(sigma_hit,2)); 
            likelihoodField.at<float>(i,j) = exp(-0.5 * (dist*dist) / (sigma_hit *sigma_hit)); 
            if(maxTerm < likelihoodField.at<float>(i,j))    maxTerm = likelihoodField.at<float>(i,j);
            sumTerm += likelihoodField.at<float>(i,j);
        }
    }
    // std::cout << "createField sum : " << sumTerm << " | maxTerm " << maxTerm <<std::endl;

    return likelihoodField;
}

double mcl::calculateScanLikelihood(const Eigen::Matrix4Xf scan, const Eigen::Matrix4f pose){    
    Eigen::Matrix4Xf transScan = pose * tf_pc2robot * scan; // [m]

    // double q = 1.0; 
    double q_log = 0.0; 
    double z_hit = 0.8;
    double z_rand = 0.2;
    double z_max = 5;
    double random_likelihood = 1.0/z_max;
    double likelihood_min = 1e-3;

    int overlap_cnt = 0;
    int total_cnt = transScan.cols();

    for(int i=0; i<total_cnt; ++i){
        int ptX_px = static_cast<int>((transScan(0,i) - mapCenterX) / imageResolution + pxCenterX); // [px]
        int ptY_px = static_cast<int>((transScan(1,i) - mapCenterY) / imageResolution + pxCenterY); // [px]
        
        double likelihood = static_cast<double>(std::max(likelihoodField.at<float>(ptY_px, ptX_px), (float)likelihood_min) );
        q_log = q_log + log(z_hit*likelihood+z_rand*random_likelihood);

        if (likelihood > likelihood_min) overlap_cnt++;
    }
    q_log = static_cast<double>(q_log / transScan.cols()); // mean log

    float overlap_ratio = static_cast<float>(overlap_cnt) / total_cnt;

    if (overlap_ratio < 0.3) {  // 최소 겹치는 비율 설정
        return 1e-6;
    }

    return exp(q_log);
}

std::string mcl::isInside_OGM(const cv::Mat& gridMap, double x_px, double y_px){
  int currVal = gridMap.at<uchar>(y_px, x_px);
  if(currVal >= 250) return "free";
  else if(currVal == 0) return "occupied";
  else return "unknown";
}

void mcl::updateScan(Eigen::Matrix4Xf scan){
    // std::cout << "updateData - init" << std::endl;    
    auto start_time = std::chrono::high_resolution_clock::now();

    weightning(scan);
    predictionCounter++;
    if(predictionCounter == repropagateCountNeeded){
        resampling();
        predictionCounter = 0;
    }

    m_sync_count = m_sync_count +1;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    std::cout << "updateScan 실행 시간: " << duration << " ms" << std::endl;
    //  std::cout << "updateData - END" << std::endl;
}

void mcl::updatePredict(Eigen::Matrix4f pose){
    // std::cout << "updatePredict - init" << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();

    if(is1stPose){
        odomBefore = pose;
        mapCenterX = pose(0,3);
        mapCenterY = pose(1,3);
        mapCenterZ = pose(2,3);

        initializeParticles();
        is1stPose = false;
    }

    Eigen::Matrix4f diffOdom = odomBefore.inverse()*pose; // odom After = odom New * diffOdom
    prediction(diffOdom); 
    odomBefore = pose;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    std::cout << "updatePredict 실행 시간: " << duration << " ms" << std::endl;
    // std::cout << "updatePredict - END" << std::endl;
}

void mcl::publishPose(Eigen::Matrix4f newPose, double t){
    // 0. get difference between computed pose & current pose
    Eigen::Matrix4f newPose_moved = newPose;
    newPose_moved(0,3) = newPose(0,3) - mapCenterX;
    newPose_moved(1,3) = newPose(1,3) - mapCenterY;
    Eigen::Matrix4f diffOdom = odomBefore.inverse()*newPose_moved; // odom After = odom New * diffOdom

    // 1. predict with new pose 
    Eigen::Vector3f weightedTranslation = Eigen::Vector3f::Zero();
    Eigen::Quaternionf weightedRotation(0,0,0,0);
    
    float x_all(0.0), y_all(0.0), theta_all(0.0);
    for(auto& p : particles){
        weightedTranslation += p.pose.block<3,1>(0,3) * p.score;

        Eigen::Quaternionf rotation(p.pose.block<3,3>(0,0));
        weightedRotation.coeffs() += rotation.coeffs() * p.score;
    }
    weightedRotation.normalize();

    double variance = calculateVariance(particles, x_all, y_all);
    double varianceThreshold = 0.5;
    Eigen::Matrix4f retPose = maxProbParticle.pose;
    if(variance < varianceThreshold){ // converged, use maxProbParticle pose
        std::cout << "PUB pose - converged | variance: " << variance <<std::endl;

    }else if(varianceThreshold < variance && variance < 200){ // diverged, use weightedSum pose
        std::cout << "PUB pose - diverged | variance: " << variance <<std::endl;
        retPose.block<3,1>(0,3) = weightedTranslation;
        retPose.block<3,3>(0,0) = weightedRotation.toRotationMatrix();
    }else{
        std::cout << "PUB pose - no dap | variance: " << variance <<std::endl;
        retPose = newPose;
    }

    // retPose = retPose * diffOdom;
    Eigen::VectorXf weightedMeanPose = tool::eigen2xyzrpy(retPose);
    Eigen::VectorXf retVal = weightedMeanPose;
    
    // 2. publish msg
    nav_msgs::Odometry mcl_msg;
    mcl_msg.header.stamp = ros::Time(t);
    mcl_msg.header.frame_id = "odom";
    mcl_msg.child_frame_id = "base_link";   

    Eigen::Matrix4f eigenRetVal = tool::xyzrpy2eigen(retVal(0), retVal(1), retVal(2), retVal(3), retVal(4), retVal(5));
    mcl_msg.pose.pose.position.x = retVal(0);
    mcl_msg.pose.pose.position.y = retVal(1);
    mcl_msg.pose.pose.position.z = retVal(2); // assume 0

    tf::Matrix3x3 rotation_matrix(eigenRetVal(0,0), eigenRetVal(0,1), eigenRetVal(0,2),
                                  eigenRetVal(1,0), eigenRetVal(1,1), eigenRetVal(1,2),
                                  eigenRetVal(2,0), eigenRetVal(2,1), eigenRetVal(2,2)); 
    tf::Quaternion q;
    rotation_matrix.getRotation(q);
    mcl_msg.pose.pose.orientation.x =  q.x();
    mcl_msg.pose.pose.orientation.y =  q.y();
    mcl_msg.pose.pose.orientation.z =  q.z();
    mcl_msg.pose.pose.orientation.w =  q.w();

    static ros::Publisher pub_MCL = ros::NodeHandle().advertise<nav_msgs::Odometry>("/mcl2", 1);
    pub_MCL.publish(mcl_msg);
}

double mcl::calculateVariance(const std::vector<mcl::particle>& particles, double mean_x, double mean_y){
    double variance = 0.0;
    double total_weight = 0.0;

    for (const auto& particle:particles){
        double dx = particle.pose(0,3) - mean_x;
        double dy = particle.pose(1,3) - mean_y;
        variance += particle.score * (dx*dx + dy*dy);
        total_weight += particle.score;
    }

    return variance/total_weight;
}
