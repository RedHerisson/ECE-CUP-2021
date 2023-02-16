#include "Robot.hpp"


float getOrientation(Point p1, Point p2) {
    float angle;
    double Hyp = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    double Opp = (p1.x - p2.x);
    double sin = Opp / Hyp;

    angle  = - asin(sin);

    angle = (angle * (180 / 3.141592));
    if (p1.x <= p2.x && p1.y < p2.y) angle = 180 - angle;
    if(p1.x >= p2.x && p1.y > p2.y) angle = -180 - angle;

    if (angle < 0) angle = - angle + 180;

    return angle;
}

int mapCommand(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Robot::Robot()
    : m_timerFPS(0), m_findTarget(0), m_targetType(0), m_lastErrorTimer(0)
{
    AStar::Generator generator;
    generator.setWorldSize({ 640 / (SIZE_GRID_ASTAR*RESCALE_FACTOR), 480 / (SIZE_GRID_ASTAR * RESCALE_FACTOR) });
    generator.setHeuristic(AStar::Heuristic::manhattan);
    generator.setDiagonalMovement(true);
    m_generator = generator;
   
}


int Robot::initArUco(int argc, char* argv[])
{
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    m_showRejected = parser.has("r");
    m_estimatePose = 0;
    m_markerLength = parser.get<float>("l");
  
    m_detectorParams = aruco::DetectorParameters::create();
    if (parser.has("dp")) {
        FileStorage fs(parser.get<string>("dp"), FileStorage::READ);
        bool readOk =  m_detectorParams->readDetectorParameters(fs.root());
        if (!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }

    if (parser.has("refine")) {
        //override cornerRefinementMethod read from config file
         m_detectorParams->cornerRefinementMethod = parser.get<int>("refine");
    }
    std::cout << "Corner refinement method (0: None, 1: Subpixel, 2:contour, 3: AprilTag 2): " <<  m_detectorParams->cornerRefinementMethod << std::endl;

    int camId = parser.get<int>("ci");

    String video;
    if (parser.has("v")) {
        video = parser.get<String>("v");
    }

    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }

    m_dictionary = aruco::getPredefinedDictionary(0);
    if (parser.has("d")) {
        int dictionaryId = parser.get<int>("d");
        m_dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    }
    else if (parser.has("cd")) {
        FileStorage fs(parser.get<std::string>("cd"), FileStorage::READ);
        bool readOk = m_dictionary->aruco::Dictionary::readDictionary(fs.root());
        if (!readOk) {
            std::cerr << "Invalid dictionary file" << std::endl;
            return 0;
        }
    }
    else {
        std::cerr << "Dictionary not specified" << std::endl;
        return 0;
    }


    return 1;
}


Robot::~Robot()
{
}

bool Robot::updatePosition(Mat* image, Mat* imageCopy)
{
   

    vector< int > ids;
    vector< vector< Point2f > > corners, rejected;
    vector< Vec3d > rvecs, tvecs;
    bool findTag;

    // detect markers and estimate pose
    aruco::detectMarkers(*image,m_dictionary, corners, ids,  m_detectorParams, rejected);
    if (m_estimatePose && ids.size() > 0)
        aruco::estimatePoseSingleMarkers(corners, 10, m_camMatrix, m_distCoeffs, rvecs, tvecs);
   

    // draw results
    image->copyTo(*imageCopy);
    if (ids.size() > 0 && ids[0] == 19) {
        aruco::drawDetectedMarkers(*imageCopy, corners, ids);




        if (m_estimatePose) {
            for (unsigned int i = 0; i < ids.size(); i++) {
                cv::drawFrameAxes(*imageCopy, m_camMatrix, m_distCoeffs, rvecs[i], tvecs[i], m_markerLength * 1.5f, 2);

            }
        }
    }

    if (m_showRejected && rejected.size() > 0)
        aruco::drawDetectedMarkers(*imageCopy, rejected, noArray(), Scalar(100, 0, 255));


    int width = imageCopy->size().width;
    int height = imageCopy->size().height;

    Mat Rmatrix(3, 3, CV_64FC1);
    if (ids.size() > 0 && ids[0] == 19) {

        line(*imageCopy, Point(corners[0][3].x, corners[0][0].y), corners[0][0], cv::Scalar(255, 0, 0));
        line(*imageCopy, Point(corners[0][3].x, corners[0][3].y), corners[0][0], cv::Scalar(0, 0, 255));
        line(*imageCopy, Point(corners[0][3].x, corners[0][0].y), corners[0][3], cv::Scalar(0, 0, 255));

        // récupération de la position du robot
        m_Position.x = (int)((corners[0][0].x + corners[0][2].x) / 2);
        m_Position.y = (int)((corners[0][0].y + corners[0][2].y) / 2);
        m_RobotAngle = getOrientation(corners[0][3], corners[0][0]);
        
        ///gestion de la hitBox
        Point pShiftPince; 


        pShiftPince.x = corners[0][0].x + 1.5 * (corners[0][0].x - corners[0][3].x);
        pShiftPince.y = corners[0][0].y + 1.5 * (corners[0][0].y - corners[0][3].y);
        Rect tempoRect(pShiftPince, corners[0][2]);
        m_hitBox = tempoRect;




       //m_hitBox = tempoHitBox;
        rectangle(*imageCopy, m_hitBox, Scalar(255, 255, 0));


        if(SHOW_ROBOT_ANGLE) cout <<"ANGLE ROBOT:" << m_RobotAngle << endl;
        if(SHOW_ROBOT_POS) cout << "POS ROBOT:" << m_Position << endl;


        findTag = 1;

    }
    else findTag = 0;

    return findTag;

}

void Robot::setCubeList()
{
    vector<vector<int> > color(COLOR_NBR);

    ///               low_H | high_H | low_S | high_S | low_V | high_H   
    color[VERT] = { 39, 63, 128, 173, 113, 192 }; /// Vert
    color[ROUGE] = { 125,   179,    203,    238,     66,    121 }; /// Rouge
    color[BLEU] = { 118,   118,    130,    132,     99,    99, }; /// Bleu
    color[NOIR] = { 155,   179,    26,     56,      21,    59 };  /// Noir 
    //{ 149, 179, 88, 255, 168, 255 };//
    /* TABLE THRESHOLD
    color[VERT] = { 23,     89,     120,    255,    111,    166 }; /// Vert
    color[ROUGE] = { 166,    179,    149,    179,    163,    212, }; /// Rouge
    color[BLEU] = { 96,     113,    137,    215,    68,     119, }; /// Bleu
    color[NOIR] = { 89,     108,    31,     95,     36,     109, };  /// Noir
    */


    for (int i = 0; i < COLOR_NBR; i++) {
        cubeType cube;
        m_cubeTypeList.push_back(cube);
        m_cubeTypeList[i].colorThreshold = color[i];
        m_cubeTypeList[i].color = i;
    }
    Cube cube1(1, "vert");
    m_cubeTypeList[VERT].cubeList.push_back(cube1);
    Cube cube2(2, "vert");
    m_cubeTypeList[VERT].cubeList.push_back(cube2);
    Cube cube3(3, "rouge");
    m_cubeTypeList[ROUGE].cubeList.push_back(cube3);
    Cube cube4(4, "bleu");
    m_cubeTypeList[BLEU].cubeList.push_back(cube4);
    Cube cube5(5, "noir");
    m_cubeTypeList[NOIR].cubeList.push_back(cube5);
}

void Robot::MajCubePos(Mat image)
{
    for (int colorID = 0; colorID < COLOR_NBR; colorID++) {
        Mat imgHSV;
        Mat imgThresholded;
        vector<vector<Point> > contours;
        vector<vector<Point> > Selectedcontours;
        Point2f tempoContours[4];
        cvtColor(image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV 

        inRange(imgHSV, Scalar(m_cubeTypeList[colorID].colorThreshold[0], m_cubeTypeList[colorID].colorThreshold[2], m_cubeTypeList[colorID].colorThreshold[4]), Scalar(m_cubeTypeList[colorID].colorThreshold[1], m_cubeTypeList[colorID].colorThreshold[3], m_cubeTypeList[colorID].colorThreshold[5]), imgThresholded);
        //erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        //dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        findContours(imgThresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++) { /// test de la taille des contours

            if (((int)contourArea(contours[i]) > MIN_AREA_CUBE) && ((int)contourArea(contours[i]) < MAX_AREA_CUBE)) {

                Selectedcontours.push_back(contours[i]);
            }

        }
        if (Selectedcontours.size() >= m_cubeTypeList[colorID].cubeList.size()) {
            for (int i = 0; i < m_cubeTypeList[colorID].cubeList.size(); i++) {
                vector<Point> rectangleContours;
                minAreaRect(Selectedcontours[i]).points(tempoContours);
                for (int j = 0; j < 4; j++) rectangleContours.push_back(Point((int)tempoContours[j].x, (int)tempoContours[j].y));
                m_cubeTypeList[colorID].cubeList[i].updatePosition(rectangleContours);
            }
        }
    }
    
}

void Robot::setTeam(Mat image)
{
    if (m_Position.x > (int)image.size().width / 2) {
        m_team = 0; // bleu
        Rect tempoRect(Point(P1_BLUE_DEPOSIT_X, P1_BLUE_DEPOSIT_Y), Point(P2_BLUE_DEPOSIT_X, P2_BLUE_DEPOSIT_Y));
        m_depositZone = tempoRect;
    }
    else {
        m_team = 1; // rouge
        Rect tempoRect(Point(P1_RED_DEPOSIT_X, P1_RED_DEPOSIT_Y), Point(P2_RED_DEPOSIT_X, P2_RED_DEPOSIT_Y));
        m_depositZone = tempoRect;


    }

}

void Robot::setTrajectory(vector<Point> CoordonateList)
{
    m_trajectory.clear();
    //
    if (m_findTarget == 1) {
        if (!CoordonateList.empty()) m_startPos = CoordonateList.front();

        float LastSlope = 1000000;
        for (int i = 1; i < CoordonateList.size(); i++) {
            seg_equation newSegment;
            Point firstPoint = CoordonateList[i - 1];
            Point secondPoint = CoordonateList[i];
            if ((firstPoint.x - secondPoint.x) != 0) newSegment.slope = (float)(firstPoint.y - secondPoint.y) / (firstPoint.x - secondPoint.x);
            else newSegment.Vertical = 1;
            if (newSegment.slope != LastSlope) {
                newSegment.angle = getOrientation(firstPoint, secondPoint);

                newSegment.offset = firstPoint.y - firstPoint.x * newSegment.slope;
                newSegment.startPoint = firstPoint;
                newSegment.endPoint = secondPoint;
                m_trajectory.push_back(newSegment);
                LastSlope = newSegment.slope;
            }
            else m_trajectory.back().endPoint = CoordonateList[i];
        }
        m_trajState = 0;
    }



}

bool Robot::findNewTarget(time_t timer, Mat currentImage)
{

    MajCubePos(currentImage);
    int ImageCenter = currentImage.size().width / 2;
    int couleur = 0;

    Cube targetChoose;
    bool finish = 0;

    if (m_targetType == 0) {
        if (m_team == true) { ///Pas la meme partie de terrain pour bleu et rouge
            for (int i = COLOR_NBR - 1; i >= 0; i--) { /// parcours des couleurs
                int distance = 10000;
                if (i != BLEU) {

                    for (int cube = 0; cube < m_cubeTypeList[i].cubeList.size(); cube++) {
                        if (m_cubeTypeList[i].cubeList[cube].getAvailable() == 1 && norm(m_Position - m_cubeTypeList[i].cubeList[cube].getPosition()) < distance) {
                            distance = norm(m_Position - m_cubeTypeList[i].cubeList[cube].getPosition());
                            targetChoose = m_cubeTypeList[i].cubeList[cube];
                            finish = 1;
                        }
                    }
                }
                if (finish) break;
            }
        }
        else if (m_team == false) { ///Pas la meme partie de terrain pour bleu et rouge
            for (int i = COLOR_NBR - 1; i >= 0; i--) { /// parcours des couleurs
                int distance = 10000;
                if (i != ROUGE) {

                    for (int cube = 0; cube < m_cubeTypeList[i].cubeList.size(); cube++) {
                        if (m_cubeTypeList[i].cubeList[cube].getAvailable() == 1 && norm(m_Position - m_cubeTypeList[i].cubeList[cube].getPosition()) < distance) {
                            distance = norm(m_Position - m_cubeTypeList[i].cubeList[cube].getPosition());
                            targetChoose = m_cubeTypeList[i].cubeList[cube];
                            finish = 1;
                        }
                    }
                }
                if (finish) break;
            }
        }
        m_targetPos = targetChoose.getPosition();

    }
    else {
        m_targetPos.x = (int)((m_depositZone.br().x + m_depositZone.tl().x) / 2);
        m_targetPos.y = (int)((m_depositZone.br().y + m_depositZone.tl().y) / 2);
        finish = 1;
    }
    m_findTarget = finish;
    setTrajectory({ m_Position, m_targetPos });
    return finish;

}

void Robot::updateTraj(Connexion* client) {
    if (m_trajectory.size() > 0 && m_finalRotation == 0) { /// traj en cours
        if (m_trajectory.size() == 1) {
            if ((m_Position.x > m_trajectory.front().endPoint.x - DELTA_GET_CUBE && m_Position.x < m_trajectory.front().endPoint.x + DELTA_GET_CUBE) &&
                (m_Position.y > m_trajectory.front().endPoint.y - DELTA_GET_CUBE && m_Position.y < m_trajectory.front().endPoint.y + DELTA_GET_CUBE)) {
                m_finalRotation = 1;
                m_trajState = 0;
                setTrajectory({ m_Position, m_targetPos });
            }
            
        }
        else {
            m_finalRotation = 0;
            if ((m_Position.x > m_trajectory.front().endPoint.x - DELTA_UP_TRAJ && m_Position.x < m_trajectory.front().endPoint.x + DELTA_UP_TRAJ) &&
                (m_Position.y > m_trajectory.front().endPoint.y - DELTA_UP_TRAJ && m_Position.y < m_trajectory.front().endPoint.y + DELTA_UP_TRAJ)) {
                m_trajectory.erase(m_trajectory.begin());
                m_trajState = 0;
            }
        }
        if (m_trajectory.size() == 0) {
            m_findTarget = 0;
            m_targetType = !m_targetType;
        }
    }
    else if (m_finalRotation == 2) { /// final rotation validé
        int sendCommandCounter = 0;
        while(sendCommandCounter++ < 50) {
            client->SendCommand(0, 5);
            Sleep(10);
        }
        Sleep(1000);
        cout << "next" << endl;
        m_trajectory.erase(m_trajectory.begin());
        m_trajState = 0;
        m_findTarget = 0;
        m_targetType = !m_targetType;
        m_finalRotation = 0;
    }
}

void Robot::afficherTraj(Mat* ResultImage) {
    
    for (int i = 0; i < m_trajectory.size(); i++) {
        line(*ResultImage, m_trajectory[i].startPoint, m_trajectory[i].endPoint, cv::Scalar(0, 255, 255));
        circle(*ResultImage, m_trajectory[i].endPoint, 3, cv::Scalar(0, 0, 255), FILLED);
        circle(*ResultImage, m_trajectory[i].startPoint, 3, cv::Scalar(0, 0, 255),FILLED);
    }
    if(m_trajectory.size() > 0) circle(*ResultImage, m_trajectory.front().endPoint, 3, cv::Scalar(0, 255, 0), FILLED);

    if(m_finalRotation == 0) circle(*ResultImage, m_targetPos, DELTA_GET_CUBE, cv::Scalar(0, 255, 0));
    else circle(*ResultImage, m_targetPos, DELTA_GET_CUBE, cv::Scalar(0, 0, 255));

    if (SHOW_ROBOT_EQUATION) cout << "Equation is : " << m_trajectory.front().slope << "x -y +" << m_trajectory.front().offset << "=0" << endl;
    if (SHOW_ANGLE_TRAJ) cout << "ANGLE TRAJ:" << m_trajectory.front().angle << endl;
    
}

void Robot::afficherFPS(Mat* ResultImage)
{
    m_timerFPS++;
    double tick = (double)getTickCount();
    double currentTime = (tick - m_lastTick) / getTickFrequency();
    m_FPSBuffer += (1 / currentTime);
    if(m_timerFPS == REFRESH_TIME_FPS) {
    m_FPS = (int)m_FPSBuffer / m_timerFPS;
    m_timerFPS = 0;
    m_FPSBuffer = 0;
    }
    rectangle(*ResultImage, Point(0,0), Point(50, 30), Scalar(0,0,0), FILLED, 8);
    cv::putText(*ResultImage,to_string(m_FPS), cv::Point(10,20), cv::FONT_HERSHEY_DUPLEX,0.5, CV_RGB(255,255,255), 2);
    
    
    m_lastTick = tick;
}

void Robot::PIDTraj(Mat* ResultImage, Connexion* client) {
    if (m_trajectory.size() != 0 ) {
        float errorTraj= 0;
        float commandTraj = 0;
        float commandRotation = 0;
        float command = 0;
        int formattedCommand = 0;
        int errorAngle = 0;
        bool angleConvergent = 0; // variable pour savoir si l'angle est divergent ou convergent

        int invPointRot = ((int)(m_trajectory.front().angle - m_RobotAngle + 540)) % 360 - 180;

        int OppositeAngle;
        if (m_trajectory.front().Vertical) errorTraj = m_trajectory.front().startPoint.x - m_Position.x;
        else errorTraj = (m_Position.y - m_trajectory.front().slope * m_Position.x - m_trajectory.front().offset) / sqrt(1 + m_trajectory.front().slope * m_trajectory.front().slope);
        
        errorAngle = MIN(abs(m_trajectory.front().angle - m_RobotAngle), 360 - abs(m_trajectory.front().angle - m_RobotAngle));

        // calcule des commandes de trajectoire
        if (invPointRot > 0) {
            commandRotation = -(errorAngle); // il faut tourner à droite
        }
        else {
            commandRotation = errorAngle; // il faut tourner à gauche
        }
        commandTraj = errorTraj;
        command = commandTraj * KT + commandRotation * KR;
        if (errorTraj > 0 && invPointRot > 0 || errorTraj < 0 && invPointRot < 0) angleConvergent = 1;// test si l'angle du robot est vers
            
        if (m_finalRotation == 0 && ( ((errorAngle > MIN_ANGLE_ENTRY_ERROR) && !m_trajState) || m_trajState && (errorAngle > MIN_ANGLE_ERROR) && angleConvergent) ||
            m_finalRotation == 1 && ( ((errorAngle > MIN_ANGLE_GET_CUBE) && !m_trajState) || m_trajState && (errorAngle > MIN_ANGLE_GET_CUBE) && angleConvergent))  {
            m_trajState = 0;
            int commandP, commandD, commandI,finalcommand,commandeTransfert;
            commandP = abs(errorAngle);
            commandD = errorAngle - m_lastErrorAngle;
            if (m_lastErrorTimer == DELA_SAMPLE_D_ROT) {
                m_lastErrorAngle = errorAngle;
                m_lastErrorTimer = 0;
            }
            m_lastErrorTimer++;
            
            
            if (invPointRot > 0) {
                commandeTransfert = 1;
                
            }
            else {
                commandeTransfert = 2;
                
            }
            
            finalcommand = commandP*KP_ROT + commandD*KD_ROT;
            if (finalcommand < 0) finalcommand = 0;
            if (finalcommand > 100) finalcommand = 100;
            finalcommand = 100 - finalcommand;
            client->SendCommand(finalcommand, commandeTransfert);

            if (SHOW_PID_ERROR) {
                cout << "==== TOURNER ====" << endl;
                cout << "commande P: " <<commandP << endl;
                cout << "commande D: " << commandD<< endl;
                cout << "commande final: " <<finalcommand << endl;
            }

        }
        else if(m_finalRotation > 0) {
            m_finalRotation = 2;
        }
        else if (abs(errorTraj) > MAX_ERROR_TRAJ +  DELTA_NEW_TRAJ) {
            setTrajectory({ m_Position, m_targetPos});
        }
        else { ///régulation sur la trajectoire seulement si le robot à le bon angle de départ
            m_trajState = 1;

            if (command > 0) {
                formattedCommand = mapCommand(abs(command - (MAX_ERROR)), 0, MAX_ERROR, MIN_RPM, 100);
                client->SendCommand(formattedCommand, 3); /// correction droite
            }
            else {
                formattedCommand = mapCommand(abs(-command - (MAX_ERROR)), 0, MAX_ERROR, MIN_RPM, 100);
                client->SendCommand(formattedCommand, 4); /// correction gauche

            }

            if (SHOW_PID_ERROR) {
                if (m_trajState) {
                    cout << "==== AVANCER ====" << endl;
                    cout << "Commande de rotation :" << commandRotation << endl;
                    cout << "Commande de trajectoire :" << commandTraj << endl;
                    if (command < 0) cout << "Commande final:" << formattedCommand << " droite" << endl;
                    else cout << "Commande final:" << formattedCommand << " gauche" << endl;
                }
            }
            if (SHOW_PID_COMMAND_TO_MOTOR) {
                if (command < 0) cout << "commande pour moteur droit :" << formattedCommand << endl;
                else cout << "commande pour moteur gauche :" << formattedCommand << endl;
            }
        }
    }
}

void Robot::afficherCube(Mat* resultImage)
{
    for (int color = 0; color < m_cubeTypeList.size(); color++) {
        for (int i = 0; i < m_cubeTypeList[color].cubeList.size(); i++) {
            if (m_cubeTypeList[color].cubeList[i].getAvailable()) polylines(*resultImage, m_cubeTypeList[color].cubeList[i].getContour(), true, cv::Scalar(255, 0, 0), 1, 8);
            else polylines(*resultImage, m_cubeTypeList[color].cubeList[i].getContour(), true, cv::Scalar(0, 0, 255), 1, 8);

            cv::putText(*resultImage, to_string(m_cubeTypeList[color].cubeList[i].getID()), m_cubeTypeList[color].cubeList[i].getPosition(), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(255, 255, 255), 2);

        }
    }
}





