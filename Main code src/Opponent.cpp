
#include "Opponent.h"


float getOrientation_op(Point p1, Point p2) {
    float angle;
    double Hyp = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    double Opp = (p1.x - p2.x);
    double sin = Opp / Hyp;

    angle = -asin(sin);

    angle = (angle * (180 / 3.141592));
    if (p1.x <= p2.x && p1.y < p2.y) angle = 180 - angle;
    if (p1.x >= p2.x && p1.y > p2.y) angle = -180 - angle;

    if (angle < 0) angle = -angle + 180;

    return angle;
}

Opponent::Opponent()
    : LastPositionCounter(0)
{
}


Opponent::~Opponent()
{
}

void Opponent::initTracking(Mat frame, Mat* imageCopy) 
{
        // setup initial location of window
        Mat ResizedFrame;
        resize(frame, ResizedFrame, Size((int)frame.size().width / RESCALE_FACTOR, (int)frame.size().height / RESCALE_FACTOR), INTER_LINEAR);

        m_track_window = selectROI(ResizedFrame);
        // set up the ROI for tracking
        m_roi = ResizedFrame(m_track_window);
        m_Position = (m_track_window.br() + m_track_window.tl()) / 2;
        m_contour = m_track_window;
        cvtColor(frame,m_lastImage, COLOR_BGR2GRAY);
        GaussianBlur(m_lastImage, m_lastImage, Size(m_mTBlur, m_mTBlur), 0);

}

bool Opponent::updatePosition(Mat frame, Mat* imageCopy, Robot bobby) {

    if (frame.empty())
        cout << "no frame" << endl;
    else {
        Mat grayImage; 
        vector<vector<Point> > contours;
        Point2f tempoContours[4];
        
        vector<Point> rectangleContours, Selectedcontours;
        cvtColor(frame,grayImage, COLOR_BGR2GRAY);
        GaussianBlur(grayImage, grayImage, Size(m_mTBlur, m_mTBlur), 0);
        absdiff(grayImage, m_lastImage, m_deltaImg);
        threshold(m_deltaImg,m_deltaImg, m_seuil, 255, THRESH_BINARY);
        dilate(m_deltaImg, m_deltaImg, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
        m_lastImage = grayImage;
        findContours(m_deltaImg, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++) { /// test de la taille des contours

            if (((int)contourArea(contours[i]) > MIN_AREA_OP) && ((int)contourArea(contours[i]) < MAX_AREA_OP)) {

                
                Selectedcontours = contours[i];
                minAreaRect(Selectedcontours).points(tempoContours);
                for (int j = 0; j < 4; j++) rectangleContours.push_back(Point((int)tempoContours[j].x, (int)tempoContours[j].y));
                if (!bobby.getHitBox().contains((tempoContours[0] + tempoContours[2] )/2)) {
                    Rect tempoRect(tempoContours[0], tempoContours[2]);
                    m_Position = (tempoRect.br() + tempoRect.tl()) / 2;
                    m_contour = tempoRect;
                    break;
                }
                
            }

        }
        
        rectangle(*imageCopy, m_contour, Scalar(255, 255, 0));
        if(SHOW_OP_POS) cout << m_Position << endl;

    }


    
    return 1;
}


void Opponent::DetectTrajectory(Mat currentImage, Mat* ResultImage)
{
    prevTraj.slope = 0;
    prevTraj.Vertical = 0;

    prevTraj.angle = getOrientation_op(m_Position, m_LastPosition);
    if ((m_Position.x - m_LastPosition.x) != 0) prevTraj.slope = (float)(m_Position.y - m_LastPosition.y) / (m_Position.x - m_LastPosition.x);
    else prevTraj.Vertical = 1;
    prevTraj.offset = m_Position.y - m_Position.x * prevTraj.slope;

    m_speed = sqrt(pow(m_Position.x - m_LastPosition.x, 2) + pow(m_Position.y - m_LastPosition.y, 2));
    if(SHOW_OP_EQUATION) cout << "Equation is : " << prevTraj.slope << "x -y +" << prevTraj.offset <<"=0" << endl;
    if (SHOW_OP_SPEED) cout << "SPEED:" << m_speed<< endl;

    int drawOffset;
    if (m_Position.x - m_LastPosition.x < 0) drawOffset = 0;
    else drawOffset = currentImage.size().width;

    if (m_LastPosition.y > m_Position.y) m_direction = 1;
    else m_direction = 0;
    
    if (m_speed > SPEED_MIN_OP) {
        if(!prevTraj.Vertical) line(*ResultImage, m_Position, Point((drawOffset), (drawOffset)*prevTraj.slope + prevTraj.offset), cv::Scalar(0, 255, 255));
        else {
            if (m_direction) line(*ResultImage, m_Position, Point(m_Position.x, 0), cv::Scalar(0, 255, 0));
            else line(*ResultImage, m_Position, Point(m_Position.x, currentImage.size().height), cv::Scalar(0, 255, 0));

        }
    }
    }

void Opponent::afficherTraj(Mat* ResultImage) {
   
}

void Opponent::collision(Mat currentImage, Mat* ResultImage, Robot* bobby)
{

    Point InterTraj;
    
    bool intersection = 0;
    float distanceOpTraj = 0.0;
    float distanceOpRob = 0.0;
    int Xmin, Xmax;
    
    if(m_collision) bobby->setTrajectory({ bobby->getPosition(), bobby->getTargetPos() });

    /// COllision avec Opposant trop proche de la trajectoire
    for (auto traj : bobby->getTrajectory()) {
        if (traj.Vertical) distanceOpTraj = traj.startPoint.x - m_Position.x;
        else distanceOpTraj = (m_Position.y - traj.slope * m_Position.x - traj.offset) / sqrt(1 + traj.slope * traj.slope);
        
        distanceOpRob = norm(m_Position - bobby->getPosition());

        if ((abs(distanceOpTraj) < DIST_MIN_TRAJ_INTERSECTION) && (abs(distanceOpRob) < DIST_MIN_OP_ROBOT)) m_collision = 1;
        else m_collision = 0;
    }

    if (m_collision) {
        vector<Point> CoordinateListInPoint;
        bool isTrajPossible = 1;

        if ((bobby->getTargetPos().x < m_Position.x + EXCL_ZONE_ASTAR && bobby->getTargetPos().x > m_Position.x - EXCL_ZONE_ASTAR && bobby->getTargetPos().y < m_Position.y + EXCL_ZONE_ASTAR && bobby->getTargetPos().y > m_Position.y - EXCL_ZONE_ASTAR)) {
            isTrajPossible = 0;
        }

        if (isTrajPossible) {
            for (int i = m_Position.x - EXCL_ZONE_ASTAR; i < m_Position.x + EXCL_ZONE_ASTAR; i++) {
                for (int j = m_Position.y - EXCL_ZONE_ASTAR; j < m_Position.y + EXCL_ZONE_ASTAR; j++) {
                    if (i != bobby->getTargetPos().x && j != bobby->getTargetPos().y) {
                        bobby->getGeneratorAStar()->addCollision({ i / SIZE_GRID_ASTAR,j / SIZE_GRID_ASTAR });
                    }
                }
            }




            rectangle(*ResultImage, Point(m_Position.x + EXCL_ZONE_ASTAR, m_Position.y + EXCL_ZONE_ASTAR), Point(m_Position.x - EXCL_ZONE_ASTAR, m_Position.y - EXCL_ZONE_ASTAR), cv::Scalar(255, 0, 0));
            
            AStar::CoordinateList path;
            if (bobby->getTrajectory().empty()) path = bobby->getGeneratorAStar()->findPath({ bobby->getPosition().x / SIZE_GRID_ASTAR, bobby->getPosition().y / SIZE_GRID_ASTAR }, { bobby->getTargetPos().x / SIZE_GRID_ASTAR, bobby->getTargetPos().y / SIZE_GRID_ASTAR });
            else path = bobby->getGeneratorAStar()->findPath({ bobby->getTrajectory().front().startPoint.x / SIZE_GRID_ASTAR, bobby->getTrajectory().front().startPoint.y / SIZE_GRID_ASTAR }, { bobby->getTargetPos().x / SIZE_GRID_ASTAR, bobby->getTargetPos().y / SIZE_GRID_ASTAR });
            

            for (auto point : path) {
                CoordinateListInPoint.push_back(Point(point.x * SIZE_GRID_ASTAR, point.y * SIZE_GRID_ASTAR));

            }
            if(!CoordinateListInPoint.empty())
            reverse(CoordinateListInPoint.begin(), CoordinateListInPoint.end());

            bobby->setTrajectory(CoordinateListInPoint);
            m_collision = 1;
        }
    }    
}








