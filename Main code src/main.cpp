#include "Opponent.h"



void affichage(Mat imgThresholded, Mat ResultImage, Robot bobby, time_t timer);
void ThresholdUpdate(VideoCapture inputVideo, vector<cubeType> cubeTypeList);
void getArea(Mat image);

int main(int argc, char* argv[]) {

    int config = 0;
    bool findTeam = 0;

    Connexion client;
    client.initConnexionToServer();

    Robot bobby;

    Opponent OpRobot;
    float range_[] = { 0, 180 };
    const float* range[] = { range_ };

    bobby.setCubeList();
    bobby.initArUco(argc, argv);
    namedWindow("out", WINDOW_NORMAL);
    moveWindow("out", 100, 100);
    
    Mat imgThresholded;
    Mat ResultImage;

    VideoCapture inputVideo(1);
    //VideoCapture inputVideo("http://169.254.252.202:8081");
    if (inputVideo.isOpened() == false)
    {
        cout << "La camera ne peut etre ouverte" << endl;
        cin.get();
        return -1;
    }

    //CameraConfig(inputVideo);
    Mat image;
    inputVideo.read(image);

    if (config == 1) ThresholdUpdate(inputVideo, bobby.getCubeList());
    else if (config == 2) getArea(image);
    else {
        OpRobot.initTracking(image, &ResultImage);
        time_t startTimerMs = std::time(nullptr), timerMs;

        while (inputVideo.grab()) {
            Mat currentImage;
            Mat preScaleImage;
            bool findBobbyPos;
            inputVideo.read(preScaleImage);
            resize(preScaleImage, currentImage, Size((int)preScaleImage.size().width / RESCALE_FACTOR,(int)preScaleImage.size().height / RESCALE_FACTOR), INTER_LINEAR);
            timerMs = std::time(nullptr) - startTimerMs;

            findBobbyPos = bobby.updatePosition(&currentImage, &ResultImage);
            if (findBobbyPos && !findTeam) {
                bobby.setTeam(currentImage);
                findTeam = true;
            }
            if (!bobby.getIfFindTarget() && findBobbyPos) bobby.findNewTarget(timerMs, currentImage);

            OpRobot.updatePosition(currentImage, &ResultImage, bobby);
            OpRobot.DetectTrajectory(currentImage, &ResultImage);
            OpRobot.collision(currentImage, &ResultImage, &bobby);

            bobby.PIDTraj(&ResultImage, &client);
            bobby.updateTraj(&client);
            //Rect(ResultImage, Point(OpRobot.getPos().x - EXCL_ZONE_ASTAR, OpRobot.getPos().y - EXCL_ZONE_ASTAR), Point(OpRobot.getPos().x + EXCL_ZONE_ASTAR, OpRobot.getPos().y + EXCL_ZONE_ASTAR), cv::Scalar(255, 0, 0));

            bobby.afficherFPS(&ResultImage);
            affichage(imgThresholded, ResultImage, bobby, timerMs);

            if (waitKey(1) == 27)
            {
                cout << "Echap pressee, la video est interrompue" << endl;
                break;
            }
            if (timerMs > GAME_DURATION) {
                while(1) client.SendCommand(7, 7);
            }
            bobby.getGeneratorAStar()->clearCollisions();

        }
    }
}

void affichage(Mat imgThresholded, Mat ResultImage, Robot bobby, time_t timer) {
 
    bobby.afficherTraj(&ResultImage);
    bobby.afficherCube(&ResultImage);
    rectangle(ResultImage, bobby.getDepositZone(), cv::Scalar(255, 255, 255));
    rectangle(ResultImage, Point(ResultImage.size().width, 0), Point(ResultImage.size().width - 50, 30), Scalar(0, 0, 0), FILLED, 8);
    cv::putText(ResultImage, to_string(timer), cv::Point(ResultImage.size().width-40, 25), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 255, 255), 2);
    Point2f hitBoxCorners[4]; 
    imshow("out", ResultImage); //show the original image 
}

void ThresholdUpdate(VideoCapture inputVideo, vector<cubeType> cubeTypeList) {
    
    char choix = 'o';
    int color;

    do {
        
        cout << "Espace de configuration des seuils de couleur, veuillez choisir une couleur à modifier:" << endl;
        cout << "0 - VERT" << endl;
        cout << "1 - ROUGE" << endl;
        cout << "2 - BLEU" << endl;
        cout << "3 - NOIR" << endl;
        cout << "       //" << endl;
        cin >> color;
        cout << "CHOIX DE LA COULEUR : " << color << endl;
        namedWindow("Control", WINDOW_AUTOSIZE); //create a window called "Control" 
        createTrackbar("LowH", "Control", &cubeTypeList[color].colorThreshold[0], 179); //Hue (0 - 179) 
        createTrackbar("HighH", "Control", &cubeTypeList[color].colorThreshold[1], 179);

        createTrackbar("LowS", "Control", &cubeTypeList[color].colorThreshold[2], 255); //Saturation (0 - 255) 
        createTrackbar("HighS", "Control", &cubeTypeList[color].colorThreshold[3], 255);

        createTrackbar("LowV", "Control", &cubeTypeList[color].colorThreshold[4], 255); //Value (0 - 255) 
        createTrackbar("HighV", "Control", &cubeTypeList[color].colorThreshold[5], 255);
        while (inputVideo.grab()) {
            Mat image;
            inputVideo.read(image);


            Mat imgHSV;
            Mat imgThresholded;
            cvtColor(image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV 

            inRange(imgHSV, Scalar(cubeTypeList[color].colorThreshold[0], cubeTypeList[color].colorThreshold[2], cubeTypeList[color].colorThreshold[4]), Scalar(cubeTypeList[color].colorThreshold[1], cubeTypeList[color].colorThreshold[3], cubeTypeList[color].colorThreshold[5]), imgThresholded);
            //erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
            //dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
            imshow("Thresholded", imgThresholded);

            if (waitKey(1) == 27)
            {
                cout << "Echap pressee, la video est interrompue" << endl;
                break;
            }

        }

        cout << "continuer la configuration ?" << " o/n" << endl;
        cin >> choix;
        destroyWindow("Control");
    } while (choix == 'o');
        cout << "=========================" << endl;
        cout << "Resultat de la configuration :" << endl;
        for (auto type : cubeTypeList) {
            cout << "{";
            for (auto seuil : type.colorThreshold) {
                cout << seuil << ",";
            }
            cout << "}" << endl;
        }
}

void getArea(Mat image) {
    // setup initial location of window
    Rect trackWindow;
    Rect windows(130, 70, 100, 50); // simply hardcoded the values
    trackWindow = selectROI(image);
    cout << "aire :" << trackWindow.area() << endl;
    cout << "top Left:" << trackWindow.tl() << endl;
    cout << "bottom Right" << trackWindow.br() << endl;
}



