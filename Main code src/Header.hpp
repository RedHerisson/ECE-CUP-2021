
#include "aruco_samples_utility.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <iostream>
#include <stack>
#include <algorithm>
#include <ctime>
#include "aStar.h"

#include<stdio.h>
#include<winsock2.h>
#include <iostream>


#pragma comment(lib,"ws2_32.lib") //Winsock Library
#pragma warning(disable:4996) 

#define SERVER "172.20.10.2"
#define BUFLEN 1	
#define PORT 4210	



namespace {
	const char* about = "Basic marker detection";

	//! [aruco_detect_marktgfders_keys]
	const char* keys =
		"{d        |  1     | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
		"DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
		"DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
		"DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
		"DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
		"{cd       |       | Input file with custom dictionary }"
		"{c		   |1	   |}"
		"{v        |       | Input from video or image file, if ommited, input comes from camera }"
		"{ci       | 0     | Camera id if input doesnt come from video (-v) }"
		"{c        |       | Camera intrinsic parameters. Needed for camera pose }"
		"{l        | 0.1   | Marker side length (in meters). Needed for correct scale in camera pose }"
		"{dp       |       | File of marker detector parameters }"
		"{r        |       | show rejected candidates too }"
		"{refine   |  1    | Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1,"
		"CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}";
}
//! [aruco_detect_markers_keys]

using namespace std;
using namespace cv;

int CameraConfig(cv::VideoCapture videoCapture);

/// CONST OU PARAM

struct seg_equation {
	float slope;
	int offset;
	float angle;
	bool Vertical = 0;
	Point startPoint;
	Point endPoint;
};

#define RESCALE_FACTOR 1

//COLISION
#define DIST_MIN_TRAJ_INTERSECTION 100
#define DIST_MIN_OP_ROBOT 200
#define SPEED_MIN_OP 10
#define EXCL_ZONE_TRAJ_ASTAR 20
#define DELA_SAMPLE_OP_LAST_POS 20 // zone pour valider une fin de traj

// ROBOT

#define HIT_BOX_HEIGHT 90 / RESCALE_FACTOR
#define HIT_BOX_WIDTH 60 / RESCALE_FACTOR

// OPPOSANT

#define MIN_AREA_OP 4500
#define MAX_AREA_OP 7000

// PID CONST
#define MIN_ANGLE_ERROR 20
#define MIN_ANGLE_ENTRY_ERROR 2
#define MIN_ANGLE_GET_CUBE 2
#define MAX_ERROR_TRAJ 10
#define DELTA_NEW_TRAJ 30
#define MIN_RPM 66
#define KT 1
#define KR 2
#define KP_ROT 1.5
#define KD_ROT 3
#define DELA_SAMPLE_D_ROT 5
#define MAX_ERROR MIN_ANGLE_ERROR* KR + MAX_ERROR_TRAJ*KT

#define DELTA_UP_TRAJ 5
#define DELTA_GET_CUBE 80

// ASTAR CONST
#define EXCL_ZONE_ASTAR 100
#define SIZE_GRID_ASTAR 15

///CUBE
#define COLOR_NBR 4
#define MIN_AREA_CUBE 1000 / RESCALE_FACTOR //1000
#define MAX_AREA_CUBE 2000/ RESCALE_FACTOR //2000 

#define P1_RED_DEPOSIT_X 33 / RESCALE_FACTOR
#define P1_RED_DEPOSIT_Y 149 / RESCALE_FACTOR
#define P2_RED_DEPOSIT_X 76 / RESCALE_FACTOR
#define P2_RED_DEPOSIT_Y 361 / RESCALE_FACTOR

#define P1_BLUE_DEPOSIT_X 575 / RESCALE_FACTOR
#define P1_BLUE_DEPOSIT_Y 158 / RESCALE_FACTOR
#define P2_BLUE_DEPOSIT_X 611 / RESCALE_FACTOR
#define P2_BLUE_DEPOSIT_Y 388 / RESCALE_FACTOR

#define REFRESH_TIME_FPS 30
#define GAME_DURATION 900


// CONSOLE AFF
#define SHOW_ROBOT_POS 0
#define SHOW_ROBOT_ANGLE 0
#define SHOW_ROBOT_EQUATION 0
#define SHOW_ANGLE_TRAJ 0

#define SHOW_OP_POS 0
#define SHOW_OP_EQUATION 0
#define SHOW_OP_SPEED 0
#define SHOW_INTERSECTION 0

#define SHOW_DATA_SEND 1

#define SHOW_PID_ERROR 1
#define SHOW_PID_COMMAND_TO_MOTOR 0
#define SHOW_TRAJ_STATE 0

#define VERT	0
#define NOIR	1
#define BLEU	2
#define ROUGE	3








