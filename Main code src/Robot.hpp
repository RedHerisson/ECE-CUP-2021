#include "cube.hpp"


class Robot {
	
	private : 
		cv::Point m_Position;
		Rect m_hitBox;
		double m_RobotAngle;
		AStar::Generator m_generator;

		Mat m_camMatrix;
		Mat m_distCoeffs;
		Ptr<aruco::Dictionary> m_dictionary;
		Ptr<aruco::DetectorParameters> m_detectorParams;
		bool m_showRejected;
		bool m_estimatePose;
		float m_markerLength;
		float m_lastTick;
		int m_timerFPS;
		int m_FPSBuffer;
		int m_FPS;

		vector<seg_equation> m_trajectory;

		Point m_targetPos;
		Point m_startPos;
		bool m_findTarget;
		bool m_trajState;
		int m_finalRotation; //0 - false 1- true - 2 validé

		bool m_targetType; /// 0 - cube 1 - depot

		int m_distanceFromSensor;
		int m_cubeDetectedCounter; 

		bool m_team;
		Rect m_depositZone;

		vector<cubeType> m_cubeTypeList;

		int m_lastErrorAngle;
		int m_lastErrorTimer;


	
	public : 
		Robot();
		~Robot();

		int initArUco(int argc, char* argv[]);
		
		bool updatePosition(Mat* image, Mat* imageCopy);
		void updateTraj(Connexion* client);
		bool findNewTarget(time_t timer, Mat currentImage);
		void MajCubePos(Mat image);
		void PIDTraj(Mat* ResultImage, Connexion* client);

		void setCubeList();
		void setTeam(Mat image);
		void setTrajectory(vector<Point> CoordonateList);

		bool getIfFindTarget() { return m_findTarget; }
		float getCoeffTraj() { 
			if (m_trajectory.size() != 0) return m_trajectory.front().slope; 
			else return 0; 
		}
		int getoffsetTraj() {
			if (m_trajectory.size() != 0) return m_trajectory.front().offset;
			else return 0;
		}

		Point getPosition() { return m_Position; }
		Point getTargetPos() { return m_targetPos; }
		vector<seg_equation> getTrajectory() { return m_trajectory; }
		AStar::Generator* getGeneratorAStar() { return &m_generator; }
		Rect getDepositZone() { return m_depositZone;  }
		vector<cubeType> getCubeList() { return m_cubeTypeList; }
		Rect getHitBox() { return m_hitBox; }

		void afficherTraj(Mat* ResultImage);
		void afficherFPS(Mat* ResultImage);
		void afficherCube(Mat* resultImage);
		



};
