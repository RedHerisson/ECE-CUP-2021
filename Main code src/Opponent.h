
#include "Robot.hpp"

class Opponent
{
	private:
		Point m_Position;
		Rect m_contour;
		double m_RobotAngle;
		int LastPositionCounter;
		Point m_LastPosition;
		float m_speed; 

		int m_direction;
		seg_equation prevTraj;

		bool m_collision;
		bool m_lastCollision;

		Mat m_roi;

		const float** m_range;
		int* m_histSize;
		int m_channels[10];
		TermCriteria m_term_crit;

		Rect m_track_window;

		int m_mTBlur = 1;
		int m_seuil = 15;
		Mat m_lastImage;
		Mat m_deltaImg;


	public:
		Opponent();
		~Opponent();

		void initTracking(Mat frame, Mat* imageCopy);
		bool updatePosition(Mat image, Mat* imageCopy,Robot bobby);
		void DetectTrajectory(Mat currentImage, Mat* ResultImage);
		void afficherTraj(Mat* ResultImage);
		void collision(Mat currentImage, Mat* ResultImage,Robot* bobby);
		Point getPos() { return m_Position; }
};
