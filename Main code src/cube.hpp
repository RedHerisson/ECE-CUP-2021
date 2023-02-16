
#include "Connexion.h"

class Cube {

private:
	vector<Point> m_contour;
	cv::Point m_Position;
	int m_ID;
	string m_color;

	cv::Point m_LastPosition;
	int m_hasMovedCounter;

	int m_availableToTake;
	

public: 
	Cube();
	Cube(int id, string color);
	~Cube();
	void config();
	int checkCubeMovement(Mat image);
	void updatePosition(vector<Point> contour);

	void setAvailable(int available) { m_availableToTake = available; };

	vector<Point> getContour() { return m_contour; }
	int getID() { return m_ID;}
	string getColorString() { return m_color; }
	Point getPosition() { return m_Position; }
	int getAvailable() { return m_availableToTake; }
};

struct cubeType {
	vector<Cube> cubeList;
	vector<int> colorThreshold;
	int color;
};