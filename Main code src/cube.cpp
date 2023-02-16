#include "cube.hpp"

Cube::Cube()
{

}

Cube::Cube(int id, string color)
    :m_ID(id), m_color(color), m_availableToTake(-1), m_hasMovedCounter(0)
{

}

Cube::~Cube()
{

}



int Cube::checkCubeMovement(Mat image)
{

	return 0;
}

void Cube::updatePosition(vector<Point> contour)
{
    m_contour = contour;
    m_Position.x = (contour[0].x + contour[2].x) / 2;
    m_Position.y = (contour[0].y + contour[2].y) / 2;
    Rect deposit1(Point(P1_RED_DEPOSIT_X, P1_RED_DEPOSIT_Y), Point(P2_RED_DEPOSIT_X, P2_RED_DEPOSIT_Y));
    Rect Deposit2(Point(P1_BLUE_DEPOSIT_X, P1_BLUE_DEPOSIT_Y), Point(P2_BLUE_DEPOSIT_X, P2_BLUE_DEPOSIT_Y));
    if (m_availableToTake != 0 && m_Position.x != 0 && m_Position.y != 0) m_availableToTake = 1;
    
    if (deposit1.contains(m_Position) || Deposit2.contains(m_Position)) m_availableToTake = 0;
}


