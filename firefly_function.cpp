// Markus Buchholz
#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>
#include <math.h>
#include <random>

//--------Path Planner---------------------------------------------------------------------------

float xmin = -5.0; // 0.0;
float xmax = 5.0;  // 50.0;
float ymin = -5.0; // 0.0;
float ymax = 5.0;  // 50.0;

float obsX = 25.0;
float obsY = 25.0;
float obsR = 5.0;

float goalX = 45.0;
float goalY = 45.0;

float startX = 2.0;
float startY = 2.0;

float factorKp = 10.0;
float deltaK = 10.0;

//----------------------FireFly Optimizer----------------------------------------

int EVOLUTIONS = 500;
int FIREFLYS = 100;
float beta0 = 1.0;
float Gamma = 0.01;
float alfa0 = 1.0;
float delta = 0.97;
float SCALEX = xmax - xmin;
float SCALEY = ymax - ymin;

//--------------------------------------------------------------------------------

struct Pos
{

    float x;
    float y;
};

//--------------------------------------------------------------------------------

float euclid(Pos a, Pos b)
{

    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}
//--------------------------------------------------------------------------------

float generateRandom()
{

    std::random_device engine;
    std::uniform_real_distribution<float> distrib(-1.0, 1.0);
    return distrib(engine);
}

//--------------------------------------------------------------------------------

float valueGenerator(float low, float high)
{

    return low + generateRandom() * (high - low);
}

//--------------------------------------------------------------------------------

std::vector<float> function(std::vector<Pos> pos)
{
    std::vector<float> funcValue;

    for (auto &ii : pos)
    {

        funcValue.push_back(ii.x * ii.y);
    }

    return funcValue;
}

//--------------------------------------------------------------------------------

float func(Pos pos)
{

    return pos.x * pos.y;
}

//--------------------------------------------------------------------------------

Pos positionUpdateCheck(Pos actualPos)
{

    Pos Pnew = actualPos;

    if (Pnew.x < xmin)
    {
        Pnew.x = xmin;
    }

    if (Pnew.x > xmax)
    {
        Pnew.x = xmax;
    }

    if (Pnew.y < ymin)
    {
        Pnew.y = ymin;
    }

    if (Pnew.y > ymax)
    {
        Pnew.y = ymax;
    }

    return Pnew;
}

//--------------------------------------------------------------------------------

Pos posUpdate(Pos flya, Pos flyb)
{

    Pos Xnew;

    float t = -Gamma * std::pow(euclid(flya, flyb), 2);
    float r = generateRandom();

    Xnew.x = flya.x + beta0 * std::exp(t) * (flyb.x - flya.x) + alfa0 * delta * (r - 0.5) * SCALEX;
    Xnew.y = flya.y + beta0 * std::exp(t) * (flyb.y - flya.y) + alfa0 * delta * (r - 0.5) * SCALEY;

    return positionUpdateCheck(Xnew);
}

//--------------------------------------------------------------------------------

std::vector<Pos> initPosXY()
{

    std::vector<Pos> pos;

    for (int ii = 0; ii < FIREFLYS; ii++)
    {

        pos.push_back({valueGenerator(xmin, xmax), valueGenerator(ymin, ymax)});
    }

    return pos;
}

//-------------------------------------------------------------------------------
void runFireFly()
{

    std::vector<Pos> actualPositions = initPosXY();
    std::vector<float> actualValueFunction = function(actualPositions);

    for (int ii = 0; ii < EVOLUTIONS; ii++)
    {

        for (int fly_i = 0; fly_i < FIREFLYS; fly_i++)
        {

            for (int fly_j = 0; fly_j < FIREFLYS; fly_j++)
            {

                if (fly_i != fly_j)
                {

                    // std::cout << "fly i = " << fly_i << " fly j " << fly_j << "\n";
                    // std::cout <<"----------------------------------------------" <<std::endl;
                    if (actualValueFunction[fly_i] > actualValueFunction[fly_j])
                    {

                        Pos fly_i_NewPos = posUpdate(actualPositions[fly_i], actualPositions[fly_j]);
                        float fly_i_NewFunction = func(fly_i_NewPos);
                        if (fly_i_NewFunction < actualValueFunction[fly_i])
                        {
                            actualValueFunction[fly_i] = fly_i_NewFunction;
                            actualPositions[fly_i] = fly_i_NewPos;
                        }
                    }
                }
            }
        }
        // for (auto &ii : actualPositions)
        // {
        //     std::cout << "x = " << ii.x << " y = " << ii.y << "\n";
        // }
        // std::cout <<"----------------------------------------------" <<std::endl;
        for (auto &ii : actualValueFunction)
        {
            std::cout << "f = " << ii << "\n";
        }
    }
}

//--------------------------------------------------------------------------------

int main()
{
    runFireFly();
}