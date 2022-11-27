// Markus Buchholz
// g++ firefly_robot.cpp -o t -I/usr/include/python3.8 -lpython3.8

#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>
#include <math.h>
#include <random>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

//--------Path Planner----------------------------------------------------------

float xmin = 0.0;
float xmax = 50.0;
float ymin = 0.0;
float ymax = 50.0;

float obsX = 25.0;
float obsY = 25.0;
float obsR = 3.0;

float goalX = 45.0;
float goalY = 45.0;

float startX = 2.0;
float startY = 2.0;

//----------------------FireFly Optimizer----------------------------------------

int EVOLUTIONS = 100;
int FIREFLYS = 100;
float beta0 = 1.0;
float Gamma = 0.01;
float alfa0 = 1.0;
float delta = 0.97;
float SCALEX = xmax - xmin;
float SCALEY = ymax - ymin;

float K1 = 1.0 / obsR; // fitting parameter table 1
float K2 = 0.0001;     // fitting parameter table 2

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
    Pos Obs{obsX, obsY};
    Pos Goal{goalX, goalY};

    for (auto &ii : pos)
    {

        funcValue.push_back(K1 * (1 / euclid(Obs, ii)) + K2 * euclid(Goal, ii));
    }

    return funcValue;
}

//--------------------------------------------------------------------------------

float func(Pos pos)
{
    Pos Obs{obsX, obsY};
    Pos Goal{goalX, goalY};

    return K1 * (1 / euclid(Obs, pos)) + K2 * euclid(Goal, pos);
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
std::vector<Pos> runFireFly()
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
    }

    return actualPositions;
}

//--------------------------------------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>> gen_circle(float a, float b, float r)
{

    std::vector<float> xX;
    std::vector<float> yY;

    for (float dt = -M_PI; dt < M_PI; dt += 0.01)
    {

        xX.push_back(a + r * std::cos(dt));
        yY.push_back(b + r * std::sin(dt));
    }
    return std::make_tuple(xX, yY);
}

//-----------------------------------------------------------------------------------------

void plot2D(std::vector<float> xX, std::vector<float> yY)
{
    std::sort(xX.begin(), xX.end());
    std::sort(yY.begin(), yY.end());

    std::tuple<std::vector<float>, std::vector<float>> circle = gen_circle(obsX, obsY, obsR);

    std::vector<float> xObs = std::get<0>(circle);
    std::vector<float> yObs = std::get<1>(circle);

    plt::plot(xX, yY);
    plt::plot(xObs, yObs);
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::show();
}

//------------------------------------------------------------------------------------------

int main()
{
    std::vector<Pos> path = runFireFly();

    std::vector<float> xX;
    std::vector<float> yY;

    for (auto &ii : path)
    {
        xX.push_back(ii.x);
        yY.push_back(ii.y);

        std::cout << ii.x << " ," << ii.y << "\n";
    }

    plot2D(xX, yY);
}
