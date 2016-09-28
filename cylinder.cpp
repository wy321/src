/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  Cristian Rodriguez <u5419700@anu.edu.au>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <iostream>
#include "cylinder.hpp"

Cylinder::Cylinder()
{
    tapes = new std::vector<Tape>;
    this->Cx = 0;
    this->Cy = 0;
    this->Xw = 0;
    this->Zw = 0.0;
    this->f  = 320.0f * tan(29.25);
    this->invF = 1.0f/f;
}

Cylinder::~Cylinder()
{
}

bool Cylinder::addTape(Tape tape_)
{
    if(this->tapes->empty())
    {
        this->Cx = tape_.Cx;
        this->Zw = tape_.Zw;
        this->tapes->push_back(tape_);
        return true;
    }
    else
    {
        if(tape_.Cx <= this->Cx + 15 && this->Cx - 15 <= tape_.Cx)
        {  
            this->Cx = tape_.Cx + this->Cx;
            this->tapes->push_back(tape_);
            return true;
        }
        else
        {
            return false;
        }
    }
}

std::vector<Tape> Cylinder::getTape()
{
    return *tapes;
}

/**
 * 
 * 
 */
uint Cylinder::getLabel()
{
    if(this->tapes->size() == 4)
    {
        std::sort (this->tapes->begin(),this->tapes->end(),sortTapes);
        colors aux;
        aux.first = tapes->at(0).Color;
        aux.second = tapes->at(1).Color;
        aux.third = tapes->at(2).Color;
        aux.fourth = tapes->at(3).Color;
        std::map<colors,int>::const_iterator it = labelMap.find(aux);
        if (it != labelMap.end())
            return it->second;
        else
            return 100;
    }
    else
    {
        return 100;        
    }
}

    
cv::Point2d Cylinder::getWorld()
{   
    this->Xw = (int)((Zw * (Cx-320) * invF));
    cv::Point2d ret(Xw,Zw);
    return ret;
}
