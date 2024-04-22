/*
 *	Copyright (C)2020 by YOUR NAME HERE
 *
 *	RoboComp is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	RoboComp is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

#include <utility>
#include <algorithm>
#include <random>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx, bool WorldFR, bool DrawObstacles) : GenericWorker(mprx)
{
	this->data_read.gridSize = -1; // Just to know when it's been initialised
    this->WorldFR = WorldFR;
    this->DrawObstacles = DrawObstacles;

    
	initialiseNeighbours();

	srand(time(NULL));

    this->robot_x = 0;
    this->robot_y = 0;
    this->robot_angle = 0;

    target_x = 0.;
    target_y = 0.;

	world_target_x = 0.;
	world_target_y = 0.;

    prev_adv = 0.;
    prev_rot = 0.;


}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    omnirobot_proxy->setSpeedBase(0., 0., 0.);
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);
}

/*
 * Generates the vector of neighbours to check.
 * 
 * The loops are a little bit convoluted because I wanted to make sure that close neighbours have priority over
 * neighbours in the same direction which are far away.
 * 
 * Basically, we iterate in x and y distance, and their two possible signs. Points are only added if there is
 * no previous (closer) neighbours in the same direction.
 * 
 */ 
void SpecificWorker::initialiseNeighbours()
{
    int exp_factor = 2;
	for (int ix = 0; ix<=exp_factor; ix++)
	{
		for (int iy = 0; iy<=exp_factor; iy++)
		{
			for (int mx=-1; mx <= 1; mx+=2)
			{
				for (int my=-1; my <= 1; my+=2)
				{
					int x = ix*mx;
					int y = iy*my;
					
					if (x==0 and y==0) continue;
                    float angle = atan2(y, x);
                    bool skip = false;
                    for (auto p : neighList)
                    {
                        float angle2 = atan2(p.y, p.x);
                        float angle_diff = abs(angle2-angle);
                        if (angle_diff < 0.01 and (norm(Point(x,y))>norm(Point(p.x,p.y))))
                        {
                            skip = true;
                            break;
                        }
                    }
                    if (skip) continue;
					neighList.push_back(Point(x,y));
				}
			}
		}
	}
}

void SpecificWorker::compute()
{
    //std::cout << "Coordinates (" << this->target_x << ", " << this->target_y << ")" << std::endl;
    mtx_read.lock();
	if (this->grid.total() == 0 and this->data_read.gridSize > 0)
	{
        grid = cv::Mat(this->data_read.gridSize, this->data_read.gridSize, CV_32FC1);
        // std::cout<<"grid size "<<this->data_read.gridSize<<std::endl;
        SOCNAV_AREA_WIDTH = 1500; //this->data_read.areaSize;
	}

    mtx_read.unlock();

    //std::cout << this->grid.total() << std::endl;
    //std::cout << this->data_read.g.size() << std::endl;

	// for (int i=0; i<50; i++)
	// {
	// 	this->grid.at<float>(i,i) = 1;
	// }

    // ASSUMING 150 for grid size a 15mx15m for area size. Each cell represents 10cmx10cm

	if (this->grid.total() > 0)
	{
        int new_size = (int) rint(this->data_read.areaSize/10.);
        Mat tempGrid(this->data_read.gridSize, this->data_read.gridSize, CV_32FC1);
        
        memcpy(tempGrid.ptr(), &this->data_read.g[0], this->data_read.g.size()*4);
        cv::resize(tempGrid, tempGrid, Size(new_size, new_size));
        cv::resize(this->grid, this->grid, Size(150,150));
        this->grid.setTo(1.);
        if(tempGrid.rows>this->grid.rows)
            qFatal("wrong size for grid");
        int margin = (grid.rows-tempGrid.rows)/2;
        tempGrid.copyTo(grid(Rect(margin, margin, tempGrid.rows, tempGrid.cols)));
		cv::resize(this->grid, this->grid_big, cv::Size(), 5, 5);

        if(DrawObstacles)
        {
            add_objects_to_grid(this->grid_big);
            add_walls_to_grid(this->grid_big);
        }

        this->grid_big.convertTo(visible_grid, CV_8UC1, 255);

        cv::cvtColor(this->visible_grid, this->visible_grid, COLOR_GRAY2RGB);

//		Vec3f wTarget = r2w*Vec3f(-this->target_x*100, -this->target_y*100, 1);

		//std::cout << "wtarget " << wTarget[0] <<" " << wTarget[1] << "previous" << world_target_x << " " << world_target_y << std::endl;

        mtx_goal.lock();
        float goal_x = this->target_x*100 / SOCNAV_AREA_WIDTH;
        float goal_y = this->target_y*100 / SOCNAV_AREA_WIDTH;
        mtx_goal.unlock();

        //std::cout << "GOAL " << goal_x << goal_y<< std::endl;

        int img_goal_x = int(goal_x*grid_big.cols + grid_big.cols/2);
        int img_goal_y = int(-goal_y*grid_big.cols + grid_big.cols/2);

        float init_x = this->robot_x*100 / SOCNAV_AREA_WIDTH;
        float init_y = this->robot_y*100 / SOCNAV_AREA_WIDTH;

        int img_init_x = int(init_x*grid_big.cols + grid_big.cols/2);
        int img_init_y = int(-init_y*grid_big.cols + grid_big.cols/2);


        // Redefine the goal if it is outside the grid
        // if(img_goal_x<GRID_BORDER or img_goal_x>=grid_big.cols-GRID_BORDER or img_goal_y<GRID_BORDER or img_goal_y>=grid_big.rows-GRID_BORDER)
        // {
        //     int new_goal_x, new_goal_y;
        //     redefine_target(img_goal_x, img_goal_y, new_goal_x, new_goal_y);
        //     qDebug()<<"redefine goal"<<img_goal_x<<img_goal_y<<new_goal_x<<new_goal_y;
        //     img_goal_x = new_goal_x;
        //     img_goal_y = new_goal_y;
        // }

        // Redefine the goal if it is an occupied position
        int new_goal_x, new_goal_y;

        find_free_target(img_goal_x, img_goal_y, new_goal_x, new_goal_y, 11);
        img_goal_x = new_goal_x;
        img_goal_y = new_goal_y;



        cv::line(visible_grid, Point(img_goal_x-10, img_goal_y), Point(img_goal_x+10, img_goal_y), Scalar(255, 0, 0), 4);
        cv::line(visible_grid, Point(img_goal_x, img_goal_y-10), Point(img_goal_x, img_goal_y+10), Scalar(255, 0, 0), 4);
        cv::line(visible_grid, Point(img_init_x-10, img_init_y), Point(img_init_x+10, img_init_y), Scalar(0, 0, 255), 4);
        cv::line(visible_grid, Point(img_init_x, img_init_y-10), Point(img_init_x, img_init_y+10), Scalar(0, 0, 255), 4);


        final_path.clear();
        Point start = Point(img_init_x, img_init_y);//Point(this->grid_big.cols/2, this->grid_big.rows/2);
        Point goal = Point(img_goal_x, img_goal_y);

        final_path.push_back(start);
        final_path.push_back(goal);
        std::vector<Point> path;

        if (! check_free_path(final_path, this->grid_big))
        {
            path = find_free_path(this->grid_big, start, goal);

            // qDebug()<<"no direct path"<<path.size();

            final_path.clear();

            if (path.size()>1)
            {
                final_path = create_final_path(path);
            }
            else
            {
                final_path.push_back(start);
                final_path.push_back(goal);
                find_linear_new_target(final_path, this->grid_big);

            }
        }

        std::vector<Point> smoothPath;

        if(final_path.size()>=2)
        {
            cv::approxPolyDP(final_path, smoothPath, 5, false);
            follow_path(smoothPath);
        }
        else
        {
            omnirobot_proxy->setSpeedBase(0., 0., 0.);
            prev_rot = 0;
        }

        // qDebug()<<"final path size"<< final_path.size();
            

        // for (int i=0; i< (int) path.size()-1; i++)
        // {
        //     cv::line(visible_grid, path[i], path[i+1], Scalar(255, 0, 0), 4);
        // }

        for (int i=0; i< (int) smoothPath.size()-1; i++)
        {
            cv::line(visible_grid, smoothPath[i], smoothPath[i+1], Scalar(0, 255, 0), 2);
        }

        Mat resized_grid;
        cv::resize(visible_grid, resized_grid, Size(600,600));
        cv::imshow( "Display window", resized_grid);
	}
	cv::waitKey(1);

    qDebug()<<"compute done";
}

void SpecificWorker::find_free_target(int gx, int gy, int & new_gx, int & new_gy, int w)
{
    Point curr_Point, new_Point, minP, maxP;
    int nIters=0;

    curr_Point.x = gx;
    curr_Point.y = gy;

    new_Point = curr_Point;

    while(grid_big.at<float>(new_Point)<0.5 and nIters<5)
    {   
        int x1 = curr_Point.x-w/2;
        int x2 = curr_Point.x+w/2;
        int y1 = curr_Point.y-w/2;
        int y2 = curr_Point.y+w/2;
        if(x1<0) x1 = 0;
        if(x2>grid_big.cols) x2 = grid_big.cols;
        if(y1<0) y1 = 0;
        if(y2>grid_big.rows) y2 = grid_big.rows;

        Rect area = Rect(x1, y1, x2-x1, y2-y1);
        double min, max;
        Mat im_area = grid_big(area);
        cv::minMaxLoc(im_area, &min, &max, &minP, &maxP);
        new_Point = maxP+Point(x1,y1);
        nIters++;
        w+=9;
    }

    new_gx = new_Point.x;
    new_gy = new_Point.y;

}


std::vector<Point> SpecificWorker::find_free_path(Mat grid, Point start, Point goal)
{
	Mat grid1, grid2;

    cv::resize(grid, grid1, cv::Size(GRID_SIZE1, GRID_SIZE1), cv::INTER_CUBIC);

//    Mat visible_grid1;
//    grid1.convertTo(visible_grid1, CV_8UC1, 255);

//    cv::cvtColor(visible_grid1, visible_grid1, COLOR_GRAY2RGB);

//    cv::imshow( "Display grid1", visible_grid1);


	float sc1 = (float)GRID_SIZE1/(float) grid.cols;

	Point start1 = Point(int(start.x*sc1), int(start.y*sc1));
	Point goal1 = Point(int(goal.x*sc1), int(goal.y*sc1));

    int border1 = 2;//(int) ceil((GRID_BORDER*GRID_SIZE1)/DRAW_SIZE);

    std::vector<Point> path1 = path_planning(grid1, start1, goal1, border1);

	Mat flags_image, min_image;
	grid1.copyTo(flags_image);
	flags_image.setTo(0.);
    int flags_width = 10;

	for (int i=0; i<(int) path1.size()-1; i++)
	{
		cv::line(flags_image, path1[i], path1[i+1], 1, flags_width);
	}

	cv::resize(grid, grid2, cv::Size(GRID_SIZE2, GRID_SIZE2),cv::INTER_CUBIC);


	cv::resize(flags_image, flags_image, Size(GRID_SIZE2, GRID_SIZE2), cv::INTER_CUBIC);

	cv::min(grid2, flags_image, min_image);



	float sc2 = (float)GRID_SIZE2/(float)grid.cols;

	Point start2 = Point(int(start.x*sc2), int(start.y*sc2));
	Point goal2 = Point(int(goal.x*sc2), int(goal.y*sc2));

//    Mat visible_min;
//    min_image.convertTo(visible_min, CV_8UC1, 255);

//    cv::cvtColor(visible_min, visible_min, COLOR_GRAY2RGB);

//    cv::imshow( "Display min", visible_min);

    int border2 = 3;//floor((border1*GRID_SIZE2)/GRID_SIZE1);

    std::vector<Point> path2 = path_planning(min_image, start2, goal2, border2);

	std::vector<Point> path;

	for (uint i = 0; i<path2.size(); i++)
		path.push_back(Point(int(path2[i].x/sc2), int(path2[i].y/sc2)));

	return path;

}

bool SpecificWorker::check_free_path(std::vector<Point> path, Mat grid)
{
	return false;
	Mat grid2;
	cv::resize(grid, grid2, cv::Size(GRID_SIZE2, GRID_SIZE2), cv::INTER_CUBIC);

    int width=4;

	float sc2 = (float)GRID_SIZE2/(float)grid.cols;
	std::vector<Point> path2;
	for (uint i=0; i<path.size(); i++)
	{
		path2.push_back(Point(int(path[i].x*sc2), int(path[i].y*sc2)));
	}

	Point p1, p2;
	float valid_limit = 0.5;
	for (int i=0; i<(int)path2.size()-1; i++)
	{
		if (abs(path2[i].x-path2[i+1].x) > abs(path2[i].y-path2[i+1].y))
		{
			if (path2[i].x<path2[i+1].x)
			{
				p1 = path2[i];
				p2 = path2[i+1];
			}
			else
			{
				p2 = path2[i];
				p1 = path2[i+1];
			}
			for (int x = p1.x; x < p2.x; x++)
			{
				int y = int((float)((x-p1.x)*(p2.y-p1.y))/(float)(p2.x-p1.x) + p1.y);
                if (!check_free_area(grid2, Point(x,y), width*2, valid_limit))
                    return false;
				// if (grid2.at<float>(y,x)<valid_limit)
				// {
				// 	return false;
				// }
			}
		}
		else
		{
			if (path2[i].y<path2[i+1].y)
			{
				p1 = path2[i];
				p2 = path2[i+1];
			}
			else
			{
				p2 = path2[i];
				p1 = path2[i+1];
			}
			for (int y = p1.y; y < p2.y; y++)
			{
				int x = int((float)((y-p1.y)*(p2.x-p1.x))/(float)(p2.y-p1.y) + p1.x);
                if (!check_free_area(grid2, Point(x,y), width*2, valid_limit))
                    return false;

				// if (grid2.at<float>(y,x)<valid_limit)
				// {
				// 	return false;
				// }
			}
		}
	}

	return true;
}

void SpecificWorker::find_linear_new_target(std::vector<Point> &path, Mat grid)
{
	Mat grid2;
	cv::resize(grid, grid2, cv::Size(GRID_SIZE2, GRID_SIZE2), cv::INTER_CUBIC);

    int width=4;

    if(path.size()!=2)
    {
        qFatal("find_linear_new_target: the size of the path should be 2");
    }

	float sc2 = (float)GRID_SIZE2/(float)grid.cols;
	std::vector<Point> path2;
   	for (uint i=0; i<path.size(); i++)
	{
		path2.push_back(Point(int(path[i].x*sc2), int(path[i].y*sc2)));
	}

    float min_cost = 1.;
	Point p1, p2, newTarget = path2[0];
	float valid_limit = 0.5;
    if (abs(path2[0].x-path2[1].x) > abs(path2[0].y-path2[1].y))
    {
        if (path2[0].x<path2[1].x)
        {
            p1 = path2[0];
            p2 = path2[1];
        }
        else
        {
            p2 = path2[0];
            p1 = path2[1];
        }
        for (int x = p1.x; x < p2.x; x++)
        {
            int y = int((float)((x-p1.x)*(p2.y-p1.y))/(float)(p2.x-p1.x) + p1.y);
            float area_cost = compute_min_area_cost(grid2, Point(x,y), width);
            if(area_cost < valid_limit or area_cost < min_cost)
            {
                min_cost = area_cost;
                newTarget = Point(x,y);
            }
            else
                break;

        }
    }
    else
    {
        if (path2[0].y<path2[1].y)
        {
            p1 = path2[0];
            p2 = path2[1];
        }
        else
        {
            p2 = path2[0];
            p1 = path2[1];
        }
        for (int y = p1.y; y < p2.y; y++)
        {
            int x = int((float)((y-p1.y)*(p2.x-p1.x))/(float)(p2.y-p1.y) + p1.x);
            float area_cost = compute_min_area_cost(grid2, Point(x,y), width);
            if(area_cost < valid_limit or area_cost < min_cost)
            {
                min_cost = area_cost;
                newTarget = Point(x,y);
            }
            else
                break;

        }
    }
    path[1] = Point(int(newTarget.x/sc2), int(newTarget.y/sc2));
}


std::vector<Point> SpecificWorker::create_final_path(std::vector<Point> path, float s)
{
	std::vector<Point> finalPath;
	Point iniLine = path[0];
	Point endLine = path[1];
	int i_end = 1;
	finalPath.push_back(Point(int(iniLine.x*s), int(iniLine.y*s)));
	finalPath.push_back(Point(int(endLine.x*s), int(endLine.y*s)));
	for (uint i=2; i < path.size(); i++)
	{
		Point newNode = path[i];
		Point v1 = newNode - iniLine;
		Point v2 = newNode - endLine;
		float collinearity = fabs((float)v1.dot(v2)/((float) norm(v1)*norm(v2)));
		if (collinearity > 0.98)
		{
			finalPath.erase(finalPath.begin()+i_end);
		}
		else
		{
			iniLine = endLine;
		}
		endLine = newNode;
		finalPath.push_back(Point(int(newNode.x*s), int(newNode.y*s)));
		i_end = finalPath.size()-1;
	}

	return finalPath;

}

float SpecificWorker::grid_cost(Mat grid, Point p)
{
	float cost = 1. - grid.at<float>(p);
	cost = 30.*cost;
	return cost;
}


std::vector<nodeType> SpecificWorker::validActions(Mat grid, Point p, int grid_border)
{
	std::vector<nodeType> actions;
	nodeType node;
    float area_cost;

	// std::random_shuffle(neighList.begin(), neighList.end());

	float current_value = grid.at<float>(p);
	
	for (int i=0; i<int(neighList.size()); i++)
	{
// 		int n = rand() % neighList.size();
		int n = i;
		node.p = Point(p.x + neighList[n].x, p.y + neighList[n].y);
        if (grid_border<=node.p.x && node.p.x<grid.cols-grid_border && grid_border<=node.p.y && node.p.y<grid.rows-grid_border)
		{
            area_cost = compute_area_cost(grid, node.p, grid_border*2);

            float explored_value = grid.at<float>(node.p);
            if (explored_value >= 0.4 or current_value < explored_value)
            {
                float dist = norm(neighList[n]);
                node.cost = dist*(1+area_cost*1.5);//grid_cost(grid, node.p);
                actions.push_back(node);
            }


		}
	}

	return actions;
}


float SpecificWorker::h_function(Point p1, Point p2)
{
	Point dif = p1 - p2;
	return norm(dif);
}

std::vector<Point> SpecificWorker::path_planning(Mat grid, Point start, Point goal, int grid_border)
{
	std::vector<Point> path;

	nodeType start_node;

	start_node.cost = 0;
	start_node.p = start;
	auto cmp = [](nodeType left, nodeType right) { return left.cost > right.cost; };

	std::priority_queue<nodeType, std::vector<nodeType>, decltype(cmp)> queue(cmp);
	queue.push(start_node);

	std::set<int> visited;

	std::map<int, Point> branch;
	bool found = false;

	while (!queue.empty())
	{
		nodeType item = queue.top();
		queue.pop();
		float current_cost = item.cost;
		Point current_node = item.p;
//        qDebug()<<"current node"<<current_node.x<<current_node.y<<"goal"<<goal.x<<goal.y;
		if (current_node == goal)
		{
			found = true;
			break;
		}
		else
		{
            std::vector<nodeType> actions = validActions(grid, current_node, grid_border);
            //std::cout << "Valid actions size "<<actions.size()<<std::endl;
			for (nodeType a: actions)
			{
				nodeType next_node;
				next_node.p = a.p;
                next_node.cost = current_cost + a.cost;// + h_function(next_node.p, goal);
				int linear_pos = next_node.p.y*grid.cols+next_node.p.x;
				if (visited.find(linear_pos)==visited.end())
				{
					visited.insert(next_node.p.y*grid.cols+next_node.p.x);
					queue.push(next_node);
					branch[linear_pos] = current_node;
				}
			}
		}
	}

	if (found)
	{
        path.insert(path.begin(),goal);
		int cur_linear_pos = goal.y*grid.cols+goal.x;
        if (not branch.empty())
        {
            while (branch[cur_linear_pos] != start)
            {
                Point next_p = branch[cur_linear_pos];
                path.insert(path.begin(),next_p);
                cur_linear_pos = next_p.y*grid.cols+next_p.x;
            }
            path.insert(path.begin(),branch[cur_linear_pos]);
        }
	}

	return path;
}

float sign(float num)
{
	if(num<0)
		return -1.;
	return 1.;
}

void SpecificWorker::follow_path(std::vector<Point> path)
{
	float adv=0, rot=0;
	float sideways = 0;

    float maxRot = M_PI/10.;
    float maxAdv = 0.2;


	Point vP = path[1]-path[0];
	float dist = norm(vP);

	uint next = 1;

    while (dist<=20 and next<path.size())
	{
		next++;
		vP = path[next]-path[0];
		dist = norm(vP);
	}

	if (next<path.size())
	{
        // vP = path[path.size()-1]-path[0];

        float target_orientation = atan2(vP.x, -vP.y);
        float dif_orientation = -atan2(sin(target_orientation-robot_angle), cos(target_orientation-robot_angle));

        // qDebug()<<target_orientation<<robot_angle<<dif_orientation;

        // float target_orientation = atan2(vP.x, -vP.y);

        // float dif_orientation = fabs(atan2(sin(target_orientation-robot_angle), cos(target_orientation-robot_angle)));


        // float dRot = atan2(sin(target_orientation-robot_angle), cos(target_orientation-robot_angle)); //*180./M_PI;

        if(fabs(dif_orientation)>0.4)
            rot = dif_orientation;
        else
            rot = 0;


        if(fabs(dif_orientation)>M_PI/4)
        {
            rot = maxRot*sign(dif_orientation);
            // qDebug()<<"rotation only"<<rot<<prev_rot;
        }
        else
        {
            if(fabs(dif_orientation)>0.4)
            {
                
                rot = dif_orientation;
                if(dist<50.)// && dist>200)
                    adv = dist/500.;
                if(fabs(rot)>maxRot)
                {
                    rot = maxRot*sign(dif_orientation);
                    adv = adv/(dif_orientation / rot);
                }

            }
            else
            {
                if(dist>20.)
                    adv = maxAdv;

            }
        }

        // if(sign(rot)!=sign(prev_rot))
        //     rot = rot+prev_rot;

        prev_rot = rot;
        prev_adv = adv;

	}
    
    omnirobot_proxy->setSpeedBase(adv, sideways, rot);
}


void SpecificWorker::add_objects_to_grid(Mat& grid)
{
    std::vector<BBType> local_BB_list;

    mtx_objects.lock();
    local_BB_list = objectBB_list;
    mtx_objects.unlock();

    Mat grid_gray;

    grid.convertTo(grid_gray, CV_8U, 255);
    for (auto o: local_BB_list)
    {
        std::vector< std::vector<Point> > poly;
        Point p;

        poly.resize(1);
        for(uint i = 0; i < o.size(); i++)
        {
            p.x = int((o[i].x*grid.cols)/SOCNAV_AREA_WIDTH + grid.cols/2);
            p.y = int(-(o[i].y*grid.rows)/SOCNAV_AREA_WIDTH + grid.rows/2);
            poly[0].push_back(p);
        }

        fillPoly(grid_gray, poly, 0, LINE_AA);
    }
    grid_gray.convertTo(grid, CV_32F, 1./255.);

}

void SpecificWorker::add_walls_to_grid(Mat& _grid)
{
    std::vector<wallType> local_wall_list;

    mtx_walls.lock();
    local_wall_list = wall_list;
    mtx_walls.unlock();

    Mat grid_gray;

    _grid.convertTo(grid_gray, CV_8U, 255);
    for (auto w: local_wall_list)
    {
        Point p1, p2;

        p1.x = int((w.p1.x*_grid.cols)/SOCNAV_AREA_WIDTH + _grid.cols/2);
        p1.y = int(-(w.p1.y*_grid.rows)/SOCNAV_AREA_WIDTH + _grid.rows/2);
        p2.x = int((w.p2.x*_grid.cols)/SOCNAV_AREA_WIDTH + _grid.cols/2);
        p2.y = int(-(w.p2.y*_grid.rows)/SOCNAV_AREA_WIDTH + _grid.rows/2);

        line(grid_gray, p1, p2, 0, 15, LINE_AA);
    }
    grid_gray.convertTo(_grid, CV_32F, 1./255.);

}

float SpecificWorker::compute_area_cost(Mat& _grid, Point center, int w)
{

    double min, max;
    Rect area = Rect(center.x-w/2, center.y-w/2, w, w);
    Mat im_area;
    im_area = 1. - _grid(area);    
    cv::minMaxLoc(im_area, &min, &max);
    return max;

}

float SpecificWorker::compute_min_area_cost(Mat& _grid, Point center, int w)
{

    double min, max;
    Rect area = Rect(center.x-w/2, center.y-w/2, w, w);
    Mat im_area;
    im_area = 1. - _grid(area);    
    cv::minMaxLoc(im_area, &min, &max);
    return min;

}

bool SpecificWorker::check_free_area(Mat& _grid, Point center, int w, float limit)
{
    float sum_area;

    int x1, y1, x2, y2;

    x1 = center.x-w/2;
    x2 = center.x+w/2;
    y1 = center.y-w/2;
    y2 = center.y+w/2;
    if(x1<0) x1 = 0;
    if(x2>_grid.cols) x2 = grid.cols;
    if(y1<0) y1 = 0;
    if(y2>_grid.rows) y2 = grid.rows;

    Rect area = Rect(x1, y1, x2-x1, y2-y1);
    // Rect area = Rect(center.x-w/2, center.y-w/2, w, w);
    Mat im_area;
    threshold(_grid(area), im_area, limit, 1., cv::THRESH_BINARY_INV);
    sum_area = cv::sum(im_area)[0];
    if(sum_area<limit)
        return true;
    return false;
}


void SpecificWorker::redefine_target(int gx, int gy, int & new_gx, int & new_gy)
{
    int limit = GRID_BORDER;
    int cx = grid_big.cols/2;
    int cy = grid_big.rows/2;

    new_gx = cx;
    new_gy = cy;
    float new_dist = sqrt((float)((gx-cx)*(gx-cx)) + (float)((gy-cy)*(gy-cy)));
    Point p1, p2;

    float valid_limit = 0.5;

    if (abs(gx-cx) > abs(gy-cy))
    {
        if (gx > cx)
        {
            p1 = Point(cx,cy);
            p2 = Point(gx, gy);
        }
        else
        {
            p2 = Point(cx,cy);
            p1 = Point(gx, gy);
        }
        for (int x = p1.x; x < p2.x; x++)
        {
            int y = int((float)((x-p1.x)*(p2.y-p1.y))/(float)(p2.x-p1.x) + p1.y);
            if(x>=limit and x<grid_big.cols-limit and y>=limit and y<grid_big.rows-limit)
                if (check_free_area(grid_big, Point(x,y), limit*2, valid_limit))
                {
                    float cur_dist = sqrt((float)((x-gx)*(x-gx)) + (float)((y-gy)*(y-gy)));
                    if(cur_dist<new_dist)
                    {
                        new_gx = x;
                        new_gy = y;
                        new_dist = cur_dist;
                    }
                }
        }
    }
    else
    {
        if (gy > cy)
        {
            p1 = Point(cx,cy);
            p2 = Point(gx, gy);
        }
        else
        {
            p2 = Point(cx,cy);
            p1 = Point(gx, gy);
        }
        for (int y = p1.y; y < p2.y; y++)
        {
            int x = int((float)((y-p1.y)*(p2.x-p1.x))/(float)(p2.y-p1.y) + p1.x);
            if(x>=limit and x<grid_big.cols-limit and y>=limit and y<grid_big.rows-limit )
                if (check_free_area(grid_big, Point(x,y), limit, valid_limit))
                {
                    float cur_dist = sqrt((float)((x-gx)*(x-gx)) + (float)((y-gy)*(y-gy)));
                    if(cur_dist<new_dist)
                    {
                        new_gx = x;
                        new_gy = y;
                        new_dist = cur_dist;
                    }

                }
        }
    }

}


void SpecificWorker::ObjectDetector_gotobjects(const ObjectList &lst)
{

    std::vector<BBType> local_objectBB_list;
    BBType objectBB;
	for (auto o: lst)
	{
        if(o.id>=0)
        {
            objectBB.clear();
            objectBB.push_back(Point2f(o.bbx1-0.25, o.bby1-0.25));
            objectBB.push_back(Point2f(o.bbx2+0.25, o.bby1-0.25));
            objectBB.push_back(Point2f(o.bbx2+0.25, o.bby2+0.25));
            objectBB.push_back(Point2f(o.bbx1-0.25, o.bby2+0.25));

            for(int i=0; i<4; i++)
            {
                float x = (cos(o.angle)*objectBB[i].x + sin(o.angle)*objectBB[i].y + o.x)*100;
                float y = (-sin(o.angle)*objectBB[i].x + cos(o.angle)*objectBB[i].y + o.y)*100;
                objectBB[i].x = x;
                objectBB[i].y = y;
            }
            local_objectBB_list.push_back(objectBB);
        }
        else
        {
            if(o.id==-2)
            {
                if(WorldFR)
                {
                    this->robot_x = o.x;
                    this->robot_y = o.y;
                    this->robot_angle = o.angle;
                }
                else
                {
                    this->robot_angle = o.angle;
                }

            }
        }
	}
    mtx_objects.lock();
    objectBB_list = local_objectBB_list;
    mtx_objects.unlock();
}

void SpecificWorker::PeopleDetector_gotpeople(const PeopleList &lst)
{
//	for (auto p: lst)
//	{
//        if (p.id == -1)
//        {
//            this->robot_x = -100*p.x;
//            this->robot_y = -100*p.y;
//            this->robot_angle = p.angle;
//            r2w(0,0) = cos(-this->robot_angle);
//            r2w(0,1) = sin(-this->robot_angle);
//            r2w(0,2) = this->robot_x;
//            r2w(1,0) = -sin(-this->robot_angle);
//            r2w(1,1) = cos(-this->robot_angle);
//            r2w(1,2) = this->robot_y;
//            r2w(2,0) = 0;
//            r2w(2,1) = 0;
//            r2w(2,2) = 1;
//            cv::invert(r2w, w2r);
//            //std::cout << "robot pose "<< this->robot_x << " " << this->robot_y << " " << this->robot_angle << std::endl;
//        }
//	}

}

void SpecificWorker::WallDetector_gotwalls(const RoboCompWallDetector::WallList &lst)
{
    std::vector<wallType> local_wall_list;

    for (auto w: lst)
    {
        wallType wall;

        wall.p1 = Point2f(w.x1*100, w.y1*100);
        wall.p2 = Point2f(w.x2*100, w.y2*100);

        local_wall_list.push_back(wall);
    }

    mtx_walls.lock();
    wall_list = local_wall_list;
    mtx_walls.unlock();

}


void SpecificWorker::SNGNN2D_gotgrid(const SNGNN2DData &d)
{
	// Safe double-buffered data copying
	mtx_write.lock();
	this->data_write = d;
	mtx_read.lock();
	std::swap(this->data_write, this->data_read);
	mtx_read.unlock();
	mtx_write.unlock();
}


void SpecificWorker::GoalPublisher_goalupdated(const RoboCompGoalPublisher::GoalT &goal)
{
//    qDebug()<<"new goal"<<goal.x<<goal.y;
    mtx_goal.lock();
    target_x = goal.x;
    target_y = goal.y;
    mtx_goal.unlock();
}
