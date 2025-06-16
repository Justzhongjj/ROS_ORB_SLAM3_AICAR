/*
@Function:Astar Algorithm Implementation
@Time:2022-01-10
@Author:Tang Gu Jie
@E-Mail:2822902808@qq.com
*/
#include "../include/grid_map_search/Astar.h"
#include <functional>
#include <math.h>

std::function<float(int, int, int, int)>Distance;

/***
 *@函数功能   计算两点间的欧式距离
 ------------------------------------------------
 *@参数       _xNode    节点的x值
 *@参数       _yNode    节点的y值
 *@参数       _xTarget  终点的y值
 *@参数       _yTarget  终点的y值
 ------------------------------------------------
 *@返回值     欧式距离值：根号√((x2-x1)^2+(y2-y1)^2)
 */
float EuclideanDistance(int _xNode, int _yNode, int _xTarget, int _yTarget)
{
	float d = sqrt(pow(_xNode - _xTarget, 2) + pow(_yNode - _yTarget, 2));
	return d;
}

/***
*@函数功能   计算两点间的曼哈顿距离
------------------------------------------------
*@参数       _xNode    节点的x值
*@参数       _yNode    节点的y值
*@参数       _xTarget  终点的y值
*@参数       _yTarget  终点的y值
------------------------------------------------
*@返回值    曼哈顿距离值：|(x2-x1)+(y2-y1)|
*/
float ManhattanDistance(int _xNode, int _yNode, int _xTarget, int _yTarget)
{
	float d = std::abs(_xNode - _xTarget) + std::abs(_yNode - _yTarget);
	return d;
}

ASTAR::CAstar::CAstar(
    int _xStart,                               //起始点X值
	int _yStart,                               //起始点Y值
	int _xStop,                                //目标点X值
	int _yStop,                                //目标点Y值
	float _weightA,                            //权重a值
	float _weightB,                            //权重b值
	PathType _noPathFlag,                      //路径是否生成标志)
	std::string _distanceType                 //距离类型
) :m_xStart(_xStart), m_yStart(_yStart),m_xStop(_xStop), m_yStop(_yStop), m_noPathFlag(_noPathFlag),m_weightA(_weightA),m_weightB(_weightB)
{
    if (_distanceType=="euclidean")
	{   
		std::cout<<"use euclidean distance"<<std::endl;
		Distance=EuclideanDistance;
	}
     else if (_distanceType=="manhattan")
	{
		std::cout<<"use manhattan distance"<<std::endl;
		Distance=ManhattanDistance;
	}
    std::cout<<"weight_a:"<<_weightA<<" weight_b:"<<_weightB<<std::endl;
}

/***
*@函数功能   初始化一维地图数据
------------------------------------------------
*@参数       _mapData  地图数据
*@参数       _width    地图的宽
*@参数       _height   地图的高
------------------------------------------------
*/
void ASTAR::CAstar::InitMap(std::vector<int> _mapData,int _width,int _height)
{
	m_mapData.swap(_mapData);
	m_width=_width;
	m_height=_height;
}

/***
 *@函数功能   向openList插入新的节点
 ------------------------------------------------
 *@参数       _xVal            当前节点的x值
 *@参数       _yVal            当前节点的y值
 *@参数       _parentXval      当前节点的父节点的x值
 *@参数       _parentYval      当前节点的父节点的y值
 *@参数       _hn              当前节点的h值（当前节点与终点的关系）
 *@参数       _gn              当前节点的g值（当前节点与上一个的关系）
 *@参数       _fn              当前节点的f值（f=g+h）
 ------------------------------------------------
 *@返回值     Node             赋有上述参数的节点
 */
ASTAR::Node ASTAR::CAstar::InsertOpen(int _xVal, int _yVal, int _parentXval, int _parentYval, float _hn, float _gn, float _fn)
{
	Node node;
	node.SnodeX = _xVal;
	node.SnodeY = _yVal;
	node.SparentX = _parentXval;
	node.SparentY = _parentYval;
	node.Shn = _hn;
	node.Sgn = _gn; 
	node.Sfn = _fn;
	return node;
}


/***
 *@函数功能   扩展一个节点的周围邻居节点
 ------------------------------------------------
 *@参数       _xNodeExpanded        要扩展邻居的节点x值
 *@参数       _yNodeExpanded        要扩展邻居的节点y值
 *@参数       _gNodeExpanded        要扩展邻居的节点g值
 *@参数       _xTarget              终点x值
 *@参数       _yTarget              终点y值
 *@参数       _mapData              地图数据
 *@参数       _closeList            关闭列表
 ------------------------------------------------
 *@返回值     none
 */
std::vector<ASTAR::Node> ASTAR::CAstar::ExpandArray(int _xNodeExpanded, int _yNodeExpanded, float _gNodeExpanded, int _xTarget, int _yTarget,std::vector<int> _mapData, std::vector<ASTAR::Node>_closeList)
{
	//节点的8领域，顺序为右下、下、左下、右、左、右上、上、左上
	int mapHeight = m_height;
	int mapWidth = m_width;
	Node node;
	std::vector<Node> nodeList;
	bool expandFlag = false;
	for (int k = 1; k >= -1; k--)
	{
		for (int j = 1; j >= -1; j--)
		{
			if (k != j || j != 0)
			{
				int sx = _xNodeExpanded + k;
				int sy = _yNodeExpanded + j;
				if ((sx >= 0 && sx < mapHeight) && (sy >= 0 && sy < mapWidth) && _mapData[sx*mapWidth+sy] != 100)
				{
					expandFlag = true;
					for (int i = 0; i < _closeList.size(); i++)
					{
						if (sx == _closeList[i].SnodeX && sy == _closeList[i].SnodeY)
						{
							expandFlag = false;
							break;
						}
					}
					if (expandFlag == true)
					{
						node.SnodeX = sx;
						node.SnodeY = sy;
						node.SparentX = -1;     //这个随便填
						node.SparentY = -1;     //这个随便填
						node.Sgn = (_gNodeExpanded + EuclideanDistance(_xNodeExpanded, _yNodeExpanded, sx, sy));
						node.Shn = (Distance(_xTarget, _yTarget, sx, sy));
						node.Sfn = m_weightA*node.Sgn + m_weightB*node.Shn;
						nodeList.emplace_back(node);
					}
				}
			}
		}
	}
	return nodeList;
}


/***
 *@函数功能   查询节点(x,y)在openList中的Key索引
 ------------------------------------------------
 *@参数       _openList      开启列表
 *@参数       _xNode         节点x值
 *@参数       _yNode         节点y值
 ------------------------------------------------
 *@返回值     key值索引，注意因为在multimap中，Key可能是重复的，-1.0表示没找到
 */
float ASTAR::CAstar::NodeIndex(std::multimap<float, Node> _openList, int _xNode, int _yNode)
{
	for (std::multimap<float, Node>::iterator it = _openList.begin(); it != _openList.end(); it++)
	{
		if (it->second.SnodeX == _xNode && it->second.SnodeY == _yNode)
		{
			return it->first;
		}
	}
	return -1.0;
}


/***
 *@函数功能   查询节点(x,y)在closeList中的索引
 ------------------------------------------------
 *@参数       _closeList        关闭列表
 *@参数       _xNode            节点x值
 *@参数       _yNode            节点y值
 ------------------------------------------------
 *@返回值     i表示节点在vector中的索引，-1表示没有找到
 */
int ASTAR::CAstar::CloseNodeIndex(std::vector<Node> _closeList, int _xNode, int _yNode)
{
	for (int i = 0; i < _closeList.size(); i++)
	{
		if (_closeList[i].SnodeX == _xNode &&  _closeList[i].SnodeY == _yNode)
		{
			return i;
		}
	}
	return -1;
}


/***
 *@函数功能   Astar的核心函数，所有的数据处理在这里完成
 ------------------------------------------------
 *@参数       无
 *@参数       无
 ------------------------------------------------
 *@返回值     无，其实有一个隐藏的输出closeList，最终函数完成后，会完善closeList，最后根据closeList来逆变路径点
 */
void ASTAR::CAstar::AstarCoreFunction()
{
	
	float goalDistance = EuclideanDistance(m_xStart, m_yStart, m_xStop, m_yStop);
	float pathCost =0.0;
	float f_ = m_weightA*goalDistance + m_weightB*pathCost;
	//将起点放入到openList中
	Node node = InsertOpen(m_xStart, m_yStart, m_xStart, m_yStart, goalDistance, pathCost, f_);
	m_openList.insert(std::make_pair(goalDistance, node));

	while (true)
	{
		//对应伪代码-->if the queue(openList) is empty, return False; break; 
		if (m_openList.size() == 0)
			break;

		//对应伪代码-->Remove the node "n" with the lowest f(n) from the priority queue.
		std::multimap<float, Node>::iterator pos = m_openList.begin();   //multimap键值默认是从小到大排布
		int xNodeExpanded = pos->second.SnodeX;
		int yNodeExpanded = pos->second.SnodeY;
		float gNodeExpanded = pos->second.Sgn;

		// 对应伪代码-->Mark node "n" as expanded
		Node closeNode = pos->second;
		m_closeList.emplace_back(closeNode);
		m_openList.erase(pos);

		//对应伪代码-->if the node "n" is the goal state, return TRUE; break; 
		if (xNodeExpanded == m_xStop && yNodeExpanded == m_yStop)
		{
			m_noPathFlag = FINDPATHPOINT;
			break;
		}
		//获得所有节点n的所有“可用”邻居集合
		std::vector<Node> nodeList = ExpandArray(xNodeExpanded, yNodeExpanded, gNodeExpanded, m_xStop, m_yStop, m_mapData, m_closeList);
		for (int i = 0; i < nodeList.size(); i++)
		{
			int xNode = nodeList[i].SnodeX;
			int yNode = nodeList[i].SnodeY;
			float nodeIndex = NodeIndex(m_openList, xNode, yNode);
			if (nodeIndex == -1.0)
			{
			    //对应伪代码-->If node m is not in openList, push node m into openList 
				float gn = gNodeExpanded + EuclideanDistance(xNode, yNode, xNodeExpanded, yNodeExpanded);
				float hn = Distance(xNode, yNode, m_xStop, m_yStop);
				float fn = m_weightA*gn+ m_weightB*hn;

				Node node_ = InsertOpen(xNode, yNode, xNodeExpanded, yNodeExpanded, hn, gn, fn);
				m_openList.insert(std::make_pair(fn, node_));
			}
			else
			{
				//对应伪代码-->If g(m)>g(n)+Cnm
				//lower_bound返回查找结果第一个迭代器,upper_bound返回最后一个查找结果的下一个位置的迭代器
				std::multimap<float, Node>::iterator indexLow = m_openList.lower_bound(nodeIndex);
				std::multimap<float, Node>::iterator indexUpper = m_openList.upper_bound(nodeIndex);
				indexUpper--;
				if (indexLow == indexUpper)
				{
					//表示没有重复的键值
					if (indexLow->second.Sgn > (gNodeExpanded + EuclideanDistance(xNode, yNode, xNodeExpanded, yNodeExpanded)))
					{
						indexLow->second.SparentX = xNodeExpanded;
						indexLow->second.SparentY = yNodeExpanded;
						indexLow->second.Sgn = gNodeExpanded + EuclideanDistance(xNode, yNode, xNodeExpanded, yNodeExpanded);
						indexLow->second.Sfn =  m_weightA*indexLow->second.Sgn + m_weightB*indexLow->second.Shn;
					}
				}
				else
				{
					//表示有重复的键值
					while (indexLow != indexUpper)
					{
						if (indexLow->second.SnodeX == xNode && indexLow->second.SnodeY == yNode)
							break;
						indexLow++;
					}

					if (indexLow->second.Sfn > (gNodeExpanded + EuclideanDistance(xNode, yNode, xNodeExpanded, yNodeExpanded)))
					{
						indexLow->second.SparentX = xNodeExpanded;
						indexLow->second.SparentY = yNodeExpanded;
						indexLow->second.Sgn = gNodeExpanded + EuclideanDistance(xNode, yNode, xNodeExpanded, yNodeExpanded);
						indexLow->second.Sfn = m_weightA*indexLow->second.Sgn + m_weightB*indexLow->second.Shn;
					}
				}

			}
		}
	}
}


/***
 *@函数功能   路径寻找函数
 ------------------------------------------------
 *@参数       _closeList       关闭列表
 *@参数       _xStart          起点X值
 *@参数       _yStart          起点Y值
 *@参数       _xStop           终点X值
 *@参数       _yStop           终点Y值
 ------------------------------------------------
 *@返回值     路径节点 类型[(first,second),(),()...]
 */
std::vector<std::pair<int, int>> ASTAR::CAstar::FindPath(std::vector<Node>_closeList, int _xStart, int _yStart, int _xStop, int _yStop)
{
	std::pair<int, int>path;
	std::vector<std::pair<int, int>>findPath;
	path.first = _xStop;
	path.second = _yStop;
	findPath.emplace_back(path);
	int index = CloseNodeIndex(_closeList, _xStop, _yStop);

	while (true)
	{
		if (_closeList[index].SparentX == _xStart && _closeList[index].SparentY == _yStart)
		{
			break;
		}
		int nodeX = _closeList[index].SparentX;
		int nodeY = _closeList[index].SparentY;

		path.first = nodeX;
		path.second = nodeY;
		findPath.emplace_back(path);

		index = CloseNodeIndex(_closeList, nodeX, nodeY);
	}
	return findPath;
}


/***
 *@函数功能   对外的A星算法的接口
 ------------------------------------------------
 *@参数       无
 ------------------------------------------------
 *@返回值     路径节点 类型[(first,second),(),()...]
 */
std::vector<std::pair<int, int>> ASTAR::CAstar::PathPoint()
{
	//AstarCoreFunction完善openList与closeList
	AstarCoreFunction();
	//FindPath通过closeList进行寻找路径点
	std::vector<std::pair<int, int> > pathPoint = FindPath(m_closeList, m_xStart, m_yStart, m_xStop, m_yStop);
	return pathPoint;
}
