/*
@Function:Astar Algorithm Implementation
@Time:2022-01-10
@Author:Tang Gu Jie
@E-Mail:2822902808@qq.com
*/
#pragma once
#ifndef __ASTAR__
#define __ASTAR__
#include <iostream>
#include <vector>
#include <string>
#include <map>
//命名空间 ASTAR
namespace ASTAR
{
	//节点结构体 
	struct Node
	{
		int SnodeX;            //节点X值
		int SnodeY;            //节点Y值
		int SparentX;          //父节点X值
		int SparentY;          //父节点Y值
		float Sgn;;            //节点的g值
		float Shn;             //节点的h值
		float Sfn;             //节点的f值
	};
	class CAstar
	{
	public:
		//路径类型的枚举类型，NOFINDPATHPOINT表示没找到路径点，FINDPATHPOINT找到路径点
		enum PathType {NOFINDPATHPOINT=0, FINDPATHPOINT =1};
		PathType m_noPathFlag;

	public:
		/*初始化函数*/
		CAstar(
			int _xStart,                               //起始点X值
			int _yStart,                               //起始点Y值
			int _xStop,                                //目标点X值
			int _yStop,                                //目标点Y值
			float _weightA,                            //权重a值
			float _weightB,                            //权重b值
			PathType _noPathFlag,                      //路径是否生成标志)
			std::string _distanceType                  //距离类型
		);
	    /*初始化地图*/
		void InitMap(std::vector<int> _mapData,int _width,int _height);
		/*生成路径点*/
		std::vector<std::pair<int, int>> PathPoint();
	private:
		/*向OpenList插入数据*/
		Node InsertOpen(int _xVal, int _yVal, int _parentXval, int _parentYval, float _hn, float _gn, float _fn);
		
		/*扩展一个节点的周围邻居*/
		std::vector<Node> ExpandArray(int _xNodeExpanded, int _yNodeExpanded, float _gNodeExpanded, int _xTarget, int _yTarget,
			std::vector<int> _mapData, std::vector<Node>_closeList);

		/*该节点在openList-->multimap中的索引*/
		float NodeIndex(std::multimap<float, Node> _openList, int _xNode, int _yNode);
	    
		/*该节点在closeList-->vector中的索引*/
		int CloseNodeIndex(std::vector<Node> _closeList, int _xNode, int _yNode);

		/*Astar的核心函数*/
		void AstarCoreFunction();

		/*获得A星算法的路径*/
		std::vector<std::pair<int, int>> FindPath(std::vector<Node>_closeList, int _xStart, int _yStart, int _xStop, int _yStop);
	private:
		std::multimap<float, Node> m_openList;      //openList列表,multimap类型（默认Key是从小到大排序）
		std::vector<Node> m_closeList;              //closeList列表，vector列表
		//std::vector<std::vector<int>> m_mapData;    //地图数据，双vector类型
		std::vector<int>m_mapData;                 //一维地图数据
		int m_width;                               //地图的宽度
		int m_height;                              //地图的高度
		float m_weightA;                           //权重a
		float m_weightB;                           //权重b
		int m_xStart;                              //起点x值
		int m_yStart;                              //起点y值
		int m_xStop;                               //终点x值
		int m_yStop;                               //终点y值

	};
}

#endif
