#include <iostream>
#include <string>
#include <sstream>
#include "voronoi_core.h"
#include "geometry.h"

namespace hps
{
namespace voronoi
{

struct Parser
{
  static void BoardStates(std::string buffer,std::string start, 
		   std::string end, Voronoi::StoneList* stoneList)
  {
    stoneList->clear();
    size_t prev = 0;
    size_t next = buffer.find_first_of("\n");
    buffer = buffer.substr(buffer.find(start)+start.length());
    while(next!=std::string::npos)
    {
      std::stringstream ss;
      int player;
      int x;
      int y;
      std::string sep;
      std::string buf = buffer.substr(prev,next-prev);
      if(buf.length()>0)
      {
	ss << buf;
	ss >> player >> sep >> x >> y;
	Vector2<int> pos(x,y); 
	Stone stone(player,pos);
	std::cout << "player: " << player << ", x: " << x << ", y: " << y <<std::endl;
	stoneList->push_back(stone);
      }
      prev = next+1;
      next = buffer.find_first_of("\n",next+1);
    }
  }
};

}
}
