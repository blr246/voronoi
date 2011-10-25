#include "player.h"
namespace hps
{
namespace voronoi
{

virtual void DefensivePlayer::Play(Voronoi& game)
{
  const Voronoi::StoneList stoneList = game.Played();
  int currentPlayer = game.CurrentPlayer();
  int stoneIdx = -1;
  
  Position pos = GetCenterOfLargestPolygon(stoneList,currentPlayer,&stoneIdx);
  
  assert(stoneIdx >=0 && stoneIdx < stoneList.size());
  Stone stone = stoneList.at(stoneIdx);
  stone.player = currentPlayer;
  stone.pos = pos;
  
  game.Play(stone);
}
  
Position DefensivePlayer::GetCenterOfLargestPolygon(const Voronoi::StoneList& played,const int currentPlayer, int* stoneIdx) const
{
  *stoneIdx = GetIndexOfLargestAreaPolygon(played,currentPlayer);
  assert(*stoneIdx >=0 && *stoneIdx < played.size());
  Stone stone = played.at(*stoneIdx);
  
  Position pos;
  ComputeCentroid(stone.vertices,&pos);
  
  return pos;
}
  
int DefensivePlayer::GetIndexOfLargestAreaPolygon(const Voronoi::StoneList& stoneList, const int currentPlayer) const
{
  return -1;
}

void DefensivePlayer::ComputeCentroid(const Stone::Vertices& vertices, Position* pos) const
{
  int numVertices = vertices.size();
  for(int i=0;i<numVertices;++i)
  {
    pos->x += vertices.at(i).x;
    pos->y += vertices.at(i).y;
  }
    pos->x = pos->x/numVertices;
    pos->y = pos->y/numVertices;
}
