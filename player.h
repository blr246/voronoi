#ifndef _HPS_VORONOI_PLAYER_H_
#define _HPS_VORONOI_PLAYER_H_

#include "voronoi_core.h"
#include <vector>
#include <algorithm>

namespace hps
{
namespace voronoi
{
typedef Vector2<int> Position;

struct RandomPlayer{
  static void RandomPosition(const Voronoi& game, Stone* move)
  {
    Position boardSize = game.GetBoardSize();
    move->pos.x = math::RandBound(boardSize.x);
    move->pos.y = math::RandBound(boardSize.y);
  }

  static void Play(Voronoi& game)
  {
    bool moveWasValid = false;
    Stone move;
    while(!moveWasValid){
      RandomPosition(game, &move);
      moveWasValid = game.Play(move);
    }
  }
};

struct Tile
{
  Tile(Position center_, int x, int y)
    : center(center_),
      XEdgeLength(x),
      YEdgeLength(y)
  {};

  Position center;
  int XEdgeLength;
  int YEdgeLength;

  const bool operator()(const Position pos){ PositionIsWithin(pos); } // For remove_if

  const bool PositionIsWithin(const Position pos)
  {
    return (pos.x < center.x + XEdgeLength/2 && pos.x >= center.x - XEdgeLength/2 &&
        pos.y < center.y + YEdgeLength/2 && pos.y >= center.y - YEdgeLength/2);
  }

};

struct GreedyPlayer{
  static void Tiles(const Voronoi& game, const int tilesPerSide, std::vector<Tile> *tiles)
  {
    assert(tiles->size() == 0);
    assert(tilesPerSide > 0);
    Position boardSize = game.GetBoardSize();

    int xIncrement = boardSize.x/tilesPerSide;
    int yIncrement = boardSize.x/tilesPerSide;

    for(int x = xIncrement/2; x < boardSize.x; x+= xIncrement)
    {
      for(int y = yIncrement/2; y < boardSize.y; y+= yIncrement)
      {
        tiles->push_back(Tile(Position(x, y),xIncrement, yIncrement));
      }
    }
  }

  static void UnplayedTileCenters(const Voronoi& game, const int tilesPerSide, std::vector<Tile> *tiles)
  {
    Tiles(game, tilesPerSide, tiles);
    Voronoi::BoardSize boardSize = game.GetBoardSize();
    Voronoi::StoneList playedStones = game.Played();
    int xRadius = boardSize.x/(tilesPerSide*2);
    int yRadius = boardSize.y/(tilesPerSide*2);
    
    for(unsigned int i = 0; i < tiles->size(); i++)
    {
      Tile c = (*tiles)[i];
      for(unsigned int j = 0; j < playedStones.size(); j++)
      {
        Position p = playedStones[j].pos;
        if(c.PositionIsWithin(p))
        {
          //Remove tile from tile list;
        }
      }
    }
    
  }
};

}
}
#endif //_HPS_VORONOI_PLAYER_H_
