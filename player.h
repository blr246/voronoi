#ifndef _HPS_VORONOI_PLAYER_H_
#define _HPS_VORONOI_PLAYER_H_

#include "voronoi_core.h"
#include <vector>

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

struct GreedyPlayer{
  static void TileCenters(const Voronoi& game, const int tilesPerSide, std::vector<Position> *centers)
  {
    assert(centers->size() == 0);
    Position boardSize = game.GetBoardSize();

    int xIncrement = boardSize.x/tilesPerSide;
    int yIncrement = boardSize.x/tilesPerSide;

    for(int x = xIncrement/2; x < boardSize.x; x+= xIncrement)
    {
      for(int y = yIncrement/2; y < boardSize.y; y+= yIncrement)
      {
        centers->push_back(Position(x, y));
      }
    }
    
    return;
  }
};

}
}
#endif //_HPS_VORONOI_PLAYER_H_
