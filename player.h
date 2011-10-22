#ifndef _HPS_VORONOI_PLAYER_H_
#define _HPS_VORONOI_PLAYER_H_

#include "voronoi_core.h"

namespace hps
{
namespace voronoi
{
typedef Vector2<int> Position;

struct RandomPlayer{
  static void RandomPosition(Voronoi& game, Stone* move)
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

}
}
#endif //_HPS_VORONOI_PLAYER_H_
