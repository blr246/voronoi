#ifndef _HPS_VORONOI_PLAYER_GTEST_H_
#define _HPS_VORONOI_PLAYER_GTEST_H_
#include "player.h"
#include "voronoi_core.h"
#include "gtest/gtest.h"

namespace _hps_voronoi_player_gtest_h_
{
  using namespace hps::voronoi;

TEST(RandomPlayer, Play)
{
  enum { Players = 2, };
  enum { StonesPerPlayer = 10, };
  enum { BoardDim = 1000, };
  Voronoi game(Players, StonesPerPlayer, Voronoi::BoardSize(BoardDim, BoardDim));
  
  EXPECT_EQ(0, game.Played().size());

  RandomPlayer::Play(game);

  EXPECT_EQ(1, game.Played().size());

  RandomPlayer::Play(game);
  EXPECT_EQ(2, game.Played().size());
}

}

#endif //_HPS_VORONOI_PLAYER_H_
