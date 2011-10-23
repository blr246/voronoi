#ifndef _HPS_VORONOI_PLAYER_GTEST_H_
#define _HPS_VORONOI_PLAYER_GTEST_H_
#include "player.h"
#include "voronoi_core.h"
#include "gtest/gtest.h"
#include <math.h>

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

TEST(GreedyPlayer, TileCenters)
{
  
  enum { Players = 2, };
  enum { StonesPerPlayer = 10, };
  enum { BoardDim = 1000, };

  Voronoi game(Players, StonesPerPlayer, Voronoi::BoardSize(BoardDim, BoardDim));

  std::vector<Position> centers;
  GreedyPlayer::TileCenters(game, 1, &centers);

  ASSERT_EQ(static_cast<float>(centers.size()), pow(1.0f, 2.0f));
  EXPECT_EQ(centers[0].x, BoardDim/2);
  EXPECT_EQ(centers[0].y, BoardDim/2);

  centers.clear();
  GreedyPlayer::TileCenters(game, 4, &centers);

  ASSERT_EQ(static_cast<float>(centers.size()), pow(4.0f, 2.0f));
  EXPECT_EQ(centers[0].x, 125);
  EXPECT_EQ(centers[0].y, 125);
}

}

#endif //_HPS_VORONOI_PLAYER_H_
