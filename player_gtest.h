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

  std::vector<Tile> tiles;
  GreedyPlayer::Tiles(game, 1, &tiles);

  ASSERT_EQ(static_cast<float>(tiles.size()), pow(1.0f, 2.0f));
  EXPECT_EQ(tiles[0].center.x, BoardDim/2);
  EXPECT_EQ(tiles[0].center.y, BoardDim/2);

  tiles.clear();
  GreedyPlayer::Tiles(game, 4, &tiles);

  ASSERT_EQ(static_cast<float>(tiles.size()), pow(4.0f, 2.0f));
  EXPECT_EQ(tiles[0].center.x, 125);
  EXPECT_EQ(tiles[0].center.y, 125);
}

TEST(Tile, PositionIsWithin)
{
  Tile one(Position(125, 125), 250, 250);

  EXPECT_TRUE(one.PositionIsWithin(Position(0,0)));
  EXPECT_TRUE(one.PositionIsWithin(Position(125,125)));
  EXPECT_FALSE(one.PositionIsWithin(Position(250,250)));
  EXPECT_FALSE(one.PositionIsWithin(Position(250,0)));
  EXPECT_FALSE(one.PositionIsWithin(Position(0,250)));

  Tile two(Position(375, 375), 250, 250);
  EXPECT_TRUE(two.PositionIsWithin(Position(250, 250)));
}

}

#endif //_HPS_VORONOI_PLAYER_H_
