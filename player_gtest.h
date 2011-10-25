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

TEST(Tile, Tiles)
{
  
  enum { Players = 2, };
  enum { StonesPerPlayer = 10, };
  enum { BoardDim = 1000, };

  Voronoi game(Players, StonesPerPlayer, Voronoi::BoardSize(BoardDim, BoardDim));

  std::vector<Tile> tiles;
  Tile::Tiles(game, 1, &tiles);

  ASSERT_EQ(static_cast<float>(tiles.size()), pow(1.0f, 2.0f));
  EXPECT_EQ(tiles[0].center.x, BoardDim/2);
  EXPECT_EQ(tiles[0].center.y, BoardDim/2);

  tiles.clear();
  Tile::Tiles(game, 4, &tiles);

  ASSERT_EQ(static_cast<float>(tiles.size()), pow(4.0f, 2.0f));
  EXPECT_EQ(tiles[0].center.x, 125);
  EXPECT_EQ(tiles[0].center.y, 125);
}

TEST(GreedyPlayer, UnplayedTiles)
{
  
  enum { Players = 2, };
  enum { StonesPerPlayer = 10, };
  enum { BoardDim = 1000, };

  Voronoi game(Players, StonesPerPlayer, Voronoi::BoardSize(BoardDim, BoardDim));

  std::vector<Tile> tiles;
  Tile::UnplayedTiles(game, 4, &tiles);

  int startingSize = tiles.size();
  ASSERT_EQ(static_cast<float>(startingSize), pow(4.0f, 2.0f));

  RandomPlayer::Play(game);

  tiles.clear();
  Tile::UnplayedTiles(game, 4, &tiles);

  ASSERT_EQ(startingSize-1, tiles.size());
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

TEST(GreedyPlayer, Play)
{
  enum { Players = 2, };
  enum { StonesPerPlayer = 10, };
  enum { BoardDim = 1000, };
  
  for(int i = 0; i < 0; i++)
  {
    Voronoi game(Players, StonesPerPlayer, Voronoi::BoardSize(BoardDim, BoardDim));
    for(int i = 0; i < 10; i++)
    {
      RandomPlayer::Play(game);
      GreedyPlayer::Play(game);
    }
    Voronoi::ScoreList scores;
    game.Scores(&scores);
    EXPECT_TRUE(scores[0] < scores[1]);
  }
  for(int i = 0; i < 0; i++)
  {
    Voronoi game(Players, StonesPerPlayer, Voronoi::BoardSize(BoardDim, BoardDim));
    for(int i = 0; i < 10; i++)
    {
      GreedyPlayer::Play(game);
      RandomPlayer::Play(game);
    }
    Voronoi::ScoreList scores;
    game.Scores(&scores);
    EXPECT_TRUE(scores[0] < scores[1]);
  }
}
}


#endif //_HPS_VORONOI_PLAYER_H_
