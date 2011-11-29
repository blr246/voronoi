#ifndef _HPS_VORONOI_PLAYER_GTEST_H_
#define _HPS_VORONOI_PLAYER_GTEST_H_
#include "player.h"
#include "voronoi_core.h"
#include "geometry.h"
#include "util.h"
#include "gtest/gtest.h"
#include <math.h>

namespace _hps_voronoi_player_gtest_h_
{
using namespace hps;

TEST(Parser,BoardStates)
{
  enum { BoardDim = 1000, };
  const std::string stateString =
    "GLOBALS\n"
    "Total Turns: 10\n"
    "Total Players: 2\n"
    "You are Player: 0\n"
    "BOARD STATE\n"
    "0: 300 400\n"
    "1: 400 500\n"
    "0: 600 700\n"
    "1: 800 900\n"
    "0: 450 600\n"
    "1: 900 678\n"
    "Enter New position \"X Y\":";
  Voronoi game;
  int myPlayer;
  Parser::Parse(stateString, Voronoi::BoardSize(BoardDim, BoardDim),
                &game, &myPlayer);
  ASSERT_EQ(6, game.Played().size());
  // "0: 300 400\n"
  Voronoi::StoneList::const_iterator stone = game.Played().begin();
  EXPECT_EQ(0, stone->player);
  EXPECT_EQ(Vector2<int>(300, 400), stone->pos);
  // "1: 400 500\n"
  ++stone;
  EXPECT_EQ(1, stone->player);
  EXPECT_EQ(Vector2<int>(400, 500), stone->pos);
  // "0: 600 700\n"
  ++stone;
  EXPECT_EQ(0, stone->player);
  EXPECT_EQ(Vector2<int>(600, 700), stone->pos);
  // "1: 800 900\n"
  ++stone;
  EXPECT_EQ(1, stone->player);
  EXPECT_EQ(Vector2<int>(800, 900), stone->pos);
  // "0: 450 600\n"
  ++stone;
  EXPECT_EQ(0, stone->player);
  EXPECT_EQ(Vector2<int>(450, 600), stone->pos);
  // "1: 900 678\n"
  ++stone;
  EXPECT_EQ(1, stone->player);
  EXPECT_EQ(Vector2<int>(900, 678), stone->pos);
}

TEST(RandomPlayer, Play)
{
  enum { Players = 2, };
  enum { BoardDim = 1000, };
  Voronoi game;
  game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));
  RandomPlayer p;
  
  EXPECT_EQ(0, game.Played().size());

  p.Play(game);

  EXPECT_EQ(1, game.Played().size());

  p.Play(game);
  EXPECT_EQ(2, game.Played().size());
}

//Tests for Defensive Player.
TEST(DefensivePlayer, ComputeCentroid)
{
  //Test Data.
  Stone::Vertices vertices;
  vertices.push_back(Position(300,400));
  vertices.push_back(Position(284,390));
  vertices.push_back(Position(934,79));
  vertices.push_back(Position(56,90));
  vertices.push_back(Position(80,64));

  Position pos;
  DefensivePlayer::ComputeCentroid(vertices,&pos);

  EXPECT_EQ(pos.x,330);
  EXPECT_EQ(pos.y,204);
  
}

TEST(DefensivePlayer,GetArea)
{
  //Test Data.
  Stone::Vertices vertices;
  vertices.push_back(Position(4,10));
  vertices.push_back(Position(9,7));
  vertices.push_back(Position(11,2));
  vertices.push_back(Position(2,2));

  float area = DefensivePlayer::GetArea(vertices);
  EXPECT_EQ(area,45.5);

  vertices.clear();
  vertices.push_back(Position(2,12));
  vertices.push_back(Position(500,8));
  vertices.push_back(Position(978,456));

  area = DefensivePlayer::GetArea(vertices);
  EXPECT_EQ(area,112508);

  vertices.clear();
  vertices.push_back(Position(20,11));
  vertices.push_back(Position(34,5));
  vertices.push_back(Position(145,4));

  area = DefensivePlayer::GetArea(vertices);
  EXPECT_EQ(area,326);
}

TEST(DefensivePlayer,ComputeIndexOfLargestAreaPolygon)
{
  Voronoi::StoneList stoneList;

  //Stone 1
  Stone stone1;
  stone1.vertices.push_back(Position(4,10));
  stone1.vertices.push_back(Position(9,7));
  stone1.vertices.push_back(Position(11,2));
  stone1.vertices.push_back(Position(2,2));
  stone1.player = 1;

  //Stone 2
  Stone stone2;
  stone2.vertices.push_back(Position(20,11));
  stone2.vertices.push_back(Position(34,5));
  stone2.vertices.push_back(Position(145,4));
  //  stone2.vertices.push_back(Position(2,2));
  stone2.player = 0;

  //Stone 3
  Stone stone3;
  stone3.vertices.push_back(Position(2,12));
  stone3.vertices.push_back(Position(500,8));
  stone3.vertices.push_back(Position(978,456));
  //  stone3.vertices.push_back(Position(2,2));
  stone3.player=1;

  stoneList.push_back(stone1);
  stoneList.push_back(stone2);
  stoneList.push_back(stone3);

  int currentPlayer = 0;
  int stoneIdx = -1;
  DefensivePlayer::ComputeIndexOfLargestAreaPolygon(stoneList,currentPlayer,&stoneIdx);
  EXPECT_EQ(stoneIdx,2);
}

TEST(DefensivePlayer,GetCenterOfLargestPolygon)
{
  Voronoi::StoneList stoneList;

  //Stone 1
  Stone stone1;
  stone1.vertices.push_back(Position(4,10));
  stone1.vertices.push_back(Position(9,7));
  stone1.vertices.push_back(Position(11,2));
  stone1.vertices.push_back(Position(2,2));
  stone1.player = 1;

  //Stone 2
  Stone stone2;
  stone2.vertices.push_back(Position(20,11));
  stone2.vertices.push_back(Position(34,5));
  stone2.vertices.push_back(Position(145,4));
  //  stone2.vertices.push_back(Position(2,2));
  stone2.player = 0;

  //Stone 3
  Stone stone3;
  stone3.vertices.push_back(Position(2,12));
  stone3.vertices.push_back(Position(500,8));
  stone3.vertices.push_back(Position(978,456));
  //  stone3.vertices.push_back(Position(2,2));
  stone3.player=1;

  stoneList.push_back(stone1);
  stoneList.push_back(stone2);
  stoneList.push_back(stone3);

  int currentPlayer = 0;
  int stoneIdx = -1;
  Position pos = DefensivePlayer::GetCenterOfLargestPolygon(stoneList,currentPlayer,&stoneIdx);
  EXPECT_EQ(stoneIdx,2);
  EXPECT_EQ(pos.x,493);
  EXPECT_EQ(pos.y,158);
}
//End Tests for Defensive Player.

TEST(Tile, Tiles)
{
  
  enum { Players = 2, };
  enum { BoardDim = 1000, };

  Voronoi game;
  game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));

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

TEST(Tile, UnplayedTiles)
{
  
  enum { Players = 2, };
  enum { BoardDim = 1000, };

  Voronoi game;
  game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));

  std::vector<Tile> tiles;
  Tile::UnplayedTiles(game, 4, &tiles);

  int startingSize = tiles.size();
  ASSERT_EQ(static_cast<float>(startingSize), pow(4.0f, 2.0f));

  RandomPlayer p;
  p.Play(game);

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

TEST(GreedyPlayer, VersusRandomPlayer)
{
  enum { Players = 2, };
  enum { StonesPerPlayer = 10, };
  enum { BoardDim = 1000, };
  enum { NumGames = 1};
  enum { GreedyPlayerTiles = 30};

  for(int i = 0; i < NumGames; i++)
  {
    Voronoi game;
    game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));
    GreedyPlayer gp(game, GreedyPlayerTiles);
    RandomPlayer rp;
    for(int i = 0; i < StonesPerPlayer; i++)
    {
      rp.Play(game);
      gp.Play(game);
    }
    Voronoi::ScoreList scores;
    game.Scores(&scores);
    float greedyScore = scores[1];
    float randomScore = scores[0];
    EXPECT_TRUE(greedyScore > randomScore);
  }
  for(int i = 0; i < NumGames; i++)
  {
    Voronoi game;
    game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));
    GreedyPlayer gp(game, GreedyPlayerTiles);
    RandomPlayer rp;
    for(int i = 0; i < StonesPerPlayer; i++)
    {
      gp.Play(game);
      rp.Play(game);
    }
    Voronoi::ScoreList scores;
    game.Scores(&scores);
    float greedyScore = scores[0];
    float randomScore = scores[1];
    EXPECT_TRUE(greedyScore > randomScore);
  }
}

/*
TEST(GreedyPlayer, VersusDefensivePlayer)
{
  enum { Players = 2, };
  enum { StonesPerPlayer = 10, };
  enum { BoardDim = 1000, };
  enum { NumGames = 1};
  enum { GreedyPlayerTiles = 30};
  
  for(int i = 0; i < NumGames; i++)
  {
    Voronoi game(Players, Voronoi::BoardSize(BoardDim, BoardDim));
    GreedyPlayer gp(game, GreedyPlayerTiles);
    DefensivePlayer dp;
    for(int i = 0; i < StonesPerPlayer; i++)
    {
      dp.Play(game);
      gp.Play(game);
    }
    Voronoi::ScoreList scores;
    game.Scores(&scores);
    float greedyScore = scores[1];
    float defensiveScore = scores[0];
    EXPECT_TRUE(greedyScore > defensiveScore);//Not sure what the actual result will be --Rafi
  }
  for(int i = 0; i < NumGames; i++)
  {
    Voronoi game(Players, Voronoi::BoardSize(BoardDim, BoardDim));
    GreedyPlayer gp(game, GreedyPlayerTiles);
    for(int i = 0; i < StonesPerPlayer; i++)
    {
      gp.Play(game);
      DefensivePlayer dp.Play(game);
    }
    Voronoi::ScoreList scores;
    game.Scores(&scores);
    float greedyScore = scores[0];
    float defensiveScore = scores[1];
    EXPECT_TRUE(greedyScore > defensiveScore);// Not sure what the actual result will be --Rafi
  }
}*/

}


#endif //_HPS_VORONOI_PLAYER_H_
