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
  using namespace hps::voronoi;

TEST(Parser,BoardStates)
{
  std::string buffer = "BOARD STATE\n0: 300 400\n1: 400 500\n0: 600 700\n1: 800 900\n0: 450 600\n1: 900 678\n\nEnter New position \"X Y\":";
  std::string end = "Enter New position \"X Y\":";
  std::string start = "BOARD STATE";
  Voronoi::StoneList stoneList;
  Parser::BoardStates(buffer,start, end, &stoneList);
  EXPECT_EQ(6,stoneList.size());
  EXPECT_EQ(800,stoneList.at(3).pos.x);
  EXPECT_EQ(900,stoneList.at(3).pos.y);
}

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

TEST(GreedyPlayer, VersusRandomPlayer)
{
  enum { Players = 2, };
  enum { StonesPerPlayer = 10, };
  enum { BoardDim = 1000, };
  enum { NumGames = 10};
  
  for(int i = 0; i < NumGames; i++)
  {
    Voronoi game(Players, StonesPerPlayer, Voronoi::BoardSize(BoardDim, BoardDim));
    for(int i = 0; i < StonesPerPlayer; i++)
    {
      RandomPlayer::Play(game);
      GreedyPlayer::Play(game);
    }
    Voronoi::ScoreList scores;
    //game.Scores(&scores);
    NaiveScore(game, &scores);
    float greedyScore = scores[1];
    float randomScore = scores[0];
    EXPECT_TRUE(greedyScore > randomScore);
  }
  for(int i = 0; i < NumGames; i++)
  {
    Voronoi game(Players, StonesPerPlayer, Voronoi::BoardSize(BoardDim, BoardDim));
    for(int i = 0; i < StonesPerPlayer; i++)
    {
      GreedyPlayer::Play(game);
      RandomPlayer::Play(game);
    }
    Voronoi::ScoreList scores;
    //game.Scores(&scores);
    NaiveScore(game, &scores);
    float greedyScore = scores[0];
    float randomScore = scores[1];
    EXPECT_TRUE(greedyScore > randomScore);
  }
}

TEST(GreedyPlayer, VersusDefensivePlayer)
{
  enum { Players = 2, };
  enum { StonesPerPlayer = 10, };
  enum { BoardDim = 1000, };
  enum { NumGames = 10};
  
  for(int i = 0; i < NumGames; i++)
  {
    Voronoi game(Players, StonesPerPlayer, Voronoi::BoardSize(BoardDim, BoardDim));
    for(int i = 0; i < StonesPerPlayer; i++)
    {
      DefensivePlayer::Play(game);
      GreedyPlayer::Play(game);
    }
    Voronoi::ScoreList scores;
    //game.Scores(&scores);
    NaiveScore(game, &scores);
    float greedyScore = scores[1];
    float defensiveScore = scores[0];
    EXPECT_TRUE(greedyScore > defensiveScore);//Not sure what the actual result will be --Rafi
  }
  for(int i = 0; i < NumGames; i++)
  {
    Voronoi game(Players, StonesPerPlayer, Voronoi::BoardSize(BoardDim, BoardDim));
    for(int i = 0; i < StonesPerPlayer; i++)
    {
      GreedyPlayer::Play(game);
      DefensivePlayer::Play(game);
    }
    Voronoi::ScoreList scores;
    //game.Scores(&scores);
    NaiveScore(game, &scores);
    float greedyScore = scores[0];
    float defensiveScore = scores[1];
    EXPECT_TRUE(greedyScore > defensiveScore);// Not sure what the actual result will be --Rafi
  }
}


}


#endif //_HPS_VORONOI_PLAYER_H_
