#ifndef _HPS_VORONOI_VORONOI_CORE_GTEST_H_
#define _HPS_VORONOI_VORONOI_CORE_GTEST_H_
#include "voronoi_core.h"
#include "gtest/gtest.h"
#include <numeric>

namespace _hps_voronoi_voronoi_core_gtest_h_ 
{
using namespace hps;

TEST(Stone, voronoi_core)
{
  Stone stone;
  {
    stone.color = 0;
    stone.pos = Stone::Position(0, 5);
  }
}

TEST(Voronoi, voronoi_core)
{
  enum { Players = 2, };
  enum { StonesPerPlayer = 10, };
  enum { BoardDim = 1000, };
  Voronoi game(Players, StonesPerPlayer, Voronoi::BoardSize(BoardDim, BoardDim));
  // Verify params.
  EXPECT_EQ(Players, game.NumPlayers());
  EXPECT_EQ(StonesPerPlayer, game.StonesPerPlayer());
  // Verify no plays.
  const Voronoi::StoneList& played = game.Played();
  EXPECT_EQ(0, played.size());
  // Verify intial score.
  Voronoi::ScoreList scores;
  game.Scores(&scores);
  EXPECT_EQ(Players, scores.size());
  {
    const float totalScore = std::accumulate(scores.begin(), scores.end(), 0.0f);
    EXPECT_EQ(0.0, totalScore);
  }
}

TEST(VoronoiScore, voronoi_core)
{
}

}

#endif //_HPS_VORONOI_VORONOI_CORE_GTEST_H_
