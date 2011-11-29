#ifndef _HPS_VORONOI_VORONOI_CORE_GTEST_H_
#define _HPS_VORONOI_VORONOI_CORE_GTEST_H_
#include "voronoi_core.h"
#include "gtest/gtest.h"
#include <numeric>

namespace _hps_voronoi_voronoi_core_gtest_h_ 
{
using namespace hps;

typedef Voronoi::FloatType FloatType;

TEST(voronoi_core, Stone)
{
  Stone stone(0, Stone::Position(0, 5));
  EXPECT_EQ(0, stone.player);
  EXPECT_EQ(Stone::Position(0, 5), stone.pos);
}

TEST(voronoi_core, Voronoi)
{
  enum { Players = 2, };
  enum { BoardDim = 1000, };
  Voronoi game;
  game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));
  // Verify params.
  EXPECT_EQ(Players, game.NumPlayers());
  // Verify no plays.
  const Voronoi::StoneList& played = game.Played();
  EXPECT_EQ(0, played.size());
  // Verify intial score.
  Voronoi::ScoreList scores;
  game.Scores(&scores);
  EXPECT_EQ(Players, scores.size());
  {
    const FloatType totalScore = std::accumulate(scores.begin(), scores.end(),
                                                 static_cast<FloatType>(0));
    EXPECT_EQ(0.0, totalScore);
  }
}

TEST(voronoi_core, VoronoiScore)
{
  enum { Players = 2, };
  enum { BoardDim = 1000, };
  // Single stone.
  {
    Voronoi game;
    game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));
    // Play one stone. Should be score of 100%.
    const Stone playStone(0, Stone::Position(RandBound(BoardDim + 1),
                                             RandBound(BoardDim + 1)));
    game.Play(playStone);
    // Verify scoring.
    Voronoi::ScoreList scores;
    game.Scores(&scores);
    const FloatType expectScore = static_cast<FloatType>(BoardDim * BoardDim);
    EXPECT_EQ(scores.front(), expectScore);
  }
  // Corner combinations.
  {
    const FloatType expectScore = 0.5f * static_cast<FloatType>(BoardDim * BoardDim);
    // (0, 0) & (BoardDim, BoardDim)
    {
      Voronoi game;
      game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));
      game.Play(Stone(0, Stone::Position(0, 0)));
      game.Play(Stone(1, Stone::Position(BoardDim, BoardDim)));
      // Verify scoring.
      Voronoi::ScoreList scores;
      game.Scores(&scores);
      EXPECT_NEAR(scores[0], expectScore, 1.0f);
      EXPECT_NEAR(scores[1], expectScore, 1.0f);
    }
    // (0, BoardDim) & (BoardDim, 0)
    {
      Voronoi game;
      game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));
      game.Play(Stone(0, Stone::Position(0, BoardDim)));
      game.Play(Stone(1, Stone::Position(BoardDim, 0)));
      // Verify scoring.
      Voronoi::ScoreList scores;
      game.Scores(&scores);
      EXPECT_NEAR(scores[0], expectScore, 1.0f);
      EXPECT_NEAR(scores[1], expectScore, 1.0f);
    }
    // (0, 0) & (0, BoardDim)
    {
      Voronoi game;
      game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));
      game.Play(Stone(0, Stone::Position(0, 0)));
      game.Play(Stone(1, Stone::Position(0, BoardDim)));
      // Verify scoring.
      Voronoi::ScoreList scores;
      game.Scores(&scores);
      EXPECT_NEAR(scores[0], expectScore, 1.0f);
      EXPECT_NEAR(scores[1], expectScore, 1.0f);
    }
    // (BoardDim, 0) & (BoardDim, BoardDim)
    {
      Voronoi game;
      game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));
      game.Play(Stone(0, Stone::Position(BoardDim, 0)));
      game.Play(Stone(1, Stone::Position(BoardDim, BoardDim)));
      // Verify scoring.
      Voronoi::ScoreList scores;
      game.Scores(&scores);
      EXPECT_NEAR(scores[0], expectScore, 1.0f);
      EXPECT_NEAR(scores[1], expectScore, 1.0f);
    }
  }
  // Four points diagonal.
  {
    const FloatType expectScore = 0.5f * static_cast<FloatType>(BoardDim * BoardDim);
    Voronoi game;
    game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));
    game.Play(Stone(0, Stone::Position(0, 0)));
    game.Play(Stone(1, Stone::Position(BoardDim, 0)));
    game.Play(Stone(0, Stone::Position(BoardDim, BoardDim)));
    game.Play(Stone(1, Stone::Position(0, BoardDim)));
    // Verify scoring.
    Voronoi::ScoreList scores;
    game.Scores(&scores);
    EXPECT_NEAR(scores[0], expectScore, 1.0f);
    EXPECT_NEAR(scores[1], expectScore, 1.0f);
  }
  // Four points.
  {
    const FloatType expectScore = 0.5f * static_cast<FloatType>(BoardDim * BoardDim);
    Voronoi game;
    game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));
    game.Play(Stone(0, Stone::Position(0, 0)));
    game.Play(Stone(1, Stone::Position(0, BoardDim)));
    game.Play(Stone(0, Stone::Position(BoardDim, 0)));
    game.Play(Stone(1, Stone::Position(BoardDim,  BoardDim)));
    // Verify scoring.
    Voronoi::ScoreList scores;
    game.Scores(&scores);
    EXPECT_NEAR(scores[0], expectScore, 1.0f);
    EXPECT_NEAR(scores[1], expectScore, 1.0f);
  }
  // Four points quadrants.
  {
    const FloatType expectScore = 0.5f * static_cast<FloatType>(BoardDim * BoardDim);
    Voronoi game;
    game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));
    const int quaterDim = BoardDim / 4;
    game.Play(Stone(0, Stone::Position(quaterDim, quaterDim)));
    game.Play(Stone(1, Stone::Position(3 * quaterDim, quaterDim)));
    game.Play(Stone(0, Stone::Position(3 * quaterDim,  3 * quaterDim)));
    game.Play(Stone(1, Stone::Position(quaterDim,  3 * quaterDim)));
    // Verify scoring.
    Voronoi::ScoreList scores;
    game.Scores(&scores);
    EXPECT_NEAR(scores[0], expectScore, 1.0f);
    EXPECT_NEAR(scores[1], expectScore, 1.0f);
  }
  // Four points quadrants.
  {
    const FloatType expectScore = 0.5f * static_cast<FloatType>(BoardDim * BoardDim);
    Voronoi game;
    game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));
    const int eighthDim = BoardDim / 8;
    game.Play(Stone(0, Stone::Position(eighthDim, eighthDim)));
    game.Play(Stone(1, Stone::Position(BoardDim - eighthDim, BoardDim - eighthDim)));
    game.Play(Stone(0, Stone::Position(2 * eighthDim, 2 * eighthDim)));
    game.Play(Stone(1, Stone::Position(BoardDim - (2 * eighthDim), BoardDim - (2 * eighthDim))));
    // Verify scoring.
    Voronoi::ScoreList scores;
    game.Scores(&scores);
    EXPECT_NEAR(scores[0], expectScore, 1.0f);
    EXPECT_NEAR(scores[1], expectScore, 1.0f);
  }
  // Random cases. Make sure that the area sums to the expected total.
  {
    const float kExpectSuccessPct = 0.95f;
    enum { RandomIterations = 500, };
    const int kExpectScoreSuccessCount =
      static_cast<int>(RandomIterations * kExpectSuccessPct);
    const FloatType expectTotalScore = static_cast<FloatType>(BoardDim * BoardDim);
    int scoreSuccessCount = 0;
    for (int iteration = 0; iteration < RandomIterations; ++iteration)
    {
      enum { Plays = 3, };
      Voronoi game;
      game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));
      // Play random stones.
      for (int playIdx = 0; playIdx < Plays; ++playIdx)
      {
        const int player = playIdx % Players;
        game.Play(Stone(player, Stone::Position(RandBound(BoardDim + 1),
                                                RandBound(BoardDim + 1))));
      }
      // Verify scoring.
      Voronoi::ScoreList scores;
      const bool fortuneSuccess = game.Scores(&scores);
      const FloatType totalScore = std::accumulate(scores.begin(), scores.end(),
                                                   static_cast<FloatType>(0));
      if (fortuneSuccess)
      {
        EXPECT_NEAR(expectTotalScore, totalScore, totalScore * 0.05f)
          << "Fortune scores did not sum properly on iteration " << iteration << " of "
          << RandomIterations << ".";
        ++scoreSuccessCount;
      }
      else
      {
        EXPECT_NEAR(expectTotalScore, totalScore, totalScore * 0.01f)
          << "Naieve scores did not sum properly on iteration " << iteration << " of "
          << RandomIterations << ".";
      }
    }
    std::cout << "Scored " << scoreSuccessCount << " of " << RandomIterations
              << " iterations successfully." << std::endl;
    EXPECT_GE(scoreSuccessCount, kExpectScoreSuccessCount);
  }
  // Random cases. Make sure that the area sums to the expected total.
  {
    const float kExpectSuccessPct = 0.95f;
    enum { RandomIterations = 500, };
    const int kExpectScoreSuccessCount =
      static_cast<int>(RandomIterations * kExpectSuccessPct);
    const FloatType expectTotalScore = static_cast<FloatType>(BoardDim * BoardDim);
    int scoreSuccessCount = 0;
    for (int iteration = 0; iteration < RandomIterations; ++iteration)
    {
      enum { Plays = 10, };
      Voronoi game;
      game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));
      // Play random stones.
      for (int playIdx = 0; playIdx < Plays; ++playIdx)
      {
        const int player = playIdx % Players;
        while (!game.Play(Stone(player, Stone::Position(RandBound(BoardDim + 1),
                                                        RandBound(BoardDim + 1)))));
      }
      // Verify scoring.
      Voronoi::ScoreList scores;
      const bool fortuneSuccess = game.Scores(&scores);
      const FloatType totalScore = std::accumulate(scores.begin(), scores.end(),
                                                   static_cast<FloatType>(0));
      if (fortuneSuccess)
      {
        const FloatType totalScore = std::accumulate(scores.begin(), scores.end(),
                                                     static_cast<FloatType>(0));
        EXPECT_NEAR(expectTotalScore, totalScore, totalScore * 0.05f)
          << "Fortune scores did not sum properly on iteration " << iteration << " of "
          << RandomIterations << ".";
        ++scoreSuccessCount;
      }
      else
      {
        EXPECT_NEAR(expectTotalScore, totalScore, totalScore * 0.01f)
          << "Naieve scores did not sum properly on iteration " << iteration << " of "
          << RandomIterations << ".";
      }
    }
    std::cout << "Scored " << scoreSuccessCount << " of " << RandomIterations
              << " iterations successfully." << std::endl;
    EXPECT_GE(scoreSuccessCount, kExpectScoreSuccessCount);
  }
  // Random cases. Make sure that the area sums to the expected total.
  {
    const float kExpectSuccessPct = 0.95f;
    enum { RandomIterations = 500, };
    const int kExpectScoreSuccessCount =
      static_cast<int>(RandomIterations * kExpectSuccessPct);
    const FloatType expectTotalScore = static_cast<FloatType>(BoardDim * BoardDim);
    int scoreSuccessCount = 0;
    for (int iteration = 0; iteration < RandomIterations; ++iteration)
    {
      enum { Plays = 20, };
      Voronoi game;
      game.Initialize(Players, Voronoi::BoardSize(BoardDim, BoardDim));
      // Play random stones.
      for (int playIdx = 0; playIdx < Plays; ++playIdx)
      {
        const int player = playIdx % Players;
        while (!game.Play(Stone(player, Stone::Position(RandBound(BoardDim + 1),
                                                        RandBound(BoardDim + 1)))));
      }
      // Verify scoring.
      Voronoi::ScoreList scores;
      const bool fortuneSuccess = game.Scores(&scores);
      const FloatType totalScore = std::accumulate(scores.begin(), scores.end(),
                                                   static_cast<FloatType>(0));
      if (fortuneSuccess)
      {
        const FloatType totalScore = std::accumulate(scores.begin(), scores.end(),
                                                     static_cast<FloatType>(0));
        EXPECT_NEAR(expectTotalScore, totalScore, totalScore * 0.05f)
          << "Fortune scores did not sum properly on iteration " << iteration << " of "
          << RandomIterations << ".";
        ++scoreSuccessCount;
      }
      else
      {
        EXPECT_NEAR(expectTotalScore, totalScore, totalScore * 0.01f)
          << "Naieve scores did not sum properly on iteration " << iteration << " of "
          << RandomIterations << ".";
      }
    }
    std::cout << "Scored " << scoreSuccessCount << " of " << RandomIterations
              << " iterations successfully." << std::endl;
    EXPECT_GE(scoreSuccessCount, kExpectScoreSuccessCount);
  }
}

}

#endif //_HPS_VORONOI_VORONOI_CORE_GTEST_H_
