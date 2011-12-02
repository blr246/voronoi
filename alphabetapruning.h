#ifndef _NO_TIPPING_GAME_ALPHABETAPRUNING_H_
#define _NO_TIPPING_GAME_ALPHABETAPRUNING_H_
#include "voronoi_core.h"
#include <omp.h>

namespace hps
{
namespace voronoi
{

struct AlphaBetaPruning
{
  typedef Voronoi::FloatType FloatType;

  /// <summary> Helper struct to pass for thread-level processing. </summary>
  struct ThreadParams
  {
    ThreadParams()
      : game(),
        depth(-1),
        maxDepth(-1),
        bestMinimax(0),
        bestPlyIdx(-1),
        victoryIsMine(NULL)
    {}

    Voronoi game;
    int depth;
    int maxDepth;
    FloatType bestMinimax;
    int bestPlyIdx;
    volatile bool* victoryIsMine;
  };

  /// <summary> The parallel minimax parameters. </summary>
  struct Params
  {
    Params()
      : maxDepth(8),
        depth(0),
        threadData()
    {}

    int maxDepth;
    int depth;
    std::vector<ThreadParams> threadData;
  };

  /// <summary> Run alpha-beta pruning to get a stone for the game. </summary>
  template <class PlyGenerationFunction, class BoardEvaulationFunction>
  static FloatType Run(Params* params,
                       Voronoi* game,
                       const PlyGenerationFunction& plyFunc,
                       const BoardEvaulationFunction* evalFunc,
                       Stone* stone)
  {
    assert(params && game && evalFunc && stone);

    int maxDepth = params->maxDepth;
    int& depth = params->depth;
    assert(maxDepth > 1);
    assert(depth < maxDepth);
//    std::cout << "AlphaBetaPruning::Run() : maxDepth = " << maxDepth
//              << "." << std::endl;
    ++depth;

    // Get the children of the current game.
    Voronoi::StoneList plys;
    plyFunc(*game, &plys);
    assert(!plys.empty());
    volatile bool victoryIsMine = false;
    FloatType minimax;
    {
      // Setup threads.
      std::vector<ThreadParams>& threadData = params->threadData;
      {
        const int numProcs = omp_get_num_procs();
        threadData.resize(numProcs);
        for (int threadIdx = 0; threadIdx < numProcs; ++threadIdx)
        {
          ThreadParams& threadParams = threadData[threadIdx];
          {
            threadParams.bestMinimax = std::numeric_limits<FloatType>::min();
            threadParams.bestPlyIdx = -1;
            threadParams.depth = depth;
            threadParams.maxDepth = maxDepth;
            threadParams.game = *game;
            threadParams.victoryIsMine = &victoryIsMine;
          }
        }
      }
      // Initialize alpha and beta for depth 1.
      const FloatType alpha = std::numeric_limits<FloatType>::min();
      const FloatType beta = std::numeric_limits<FloatType>::max();
      // Parallelize the first level.
#pragma omp parallel for schedule(dynamic, 1)
      for (int plyIdx = 0; plyIdx < static_cast<int>(plys.size()); ++plyIdx)
      {
        const int threadIdx = omp_get_thread_num();
        ThreadParams& threadParams = threadData[threadIdx];
        if (!victoryIsMine)
        {
          // Apply the ply for this game.
          const Stone& mkChildPly = plys[plyIdx];
          threadParams.game.Play(mkChildPly);
          // Run on the subtree.
          const FloatType minimax = RunThread(alpha, beta,
                                              plyFunc, evalFunc, &threadParams);
          // Undo the ply for the next worker.
          threadParams.game.Undo();
          // Collect best minimax for this thread.
          if ((-1 == threadParams.bestPlyIdx) ||
              (minimax > threadParams.bestMinimax))
          {
            threadParams.bestMinimax = minimax;
            threadParams.bestPlyIdx = plyIdx;
            if (std::numeric_limits<FloatType>::max() == minimax)
            {
//              std::cout << "Thread " << threadIdx << " found victoryIsMine on "
//                        << "ply " << plyIdx << " of " << plys.size()
//                        << "." << std::endl;
              victoryIsMine = true;
            }
          }
        }
      }
      // Gather best result from all threads.
      {
        int bestPlyIdx;
        GatherRunThreadResults<std::greater<FloatType> >(threadData,
                                                         &minimax, &bestPlyIdx);
        // Set MINIMax ply.
        *stone = plys[bestPlyIdx];
      }
    }

    --depth;
    if (std::numeric_limits<FloatType>::min() == minimax)
    {
//      std::cout << "No guaranteed victory." << std::endl;
      // Select ply based on heuristic.
      game->Play(plys.front());
      FloatType bestPlyScore = (*evalFunc)(*game);
      *stone = plys.front();
      game->Undo();
      for (Voronoi::StoneList::const_iterator testPly = plys.begin();
           testPly != plys.end();
           ++testPly)
      {
        game->Play(*testPly);
        FloatType plyScore = (*evalFunc)(*game);
        if (plyScore > bestPlyScore)
        {
          bestPlyScore = plyScore;
          *stone = *testPly;
        }
        game->Undo();
      }
    }
    return minimax;
  }

private:
  /// <summary> Helper to identify MAX based on depth. </summary>
  inline static bool IdentifyMax(const int depth)
  {
    return depth & 1;
  }

  template <typename MinimaxFunc>
  static void GatherRunThreadResults(const std::vector<ThreadParams>& data,
                                     FloatType* minimax, int* bestPlyIdx)
  {
    std::vector<ThreadParams>::const_iterator result = data.begin();
    *minimax = result->bestMinimax;
    *bestPlyIdx = result->bestPlyIdx;
    MinimaxFunc minimaxFunc;
    for (; result < data.end(); ++result)
    {
      if (minimaxFunc(result->bestMinimax, *minimax))
      {
        *minimax = result->bestMinimax;
        *bestPlyIdx = result->bestPlyIdx;
      }
    }
  }

  template <class MinimaxFunc,
            class PlyGenerationFunction, class BoardEvaulationFunction>
  static void ABPruningChildrenHelper(ThreadParams* params,
                                      Voronoi::StoneList::const_iterator testPly,
                                      Voronoi::StoneList::const_iterator endPly,
                                      const PlyGenerationFunction& plyFunc,
                                      const BoardEvaulationFunction* evalFunc,
                                      const FloatType* alpha,
                                      const FloatType* beta,
                                      FloatType* minimax)
  {
    assert(params && evalFunc);

    Voronoi* game = &params->game;
    volatile bool* victoryIsMine = params->victoryIsMine;
    // Score all plys to find minimax.
    MinimaxFunc minimaxFunc;
    for (; testPly != endPly; ++testPly)
    {
      if (*victoryIsMine)
      {
        //std::cout << "Got victory signal." << std::endl;
        *minimax = 0;
        break;
      }
      game->Play(*testPly);
      FloatType score = RunThread(*alpha, *beta, plyFunc, evalFunc, params);
      if (minimaxFunc(score, *minimax))
      {
        *minimax = score;
      }
      game->Undo();
      if (*alpha >= *beta)
      {
        break;
      }
    }
  }

  template <class PlyGenerationFunction, class BoardEvaulationFunction>
  static FloatType RunThread(const FloatType a,
                             const FloatType b,
                             const PlyGenerationFunction& plyFunc,
                             const BoardEvaulationFunction* evalFunc,
                             ThreadParams* params)
  {
    assert(params && evalFunc);

    int& depth = params->depth;
    const int& maxDepth = params->maxDepth;
    Voronoi* game = &params->game;
    assert(depth < maxDepth);
    ++depth;

    // Get the children of the current game.
    Voronoi::StoneList plys;
    plyFunc(*game, &plys);
    // Collect incoming a and b.
    FloatType alpha = a;
    FloatType beta = b;
    // Is this a leaf?
    FloatType minimax;
    assert(!plys.empty());
    // If depth bound reached, return score current game.
    if (maxDepth == depth)
    {
      minimax = (*evalFunc)(*game);
    }
    else
    {
      // Init score.
      Voronoi::StoneList::const_iterator testPly = plys.begin();
      {
        game->Play(*testPly);
        minimax = RunThread(alpha, beta, plyFunc, evalFunc, params);
        game->Undo();
      }
      // If I am MAX, then maximize my score.
      if (IdentifyMax(depth))
      {
        alpha = minimax;
        ABPruningChildrenHelper<std::greater<FloatType> >(params, ++testPly,
                                                          plys.end(),
                                                          plyFunc, evalFunc,
                                                          &alpha, &beta, &alpha);
        if (alpha >= beta)
        {
          minimax = beta;
        }
        else
        {
          minimax = alpha;
        }
      }
      // I am MIN. Minimize it.
      else
      {
        beta = minimax;
        ABPruningChildrenHelper<std::less<FloatType> >(params, ++testPly,
                                                       plys.end(),
                                                       plyFunc, evalFunc,
                                                       &alpha, &beta, &beta);
        if (alpha >= beta)
        {
          minimax = alpha;
        }
        else
        {
          minimax = beta;
        }
      }
    }
    --depth;
    return minimax;
  }
};

}
using namespace voronoi;
}

#endif //_NO_TIPPING_GAME_ALPHABETAPRUNING_H_
