#ifndef _HPS_VORONOI_PLAYER_H_
#define _HPS_VORONOI_PLAYER_H_
#include "voronoi_core.h"
#include "alphabetapruning.h"
#include "rand_bound.h"
#include <vector>
#include <algorithm>
#include <functional>
#include <numeric>

namespace hps
{
namespace voronoi
{
typedef Vector2<int> Position;

struct Player
{
  virtual ~Player() {}
  virtual void Play(Voronoi& game) = 0;
};

struct RandomPlayer: public Player
{
  static void RandomPosition(const Voronoi& game, Stone* move)
  {
    Position boardSize = game.GetBoardSize();
    std::cout << "size: " << boardSize.x << " X " << boardSize.y <<std::endl;
    move->pos.x = math::RandBound(boardSize.x) + 1;
    std::cout << "x: " << move->pos.x << std::endl;
     move->pos.y = math::RandBound(boardSize.y) + 1;
     std::cout << "y: " << move->pos.y << std::endl;
  }

  void Play(Voronoi& game)
  {
    //std::cout << "Random player playing..." << std::endl();
    bool moveWasValid = false;
    Stone move;
    move.player = game.CurrentPlayer();
    while(!moveWasValid){
      RandomPosition(game, &move);
      moveWasValid = game.Play(move);
    }
  }
};


struct GreedyPlayer: public Player
{
  GreedyPlayer(Voronoi& game, int tilesPerSide)
    :tiles()
  {
    Tile::UnplayedTiles(game, tilesPerSide, &tiles);
  }


  void Play(Voronoi& game)
  {
    std::cout << "Greedy player playing..." << std::endl;
    Stone stone;
    stone.player = game.CurrentPlayer();

    //Play in the center first
    if(game.Played().empty())
    {
      Position p = game.GetBoardSize();
      stone.pos = Position(p.x/2, p.y/2);
      game.Play(stone);
      return;
    }

    //Don't consider tiles that have been played in.
    if(game.NumPlayers() == 2)
    {
      Position lastPlay = game.Played().back().pos;
      Tile::RemoveTilesContaining(lastPlay, &tiles);
    }else
    {
      std::cout << "Inefficient check of all tiles for plays" << std::endl;
      Tile::RemovePlayedTiles(game, &tiles);
    }

    int bestTileIt =-1;
    float bestScore = std::numeric_limits<float>::min();

    for(int i = 0; i < static_cast<int>(tiles.size()); i++)
    {
      //std::cout << ".";
      stone.pos = tiles.at(i).center;
      game.Play(stone);
      float curScore = GameScore(game);
      game.Undo();
      if(curScore > bestScore)
      {
        bestTileIt = i;
        bestScore = curScore;
      }
    }
    assert(bestTileIt >=0);
    assert(bestScore > 0);
    //std::cout << std::endl;

    stone.pos = tiles.at(bestTileIt).center;
    game.Play(stone);
    tiles.erase(tiles.begin()+bestTileIt); // Remove the played tile;
    std::cout << "Playing at: player: " << stone.player
              << ", x: " << stone.pos.x << ", y: " << stone.pos.y <<std::endl;
  }

  static float GameScore(const Voronoi& game)
  {
    Voronoi::StoneList playedStones= game.Played();
    int lastPlayer = playedStones.back().player;
    Voronoi::ScoreList scores;
    game.Scores(&scores);

    // This is where a greedy heuristic goes, we want score plus some notion of defensibility,
    // Defensibility should correspond to how many polygons you own or how spread out your area is.
    return scores[lastPlayer]; 
  }

  Tile::TileList tiles;
};


struct DefensivePlayer: public Player
{
  
  void Play(Voronoi& game)
  {
    const Voronoi::StoneList stoneList = game.Played();
    int currentPlayer = game.CurrentPlayer();
    int stoneIdx = -1;
    
    Position pos = GetCenterOfLargestPolygon(stoneList,currentPlayer,&stoneIdx);
    
    assert(stoneIdx >=0 && stoneIdx < static_cast<int>(stoneList.size()));
    Stone stone = stoneList.at(stoneIdx);
    stone.player = currentPlayer;
    stone.pos = pos;
    
    game.Play(stone);
  }
 
  static Position GetCenterOfLargestPolygon(const Voronoi::StoneList& played,
					    const int currentPlayer, int* stoneIdx)
  {
    ComputeIndexOfLargestAreaPolygon(played,currentPlayer,stoneIdx);
    assert(*stoneIdx >=0 && *stoneIdx < static_cast<int>(played.size()));
    Stone stone = played.at(*stoneIdx);
    
    Position pos;
    ComputeCentroid(stone.vertices,&pos);
  
    return pos;
  }
  
  static void ComputeIndexOfLargestAreaPolygon(const Voronoi::StoneList& stoneList, 
					      const int currentPlayer, int* stoneIdx)
  {
    float largestArea = std::numeric_limits<float>::min();
    for(int i=0;i<static_cast<int>(stoneList.size());++i)
    {
      Stone stone = stoneList.at(i);
      if(stone.player != currentPlayer)
      {
        float area = GetArea(stone.vertices);
        if(largestArea < area)
        {
          largestArea = area;
          *stoneIdx = i;
        }
      }
    }
  }
  
  static float GetArea(const Stone::Vertices& vertices)
  {
    int sum = 0;
    int i=0;
    for(;i<static_cast<int>(vertices.size()-1);++i)
    {
      Vector2<int> v1 = vertices.at(i);
      Vector2<int> v2 = vertices.at(i+1);
      //std::cout << "x1: " << v1.x << ", y2: " << v2.y << ", y1: " << v1.y << ", x2: " << v2.x <<std:: endl;
      sum += (v1.x*v2.y - v1.y*v2.x);
    }
    Vector2<int> v1 = vertices.at(i);
    Vector2<int> v2 = vertices.at(0);
    // std::cout << "x1: " << v1.x << ", y2: " << v2.y << ", y1: " << v1.y << ", x2: " << v2.x << std::endl;
    sum += (v1.x*v2.y - v1.y*v2.x);
    sum = std::abs(sum);
    float area = static_cast<float>(sum)/2;
    return area;
  }

  static void ComputeCentroid(const Stone::Vertices& vertices, Position* pos)
  {
    int numVertices = vertices.size();
    for(int i=0;i<numVertices;++i)
    {
      pos->x += vertices.at(i).x;
      pos->y += vertices.at(i).y;
    }
    pos->x = pos->x/numVertices;
    pos->y = pos->y/numVertices;
  }
};

/// <summary> A grid-based alpha beta pruning player. </summary>
struct AlphaBetaPlayer : public Player
{
  AlphaBetaPlayer(const int searchDepth_) : searchDepth(searchDepth_) {}
  inline static int RandPlusMinus()
  {
    return (RandBound(2) > 0) ? 1 : -1;
  }
  /// <summary> Generate a grid of possible stones. </summary>
  struct PlyGenerationFunction
  {
    PlyGenerationFunction(const int gridDim_) : gridDim(gridDim_) {}
    void operator()(const Voronoi& game, Voronoi::StoneList* stones) const
    {
      assert(stones);
      stones->clear();
      Tile::TileList tiles;
      Tile::Tiles(game, gridDim, &tiles);
      const int divX_2 = tiles.front().XEdgeLength / 2;
      const int divY_2 = tiles.front().YEdgeLength / 2;
      const Voronoi::StoneList::const_iterator playedBegin = game.Played().begin();
      const Voronoi::StoneList::const_iterator playedEnd = game.Played().end();
      // Generate gridDim x gridDim moves.
      for (int tileIdx = 0; tileIdx < static_cast<int>(tiles.size()); ++tileIdx)
      {
        // Don't move on an occupied spot. Use random offset from tile center.
        stones->push_back(Stone());
        Stone& stone = stones->back();
        stone.player = game.CurrentPlayer();
        bool posFound;
        int xPlay = tiles[tileIdx].center.x;
        int yPlay = tiles[tileIdx].center.y;
        do
        {
          posFound = true;
          int randOffsetX = RandPlusMinus() * RandBound(divX_2);
          int randOffsetY = RandPlusMinus() * RandBound(divY_2);
          stone.pos = Stone::Position(xPlay + randOffsetX, yPlay + randOffsetY);
          for (Voronoi::StoneList::const_iterator sPlay = playedBegin;
               sPlay != playedEnd;
               ++sPlay)
          {
            if (sPlay->pos == stone.pos)
            {
              posFound = false;
              break;
            }
          }
        } while (!posFound);
      }
    }
    int gridDim;
  };

  /// <summary> Compute the score for the current player. </summary>
  struct BoardEvaulationFunction
  {
    BoardEvaulationFunction(const int playerNum_) : playerNum(playerNum_) {}
    inline Voronoi::FloatType operator()(const Voronoi& game) const
    {
      Voronoi::ScoreList scores;
      game.Scores(&scores);
      const Voronoi::FloatType totalScore = std::accumulate(
        scores.begin(), scores.end(), static_cast<Voronoi::FloatType>(0));
      return scores[playerNum] / totalScore;
    }
    int playerNum;
  };

  virtual void Play(Voronoi& game)
  {
    typedef Voronoi::FloatType FloatType;

    std::cout << "Alphabeta player playing..." << std::endl;
    // Get the best move from alpha beta.
    enum { MaxOps = 512000, };
    enum { MaxGridDim = 128, };
    const int totalStones = game.StonesPerPlayer() * game.NumPlayers();
    const int stonesRemaining = totalStones -
                                static_cast<int>(game.Played().size());
    params.maxDepth = std::min(stonesRemaining, searchDepth);
    const float gridDimSq = pow(MaxOps, 1.0f / params.maxDepth);
    const int gridDim = std::min(std::min(static_cast<int>(sqrt(gridDimSq)),
                                          game.GetBoardSize().x / 3),
                                 static_cast<int>(MaxGridDim));
    std::cout << "params.maxDepth: " << params.maxDepth << std::endl;
    std::cout << "gridDim: " << gridDim << std::endl;
    Stone stone;
    BoardEvaulationFunction evalFunc(game.CurrentPlayer());
    const FloatType minimax = AlphaBetaPruning::Run(&params, &game,
                                                    PlyGenerationFunction(gridDim),
                                                    &evalFunc,
                                                    &stone);
    game.Play(stone);
    std::cout << "Alphabeta got minimax " << minimax << " on stone "
              << game.Played().size() << " of " << totalStones << std::endl;
    std::cout << "Playing at: player: " << stone.player
              << ", x: " << stone.pos.x << ", y: " << stone.pos.y <<std::endl;
  }

  int searchDepth;
  AlphaBetaPruning::Params params;
};

}
}
#endif //_HPS_VORONOI_PLAYER_H_
