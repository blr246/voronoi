#ifndef _HPS_VORONOI_PLAYER_H_
#define _HPS_VORONOI_PLAYER_H_

#include "voronoi_core.h"
#include <vector>
#include <algorithm>

namespace hps
{
namespace voronoi
{
typedef Vector2<int> Position;

struct RandomPlayer{
  static void RandomPosition(const Voronoi& game, Stone* move)
  {
    Position boardSize = game.GetBoardSize();
    move->pos.x = math::RandBound(boardSize.x);
    move->pos.y = math::RandBound(boardSize.y);
  }

  static void Play(Voronoi& game)
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


struct GreedyPlayer{
  static void Play(Voronoi& game)
  {
    std::cout << "Greedy player playing..." << std::endl;
    Tile::TileList tiles;
    Tile::UnplayedTiles(game, 20, &tiles);
    Tile::TileList::iterator bestTileIt;
    float bestScore = 0;// -infinity?
    Stone stone;
    stone.player = game.CurrentPlayer();

    for(Tile::TileList::iterator i = tiles.begin(); i < tiles.end(); i++)
    {
      std::cout << ".";
      stone.pos = i->center;
      game.Play(stone);
      float curScore = GameScore(game);
      game.Undo();
      if(curScore >= bestScore)
      {
        bestTileIt = i;
        bestScore = curScore;
      }
    }
    std::cout << std::endl;

    stone.pos = bestTileIt->center;
    game.Play(stone);
  }

  static float GameScore(const Voronoi& game)
  {
    Voronoi::StoneList playedStones= game.Played();
    int lastPlayer = playedStones.back().player;
    Voronoi::ScoreList scores;
    //game.Scores(&scores);
    NaiveScore(game, &scores);

    // This is where a greedy heuristic goes, we want score plus some notion of defensibility,
    // Defensibility should correspond to how many polygons you own or how spread out your area is.
    return scores[lastPlayer]; 
  }
};

}
}
#endif //_HPS_VORONOI_PLAYER_H_
