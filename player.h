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

struct Tile
{
  typedef std::vector<Tile> TileList;

  Tile(Position center_, int x, int y)
    : center(center_),
      XEdgeLength(x),
      YEdgeLength(y)
  {};

  Position center;
  int XEdgeLength;
  int YEdgeLength;

  const bool operator()(const Position pos){ PositionIsWithin(pos); } // For remove_if

  const bool PositionIsWithin(const Position pos)
  {
    return (pos.x < center.x + XEdgeLength/2 && pos.x >= center.x - XEdgeLength/2 &&
        pos.y < center.y + YEdgeLength/2 && pos.y >= center.y - YEdgeLength/2);
  }

  static void Tiles(const Voronoi& game, const int tilesPerSide, TileList *tiles)
  {
    assert(tiles->size() == 0);
    assert(tilesPerSide > 0);
    Position boardSize = game.GetBoardSize();

    int xIncrement = boardSize.x/tilesPerSide;
    int yIncrement = boardSize.x/tilesPerSide;

    for(int x = xIncrement/2; x < boardSize.x; x+= xIncrement)
    {
      for(int y = yIncrement/2; y < boardSize.y; y+= yIncrement)
      {
        tiles->push_back(Tile(Position(x, y),xIncrement, yIncrement));
      }
    }
  }

  static void UnplayedTiles(const Voronoi& game, const int tilesPerSide, TileList *tiles)
  {
    Tiles(game, tilesPerSide, tiles);
    RemovePlayedTiles(game, tiles);
  }

  static void RemovePlayedTiles(const Voronoi& game, TileList *tiles)
  {
    Voronoi::StoneList playedStones = game.Played();
    
    for(unsigned int i = 0; i < tiles->size(); i++)
    {
      Tile c = (*tiles)[i];
      for(unsigned int j = 0; j < playedStones.size(); j++)
      {
        Position p = playedStones[j].pos;
        if(c.PositionIsWithin(p))
        {
          tiles->erase(tiles->begin() + i); //Remove tile from tile list;
          i--;
          break;
        }
      }
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
