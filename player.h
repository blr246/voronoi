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

struct Player
{
  virtual void Play(Voronoi& game) = 0;
};

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
  GreedyPlayer(Voronoi& game_, int tilesPerSide)
    :tiles(),
    game(game_)
  {
    Tile::UnplayedTiles(game, tilesPerSide, &tiles);
  }


  void Play(Voronoi& game)
  {
    std::cout << "Greedy player playing..." << std::endl;
    Tile::TileList::iterator bestTileIt;
    float bestScore = 0;// -infinity?
    Stone stone;
    stone.player = game.CurrentPlayer();

    //Play in the center first
    if(game.Played().size() == 0)
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
    tiles.erase(bestTileIt); // Remove the played tile;
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

  Tile::TileList tiles;
  Voronoi& game;
};


struct DefensivePlayer
{
  
  void Play(Voronoi& game)
  {
    const Voronoi::StoneList stoneList = game.Played();
    int currentPlayer = game.CurrentPlayer();
    int stoneIdx = -1;
    
    Position pos = GetCenterOfLargestPolygon(stoneList,currentPlayer,&stoneIdx);
    
    assert(stoneIdx >=0 && stoneIdx < stoneList.size());
    Stone stone = stoneList.at(stoneIdx);
    stone.player = currentPlayer;
    stone.pos = pos;
    
    game.Play(stone);
  }
 
  static Position GetCenterOfLargestPolygon(const Voronoi::StoneList& played,
					    const int currentPlayer, int* stoneIdx)
  {
    ComputeIndexOfLargestAreaPolygon(played,currentPlayer,stoneIdx);
    assert(*stoneIdx >=0 && *stoneIdx < played.size());
    Stone stone = played.at(*stoneIdx);
    
    Position pos;
    ComputeCentroid(stone.vertices,&pos);
  
    return pos;
  }
  
  static void ComputeIndexOfLargestAreaPolygon(const Voronoi::StoneList& stoneList, 
					      const int currentPlayer, int* stoneIdx)
  {
    float largestArea = std::numeric_limits<float>::min();
    for(int i=0;i<stoneList.size();++i)
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
    for(;i<vertices.size()-1;++i)
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
    float area = static_cast<float>(sum)/2;
    return std::abs(area);
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

}
}
#endif //_HPS_VORONOI_PLAYER_H_
