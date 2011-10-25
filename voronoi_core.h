#ifndef _HPS_VORONOI_VORONOI_CORE_H_
#define _HPS_VORONOI_VORONOI_CORE_H_
#include "geometry.h"
#include <vector>
#include <limits>
#include <sstream>

namespace hps
{
namespace voronoi
{

namespace detail
{
/// <summary> A Voronoi game stone. </summary>
template <typename NumericType>
struct GenericStone
{
  typedef Vector2<NumericType> Position;
  typedef std::vector<Position> Vertices;
  GenericStone() : player(-1), pos() {}
  GenericStone(const int player_, const Position& pos_)
    : player(player_),
      pos(pos_)
  {}
  /// <summary> Player id indicating player that placed the stone. </summary>
  int player;
  /// <summary> The stone position. </summary>
  Position pos;
  /// <summary> The vertices of the polygon surrounding the Stone.</summary>
  Vertices vertices;
};
}

/// <summary> A stone in the Voronoi game. </summary>
typedef detail::GenericStone<int> Stone;

/// <summary> A Voronoi game. </summary>
class Voronoi
{
public:
  /// <summary> Control the precision of the Voronoi game. </summary>
  typedef float FloatType;
  /// <summary> A Voronoi game board size. </summary>
  typedef Vector2<int> BoardSize;
  /// <summary> A Voronoi game board. </summary>
  typedef AxisAlignedBox<int> Board;
  /// <summary> A Voronoi game board in normalized coordinates. </summary>
  typedef AxisAlignedBox<FloatType> BoardNorm;
  /// <summary> A list of player scores. </summary>
  typedef std::vector<FloatType> ScoreList;
  /// <summary> A list of stones in play order. </summary>
  typedef std::vector<Stone> StoneList;
  /// <summary> A stone in normalized coordinates. </summary>
  typedef detail::GenericStone<FloatType> StoneNormalized;
  /// <summary> A list of stones with normalized coordinates. </summary>
  typedef std::vector<StoneNormalized> StoneNormalizedList;

  /// <summary> Create a Voronoi game. </summary>
  Voronoi(const int players, const int stonesPerPlayer,
          const BoardSize& boardSize);

  /// <summary> Play a single stone. </summary>
  /// <remarks>
  ///   <para> Returns false if the stone is an invalid move. </para>
  /// </remarks>
  bool Play(const Stone& stone);
  /// <summary> Remove the last stone. </summary>
  void Undo();

  /// <summary> Access the list of stones played so far. </summary>
  inline const StoneList& Played() const
  {
    return m_stonesPlayed;
  }

  /// <summary> Compute the score for all players. </summary>
  bool Scores(ScoreList* scores) const;
  bool FortuneScores(ScoreList* scores) const;

  /// <summary> Get number of players. </summary>
  inline int NumPlayers() const
  {
    return m_players;
  }

  /// <summary> Get current player. </summary>
  inline int CurrentPlayer() const
  {
    StoneList played = Played();
    if(played.size() == 0)
    {
      return 0;
    }else{
      int lastPlayer = Played().back().player;
      int currentPlayer = (lastPlayer + 1) % m_players;
      assert(currentPlayer >= 0);
      assert(currentPlayer < m_players);
      return currentPlayer;
    }
  }

  /// <summary> Get initial number of stones per player. </summary>
  inline int StonesPerPlayer() const
  {
    return m_stonesPerPlayer;
  }

  inline BoardSize GetBoardSize() const
  {
    return m_board.maxs;
  }

  inline Board GetBoard() const
  {
    return m_board;
  }

  void InitBoard(std::string buffer, std::string start, std::string end);
  
  inline std::string Compute()
  {
    int x = m_stonesPlayed.back().pos.x;
    int y = m_stonesPlayed.back().pos.y;
    std::stringstream ss;
    ss << x << " " << y;
    return ss.str();
  }

private:
  /// <summary> Number of players. </summary>
  int m_players;
  /// <summary> Number of stones per player. </summary>
  int m_stonesPerPlayer;
  /// <summary> Stones played so far. </summary>
  StoneList m_stonesPlayed;
  /// <summary> Stones played so far in normalized coordinates. </summary>
  StoneNormalizedList m_stonesPlayedNorm;
  /// <summary> The game play area in world coordinates. </summary>
  Board m_board;
  /// <summary> The normalized game board. </summary>
  BoardNorm m_boardNorm;
};


struct Tile
{
  typedef Vector2<int> Position;
  typedef std::vector<Tile> TileList;

  Tile(Position center_, int x, int y)
    : center(center_),
      XEdgeLength(x),
      YEdgeLength(y)
  {}

  Position center;
  int XEdgeLength;
  int YEdgeLength;

  const bool operator()(const Position pos){ return PositionIsWithin(pos); } // For remove_if

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

  static void RemoveTilesContaining(Position pos, TileList *tiles)
  {
    
    for(unsigned int i = 0; i < tiles->size(); i++)
    {
      Tile c = (*tiles)[i];
      if(c.PositionIsWithin(pos))
      {
        tiles->erase(tiles->begin() + i); //Remove tile from tile list;
        i--;
        break;
      }
    }
  }
};


static int SquaredDistance(Vector2<int> p1, Vector2<int> p2)
{
	int x = std::abs(p2.x-p1.x);
	int y = std::abs(p2.y-p1.y);
	return x*x + y*y;
}
	
static void ScoreNearestStone(const Voronoi::StoneList& stoneList, const std::vector<Tile>& tileList, Voronoi::ScoreList* scores)
{
  int bestDistance = std::numeric_limits<int>::max();
  int playerIndex = -1;
  for(unsigned int i=0;i<tileList.size();++i)
  {
    Tile tile = tileList[i];
    for(unsigned int j=0;j<stoneList.size();++j)
    {
      Stone stone = stoneList[j];
      int distance = SquaredDistance(stone.pos,tile.center);
      if(bestDistance > distance)
      {
        bestDistance = distance;
        playerIndex = stone.player;
      }
    }
    assert(playerIndex >=0);
    assert(playerIndex < scores->size());
    scores->at(playerIndex) += 1.0f;
  }
}

void NaiveScore(const Voronoi& game, Voronoi::ScoreList* scores);

}
using namespace voronoi;
}

#endif //_HPS_VORONOI_VORONOI_CORE_H_
