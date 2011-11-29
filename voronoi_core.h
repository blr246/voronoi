#ifndef _HPS_VORONOI_VORONOI_CORE_H_
#define _HPS_VORONOI_VORONOI_CORE_H_
#include "geometry.h"
#include <vector>
#include <limits>
#include <sstream>
#include <cstdlib>
#include <cstddef>

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

  Voronoi();

  /// <summary> Initialize a Voronoi game. </summary>
  void Initialize(const int players, const BoardSize& boardSize);

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

  /// <summary> Get current player. </summary>
  inline int CurrentPlayer() const
  {
    StoneList played = Played();
    if(played.empty())
    {
      return 0;
    }
    else
    {
      int lastPlayer = Played().back().player;
      int currentPlayer = (lastPlayer + 1) % m_players;
      assert(currentPlayer >= 0);
      assert(currentPlayer < m_players);
      return currentPlayer;
    }
  }

  /// <summary> Get number of players. </summary>
  inline int NumPlayers() const
  {
    return m_players;
  }
  /// <summary> Access board size. </summary>
  inline const BoardSize& GetBoardSize() const
  {
    return m_board.maxs;
  }
  /// <summary> Access game board. </summary>
  inline const Board& GetBoard() const
  {
    return m_board;
  }
  /// <summary> Access last stone played. </summary>
  inline const Stone& LastStone() const
  {
    return m_stonesPlayed.back();
  }

private:
  /// <summary> Compute scores using Fortune's algorithm. </summary>
  /// <remarks>
  ///   <para> This method may fail due to the error sensitivity of the
  ///     geometric sweep objects.
  ///   </para>
  /// </remarks>
  bool FortuneScores(ScoreList* scores) const;

  /// <summary> Number of players. </summary>
  int m_players;
  /// <summary> Stones played so far. </summary>
  /// <remarks>
  ///   <para> Must be mutable since polygons updated suring scoring. </para>
  /// </remarks>
  mutable StoneList m_stonesPlayed;
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

void ScoreNearestStone(const Voronoi::StoneList& stoneList,
                       const std::vector<Tile>& tileList,
                       Voronoi::ScoreList* scores);

void NaiveScore(const Voronoi& game, Voronoi::ScoreList* scores);

}
using namespace voronoi;
}

#endif //_HPS_VORONOI_VORONOI_CORE_H_
