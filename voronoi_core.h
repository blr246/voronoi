#ifndef _HPS_VORONOI_VORONOI_CORE_H_
#define _HPS_VORONOI_VORONOI_CORE_H_
#include "geometry.h"
#include <vector>

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
  GenericStone() : player(-1), pos() {}
  GenericStone(const int player_, const Position& pos_)
    : player(player_),
      pos(pos_)
  {}
  /// <summary> Player id indicating player that placed the stone. </summary>
  int player;
  /// <summary> The stone position. </summary>
  Position pos;
};
}

/// <summary> A stone in the Voronoi game. </summary>
typedef detail::GenericStone<int> Stone;

/// <summary> A Voronoi game. </summary>
class Voronoi
{
public:
  /// <summary> A Voronoi game board size. </summary>
  typedef Vector2<int> BoardSize;
  /// <summary> A Voronoi game board. </summary>
  typedef AxisAlignedBox<int> Board;
  /// <summary> A list of player scores. </summary>
  typedef std::vector<float> ScoreList;
  /// <summary> A list of stones in play order. </summary>
  typedef std::vector<Stone> StoneList;
  /// <summary> A stone in normalized coordinates. </summary>
  typedef detail::GenericStone<float> StoneNormalized;

  /// <summary> Create a Voronoi game. </summary>
  Voronoi(const int players, const int stonesPerPlayer,
          const BoardSize& boardSize)
    : m_players(players),
      m_stonesPerPlayer(stonesPerPlayer),
      m_stonesPlayed(),
      m_stonesPlayedNorm(),
      m_board(Vector2<int>(0, 0), Vector2<int>(boardSize.x, boardSize.y)),
      m_scoreData()
  {}

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
  void Scores(ScoreList* scores) const;

  /// <summary> Get number of players. </summary>
  inline int NumPlayers() const
  {
    return m_players;
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

private:
  /// <summary> Data needed to score the game. </summary>
  struct ScoreData
  {
    void Reset()
    {
      candidateEdges.clear();
      candidateEdgeNormals.clear();
      edges.clear();
      vertices.clear();
    }
    std::vector<Line<float> > candidateEdges;
    std::vector<Line<float> > candidateEdgeNormals;
    std::vector<Line<float> > edges;
    std::vector<Vector2<float> > vertices;
  };

  /// <summary> Number of players. </summary>
  int m_players;
  /// <summary> Number of stones per player. </summary>
  int m_stonesPerPlayer;
  /// <summary> Stones played so far. </summary>
  StoneList m_stonesPlayed;
  /// <summary> Stones played so far in normalized coordinates. </summary>
  std::vector<StoneNormalized> m_stonesPlayedNorm;
  /// <summary> The game play area in world coordinates. </summary>
  Board m_board;
  /// <summary> Internal memory used to compute scores. </summary>
  mutable ScoreData m_scoreData;
};

}
using namespace voronoi;
}

#endif //_HPS_VORONOI_VORONOI_CORE_H_
