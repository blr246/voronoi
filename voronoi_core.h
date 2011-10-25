#ifndef _HPS_VORONOI_VORONOI_CORE_H_
#define _HPS_VORONOI_VORONOI_CORE_H_
#include "geometry.h"
#include <vector>
#include <limits>

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
  /// <summary> Control the precision of the Voronoi game. </summary>
  typedef float FloatType;
  /// <summary> A Voronoi game board size. </summary>
  typedef Vector2<int> BoardSize;
  /// <summary> A Voronoi game board. </summary>
  typedef AxisAlignedBox<int> Board;
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
  void Scores(ScoreList* scores) const;

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
      return (lastPlayer + 1) % m_players;
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

private:
  /// <summary> Data needed to score the game. </summary>
  struct ScoreData
  {
    ScoreData(const int players);
    inline void Reset()
    {
      candidateEdges.clear();
      vertices.clear();
      assert(4 == boardEdges.size());
    }
    std::vector<Line<FloatType> > candidateEdges;
    std::vector<Line<FloatType> > boardEdges;
    std::vector<Vector2<FloatType> > vertices;
  };

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
  /// <summary> Internal memory used to compute scores. </summary>
  mutable ScoreData m_scoreData;
};

static int SquaredDistance(Vector2<int> p1, Vector2<int> p2)
{
	int x = std::abs(p2.x-p1.x);
	int y = std::abs(p2.y-p1.y);
	return x*x + y*y;
}
	
static void ScoreNearestStone(const Voronoi::StoneList& stoneList, const AxisAlignedBox<int>& box, Voronoi::ScoreList* scores)
{
	for(int i=box.mins.x;i<box.maxs.x;++i)
	{
		for(int j=box.mins.y;box.maxs.y;++i)
		{	
			int bestDistance = std::numeric_limits<int>::max();
			int playerIndex = -1;
			for(int k=0;k<stoneList.size();++k)
			{
				Stone stone = stoneList[k];
				int distance = SquaredDistance(stone.pos,Vector2<int>(i,j));
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
}

static void NaiveScore(const Voronoi& game, Voronoi::ScoreList* scores)
{
  Voronoi::StoneList stones = game.Played();
  Voronoi::Board board = game.GetBoard();

  scores->clear();
  for(unsigned int i = 0; i < game.NumPlayers(); i++)
  {
    scores->push_back(0.0f);
  }

  ScoreNearestStone(stones, board, scores);
}


}
using namespace voronoi;
}

#endif //_HPS_VORONOI_VORONOI_CORE_H_
