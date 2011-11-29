#include "voronoi_core.h"
#include "geometry.h"
#include <string>
#include <sstream>
#include <iostream>

namespace hps
{
namespace voronoi
{

/// <summary> Setup a voronoi game from a state string. </summmary>
struct Parser
{
  /// <summary> Beginning of game board data. </summary>
  inline static const char* StateStringBegin()
  {
    return "BOARD STATE";
  }
  /// <summary> End of game board data. </summary>
  inline static const char* StateStringEnd()
  {
    return "Enter New position \"X Y\":";
  }

  inline static void ExtractKeyValuePair(const std::string& strPair,
                                         std::string* key, int* value)
  {
    assert(!strPair.empty());
    std::string::size_type colonPos = strPair.find(':');
    assert(std::string::npos != colonPos);
    *key = strPair.substr(0, colonPos);
    std::stringstream ssValue(strPair.substr(colonPos + 1));
    ssValue >> *value;
  }

  inline static void ExtractStone(const std::string& line, Stone* stone)
  {
    assert(!line.empty());
    std::string::size_type colonPos = line.find(':');
    assert(std::string::npos != colonPos);
    std::stringstream ssPlayer(line.substr(0, colonPos));
    ssPlayer >> stone->player;
    std::stringstream ssCoords(line.substr(colonPos + 1));
    ssCoords >> stone->pos.x >> stone->pos.y;
  }

  /// <summary> Read next non-empty line from stream. </summary>
  inline static bool ReadNextLineNonEmpty(std::istream& stream, std::string* line)
  {
    while (stream.good())
    {
      std::getline(stream, *line);
      if (!line->empty())
      {
        return true;
      }
    }
    return false;
  }

  /// <summary> Construct game from state string. </summary>
  static bool Parse(const std::string& stateString,
                    const Voronoi::BoardSize& boardSize,
                    Voronoi* game, int* myPlayer)
  {
    std::stringstream ssState(stateString);
    std::string line;

    // Extract game info.
    if (!ReadNextLineNonEmpty(ssState, &line)) { return false; }
    assert(line == "GLOBALS");
    std::string key;
    // Turns.
    if (!ReadNextLineNonEmpty(ssState, &line)) { return false; }
    int numTurns;
    ExtractKeyValuePair(line, &key, &numTurns);
    if (key.empty()) { return false; }
    assert("Total Turns" == key);
    // Players.
    if (!ReadNextLineNonEmpty(ssState, &line)) { return false; }
    int numPlayers;
    ExtractKeyValuePair(line, &key, &numPlayers);
    if (key.empty()) { return false; }
    assert("Total Players" == key);
    // Who.
    if (!ReadNextLineNonEmpty(ssState, &line)) { return false; }
    int playerNum;
    ExtractKeyValuePair(line, &key, &playerNum);
    if (key.empty()) { return false; }
    assert("You are Player" == key);

    // Init game.
    game->Initialize(numPlayers, boardSize);
    *myPlayer = playerNum;

    // Ignore scores.
    if (!ReadNextLineNonEmpty(ssState, &line)) { return false; }
    assert(line == "PLAYER SCORES");
    for (int playerIdx = 0; playerIdx < numPlayers; ++playerIdx)
    {
      if (!ReadNextLineNonEmpty(ssState, &line)) { return false; }
    }

    // Extract remianing state information.
    if (!ReadNextLineNonEmpty(ssState, &line)) { return false; }
    assert(line == "BOARD STATE");
    do
    {
      if (!ReadNextLineNonEmpty(ssState, &line)) { return false; }
      if (std::string::npos == line.find("Enter"))
      {
        Stone stone;
        ExtractStone(line, &stone);
        const bool playRes = game->Play(stone);
        assert(playRes); (void)playRes;
      }
    } while (ssState.good());

    return true;
  }
};

}
}
