#include "util.h"
#include "player.h"
#include "voronoi_core.h"
#include "geometry.h"
#include <iostream>
#include <string>
#include <string.h>
#ifdef WIN32
#include <winsock.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netdb.h> 
#endif

using namespace hps;

/// <summary> Predefined game board dimension. </summary>
enum { BoardDim = 1000, };
/// <summary> Buffer size used to read from server socket. </summery>
enum { SockBufferSize = 1024, };

/// <summary> Write a string message to the socket. </summary>
inline bool Write(const int sockfd, const std::string& data)
{
  assert(!data.empty());
  const size_t sent = send(sockfd, data.c_str(), data.size(), 0);
  return sent == data.size();
}

/// <summary> Read all available data on the socket. </summary>
int Read(const int sockfd, const int timeoutMs, std::string* data)
{
  assert(data);
  data->clear();

  // Wait for first chunk?
  fd_set read;
  if (timeoutMs >= 0)
  {
    timeval timeout;
    {
      timeout.tv_sec = timeoutMs / 1000;
      timeout.tv_usec = 1000 * (timeoutMs % 1000);
    }
    FD_ZERO(&read);
    FD_SET(sockfd, &read);
    const int ready = select(sockfd + 1, &read, NULL, NULL, &timeout);
    if (ready < 1)
    {
      return 0;
    }
  }

  // Read data until no more on the socket.
  char buffer[SockBufferSize];
  int numRead = 0;
  for (;;)
  {
    // Read first chunk of data.
    const int sizeRecv = recv(sockfd, buffer, sizeof(buffer), 0);
    numRead += sizeRecv;
    timeval timeout;
    memset(&timeout, 0, sizeof(timeout));
    if (sizeRecv <= 0)
    {
      break;
    }
    *data = *data + std::string(buffer, buffer + sizeRecv);
    // Any more data waiting?
    FD_ZERO(&read);
    FD_SET(sockfd, &read);
    const int ready = select(sockfd + 1, &read, NULL, NULL, &timeout);
    if (ready < 1)
    {
      break;
    }
  }

  return numRead;
}

int main(int argc, char *argv[])
{
#ifdef WIN32
  WORD wsaRqdVersion = MAKEWORD(2, 0);
  WSADATA wsaData;
  WSAStartup(wsaRqdVersion, &wsaData);
#endif

  // Check args.
  if (argc < 3)
  {
    std::cerr << "Usage: " << argv[0] << " HOSTNAME PORT" << std::endl;
    return 1;
  }

  // Open port and start connection (server should be listening).
  const short portno = static_cast<short>(atoi(argv[2]));
  const int sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) 
  {
    std::cerr << "ERROR: failed opening socket." << std::endl;
    return 1;
  }
  const struct hostent* server = gethostbyname(argv[1]);
  if (NULL == server)
  {
    std::cerr << "ERROR: no such host." << std::endl;
    return 1;
  }
  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
  serv_addr.sin_port = htons(portno);
  if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
  {
    std::cerr << "ERROR: failed connecting to server." << std::endl;
    return 1;
  }

  // Play until the server disconnects.
  int roundsPlayed = 0;
  std::string stateString;
  while (Read(sockfd, -1, &stateString) > 0)
  {
    // Wait for primmadonna server to end.
    if (stateString.find("END"))
    {
      break;
    }
    Voronoi game;
    int myPlayer;
    Parser::Parse(stateString, Voronoi::BoardSize(BoardDim, BoardDim),
                  &game, &myPlayer);
    GreedyPlayer player(game, 30);
    player.Play(game);
    std::stringstream ssMove;
    const Stone& stone = game.LastStone();
    ssMove << stone.pos.x << " " << stone.pos.y;
    Write(sockfd, ssMove.str());
    ++roundsPlayed;
  }
  std::cout << "Played " << roundsPlayed << " rounds." << std::endl;

  // Wait for primmadonna server to end.
  Read(sockfd, -1, &stateString);

  // Disconnect.
#ifdef WIN32
  closesocket(sockfd);
  WSACleanup();
#else
  close(sockfd);
#endif

  return 0;
}

