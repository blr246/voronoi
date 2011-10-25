#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <iostream>
#include <string>
#include "util.h"
#include "player.h"
#include "voronoi_core.h"
#include "geometry.h"

int g_num_turns;
int g_num_players;
int g_my_player;
enum {BoardDim=1000,};

std::string boardState;
std::string boardStart;
std::string boardEnd;

hps::voronoi::Voronoi* game;

void parseMessage( char buffer[1024] ) {
    
    /* Parsing the GLOBALS */
    sscanf( buffer, "GLOBALS\nTotal Turns: %d\nTotal Players: %d\nYou are Player: %d", 
            &g_num_turns, &g_num_players, &g_my_player );
        
    char* boardStateConst = "BOARD STATE";
    char* boardStateBuffer = strstr(buffer,boardStateConst);
    boardStateBuffer = boardStateBuffer + strlen(boardStateConst);
    
    boardState = std::string(boardStateBuffer);
    boardEnd = "Enter New position \"X Y\":";
    boardStart = "BOARD STATE";
}


void error(const char *msg)
{
    perror(msg);
    exit(0);
}

/* Check for the end of the message (which is '":') */
int isEndOfMessage( char buffer[1024] ) {
    if ( buffer[strlen(buffer)-2] == '\"' && 
	         buffer[strlen(buffer)-1] == ':' ) 
 	    return 1;
    else
      return 0;
}

void writeSocket ( int sockfd, char buffer[1024] ) {
    int n;

    //bzero(buffer,1024);
    memset( buffer, 0, sizeof(char)*1024);
    
    hps::voronoi::Player* player = new hps::voronoi::GreedyPlayer(*game,30);
    player->Play(*game);
    std::string move = game->Compute();
    //std::cout << "returned from player..." <<std::endl;
    //std::cout << move << std::endl;
    buffer = const_cast<char*>(move.c_str());
    /* TODO: Place your new position here (e.g. "100 500" ) */
    //fputs(buffer,1023,stdin);
    n = write(sockfd,buffer,strlen(buffer));
    if (n < 0) 
         error("ERROR writing to socket");
}

void readSocket ( int sockfd, char buffer[1024] ) {
    int n;
    char tmp[256];

    memset( tmp, 0, sizeof(char)*256);
    strcpy( buffer, "" );
    /* Keep reading until we get to the end of the message */
    while ( !isEndOfMessage( buffer ) && (n = read(sockfd,tmp,256)) != 0 ) {
    	strcat(buffer, tmp);
        memset( tmp, 0, sizeof(char)*256);
    }
    if (n < 0) 
         error("ERROR reading from socket");
    
    /* Print message in screen */
    //printf( "buffer: %s", buffer );

    /* Parse message */
    parseMessage( buffer );
    
    
}

void startGame()
{
    hps::geometry::Vector2<int> v(BoardDim,BoardDim);
    game = new hps::voronoi::Voronoi(g_num_players,g_num_turns,v);
    
    if(boardState.length() > 0)
    {
      game->InitBoard(boardState,boardStart,boardEnd);
    }

}

int main(int argc, char *argv[])
{
    int sockfd, portno, i;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[1024];

    /* Check args */
    if (argc < 3) {
       fprintf(stderr,"usage %s hostname port\n", argv[0]);
       exit(0);
    }

    /* Open port and start connection (hopefully the server will be waiting for you) */
    portno = atoi(argv[2]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname(argv[1]);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    memset((char *) &serv_addr, 0, sizeof(char) * sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");

    /* Read from socket */
    readSocket( sockfd, buffer );

    startGame();
    
    /* Write into socket */
    writeSocket( sockfd, buffer );

    
    /* Now we know how many turns we have, let's repeat */
    for ( i = 1; i < g_num_turns; i++ ) {
        readSocket( sockfd, buffer );
	startGame();
        writeSocket( sockfd, buffer );
    }

    close(sockfd);

    return 0;
}

