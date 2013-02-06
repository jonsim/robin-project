/**
 * @file        TCPInterface.h
 * @brief       Functions to manage and control the TCP Interface (the server component).
 */
#ifndef TCPINTERFACE_H
#define TCPINTERFACE_H

/*-------------------- INCLUDES --------------------*/
#include "Common.h"
// networking
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>


/*-------------------- DEFINES  --------------------*/
#define SERVER_ADDRESS JPANDA_ADDRESS
#define CLIENT_ADDRESS JLAPTOP_ADDRESS
#define STREAMING_PORT_D 1401
#define STREAMING_PORT_C 1402

enum TCPInterfaceMode
{
    TCPCLIENT = 1,
    TCPSERVER = 2
};


/*--------------------  MACROS  --------------------*/
// Check standard function return values (error if < 0) and throw an error if necessary
#define CHECK_TCPMODE(requiredMode)                                                 \
    if (mMode != requiredMode)                                                      \
    {                                                                               \
        printf("ERROR: TCPInterface is not configured to perform this action.\n");  \
        exit(EXIT_FAILURE);                                                         \
    }


/*---------------- CLASS DEFINITION ----------------*/
class TCPInterface
{
public:
    TCPInterface (TCPInterfaceMode mode, int port);
    ~TCPInterface (void);
    
    void checkForServers (void);
    void checkForClients (void);
    void writeBytes (const void* bs, size_t n);
    void readBytes  (void* bs, size_t n);
    void writeFrame (void);
    void readFrame  (void);


private:
    void setupServer (void);
    void setupClient (void);
    void setBlocking (bool_t blocking);
    void connectToServer (void);
    void connectToClient (void);
    void shutdownServer (void);
    void shutdownClient (void);
    
    TCPInterfaceMode mMode;
    int mServerSocket;
    int mClientSocket;
    struct sockaddr_in mServerAddress;
    struct sockaddr_in mClientAddress;
    int mServerPort;
    uint8_t mClientConnected;
};

#endif
