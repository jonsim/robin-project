#include "TCPInterface.h"


/// @brief  Constructor.
/// @param  mode  The mode for this interface to act in. Can be either TCPCLIENT or TCPSERVER.
/// @param  port  The server port which will, in the case of the client, be connected to or, in the
///               case of the server, be used to accept incoming connections.
TCPInterface::TCPInterface (TCPInterfaceMode mode, int port) : mMode(mode),
                                                               mServerPort(port)
                                                               
{
    if (mode == TCPCLIENT)
        setupClient(SERVER_ADDRESS);
    else
        setupServer();
}


/// @brief  Deconstructor.
TCPInterface::~TCPInterface (void)
{
    if (mMode == TCPCLIENT)
        shutdownClient();
    else
        shutdownServer();
}


/// @brief  Configures and starts a TCP server at the given port on the local machine.
void TCPInterface::setupServer (void)
{
    CHECK_TCPMODE(TCPSERVER);
    int retVal;
    
    // Create an IP streaming socket (which uses TCP).
    mServerSocket = socket(PF_INET, SOCK_STREAM, 0);
    CHECK_RETURN(mServerSocket, "socket");

    // Bind to the port, setting up the address of the server given by the defines.
	// Note that both address and port number must be in network byte order.
    memset(&mServerAddress, 0, sizeof(mServerAddress));
    mServerAddress.sin_family = AF_INET;
    mServerAddress.sin_port   = htons(mServerPort);  // host to network short: converts a u_short from host to TCP network byte order (big-endian).
    retVal = bind(mServerSocket, (struct sockaddr*) &mServerAddress, sizeof(mServerAddress));
    CHECK_RETURN(retVal, "bind");

    // Set to listen on the port with a connection queue length of 1.
    retVal = listen(mServerSocket, 1);
    CHECK_RETURN(retVal, "listen");
    
    // turn off read blocking
    setBlocking(false);
}


/// @brief  Configures and starts a TCP client and loads the data for the server to connect to
///         (though it does not connect at this stage, see the connectToServer() method).
void TCPInterface::setupClient (const char* serverAddress)
{
    CHECK_TCPMODE(TCPCLIENT);
    int retVal;
    
    // Create an IP streaming socket (which uses TCP).
    mClientSocket = socket(PF_INET, SOCK_STREAM, 0);
    CHECK_RETURN(mClientSocket, "socket");
    
    // Set up the address of the server as given by the defines.
	// Note that both address and port number must be in network byte order.
	memset(&mServerAddress, 0, sizeof(mServerAddress));
	mServerAddress.sin_family = AF_INET;
	mServerAddress.sin_port   = htons(mServerPort);  // host to network short: converts to TCP network byte order (big-endian).
	retVal = inet_aton(serverAddress, &(mServerAddress.sin_addr));
	CHECK_RETURN(retVal, "inet_aton");      // NB: There was originally a close(client_socket) in this if.
}


/// @brief  Shuts down the server, closing all sockets (thus terminating all connections).
void TCPInterface::shutdownServer (void)
{
    CHECK_TCPMODE(TCPSERVER);
    close(mClientSocket);
    close(mServerSocket);
}


/// @brief  Shuts down the client, closing all sockets (thus terminating all connections).
void TCPInterface::shutdownClient (void)
{
    CHECK_TCPMODE(TCPCLIENT);
    close(mClientSocket);
}


/// @brief  Sets whether the server port is blocking or not, i.e. if accepting connections from the
///         port will block if no connections are currently available. This is only available in
///         server mode for the time being.
/// @param  blocking  Whether or not accepting connections will block. This defaults to true when
///                   you open a connection. 
void TCPInterface::setBlocking (bool_t blocking)
{
    CHECK_TCPMODE(TCPSERVER);   // TODO should this work in client mode?
    if (blocking)
        fcntl(mClientSocket, F_SETFL, !O_NONBLOCK); // TODO is this the right socket for server mode?
    else
        fcntl(mClientSocket, F_SETFL, O_NONBLOCK);
}


/// @brief  Connects to the server (whose details are either defined or given as to the constructor).
void TCPInterface::connectToServer (void)
{
    CHECK_TCPMODE(TCPCLIENT);
    int retVal;
    
    printf("Waiting to connect to server on %s:%d...\n", SERVER_ADDRESS, mServerPort);
    retVal = connect(mClientSocket, (struct sockaddr*) &mServerAddress, sizeof(mServerAddress));
    CHECK_RETURN(retVal, "connect");
    printf("Connected.\n");
}


/// @brief  Connects to a client, if one has requested to join, or waits until one requests to do
///         so. Blocking mode must be set to true.
///         FOR NON-BLOCKING CONNECTIONS USE checkForClients()!
void TCPInterface::connectToClient (void)
{
    CHECK_TCPMODE(TCPSERVER);
    socklen_t sizeofmClientAddress = sizeof(mClientAddress);
    mClientSocket = accept(mServerSocket, (struct sockaddr*) &mClientAddress, &sizeofmClientAddress);
    CHECK_RETURN(mClientSocket, "accept");
    printf("Connection from %s:%d.\n", inet_ntoa(mClientAddress.sin_addr), htons(mClientAddress.sin_port));
}


/// @brief  Waits for the server to become ready and then connects to it.
void TCPInterface::waitForServerConnection (void)
{
    connectToServer();
}


/// @brief  Checks whether any clients have requested to join and, if they have, accepts them. If
///         not the function will return. Blocking mode must be set to false.
///         FOR BLOCKING CONNECTIONS USE connectToClient()!
/// @return Whether or not a client was found (true = yes, false = no).
bool TCPInterface::checkForClients (void)
{
    CHECK_TCPMODE(TCPSERVER);
    socklen_t sizeofmClientAddress = sizeof(mClientAddress);
    mClientSocket = accept(mServerSocket, (struct sockaddr*) &mClientAddress, &sizeofmClientAddress);
    if ((mClientSocket < 0) && (errno == EWOULDBLOCK))
        return false;
    CHECK_RETURN(mClientSocket, "accept");
    printf("Connection from %s:%d.\n", inet_ntoa(mClientAddress.sin_addr), htons(mClientAddress.sin_port));
    return true;
}


/// @brief  Writes the supplied bytes to the socket (sending them to the other party).
/// @param  bs  The bytes to write.
/// @param  n   The number of bytes from bs to write.
void TCPInterface::writeBytes (const void* bs, size_t n)
{
    int retVal = write(mClientSocket, bs, n);
    CHECK_RETURN(retVal, "writeBytes");
}


/// @brief  Reads the given number of bytes from the socket. This will block until the requested
///         number of bytes have been read (unlike the read command by default which will read only
///         as many bytes as are available).
/// @param  bs  The byte array to read into.
/// @param  n   The number of bytes to read.
void TCPInterface::readBytes (void* bs, size_t n)
{
    int    bytesRead;
    size_t totalBytesRead = 0;
    while ((bytesRead = read(mClientSocket, (((uint8_t*) bs) + totalBytesRead), (n - totalBytesRead))) > 0)
    {
        totalBytesRead += bytesRead;
        if (totalBytesRead >= n)
            return;
    }
    CHECK_RETURN(bytesRead, "readBytes");
}
