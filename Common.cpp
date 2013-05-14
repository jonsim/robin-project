#include "Common.h"



sint8_t make_sint8_t (const uint8_t b)
{
    return (b - 0xFF);
}


uint16_t make_uint16_t (const uint8_t bh, const uint8_t bl)
{
    uint16_t r = (bh << 8) + bl;
    return r;
}


sint16_t make_sint16_t (const uint8_t bh, const uint8_t bl)
{
//    sint16_t r = ((bh << 8) + bl) - 0xFFFF;
    sint16_t r = bl + ((bh & 0x7f) << 8) - ((bh & 0x80) << 8);
//    printf("bh=%x, bl = %x, r = %x, %d\n", bh, bl, r, r);
    return r;
}


float euclidean_distance (const Point2i& p1, const Point2i& p2)
{
    float dx = (float) p1.x - p2.x;
    float dy = (float) p1.y - p2.y;
    return sqrt(dx*dx + dy*dy);
}


float euclidean_distance2 (const Point2i& p1, const Point2i& p2)
{
    float dx = (float) p1.x - p2.x;
    float dy = (float) p1.y - p2.y;
    return dx*dx + dy*dy;
}


/// @brief  Uses the Box-Muller transform to generate a single normally distributed random number.
///         NB: This is not efficient. If I find myself calling it a lot consider using the polar
///             form of the transform or potentially even the Ziggurat algorithm.
///         NB: rand() must be seeded when this function is called. If in doubt, srand!
/// @param  mu      The mean of the distribution.
/// @param  sigma   The standard deviation of the distribution.
/// @return         A random number normally distributed according to the supplied parameters.
float randNormallyDistributed (float mu, float sigma)
{
    float U1 = ((float) rand()) / ((float) RAND_MAX);
    float U2 = ((float) rand()) / ((float) RAND_MAX);
    float Z0 = sqrt(-2 * log(U1)) * cos(2 * PI * U2);
    return (Z0 * sigma) + mu;
}


void msleep (const uint32_t msec)
{
    usleep(msec * 1000);
}


int _kbhit (void)
{
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}


/// @brief  Code snippet from stackoverflow
std::vector<std::string>& split(const std::string& s, char delim, std::vector<std::string>& elems)
{
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim))
        elems.push_back(item);
    return elems;
}


/// @brief  Code snippet from stackoverflow
std::vector<std::string> split(const std::string& s, char delim)
{
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}
