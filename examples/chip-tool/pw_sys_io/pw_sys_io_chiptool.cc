
#include <cinttypes>
#include "pw_sys_io/sys_io.h"
#include <cassert>
#include <cstddef>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>

int rpc_socket_fd;
int rpc_socket_fd_spare;

extern "C" void pw_sys_io_Init()
{
    rpc_socket_fd = open("/dev/tnt1", O_RDWR);
    rpc_socket_fd_spare = open("/dev/tnt0", O_RDWR);
    struct termios tty;

    if (rpc_socket_fd == -1) {
        /* Could not open the port. */
        perror("open_port: Unable to open /dev/tnt1 - ");
    }

    // Read in existing settings, and handle any error
    if(tcgetattr(rpc_socket_fd, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed


    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(rpc_socket_fd, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return;
    }
}

namespace pw::sys_io {

Status ReadByte(std::byte * dest)
{
    if (!dest) {
        return Status::INVALID_ARGUMENT;
    }
    fputs("*****ReadByte", stderr);
    char buffer[1];
    int n = read(rpc_socket_fd, buffer, 1);
    if (n < 0) {
        fputs("read failed!\n", stderr);
        return Status::FAILED_PRECONDITION;
    }
    const int c = buffer[0];
    *dest       = static_cast<std::byte>(c);
    return c < 0 ? Status::FAILED_PRECONDITION : Status::OK;
}

Status WriteByte(std::byte b)
{
    fputs("*****WriteByte", stderr);
    char c[1]; 
    c[0] = static_cast<char>(b);
    int n = write(rpc_socket_fd, c, 1);
    if (n < 0) {
        fputs("write failed!\n", stderr);
        return Status::FAILED_PRECONDITION;
    }
    return Status::OK;
}

// Writes a string using pw::sys_io, and add newline characters at the end.
StatusWithSize WriteLine(const std::string_view & s)
{
    size_t chars_written  = 0;
    fputs("*****WriteLine", stderr);
    StatusWithSize result = WriteBytes(std::as_bytes(std::span(s)));
    if (!result.ok())
    {
        return result;
    }
    chars_written += result.size();

    // Write trailing newline.
    result = WriteBytes(std::as_bytes(std::span("\r\n", 2)));
    chars_written += result.size();

    return StatusWithSize(result.status(), chars_written);
}

} // namespace pw::sys_io