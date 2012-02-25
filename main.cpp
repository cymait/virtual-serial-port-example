/**
 * Serial port example.
 * Compile with: g++ main.cpp -lpthread -o main
 * 
 * Cymait http://cymait.com
 **/

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <pthread.h>
#include <termios.h>

#define BUFFER_SIZE 128
#define BAUDRATE    B9600

void usage(char* cmd) {
    std::cerr << "usage: " << cmd << " slave|master [device, only in slave mode]" << std::endl;
    exit(1);
}

void* reader_thread(void* pointer) {
    int fd = (int)pointer;
    char inputbyte;
    while (read(fd, &inputbyte, 1) == 1) {
        std::cout << inputbyte;
        std::cout.flush();
    }

    return 0;
}

int main(int argc, char** argv) {
    if (argc < 2) usage(argv[0]);

    int fd = 0;
    std::string mode = argv[1];

    if (mode == "slave") {
        if (argc < 3) usage(argv[1]);

        fd = open(argv[2], O_RDWR);
        if (fd == -1) {
            std::cerr << "error opening file" << std::endl;
            return -1;
        }

    }else if (mode=="master") {
        fd = open("/dev/ptmx", O_RDWR | O_NOCTTY);
        if (fd == -1) {
            std::cerr << "error opening file" << std::endl;
            return -1;
        }

        grantpt(fd);
        unlockpt(fd);

        char* pts_name = ptsname(fd);
        std::cerr << "ptsname: " << pts_name << std::endl;
    } else {
        usage(argv[1]);
    }

    /* serial port parameters */
    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio));
    struct termios oldtio;
    tcgetattr(fd, &oldtio);

    newtio = oldtio;
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = 0;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 0;
    tcflush(fd, TCIFLUSH);

    cfsetispeed(&newtio, BAUDRATE);
    cfsetospeed(&newtio, BAUDRATE);
    tcsetattr(fd, TCSANOW, &newtio);

    /* start reader thread */
    pthread_t thread;
    pthread_create(&thread, 0, reader_thread, (void*) fd);

    /* read from stdin and send it to the serial port */
    char c;
    while (true) {
        std::cin >> c;
        write(fd, &c, 1);
    }

    close(fd);
    return 0;
}
