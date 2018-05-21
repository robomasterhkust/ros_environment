#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <ctime>
#include <time.h>
#include <sys/time.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>

int fd = 0;
struct timeval tv_begin;
struct timeval tv_end;
char* angleData = (char*)malloc(14 * sizeof(char));

void sendGimbalAngle();

int UART0_Open(int fd, char *port) {
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (false == fd) return (false);
    if (fcntl(fd, F_SETFL, 0) < 0) return (false);
    if (0 == isatty(STDIN_FILENO)) return (false);
    return fd;
}

void UART0_Close(int fd) {
    close(fd);
}

int UART0_Init(int fd, int speed, int flow_ctrl, int angleDatabits, int stopbits, int parity) {
    // Parameters below are hardcoded
    int i;
    int status;
    struct termios options;

    if (tcgetattr(fd, &options) != 0) return (false);

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= CLOCAL;
    options.c_cflag |= CREAD;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_iflag &= ~INPCK;
    options.c_cflag &= ~CSTOPB;
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 1;

    tcflush(fd, TCIFLUSH);

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("com set error!\n");
        return (false);
    }
    return (true);
}

int UART0_Send(int fd, char *send_buf, int angleData_len) {
    int len = 0;

    len = write(fd, send_buf, angleData_len);
    if (len == angleData_len)
        return len;
    else {
        tcflush(fd, TCOFLUSH);
        return false;
    }
}

void serialSetup() {
    int err;
    char port[] = "/dev/ttyTHS2";

    fd = UART0_Open(fd, port);
    do {
        err = UART0_Init(fd, 115200, 0, 8, 1, 'N');
    } while (false == err || false == fd);
}

void handle(union sigval v) {
    time_t t;
    char p[32];

    time(&t);
    strftime(p, sizeof(p), "%T", localtime(&t));

    sendGimbalAngle();
}

void serialStart() {
    struct sigevent evp;
    struct itimerspec ts;
    timer_t timer;
    int ret;

    memset(&evp, 0, sizeof(evp));
    evp.sigev_value.sival_ptr = &timer;
    evp.sigev_notify = SIGEV_THREAD;
    evp.sigev_notify_function = handle;
    evp.sigev_value.sival_int = 3; //as a parameter for handle()

    ret = timer_create(CLOCK_REALTIME, &evp, &timer);

    ts.it_interval.tv_sec = 0;
    ts.it_interval.tv_nsec = 1000000000; //transmitting time interval in nanosecond
    ts.it_value.tv_sec = 3;
    ts.it_value.tv_nsec = 0;

    ret = timer_settime(timer, TIMER_ABSTIME, &ts, NULL);
}

/*************************
********** API ***********
**************************/
void setGimbalAngle(int index, float pitch, float yaw, float dist_z){

    char header = 0xa5;
	char shootIndex = index;
    float targetAngle[2] = {pitch, yaw};

    memcpy(angleData, &header, sizeof(char));
    memcpy(angleData + 1, targetAngle, 2*sizeof(float));
	memcpy(angleData + 9, &shootIndex, sizeof(char));
	memcpy(angleData + 10, &dist_z, sizeof(float));
}

void sendGimbalAngle(){
 
    int len = UART0_Send(fd, angleData, 14);
}

/*
int main()
{

    serialSetup();
    serialStart();

    //UART0_Close(fd);

    return 0;
}
*/