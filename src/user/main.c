#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <signal.h>
#include <time.h>
#include <errno.h>

#ifdef PLATFORM_LINUX
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#endif

#ifdef PLATFORM_WINDOWS
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <BaseTsd.h>  // 这个头文件定义了SSIZE_T
typedef SSIZE_T ssize_t;
#pragma comment(lib, "ws2_32.lib")
#endif
static volatile sig_atomic_t keep_running = 1;

void handle_sigint(int sig) {
    (void)sig;
    keep_running = 0;
}
#ifdef PLATFORM_WINDOWS
void PrintLastWSAError() {
    DWORD errorCode = WSAGetLastError();
    LPSTR messageBuffer = NULL;
    DWORD flags = FORMAT_MESSAGE_ALLOCATE_BUFFER |
        FORMAT_MESSAGE_FROM_SYSTEM |
        FORMAT_MESSAGE_IGNORE_INSERTS;

    // 尝试获取系统错误描述
    DWORD size = FormatMessageA(
        flags,
        NULL,                   // 源（系统错误）
        errorCode,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPSTR)&messageBuffer,  // 输出缓冲区
        0,
        NULL
    );

    // 如果失败，尝试从 winsock 模块获取
    if (size == 0) {
        HMODULE hNet = LoadLibraryExA("ws2_32.dll", NULL, LOAD_LIBRARY_AS_DATAFILE);
        size = FormatMessageA(
            flags | FORMAT_MESSAGE_FROM_HMODULE,
            hNet,                // 从 ws2_32.dll 获取
            errorCode,
            MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
            (LPSTR)&messageBuffer,
            0,
            NULL
        );
        if (hNet) FreeLibrary(hNet);
    }

    if (size > 0 && messageBuffer) {
        fprintf(stderr,"Error %d: %s", errorCode, messageBuffer);
        LocalFree(messageBuffer); // 释放缓冲区
    }
    else {
        fprintf(stderr,"Unknown error: %d", errorCode);
    }
}
#endif
int main(void) {
    #ifdef PLATFORM_WINDOWS
    SOCKET sock = INVALID_SOCKET;
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        printf("Failed. Error Code: %d", WSAGetLastError());
        return 1;
    }
    #else
    int sock = -1;
    #endif
    struct sockaddr_in dest;
    #ifndef PLATFORM_WINDOWS
    signal(SIGINT, handle_sigint);
    
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return 1;
    }
    #else
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        printf("Failed to create socket. Error Code: %d", WSAGetLastError());
        WSACleanup();
        return 1;
    }
    #endif  

    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(15000);
    if (inet_pton(AF_INET, "127.0.0.1", &dest.sin_addr) != 1) {
        #ifndef PLATFORM_WINDOWS
        perror("inet_pton");
        close(sock);
        #else
        closesocket(sock);
        WSACleanup();
        #endif
        return 1;
    }

    const char *service_name = "positioning_service";
    const char *payload = "payload";
    char msg[512];
    int msg_len;
    while (keep_running) {
        msg_len = snprintf(msg, sizeof(msg), "service:%s,payload:%s", service_name, payload);
        if (msg_len < 0 || msg_len >= (int)sizeof(msg)) {
            fprintf(stderr, "message too long or encoding error\n");
            break;
        }

        ssize_t sent = sendto(sock, msg, (size_t)msg_len, 0,
                              (struct sockaddr *)&dest, sizeof(dest));
        if (sent < 0) {
#ifdef PLATFORM_WINDOWS
            PrintLastWSAError();
#else
            fprintf(stderr, "sendto failed:%s\n", strerror(errno));
#endif
        } else {
            printf("sent (%zd bytes): %s\n", sent, msg);
            fflush(stdout);
        }
        #ifndef PLATFORM_WINDOWS
        struct timespec ts = {1, 0}; /* 1 second */
        while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
            if (!keep_running) break;
        }
        #else
        Sleep(1000);
        #endif
    }

    #ifndef PLATFORM_WINDOWS
    close(sock);
    #else
    closesocket(sock);
    WSACleanup();
    #endif
    printf("exiting\n");
    return 0;
}
