#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

int main() {
    int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) { perror("socket"); return 1; }

    int opt = 1;
    setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port= htons(8888);
    if(bind(socket_fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return 1;
    }

    if (listen(socket_fd, 128) < 0) {
        perror("listen");
        return 1;
    }
    std::cout << "Echo Server listening 0.0.0.0:8888\n";

    while(true) {
        sockaddr_in client{};
        socklen_t len;
        int conn_fd = accept(socket_fd, (sockaddr*)&client, &len);
        if (conn_fd < 0) { perror("accept"); continue; }

        char buf[1024];
        while(true) {
            int n = read(conn_fd, buf, sizeof(buf));
            if (n <=0) { perror("read"); break; }
            if (write(conn_fd, buf, n) < 0) {
                perror("write");
                break;
            }
        }

        close(conn_fd);
        std::cout << "Client disconnected.\n";
    }
    close(socket_fd);
    return 0;
}
