#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include "my_struct.h"
#include "Obstacle_detection_core.h"

void  intial_socket(int &socket_fd)
{
    char *server_ip_addr = "192.168.1.100";//ip地址
    int server_ip_port = 8000;//端口
 
    //要发送给server的数据
   // char *send_message = "hello";
 
    //创建client端的socket套接口
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) {
        fprintf(stderr, "socket error %s errno: %d\n", strerror(errno), errno);
    }
 
    //初始化sockaddr_in结构体
    struct sockaddr_in t_sockaddr;
    memset(&t_sockaddr, 0, sizeof(struct sockaddr_in));
    t_sockaddr.sin_family = AF_INET;
    t_sockaddr.sin_port = htons(server_ip_port);
    inet_pton(AF_INET, server_ip_addr, &t_sockaddr.sin_addr);
 
    while(1)
    {
        //连接
        if((connect(socket_fd, (struct sockaddr*)&t_sockaddr, sizeof(struct sockaddr))) < 0 ) {
        //fprintf(stderr, "connect error %s errno: %d\n", strerror(errno), errno);
        }

    }


}

void sendMessage_socket(int socket_fd, struct obstacle aa)
{
  //向server发送数据
    unsigned char tmp[44];
    memset(tmp, 0, sizeof(tmp));
    memcpy(tmp, &aa, sizeof(aa));
    //cout << sizeof(tmp) << endl;
    //cout<<"temp="<<(int)tmp[5]<<endl;
    //cout <<"temp10="<< (int)tmp[10] << endl;
    //cout << sizeof(tmp) << endl;
    if((send(socket_fd, tmp, 44, NULL)) < 0) {
        fprintf(stderr, "send message error: %s errno : %d", strerror(errno), errno);
 
    }
    //sleep(2);
    //usleep(100);
}



void recvMessage_socket(struct order order,int &ok)
{
    unsigned char recvbuf[100];
    
    while(1)
    {
        memset(recvbuf,'z',100);
    
        int relen = recv(socketfd,recvbuf,100,0);

        // cout << "ok" << endl;
        
        if (relen< 0)
        {
            //cout << "receive default" << endl;
        }
        else
        {
            //cout << "receive successfully" << endl;


        // cout<<"a="<<(int)recvbuf[0]<<" "<<(int)recvbuf[1]<<endl;

            memset(&order, 0, sizeof(order));
            memcpy(&order,recvbuf,sizeof(recvbuf));

            //cout << "order.code" << order.Code << "order.control" << order.control<< endl;

            if(order.Code == 1 && order.control == 1)
            {
                ok = 1;
                cout << "ok = " << endl;

            }
            
            if(order.Code == 1 && order.control == 0)
            {
                ok = 0;
            }

            
        }
    }
    
}