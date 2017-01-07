#include <cmath>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Message.h"
#include "Shape.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>

#include "MessageSender.h"


MessageSender::MessageSender(char *server, char *port)
{
  struct addrinfo hints, *p;

  int s, i, slen = sizeof(sockaddr_in);

  memset((char *) &hints, 0, sizeof(hints));
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_family = AF_UNSPEC;
  hints.ai_flags = AI_NUMERICSERV;
  hints.ai_protocol = 0;

  int rv;

  if ((rv = getaddrinfo(server, port, &hints, &info)) != 0) 
  {
    std::cout<<"couldn't get address info"<<std::endl;
    sock=-1;
    return;
  }

  if ((sock = socket(info->ai_family, info->ai_socktype,
      info->ai_protocol)) == -1) 
  {
    std::cout<<"couldn't open socket"<<std::endl;
    sock=-1;
    return;
  }
}

void MessageSender::sendMessage(Shape *left = NULL,
		Shape *middle = NULL, Shape *right = NULL) 
{
  Message msg;
  if (left != NULL) 
  {
    msg.posX1 = (left->maxX + left->minX) / 2;
    msg.posY1 = (left->maxY + left->minY) / 2;
  }

  if (middle != NULL) 
  {
    msg.posX2 = (middle->maxX + middle->minX) / 2;
    msg.posY2 = (middle->maxY + middle->minY) / 2;
  }

  if (right != NULL) 
  {
    msg.posX3 = (right->maxX + right->minX) / 2;
    msg.posY3 = (right->maxY + right->minY) / 2;
  }

  if (sock > -1)
  {
    if (sendto(sock, (unsigned char*) &msg, sizeof(msg), 0,
		    info->ai_addr, info->ai_addrlen) == -1) 
    {
	std::cout<<"sendto failed"<<std::endl;
    }
  }
}
