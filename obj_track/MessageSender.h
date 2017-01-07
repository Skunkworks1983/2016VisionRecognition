#ifndef MESSAGE_SENDER
#define MESSAGE_SENDER

class MessageSender
{
  private:
    int sock;
    addrinfo *info;

  public:
    MessageSender(char *server, char *port);

    ~MessageSender();
    void sendMessage(Shape *left, Shape *middle, Shape *right);
};

#endif