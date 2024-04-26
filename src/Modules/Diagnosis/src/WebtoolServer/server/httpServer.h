#ifndef HTTPSERVER_H
#define HTTPSERVER_H
#include <mutex>
#include <thread>
#include <functional>
#include <map>
#include <vector>
#include <string>

class HTTPServer
{
public:
    using HeadHandler = std::function<void(int statusCode, const std::string &extraHeaders)>;
    using BodyHander = std::function<void(const char *buf, size_t len)>;
    using PostHandlerEx = std::function<void(const std::string &msg, HeadHandler headSender, BodyHander bodySender)>;
    using PostHandler = std::function<int(const std::string &msg, std::string& resp)>;
    using GetHandlerEx = std::function<void(const std::string &msg, HeadHandler headSender, BodyHander bodySender)>;
    using GetHandler = std::function<int(const std::string &msg, std::string& resp)>;
    using DownloadHandler = std::function<bool(const std::string &msg, std::string& path, std::string& file)>;
    using Checker = std::function<bool(const std::string& param)>;

private:
    std::timed_mutex m_mutex;
    std::thread m_thread;
    static struct mg_serve_http_opts s_http_server_opts;
    std::map<std::string, std::pair<Checker,PostHandlerEx> > m_postHandlerEx;
    std::map<std::string, std::pair<Checker,PostHandler> > m_postHandler;
    std::map<std::string, std::pair<Checker,GetHandlerEx> > m_getHandlerEx;
    std::map<std::string, std::pair<Checker,GetHandler> > m_getHandler;
    std::map<std::string, std::pair<Checker,DownloadHandler> > m_downloadHandler;
    std::string m_saveDir;

private:
    HTTPServer();
    ~HTTPServer();

    static HTTPServer *&_instance()
    {
        static HTTPServer *p = nullptr;
        return p;
    }
    static void threadFn(std::string port, std::string web_root);
    static void ev_handler(struct mg_connection *nc, int ev, void *ev_data);
    static inline std::vector<std::string> split(const std::string &str,const std::string &pattern);
    static inline void handlePost(struct mg_connection *nc, struct http_message *hm);
    static inline void handleGet(struct mg_connection *nc, struct http_message *hm);
    static inline bool handleDownload(struct mg_connection *nc, struct http_message *hm);
    static inline void handleWebsocketMessage(struct mg_connection *nc,int event_type,  void *p);
    static void handle_upload(struct mg_connection *nc, int ev, void *p);

public:
    static HTTPServer &getObj()
    {
        if (!_instance())
        {
            _instance() = new HTTPServer();
        }
        return *_instance();
    }

    bool addPostEx(const std::string &uri, PostHandlerEx callback, Checker checker = Checker());
    bool addPost(const std::string &uri, PostHandler callback, Checker checker = Checker());
    bool addGetEx(const std::string &uri, GetHandlerEx callback, Checker checker = Checker());
    bool addGet(const std::string &uri, GetHandler callback, Checker checker = Checker());
    bool addDownload(const std::string &uri, DownloadHandler callback, Checker checker = Checker());
    bool start(std::string port, std::string web_root, std::string save_dir);
    void sendWebsocketMsg(mg_connection *connection, std::string msg);
    bool stop();
    void join();
};

#define HTTP_SERVER_OBJ HTTPServer::getObj()

#endif
