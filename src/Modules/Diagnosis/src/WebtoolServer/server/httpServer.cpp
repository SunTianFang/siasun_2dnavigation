#include "httpServer.h"
#include <chrono>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include "json/json.h"
#include "../mongoose/mongoose.h"
struct mg_serve_http_opts HTTPServer::s_http_server_opts;

HTTPServer::HTTPServer()   
{
}

HTTPServer::~HTTPServer()
{
}

void HTTPServer::handle_upload(struct mg_connection *nc, int ev, void *p) {
  FILE *data = (FILE *) nc->user_data;
  struct mg_http_multipart_part *mp = (struct mg_http_multipart_part *) p;

  switch (ev) {
    case MG_EV_HTTP_PART_BEGIN: {
      if (data == NULL) {
        std::string fullName = getObj().m_saveDir + "/" + mp->file_name;
        data = mg_fopen(fullName.c_str(),"wb");//tmpfile();
        if (data == NULL) {
          mg_printf(nc, "%s",
                    "HTTP/1.1 500 Failed to open a file\r\n"
                    "Content-Length: 0\r\n\r\n");
          nc->flags |= MG_F_SEND_AND_CLOSE;

          return;
        }
        nc->user_data = (void *) data;
      }
      break;
    }
    case MG_EV_HTTP_PART_DATA: {
      if (fwrite(mp->data.p, 1, mp->data.len, data) != mp->data.len) {
        mg_printf(nc, "%s",
                  "HTTP/1.1 500 Failed to write to a file\r\n"
                  "Content-Length: 0\r\n\r\n");
        nc->flags |= MG_F_SEND_AND_CLOSE;
        return;
      }
      break;
    }
    case MG_EV_HTTP_PART_END: {
      mg_printf(nc,
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/plain\r\n"
                "Connection: close\r\n\r\n"
                "Written %ld of POST data to a temp file\n\n",
                (long) ftell(data));
      nc->flags |= MG_F_SEND_AND_CLOSE;
      fclose(data);
      nc->user_data = NULL;
      break;
    }
  }
}

void HTTPServer::threadFn(std::string port, std::string web_root)
{
    struct mg_mgr mgr;
    struct mg_connection *nc;
    /* Open listening socket */
    mg_mgr_init(&mgr, NULL);
    nc = mg_bind(&mgr, port.c_str(), ev_handler);

    s_http_server_opts.document_root = web_root.c_str();
    mg_register_http_endpoint(nc, "/api/upload", handle_upload);
    mg_set_protocol_http_websocket(nc);

    while (!getObj().m_mutex.try_lock_for(std::chrono::microseconds(1)))
    {
        mg_mgr_poll(&mgr, 1000);
    }
    mg_mgr_free(&mgr);
    getObj().m_mutex.unlock();
}

void HTTPServer::handlePost(struct mg_connection *nc, struct http_message *hm)
{
    std::cout<<"By Yu Test : In http::handlePost"<<std::endl;
    std::string uri(hm->uri.p, hm->uri.len);
    std::string param(hm->body.p, hm->body.len);  
    if(getObj().m_postHandler.find(uri) != getObj().m_postHandler.end())
    {

        if(getObj().m_postHandler[uri].first && !getObj().m_postHandler[uri].first(param))
        {
            mg_printf(nc, "%s",
                      "HTTP/1.1 403 Forbidden\r\n"
                      "Content-Length: 0\r\n\r\n");
            return;
        }
        if (getObj().m_postHandler[uri].second)
        {
            std::string resp;
            int code = getObj().m_postHandler[uri].second(param,resp);
            mg_send_head(nc, code, -1, "");
            mg_send_http_chunk(nc, resp.c_str(), resp.size());
            mg_send_http_chunk(nc, "", 0);
            return;
        }
        else
        {
            mg_printf(nc, "%s",
                      "HTTP/1.1 501 Not Implemented\r\n"
                      "Content-Length: 0\r\n\r\n");
            return;
        }
    }
    else if (getObj().m_postHandlerEx.find(uri) != getObj().m_postHandlerEx.end())
    {
        if(getObj().m_postHandlerEx[uri].first && !getObj().m_postHandlerEx[uri].first(param))
        {
            mg_printf(nc, "%s",
                      "HTTP/1.1 403 Forbidden\r\n"
                      "Content-Length: 0\r\n\r\n");
            return;
        }
        if (getObj().m_postHandlerEx[uri].second)
        {
            getObj().m_postHandlerEx[uri].second(
                param,
                [nc](int statusCode, const std::string &extraHeaders) {
                    mg_send_head(nc, statusCode, -1, extraHeaders.c_str());
                },
                [nc](const char *buf, size_t len) {
                    mg_send_http_chunk(nc, buf, len);
                });
            return;
        }
        else
        {
            mg_printf(nc, "%s",
                      "HTTP/1.1 501 Not Implemented\r\n"
                      "Content-Length: 0\r\n\r\n");
            return;
        }
    }
    else
    {


        mg_serve_http(nc, hm, s_http_server_opts); /* Serve static content */
    }
}

void HTTPServer::handleGet(struct mg_connection *nc, struct http_message *hm)
{
    std::string uri(hm->uri.p, hm->uri.len);
    std::string param(hm->query_string.p, hm->query_string.len);

    if(getObj().m_getHandler.find(uri) != getObj().m_getHandler.end())
    {
        if(getObj().m_getHandler[uri].first && !getObj().m_getHandler[uri].first(param))
        {
            mg_printf(nc, "%s",
                      "HTTP/1.1 403 Forbidden\r\n"
                      "Content-Length: 0\r\n\r\n");
            return;
        }
        if (getObj().m_getHandler[uri].second)
        {
            std::string resp;
            int code = getObj().m_getHandler[uri].second(param,resp);
            mg_send_head(nc, code, -1, "");
            mg_send_http_chunk(nc, resp.c_str(), resp.size());
            mg_send_http_chunk(nc, "", 0);
            return;
        }
        else
        {
            mg_printf(nc, "%s",
                      "HTTP/1.1 501 Not Implemented\r\n"
                      "Content-Length: 0\r\n\r\n");
            return;
        }
    }
    else if (getObj().m_getHandlerEx.find(uri) != getObj().m_getHandlerEx.end())
    {
        if(getObj().m_getHandlerEx[uri].first && !getObj().m_getHandlerEx[uri].first(param))
        {
            mg_printf(nc, "%s",
                      "HTTP/1.1 403 Forbidden\r\n"
                      "Content-Length: 0\r\n\r\n");
            return;
        }
        if (getObj().m_getHandlerEx[uri].second)
        {
            getObj().m_getHandlerEx[uri].second(
                param,
                [nc](int statusCode, const std::string &extraHeaders) {
                    mg_send_head(nc, statusCode, -1, extraHeaders.c_str());
                },
                [nc](const char *buf, size_t len) {
                    mg_send_http_chunk(nc, buf, len);
                });
            return;
        }
        else
        {
            mg_printf(nc, "%s",
                      "HTTP/1.1 501 Not Implemented\r\n"
                      "Content-Length: 0\r\n\r\n");
            return;
        }
    }
    else
    {
        mg_serve_http(nc, hm, s_http_server_opts); /* Serve static content */
    }
}

bool HTTPServer::handleDownload(mg_connection *nc, http_message *hm)
{
    std::string uri(hm->uri.p, hm->uri.len);
    std::string param(hm->query_string.p, hm->query_string.len);
    std::string path,file;
    if(getObj().m_downloadHandler.find(uri) == getObj().m_downloadHandler.end())
    {
        return false;
    }
    if(getObj().m_downloadHandler[uri].first && !getObj().m_downloadHandler[uri].first(param))
    {
        mg_printf(nc, "%s",
                  "HTTP/1.1 403 Forbidden\r\n"
                  "Content-Length: 0\r\n\r\n");
    }
    else if (getObj().m_downloadHandler[uri].second
             && getObj().m_downloadHandler[uri].second(param,path,file))
    {
        std::string config = "Content-Disposition:attachment;filename=" + file;
        mg_http_serve_file(nc, hm, (path + file).c_str(), mg_mk_str("application/octet-stream"), mg_mk_str(config.data()));

    }
    else
    {
        mg_printf(nc, "%s",
                  "HTTP/1.1 403 Forbidden\r\n"
                  "Content-Length: 0\r\n\r\n");
    }
    return true;
}

//void HTTPServer::handleWebsocketMessage(mg_connection *nc, int event_type, void *p)
//{
//    if (event_type == MG_EV_WEBSOCKET_HANDSHAKE_DONE)
//        {
//            printf("client websocket connected\n");
//            // 获取连接客户端的IP和端口
//            char addr[32];
//            mg_sock_addr_to_str(&connection->sa, addr, sizeof(addr), MG_SOCK_STRINGIFY_IP | MG_SOCK_STRINGIFY_PORT);
//            printf("client addr: %s\n", addr);

//            // 添加 session
//            s_websocket_session_set.insert(connection);

//            SendWebsocketMsg(connection, "client websocket connected");
//        }
//        else if (event_type == MG_EV_WEBSOCKET_FRAME)
//        {
//            mg_str received_msg = {
//                (char *)ws_msg->data, ws_msg->size
//            };

//            char buff[1024] = {0};
//            strncpy(buff, received_msg.p, received_msg.len); // must use strncpy, specifiy memory pointer and length

//            // do sth to process request
//            printf("received msg: %s\n", buff);
//            SendWebsocketMsg(connection, "send your msg back: " + std::string(buff));
//            //BroadcastWebsocketMsg("broadcast msg: " + std::string(buff));
//        }
//        else if (event_type == MG_EV_CLOSE)
//        {
//            if (isWebsocket(connection))
//            {
//                printf("client websocket closed\n");
//                // 移除session
//                if (s_websocket_session_set.find(connection) != s_websocket_session_set.end())
//                    s_websocket_session_set.erase(connection);
//            }
//        }
//}

void sendWebsocketMsg(mg_connection *connection, std::string msg)
{
    mg_send_websocket_frame(connection, WEBSOCKET_OP_TEXT, msg.c_str(), strlen(msg.c_str()));
}


void HTTPServer::ev_handler(struct mg_connection *nc, int ev, void *ev_data)
{
    struct http_message *hm = (struct http_message *)ev_data;

    switch (ev)
    {
    case MG_EV_HTTP_REQUEST:
        if (mg_vcmp(&hm->method, "POST") == 0)
        {
            handlePost(nc, hm);
        }
        else if (mg_vcmp(&hm->method, "GET") == 0)
        {
            if(!handleDownload(nc, hm))
            {
                handleGet(nc, hm);
            }
        }
        else
        {
            mg_serve_http(nc, hm, s_http_server_opts); /* Serve static content */
        }

        break;
    default:
        break;
    }
}

std::vector<std::string> HTTPServer::split(const std::string &str,const std::string &pattern)
{
    std::vector<std::string> resVec;

    if ("" == str)
    {
        return resVec;
    }
    //方便截取最后一段数据
    std::string strs = str + pattern;

    size_t pos = strs.find(pattern);
    size_t size = strs.size();

    while (pos != std::string::npos)
    {
        std::string x = strs.substr(0,pos);
        resVec.push_back(x);
        strs = strs.substr(pos+1,size);
        pos = strs.find(pattern);
    }

    return resVec;
}


bool HTTPServer::start(std::string port, std::string web_root, std::string save_dir)
{
    if (m_mutex.try_lock())
    {
        m_saveDir = save_dir;
        m_thread = std::thread(threadFn, port, web_root);
        return true;
    }
    else
    {
        return false;
    }
}

bool HTTPServer::stop()
{
    if (!m_mutex.try_lock())
    {
        m_mutex.unlock();
        return true;
    }
    else
    {
        m_mutex.unlock();
        return false;
    }
}

void HTTPServer::join()
{
    m_thread.join();
}

bool HTTPServer::addPostEx(const std::string &uri, PostHandlerEx callback, Checker checker)
{
    if (m_mutex.try_lock())
    {
        if(m_postHandlerEx.find(uri) != m_postHandlerEx.end()
                || m_postHandler.find(uri) != m_postHandler.end())
        {
            m_mutex.unlock();
            return false;
        }
        m_postHandlerEx[uri] = std::make_pair(checker,callback);
        m_mutex.unlock();
        return true;
    }
    return false;
}

bool HTTPServer::addPost(const std::string &uri, HTTPServer::PostHandler callback, Checker checker)
{
    if (m_mutex.try_lock())
    {        
        if(m_postHandlerEx.find(uri) != m_postHandlerEx.end()
                || m_postHandler.find(uri) != m_postHandler.end())
        {
            m_mutex.unlock();
            return false;
        }
        m_postHandler[uri] = std::make_pair(checker,callback);
        m_mutex.unlock();
        return true;
    }
    return false;
}

bool HTTPServer::addGetEx(const std::string &uri, HTTPServer::GetHandlerEx callback, Checker checker)
{
    if (m_mutex.try_lock())
    {
        if(m_getHandlerEx.find(uri) != m_getHandlerEx.end()
                || m_getHandler.find(uri) != m_getHandler.end())
        {
            m_mutex.unlock();
            return false;
        }
        m_getHandlerEx[uri] = std::make_pair(checker,callback);
        m_mutex.unlock();
        return true;
    }
    return false;
}

bool HTTPServer::addGet(const std::string &uri, HTTPServer::GetHandler callback, Checker checker)
{
    if (m_mutex.try_lock())
    {
        if(m_getHandlerEx.find(uri) != m_getHandlerEx.end()
                || m_getHandler.find(uri) != m_getHandler.end())
        {
            m_mutex.unlock();
            return false;
        }
        m_getHandler[uri] = std::make_pair(checker,callback);
        m_mutex.unlock();
        return true;
    }
    return false;
}

bool HTTPServer::addDownload(const std::string &uri, HTTPServer::DownloadHandler callback, HTTPServer::Checker checker)
{
    if (m_mutex.try_lock())
    {
        if(m_downloadHandler.find(uri) != m_downloadHandler.end())
        {
            m_mutex.unlock();
            return false;
        }
        m_downloadHandler[uri] = std::make_pair(checker,callback);
        m_mutex.unlock();
        return true;
    }
    return false;
}

