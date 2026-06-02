#include "mongoose.h"
#include <iostream>

static const char* s_http_port = "8000";

static void fn(struct mg_connection* c, int ev, void* ev_data) {
    if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message* hm = (struct mg_http_message*)ev_data;

        //路由分发
        if (mg_match(hm->uri, mg_str("/health"), NULL)) {
            mg_http_reply(c, 200, "Content-Type: application/json\r\n", "{\"status\":\"ok\"}");
            return;
        }

        //默认
        mg_http_reply(c, 200, "Contect-Type: text/plain\r\n", "Navigation API Server\n"
                                                               "Endpoints:\n"
                                                               "  GET /health\n"
                                                               " POST /api/route (coming soon)\n");
    }
}

int main() {
    struct mg_mgr mgr;
    mg_mgr_init(&mgr);
    mg_http_listen(&mgr, "http://0.0.0.0:8000", fn, NULL);

    std::cout << "Server listening on http://localhost:" << s_http_port << "\n";

    for (;;) {
        mg_mgr_poll(&mgr, 1000);
    }

    mg_mgr_free(&mgr);
    return 0;
}
