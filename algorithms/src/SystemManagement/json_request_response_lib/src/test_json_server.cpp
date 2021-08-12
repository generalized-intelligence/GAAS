#include "third_party/cpp-httplib/httplib.h"
using std::cout;
using std::endl;

int start_server()
{
    httplib::Server svr;

    svr.set_logger([](const auto& req, const auto& res) {
        cout<<"request:"<<req.body<<";"<<endl<<endl<<"Response:"<<res.body<<endl<<endl;
    });

    svr.Get("/hi", [](const httplib::Request& req, httplib::Response& res) {
        res.set_content("Hello World!", "text/plain");
    });
    svr.Post("/api/ping",[](const httplib::Request& req, httplib::Response& res){
        res.set_content("{\"state\":\"correct\",\"data\":\"pong!\" }", "text/plain");
    });
    cout<<"Server start..."<<endl;
    svr.listen("localhost", 8080);
    return 0;
}

int main(int argc,char** argv)
{
    start_server();
    return 0;
}
