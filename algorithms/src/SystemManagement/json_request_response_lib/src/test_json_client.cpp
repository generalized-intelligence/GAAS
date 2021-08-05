#include "third_party/cpp-httplib/httplib.h"


using std::cout;
using std::endl;


int do_request()
{
    httplib::Client cli("localhost", 8080);
    if (auto res = cli.Get("/hi"))
    {
        cout << res->status << endl;
        cout << res->get_header_value("Content-Type") << endl;
        cout << res->body << endl;
    } else {
        cout << "error code: " << res.error() << endl;
    }
    return 0;
}

int main(int argc,char** argv)
{
    do_request();
    return 0;
}
