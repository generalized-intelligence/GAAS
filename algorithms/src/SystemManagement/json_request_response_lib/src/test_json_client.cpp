#include "third_party/cpp-httplib/httplib.h"
#include "EncryptedJSON.h"
#include <sstream>

using std::cout;
using std::endl;


//int do_request()
//{
//    httplib::Client cli("localhost", 8080);
//    if (auto res = cli.Get("/hi"))
//    {
//        cout << res->status << endl;
//        cout << res->get_header_value("Content-Type") << endl;
//        cout << res->body << endl;
//    } else {
//        cout << "error code: " << res.error() << endl;
//    }
//    return 0;
//}

int main(int argc,char** argv)
{
    for(int i = 0;i<100;i++)
    {
        EncryptedJSON::EncryptedJSONRequest json_req;
        json j;
        std::stringstream ss("{\"function\": \"query\"}");
        ss>>j;
        json_req.start_request(j,"/api/ping");
        EncryptedJSON::EncryptedJSONResponse::Ptr pRes;
        while(!json_req.getResponseSafe(pRes)&&!json_req.everTimeout())
        {
            cout<<"still waiting..."<<endl;
            usleep(20000);
        }
        if(json_req.everTimeout())
        {
            cout<<"Error:timeout!"<<endl;
        }
        cout<<"Result:"<<pRes->pResponse_json->dump(4)<<endl;
        if(pRes->reason_of_failure==std::string("http_error"))
        {
            cout<<"Request has failed!"<<endl;
        }
    }
    return 0;
}
