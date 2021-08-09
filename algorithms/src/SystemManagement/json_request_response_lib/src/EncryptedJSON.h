#ifndef GAAS_ENCRYPTED_JSON_HEADER
#define GAAS_ENCRYPTED_JSON_HEADER

#include <iostream>
#include <sstream>
#include <mutex>
#include <utility>
#include <future>
#include <thread>
#include <chrono>
#include <glog/logging.h>
#include "third_party/nlohmann_json/single_include/nlohmann/json.hpp"
#include "third_party/cpp-httplib/httplib.h"

using json = nlohmann::json;
using std::string;
using std::endl;

namespace EncryptedJSON
{

const static int REQUEST_NOT_FINISHED = 0;
const static int REQUEST_FINISHED_SUCCESSFULLY = 1;
const static int REQUEST_FINISHED_WITH_ERROR = -1;

struct EncryptedJSONRequest;
struct EncryptedJSONResponse;
struct EncryptedJSONResponse
{
    EncryptedJSONResponse()
    {
        pResponse_json = std::shared_ptr<json>(new json);
    }
    using Ptr = std::shared_ptr<EncryptedJSONResponse>;
    bool timeout = false;//是否超时.
    string reason_of_failure = "Unknown";
    std::shared_ptr<json> pResponse_json;
};

static std::string request_function(const json& j_,const string& api_url)//,std::promise<EncryptedJSONResponse::Ptr>& promise)
{
    try{
        httplib::Client cli("localhost", 8080);
        std::string json_str = j_.dump(4);
        httplib::Headers header;
        auto res = cli.Post(api_url.c_str(),header,json_str,"text/plain; charset=utf-8");
        if(res)
        {
            LOG(INFO) << "Received HTTP response with status:"<< res->status << endl;
            LOG(INFO) << res->get_header_value("Content-Type") << endl;
            LOG(INFO) << res->body << endl;
        }
        else
        {
            LOG(INFO) << "error code: " << res.error() << endl;
            return std::string("{\"state\":\"http_error\"}");
        }
        return std::string(res->body);
    }
    catch(std::exception e)
    {
        LOG(ERROR)<<"[EncryptedJSONRequest] Caught error in http request:"<<e.what()<<endl;
        return std::string("{\"state\":\"http_error\"}");
    }
    catch (const char* msg)
    {
        LOG(ERROR)<<"[EncryptedJSONRequest] Caught error in http request!"<<msg<<endl;
        return std::string("{\"state\":\"http_error\"}");
    }
    catch(...)
    {
        LOG(ERROR)<<"[EncryptedJSONRequest] Caught unknown error in http request!"<<endl;
        return std::string("{\"state\":\"http_error\"}");
    }
}

class EncryptedJSONRequest
{
public:
    typedef std::chrono::time_point<std::chrono::high_resolution_clock> ChronoTimeT;
    void start_request(const json& request_json,const string& api_url)
    {
        //        this->request_future = this->request_promise.get_future();
        //        //this->pThread = std::shared_ptr<std::thread>(new std::thread(request_function,std::cref(request_json),std::ref(this->request_promise)));
        //        this->pAsync = std::shared_ptr<std::async>( new std::async(std::launch::async,request_function,std::cref(request_json))//,std::ref(this->request_promise))
        //                );
        this->api_url = api_url;
        this->t_start = std::chrono::system_clock::now();
        this->request_future = std::async(std::launch::async,request_function,std::cref(request_json),std::cref(api_url));
    }

    bool getResponseSafe(EncryptedJSONResponse::Ptr& ptr_output)
    {
        //先查询是否完成,后获取.
        if(queryIfFinished())
        {
            ptr_output = getResponse();
            return true;
        }
        else
        {
            return false;
        }
    }
    bool everTimeout()//判断是否已经超时.
    {
        ChronoTimeT t_end = std::chrono::system_clock::now();
        auto duration_t = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
        double cost_sec = (double(duration_t.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den);
        static const double TIMEOUT_THRES_SECOND = 8;
        if(cost_sec>8)
        {
            LOG(WARNING)<<"[WARNING] [EncryptedJSONRequest] Request time out!"<<endl;
            return true;
        }
        return false;
    }
    std::string getCurrentApiUrl()
    {
        return this->api_url;
    }


private:
    std::string api_url;
    std::future<std::string> request_future;
    ChronoTimeT t_start;
    //std::shared_ptr<std::async> pAsync;
    bool queryIfFinished()
    {
        std::future_status status;
        status = this->request_future.wait_for(std::chrono::seconds(0));
        if(status == std::future_status::deferred)
        {
            return false;//未开始.
        }
        else if(status == std::future_status::timeout)
        {
            return false;//尚未结束.
        }
        else if(status == std::future_status::ready)
        {
            return true;
        }

    }
    EncryptedJSONResponse::Ptr getResponse()
    {
        if(!queryIfFinished())
        {
            LOG(ERROR)<<"[ERROR] In EncryptedJSONResponse::getResponse: response not ready!"<<endl;
            throw "Error!";
        }
        std::string ret_str;
        if(everTimeout())
        {
            EncryptedJSONResponse::Ptr ret_ptr(new EncryptedJSONResponse);
            ret_ptr->timeout = true;
            ret_ptr->reason_of_failure = "timeout";
            return ret_ptr;
        }
        ret_str = this->request_future.get();

        EncryptedJSONResponse::Ptr retval (new EncryptedJSONResponse);
        LOG(INFO)<<"ret_str value:"<<ret_str<<endl;
        std::stringstream ss(ret_str);
        ss>>*(retval->pResponse_json);
        if(retval->pResponse_json->count("state"))
        {
            if(retval->pResponse_json->at("state")==std::string("http_error"))
            {
                retval->reason_of_failure = "http_error";
            }
        }
        return retval;
    }
};



class EncryptedJSONClient
{
private:
    string vehicle_id; // 飞行器唯一ID.
    string vehicle_pubkey; //公钥.用于加密通信.
    string vehicle_private_key;//私钥
    string https_server_url;
    std::map<string,EncryptedJSONRequest> curr_requests;
public:
    bool initEncryptedJSONClient(string https_server_url_setting);
    bool startAsyncRequest();
    bool queryAsyncRequest();
    bool getAsyncResponse();
};


}
#endif
