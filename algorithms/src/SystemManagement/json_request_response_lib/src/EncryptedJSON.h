#ifndef GAAS_ENCRYPTED_JSON_HEADER
#define GAAS_ENCRYPTED_JSON_HEADER

#include <iostream>
#include <mutex>



using std::string;
using std::endl;



struct EncryptedJSONRequest
{

};
struct EncryptedJSONResponse
{

};
struct RequestAndResponse
{
    EncryptedJSONRequest req;
    EncryptedJSONResponse resp;
    bool finished = false;
};
class EncryptedJSONClient
{
private:
    string vehicle_id; // 飞行器唯一ID.
    string vehicle_pubkey; //公钥.用于加密通信.
    string vehicle_private_key;//私钥
    string https_server_url;
    std::map<string,RequestAndResponse> curr_requests;
public:
    bool initEncryptedJSONClient(string https_server_url_setting);

    bool startAsyncRequest();
    bool queryAsyncRequest();
    bool getAsyncResponse();

};



#endif
